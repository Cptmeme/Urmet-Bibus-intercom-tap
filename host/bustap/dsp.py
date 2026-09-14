"""Signal conditioning for unknown bus traffic.

Pipeline: noise estimate -> event detection -> two-level slicing with hysteresis ->
run extraction with sub-sample edge timing. Nothing here assumes a particular
encoding; that is infer.py's job.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

_TINY = 1e-12


def robust_sigma(x) -> float:
    """Standard deviation estimate that ignores outliers (scaled MAD)."""
    x = np.asarray(x, dtype=np.float64)
    if x.size == 0:
        return 0.0
    return 1.4826 * float(np.median(np.abs(x - np.median(x))))


def noise_sigma(x) -> float:
    """White-noise estimate from first differences. Sparse edges barely move it,
    so it stays honest even when a capture is mostly activity."""
    x = np.asarray(x, dtype=np.float64)
    if x.size < 3:
        return 0.0
    return robust_sigma(np.diff(x)) / np.sqrt(2.0)


def active_body(x, frac: float = 0.25) -> np.ndarray:
    """Trim idle padding: the span between the first and last samples departing from
    the median by more than `frac` of the largest departure."""
    x = np.asarray(x, dtype=np.float64)
    if x.size == 0:
        return x
    dev = np.abs(x - np.median(x))
    if dev.max() <= 0:
        return x
    idx = np.flatnonzero(dev > frac * dev.max())
    return x[idx[0]: idx[-1] + 1]


def transition_fraction(x) -> float:
    """Share of samples sitting between the two logic levels. Flat-topped baseband
    pulses measure ~0.00-0.08; a sinusoid ~0.2-0.3, because it spends a large part of
    each cycle in the middle of its range."""
    x = np.asarray(x, dtype=np.float64)
    if x.size < 8:
        return 0.0
    lv = estimate_levels(x)
    span = lv.high - lv.low
    if span <= 0:
        return 0.0
    return float(np.mean((x > lv.low + 0.25 * span) & (x < lv.high - 0.25 * span)))


def moving_average(x, n: int) -> np.ndarray:
    """Centred moving average, same length as x, edges held."""
    x = np.asarray(x, dtype=np.float64)
    n = int(max(1, min(n, len(x))))
    if n == 1 or len(x) == 0:
        return x.copy()
    c = np.cumsum(np.concatenate([[0.0], x]))
    ma = (c[n:] - c[:-n]) / n
    left = (len(x) - len(ma)) // 2
    right = len(x) - len(ma) - left
    return np.concatenate([np.full(left, ma[0]), ma, np.full(right, ma[-1])])


def rolling_baseline(x, fs: float, window_s: float = 2.0, step_s: float = 0.02) -> np.ndarray:
    """Slow baseline: block medians, then a rolling median over those blocks.
    Cheap on long captures and unaffected by short bursts."""
    x = np.asarray(x, dtype=np.float64)
    step = max(1, int(fs * step_s))
    nblk = len(x) // step
    if nblk < 3:
        return np.full(len(x), np.median(x) if len(x) else 0.0)
    blocks = np.median(x[: nblk * step].reshape(nblk, step), axis=1)
    w = max(1, int(round(window_s / step_s))) | 1
    w = min(w, (nblk // 2) * 2 + 1)
    padded = np.pad(blocks, w // 2, mode="edge")
    smooth = np.median(sliding_window_view(padded, w), axis=1)
    base = np.repeat(smooth, step)
    if len(base) < len(x):
        base = np.concatenate([base, np.full(len(x) - len(base), smooth[-1])])
    return base


@dataclass
class Event:
    start: int  # inclusive sample index
    end: int  # exclusive
    peak_sigma: float  # largest deviation from baseline, in noise sigmas

    def duration_s(self, fs: float) -> float:
        return (self.end - self.start) / fs


def find_events(x, fs: float, *, k: float = 8.0, merge_gap_s: float = 0.05,
                min_len_s: float = 0.0005, pad_s: float = 0.02, baseline_window_s: float = 2.0,
                gaps: Sequence[int] = (), sigma_floor: float = 0.0) -> List[Event]:
    """Regions where the signal departs from its slow baseline, or where edge
    activity rises above the idle noise. Events never straddle a capture gap.
    sigma_floor (typically one ADC code) stops a near-noiseless trace from triggering
    on single-code steps."""
    x = np.asarray(x, dtype=np.float64)
    if len(x) < 16:
        return []
    base = rolling_baseline(x, fs, baseline_window_s)
    dev = x - base
    sig = max(noise_sigma(x), sigma_floor, _TINY)
    act = moving_average(np.abs(np.diff(x, prepend=x[0])), max(3, int(0.002 * fs)))
    act_thr = np.median(act) + k * max(robust_sigma(act), sig * 0.1)
    mask = (np.abs(dev) > k * sig) | (act > act_thr)
    if not mask.any():
        return []

    flips = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(np.int8), [0]])))
    spans = list(zip(flips[0::2].tolist(), flips[1::2].tolist()))
    merge_gap, min_len, pad = int(merge_gap_s * fs), int(min_len_s * fs), int(pad_s * fs)

    merged: List[List[int]] = []
    for a, b in spans:
        if merged and a - merged[-1][1] <= merge_gap:
            merged[-1][1] = b
        else:
            merged.append([a, b])

    cuts = sorted(int(g) for g in gaps if 0 < g < len(x))
    events: List[Event] = []
    for a, b in merged:
        if b - a < max(1, min_len):
            continue
        a, b = max(0, a - pad), min(len(x), b + pad)
        bounds = [a] + [g for g in cuts if a < g < b] + [b]
        for s, e in zip(bounds[:-1], bounds[1:]):
            if e - s >= max(1, min_len):
                events.append(Event(s, e, float(np.max(np.abs(dev[s:e]))) / sig))
    return events


def otsu_threshold(x, bins: int = 256) -> float:
    x = np.asarray(x, dtype=np.float64)
    hist, edges = np.histogram(x, bins=bins)
    centers = (edges[:-1] + edges[1:]) / 2
    w0 = np.cumsum(hist).astype(np.float64)
    w1 = w0[-1] - w0
    m0 = np.cumsum(hist * centers)
    mu0 = m0 / np.maximum(w0, 1)
    mu1 = (m0[-1] - m0) / np.maximum(w1, 1)
    between = w0 * w1 * (mu0 - mu1) ** 2
    return float(edges[int(np.argmax(between[:-1])) + 1])


@dataclass
class Levels:
    low: float
    high: float
    threshold: float
    hysteresis: float
    separation_sigma: float  # (high - low) / noise sigma; below ~10 slicing is unreliable


def estimate_levels(x, sigma: Optional[float] = None, hysteresis_frac: float = 0.2) -> Levels:
    x = np.asarray(x, dtype=np.float64)
    t = otsu_threshold(x)
    lo_part, hi_part = x[x <= t], x[x > t]
    lo = float(np.median(lo_part)) if lo_part.size else float(x.min())
    hi = float(np.median(hi_part)) if hi_part.size else float(x.max())
    sep = hi - lo
    sigma = noise_sigma(x) if sigma is None else sigma
    return Levels(lo, hi, (lo + hi) / 2, hysteresis_frac * sep, sep / sigma if sigma > 0 else float("inf"))


def schmitt(x, lo_thr, hi_thr) -> np.ndarray:
    """Hysteresis comparator. Thresholds may be scalars or arrays (adaptive slicing)."""
    x = np.asarray(x, dtype=np.float64)
    above, below = x > hi_thr, x < lo_thr
    decisive = np.flatnonzero(above | below)
    if decisive.size == 0:
        mid = np.mean(np.broadcast_to((np.asarray(lo_thr) + np.asarray(hi_thr)) / 2, x.shape))
        return np.full(len(x), bool(np.mean(x) > mid))
    ptr = np.zeros(len(x), dtype=np.int64)
    ptr[decisive] = np.arange(1, decisive.size + 1)
    np.maximum.accumulate(ptr, out=ptr)
    vals = above[decisive]
    return np.where(ptr == 0, vals[0], vals[np.maximum(ptr - 1, 0)])


@dataclass
class Runs:
    """Constant-level stretches of a sliced signal. The first and last runs are cut
    off by the analysis window, so their durations are lower bounds only."""
    levels: np.ndarray  # bool per run
    starts: np.ndarray  # float sample position of each run's leading edge
    durations: np.ndarray  # float samples
    fs: float

    def __len__(self) -> int:
        return len(self.levels)

    def durations_us(self) -> np.ndarray:
        return self.durations * 1e6 / self.fs

    @property
    def idle_level(self) -> bool:
        return bool(self.levels[0]) if len(self.levels) else False


def slice_to_runs(x, fs: float, levels: Optional[Levels] = None,
                  adaptive_window: Optional[int] = None) -> Tuple[Runs, Levels]:
    """Digitise x and return runs with edges interpolated to the mid-threshold
    crossing, which sharpens duration histograms well below one sample."""
    x = np.asarray(x, dtype=np.float64)
    levels = levels or estimate_levels(x)
    if adaptive_window:
        mid = moving_average(x, adaptive_window)
    else:
        mid = np.full(len(x), levels.threshold)
    h = levels.hysteresis / 2
    d = schmitt(x, mid - h, mid + h)

    change = np.flatnonzero(d[1:] != d[:-1]) + 1
    search = max(2, int(0.25 * fs / 1000))  # look back up to ~0.25 ms for the crossing
    edges = np.empty(change.size, dtype=np.float64)
    for n, c in enumerate(change):
        k = int(c)
        stop = max(1, c - search)
        while k > stop and (x[k - 1] - mid[k - 1]) * (x[k] - mid[k]) > 0:
            k -= 1
        a, b = x[k - 1] - mid[k - 1], x[k] - mid[k]
        edges[n] = (k - 1) + a / (a - b) if a * b <= 0 and a != b else float(c)
    edges = np.maximum.accumulate(edges) if edges.size else edges

    starts = np.concatenate([[0.0], edges])
    ends = np.concatenate([edges, [float(len(x))]])
    run_levels = d[np.concatenate([[0], change]).astype(np.int64)] if len(x) else np.zeros(0, bool)
    return Runs(run_levels.astype(bool), starts, np.maximum(ends - starts, 0.0), fs), levels
