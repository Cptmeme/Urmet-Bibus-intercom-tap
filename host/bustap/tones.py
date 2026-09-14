"""Audio-band signalling: dominant tones, FSK demodulation and DTMF.

L1/L2 also carries speech, and plenty of intercoms signal with tones rather than
baseband pulses. Slicing a tone as baseband yields a square wave at the carrier
frequency and convincing nonsense, so events are checked for tones first.

Spectral concentration alone cannot tell a tone from a pulse train - a UART or NRZ
frame puts most of its power in a bit-rate fundamental too. What separates them is
shape: pulses have flat tops, sinusoids do not (dsp.transition_fraction).
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .dsp import active_body, moving_average, otsu_threshold, robust_sigma, schmitt, transition_fraction

TONAL_TRANSITION_FRACTION = 0.15  # measured: baseband 0.00-0.08, tones 0.21-0.30


@dataclass
class Peak:
    freq_hz: float
    rel_power: float  # share of in-band power within +/-2 bins of the peak


def spectrum_peaks(x, fs: float, n_peaks: int = 5, min_hz: float = 100.0) -> List[Peak]:
    x = np.asarray(x, dtype=np.float64)
    if len(x) < 64:
        return []
    p = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x)))) ** 2
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    band = f >= min_hz
    total = p[band].sum()
    if total <= 0:
        return []
    idx = np.flatnonzero(band)
    idx = idx[(idx > 0) & (idx < len(p) - 1)]
    maxima = idx[(p[idx] > p[idx - 1]) & (p[idx] >= p[idx + 1])]
    peaks: List[Peak] = []
    for i in maxima[np.argsort(p[maxima])[::-1]]:
        if any(abs(f[i] - k.freq_hz) < 3 * (fs / len(x)) for k in peaks):
            continue
        peaks.append(Peak(float(f[i]), float(p[max(0, i - 2): i + 3].sum() / total)))
        if len(peaks) == n_peaks:
            break
    return peaks


def looks_tonal(x, fs: float, min_hz: float = 100.0) -> bool:
    body = active_body(x)
    if len(body) < 64 or not spectrum_peaks(body, fs, n_peaks=1, min_hz=min_hz):
        return False
    return transition_fraction(body) > TONAL_TRANSITION_FRACTION


def bandpass(x, fs: float, lo_hz: float, hi_hz: float) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    spec = np.fft.rfft(x - x.mean())
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    spec[(f < lo_hz) | (f > hi_hz)] = 0
    return np.fft.irfft(spec, n=len(x))


def instantaneous_frequency(x, fs: float, band: Tuple[float, float],
                            gate: float = 0.35) -> Tuple[np.ndarray, np.ndarray]:
    """Frequency from successive zero crossings in both directions (half-cycle
    resolution). Intervals are discarded where the envelope is weak: band-limiting
    rings at the band edge through silence, which would otherwise masquerade as a tone."""
    y = bandpass(x, fs, *band)
    env = np.sqrt(moving_average(y ** 2, max(4, int(2 * fs / band[0]))))
    h = 0.1 * max(robust_sigma(y), 1e-12)
    state = schmitt(y, -h, h)
    change = np.flatnonzero(state[1:] != state[:-1]) + 1
    times = np.empty(change.size)
    for n, c in enumerate(change):
        k = int(c)
        if state[c]:
            while k > 1 and y[k - 1] > 0:
                k -= 1
        else:
            while k > 1 and y[k - 1] < 0:
                k -= 1
        a, b = y[k - 1], y[k]
        times[n] = (k - 1) + (-a / (b - a) if b != a else 0.0)
    if times.size < 3:
        return np.zeros(0), np.zeros(0)
    strong = env[np.clip(np.round(times).astype(int), 0, len(y) - 1)] > gate * env.max()
    keep = strong[:-1] & strong[1:]
    dt = np.diff(times) / fs
    mid = (times[1:] + times[:-1]) / 2 / fs
    return mid[keep], (1.0 / (2.0 * dt))[keep]


def estimate_fsk_tones(x, fs: float, band: Optional[Tuple[float, float]] = None
                       ) -> Optional[Tuple[float, float]]:
    """The two carrier frequencies, if the event holds two distinct ones."""
    if band is None:
        peaks = spectrum_peaks(active_body(x), fs, n_peaks=2)
        if not peaks:
            return None
        centre = float(np.mean([p.freq_hz for p in peaks]))
        band = (0.4 * centre, 1.8 * centre)
    _, f = instantaneous_frequency(x, fs, band)
    if len(f) < 16:
        return None
    f = np.median(np.stack([np.roll(f, -1), f, np.roll(f, 1)]), axis=0)
    thr = otsu_threshold(f)
    lo, hi = f[f <= thr], f[f > thr]
    if len(lo) < 0.1 * len(f) or len(hi) < 0.1 * len(f):
        return None
    f0, f1 = float(np.median(lo)), float(np.median(hi))
    if f1 - f0 < 0.1 * (f0 + f1) / 2:
        return None
    return f0, f1


def fsk_demod(x, fs: float, band: Optional[Tuple[float, float]] = None, gate: float = 0.2
              ) -> Optional[Tuple[np.ndarray, float, float]]:
    """Non-coherent FSK discriminator: energy near each tone in a sliding window one
    tone-spacing long, the shortest window that still separates the two tones.

    Returns a soft signal in [-1, 1] (+1 = higher tone) on the original sample grid, so
    the ordinary slicer can interpolate edges, plus both tone frequencies. Zero-crossing
    timing fails when a bit holds only one or two carrier cycles; this does not.
    Silence reads as the higher tone, the usual FSK idle (mark) convention."""
    tones = estimate_fsk_tones(x, fs, band)
    if tones is None:
        return None
    f0, f1 = tones
    xc = np.asarray(x, dtype=np.float64)
    xc = xc - xc.mean()
    t = np.arange(len(xc)) / fs
    n = max(4, int(round(fs / (f1 - f0))))

    def energy(f: float) -> np.ndarray:
        z = xc * np.exp(-2j * np.pi * f * t)
        return moving_average(z.real, n) ** 2 + moving_average(z.imag, n) ** 2

    e0, e1 = energy(f0), energy(f1)
    total = e0 + e1
    soft = (e1 - e0) / np.maximum(total, 1e-30)
    soft[total < gate * total.max()] = 1.0
    return soft, f0, f1


DTMF_LOW = (697.0, 770.0, 852.0, 941.0)
DTMF_HIGH = (1209.0, 1336.0, 1477.0, 1633.0)
DTMF_KEYS = ("123A", "456B", "789C", "*0#D")


def _tone_share(block: np.ndarray, fs: float, freqs) -> np.ndarray:
    n = len(block)
    t = np.arange(n) / fs
    energy = float(np.sum(block ** 2)) or 1e-30
    return 2 * np.abs(np.exp(-2j * np.pi * np.outer(freqs, t)) @ block) ** 2 / (n * energy)


def detect_dtmf(x, fs: float, block_s: float = 0.025, hop_s: float = 0.0125,
                min_s: float = 0.04) -> List[Tuple[float, str]]:
    """Overlapping 25 ms blocks: short enough that a 40-60 ms key spans several of
    them, long enough (40 Hz bins) to separate the 73 Hz-spaced DTMF rows."""
    x = np.asarray(x, dtype=np.float64)
    n, hop = int(block_s * fs), max(1, int(hop_s * fs))
    min_blocks = max(1, int(np.ceil((min_s - block_s) / hop_s)) + 1)
    keys: List[Tuple[float, str]] = []
    current, count, start = None, 0, 0.0
    for i in range(0, len(x) - n + 1, hop):
        blk = x[i: i + n] - x[i: i + n].mean()
        lo, hi = _tone_share(blk, fs, DTMF_LOW), _tone_share(blk, fs, DTMF_HIGH)
        li, hj = int(np.argmax(lo)), int(np.argmax(hi))
        clean = (lo[li] + hi[hj] > 0.6 and lo[li] > 4 * np.partition(lo, -2)[-2]
                 and hi[hj] > 4 * np.partition(hi, -2)[-2]
                 and 0.16 < lo[li] / max(hi[hj], 1e-12) < 6.3)  # twist within ~8 dB
        key = DTMF_KEYS[li][hj] if clean else None
        if key == current:
            count += 1
            continue
        if current and count >= min_blocks:
            keys.append((start, current))
        current, count, start = key, 1, i / fs
    if current and count >= min_blocks:
        keys.append((start, current))
    return keys
