"""Guess how bits are encoded from run durations alone.

The idea: every clocked baseband code produces pulse and gap lengths that are small
integer multiples of one time unit, and the pattern of which multiples appear on
which level gives the encoding away:

  Manchester / biphase     both levels show 1 and 2 units only
  pulse-width (PWM)        active pulses come in two lengths, gaps in one
  pulse-distance           active pulses in one length, gaps in two
  NRZ / UART               runs of 1..N units on either level
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .dsp import Runs, robust_sigma


@dataclass
class Cluster:
    center_us: float
    spread_us: float
    count: int
    multiple: Optional[int] = None
    minor: bool = False  # too few members to trust on its own

    def __str__(self) -> str:
        m = f" = {self.multiple}u" if self.multiple else ""
        return f"{self.center_us:9.1f} us ±{self.spread_us:6.1f}  x{self.count:<4d}{m}"


@dataclass
class Hypothesis:
    encoding: str
    confidence: float
    reason: str


@dataclass
class TimingModel:
    unit_us: Optional[float]
    fit_error: float  # rms deviation from integer multiples, as a fraction of a unit
    active_level: bool
    active: List[Cluster]
    idle: List[Cluster]
    separators_us: List[float]  # idle stretches long enough to be frame gaps
    sync_us: List[float]  # active stretches far longer than any data pulse
    hypotheses: List[Hypothesis] = field(default_factory=list)
    # Share of (pulse, following gap) pairs whose combined length equals the most common
    # total. Near 1.0 means a fixed bit period, which separates constant-period PWM
    # from Manchester even though both show only 1- and 2-unit runs.
    period_consistency: float = 0.0
    off_grid: int = 0  # runs too far from any whole number of units

    @property
    def bit_rate_hint(self) -> Optional[float]:
        return 1e6 / self.unit_us if self.unit_us else None

    def report(self) -> str:
        lines = []
        lvl = "high" if self.active_level else "low"
        lines.append(f"active level: {lvl}   unit: "
                     + (f"{self.unit_us:.1f} us ({1e6 / self.unit_us:.0f}/s), fit error {self.fit_error:.3f}"
                        + (f", {self.off_grid} off-grid runs" if self.off_grid else "")
                        if self.unit_us else "none found"))
        lines.append("  active pulses:")
        lines += [f"    {c}" for c in self.active] or ["    (none)"]
        lines.append("  idle gaps:")
        lines += [f"    {c}" for c in self.idle] or ["    (none)"]
        if self.sync_us:
            lines.append(f"  long active pulses (sync/preamble?): {len(self.sync_us)}, "
                         f"median {np.median(self.sync_us):.0f} us")
        if self.separators_us:
            lines.append(f"  frame gaps: {len(self.separators_us)}, median {np.median(self.separators_us):.0f} us")
        lines.append("  hypotheses:")
        for h in self.hypotheses:
            lines.append(f"    {h.confidence:4.2f}  {h.encoding:<22s} {h.reason}")
        return "\n".join(lines)


def cluster_durations(d, ratio: float = 1.25, min_frac: float = 0.03) -> List[Cluster]:
    """Split sorted durations wherever consecutive values jump by more than `ratio`.
    Small groups are kept but marked minor: they may be glitches, or legitimately rare
    lengths such as UART's longest runs, which only the unit grid can tell apart."""
    d = np.sort(np.asarray(d, dtype=np.float64))
    d = d[d > 0]
    if d.size == 0:
        return []
    groups = np.split(d, np.flatnonzero(d[1:] / d[:-1] > ratio) + 1)
    min_count = max(2, int(np.ceil(min_frac * d.size)))
    return [Cluster(float(np.median(g)), robust_sigma(g), int(g.size), minor=g.size < min_count)
            for g in groups]


def estimate_unit(clusters: List[Cluster], max_multiple: int = 12,
                  tol: float = 0.12) -> Tuple[Optional[float], float]:
    """Largest unit of which every cluster centre is a near-integer multiple."""
    if not clusters:
        return None, float("inf")
    c = np.array([k.center_us for k in clusters])
    w = np.array([k.count for k in clusters], dtype=np.float64)
    candidates = sorted({ci / m for ci in c for m in range(1, max_multiple + 1)}, reverse=True)
    for unit in candidates:
        mult = np.round(c / unit)
        if mult.min() < 1 or mult.max() > max_multiple:
            continue
        frac = c / unit - mult
        if np.sqrt(np.sum(w * frac ** 2) / w.sum()) < tol and np.abs(frac).max() < 2 * tol:
            unit = float(np.sum(w * c * mult) / np.sum(w * mult ** 2))  # least-squares refine
            mult = np.round(c / unit)
            err = float(np.sqrt(np.sum(w * (c / unit - mult) ** 2) / w.sum()))
            for k, m in zip(clusters, mult):
                k.multiple = int(m)
            return unit, err
    return None, float("inf")


def _regrid(d, unit: float, tol: float = 0.35):
    """Group durations by nearest whole number of units. Returns (clusters, fractional
    deviations of the on-grid durations, count of off-grid durations)."""
    d = np.asarray(d, dtype=np.float64)
    if d.size == 0:
        return [], np.zeros(0), 0
    x = d / unit
    m = np.round(x).astype(np.int64)
    ok = (m >= 1) & (np.abs(x - m) < tol)
    clusters = []
    for k in np.unique(m[ok]):
        g = d[ok & (m == k)]
        clusters.append(Cluster(float(np.median(g)), robust_sigma(g), int(g.size), multiple=int(k)))
    return clusters, (x - m)[ok], int(np.sum(~ok))


def infer_timing(runs: Runs, separator_factor: float = 6.0, max_run_units: float = 12.0) -> TimingModel:
    active_level = not runs.idle_level
    if len(runs) < 4:
        return TimingModel(None, float("inf"), active_level, [], [], [], [],
                           [Hypothesis("too few edges", 0.0, f"only {len(runs)} runs in this window")])

    lv = runs.levels[1:-1]  # boundary runs are truncated by the window
    du = runs.durations_us()[1:-1]
    is_active = lv == active_level
    provisional_long = du > separator_factor * float(np.median(du))
    first_active = cluster_durations(du[is_active & ~provisional_long])
    first_idle = cluster_durations(du[~is_active & ~provisional_long])
    majors = [c for c in first_active + first_idle if not c.minor]

    # Ratio gaps separate 1, 2 and 3 units cleanly but not 6 from 7 (UART runs of equal
    # bits): take the unit from the short clusters only, then put every duration on
    # that grid.
    unit, err = None, float("inf")
    if majors:
        shortest = min(c.center_us for c in majors)
        unit, err = estimate_unit([c for c in majors if c.center_us <= 4.5 * shortest])
        if unit is None:
            unit, err = estimate_unit(majors)

    off_grid = 0
    if unit:
        long_ = du > max_run_units * unit
        active, frac_a, off_a = _regrid(du[is_active & ~long_], unit)
        idle, frac_i, off_i = _regrid(du[~is_active & ~long_], unit)
        frac = np.concatenate([frac_a, frac_i])
        err = float(np.sqrt(np.mean(frac ** 2))) if frac.size else err
        off_grid = off_a + off_i
    else:
        long_ = provisional_long
        active = [c for c in first_active if not c.minor]
        idle = [c for c in first_idle if not c.minor]

    separators = du[long_ & ~is_active].tolist()
    sync = du[long_ & is_active].tolist()
    model = TimingModel(unit, err, active_level, active, idle, separators, sync, off_grid=off_grid)
    if unit:
        sums = [int(round(du[i] / unit) + round(du[i + 1] / unit)) for i in range(len(du) - 1)
                if is_active[i] and not is_active[i + 1] and not long_[i] and not long_[i + 1]]
        if sums:
            model.period_consistency = float(np.bincount(sums).max() / len(sums))
    model.hypotheses = _hypotheses(model)
    return model


def _hypotheses(m: TimingModel) -> List[Hypothesis]:
    hs: List[Hypothesis] = []
    if m.unit_us is None:
        hs.append(Hypothesis("unclocked", 0.3,
                             "durations are not multiples of one unit: tones/FSK, analogue, "
                             "or slicing errors - check the tone report and the plot"))
        return hs

    act = {c.multiple for c in m.active}
    gap = {c.multiple for c in m.idle}
    rate = 1e6 / m.unit_us
    fit = max(0.0, 1.0 - m.fit_error / 0.12)

    fixed_period = m.period_consistency >= 0.9
    pwm_shape = len(act) == 2 and len(gap) == 2 and len({a + g for a in act for g in gap}) < 4
    if act | gap <= {1, 2} and 1 in act and 1 in gap and 2 in act | gap:
        hs.append(Hypothesis("manchester/biphase", (0.5 if pwm_shape and fixed_period else 0.85) * fit,
                             f"only 1u and 2u on both levels; bit period {2 * m.unit_us:.0f} us "
                             f"({rate / 2:.0f} bit/s)"))
    if pwm_shape:
        hs.append(Hypothesis("pwm-constant-period", (0.9 if fixed_period else 0.55) * fit,
                             f"pulse+gap = {m.period_consistency:.0%} constant; "
                             f"pulses {sorted(act)} / gaps {sorted(gap)} units"))
    if len(act) == 2 and len(gap) == 1:
        hs.append(Hypothesis("pulse-width", 0.7 * fit,
                             f"active pulses of {sorted(act)} units carry the bit, gaps fixed at {sorted(gap)}"))
    if len(act) == 1 and len(gap) == 2:
        hs.append(Hypothesis("pulse-distance", 0.7 * fit,
                             f"gaps of {sorted(gap)} units carry the bit, pulses fixed at {sorted(act)}"))
    top = max(act | gap)
    if top >= 3:
        uart = " - runs up to 9u fit UART 8N1" if top <= 10 else ""
        hs.append(Hypothesis("nrz/uart", 0.6 * fit,
                             f"runs of {sorted(act | gap)} units, {rate:.0f} baud{uart}"))
    if not hs:
        hs.append(Hypothesis("unclassified", 0.2 * fit,
                             f"unit {m.unit_us:.0f} us; pulses {sorted(act)} gaps {sorted(gap)} units"))
    return sorted(hs, key=lambda h: -h.confidence)
