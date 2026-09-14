"""Candidate bit decoders.

Every decoder takes the same runs and returns bits plus an error count, so they can
be ranked against each other on one capture. A decoder that validates structure
(Manchester pairs, UART stop bits) is far more informative than one that cannot fail;
plain NRZ always "succeeds", hence its ranking prior.
"""

from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

from .dsp import Runs
from .infer import TimingModel

OFF_GRID = 0.35  # a run further than this from an integer number of units is an error
CLEAN = 0.01  # error rates at or below this count as a clean decode

# Tie-breaks between clean decodes: plain NRZ validates nothing, and the UART variants
# with extra bits can absorb a stop bit and still frame, so prefer the simpler reading.
PRIORS = {"nrz": 0.05, "uart-9N1": 0.02, "uart-8E1": 0.01, "uart-8O1": 0.01}

# Codes that are provably interchangeable on the wire, so no timing analysis can
# choose between them.
EQUIVALENT = {
    frozenset(("manchester", "bmc")):
        "biphase-mark is Manchester shifted by half a bit (bmc[k] = XNOR of adjacent Manchester "
        "bits); keep whichever reading gives the frames a stable preamble",
    frozenset(("uart", "nrz")):
        "a raw bit stream frames as a few UART bytes by chance about as often as not; "
        "record more traffic before trusting the UART reading",
}
UART_MIN_BYTES = 4  # below this, clean UART framing is weak evidence over plain NRZ

# Which timing hypotheses (infer.py) each decoder family corresponds to.
FAMILIES = {
    "manchester/biphase": ("manchester", "bmc"),
    "pwm-constant-period": ("pulse-width",),
    "pulse-width": ("pulse-width",),
    "pulse-distance": ("pulse-distance",),
    "nrz/uart": ("uart", "nrz"),
}


@dataclass
class Frame:
    bits: str
    errors: int
    symbols: int
    start_s: float

    def to_bytes(self, msb_first: bool = True, offset: int = 0) -> bytes:
        b = self.bits[offset:]
        out = bytearray()
        for i in range(len(b) // 8):
            chunk = b[8 * i: 8 * i + 8]
            out.append(int(chunk if msb_first else chunk[::-1], 2))
        return bytes(out)


@dataclass
class DecodeResult:
    decoder: str
    frames: List[Frame] = field(default_factory=list)
    note: str = ""
    ambiguous_with: List[str] = field(default_factory=list)

    @property
    def errors(self) -> int:
        return sum(f.errors for f in self.frames)

    @property
    def symbols(self) -> int:
        return sum(f.symbols for f in self.frames)

    @property
    def n_bits(self) -> int:
        return sum(len(f.bits) for f in self.frames)

    @property
    def error_rate(self) -> float:
        return self.errors / max(self.symbols, 1)

    @property
    def family(self) -> str:
        return "uart" if self.decoder.startswith("uart") else self.decoder

    @property
    def clean(self) -> bool:
        return self.error_rate <= CLEAN and self.n_bits >= 8

    def prior(self) -> float:
        return PRIORS.get(self.decoder, PRIORS.get(self.family, 0.0))


def _quantize(runs: Runs, unit: float) -> Tuple[np.ndarray, np.ndarray]:
    q = runs.durations / unit
    r = np.round(q).astype(np.int64)
    return r, np.abs(q - r) > OFF_GRID


def _frames(runs: Runs, limit_samples: float, any_level: bool = False) -> List[Tuple[int, int]]:
    """Run-index ranges [a, b) between delimiter runs (long runs, and the window edges)."""
    n = len(runs)
    if n < 3:
        return []
    delim = runs.durations > limit_samples
    if not any_level:
        delim &= runs.levels == runs.idle_level
    delim[0] = delim[-1] = True
    idx = np.flatnonzero(delim)
    return [(int(a) + 1, int(b)) for a, b in zip(idx[:-1], idx[1:]) if b - a > 1]


def _start_s(runs: Runs, a: int) -> float:
    return float(runs.starts[a] / runs.fs)


def decode_nrz(runs: Runs, unit: float, gap_units: float = 12.0) -> DecodeResult:
    q, off = _quantize(runs, unit)
    res = DecodeResult("nrz", note="1 = idle level")
    for a, b in _frames(runs, gap_units * unit):
        u = np.repeat(runs.levels[a:b], np.maximum(q[a:b], 1))
        bits = "".join("1" if v == runs.idle_level else "0" for v in u)
        res.frames.append(Frame(bits, int(off[a:b].sum()), b - a, _start_s(runs, a)))
    return res


def decode_uart(runs: Runs, unit: float, data_bits: int = 8, parity: str = "N",
                gap_units: float = 12.0) -> DecodeResult:
    q, off = _quantize(runs, unit)
    res = DecodeResult(f"uart-{data_bits}{parity}1", note="LSB first on the wire, shown MSB first")
    need = 1 + data_bits + (parity != "N") + 1
    for a, b in _frames(runs, gap_units * unit):
        mark = np.repeat(runs.levels[a:b], np.maximum(q[a:b], 1)) == runs.idle_level
        mark = np.concatenate([mark, np.ones(need, dtype=bool)])  # trailing idle shows the last stop bit
        body_len = len(mark) - need
        chunks, errors, nbytes, pos = [], int(off[a:b].sum()), 0, 0
        while pos < body_len:
            if mark[pos]:
                pos += 1
                continue
            data = mark[pos + 1: pos + 1 + data_bits]
            if parity != "N":
                ones = int(data.sum()) + int(mark[pos + 1 + data_bits])
                errors += int((parity == "E") == bool(ones % 2))
            errors += int(not mark[pos + need - 1])
            chunks.append("".join("1" if x else "0" for x in data[::-1]))
            nbytes += 1
            pos += need
        if nbytes:
            res.frames.append(Frame("".join(chunks), errors, nbytes, _start_s(runs, a)))
    return res


def decode_manchester(runs: Runs, unit: float, gap_units: float = 3.5) -> DecodeResult:
    q, off = _quantize(runs, unit)
    res = DecodeResult("manchester", note="IEEE 802.3 (low->high = 1); G.E. Thomas is the inverse")
    for a, b in _frames(runs, gap_units * unit, any_level=True):
        halves = np.repeat(runs.levels[a:b], np.clip(q[a:b], 1, 2))
        best = None
        # A half-bit at the idle level at either end of a frame merges into the idle run
        # beside it (a leading 0 starts high; a trailing 1 ends high). Try restoring one
        # at the start and/or the end, with both pair alignments. An unneeded pad only
        # leaves an unpaired half that is truncated, so it can never win on its own.
        idle = [bool(runs.idle_level)]
        for pad_start in (False, True):
            for pad_end in (False, True):
                h = np.array((idle if pad_start else []) + list(halves) + (idle if pad_end else []), dtype=bool)
                for phase in (0, 1):
                    s = h[phase:]
                    pairs = s[: len(s) // 2 * 2].reshape(-1, 2)
                    if len(pairs) and pairs[-1, 0] == pairs[-1, 1]:
                        pairs = pairs[:-1]
                    if not len(pairs):
                        continue
                    bad = int(np.sum(pairs[:, 0] == pairs[:, 1])) + int(off[a:b].sum())
                    bits = "".join("1" if (not x and y) else "0" for x, y in pairs)
                    if best is None or (bad, -len(bits)) < (best[0], -len(best[1])):
                        best = (bad, bits, len(pairs))
        if best:
            res.frames.append(Frame(best[1], best[0], best[2], _start_s(runs, a)))
    return res


def decode_bmc(runs: Runs, unit: float, gap_units: float = 2.5) -> DecodeResult:
    q, off = _quantize(runs, unit)
    res = DecodeResult("bmc", note="biphase-mark (extra mid-bit edge = 1); biphase-space is the inverse")
    for a, b in _frames(runs, gap_units * unit, any_level=True):
        qs, bits, err, i = q[a:b], [], int(off[a:b].sum()), 0
        while i < len(qs):
            if qs[i] == 2:
                bits.append("0")
                i += 1
            elif qs[i] == 1 and i + 1 < len(qs) and qs[i + 1] == 1:
                bits.append("1")
                i += 2
            elif qs[i] == 1 and i == len(qs) - 1 and runs.levels[b - 1] != runs.idle_level:
                bits.append("1")  # the second half of a final 1 merged into the trailing idle
                i += 1
            else:
                err += 1
                i += 1
        if bits:
            res.frames.append(Frame("".join(bits), err, len(bits), _start_s(runs, a)))
    return res


def _split(centers_us: List[float]) -> Tuple[float, float, float]:
    lo, hi = sorted(centers_us)[:2]
    return lo, hi, float(np.sqrt(lo * hi))


def decode_pulse_width(runs: Runs, model: TimingModel) -> DecodeResult:
    lo, hi, thr = _split([c.center_us for c in model.active])
    gap_limit = 3.0 * max((c.center_us for c in model.idle), default=hi)
    res = DecodeResult("pulse-width", note=f"active pulse > {thr:.0f} us = 1")
    du = runs.durations_us()
    for a, b in _frames(runs, gap_limit * runs.fs / 1e6):
        bits, err = [], 0
        for i in range(a, b):
            if runs.levels[i] == runs.idle_level:
                continue
            d = du[i]
            err += int(min(abs(d - lo) / lo, abs(d - hi) / hi) > OFF_GRID)
            bits.append("1" if d > thr else "0")
        if bits:
            res.frames.append(Frame("".join(bits), err, len(bits), _start_s(runs, a)))
    return res


def decode_pulse_distance(runs: Runs, model: TimingModel) -> DecodeResult:
    lo, hi, thr = _split([c.center_us for c in model.idle])
    res = DecodeResult("pulse-distance", note=f"gap > {thr:.0f} us = 1")
    du = runs.durations_us()
    for a, b in _frames(runs, 2.5 * hi * runs.fs / 1e6):
        bits, err = [], 0
        for i in range(a, b):
            if runs.levels[i] != runs.idle_level:
                continue
            d = du[i]
            err += int(min(abs(d - lo) / lo, abs(d - hi) / hi) > OFF_GRID)
            bits.append("1" if d > thr else "0")
        if bits:
            res.frames.append(Frame("".join(bits), err, len(bits), _start_s(runs, a)))
    return res


def ambiguity_hint(top: DecodeResult) -> str:
    for other in top.ambiguous_with:
        family = "uart" if other.startswith("uart") else other
        hint = EQUIVALENT.get(frozenset((top.family, family)))
        if hint:
            return hint
    return ""


def run_all(runs: Runs, model: TimingModel) -> List[DecodeResult]:
    results: List[DecodeResult] = []
    if model.unit_us:
        unit = model.unit_us * runs.fs / 1e6
        results += [decode_manchester(runs, unit), decode_bmc(runs, unit), decode_nrz(runs, unit)]
        results += [decode_uart(runs, unit, db, par) for db, par in ((8, "N"), (8, "E"), (8, "O"), (9, "N"))]
    if len(model.active) == 2:
        results.append(decode_pulse_width(runs, model))
    if len(model.idle) == 2:
        results.append(decode_pulse_distance(runs, model))
    results = [r for r in results if r.frames]

    def support(r: DecodeResult) -> float:
        return max((h.confidence for h in model.hypotheses if r.family in FAMILIES.get(h.encoding, ())),
                   default=0.0)

    def key(r: DecodeResult):
        # 1) clean decodes first, 2) agreement with the timing analysis, 3) the simpler
        # reading, 4) parsimony: the right code packs the most signal into each bit, so
        # among equally clean decodes fewer bits is better evidence than more.
        return (0 if r.clean else 1, 0.0 if r.clean else r.error_rate,
                -round(support(r), 2), r.prior(), r.n_bits)

    results.sort(key=key)
    if results and results[0].clean:
        top = results[0]
        amb = [r.decoder for r in results[1:]
               if r.clean and round(support(r), 2) == round(support(top), 2)
               and r.prior() == top.prior() and r.family != top.family]
        if top.family == "uart" and top.symbols < UART_MIN_BYTES:
            amb += [r.decoder for r in results[1:] if r.clean and r.family == "nrz" and r.decoder not in amb]
        top.ambiguous_with = amb
    return results
