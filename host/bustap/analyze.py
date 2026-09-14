"""Run the whole pipeline over one capture and describe what was found."""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from .capture import Capture
from .decoders import DecodeResult, ambiguity_hint, run_all
from .dsp import Event, Levels, Runs, estimate_levels, find_events, noise_sigma, slice_to_runs
from .infer import TimingModel, infer_timing
from .tones import Peak, detect_dtmf, fsk_demod, looks_tonal, spectrum_peaks

BUS_CHANNELS = ("DC", "AC")


@dataclass
class EventReport:
    index: int
    t_start_s: float
    t_end_s: float
    channel: str  # channel the bits were sliced from ("AC/fsk" when demodulated)
    source: str  # "bus" or "floor-call"
    levels: Optional[Levels] = None
    runs: Optional[Runs] = None
    model: Optional[TimingModel] = None
    decodes: List[DecodeResult] = field(default_factory=list)
    peaks: List[Peak] = field(default_factory=list)
    tonal: bool = False
    fsk_tones: Optional[Tuple[float, float]] = None
    dtmf: List[Tuple[float, str]] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)

    @property
    def duration_ms(self) -> float:
        return (self.t_end_s - self.t_start_s) * 1e3

    @property
    def best(self) -> Optional[DecodeResult]:
        return self.decodes[0] if self.decodes else None


def _events_s(cap: Capture, names, k: float) -> List[Tuple[float, float]]:
    spans = []
    for name in names:
        ch = cap.channels[name]
        for e in find_events(cap.bus_volts(name), ch.fs, k=k, gaps=ch.gaps,
                             sigma_floor=cap.lsb_bus_volts(name)):
            spans.append((e.start / ch.fs, e.end / ch.fs))
    spans.sort()
    merged: List[List[float]] = []
    for a, b in spans:
        if merged and a <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    # Merging DC and AC spans can rejoin pieces find_events split at a capture gap, and
    # a pulse measured across lost samples has a meaningless width: cut again at gaps.
    gap_times = sorted({g / cap.channels[n].fs for n in names for g in cap.channels[n].gaps})
    out: List[Tuple[float, float]] = []
    for a, b in merged:
        edges = [a] + [t for t in gap_times if a < t < b] + [b]
        out += [(x, y) for x, y in zip(edges[:-1], edges[1:]) if y - x > 0.0005]
    return out


def _clipped(raw: np.ndarray) -> bool:
    return raw.size > 0 and np.mean((raw <= 2) | (raw >= 4093)) > 0.005


def _analyze_span(cap: Capture, idx: int, a_s: float, b_s: float, names, source: str,
                  sigmas: dict, adaptive: bool) -> EventReport:
    rep = EventReport(idx, a_s, b_s, "", source)

    # Pick the channel on which the two logic levels stand furthest out of the noise.
    best = None
    for name in names:
        ch = cap.channels[name]
        i0, i1 = int(a_s * ch.fs), min(len(ch.raw), int(b_s * ch.fs))
        if i1 - i0 < 16:
            continue
        seg = cap.bus_volts(name)[i0:i1]
        lv = estimate_levels(seg, sigmas[name])
        score = lv.separation_sigma * (0.5 if _clipped(ch.raw[i0:i1]) else 1.0)
        if best is None or score > best[0]:
            best = (score, name, seg, lv, ch.fs, _clipped(ch.raw[i0:i1]))
    if best is None:
        rep.notes.append("event too short to analyse")
        return rep
    _, name, seg, lv, fs, clipped = best
    rep.channel, rep.levels = name, lv
    if clipped:
        rep.notes.append(f"{name} clips the ADC in this event; levels are compressed (timing still usable)")
    if lv.separation_sigma < 10:
        rep.notes.append(f"levels only {lv.separation_sigma:.1f} sigma apart - slicing is unreliable")

    # Audio-band first: slicing a tone as baseband produces convincing nonsense.
    tone_src = cap.bus_volts("AC")[int(a_s * cap.channels["AC"].fs): int(b_s * cap.channels["AC"].fs)] \
        if "AC" in cap.channels and source == "bus" else seg
    rep.peaks = spectrum_peaks(tone_src, fs)
    rep.tonal = looks_tonal(tone_src, fs)
    if rep.tonal:
        rep.dtmf = detect_dtmf(tone_src, fs)
        demod = fsk_demod(tone_src, fs)
        if demod is not None:
            seg, rep.fsk_tones = demod[0], (demod[1], demod[2])
            rep.channel, lv = "AC/fsk", estimate_levels(demod[0])
            rep.levels = lv
            rep.notes.append(f"two-tone FSK: {demod[1]:.0f} Hz = 0, {demod[2]:.0f} Hz = 1")
        elif not rep.dtmf:
            rep.notes.append("tonal but neither FSK nor DTMF; could be speech or a ring tone")
            return rep

    adaptive_window = None
    if adaptive:
        adaptive_window = max(16, int(0.02 * fs))
    rep.runs, rep.levels = slice_to_runs(seg, fs, lv, adaptive_window)
    rep.model = infer_timing(rep.runs)
    rep.decodes = run_all(rep.runs, rep.model)
    return rep


def analyze(cap: Capture, k: float = 8.0, adaptive: bool = False) -> List[EventReport]:
    sigmas = {n: max(noise_sigma(cap.bus_volts(n)), cap.lsb_bus_volts(n)) for n in cap.channels}
    reports: List[EventReport] = []
    bus = [n for n in BUS_CHANNELS if n in cap.channels]
    if bus:
        for a, b in _events_s(cap, bus, k):
            reports.append(_analyze_span(cap, len(reports), a, b, bus, "bus", sigmas, adaptive))
    if "FC" in cap.channels:
        for a, b in _events_s(cap, ["FC"], k):
            reports.append(_analyze_span(cap, len(reports), a, b, ["FC"], "floor-call", sigmas, adaptive))
    reports.sort(key=lambda r: r.t_start_s)
    for i, r in enumerate(reports):
        r.index = i
    return reports


def _frame_text(bits: str) -> str:
    if len(bits) >= 8 and len(bits) % 8 == 0:
        return bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits), 8)).hex(" ")
    return bits


def format_report(cap: Capture, reports: List[EventReport], path: str = "", top: int = 3) -> str:
    out = [f"capture {path}  label={cap.label or '-'}  {cap.duration_s():.2f} s"]
    for n, ch in cap.channels.items():
        out.append(f"  {n}: {ch.fs:.0f} samples/s, {len(ch.gaps)} gaps")
    if not reports:
        out.append("no events found - bus idle, or the threshold (k) is too high")
    for r in reports:
        out.append("")
        head = f"event {r.index}  {r.t_start_s:.3f}-{r.t_end_s:.3f} s ({r.duration_ms:.1f} ms)  [{r.source}]"
        if r.levels:
            head += (f"  channel {r.channel}  levels {r.levels.low:.2f} / {r.levels.high:.2f} V"
                     f"  ({r.levels.separation_sigma:.0f} sigma)")
        out.append(head)
        for note in r.notes:
            out.append(f"  ! {note}")
        if r.peaks:
            out.append("  spectrum: " + ", ".join(f"{p.freq_hz:.0f} Hz {100 * p.rel_power:.0f}%" for p in r.peaks[:3])
                       + ("  (tonal)" if r.tonal else ""))
        if r.dtmf:
            out.append("  DTMF: " + " ".join(f"{k}@{t * 1e3:.0f}ms" for t, k in r.dtmf))
        if r.model:
            out.append("  " + r.model.report().replace("\n", "\n  "))
        if r.best and r.best.ambiguous_with:
            out.append(f"  ~ equally clean: {', '.join(r.best.ambiguous_with)} - timing alone cannot choose")
            hint = ambiguity_hint(r.best)
            if hint:
                out.append(f"    ({hint})")
        for d in r.decodes[:top]:
            out.append(f"  decode {d.decoder:<14s} errors {d.errors}/{d.symbols} ({d.error_rate:.3f})  {d.note}")
            for f in d.frames[:6]:
                out.append(f"      @{f.start_s * 1e3:7.1f} ms  {len(f.bits):4d} bits  {_frame_text(f.bits)}")
    return "\n".join(out)
