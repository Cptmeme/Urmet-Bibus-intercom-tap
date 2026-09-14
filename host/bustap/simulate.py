"""Synthetic bus traffic, for testing the analysis without the intercom.

The bus rests high (22.5 V) and "active" symbols pull it down, as on a current-sinking
2-wire bus. Waveforms go through a model of the bus_tap front end and ADC, so the
analysis sees the same channel behaviour it will see on real captures.
"""

from typing import List, Sequence, Tuple

import numpy as np

from .capture import Capture, Channel, DIVIDER, NOMINAL_CAL, new_meta
from .dsp import moving_average

ENCODINGS = ("manchester", "bmc", "uart", "pwm", "pdm", "nrz", "fsk", "dtmf")
Segment = Tuple[bool, float]  # (level, seconds); True = idle/high


def bits_of(payload: bytes, msb_first: bool = True) -> List[int]:
    out = []
    for byte in payload:
        seq = range(7, -1, -1) if msb_first else range(8)
        out += [(byte >> i) & 1 for i in seq]
    return out


def _merge(segs: List[Segment]) -> List[Segment]:
    out: List[Segment] = []
    for lvl, d in segs:
        if out and out[-1][0] == lvl:
            out[-1] = (lvl, out[-1][1] + d)
        else:
            out.append((lvl, d))
    return out


def frame_segments(encoding: str, payload: bytes, unit_s: float) -> List[Segment]:
    """Level segments for one frame (no surrounding idle). unit_s is the half-bit for
    Manchester/BMC and the bit (or pulse unit) for everything else."""
    u, s = unit_s, []
    bits = bits_of(payload)
    if encoding == "manchester":
        for b in bits:  # IEEE 802.3: 1 = low->high
            s += [(False, u), (True, u)] if b else [(True, u), (False, u)]
    elif encoding == "bmc":
        lvl = True
        for b in bits:
            lvl = not lvl
            if b:
                s += [(lvl, u), (not lvl, u)]
                lvl = not lvl
            else:
                s.append((lvl, 2 * u))
    elif encoding == "uart":
        for byte in payload:
            s.append((False, u))  # start
            s += [(bool((byte >> i) & 1), u) for i in range(8)]  # LSB first, 1 = mark = idle
            s += [(True, u), (True, 2 * u)]  # stop + inter-byte idle
    elif encoding == "pwm":
        for b in bits:
            s += [(False, 2 * u), (True, u)] if b else [(False, u), (True, 2 * u)]
    elif encoding == "pdm":
        for b in bits:
            s += [(False, u), (True, 3 * u if b else u)]
        s.append((False, u))  # closing pulse ends the last gap
    elif encoding == "nrz":
        s.append((False, u))
        s += [(bool(b), u) for b in bits]
    else:
        raise ValueError(f"no baseband model for {encoding!r}")
    return _merge(s)


def render_bus(segments: Sequence[Segment], fs: float, idle_v: float = 22.5, dip_v: float = 3.0,
               rise_s: float = 20e-6) -> np.ndarray:
    total = sum(d for _, d in segments)
    v = np.full(int(round(total * fs)), idle_v)
    t = 0.0
    for lvl, d in segments:
        if not lvl:
            v[int(round(t * fs)): int(round((t + d) * fs))] = idle_v - dip_v
        t += d
    return moving_average(v, max(1, int(rise_s * fs)))


def render_fsk(bits: Sequence[int], bit_s: float, fs: float, f0: float = 1200.0, f1: float = 2200.0,
               idle_v: float = 22.5, amp_v: float = 0.5) -> np.ndarray:
    n = int(round(len(bits) * bit_s * fs))
    idx = np.minimum((np.arange(n) / (bit_s * fs)).astype(int), len(bits) - 1)
    freq = np.where(np.asarray(bits)[idx] == 1, f1, f0)
    phase = 2 * np.pi * np.cumsum(freq) / fs
    return idle_v + amp_v * np.sin(phase)


def render_dtmf(keys: str, fs: float, on_s: float = 0.06, off_s: float = 0.06,
                idle_v: float = 22.5, amp_v: float = 0.5) -> np.ndarray:
    from .tones import DTMF_HIGH, DTMF_KEYS, DTMF_LOW
    parts = []
    for key in keys:
        row = next(r for r, row in enumerate(DTMF_KEYS) if key in row)
        t = np.arange(int(on_s * fs)) / fs
        tone = np.sin(2 * np.pi * DTMF_LOW[row] * t) + np.sin(2 * np.pi * DTMF_HIGH[DTMF_KEYS[row].index(key)] * t)
        parts += [idle_v + amp_v / 2 * tone, np.full(int(off_s * fs), idle_v)]
    return np.concatenate(parts)


def _highpass(x: np.ndarray, fs: float, tau_s: float) -> np.ndarray:
    a = tau_s / (tau_s + 1.0 / fs)
    y = np.empty_like(x)
    y[0] = 0.0
    for i in range(1, len(x)):
        y[i] = a * (y[i - 1] + x[i] - x[i - 1])
    return y


def bus_to_capture(bus_v: np.ndarray, fs_os: float, fs_total: float, channels=("DC", "AC", "FC"),
                   fc_v: float = 12.0, adc_noise_lsb: float = 1.5, label: str = "sim", seed: int = 0,
                   **meta) -> Capture:
    """Sample an oversampled bus waveform the way the multiplexed ESP32 ADC would."""
    rng = np.random.default_rng(seed)
    n_ch = len(channels)
    os_ = int(round(fs_os / fs_total))
    fs_ch = fs_total / n_ch
    full_scale = NOMINAL_CAL["mv"][-1] / 1000.0
    chans = {}
    for k, name in enumerate(channels):
        v = bus_v[k * os_:: n_ch * os_]
        if name == "DC":
            adc = v / DIVIDER
        elif name == "AC":
            adc = 1.65 + _highpass(v, fs_ch, 0.05)
        else:
            adc = np.full(len(v), fc_v / DIVIDER)
        raw = adc / full_scale * 4095 + rng.normal(0, adc_noise_lsb, len(adc))
        chans[name] = Channel(name, np.clip(np.round(raw), 0, 4095).astype(np.uint16), fs_ch)
    m = new_meta(label, cal=NOMINAL_CAL, fs_total_requested=fs_total, simulated=True)
    m.update(meta)
    return Capture(chans, m)


def make_capture(encoding: str, payload: bytes = b"\x5a\x3c", bit_rate: float = 1000.0,
                 fs_total: float = 60000.0, repeats: int = 2, lead_s: float = 0.3, gap_s: float = 0.08,
                 dip_v: float = 3.0, label: str = "", seed: int = 0, oversample: int = 8) -> Capture:
    fs_os = fs_total * oversample
    idle = np.full(int(lead_s * fs_os), 22.5)
    gap = np.full(int(gap_s * fs_os), 22.5)
    if encoding == "fsk":
        frame = render_fsk(bits_of(payload), 1.0 / bit_rate, fs_os)
    elif encoding == "dtmf":
        frame = render_dtmf(payload.decode(), fs_os)
    else:
        unit = 1.0 / bit_rate / (2 if encoding in ("manchester", "bmc") else 1)
        frame = render_bus(frame_segments(encoding, payload, unit), fs_os, dip_v=dip_v)
    parts = [idle]
    for _ in range(repeats):
        parts += [frame, gap]
    parts.append(idle)
    return bus_to_capture(np.concatenate(parts), fs_os, fs_total, label=label or encoding, seed=seed,
                          sim={"encoding": encoding, "payload_hex": payload.hex(), "bit_rate": bit_rate})
