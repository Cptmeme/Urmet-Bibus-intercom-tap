"""Capture files (.npz) and conversion from raw ADC codes to bus voltages.

Channel meaning, set by the bus_tap front end:
  DC  1M/100k divider, unity buffer   bus volts  = adc volts * 11
  AC  AC-coupled, gain 11             bus swing  = adc volts - resting level
                                      (the gain cancels the 1/11 divider; high-pass
                                      tau ~50 ms, so only edges and short levels survive)
  FC  1M/100k divider on C1/C2        loop volts = adc volts * 11
"""

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

FORMAT_VERSION = 1
DIVIDER = 11.0

# Fallback when the ESP reports no eFuse calibration: ESP32 ADC1 at 12 dB attenuation.
# Approximate; readings above ~2.45 V are compressed on the real part.
NOMINAL_CAL = {"raw": [0, 4095], "mv": [0, 3100]}
SCALE = {"DC": DIVIDER, "FC": DIVIDER, "AC": 1.0}  # ADC volts -> bus volts


def cal_points(cal: Optional[dict]):
    """(raw codes, millivolts) from the ESP's calibration table, or the nominal curve."""
    raw, mv = (cal or {}).get("raw"), (cal or {}).get("mv")
    if not raw or not mv or len(raw) != len(mv) or min(mv) < 0:
        return NOMINAL_CAL["raw"], NOMINAL_CAL["mv"]
    return raw, mv


def raw_to_adc_volts(raw, cal: Optional[dict] = None) -> np.ndarray:
    raw_pts, mv_pts = cal_points(cal)
    return np.interp(np.asarray(raw, dtype=np.float64), raw_pts, mv_pts) / 1000.0


@dataclass
class Channel:
    name: str
    raw: np.ndarray  # uint16 ADC codes
    fs: float  # samples per second on this channel
    gaps: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.int64))
    # gaps[i] = k means samples were lost between raw[k-1] and raw[k]

    def segments(self, min_len: int = 1) -> List[Tuple[int, int]]:
        """Contiguous (start, end) index ranges between gaps."""
        edges = [0] + [int(g) for g in self.gaps if 0 < g < len(self.raw)] + [len(self.raw)]
        return [(a, b) for a, b in zip(edges[:-1], edges[1:]) if b - a >= min_len]


@dataclass
class Capture:
    channels: Dict[str, Channel]
    meta: dict = field(default_factory=dict)

    @property
    def label(self) -> str:
        return self.meta.get("label", "")

    def adc_volts(self, name: str) -> np.ndarray:
        return raw_to_adc_volts(self.channels[name].raw, self.meta.get("cal"))

    def lsb_bus_volts(self, name: str) -> float:
        """Bus-referred size of one ADC code. Used as a noise floor: on a signal so clean
        the ADC barely moves, measured noise is ~0 and single-code steps would trigger."""
        raw_pts, mv_pts = cal_points(self.meta.get("cal"))
        per_code = (mv_pts[-1] - mv_pts[0]) / (raw_pts[-1] - raw_pts[0]) / 1000.0
        return per_code * SCALE.get(name, 1.0)

    def bus_volts(self, name: str) -> np.ndarray:
        v = self.adc_volts(name)
        if name == "AC":
            return v - np.median(v)
        return v * DIVIDER

    def duration_s(self) -> float:
        return max((len(c.raw) / c.fs for c in self.channels.values()), default=0.0)

    def save(self, path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        meta = dict(self.meta)
        meta["format_version"] = FORMAT_VERSION
        meta["channels"] = {n: {"fs": c.fs} for n, c in self.channels.items()}
        arrays = {}
        for n, c in self.channels.items():
            arrays[f"raw_{n}"] = c.raw.astype(np.uint16)
            arrays[f"gaps_{n}"] = np.asarray(c.gaps, dtype=np.int64)
        np.savez_compressed(path, meta=np.array(json.dumps(meta)), **arrays)
        return path

    @classmethod
    def load(cls, path) -> "Capture":
        with np.load(Path(path), allow_pickle=False) as z:
            meta = json.loads(str(z["meta"]))
            chans = {}
            for n, info in meta.get("channels", {}).items():
                chans[n] = Channel(n, z[f"raw_{n}"].astype(np.uint16), float(info["fs"]),
                                   z[f"gaps_{n}"].astype(np.int64))
        return cls(chans, meta)


def new_meta(label: str = "", **extra) -> dict:
    meta = {"label": label, "created": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    meta.update(extra)
    return meta


def capture_filename(label: str, when: Optional[float] = None) -> str:
    when = time.time() if when is None else when
    # milliseconds: gated recording can close two captures within the same second
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(when)) + f"-{int(when % 1 * 1000):03d}"
    safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in label) or "capture"
    return f"{safe}_{stamp}.npz"
