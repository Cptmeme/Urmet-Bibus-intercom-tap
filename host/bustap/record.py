"""Talk to bus_tap_capture over UDP: discover it, watch live levels, record captures.

SAFETY: while bus_tap is wired to the intercom its GND is bus L2. The ESP32 must run
from an isolated supply with no USB link to this computer; this module only ever
talks to it over Wi-Fi.
"""

import socket
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

from . import protocol as proto
from .capture import (NOMINAL_CAL, SCALE, Capture, Channel, cal_points, capture_filename, new_meta,
                      raw_to_adc_volts)
from .dsp import noise_sigma, robust_sigma


@dataclass
class Stats:
    packets: int = 0
    samples: int = 0
    datagrams_lost: int = 0  # sequence gaps (can include lost INFO packets)
    sample_gaps: int = 0  # discontinuities in first_index: samples really missing
    overflows: int = 0  # samples lost inside the ESP before they were numbered
    rejected: int = 0  # datagrams that failed to parse
    epoch_changes: int = 0


@dataclass
class Block:
    """Samples received since the previous drain. Indices are absolute positions in
    the received stream of each channel; gaps[name] holds indices k where samples were
    lost between sample k-1 and sample k."""
    raw: Dict[str, np.ndarray] = field(default_factory=dict)
    start: Dict[str, int] = field(default_factory=dict)
    gaps: Dict[str, List[int]] = field(default_factory=dict)
    epoch_changed: bool = False

    def seconds(self, fs_channel: float) -> float:
        return max((len(a) for a in self.raw.values()), default=0) / fs_channel if fs_channel else 0.0


def channel_name(idx: int) -> str:
    return proto.CHANNEL_NAMES[idx] if idx < len(proto.CHANNEL_NAMES) else f"CH{idx}"


class Receiver:
    """Keeps the ESP streaming with a HELLO once a second, validates continuity and
    demultiplexes sample words into channels."""

    def __init__(self, esp: Optional[str] = None, port: int = proto.DEFAULT_PORT,
                 rate_hz: Optional[int] = None, channels: Optional[Sequence[str]] = None):
        self.port = port
        self.dest = esp or "255.255.255.255"
        self.hello = proto.make_hello(rate_hz, channels)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 21)
        self.sock.bind(("", 0))
        self.esp_addr = None
        self.info: Optional[dict] = None
        self.stats = Stats()
        self.rate_hz: Optional[int] = None
        self.n_channels: Optional[int] = None
        self.epoch: Optional[int] = None
        self._last_hello = -1e9
        self._next_seq = None
        self._next_index = None
        self._last_ovf = None
        self._fit: deque = deque(maxlen=20000)  # (first_index, esp_time_us) for rate measurement
        self._chunks: Dict[int, List[np.ndarray]] = {}
        self._total: Dict[int, int] = {}
        self._gaps: Dict[int, List[int]] = {}
        self._epoch_changed = False

    def close(self) -> None:
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    @property
    def fs_channel(self) -> float:
        rate = self.measured_rate() or self.rate_hz or 0
        return rate / self.n_channels if self.n_channels else 0.0

    def _send_hello(self) -> None:
        now = time.monotonic()
        if now - self._last_hello >= 1.0:
            host = self.esp_addr[0] if self.esp_addr else self.dest
            try:
                self.sock.sendto(self.hello, (host, self.port))
            except OSError:
                pass  # network down or no broadcast route; retry next second
            self._last_hello = now

    def poll(self, seconds: float = 0.1) -> int:
        """Receive for up to `seconds`; returns the number of packets handled."""
        self._send_hello()
        end, handled = time.monotonic() + seconds, 0
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return handled
            self.sock.settimeout(left)
            try:
                data, addr = self.sock.recvfrom(2048)
            except (socket.timeout, BlockingIOError):
                return handled
            except OSError:
                return handled  # e.g. ICMP port-unreachable surfacing as an error
            try:
                pkt = proto.parse(data)
            except proto.ProtocolError:
                self.stats.rejected += 1
                continue
            if self.esp_addr is None:
                self.esp_addr = addr  # lock on to the first bus_tap that answers
            elif addr[0] != self.esp_addr[0]:
                continue
            self._handle(pkt)
            handled += 1

    def _mark_gap(self) -> None:
        for c, total in self._total.items():
            gaps = self._gaps.setdefault(c, [])
            if total and (not gaps or gaps[-1] != total):
                gaps.append(total)

    def _handle(self, pkt: proto.Packet) -> None:
        h = pkt.header
        if self._next_seq is not None and h.seq != self._next_seq:
            self.stats.datagrams_lost += (h.seq - self._next_seq) % 2 ** 32
        self._next_seq = (h.seq + 1) % 2 ** 32
        if pkt.info is not None:
            self.info = pkt.info
            return

        if h.epoch != self.epoch:
            if self.epoch is not None:
                self.stats.epoch_changes += 1
                self._epoch_changed = True
                self._mark_gap()
            self.epoch, self.rate_hz, self.n_channels = h.epoch, h.rate_hz, h.n_channels
            self._next_index = None
            self._fit.clear()
        if self._last_ovf is not None and h.overflow_count != self._last_ovf:
            self.stats.overflows += (h.overflow_count - self._last_ovf) % 2 ** 32
            self._mark_gap()
        self._last_ovf = h.overflow_count
        if self._next_index is not None and h.first_index != self._next_index:
            if h.first_index < self._next_index:
                return  # duplicate or reordered datagram
            self.stats.sample_gaps += 1
            self._mark_gap()
        self._next_index = h.first_index + h.n_samples
        self._fit.append((h.first_index, h.esp_time_us))

        ch, raw = proto.split_words(pkt.words)
        for c in np.unique(ch):
            c = int(c)
            sel = raw[ch == c]
            self._chunks.setdefault(c, []).append(sel)
            self._total[c] = self._total.get(c, 0) + len(sel)
        self.stats.packets += 1
        self.stats.samples += len(raw)

    def drain(self) -> Block:
        blk = Block(epoch_changed=self._epoch_changed)
        self._epoch_changed = False
        for c, chunks in self._chunks.items():
            name = channel_name(c)
            arr = np.concatenate(chunks) if chunks else np.zeros(0, dtype=np.uint16)
            blk.raw[name] = arr
            blk.start[name] = self._total[c] - len(arr)
            blk.gaps[name] = list(self._gaps.get(c, []))
        self._chunks = {c: [] for c in self._chunks}
        self._gaps = {}
        return blk

    def measured_rate(self, min_span_s: float = 2.0) -> Optional[float]:
        """Total conversion rate from sample count against the ESP's own clock. The
        ESP32's continuous ADC can miss its nominal rate by a few percent, which would
        scale every pulse width; the crystal-timed esp_timer is good to ~20 ppm."""
        if len(self._fit) < 20:
            return None
        idx, t = np.asarray(self._fit, dtype=np.float64).T
        if t[-1] - t[0] < min_span_s * 1e6:
            return None
        rate = float(np.polyfit(t - t[0], idx - idx[0], 1)[0] * 1e6)
        if self.rate_hz and abs(rate / self.rate_hz - 1) > 0.05:
            return None  # implausible; fall back to the configured rate
        return rate

    def wait_for_stream(self, timeout: float = 10.0) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.poll(0.2)
            if self.info is not None and self.stats.packets:
                return
        raise TimeoutError(f"no bus_tap stream from {self.dest}:{self.port} within {timeout:.0f} s "
                           f"(is the ESP on Wi-Fi, and is UDP {self.port} allowed through?)")


class Track:
    """Accumulates drained blocks into one contiguous recording."""

    def __init__(self):
        self.chunks: Dict[str, List[np.ndarray]] = {}
        self.gaps: Dict[str, List[int]] = {}
        self.base: Dict[str, int] = {}

    def add(self, blk: Block) -> None:
        for name, arr in blk.raw.items():
            if name not in self.base:
                self.base[name] = blk.start[name]
                self.chunks[name], self.gaps[name] = [], []
            self.chunks[name].append(arr)
            for g in blk.gaps.get(name, []):
                rel = g - self.base[name]
                if rel > 0 and (not self.gaps[name] or self.gaps[name][-1] != rel):
                    self.gaps[name].append(rel)

    def to_capture(self, rx: Receiver, label: str, **extra) -> Capture:
        n_ch = rx.n_channels or max(1, len(self.chunks))
        fs_total = rx.measured_rate() or rx.rate_hz or 0
        chans = {name: Channel(name, np.concatenate(chunks) if chunks else np.zeros(0, dtype=np.uint16),
                               fs_total / n_ch, np.asarray(self.gaps.get(name, []), dtype=np.int64))
                 for name, chunks in self.chunks.items()}
        info = rx.info or {}
        meta = new_meta(label, cal=info.get("cal") or NOMINAL_CAL, esp=info,
                        esp_addr=rx.esp_addr[0] if rx.esp_addr else None,
                        fs_total_requested=rx.rate_hz, fs_total_measured=rx.measured_rate(),
                        # receiver_stats cover the receiver's whole life, including anything
                        # lost before this recording started; capture_gaps describe this file.
                        receiver_stats=asdict(rx.stats),
                        capture_gaps={name: len(g) for name, g in self.gaps.items()}, **extra)
        return Capture(chans, meta)


class ActivityDetector:
    """Streaming trigger. Learns each channel's resting level and noise during warm-up
    (with the ESP already transmitting, so Wi-Fi-induced noise is included), then
    flags any 10 ms slice with several samples beyond k sigma."""

    def __init__(self, k: float = 8.0, warmup_s: float = 3.0, slice_s: float = 0.01, min_hits: int = 3):
        self.k, self.warmup_s, self.slice_s, self.min_hits = k, warmup_s, slice_s, min_hits
        self._warm: Dict[str, List[np.ndarray]] = {}
        self._warm_s = 0.0
        self.level: Dict[str, float] = {}
        self.sigma: Dict[str, float] = {}

    @property
    def ready(self) -> bool:
        return bool(self.level)

    def feed(self, blk: Block, cal: Optional[dict], fs: float) -> bool:
        volts = {n: raw_to_adc_volts(a, cal) for n, a in blk.raw.items() if len(a)}
        if not volts or not fs:
            return False
        if not self.ready:
            for n, v in volts.items():
                self._warm.setdefault(n, []).append(v)
            self._warm_s += max(len(v) for v in volts.values()) / fs
            if self._warm_s >= self.warmup_s:
                raw_pts, mv_pts = cal_points(cal)
                lsb = (mv_pts[-1] - mv_pts[0]) / (raw_pts[-1] - raw_pts[0]) / 1000.0
                for n, chunks in self._warm.items():
                    w = np.concatenate(chunks)
                    self.level[n] = float(np.median(w))
                    self.sigma[n] = max(noise_sigma(w), robust_sigma(w), lsb)
            return False

        per_slice = max(8, int(self.slice_s * fs))
        active = False
        for n, v in volts.items():
            if n not in self.level:
                continue
            hits = np.abs(v - self.level[n]) > self.k * self.sigma[n]
            m = len(hits) // per_slice * per_slice
            if (m and (hits[:m].reshape(-1, per_slice).sum(axis=1) >= self.min_hits).any()) \
                    or hits[m:].sum() >= self.min_hits:
                active = True
        if not active:  # follow slow drift in the resting level
            for n, v in volts.items():
                if n in self.level:
                    self.level[n] += 0.05 * (float(np.median(v)) - self.level[n])
        return active


def monitor(rx: Receiver, seconds: Optional[float] = None, interval: float = 0.5,
            out=sys.stdout) -> None:
    """One status line per interval: bus volts, AC swing, floor-call loop, link health."""
    start = time.monotonic()
    while seconds is None or time.monotonic() - start < seconds:
        t_end = time.monotonic() + interval
        while time.monotonic() < t_end:
            rx.poll(0.05)
        blk = rx.drain()
        if not any(len(a) for a in blk.raw.values()):
            print(f"waiting for bus_tap at {rx.dest}:{rx.port} ...", file=out, flush=True)
            continue
        cal = (rx.info or {}).get("cal")
        parts = []
        for name in ("DC", "AC", "FC"):
            arr = blk.raw.get(name)
            if arr is None or not len(arr):
                continue
            v = raw_to_adc_volts(arr, cal)
            if name == "AC":
                parts.append(f"AC {1000 * (v.max() - v.min()):6.1f} mVpp")
            else:
                bus = v * SCALE[name]
                parts.append(f"{name} {bus.mean():6.2f} V [{bus.min():.2f}..{bus.max():.2f}]")
        rate, st, info = rx.measured_rate(), rx.stats, rx.info or {}
        parts.append(f"| {rx.rate_hz or 0} Hz" + (f" (measured {rate:.0f})" if rate else "")
                     + f"  rssi {info.get('rssi', '?')}  lost {st.datagrams_lost}  gaps {st.sample_gaps}"
                     + f"  ovf {st.overflows}")
        print("  ".join(parts), file=out, flush=True)


def record_for(rx: Receiver, seconds: float, out_dir, label: str,
               log: Callable[[str], None] = print) -> Path:
    rx.wait_for_stream()
    rx.drain()
    track, end = Track(), time.monotonic() + seconds
    while time.monotonic() < end:
        rx.poll(0.1)
        blk = rx.drain()
        if blk.epoch_changed:
            raise RuntimeError("ESP configuration changed mid-recording; not mixing sample rates")
        track.add(blk)
    cap = track.to_capture(rx, label, mode="fixed", seconds=seconds)
    path = cap.save(Path(out_dir) / capture_filename(label))
    gaps = max(cap.meta["capture_gaps"].values(), default=0)
    log(f"saved {path}  {cap.duration_s():.2f} s, {gaps} gap(s) in this capture")
    return path


def record_gated(rx: Receiver, out_dir, label: str, pre_s: float = 1.0, post_s: float = 1.5,
                 max_s: float = 60.0, k: float = 8.0, warmup_s: float = 3.0, count: Optional[int] = None,
                 log: Callable[[str], None] = print) -> List[Path]:
    """Wait for bus activity and save one capture per event, with `pre_s` of history
    before the trigger and until `post_s` of quiet after it. Ctrl-C to stop."""
    rx.wait_for_stream()
    rx.drain()
    det = ActivityDetector(k=k, warmup_s=warmup_s)
    ring: deque = deque()
    ring_s = quiet_s = rec_s = 0.0
    track: Optional[Track] = None
    saved: List[Path] = []
    announced = False
    try:
        while count is None or len(saved) < count:
            rx.poll(0.1)
            blk = rx.drain()
            fs = rx.fs_channel
            dur = blk.seconds(fs)
            if not dur:
                continue
            active = det.feed(blk, (rx.info or {}).get("cal"), fs)
            if det.ready and not announced:
                log("baseline learned: " + ", ".join(f"{n} {det.level[n]:.3f} V ±{1000 * det.sigma[n]:.2f} mV (ADC)"
                                                     for n in det.level) + " - waiting for activity")
                announced = True
            if track is None:
                ring.append((blk, dur))
                ring_s += dur
                while ring and ring_s - ring[0][1] >= pre_s:
                    ring_s -= ring.popleft()[1]
                if active:
                    track = Track()
                    for b, _ in ring:
                        track.add(b)
                    ring.clear()
                    ring_s, quiet_s, rec_s = 0.0, 0.0, 0.0
                    log("activity - recording")
                continue
            track.add(blk)
            rec_s += dur
            quiet_s = 0.0 if active else quiet_s + dur
            if quiet_s >= post_s or rec_s >= max_s or blk.epoch_changed:
                cap = track.to_capture(rx, label, mode="gated", pre_s=pre_s, post_s=post_s, k=k)
                path = cap.save(Path(out_dir) / capture_filename(label))
                saved.append(path)
                log(f"saved {path}  {cap.duration_s():.2f} s")
                track = None
    except KeyboardInterrupt:
        if track is not None:
            cap = track.to_capture(rx, label, mode="gated", pre_s=pre_s, post_s=post_s, k=k, interrupted=True)
            saved.append(cap.save(Path(out_dir) / capture_filename(label)))
            log(f"saved partial {saved[-1]}")
    return saved
