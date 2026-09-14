"""A stand-in for the firmware: serves a capture (real or simulated) over the real
UDP protocol, so the recorder can be exercised without the intercom or an ESP32.

It honours the channel set a HELLO asks for, but not a different rate: it can only
replay the capture at the rate it was recorded.
"""

import socket
import time
from typing import Optional

import numpy as np

from . import protocol as proto
from .capture import NOMINAL_CAL, Capture


class FakeEsp:
    def __init__(self, cap: Capture, host: str = "127.0.0.1", port: int = 0, speed: float = 1.0,
                 drop_every: int = 0, loop: bool = True):
        names = [n for n in proto.CHANNEL_NAMES if n in cap.channels]
        if not names:
            raise ValueError("capture has none of the DC/AC/FC channels")
        n = min(len(cap.channels[x].raw) for x in names)
        idx = np.array([proto.CHANNEL_NAMES.index(x) for x in names], dtype=np.uint16)
        raw = np.stack([cap.channels[x].raw[:n] for x in names], axis=1).reshape(-1)
        self.words = proto.encode_words(np.tile(idx, n), raw)
        self.fs_total = cap.channels[names[0]].fs * len(names)
        self.names, self.cap = names, cap
        self.speed, self.drop_every, self.loop = speed, drop_every, loop
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.address = self.sock.getsockname()
        self.sent_packets = self.dropped_packets = 0

    def close(self) -> None:
        self.sock.close()

    def _info(self, epoch: int) -> dict:
        return {"fw": "fake_esp/1", "target": "host", "epoch": epoch, "rate_hz": int(self.fs_total),
                "atten_db": 12, "bits": 12,
                "channels": [{"idx": proto.CHANNEL_NAMES.index(n), "name": n, "gpio": -1, "valid": True,
                              "enabled": True} for n in self.names],
                "cal": self.cap.meta.get("cal") or NOMINAL_CAL, "overflows": 0, "tx_drops": 0,
                "uptime_s": 0, "rssi": -40}

    def serve(self, seconds: Optional[float] = None) -> None:
        client, last_hello, start = None, 0.0, time.monotonic()
        stream_t0, sent, seq, epoch, last_info = None, 0, 0, 1, 0.0
        per = proto.MAX_SAMPLES_PER_PKT
        self.sock.settimeout(0.002)
        while seconds is None or time.monotonic() - start < seconds:
            now = time.monotonic()
            try:
                data, addr = self.sock.recvfrom(256)
                if data.startswith(proto.HELLO):
                    client, last_hello = addr, now
                    if stream_t0 is None:
                        stream_t0 = now
                    self.sock.sendto(proto.build_info(seq, epoch, 0, int(self.fs_total), len(self.names),
                                                      self._info(epoch)), client)
                    seq += 1
                    last_info = now
            except (socket.timeout, BlockingIOError):
                pass
            if client is None or now - last_hello > 5.0:
                continue
            if now - last_info > 1.0:
                self.sock.sendto(proto.build_info(seq, epoch, int(sent / self.fs_total * 1e6),
                                                  int(self.fs_total), len(self.names), self._info(epoch)), client)
                seq += 1
                last_info = now
            due = int((now - stream_t0) * self.fs_total * self.speed)
            while sent + per <= due:
                lo = sent % len(self.words)
                if not self.loop and sent + per > len(self.words):
                    return
                chunk = np.take(self.words, np.arange(lo, lo + per), mode="wrap")
                pkt = proto.build_samples(seq, epoch, sent, int(sent / self.fs_total * 1e6),
                                          int(self.fs_total), len(self.names), chunk)
                if self.drop_every and seq % self.drop_every == self.drop_every - 1:
                    self.dropped_packets += 1  # consumed a seq and indices, never sent: wire loss
                else:
                    self.sock.sendto(pkt, client)
                    self.sent_packets += 1
                seq += 1
                sent += per
