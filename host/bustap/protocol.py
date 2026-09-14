"""UDP wire format shared with firmware/bus_tap_capture/main/protocol.h.

tests/test_protocol.py compiles protocol.h with the system C compiler and checks the
header layout below against it, so the two cannot drift apart silently.
"""

import json
import struct
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import numpy as np

MAGIC = 0x31544255  # "UBT1"
TYPE_SAMPLES = 1
TYPE_INFO = 2
HELLO = b"UBTHELLO"

DEFAULT_PORT = 7777
MAX_CHANNELS = 3
CHANNEL_NAMES = ("DC", "AC", "FC")
CH_SHIFT = 12
DATA_MASK = 0x0FFF
MAX_SAMPLES_PER_PKT = 686

HDR = struct.Struct("<IBBHIIQQII")
HDR_SIZE = HDR.size
HDR_FIELDS = (
    "magic", "type", "n_channels", "n_samples", "seq",
    "epoch", "first_index", "esp_time_us", "rate_hz", "overflow_count",
)


class ProtocolError(ValueError):
    pass


@dataclass
class Header:
    magic: int
    type: int
    n_channels: int
    n_samples: int
    seq: int
    epoch: int
    first_index: int
    esp_time_us: int
    rate_hz: int
    overflow_count: int


@dataclass
class Packet:
    header: Header
    words: Optional[np.ndarray] = None  # uint16 sample words, TYPE_SAMPLES only
    info: Optional[dict] = None  # decoded JSON, TYPE_INFO only


def parse(datagram: bytes) -> Packet:
    if len(datagram) < HDR_SIZE:
        raise ProtocolError(f"datagram of {len(datagram)} bytes is shorter than the header")
    h = Header(*HDR.unpack_from(datagram))
    if h.magic != MAGIC:
        raise ProtocolError(f"bad magic 0x{h.magic:08x}")
    body = datagram[HDR_SIZE:]
    if h.type == TYPE_SAMPLES:
        if len(body) != 2 * h.n_samples:
            raise ProtocolError(f"header says {h.n_samples} samples, body holds {len(body) / 2:g}")
        return Packet(h, words=np.frombuffer(body, dtype="<u2").copy())
    if h.type == TYPE_INFO:
        try:
            return Packet(h, info=json.loads(body.decode("utf-8")))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ProtocolError(f"unreadable info payload: {exc}") from exc
    raise ProtocolError(f"unknown packet type {h.type}")


def _header(type_, n_samples, seq, epoch, first_index, esp_time_us, rate_hz, n_channels, overflow_count):
    return HDR.pack(MAGIC, type_, n_channels, n_samples, seq, epoch,
                    first_index, esp_time_us, rate_hz, overflow_count)


def build_samples(seq: int, epoch: int, first_index: int, esp_time_us: int, rate_hz: int,
                  n_channels: int, words: np.ndarray, overflow_count: int = 0) -> bytes:
    words = np.asarray(words, dtype="<u2")
    if len(words) > MAX_SAMPLES_PER_PKT:
        raise ProtocolError(f"{len(words)} samples exceed the per-packet limit")
    return _header(TYPE_SAMPLES, len(words), seq, epoch, first_index, esp_time_us, rate_hz,
                   n_channels, overflow_count) + words.tobytes()


def build_info(seq: int, epoch: int, esp_time_us: int, rate_hz: int, n_channels: int,
               info: dict, overflow_count: int = 0) -> bytes:
    return _header(TYPE_INFO, 0, seq, epoch, 0, esp_time_us, rate_hz, n_channels,
                   overflow_count) + json.dumps(info, separators=(",", ":")).encode()


def make_hello(rate_hz: Optional[int] = None, channels: Optional[Iterable[str]] = None) -> bytes:
    parts = [HELLO.decode()]
    if rate_hz:
        parts.append(f"rate={int(rate_hz)}")
    if channels:
        parts.append("ch=" + ",".join(c.upper() for c in channels))
    return " ".join(parts).encode()


def encode_words(channel_index: np.ndarray, raw: np.ndarray) -> np.ndarray:
    return ((np.asarray(channel_index, dtype=np.uint16) << CH_SHIFT)
            | (np.asarray(raw, dtype=np.uint16) & DATA_MASK)).astype("<u2")


def split_words(words: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    words = np.asarray(words, dtype=np.uint16)
    return (words >> CH_SHIFT).astype(np.uint8), (words & DATA_MASK).astype(np.uint16)
