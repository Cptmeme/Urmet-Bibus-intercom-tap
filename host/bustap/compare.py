"""Line up frames from differently-labelled captures to find the fields.

Record the same kind of event several times, and different kinds of event
(ring_own, ring_neighbour, door_open, handset_up...). Then, per bit position:
  '='  identical in every frame                 -> preamble, sync, fixed header
  'L'  constant within a label, differs across  -> address / command field
  '*'  varies even within one label             -> counter, checksum, or slicing noise
"""

from dataclasses import dataclass, field
from functools import reduce
from typing import Dict, List, Optional, Sequence, Tuple

MIN_DISTINCT = 4  # distinct frames needed before a checksum match counts as confirmed


@dataclass
class LabeledFrame:
    label: str
    bits: str
    source: str = ""


def _correlation(ref: str, bits: str, shift: int) -> Tuple[int, int]:
    """(agreements minus disagreements, overlap). Scoring raw agreement rather than
    the agreeing fraction stops a short chance overlap from beating the true alignment."""
    lo, hi = max(0, shift), min(len(ref), shift + len(bits))
    if hi <= lo:
        return -(10 ** 9), 0
    same = sum(1 for i in range(lo, hi) if ref[i] == bits[i - shift])
    overlap = hi - lo
    return 2 * same - overlap, overlap


def find_anchor(frames: Sequence[LabeledFrame], ref: str, max_shift: int = 24,
                lengths=(16, 12, 8), coverage: float = 0.9) -> Optional[Tuple[int, str]]:
    """A bit pattern near the start of the reference that nearly every frame contains.
    Preambles and sync words are precisely the structure frames share, and anchoring on
    one survives sparse, zero-heavy data where plain correlation lines up by chance."""
    for length in lengths:
        best = None
        for pos in range(0, min(max_shift + 1, len(ref) - length + 1)):
            pat = ref[pos: pos + length]
            lo, hi = max(0, pos - max_shift), pos + max_shift + length
            share = sum(1 for f in frames if f.bits.find(pat, lo, hi) >= 0) / len(frames)
            if share >= coverage and (best is None or share > best[0]):
                best = (share, pos, pat)
        if best:
            return best[1], best[2]
    return None


def align(frames: Sequence[LabeledFrame], max_shift: int = 24) -> List[Tuple[LabeledFrame, int]]:
    """Offset of each frame against the longest one, so shared structure lines up:
    on a common sync pattern where one exists, by correlation otherwise."""
    if not frames:
        return []
    ref = max(frames, key=lambda f: len(f.bits)).bits
    anchor = find_anchor(frames, ref, max_shift)
    out = []
    for f in frames:
        if anchor:
            pos, pat = anchor
            j = f.bits.find(pat, max(0, pos - max_shift), pos + max_shift + len(pat))
            if j >= 0:
                out.append((f, pos - j))
                continue
        min_overlap = max(8, min(len(f.bits), len(ref)) // 2)
        best = None
        for s in range(-max_shift, max_shift + 1):
            score, overlap = _correlation(ref, f.bits, s)
            if overlap < min_overlap:
                continue
            key = (score, -abs(s))
            if best is None or key > best[0]:
                best = (key, s)
        out.append((f, best[1] if best else 0))
    return out


def trim_common_start(aligned: List[Tuple[LabeledFrame, int]]) -> List[LabeledFrame]:
    """Drop leading bits so every frame starts at the same aligned column."""
    if not aligned:
        return []
    start = max(s for _, s in aligned)
    return [LabeledFrame(f.label, f.bits[start - s:], f.source) for f, s in aligned]


def classify_columns(aligned: List[Tuple[LabeledFrame, int]]) -> Tuple[int, str]:
    start = min(s for _, s in aligned)
    end = max(s + len(f.bits) for f, s in aligned)
    marks = []
    for col in range(start, end):
        by_label: Dict[str, set] = {}
        present = 0
        for f, s in aligned:
            i = col - s
            if 0 <= i < len(f.bits):
                by_label.setdefault(f.label, set()).add(f.bits[i])
                present += 1
        values = set().union(*by_label.values()) if by_label else set()
        if present < 2:
            marks.append(" ")
        elif len(values) == 1:
            marks.append("=")
        elif all(len(v) == 1 for v in by_label.values()):
            marks.append("L")
        else:
            marks.append("*")
    return start, "".join(marks)


def render_table(aligned: List[Tuple[LabeledFrame, int]], group: int = 8) -> str:
    if not aligned:
        return "(no frames)"
    start, marks = classify_columns(aligned)
    width = len(marks)
    name_w = max(len(f.label) for f, _ in aligned) + 2

    def grouped(s: str) -> str:
        return " ".join(s[i:i + group] for i in range(0, len(s), group))

    ruler = "".join(str((c // group) % 10) if c % group == 0 else "." for c in range(width))
    lines = [" " * name_w + grouped(ruler)]
    for f, s in aligned:
        row = "".join(f.bits[c - s] if 0 <= c - s < len(f.bits) else " " for c in range(start, start + width))
        lines.append(f"{f.label:<{name_w}}" + grouped(row))
    lines.append(" " * name_w + grouped(marks))
    lines.append("legend: = fixed   L differs by label only   * varies within a label")
    return "\n".join(lines)


def _reverse8(b: int) -> int:
    return int(f"{b:08b}"[::-1], 2)


def crc8(data: bytes, poly: int, init: int = 0, reflect: bool = False, xorout: int = 0) -> int:
    crc = init
    for byte in data:
        crc ^= _reverse8(byte) if reflect else byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return (_reverse8(crc) if reflect else crc) ^ xorout


def _checks():
    yield "sum8", lambda d: sum(d) & 0xFF
    yield "neg-sum8", lambda d: (-sum(d)) & 0xFF
    yield "xor8", lambda d: reduce(lambda a, b: a ^ b, d, 0)
    for poly in (0x07, 0x31, 0x1D, 0x9B, 0x2F, 0xD5):
        for init in (0x00, 0xFF):
            for reflect in (False, True):
                for xorout in (0x00, 0xFF):
                    name = f"crc8(poly=0x{poly:02x},init=0x{init:02x},refl={int(reflect)},xorout=0x{xorout:02x})"
                    yield name, (lambda d, p=poly, i=init, r=reflect, x=xorout: crc8(d, p, i, r, x))


@dataclass
class ChecksumReport:
    matches: List[str] = field(default_factory=list)
    trials: int = 0
    distinct_frames: int = 0
    expected_false: float = 0.0  # optimistic: trials are not independent

    @property
    def confirmed(self) -> bool:
        return bool(self.matches) and self.distinct_frames >= MIN_DISTINCT and self.expected_false < 0.01

    def summary(self) -> str:
        if not self.matches:
            return f"no checksum found ({self.trials} combinations over {self.distinct_frames} distinct frames)"
        verdict = ("CONFIRMED" if self.confirmed else
                   f"UNCONFIRMED - need at least {MIN_DISTINCT} distinct frames; with this few, "
                   f"chance agreement is likely")
        return (f"{len(self.matches)} checksum match(es) over {self.distinct_frames} distinct frames, "
                f"{self.trials} combinations tried: {verdict}")


def checksum_search(bit_frames: Sequence[str]) -> ChecksumReport:
    """Test whether the last byte is a checksum over some suffix of the preceding bytes.
    Frames must share a common start - run trim_common_start() on aligned frames first."""
    rep = ChecksumReport(distinct_frames=len(set(bit_frames)))
    checks = list(_checks())
    for offset in range(8):
        for msb in (True, False):
            frames = []
            for bits in bit_frames:
                b = bits[offset:]
                chunks = [b[i:i + 8] for i in range(0, len(b) // 8 * 8, 8)]
                frames.append(bytes(int(c if msb else c[::-1], 2) for c in chunks))
            min_len = min((len(f) for f in frames), default=0)
            for start in range(0, min_len - 1):
                for name, fn in checks:
                    rep.trials += 1
                    if all(fn(f[start:-1]) == f[-1] for f in frames):
                        rep.matches.append(f"{name} over bytes[{start}:-1], bit offset {offset}, "
                                           f"{'MSB' if msb else 'LSB'} first")
    rep.expected_false = rep.trials * (1 / 256) ** max(rep.distinct_frames, 1)
    return rep
