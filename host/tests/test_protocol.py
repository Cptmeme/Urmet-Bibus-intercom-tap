import shutil
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np

from bustap import protocol as p

HEADER_DIR = Path(__file__).resolve().parents[2] / "firmware" / "bus_tap_capture" / "main"


class RoundTrip(unittest.TestCase):
    def test_samples(self):
        words = p.encode_words(np.array([0, 1, 2, 0]), np.array([1, 4095, 2048, 7]))
        pkt = p.parse(p.build_samples(5, 2, 1000, 123456, 60000, 3, words, overflow_count=9))
        h = pkt.header
        self.assertEqual((h.type, h.seq, h.epoch, h.first_index, h.esp_time_us, h.rate_hz, h.n_channels,
                          h.overflow_count, h.n_samples), (p.TYPE_SAMPLES, 5, 2, 1000, 123456, 60000, 3, 9, 4))
        ch, raw = p.split_words(pkt.words)
        self.assertEqual(ch.tolist(), [0, 1, 2, 0])
        self.assertEqual(raw.tolist(), [1, 4095, 2048, 7])

    def test_info(self):
        pkt = p.parse(p.build_info(1, 1, 0, 60000, 3, {"cal": {"raw": [0, 4095], "mv": [0, 3100]}}))
        self.assertEqual(pkt.info["cal"]["mv"][-1], 3100)

    def test_rejects_malformed(self):
        good = p.build_samples(0, 1, 0, 0, 60000, 1, np.zeros(4, dtype=np.uint16))
        for bad in (good[:10], b"XXXX" + good[4:], good[:-2]):
            with self.assertRaises(p.ProtocolError):
                p.parse(bad)

    def test_hello(self):
        self.assertEqual(p.make_hello(), b"UBTHELLO")
        self.assertEqual(p.make_hello(60000, ["dc", "ac"]), b"UBTHELLO rate=60000 ch=DC,AC")


@unittest.skipUnless(shutil.which("cc"), "no C compiler available")
class MatchesFirmwareHeader(unittest.TestCase):
    """protocol.py must describe exactly the struct the firmware sends."""

    def test_layout_and_constants(self):
        fields = "".join(f'printf("off_{n} %zu\\n", offsetof(bt_hdr_t, {n}));' for n in p.HDR_FIELDS)
        src = ("#include <stddef.h>\n#include <stdio.h>\n#include \"protocol.h\"\nint main(void){"
               'printf("sizeof %zu\\n", sizeof(bt_hdr_t));' + fields +
               'printf("magic %u\\nhdr %d\\nmaxs %d\\nshift %d\\nmask %d\\nhello %s\\n",'
               "BT_MAGIC, BT_HDR_SIZE, BT_MAX_SAMPLES_PER_PKT, BT_CH_SHIFT, BT_DATA_MASK, BT_HELLO);return 0;}")
        with tempfile.TemporaryDirectory() as d:
            c, exe = Path(d) / "check.c", Path(d) / "check"
            c.write_text(src)
            subprocess.run(["cc", "-std=c11", "-I", str(HEADER_DIR), "-o", str(exe), str(c)], check=True)
            out = dict(line.split(" ", 1) for line in
                       subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.splitlines())
        offset = 0
        for name, code in zip(p.HDR_FIELDS, p.HDR.format.lstrip("<")):
            self.assertEqual(int(out[f"off_{name}"]), offset, name)
            offset += struct.calcsize("<" + code)
        self.assertEqual(int(out["sizeof"]), p.HDR_SIZE)
        self.assertEqual(int(out["hdr"]), p.HDR_SIZE)
        self.assertEqual(int(out["magic"]), p.MAGIC)
        self.assertEqual(int(out["maxs"]), p.MAX_SAMPLES_PER_PKT)
        self.assertEqual(int(out["shift"]), p.CH_SHIFT)
        self.assertEqual(int(out["mask"]), p.DATA_MASK)
        self.assertEqual(out["hello"], p.HELLO.decode())
        self.assertLessEqual(p.HDR_SIZE + 2 * p.MAX_SAMPLES_PER_PKT + 28, 1500, "datagram exceeds the MTU")


if __name__ == "__main__":
    unittest.main()
