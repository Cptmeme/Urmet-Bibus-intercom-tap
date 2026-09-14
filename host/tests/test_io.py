"""Recorder against a protocol-identical fake ESP on localhost."""

import tempfile
import threading
import unittest

from bustap.analyze import analyze
from bustap.capture import Capture
from bustap.fake_esp import FakeEsp
from bustap.record import Receiver, record_for, record_gated
from bustap.simulate import make_capture

QUIET = lambda msg: None  # noqa: E731


def serve(esp: FakeEsp, seconds: float) -> None:
    threading.Thread(target=esp.serve, kwargs={"seconds": seconds}, daemon=True).start()


def decoded_hex(cap: Capture):
    return [f.to_bytes().hex() for r in analyze(cap) if r.source == "bus" and r.best for f in r.best.frames]


class Recording(unittest.TestCase):
    def test_clean_link(self):
        esp = FakeEsp(make_capture("uart", b"\x12\x34\x56\x78", lead_s=0.5), speed=4.0)
        serve(esp, 15)
        with tempfile.TemporaryDirectory() as d, Receiver("127.0.0.1", esp.address[1]) as rx:
            path = record_for(rx, 1.5, d, "clean", log=QUIET)
            self.assertEqual((rx.stats.sample_gaps, rx.stats.datagrams_lost, rx.stats.overflows), (0, 0, 0))
            self.assertAlmostEqual(rx.measured_rate(), esp.fs_total, delta=1.0)
            cap = Capture.load(path)
            self.assertEqual(set(cap.channels), {"DC", "AC", "FC"})
            self.assertIn("12345678", decoded_hex(cap))

    def test_lossy_link_marks_gaps_and_never_measures_across_them(self):
        esp = FakeEsp(make_capture("uart", b"\x12\x34\x56\x78", lead_s=0.5), speed=4.0, drop_every=25)
        serve(esp, 15)
        with tempfile.TemporaryDirectory() as d, Receiver("127.0.0.1", esp.address[1]) as rx:
            path = record_for(rx, 1.5, d, "lossy", log=QUIET)
            # Receiver stats span its whole life, including drops before recording began.
            self.assertEqual(rx.stats.sample_gaps, rx.stats.datagrams_lost)
            cap = Capture.load(path)
            counts = {n: len(c.gaps) for n, c in cap.channels.items()}
            self.assertEqual(len(set(counts.values())), 1, counts)
            self.assertGreater(counts["DC"], 0)
            self.assertLessEqual(counts["DC"], rx.stats.sample_gaps)
            self.assertEqual(cap.meta["capture_gaps"], counts)
            gap_t = {g / c.fs for c in cap.channels.values() for g in c.gaps}
            for r in analyze(cap):
                self.assertFalse(any(r.t_start_s + 1e-4 < t < r.t_end_s - 1e-4 for t in gap_t))

    def test_gated_recording_captures_a_burst(self):
        esp = FakeEsp(make_capture("uart", b"\x12\x34\x56", lead_s=3.0, repeats=1), speed=4.0)
        serve(esp, 30)
        with tempfile.TemporaryDirectory() as d, Receiver("127.0.0.1", esp.address[1]) as rx:
            paths = record_gated(rx, d, "gated", pre_s=0.3, post_s=0.6, warmup_s=1.0, count=1, log=QUIET)
            self.assertEqual(len(paths), 1)
            self.assertIn("123456", decoded_hex(Capture.load(paths[0])))


if __name__ == "__main__":
    unittest.main()
