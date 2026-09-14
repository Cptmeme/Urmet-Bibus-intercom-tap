import unittest

import numpy as np

from bustap.analyze import analyze
from bustap.compare import LabeledFrame, align, checksum_search, crc8, trim_common_start
from bustap.simulate import bits_of, bus_to_capture, make_capture

RANDOM = bytes(np.random.default_rng(7).integers(0, 256, 6).tolist())
ENDS_IN_ONE = bytes.fromhex("a55a0f")  # a trailing idle-level half/bit is the hard case


def bus_events(cap):
    return [r for r in analyze(cap) if r.source == "bus"]


def bitstr(payload: bytes) -> str:
    return "".join(map(str, bits_of(payload)))


def contains(decode, payload: bytes) -> bool:
    return decode is not None and any(bitstr(payload) in f.bits for f in decode.frames)


class BasebandDecoding(unittest.TestCase):
    # NRZ has no framing, so trailing bits at the idle level are genuinely invisible;
    # it is only expected to recover payloads that end away from idle.
    CASES = {
        "manchester": ("manchester", (b"\x5a\x3c", RANDOM, ENDS_IN_ONE)),
        "uart": ("uart-8N1", (b"\x5a\x3c", RANDOM, ENDS_IN_ONE)),
        "pwm": ("pulse-width", (b"\x5a\x3c", RANDOM, ENDS_IN_ONE)),
        "pdm": ("pulse-distance", (b"\x5a\x3c", RANDOM, ENDS_IN_ONE)),
        "nrz": ("nrz", (b"\x5a\x3c", RANDOM)),
    }

    def test_recovers_payload(self):
        for enc, (decoder, payloads) in self.CASES.items():
            for payload in payloads:
                with self.subTest(encoding=enc, payload=payload.hex()):
                    rep = bus_events(make_capture(enc, payload))[0]
                    self.assertEqual(rep.best.decoder, decoder)
                    self.assertTrue(contains(rep.best, payload), [f.bits for f in rep.best.frames])

    def test_faint_signal_is_read_from_ac_channel(self):
        rep = bus_events(make_capture("manchester", RANDOM, dip_v=0.25))[0]
        self.assertEqual(rep.channel, "AC")
        self.assertTrue(contains(rep.best, RANDOM))

    def test_constant_period_pwm_is_not_called_manchester(self):
        rep = bus_events(make_capture("pwm", RANDOM))[0]
        self.assertEqual(rep.model.hypotheses[0].encoding, "pwm-constant-period")


class HonestAmbiguity(unittest.TestCase):
    def test_biphase_mark_is_flagged_as_manchester_equivalent(self):
        best = bus_events(make_capture("bmc", RANDOM))[0].best
        self.assertTrue({"manchester", "bmc"} <= {best.decoder, *best.ambiguous_with})

    def test_short_uart_reading_flags_nrz(self):
        rep = bus_events(make_capture("fsk", b"\x5a\x3c"))[0]
        if rep.best.family == "uart":
            self.assertIn("nrz", rep.best.ambiguous_with)
        candidates = [d for d in rep.decodes if d.decoder in (rep.best.decoder, *rep.best.ambiguous_with)]
        self.assertTrue(any(contains(d, b"\x5a\x3c") for d in candidates))


class Tones(unittest.TestCase):
    def test_fsk_tones_are_identified(self):
        rep = bus_events(make_capture("fsk", b"\x5a\x3c"))[0]
        self.assertTrue(rep.tonal)
        f0, f1 = rep.fsk_tones
        self.assertAlmostEqual(f0, 1200, delta=120)
        self.assertAlmostEqual(f1, 2200, delta=220)

    def test_dtmf(self):
        keys = "".join(k for r in analyze(make_capture("dtmf", b"1#9")) for _, k in r.dtmf)
        self.assertEqual(keys, "1#91#9")

    def test_baseband_is_not_mistaken_for_tones(self):
        for enc in ("manchester", "uart", "pwm", "pdm", "nrz"):
            with self.subTest(encoding=enc):
                self.assertFalse(bus_events(make_capture(enc, RANDOM))[0].tonal)


class EventDetection(unittest.TestCase):
    def test_quantisation_flicker_is_not_activity(self):
        fs_os = 480000.0
        cap = bus_to_capture(np.full(int(2 * fs_os), 22.5), fs_os, 60000, adc_noise_lsb=0.3)
        self.assertEqual(analyze(cap), [])

    def test_idle_floor_call_loop_raises_no_events(self):
        self.assertEqual([r for r in analyze(make_capture("manchester", RANDOM)) if r.source != "bus"], [])


def _frame(addr, cmd, seq):
    body = bytes([0xA5, addr, cmd, seq])
    return body + bytes([crc8(body, 0x31, 0xFF)])


def _bits(b):
    return "".join(f"{x:08b}" for x in b)


class Compare(unittest.TestCase):
    def setUp(self):
        self.frames = []
        for label, addr, cmd in (("ring_own", 0x12, 0x01), ("ring_neigh", 0x13, 0x01), ("door_open", 0x12, 0x07)):
            self.frames += [LabeledFrame(label, _bits(_frame(addr, cmd, 0x40))) for _ in range(2)]
        self.frames += [LabeledFrame("counter", _bits(_frame(0x20, 0x02, s))) for s in range(3)]

    def test_alignment_survives_junk_prefix_and_sparse_frames(self):
        shifted = [LabeledFrame(f.label, ("01" + f.bits) if f.label == "ring_neigh" else f.bits)
                   for f in self.frames]
        trimmed = trim_common_start(align(shifted))
        self.assertTrue(all(f.bits.startswith("10100101") for f in trimmed))
        self.assertEqual({len(f.bits) for f in trimmed}, {40})

    def test_planted_crc_is_confirmed(self):
        rep = checksum_search([f.bits for f in self.frames])
        self.assertTrue(rep.confirmed, rep.summary())
        self.assertTrue(any("poly=0x31,init=0xff,refl=0,xorout=0x00" in m and "bytes[0:-1]" in m
                            for m in rep.matches))

    def test_too_few_distinct_frames_is_never_confirmed(self):
        rep = checksum_search([self.frames[0].bits, self.frames[2].bits])
        self.assertFalse(rep.confirmed)


if __name__ == "__main__":
    unittest.main()
