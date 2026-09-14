"""bustap command line.

  python -m bustap monitor                      live levels, first check at the wall
  python -m bustap record --label ring_own --gated
  python -m bustap analyze captures/*.npz --plots plots/
  python -m bustap compare captures/*.npz
  python -m bustap simulate manchester --out sim.npz
  python -m bustap fake-esp sim.npz             serve a capture as if it were the ESP
"""

import argparse
import sys
from collections import Counter
from pathlib import Path

from . import protocol as proto


def _receiver(args):
    from .record import Receiver
    channels = [c.strip() for c in args.ch.split(",")] if args.ch else None
    return Receiver(args.esp, args.port, args.rate, channels)


def cmd_monitor(args):
    from .record import monitor
    with _receiver(args) as rx:
        try:
            monitor(rx, args.seconds)
        except KeyboardInterrupt:
            pass


def cmd_record(args):
    from .record import record_for, record_gated
    with _receiver(args) as rx:
        if args.gated:
            record_gated(rx, args.out_dir, args.label, args.pre, args.post, args.max, args.k, count=args.count)
        else:
            record_for(rx, args.seconds, args.out_dir, args.label)


def cmd_analyze(args):
    from .analyze import analyze, format_report
    from .capture import Capture
    for path in args.files:
        cap = Capture.load(path)
        reports = analyze(cap, k=args.k, adaptive=args.adaptive)
        print(format_report(cap, reports, str(path), top=args.top))
        print()
        if args.plots:
            from .plot import plot_event
            for r in reports:
                out = Path(args.plots) / f"{Path(path).stem}_ev{r.index:02d}.png"
                plot_event(cap, r, out)
                print(f"plot {out}")


def cmd_compare(args):
    from .analyze import analyze
    from .capture import Capture
    from .compare import LabeledFrame, align, checksum_search, render_table, trim_common_start
    frames, used = [], Counter()
    for path in args.files:
        cap = Capture.load(path)
        label = cap.label or Path(path).stem
        for r in analyze(cap, k=args.k):
            if r.source != "bus":
                continue
            dec = next((d for d in r.decodes if d.decoder == args.decoder), None) if args.decoder else r.best
            if dec is None or not dec.clean:
                continue
            used[dec.decoder] += 1
            frames += [LabeledFrame(label, f.bits, f"{Path(path).name}#{r.index}")
                       for f in dec.frames if len(f.bits) >= args.min_bits]
    if not frames:
        sys.exit("no clean frames found; try --decoder or check the captures with 'analyze'")
    if len(used) > 1 and not args.decoder:
        print(f"! frames came from different decoders {dict(used)}; pass --decoder to compare like with like\n")
    aligned = align(frames)
    print(render_table(aligned))
    print()
    print(checksum_search([f.bits for f in trim_common_start(aligned)]).summary())


def cmd_simulate(args):
    from .simulate import make_capture
    payload = args.payload.encode() if args.encoding == "dtmf" else bytes.fromhex(args.payload)
    cap = make_capture(args.encoding, payload, bit_rate=args.bit_rate, label=args.label or args.encoding,
                       dip_v=args.dip)
    print(f"saved {cap.save(args.out)}")


def cmd_fake_esp(args):
    from .capture import Capture
    from .fake_esp import FakeEsp
    esp = FakeEsp(Capture.load(args.capture), args.host, args.port, args.speed, args.drop_every)
    print(f"fake bus_tap on {esp.address[0]}:{esp.address[1]} - "
          f"python -m bustap monitor --esp {esp.address[0]} --port {esp.address[1]}")
    try:
        esp.serve()
    except KeyboardInterrupt:
        pass


def main(argv=None):
    p = argparse.ArgumentParser(prog="bustap", description="Urmet bus_tap capture and protocol analysis")
    sub = p.add_subparsers(dest="cmd", required=True)

    def net(sp):
        sp.add_argument("--esp", help="ESP address (default: discover by broadcast)")
        sp.add_argument("--port", type=int, default=proto.DEFAULT_PORT)
        sp.add_argument("--rate", type=int, help="total ADC rate to request, Hz")
        sp.add_argument("--ch", help="channels to request, e.g. DC,AC")

    s = sub.add_parser("monitor", help="live bus levels and link health")
    net(s)
    s.add_argument("--seconds", type=float)
    s.set_defaults(func=cmd_monitor)

    s = sub.add_parser("record", help="record captures")
    net(s)
    s.add_argument("--label", required=True, help="what is happening, e.g. ring_own, door_open")
    s.add_argument("--out-dir", default="captures")
    mode = s.add_mutually_exclusive_group()
    mode.add_argument("--seconds", type=float, default=10.0, help="fixed-length recording")
    mode.add_argument("--gated", action="store_true", help="save one capture per burst of activity")
    s.add_argument("--pre", type=float, default=1.0, help="gated: seconds kept before the trigger")
    s.add_argument("--post", type=float, default=1.5, help="gated: quiet seconds that end a capture")
    s.add_argument("--max", type=float, default=60.0, help="gated: longest single capture")
    s.add_argument("--count", type=int, help="gated: stop after this many captures")
    s.add_argument("-k", type=float, default=8.0, help="trigger threshold in noise sigmas")
    s.set_defaults(func=cmd_record)

    s = sub.add_parser("analyze", help="find events, infer the encoding, decode")
    s.add_argument("files", nargs="+")
    s.add_argument("--plots", help="directory for per-event PNGs")
    s.add_argument("-k", type=float, default=8.0)
    s.add_argument("--adaptive", action="store_true", help="adaptive slicing threshold (AC droop)")
    s.add_argument("--top", type=int, default=3, help="decoders to list per event")
    s.set_defaults(func=cmd_analyze)

    s = sub.add_parser("compare", help="align frames across labelled captures, find fields")
    s.add_argument("files", nargs="+")
    s.add_argument("--decoder", help="force one decoder, e.g. manchester, uart-8N1")
    s.add_argument("--min-bits", type=int, default=8)
    s.add_argument("-k", type=float, default=8.0)
    s.set_defaults(func=cmd_compare)

    s = sub.add_parser("simulate", help="write a synthetic capture")
    from .simulate import ENCODINGS
    s.add_argument("encoding", choices=ENCODINGS)
    s.add_argument("--payload", default="5a3c", help="hex bytes (DTMF: key string)")
    s.add_argument("--bit-rate", type=float, default=1000.0)
    s.add_argument("--dip", type=float, default=3.0, help="bus dip in volts for active symbols")
    s.add_argument("--label")
    s.add_argument("--out", default="sim.npz")
    s.set_defaults(func=cmd_simulate)

    s = sub.add_parser("fake-esp", help="serve a capture over the bus_tap UDP protocol")
    s.add_argument("capture")
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=proto.DEFAULT_PORT)
    s.add_argument("--speed", type=float, default=1.0)
    s.add_argument("--drop-every", type=int, default=0, help="simulate losing every Nth datagram")
    s.set_defaults(func=cmd_fake_esp)

    args = p.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
