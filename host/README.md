# bustap: record and decode the Urmet bus

Protocol-discovery tools for the bus_tap board. They don't assume an encoding: they
find activity, work out how bits are encoded from pulse timing, decode with every
plausible scheme, rank the results, and line up frames from different events so the
address, command and checksum fields stand out.

## Setup

```sh
cd host
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python -m unittest discover -s tests      # a few seconds, no hardware needed
```

Run commands from `host/` as `.venv/bin/python -m bustap ...`.

## At the intercom

1. Flash the firmware (see `firmware/bus_tap_capture/README.md`). Power the ESP32 from an
   **isolated** USB supply, wire J3, then J1 (L1/L2) and J2 (C1/C2).
2. **`bustap monitor`**: the first sanity check. Idle DC should read about **22.5 V**; ~0 V
   usually means L1/L2 are swapped, which is harmless. Watch `lost`, `gaps` and `ovf`
   stay at 0.
3. **Record labelled events**, one label per kind of event:
   ```sh
   bustap record --gated --label ring_own        # ring your own bell at the street panel
   bustap record --gated --label door_open       # press the door-release button
   bustap record --gated --label handset_up
   bustap record --gated --label floor_call      # your own landing button (C1/C2)
   ```
   Each burst of activity becomes its own file in `captures/`, with a second of history
   before the trigger. Calls to other flats also travel over L1/L2; capture a few, since
   the difference between your address and another is what exposes the address field.
4. **`bustap analyze captures/*.npz --plots plots/`**, per event: which channel was used,
   the pulse and gap clusters as multiples of one time unit, ranked encoding
   hypotheses, and the best decodes with their error counts. Look at the plots too.
5. **`bustap compare captures/*.npz`**, per bit position:
   `=` is identical everywhere (preamble/sync), `L` changes only between labels
   (address/command), `*` varies within a label (counter, checksum). It then searches
   for a checksum (sum, xor, CRC-8 variants) over the aligned frames.

## Reading the output honestly

- **`~ equally clean: ...`** means timing cannot choose between readings. Manchester and
  biphase-mark are *provably* interchangeable on the wire; a UART reading of fewer than
  four bytes can frame by chance. Run `compare` on both with `--decoder`, and keep the one
  that gives frames a stable preamble.
- **`CONFIRMED` checksums need at least 4 distinct frames.** With fewer, chance
  agreement across ~2500 combinations is likely and the result is marked `UNCONFIRMED`.
- **Frame edges.** A symbol at the bus's idle level at the start or end of a frame merges
  into the idle beside it. Manchester, UART, pulse-width and pulse-distance recover it.
  Biphase-mark can lose a final 0, and plain NRZ loses trailing idle-level bits: nothing
  on the wire marks where those frames end. A frame that is one bit value repeated
  (like `0x00` or `0xFF`) is ambiguous by half a bit in Manchester. Real frames with a
  preamble are not.
- **AC clipping.** Large dips saturate the AC channel. The report says so; timing is still
  good, and the DC channel carries the true levels.

## Without the intercom

```sh
bustap simulate manchester --payload a5120140 --label ring_own --out sim.npz
bustap analyze sim.npz
bustap fake-esp sim.npz --port 17777 &          # serves it over the real UDP protocol
bustap monitor --esp 127.0.0.1 --port 17777
```

`simulate` supports manchester, bmc, uart, pwm, pdm, nrz, fsk and dtmf, through a model
of the bus_tap channels and ESP32 ADC.

## Layout

| Module | Role |
|---|---|
| `protocol.py` | UDP wire format (mirrors `firmware/.../protocol.h`) |
| `record.py` | receiver, live monitor, fixed and activity-gated recording |
| `capture.py` | `.npz` capture files, raw codes to bus volts |
| `dsp.py` | noise, event detection, slicing, sub-sample edge timing |
| `infer.py` | duration clusters, time unit, encoding hypotheses |
| `decoders.py` | NRZ, UART, Manchester, biphase-mark, pulse-width, pulse-distance |
| `tones.py` | tone detection, FSK discriminator, DTMF |
| `compare.py` | frame alignment, field map, checksum search |
| `simulate.py`, `fake_esp.py` | synthetic traffic and a fake ESP for testing |
