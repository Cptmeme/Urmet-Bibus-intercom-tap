# telefoon_input: Urmet Bibus VOP intercom tap

Reverse-engineering the Urmet intercom (Artico monitor 1705/1 on bracket 1705/954,
Bibus 2nd Ed. VOP) so it can be bridged into Home Assistant.

| Path | What |
|---|---|
| `bus_tap/` | KiCad project for the analog tap board (L1/L2 + C1/C2 to ESP32 ADC), JLCPCB files in `bus_tap/jlc/` |
| `firmware/bus_tap_capture/` | ESP32 firmware streaming the tap's ADC channels over Wi-Fi/UDP |
| `host/` | `bustap`: record, analyse, decode and compare bus traffic |
| `docs/` | Urmet Bibus 2nd Ed. VOP technical manual, section 4B |
| `telefoon_input.kicad_sch` | reverse-engineered schematic of the AliExpress PC817 opto board |

Order of work: build the tap board, flash the firmware, run `bustap monitor` at the wall,
then record and decode. See `host/README.md`.

**Safety.** bus_tap GND is bus L2 and the tap is not isolated. While it is connected to
the intercom, the ESP32 runs from an isolated USB supply and talks over Wi-Fi only.
