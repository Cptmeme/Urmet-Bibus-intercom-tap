# Urmet Bibus VOP — L1/L2 + C1/C2 analog tap

Front end that lets an ESP32 act as the oscilloscope we don't have, so we can
measure the modulation depth and bit timing on the Urmet door-phone bus.

Target hardware: **Urmet Bibus 2nd Ed. VOP**, Artico monitor Ref. 1705/1 on
bracket Ref. 1705/954. Manual in `../docs/`.

## Wiring

### Intercom side
| Board | Intercom terminal | Note |
|---|---|---|
| J1-1 | **L1** | bus positive (22.5 V idle as measured) |
| J1-2 | **L2** | becomes board GND |
| J2-1 | **C1** | floor call |
| J2-2 | **C2** | floor call |

Leave **VPI / VPU alone.** They carry the monitor's operating power
(700 mA, 16–18.5 Vdc), not just video.

### ESP32 side — J3
| J3 | ESP32 (classic DevKit) | Signal |
|---|---|---|
| 1 | 3V3 | supply for U1 |
| 2 | GND | **= bus L2** |
| 3 | GPIO34 | `ADC_DC`  — channel A, absolute bus voltage |
| 4 | GPIO35 | `ADC_AC`  — channel B, AC-coupled ×11 |
| 5 | GPIO36 (VP) | `ADC_FC` — channel C, floor call |

**Use ADC1 pins only.** ADC2 is unusable while Wi-Fi is on, and we need Wi-Fi
(see safety note below). GPIO34/35/36 are input-only, which suits analog.

## Channels

- **A — absolute.** R1/R2 divide by 11, U1A buffers, R3/C1 anti-alias at ~16 kHz.
  Window 0–34 V, so 22.5 V idle sits at 2.05 V. Shows big dips and the DC level.
- **B — AC-coupled, gain 11.** C2 blocks DC, R4/R5 re-bias to 1.65 V, U1B gains
  ×11. Net effect: **1 V of bus swing = 1 V at the ADC**, centred on 1.65 V,
  ±1.5 V before clipping, 3.2 Hz high-pass. This is the channel that will show
  small modulation and edges that channel A buries in quantisation noise.
- **C — floor call.** Same divider + buffer on C1/C2.

MCP6004 is CMOS (~1 pA bias), so the 1 MΩ divider costs ~1 µV of offset. Do not
substitute a bipolar op-amp such as LM324 — it is neither rail-to-rail nor
happy at 3.3 V, and its bias current would swamp the divider.

## Connector notes

There is **no separate external GND**. Board GND *is* bus L2 - it arrives on J1-2 and
leaves on J3-2 to the ESP32. Do not add a dedicated ground pin; the one thing this
board must not have is two references.

Mis-plugging is harmless: both bus channels use identical 1M/100k front ends, so
swapping the L1/L2 and C1/C2 headers does nothing bad, and reversing L1/L2 polarity
merely pins channel A at zero (D1 clamps at -0.3 V, the 1 M holds current to ~22 uA).

The one real risk is a wire that works loose and *dangles* behind the intercom, where
it could bridge L1 to L2. A disconnected tap is harmless - it is passive and
high-impedance, you just lose data - but a flapping conductor on a shared bus is not.
Use short leads with heat-shrink strain relief, or fit keyed/latching connectors
(JST-XH is 2.54 mm pitch, polarised, and JLC-stocked) if you want it fit-and-forget.

## Safety

1. **This tap is NOT isolated** — board GND is bus L2. Power the ESP32 from an
   isolated USB wall wart and get data off over **Wi-Fi/UDP**. Never plug the
   laptop's USB in while J1 is connected to the bus; that would tie the
   building's bus rail to your laptop's ground.
2. **Bus loading:** 22.5 V / 1.1 MΩ ≈ **20 µA**. The manual specs the whole
   monitor's standby draw on L1/L2 as **1.6 mA max**, so this tap is ~1.3% of
   it. This is why the PC817 opto board must not go here — at 3.3 kΩ it would
   draw 5.9 mA, ~3.7× the entire monitor.
3. **JP1** links C2 to board GND. Fit it **only** after confirming with a
   multimeter that the C1/C2 loop shares a reference with L2. If that loop
   floats, leave JP1 open and sense C1/C2 with the separate opto board — C1/C2
   is local to this monitor only, so nothing there can affect neighbours.

## JLCPCB part status

Checked against JLCPCB's Economic Parts list (Basic + Preferred Extended = $0 feeder
fee). Everything below is in that list **except** the three noted.

| Part | LCSC | Library | Note |
|---|---|---|---|
| R 1k 0402 | C11702 | Basic | |
| R 10k 0402 | C25744 | Basic | |
| R 100k 0402 | C25741 | Basic | |
| R 1M 0805 | C17514 | Basic | **150 V rated** |
| C 10n 0402 | C15195 | Basic | |
| C 100n 0402 | C1525 | Basic | |
| C 1u 0402 | C52923 | Basic | |
| C 10u 0402 | C15525 | Basic | 6.3 V X5R |
| BAT54S SOT-23 | C7420333 | Preferred Ext. | $0 fee, 1.2 M in stock |
| MCP6004T-I/SL | C7378 | **Extended** | feeder fee |
| 2.54 mm 1x02 header | - | **Extended** | no connectors are economic parts |
| 2.54 mm 1x05 header | - | **Extended** | " |

**3 unique Extended parts -> roughly $9 in one-off feeder fees.**

Why R1/R9 stay 0805: the 0402 1M (C26083) is rated **50 V**, the 0805 1M (C17514)
is **150 V**. R1/R9 take 10/11 of anything appearing on L1/L2, and the door strike
is inductive. This is the one place the extra millimetre is worth it.

### Cutting the feeder fees to zero

1. **Hand-solder the four headers.** They are the only through-hole parts and the
   only reason to pay for THT assembly at all. Removes 2 of the 3 Extended parts.
2. **Swap the quad for 2x MCP6002 (C7377, Preferred Extended, $0).** Two SOIC-8
   instead of one SOIC-14. Only 3 of the 4 amplifiers are used, so two duals give
   the same single spare. Costs a schematic edit and one more decoupling cap.

### Do NOT substitute LM324

`LM324DT` (C71035) is a Basic part, SOIC-14, and pin-compatible with the MCP6004.
It will not work here. Its input common-mode range is Vee to Vcc-1.5 V, i.e. 0 to
**1.8 V** on a 3.3 V rail - but channel A's input sits at **2.05 V**, outside that
range entirely. Its 45 nA input bias across the 90.9 kOhm source would also add
several mV of offset where the MCP6004's ~1 pA adds a microvolt. Same footprint,
free, and completely wrong.

## Files

| File | Purpose |
|---|---|
| `bom.csv` | full engineering BOM (refs, values, footprints, LCSC, JLC library) |
| `bom_jlcpcb.csv` | **upload this to JLCPCB** - Comment / Designator / Footprint / LCSC Part #, SMD only |
| `bus_tap.pdf` | schematic print |
| `erc.rpt` | ERC report |

`bom_jlcpcb.csv` covers 21 assembled parts over 10 lines. J1, J2, J3 and JP1 are
deliberately excluded - they are the only through-hole parts, they are the only
Extended parts left once the MCP6004 fee is paid, and hand-fitting four headers takes
minutes. Designators are written as explicit lists (`R2,R4,R5,R6,R10`), not KiCad's
default ranges (`R4-R6`), which JLC's BOM parser does not understand.

Each symbol carries `LCSC` and `JLC` fields, so both BOMs regenerate straight from
the schematic.

## JLCPCB upload files (`jlc/`)

| File | Upload as |
|---|---|
| `bus_tap-gerbers.zip` | PCB (11 fab layers, gerber drill included) |
| `bom_jlc.csv` | BOM - `Comment / Designator / Footprint / LCSC Part #` |
| `cpl_jlc.csv` | CPL - `Designator / Mid X / Mid Y / Layer / Rotation` |

J1, J2, J3 and JP1 are excluded from both CSVs (hand-soldered). BOM and CPL
designators cross-check: 21 each, no orphans.

### CPL rotation corrections - ALREADY APPLIED to cpl_jlc.csv

JLC's library orientation differs from KiCad's for two packages here. Determined
empirically from JLC's upload preview, and **baked into `cpl_jlc.csv`**:

| Part | Package | KiCad rot | Correction | CPL value |
|---|---|---|---|---|
| D1 | SOT-23 | 0 | +180 | **180** |
| D2 | SOT-23 | 180 | +180 | **0** |
| U1 | SOIC-14 | 0 | -90 (90 CW) | **270** |

Rotation in the CPL is **counter-clockwise-positive**, matching KiCad's own pos file
(which emits -90 for C6/C7 and +90 for R10). KiCad places SOIC-14 with the long axis
vertical and pin 1 top-left; JLC's library has it horizontal, hence the 90 deg.

The 18 chip R/C parts need no correction - they are two-terminal and non-polarised,
so 0 vs 180 is electrically identical.

**Verification on next upload: the preview should need NO manual rotation.** If any of
these three still looks wrong, the correction above is stale - re-derive it rather than
nudging it in the preview.

## Status

- Schematic: complete, hand-tidied in eeschema, **ERC clean (0 violations)**, netlist verified unchanged.
- Footprints: **assigned on all 25 parts**, 9 distinct footprints, all verified present in the KiCad libraries.
- PCB: not started. Target is JLCPCB Economic PCBA. Run **Tools > Update PCB from Schematic (F8)** in the PCB editor to seed it.
