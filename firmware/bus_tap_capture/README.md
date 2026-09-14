# bus_tap_capture

ESP32 firmware for the bus_tap board. Samples the three analog channels continuously
and streams them over Wi-Fi/UDP to the host tools in `host/`.

> **Safety.** bus_tap GND *is* Urmet bus L2. Flash and configure the ESP32 **before** it
> goes near the intercom. Once J1 is connected, power the ESP32 from an **isolated USB
> supply only** and never plug it into a computer. That is why data leaves over Wi-Fi.

## Build and flash (ESP-IDF 5.3)

```sh
cd firmware/bus_tap_capture
export IDF_PYTHON_ENV_PATH=$HOME/.espressif/python_env/idf5.3_py3.13_env
. $HOME/esp/esp-idf/export.sh
idf.py set-target esp32
idf.py menuconfig          # "Bus tap capture": Wi-Fi SSID and password
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

`IDF_PYTHON_ENV_PATH` is needed on this machine: without it `export.sh` picks Homebrew's
Python 3.14 and looks for a py3.14 IDF environment that does not exist.

The Wi-Fi password is stored in plain text in `sdkconfig`; don't share that file.

## Wiring: ESP32 DevKit to bus_tap J3

| J3 | ESP32 | Signal |
|---|---|---|
| 1 | 3V3 | op-amp supply |
| 2 | GND | = bus L2 |
| 3 | GPIO34 | DC, absolute bus voltage |
| 4 | GPIO35 | AC, bus swing |
| 5 | GPIO36 (VP) | FC, floor call |

ADC1 pins only; ADC2 does not work while Wi-Fi is on. Pins are configurable in menuconfig.

## Behaviour

- **Rate.** 60 kS/s total by default, shared by the enabled channels (20 kS/s each with
  all three). The host can change it at runtime, e.g.
  `python -m bustap monitor --ch AC --rate 60000` puts all 60 kS/s on the AC channel.
  Worth doing for fine timing: the board's anti-alias filter sits at ~16 kHz, above the
  10 kHz Nyquist limit you get with three channels sharing 60 kS/s.
- **Streaming** runs only while a host keeps sending `UBTHELLO` (5 s lease), to the host
  that sent the most recent one.
- **INFO** once a second: configuration, the eFuse calibration table, DMA overflow and
  Wi-Fi TX-drop counters, RSSI.
- **Protocol** is defined in `main/protocol.h`; `host/tests/test_protocol.py` compiles it
  and checks `host/bustap/protocol.py` against it.

## What has and hasn't been verified

Verified: builds for ESP32 on IDF 5.3.2 with no warnings from `main/`; the protocol
layout matches the host byte for byte; the host recorder, gap accounting and gated
recording pass against a protocol-identical fake on localhost.

**Not yet verified on hardware:** sustained 60 kS/s over your actual Wi-Fi, the real
ADC sample rate (the host measures it against `esp_timer`, so a few percent of error
is corrected rather than silently scaling pulse widths), and ADC behaviour at the
extremes. The ESP32 ADC compresses above about 2.45 V at 12 dB attenuation, so AC
channel swings beyond roughly +0.8 V read low. Timing is unaffected.
