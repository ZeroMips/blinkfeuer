# Blinkfeuer

[![CI](https://github.com/ZeroMips/blinkfeuer/actions/workflows/ci.yml/badge.svg)](https://github.com/ZeroMips/blinkfeuer/actions/workflows/ci.yml)

A logic analyzer application for Linux, written in Rust. It looks and works like a bench instrument
(black plot, dotted graticule, markers with `M1 to M2` readouts, a Bus/Signal tree) rather than like a
phone app, and it is built for long captures: the capture store only keeps the samples where something
changes, so quiet signals cost almost no memory.

*Blinkfeuer* is the old German word for a flashing beacon, the light that tells you something is out there.

![Blinkfeuer showing a triggered capture of the simulator with UART, SPI and I2C decoded, the decoder setup and the decode table](docs/screenshot.png)

*A triggered capture of the built-in simulator: UART characters grouped into a telegram, SPI words, I2C
transactions, two markers, and the decode table below. Times are counted from the trigger (the orange **T**).*

## Highlights

- **Live view**: zoom at the pointer, pan, follow mode, an overview strip, scale in time per division.
- **Protocol decoders** for UART, SPI and I2C, plus **Modbus RTU** on top of UART. They cost time in
  proportion to the number of edges, not samples, and a setup screen shows example traces for the chosen
  settings. UART baud rate and frame format can be detected from the capture.
- **Triggers on the protocol**: wait for the text `sim 0`, the I2C sequence `S @50W 01 XX P`, or a Modbus
  frame with a bad CRC, with separate pre-trigger and post-trigger. See [triggers](docs/triggers.md).
- **Markers and measurements**: M1/M2 snap to edges, delta and 1/dt, and per-channel frequency, duty cycle
  and pulse widths over the window, between the markers or over the whole capture.
- **Bus/Signal setup**: names, trace colors, buses with hex/decimal/binary values, collapsible groups, drag
  and drop reordering.
- **Decode table** linked to the waveform, with search.
- **Files**: compact checksummed captures (`.bfc`), saved setups, VCD and CSV export, and import of sigrok
  sessions (`.sr` from PulseView or sigrok-cli).
- **Honest about lost data**: when the analyzer cannot keep up, the capture reports an error instead of
  silently dropping samples (the FX2 firmware cannot tell, see below).

## Hardware

It is made for the [Sipeed SLogic16U3](https://wiki.sipeed.com/hardware/en/logic_analyzer/slogic16u3/slogic16u3.html)
(16 channels, USB 3), and has a simulator so you can try everything without hardware.

| Analyzer | Status |
|---|---|
| Simulator | Everything works and is covered by tests. |
| SLogic16U3, stock firmware | Runs at full rate with exact sample counts on the analyzer's test patterns. Not yet tried with real probe signals. |
| SLogic16U3, open LiteX gateware | Runs on the real device with test patterns; 16 channels at 160 MHz is not reliable yet, and the highest rates need a host that is not busy. |
| FX2 analyzers ("Saleae Logic" clones) | 8 channels up to 24 MHz with sigrok's fx2lafw firmware, used with real signals. |

Details, known problems and how each driver works: [Devices and drivers](docs/devices.md).
Linux only.

## Quick start

You need Rust 1.88 or newer.

```sh
cargo run --release -p bf-app --bin blinkfeuer -- --demo    # a capture of the simulator right away
cargo run --release -p bf-app --bin blinkfeuer              # the application
cargo run --release -p bf-app --bin bfcli -- --help         # command line capture and checks
```

F5 runs a capture, F8 or Shift+F5 stops it. Desktop entry, USB access without root and the WSL notes are in
[Installing and running](docs/install.md).

## Documentation

- [Devices and drivers](docs/devices.md): backends, verified status, known problems, protocol sources.
- [Installing and running](docs/install.md): build, desktop entry, udev rule, WSL.
- [Triggers and patterns](docs/triggers.md): the trigger and search pattern language.

## Layout of the code

| Crate | What it holds |
|---|---|
| `bf-core` | Sample formats, the capture store (sparse change events plus per-block summaries), trigger conditions |
| `bf-device` | The device interface, the simulator, the stock-firmware, LiteX-gateware and fx2lafw backends, the capture session with the trigger logic |
| `bf-decode` | UART, SPI and I2C decoders, Modbus RTU, baud rate and frame format detection, the decoding engine, the pattern language |
| `bf-io` | Capture file format and VCD export |
| `bf-app` | The application (`blinkfeuer`) and the command line tool (`bfcli`) |

`cargo test --workspace` runs the tests; `docs/screenshot.sh` makes the screenshot again.

## Licence

Dual licensed under [MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE), at your option. The protocol
implementations were written from public descriptions without taking code from GPL-licensed drivers, see
[Devices and drivers](docs/devices.md).
