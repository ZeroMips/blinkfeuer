# Blinkfeuer

[![CI](https://github.com/ZeroMips/blinkfeuer/actions/workflows/ci.yml/badge.svg)](https://github.com/ZeroMips/blinkfeuer/actions/workflows/ci.yml)

A logic analyzer application for Linux, written in Rust. It looks and works like a bench instrument
(black plot, dotted graticule, markers with `M1 to M2` readouts, a Bus/Signal tree) rather than like a
phone app, and it is built for long captures: the capture store only keeps the samples where something
changes, so quiet signals cost almost no memory.

*Blinkfeuer* is the old German word for a flashing beacon, the light that tells you something is out there.

![Blinkfeuer showing a triggered capture of the simulator: UART, SPI and I2C decoded, with markers and the table of decoded items](docs/screenshot.png)

The screenshot shows a capture of the built-in simulator, triggered on the UART text "sim 0" (the orange **T**), with
the UART characters grouped into a telegram, SPI words, I2C transactions, two markers and the table of decoded items.
Times are counted from the trigger.

It is made for the [Sipeed SLogic16U3](https://wiki.sipeed.com/hardware/en/logic_analyzer/slogic16u3/slogic16u3.html)
(16 channels, USB 3), and has a simulator so you can try everything without hardware.

## Status

A working first version. What is verified and what is not:

- **Simulator**: everything below works on it, and most of it is covered by tests.
- **SLogic16U3 with the stock firmware**: acquisition has been run on hardware at full rate with the
  analyzer's own test patterns, with exact sample counts. **Not yet tried with real probe signals.**
- **LiteX gateware backend** (the open gateware for the same hardware, `bfcli --device litex`, selectable in the
  application when the analyzer runs it): written from the gateware's own library and tested against a fake
  register file, but **not yet run on the real device**. Everything except the USB calls is covered by tests.
- Linux only.

## Features

- **Live view** with zoom, pan, follow mode and an overview strip. Scale is given in time per division,
  with times shown in groups ("10 ms 000 µs 400 ns").
- **Markers** M1 and M2 that snap to edges, with their delta and 1/dt; per-row values at the markers.
- **Bus/Signal setup**: names, 16 trace colors, buses with hex/decimal/binary values, collapsible groups,
  drag and drop reordering, hiding rows. Decoders can sit inside groups.
- **Protocol decoders** for UART, SPI and I2C, written to be fast (cost proportional to the number of
  edges, not samples), with a setup screen that shows example traces for the chosen settings. UART
  characters are also grouped into readable telegrams.
- **Annotation table** of decoded items with search; clicking in the table moves the waveform and vice versa.
- **Triggers** (done on the host, the analyzer has none in hardware): level and edge conditions on any
  channels, or a protocol trigger (UART text or error, SPI word with a bit mask, I2C address, START/STOP,
  NAK, data byte). Pre-trigger and post-trigger are set separately in samples or time, times can be
  shown relative to the trigger, and a triggered capture stops instead of dropping data when memory is full.
- **Files**: capture files (`.bfc`, compact and checksummed), setups, VCD export, CSV export of decoded
  items. The setup is also kept between runs.

## Build and run

You need a recent stable Rust toolchain (the workspace sets `rust-version = 1.80`).

```sh
cargo run --release -p bf-app --bin blinkfeuer              # the application
cargo run --release -p bf-app --bin blinkfeuer -- --demo    # a capture of the simulator right away
cargo run --release -p bf-app --bin bfcli -- --help         # command line capture and checks
cargo test --workspace
```

To use the analyzer without root, install the udev rule and replug it:

```sh
sudo cp udev/99-blinkfeuer.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

The stock firmware only enumerates over a SuperSpeed (USB 3) link, so use a USB 3 port and cable. If the
analyzer does not show up on a port that worked before, rebinding the host controller has fixed it.

## Layout of the code

| Crate | What it holds |
|---|---|
| `bf-core` | Sample formats, the capture store (sparse change events plus per-block summaries), trigger conditions |
| `bf-device` | The device interface, the simulator, the stock-firmware and LiteX-gateware backends, the capture session with the trigger logic |
| `bf-decode` | UART, SPI and I2C decoders, the decoding engine, protocol triggers |
| `bf-io` | Capture file format and VCD export |
| `bf-app` | The application (`blinkfeuer`) and the command line tool (`bfcli`) |

The protocol of the stock firmware is implemented from Sipeed's written description in the
[SLogic repository](https://github.com/sipeed/SLogic); no code was taken from their GPL-licensed driver.
The LiteX backend follows the register map and transport of the gateware's own BSD-2-Clause library
(libslogic16u3, Enjoy-Digital).

## Licence

Dual licensed under [MIT](LICENSE-MIT) or [Apache 2.0](LICENSE-APACHE), at your option.
