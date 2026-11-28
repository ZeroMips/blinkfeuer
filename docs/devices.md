# Devices and drivers

Blinkfeuer talks to its analyzers through one device interface (`bf-device`). Every backend can also be
used from the command line with `bfcli --device <name>`, and shows up in the application when the hardware is
attached. This page holds what is verified for each one, the known problems, and where each protocol
implementation comes from. The [README](../README.md) has the short version.

| Backend | `bfcli --device` | Hardware | Channels / rates |
|---|---|---|---|
| Simulator | `sim` | none | up to 16 channels, test patterns for UART, SPI, I2C |
| Stock firmware | `stock` | Sipeed SLogic16U3 (`359f:3031`) | 16 ch up to 200 MHz, 8 ch 400 MHz, 4 ch 800 MHz |
| LiteX gateware | `litex` | Sipeed SLogic16U3 with the open gateware (`1d50:605a`) | as above |
| fx2lafw | `fx2lafw` | FX2 analyzers ("Saleae Logic" clones) | 8 ch, 20 kHz to 24 MHz |

Linux only.

## Simulator

Everything in the application works on it, and most of it is covered by tests. `blinkfeuer --demo` starts a
capture of the simulator right away.

## SLogic16U3, stock firmware

Acquisition has been run on hardware at full rate with the analyzer's own test patterns, with exact sample
counts. **Not yet tried with real probe signals.**

The stock firmware only enumerates over a SuperSpeed (USB 3) link, so use a USB 3 port and cable. If the
analyzer does not show up on a port that worked before, rebinding the host controller has fixed it.

The protocol is implemented from Sipeed's written description in the
[SLogic repository](https://github.com/sipeed/SLogic); no code was taken from their GPL-licensed driver.

## SLogic16U3, LiteX gateware

The open gateware for the same hardware, selectable in the application when the analyzer runs it. It runs on
the real device with the analyzer's test patterns only (no probe signals yet). It needs a gateware with the
Link Commands fix ([usb3_pipe#214](https://github.com/enjoy-digital/usb3_pipe/pull/214)); without it streams
die at a few MB/s.

- **Verified**: 16 channels at 100 and 200 MHz (up to ~400 MB/s), 8 channels at 400 MHz and 4 channels at
  800 MHz were captured through the whole application. The raw stream is exact in most runs.
- **Known problems**:
  - The gateware's 16 KB FIFO is sensitive to host jitter at full rate. This shows up as an occasional
    "device FIFO overflow", which is reported as an error, never as silent data loss.
  - 16 channels at 160 MHz (about 80% of the link, 320 MB/s) is not reliable with the current production
    gateware. On an idle machine its raw stream ended with "device FIFO overflow" after 80 to 260 MB in all
    four runs, and the device then re-enumerated with broken descriptors (a replug or a JTAG reload fixes
    that). A debug build of the same gateware with a Bulk IN burst of 8 packets instead of 16 streamed exactly
    in 28 of 28 runs (up to 1 GB each), so this depends on the gateware build; the cause is under
    investigation. 100 MHz and 200 MHz at 16 channels are not affected.
  - Host load makes it worse. With every core busy the burst 8 build also overflowed: a plain read loop that
    verifies the data on the reading thread failed every time, and the application's capture path (separate
    reader thread and queue) completed 4 of 5 runs with twice as many busy processes as cores. Close heavy
    programs while capturing at high rates.
  - One USB 3 port of the test machine does not work with any firmware.
- **Transfers** are kept small (512 KiB, up to 24 in flight) to stay within Linux's default `usbfs_memory_mb`
  of 16. Raise it if other programs use USB bulk transfers at the same time.
- A dedicated reader thread with a bounded queue (about 256 MiB) services the device, so a slow ingest shows as
  an overflow report instead of stalling the USB queue.

The backend follows the register map and transport of the gateware's own BSD-2-Clause library
(libslogic16u3, Enjoy-Digital).

## FX2 analyzers with fx2lafw

8 channels, 20 kHz to 24 MHz, no threshold and no test patterns. Used on a Saleae Logic clone with real
signals (UART / Modbus RTU at 16 MHz, also through usbipd under WSL).

Blinkfeuer loads the firmware itself from sigrok's firmware directory, so install it first (Debian/Ubuntu:
`sigrok-firmware-fx2lafw`; or set `BLINKFEUER_FIRMWARE_DIR`). The firmware cannot report lost data: when the
host falls behind, the capture silently has a gap.

The analyzer reconnects when the firmware is loaded: a Saleae Logic clone keeps its ID `0925:3881`, generic
FX2 boards come back as `1d50:608c`.

The backend talks the command set of sigrok's fx2lafw firmware (three vendor requests and a bulk endpoint) and
the FX2's built-in firmware loader; no libsigrok code is used. The GPL-licensed firmware is not distributed
with Blinkfeuer, it is loaded from the system at runtime.

## Access rights

See [install.md](install.md) for the udev rule and the WSL notes.
