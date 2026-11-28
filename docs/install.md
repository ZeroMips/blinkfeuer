# Installing and running

You need Rust 1.88 or newer (`rust-version` of the workspace; the dependencies need it).

```sh
cargo run --release -p bf-app --bin blinkfeuer              # the application
cargo run --release -p bf-app --bin blinkfeuer -- --demo    # a capture of the simulator right away
cargo run --release -p bf-app --bin bfcli -- --help         # command line capture and checks
cargo test --workspace
```

## Desktop entry and icon

To get the icon in the task bar and the application in the menu, install a desktop entry and the icon,
pointing at the binary you run it from. For your user (into `~/.local/share`):

```sh
target/release/blinkfeuer --install-desktop
```

## USB access without root

Install the udev rule and replug the analyzer:

```sh
sudo cp udev/99-blinkfeuer.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

The rule covers the SLogic16U3 (stock firmware and LiteX gateware) and FX2 analyzers, before and after the
firmware load. Device details are in [devices.md](devices.md).

## Windows Subsystem for Linux

WSLg only reads desktop entries from the system folders, so install them there:

```sh
sudo target/release/blinkfeuer --install-desktop=/usr/local/share
```

Blinkfeuer opens its window over X11 (XWayland), where the window shows its own icon. With Wayland, WSLg's
compositor crashed now and then once the desktop entry was installed (all WSL windows go away with it).
`BLINKFEUER_WAYLAND=1` keeps Wayland.

USB devices come in through usbipd. An FX2 analyzer reconnects when blinkfeuer has loaded its firmware, so
keep `usbipd attach --wsl --busid <BUSID> --auto-attach` running and the analyzer is attached again after the
reconnect.
