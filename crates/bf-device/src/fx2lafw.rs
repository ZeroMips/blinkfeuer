//! Cypress FX2 analyzers running sigrok's open `fx2lafw` firmware, such as the cheap 8 channel
//! "Saleae Logic" clones (USB `0925:3881` without firmware).
//!
//! The FX2 has no firmware of its own: on every plug-in the host writes the firmware into its RAM
//! through the chip's built-in loader, and the device then disconnects and enumerates again. The
//! Saleae build of the firmware keeps the Saleae ID `0925:3881`, generic FX2 boards come back as
//! `1d50:608c`; either way the running firmware is recognised by its descriptor strings ("sigrok",
//! "fx2lafw"), as sigrok does it. The
//! firmware is GPL licensed and not part of blinkfeuer; it is loaded at runtime from sigrok's firmware
//! directory (Debian/Ubuntu package `sigrok-firmware-fx2lafw`).
//!
//! The protocol is small: a vendor request for the firmware version, one to start the acquisition
//! with the sample clock, and the bulk IN endpoint 2 that delivers one byte per sample (LSB = CH0),
//! which is already the 8 channel wire format of `bf_core::format`. There is no stop command: the
//! host stops reading, and the next start resets the FIFOs. The firmware cannot report lost data;
//! when the host does not keep up, the FX2 pauses sampling and the capture has a gap.

use crate::stock::{plan_ring_with, transfer_err, usb_err, Accounting, RingPlan};
use crate::{
    Capabilities, CaptureConfig, ChannelMode, Device, DeviceError, DeviceInfo, Pattern, ReadResult,
    Stream, StreamReport,
};
use nusb::transfer::{Bulk, ControlIn, ControlOut, ControlType, In, Recipient, TransferError};
use nusb::{Endpoint, Interface, MaybeFuture};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Saleae Logic and its clones before the firmware is loaded.
pub const VID_SALEAE: u16 = 0x0925;
/// See [`VID_SALEAE`].
pub const PID_SALEAE: u16 = 0x3881;
/// Generic FX2 boards running fx2lafw.
pub const VID_FX2LAFW: u16 = 0x1D50;
/// See [`VID_FX2LAFW`].
pub const PID_FX2LAFW: u16 = 0x608C;
/// File name of the firmware for the Saleae Logic and its clones.
pub const FIRMWARE_SALEAE: &str = "fx2lafw-saleae-logic.fw";

const EP_IN: u8 = 0x82;
const CMD_GET_FW_VERSION: u8 = 0xB0;
const CMD_START: u8 = 0xB1;
const REQUIRED_FW_MAJOR: u8 = 1;
const FLAG_CLK_48MHZ: u8 = 1 << 6;
/// Longest sample delay the firmware's GPIF waveform supports.
const MAX_SAMPLE_DELAY: u32 = 6 * 256;

/// The FX2's built-in loader: vendor request 0xA0 writes internal RAM, CPUCS at 0xE600 holds the 8051
/// in reset while its code is replaced.
const REQ_FIRMWARE_LOAD: u8 = 0xA0;
const REG_CPUCS: u16 = 0xE600;
const LOAD_CHUNK: usize = 4096;
/// Code RAM of the FX2LP (CY7C68013A).
const FX2_RAM: usize = 0x4000;
/// Time the device needs to disconnect and come back with the firmware running. Locally this takes
/// about two seconds, but a usbipd re-attach (WSL) adds seven or more.
const RENUM_TIMEOUT: Duration = Duration::from_secs(20);

const IDLE_LIMIT: Duration = Duration::from_secs(1);
const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);

/// The samplerates offered, in Hz (ascending).
pub const RATES: [f64; 16] = [
    20e3, 25e3, 50e3, 100e3, 200e3, 250e3, 500e3, 1e6, 2e6, 3e6, 4e6, 6e6, 8e6, 12e6, 16e6, 24e6,
];

// ------------------------------------------------------------------------------------------------
// Protocol (generic over the control transport so it can be tested without hardware).

/// Vendor control transfers to the device (abstract, so the protocol can be tested without
/// hardware).
pub trait ControlIo {
    /// Send `data` with vendor request `request` and `wValue` `value`.
    fn vendor_out(&mut self, request: u8, value: u16, data: &[u8]) -> Result<(), DeviceError>;
    /// Read up to `len` bytes with vendor request `request` and `wValue` `value`.
    fn vendor_in(&mut self, request: u8, value: u16, len: u16) -> Result<Vec<u8>, DeviceError>;
}

/// Hold the 8051 in reset, write the image to RAM from address 0, release the reset. The device then
/// disconnects and enumerates again with the new firmware.
pub fn upload_firmware<C: ControlIo>(c: &mut C, image: &[u8]) -> Result<(), DeviceError> {
    if image.is_empty() || image.len() > FX2_RAM {
        return Err(DeviceError::Other(format!(
            "firmware image of {} bytes does not fit the FX2 ({FX2_RAM} bytes)",
            image.len()
        )));
    }
    c.vendor_out(REQ_FIRMWARE_LOAD, REG_CPUCS, &[1])?;
    for (i, chunk) in image.chunks(LOAD_CHUNK).enumerate() {
        c.vendor_out(REQ_FIRMWARE_LOAD, (i * LOAD_CHUNK) as u16, chunk)?;
    }
    c.vendor_out(REQ_FIRMWARE_LOAD, REG_CPUCS, &[0])
}

/// Version of the running firmware: (major, minor).
pub fn firmware_version<C: ControlIo>(c: &mut C) -> Result<(u8, u8), DeviceError> {
    let v = c.vendor_in(CMD_GET_FW_VERSION, 0, 2)?;
    match v[..] {
        [major, minor, ..] => Ok((major, minor)),
        _ => Err(DeviceError::Usb(format!(
            "firmware version: short read ({} bytes)",
            v.len()
        ))),
    }
}

/// The start command for a samplerate: flags (clock source, 8 bit samples) and the sample delay in
/// clock cycles, big endian. The 48 MHz clock is preferred; rates it cannot reach use 30 MHz.
pub fn start_command(rate_hz: f64) -> Result<[u8; 3], DeviceError> {
    let rate = rate_hz.round() as u32;
    let pick = [(48_000_000u32, FLAG_CLK_48MHZ), (30_000_000, 0)]
        .into_iter()
        .find(|&(base, _)| rate > 0 && base % rate == 0 && base / rate - 1 <= MAX_SAMPLE_DELAY);
    let Some((base, flags)) = pick else {
        return Err(DeviceError::Config(format!(
            "{} kHz cannot be derived from the FX2's 48 or 30 MHz clock",
            rate_hz / 1e3
        )));
    };
    let delay = base / rate - 1;
    Ok([flags, (delay >> 8) as u8, delay as u8])
}

/// Start the acquisition at `rate_hz`.
pub fn start<C: ControlIo>(c: &mut C, rate_hz: f64) -> Result<(), DeviceError> {
    c.vendor_out(CMD_START, 0, &start_command(rate_hz)?)
}

/// Directories searched for the firmware, in order: `$BLINKFEUER_FIRMWARE_DIR`,
/// `$SIGROK_FIRMWARE_DIR`, then sigrok's data directories.
pub fn firmware_dirs() -> Vec<PathBuf> {
    let mut dirs: Vec<PathBuf> = ["BLINKFEUER_FIRMWARE_DIR", "SIGROK_FIRMWARE_DIR"]
        .iter()
        .filter_map(std::env::var_os)
        .map(PathBuf::from)
        .collect();
    let data_home = std::env::var_os("XDG_DATA_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| Path::new(&h).join(".local/share")));
    dirs.extend(data_home.map(|d| d.join("sigrok-firmware")));
    dirs.push("/usr/local/share/sigrok-firmware".into());
    dirs.push("/usr/share/sigrok-firmware".into());
    dirs
}

/// The first file called `name` in [`firmware_dirs`].
pub fn find_firmware(name: &str) -> Result<PathBuf, DeviceError> {
    firmware_dirs()
        .into_iter()
        .map(|d| d.join(name))
        .find(|p| p.is_file())
        .ok_or_else(|| {
            DeviceError::Other(format!(
                "firmware {name} not found; install sigrok's fx2lafw firmware (Debian/Ubuntu: \
                 sigrok-firmware-fx2lafw) or set BLINKFEUER_FIRMWARE_DIR"
            ))
        })
}

// ------------------------------------------------------------------------------------------------
// USB.

struct UsbControl<'a> {
    iface: &'a Interface,
}

impl ControlIo for UsbControl<'_> {
    fn vendor_out(&mut self, request: u8, value: u16, data: &[u8]) -> Result<(), DeviceError> {
        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index: 0,
                    data,
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(|e| transfer_err(&e))
    }

    fn vendor_in(&mut self, request: u8, value: u16, len: u16) -> Result<Vec<u8>, DeviceError> {
        self.iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index: 0,
                    length: len,
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(|e| transfer_err(&e))
    }
}

fn is_fx2(d: &nusb::DeviceInfo) -> bool {
    matches!(
        (d.vendor_id(), d.product_id()),
        (VID_SALEAE, PID_SALEAE) | (VID_FX2LAFW, PID_FX2LAFW)
    )
}

/// An FX2 analyzer whose firmware is fx2lafw.
fn is_running(d: &nusb::DeviceInfo) -> bool {
    is_fx2(d) && has_fx2lafw_strings(d.manufacturer_string(), d.product_string())
}

/// A Saleae Logic (clone) that still needs the firmware.
fn is_blank(d: &nusb::DeviceInfo) -> bool {
    d.vendor_id() == VID_SALEAE && d.product_id() == PID_SALEAE && !is_running(d)
}

/// Whether these USB string descriptors are those of a running fx2lafw firmware.
pub fn has_fx2lafw_strings(manufacturer: Option<&str>, product: Option<&str>) -> bool {
    manufacturer == Some("sigrok") && product == Some("fx2lafw")
}

fn list() -> Result<Vec<nusb::DeviceInfo>, DeviceError> {
    Ok(nusb::list_devices()
        .wait()
        .map_err(|e| usb_err("listing USB devices", e))?
        .collect())
}

fn claim(d: &nusb::DeviceInfo) -> Result<Interface, DeviceError> {
    d.open()
        .wait()
        .map_err(|e| usb_err("opening the device", e))?
        .claim_interface(0)
        .wait()
        .map_err(|e| usb_err("claiming interface 0", e))
}

fn describe(d: &nusb::DeviceInfo) -> DeviceInfo {
    let state = if is_blank(d) { ", no firmware yet" } else { "" };
    DeviceInfo {
        id: format!(
            "fx2lafw:{}-{}",
            d.busnum(),
            d.port_chain()
                .iter()
                .map(u8::to_string)
                .collect::<Vec<_>>()
                .join(".")
        ),
        name: format!("FX2 logic analyzer{state}"),
        backend: "fx2lafw",
    }
}

/// Load the firmware into a blank analyzer and wait until it is back with the firmware running.
fn load_firmware(blank: &nusb::DeviceInfo) -> Result<nusb::DeviceInfo, DeviceError> {
    let path = find_firmware(FIRMWARE_SALEAE)?;
    let image = std::fs::read(&path).map_err(|e| DeviceError::Other(format!("{path:?}: {e}")))?;
    let before: Vec<_> = list()?
        .iter()
        .filter(|d| is_running(d))
        .map(|d| d.id())
        .collect();
    upload_firmware(
        &mut UsbControl {
            iface: &claim(blank)?,
        },
        &image,
    )?;
    let deadline = Instant::now() + RENUM_TIMEOUT;
    while Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(200));
        if let Some(d) = list()?
            .into_iter()
            .find(|d| is_running(d) && !before.contains(&d.id()))
        {
            return Ok(d);
        }
    }
    Err(DeviceError::Other(format!(
        "firmware loaded, but the analyzer did not come back within {} s (with usbipd, keep \
         `usbipd attach --auto-attach` running so it is attached again after the reconnect)",
        RENUM_TIMEOUT.as_secs()
    )))
}

/// An FX2 analyzer running fx2lafw.
pub struct Fx2lafwDevice {
    iface: Interface,
    info: DeviceInfo,
    caps: Capabilities,
}

impl Fx2lafwDevice {
    /// Descriptions of the attached analyzers, with or without the firmware (without opening them).
    pub fn available() -> Vec<DeviceInfo> {
        list()
            .unwrap_or_default()
            .iter()
            .filter(|d| is_running(d) || is_blank(d))
            .map(describe)
            .collect()
    }

    /// Open the first analyzer, loading the firmware first if it has none yet.
    pub fn open_first() -> Result<Self, DeviceError> {
        let devices = list()?;
        let di = match devices.iter().find(|d| is_running(d)) {
            Some(d) => d.clone(),
            None => match devices.iter().find(|d| is_blank(d)) {
                Some(blank) => load_firmware(blank)?,
                None => {
                    return Err(DeviceError::Other(format!(
                        "no FX2 analyzer found ({VID_SALEAE:04x}:{PID_SALEAE:04x} or \
                         {VID_FX2LAFW:04x}:{PID_FX2LAFW:04x}); it reconnects when its firmware \
                         is loaded, so with usbipd keep `usbipd attach --auto-attach` running"
                    )))
                }
            },
        };
        let iface = claim(&di)?;
        let (major, minor) = firmware_version(&mut UsbControl { iface: &iface })?;
        if major != REQUIRED_FW_MAJOR {
            return Err(DeviceError::Other(format!(
                "fx2lafw firmware {major}.{minor} is not supported (need {REQUIRED_FW_MAJOR}.x)"
            )));
        }
        let mut info = describe(&di);
        info.name = format!("FX2 logic analyzer (fx2lafw {major}.{minor})");
        Ok(Self {
            iface,
            info,
            caps: capabilities(),
        })
    }
}

/// 8 channels up to 24 MHz; no threshold and no test patterns.
pub fn capabilities() -> Capabilities {
    Capabilities {
        modes: vec![ChannelMode {
            channels: 8,
            max_rate: 24e6,
        }],
        rates: RATES.to_vec(),
        patterns: vec![Pattern::Inputs],
        threshold: None,
    }
}

impl Device for Fx2lafwDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn capabilities(&self) -> &Capabilities {
        &self.caps
    }

    fn start(&mut self, cfg: &CaptureConfig) -> Result<Box<dyn Stream>, DeviceError> {
        self.caps.validate(cfg)?;
        let command = start_command(cfg.sample_rate)?;
        let mut ep = self
            .iface
            .endpoint::<Bulk, In>(EP_IN)
            .map_err(|e| usb_err("opening bulk IN endpoint 0x82", e))?;
        ep.clear_halt()
            .wait()
            .map_err(|e| usb_err("clearing endpoint halt", e))?;
        let limit_bytes = cfg.limit_samples.map(|n| cfg.channels.bytes_for(n));
        let plan = plan_ring_with(cfg.sample_rate, cfg.channels.get(), limit_bytes, 0);
        let mut s = Fx2Stream {
            ep,
            plan,
            acc: Accounting::new(0, limit_bytes),
            t_data: Instant::now(),
            started: false,
            finished: false,
            stopped: false,
            report: StreamReport {
                note: Some(format!("xfer={}x{}", plan.xfer, plan.count)),
                ..Default::default()
            },
        };
        // Transfers first, so the FIFO is drained from the first sample on.
        while s.ep.pending() < plan.count {
            s.ep.submit(s.ep.allocate(plan.xfer));
        }
        UsbControl { iface: &self.iface }.vendor_out(CMD_START, 0, &command)?;
        s.t_data = Instant::now();
        Ok(Box::new(s))
    }
}

struct Fx2Stream {
    ep: Endpoint<Bulk, In>,
    plan: RingPlan,
    acc: Accounting,
    t_data: Instant,
    started: bool,
    /// The requested samples were delivered; the next read reports `Ended`.
    finished: bool,
    stopped: bool,
    report: StreamReport,
}

impl Stream for Fx2Stream {
    fn read(&mut self, timeout: Duration) -> Result<ReadResult, DeviceError> {
        if self.finished || self.stopped {
            return Ok(ReadResult::Ended);
        }
        let Some(done) = self.ep.wait_next_complete(timeout) else {
            // Before the first data allow for filling the first transfer at slow rates.
            let limit = if self.started {
                IDLE_LIMIT
            } else {
                IDLE_LIMIT + Duration::from_secs(3)
            };
            if self.t_data.elapsed() > limit {
                return Err(DeviceError::Stalled(format!(
                    "no data for {} s",
                    limit.as_secs()
                )));
            }
            return Ok(ReadResult::Timeout);
        };
        let data = done.buffer[..done.actual_len.min(done.buffer.len())].to_vec();
        let status = done.status;
        // Resubmit first so the bulk pipe never idles on host work; transfers left over at the end of a
        // finite capture are cancelled by stop().
        let mut buf = done.buffer;
        buf.clear();
        buf.set_requested_len(self.plan.xfer);
        self.ep.submit(buf);
        if let Err(e) = status {
            return Err(match e {
                TransferError::Cancelled => DeviceError::Stalled("transfer cancelled".into()),
                other => transfer_err(&other),
            });
        }
        if data.is_empty() {
            return Ok(ReadResult::Timeout);
        }
        self.started = true;
        self.t_data = Instant::now();
        let (out, complete) = self.acc.feed(&data);
        self.report.bytes += out.len() as u64;
        self.finished = complete;
        if out.is_empty() {
            return Ok(if complete {
                ReadResult::Ended
            } else {
                ReadResult::Timeout
            });
        }
        Ok(ReadResult::Data(out.to_vec()))
    }

    fn stop(&mut self) -> Result<StreamReport, DeviceError> {
        if !self.stopped {
            self.stopped = true;
            self.ep.cancel_all();
            for _ in 0..5 {
                if self.ep.pending() == 0 {
                    break;
                }
                let _ = self.ep.wait_next_complete(Duration::from_secs(1));
            }
        }
        Ok(self.report.clone())
    }
}

impl Drop for Fx2Stream {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bf_core::ChannelCount;

    #[derive(Default)]
    struct Fake {
        outs: Vec<(u8, u16, Vec<u8>)>,
        version: Vec<u8>,
    }

    impl ControlIo for Fake {
        fn vendor_out(&mut self, request: u8, value: u16, data: &[u8]) -> Result<(), DeviceError> {
            self.outs.push((request, value, data.to_vec()));
            Ok(())
        }

        fn vendor_in(&mut self, request: u8, _: u16, len: u16) -> Result<Vec<u8>, DeviceError> {
            assert_eq!((request, len), (CMD_GET_FW_VERSION, 2));
            Ok(self.version.clone())
        }
    }

    #[test]
    fn firmware_upload_holds_the_cpu_in_reset_and_writes_4k_chunks() {
        let image: Vec<u8> = (0..9000u32).map(|i| i as u8).collect();
        let mut f = Fake::default();
        upload_firmware(&mut f, &image).unwrap();
        let shape: Vec<(u8, u16, usize)> = f.outs.iter().map(|o| (o.0, o.1, o.2.len())).collect();
        assert_eq!(
            shape,
            vec![
                (0xA0, 0xE600, 1),
                (0xA0, 0x0000, 4096),
                (0xA0, 0x1000, 4096),
                (0xA0, 0x2000, 808),
                (0xA0, 0xE600, 1),
            ]
        );
        assert_eq!((f.outs[0].2[0], f.outs[4].2[0]), (1, 0));
        let written: Vec<u8> = f.outs[1..4].iter().flat_map(|o| o.2.clone()).collect();
        assert_eq!(written, image);
    }

    #[test]
    fn firmware_that_does_not_fit_is_refused_before_touching_the_device() {
        let mut f = Fake::default();
        assert!(upload_firmware(&mut f, &[0; FX2_RAM + 1]).is_err());
        assert!(upload_firmware(&mut f, &[]).is_err());
        assert!(f.outs.is_empty());
    }

    #[test]
    fn start_command_prefers_the_48_mhz_clock() {
        // 24 MHz: 48 MHz / 2, delay 1.
        assert_eq!(start_command(24e6).unwrap(), [0x40, 0, 1]);
        assert_eq!(start_command(1e6).unwrap(), [0x40, 0, 47]);
        // 50 kHz: delay 959 = 0x03BF, big endian.
        assert_eq!(start_command(50e3).unwrap(), [0x40, 0x03, 0xBF]);
        // 20 kHz would need a delay of 2399 at 48 MHz, beyond the waveform: 30 MHz, delay 1499.
        assert_eq!(start_command(20e3).unwrap(), [0x00, 0x05, 0xDB]);
        assert_eq!(start_command(25e3).unwrap(), [0x00, 0x04, 0xAF]);
    }

    #[test]
    fn every_offered_rate_has_a_start_command() {
        for r in RATES {
            assert!(start_command(r).is_ok(), "{r}");
        }
        assert!(start_command(7e6).is_err());
        assert!(start_command(10e3).is_err());
        assert!(start_command(0.0).is_err());
    }

    #[test]
    fn start_sends_the_command() {
        let mut f = Fake::default();
        start(&mut f, 12e6).unwrap();
        assert_eq!(f.outs, vec![(CMD_START, 0, vec![0x40, 0, 3])]);
    }

    #[test]
    fn running_firmware_is_recognised_by_its_strings() {
        assert!(has_fx2lafw_strings(Some("sigrok"), Some("fx2lafw")));
        // A clone before the firmware is loaded has no strings (or its own).
        assert!(!has_fx2lafw_strings(None, None));
        assert!(!has_fx2lafw_strings(Some("Saleae"), Some("Logic")));
    }

    #[test]
    fn firmware_version_is_read() {
        let mut f = Fake {
            version: vec![1, 4],
            ..Default::default()
        };
        assert_eq!(firmware_version(&mut f).unwrap(), (1, 4));
        f.version = vec![1];
        assert!(firmware_version(&mut f).is_err());
    }

    #[test]
    fn eight_channels_up_to_24_mhz_inputs_only() {
        let c = capabilities();
        let cfg = |ch: u8, rate: f64, pattern: Pattern| CaptureConfig {
            channels: ChannelCount::new(ch).unwrap(),
            sample_rate: rate,
            limit_samples: None,
            pattern,
            threshold_v: 1.7,
        };
        assert!(c.validate(&cfg(8, 24e6, Pattern::Inputs)).is_ok());
        assert!(c.validate(&cfg(8, 48e6, Pattern::Inputs)).is_err());
        assert!(c.validate(&cfg(16, 1e6, Pattern::Inputs)).is_err());
        assert!(c.validate(&cfg(8, 1e6, Pattern::Counter)).is_err());
    }

    #[test]
    fn firmware_search_honours_the_override() {
        let dir = std::env::temp_dir().join(format!("bf-fw-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join(FIRMWARE_SALEAE), [0u8; 16]).unwrap();
        std::env::set_var("BLINKFEUER_FIRMWARE_DIR", &dir);
        let found = find_firmware(FIRMWARE_SALEAE);
        std::env::remove_var("BLINKFEUER_FIRMWARE_DIR");
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(found.unwrap(), dir.join(FIRMWARE_SALEAE));
        assert!(find_firmware("no-such-firmware.fw").is_err());
    }
}
