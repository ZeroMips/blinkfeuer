//! Sipeed SLogic16U3 running the stock firmware (USB `359f:3031`).
//!
//! Implemented from Sipeed's written protocol specification (`slogic-protocol.md`, sections 2, 3 and 8):
//! vendor control transfers carry 32-bit register accesses, configuration goes through the AUX mailbox,
//! and the bulk IN endpoint delivers the raw packed sample stream with no framing. The first four bytes
//! of the 16U3 stream are a partial word and are dropped.

use crate::{
    Capabilities, CaptureConfig, ChannelMode, Device, DeviceError, DeviceInfo, Pattern, ReadResult,
    Stream, StreamReport,
};
use nusb::transfer::{Bulk, ControlIn, ControlOut, ControlType, In, Recipient, TransferError};
use nusb::{Endpoint, Interface, MaybeFuture, Speed};
use std::time::{Duration, Instant};

pub const VID: u16 = 0x359F;
pub const PID_16U3: u16 = 0x3031;
const EP_IN: u8 = 0x82;

const R_CTRL: u16 = 0x0004;
const R_AUX: u16 = 0x000C;
const R_AUX_PAYLOAD: u16 = 0x0010;
const CTRL_RUN: u32 = 1;
const CTRL_RST: u32 = 2;

const AUX_CHANNEL: u32 = 1;
const AUX_RATE: u32 = 2;
const AUX_VREF: u32 = 3;
const AUX_TEST: u32 = 5;

/// Header polls before an AUX transaction is declared failed.
const AUX_POLLS: usize = 8;
/// The first four bytes of a 16U3 stream are a partial word.
const FIRST_DROP_BYTES: usize = 4;
/// Complete silence this long after data has started to flow is fatal.
const IDLE_LIMIT: Duration = Duration::from_secs(1);
const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);

// ------------------------------------------------------------------------------------------------
// Register protocol (generic over the register transport so it can be tested without hardware).

pub trait RegIo {
    fn read32(&mut self, addr: u16) -> Result<u32, DeviceError>;
    fn write32(&mut self, addr: u16, value: u32) -> Result<(), DeviceError>;
}

/// Open an AUX transaction: write the command, poll the header until the ready bit is set, return the
/// payload length in words.
fn aux_open<R: RegIo>(r: &mut R, cmd: u32) -> Result<usize, DeviceError> {
    r.write32(R_AUX, cmd)?;
    for _ in 0..AUX_POLLS {
        let h = r.read32(R_AUX)?;
        if (h >> 16) & 1 == 1 {
            let words = ((h & 0xFFFF) >> 9) as usize;
            return if (1..=16).contains(&words) {
                Ok(words)
            } else {
                Err(DeviceError::Usb(format!(
                    "AUX command {cmd}: bad payload length {words}"
                )))
            };
        }
    }
    Err(DeviceError::Usb(format!(
        "AUX command {cmd}: device not ready"
    )))
}

fn aux_read<R: RegIo>(r: &mut R, words: usize) -> Result<Vec<u32>, DeviceError> {
    (0..words)
        .map(|i| r.read32(R_AUX_PAYLOAD + 4 * i as u16))
        .collect()
}

/// One single-word AUX block: write command, read the old payload, write word 0, confirm by reading back.
fn aux_set_word<R: RegIo>(r: &mut R, cmd: u32, value: u32, what: &str) -> Result<(), DeviceError> {
    let words = aux_open(r, cmd)?;
    let _old = aux_read(r, words)?;
    r.write32(R_AUX_PAYLOAD, value)?;
    let back = r.read32(R_AUX_PAYLOAD)?;
    if back != value {
        return Err(DeviceError::Usb(format!(
            "{what}: wrote {value:#x}, device reports {back:#x}"
        )));
    }
    Ok(())
}

pub fn set_channels<R: RegIo>(r: &mut R, channels: u8) -> Result<(), DeviceError> {
    let mask = if channels >= 32 {
        u32::MAX
    } else {
        (1u32 << channels) - 1
    };
    aux_set_word(r, AUX_CHANNEL, mask, "channel mask")
}

/// Walk the firmware's base-clock table until a base is a multiple of `rate`, then program the divider.
pub fn set_samplerate<R: RegIo>(r: &mut R, rate_hz: f64) -> Result<(), DeviceError> {
    let want = rate_hz.round() as u64;
    if want == 0 {
        return Err(DeviceError::Config("samplerate must be positive".into()));
    }
    let words = aux_open(r, AUX_RATE)?;
    if words < 2 {
        return Err(DeviceError::Usb(format!(
            "samplerate payload has {words} words, expected 2"
        )));
    }
    let mut p = aux_read(r, words)?;
    for _ in 0..32 {
        let (idx, base_mhz) = (p[0] & 0xFFFF, (p[0] >> 16) as u64);
        let base = base_mhz * 1_000_000;
        if base != 0 && base % want == 0 {
            let divider = (base / want - 1) as u32;
            r.write32(R_AUX_PAYLOAD + 4, divider)?;
            let back = r.read32(R_AUX_PAYLOAD + 4)?;
            if back != divider {
                return Err(DeviceError::Usb(format!(
                    "samplerate: wrote divider {divider}, device reports {back}"
                )));
            }
            return Ok(());
        }
        // Not a multiple of this base: select the next one and re-read.
        r.write32(R_AUX_PAYLOAD, (p[0] & 0xFFFF_0000) | (idx + 1) & 0xFFFF)?;
        p = aux_read(r, words)?;
        if p[0] & 0xFFFF == idx {
            break; // the device did not advance: end of the table
        }
    }
    Err(DeviceError::Config(format!(
        "no base clock yields {} MHz",
        rate_hz / 1e6
    )))
}

/// DAC code for a threshold voltage (10 bit against a 1.6 V reference): `V / 3.33 / 2 * 1024`, rounded.
pub fn vref_code(volts: f64) -> u32 {
    (volts / 3.33 / 2.0 * 1024.0).round().clamp(0.0, 1023.0) as u32
}

pub fn pattern_code(p: Pattern) -> Result<u32, DeviceError> {
    match p {
        Pattern::Inputs => Ok(0),
        Pattern::Counter => Ok(1),
        Pattern::Emulation => Ok(2),
        Pattern::Generator => Err(DeviceError::Config(
            "the blinkfeuer generator pattern is simulator only".into(),
        )),
    }
}

pub fn set_vref<R: RegIo>(r: &mut R, volts: f64) -> Result<(), DeviceError> {
    aux_set_word(r, AUX_VREF, vref_code(volts), "threshold")
}

pub fn set_pattern<R: RegIo>(r: &mut R, p: Pattern) -> Result<(), DeviceError> {
    aux_set_word(r, AUX_TEST, pattern_code(p)?, "test pattern")?;
    if p == Pattern::Inputs {
        reset(r)?; // selecting Normal also issues a reset
    }
    Ok(())
}

pub fn reset<R: RegIo>(r: &mut R) -> Result<(), DeviceError> {
    r.write32(R_CTRL, CTRL_RST)?;
    r.write32(R_CTRL, 0)
}

/// Channel -> samplerate -> vref -> pattern, each confirmed by a read-back (spec section 6.7).
pub fn configure<R: RegIo>(r: &mut R, cfg: &CaptureConfig) -> Result<(), DeviceError> {
    set_channels(r, cfg.channels.get())?;
    set_samplerate(r, cfg.sample_rate)?;
    set_vref(r, cfg.threshold_v)?;
    set_pattern(r, cfg.pattern)
}

// ------------------------------------------------------------------------------------------------
// Transfer planning and sample accounting.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RingPlan {
    /// Bytes per transfer.
    pub xfer: usize,
    /// Transfers kept in flight.
    pub count: usize,
}

const KIB: usize = 1024;
const MIB: usize = 1024 * KIB;

/// About 5 ms of data per transfer (32 KiB .. 2 MiB, 32 KiB aligned), at least 4 in flight within the
/// default Linux usbfs budget; a finite capture smaller than one transfer gets one of its own size.
pub fn plan_ring(rate: f64, channels: u8, limit_bytes: Option<u64>) -> RingPlan {
    plan_ring_with(rate, channels, limit_bytes, FIRST_DROP_BYTES)
}

/// [`plan_ring`] for a stream that has `first_drop` extra bytes at its head.
pub fn plan_ring_with(
    rate: f64,
    channels: u8,
    limit_bytes: Option<u64>,
    first_drop: usize,
) -> RingPlan {
    let byte_rate = rate * channels as f64 / 8.0;
    let ideal = (byte_rate * 0.005) as usize;
    let mut xfer = ideal.clamp(32 * KIB, 2 * MIB).div_ceil(32 * KIB) * 32 * KIB;
    let mut count = (12 * MIB / xfer).clamp(4, 16);
    if let Some(lb) = limit_bytes {
        let need = lb as usize + first_drop;
        if need < xfer {
            xfer = need.div_ceil(KIB) * KIB; // a multiple of the SuperSpeed bulk packet size
        }
        count = count.min(need.div_ceil(xfer)).max(1);
    }
    RingPlan { xfer, count }
}

/// First-word drop and exact-length accounting over arbitrarily sliced transfers.
#[derive(Debug)]
pub struct Accounting {
    drop_left: usize,
    remaining: Option<u64>,
}

impl Accounting {
    pub fn new(first_drop: usize, limit_bytes: Option<u64>) -> Self {
        Self {
            drop_left: first_drop,
            remaining: limit_bytes,
        }
    }

    /// The part of `data` to deliver, and whether the capture is complete after it.
    pub fn feed<'a>(&mut self, data: &'a [u8]) -> (&'a [u8], bool) {
        let skip = self.drop_left.min(data.len());
        self.drop_left -= skip;
        let mut data = &data[skip..];
        if let Some(rem) = &mut self.remaining {
            let take = (*rem).min(data.len() as u64) as usize;
            data = &data[..take];
            *rem -= take as u64;
            return (data, *rem == 0);
        }
        (data, false)
    }
}

// ------------------------------------------------------------------------------------------------
// USB.

pub(crate) fn usb_err(what: &str, e: impl std::fmt::Display) -> DeviceError {
    DeviceError::Usb(format!("{what}: {e}"))
}

pub(crate) fn transfer_err(e: &TransferError) -> DeviceError {
    match e {
        TransferError::Disconnected => DeviceError::Disconnected,
        other => DeviceError::Usb(other.to_string()),
    }
}

struct UsbRegs {
    iface: Interface,
}

impl RegIo for UsbRegs {
    fn read32(&mut self, addr: u16) -> Result<u32, DeviceError> {
        let data = self
            .iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0x00,
                    value: addr,
                    index: 0,
                    length: 4,
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(|e| transfer_err(&e))?;
        let b: [u8; 4] = data
            .get(..4)
            .and_then(|s| s.try_into().ok())
            .ok_or_else(|| {
                DeviceError::Usb(format!(
                    "register {addr:#x}: short read ({} bytes)",
                    data.len()
                ))
            })?;
        Ok(u32::from_le_bytes(b))
    }

    fn write32(&mut self, addr: u16, value: u32) -> Result<(), DeviceError> {
        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0x01,
                    value: addr,
                    index: 0,
                    data: &value.to_le_bytes(),
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(|e| transfer_err(&e))
    }
}

/// Throw away whatever the endpoint still delivers (after STOP, or from a killed earlier capture).
pub(crate) fn drain(ep: &mut Endpoint<Bulk, In>) {
    let deadline = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        ep.submit(ep.allocate(64 * KIB));
        match ep.wait_next_complete(Duration::from_millis(100)) {
            Some(c) if c.actual_len > 0 && c.status.is_ok() => continue,
            Some(_) => return,
            None => {
                ep.cancel_all();
                while ep.pending() > 0 {
                    let _ = ep.wait_next_complete(Duration::from_secs(1));
                }
                return;
            }
        }
    }
}

/// An SLogic16U3 with the stock firmware.
pub struct StockDevice {
    iface: Interface,
    info: DeviceInfo,
    caps: Capabilities,
}

impl StockDevice {
    /// Descriptions of the attached analyzers (without opening them).
    pub fn available() -> Vec<DeviceInfo> {
        let Ok(list) = nusb::list_devices().wait() else {
            return Vec::new();
        };
        list.filter(|d| d.vendor_id() == VID && d.product_id() == PID_16U3)
            .map(|d| describe(&d))
            .collect()
    }

    /// Open the first attached analyzer and bring it to a known idle state (stop, drain, reset).
    pub fn open_first() -> Result<Self, DeviceError> {
        let di = nusb::list_devices()
            .wait()
            .map_err(|e| usb_err("listing USB devices", e))?
            .find(|d| d.vendor_id() == VID && d.product_id() == PID_16U3)
            .ok_or_else(|| {
                DeviceError::Other(
                    "no SLogic16U3 (359f:3031) found; it only enumerates over a USB 3 link".into(),
                )
            })?;
        let info = describe(&di);
        let usb3 = !matches!(di.speed(), Some(Speed::Low | Speed::Full | Speed::High));
        let dev = di
            .open()
            .wait()
            .map_err(|e| usb_err("opening the device", e))?;
        let iface = dev
            .claim_interface(0)
            .wait()
            .map_err(|e| usb_err("claiming interface 0", e))?;
        let mut regs = UsbRegs {
            iface: iface.clone(),
        };
        regs.write32(R_CTRL, 0)?;
        if let Ok(mut ep) = iface.endpoint::<Bulk, In>(EP_IN) {
            drain(&mut ep);
        }
        reset(&mut regs)?;
        Ok(Self {
            iface,
            info,
            caps: capabilities(usb3),
        })
    }
}

fn describe(d: &nusb::DeviceInfo) -> DeviceInfo {
    let speed = match d.speed() {
        Some(Speed::Low) => "1.5M",
        Some(Speed::Full) => "12M",
        Some(Speed::High) => "480M",
        Some(Speed::Super) => "5G",
        Some(Speed::SuperPlus) => "10G",
        _ => "?",
    };
    let serial = d.serial_number().unwrap_or("-");
    DeviceInfo {
        id: format!("stock16u3:{serial}"),
        name: format!("({speed}) {}", d.product_string().unwrap_or("SLogic16 U3")),
        backend: "stock",
    }
}

/// Channel modes and rates of the 16U3; on a USB 2 link the payload caps the rate at 320 MHz / channels.
pub fn capabilities(usb3: bool) -> Capabilities {
    let cap = |ch: u8, native: f64| ChannelMode {
        channels: ch,
        max_rate: if usb3 {
            native
        } else {
            native.min(320e6 / ch as f64)
        },
    };
    Capabilities {
        modes: vec![cap(4, 800e6), cap(8, 400e6), cap(16, 200e6)],
        rates: [
            1e6, 2e6, 5e6, 10e6, 20e6, 25e6, 40e6, 50e6, 80e6, 100e6, 160e6, 200e6, 400e6, 800e6,
        ]
        .to_vec(),
        patterns: vec![Pattern::Inputs, Pattern::Counter, Pattern::Emulation],
        threshold: Some((0.0, 3.3, 0.05)),
    }
}

impl Device for StockDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn capabilities(&self) -> &Capabilities {
        &self.caps
    }

    fn start(&mut self, cfg: &CaptureConfig) -> Result<Box<dyn Stream>, DeviceError> {
        self.caps.validate(cfg)?;
        let ep = self
            .iface
            .endpoint::<Bulk, In>(EP_IN)
            .map_err(|e| usb_err("opening bulk IN endpoint 0x82", e))?;
        let limit_bytes = cfg.limit_samples.map(|n| cfg.channels.bytes_for(n));
        let plan = plan_ring(cfg.sample_rate, cfg.channels.get(), limit_bytes);
        let mut s = StockStream {
            regs: UsbRegs {
                iface: self.iface.clone(),
            },
            ep,
            cfg: cfg.clone(),
            plan,
            acc: Accounting::new(FIRST_DROP_BYTES, limit_bytes),
            to_submit: limit_bytes.map(|b| b + FIRST_DROP_BYTES as u64),
            started: false,
            rearmed: false,
            t_run: Instant::now(),
            t_data: Instant::now(),
            finished: false,
            stopped: false,
            report: StreamReport::default(),
        };
        s.report.note = Some(format!("xfer={}x{}", plan.xfer, plan.count));
        s.begin(false)?;
        Ok(Box::new(s))
    }
}

struct StockStream {
    regs: UsbRegs,
    ep: Endpoint<Bulk, In>,
    cfg: CaptureConfig,
    plan: RingPlan,
    acc: Accounting,
    /// Raw bytes still to be put into transfers (finite captures); `None` keeps the ring full forever.
    to_submit: Option<u64>,
    started: bool,
    rearmed: bool,
    t_run: Instant,
    t_data: Instant,
    /// The requested samples were delivered; the next read reports `Ended`.
    finished: bool,
    stopped: bool,
    report: StreamReport,
}

impl StockStream {
    /// stop -> (reset) -> clear halt -> configure -> fill the ring -> RUN.
    fn begin(&mut self, with_reset: bool) -> Result<(), DeviceError> {
        self.regs.write32(R_CTRL, 0)?;
        if with_reset {
            reset(&mut self.regs)?;
        }
        self.ep
            .clear_halt()
            .wait()
            .map_err(|e| usb_err("clearing endpoint halt", e))?;
        configure(&mut self.regs, &self.cfg)?;
        self.fill_ring();
        self.regs.write32(R_CTRL, CTRL_RUN)?;
        self.t_run = Instant::now();
        self.t_data = self.t_run;
        Ok(())
    }

    fn fill_ring(&mut self) {
        while self.ep.pending() < self.plan.count && self.to_submit.is_none_or(|n| n > 0) {
            self.ep.submit(self.ep.allocate(self.plan.xfer));
            if let Some(n) = &mut self.to_submit {
                *n = n.saturating_sub(self.plan.xfer as u64);
            }
        }
    }

    /// Nothing arrived after RUN: take the ring back and issue the start sequence once more.
    fn rearm(&mut self) -> Result<(), DeviceError> {
        self.rearmed = true;
        self.ep.cancel_all();
        while self.ep.pending() > 0 {
            let _ = self.ep.wait_next_complete(Duration::from_secs(1));
        }
        self.acc = Accounting::new(
            FIRST_DROP_BYTES,
            self.cfg
                .limit_samples
                .map(|n| self.cfg.channels.bytes_for(n)),
        );
        self.to_submit = self
            .cfg
            .limit_samples
            .map(|n| self.cfg.channels.bytes_for(n) + FIRST_DROP_BYTES as u64);
        self.begin(true)
    }
}

impl Stream for StockStream {
    fn read(&mut self, timeout: Duration) -> Result<ReadResult, DeviceError> {
        if self.finished || self.stopped {
            return Ok(ReadResult::Ended);
        }
        if self.ep.pending() == 0 {
            self.fill_ring();
            if self.ep.pending() == 0 {
                self.finished = true;
                return Ok(ReadResult::Ended);
            }
        }
        let Some(done) = self.ep.wait_next_complete(timeout) else {
            if self.started && self.t_data.elapsed() > IDLE_LIMIT {
                return Err(DeviceError::Stalled("no data for 1 s".into()));
            }
            let fill = self.plan.xfer as f64 * self.plan.count as f64;
            let rate = (self.cfg.sample_rate * self.cfg.channels.get() as f64 / 8.0).max(1.0);
            let grace = Duration::from_secs_f64((fill / rate * 4.0).max(1.5));
            if !self.started && self.t_run.elapsed() > grace {
                if self.rearmed {
                    return Err(DeviceError::Stalled(
                        "no data after RUN (start sequence repeated once)".into(),
                    ));
                }
                self.rearm()?;
            }
            return Ok(ReadResult::Timeout);
        };
        let data = done.buffer[..done.actual_len.min(done.buffer.len())].to_vec();
        let status = done.status;
        // Resubmit first so the bulk pipe never idles on host work.
        let mut buf = done.buffer;
        buf.clear();
        buf.set_requested_len(self.plan.xfer);
        if self.to_submit.is_none_or(|n| n > 0) {
            self.ep.submit(buf);
            if let Some(n) = &mut self.to_submit {
                *n = n.saturating_sub(self.plan.xfer as u64);
            }
        }
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
            // STOP first, while the ring still drains the device; retried.
            let mut last = Ok(());
            for _ in 0..10 {
                last = self.regs.write32(R_CTRL, 0);
                if last.is_ok() {
                    break;
                }
            }
            self.ep.cancel_all();
            for _ in 0..5 {
                if self.ep.pending() == 0 {
                    break;
                }
                let _ = self.ep.wait_next_complete(Duration::from_secs(1));
            }
            last?;
        }
        Ok(self.report.clone())
    }
}

impl Drop for StockStream {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bf_core::ChannelCount;

    /// A register file with the AUX mailbox behaviour of the specification.
    struct Fake {
        ops: Vec<(bool, u16, u32)>,
        cmd: u32,
        payload: [u32; 4],
        /// Base clock table (MHz) selected by `base_idx`.
        bases: Vec<u32>,
    }

    impl Fake {
        fn new() -> Self {
            Fake {
                ops: Vec::new(),
                cmd: 0,
                payload: [0; 4],
                bases: vec![250, 800],
            }
        }

        fn sync_rate(&mut self) {
            let idx = (self.payload[0] & 0xFFFF) as usize;
            self.payload[0] =
                (self.payload[0] & 0xFFFF) | (self.bases.get(idx).copied().unwrap_or(0) << 16);
        }
    }

    impl RegIo for Fake {
        fn read32(&mut self, addr: u16) -> Result<u32, DeviceError> {
            let v = match addr {
                R_AUX => {
                    let words = if self.cmd == AUX_RATE { 2 } else { 1 };
                    (1 << 16) | (words << 9)
                }
                a if (R_AUX_PAYLOAD..R_AUX_PAYLOAD + 16).contains(&a) => {
                    self.payload[((a - R_AUX_PAYLOAD) / 4) as usize]
                }
                _ => 0,
            };
            self.ops.push((false, addr, v));
            Ok(v)
        }

        fn write32(&mut self, addr: u16, value: u32) -> Result<(), DeviceError> {
            self.ops.push((true, addr, value));
            match addr {
                R_AUX => {
                    self.cmd = value;
                    if value == AUX_RATE {
                        self.payload = [0, 0, 0, 0];
                        self.sync_rate();
                    }
                }
                a if (R_AUX_PAYLOAD..R_AUX_PAYLOAD + 16).contains(&a) => {
                    self.payload[((a - R_AUX_PAYLOAD) / 4) as usize] = value;
                    if self.cmd == AUX_RATE && a == R_AUX_PAYLOAD {
                        self.sync_rate();
                    }
                }
                _ => {}
            }
            Ok(())
        }
    }

    fn cfg(ch: u8, rate: f64, pattern: Pattern) -> CaptureConfig {
        CaptureConfig {
            channels: ChannelCount::new(ch).unwrap(),
            sample_rate: rate,
            limit_samples: None,
            pattern,
            threshold_v: 1.7,
        }
    }

    /// Writes only, as (register, value).
    fn writes(f: &Fake) -> Vec<(u16, u32)> {
        f.ops.iter().filter(|o| o.0).map(|o| (o.1, o.2)).collect()
    }

    #[test]
    fn configure_matches_the_canonical_control_vector() {
        // Sipeed's golden sequence for {16 ch, 200 MHz, Emulation, 1.7 V}: STOP, then four AUX blocks
        // (channel, samplerate, vref, test mode), then RUN. Values from slogic_control_vectors.py.
        let mut f = Fake::new();
        f.write32(R_CTRL, 0).unwrap();
        configure(&mut f, &cfg(16, 200e6, Pattern::Emulation)).unwrap();
        f.write32(R_CTRL, CTRL_RUN).unwrap();
        let w = writes(&f);
        assert_eq!(
            w,
            vec![
                (R_CTRL, 0),
                (R_AUX, AUX_CHANNEL),
                (R_AUX_PAYLOAD, 0x0000_FFFF),
                (R_AUX, AUX_RATE),
                (R_AUX_PAYLOAD, 250 << 16 | 1), // base index 0 -> 1 (the 250 MHz base is not a multiple of 200 MHz)
                (R_AUX_PAYLOAD + 4, 3),         // 800 MHz / 200 MHz - 1
                (R_AUX, AUX_VREF),
                (R_AUX_PAYLOAD, 261),
                (R_AUX, AUX_TEST),
                (R_AUX_PAYLOAD, 2),
                (R_CTRL, CTRL_RUN),
            ]
        );
        // No RST on the capture path.
        assert!(!w.iter().any(|&(a, v)| a == R_CTRL && v == CTRL_RST));
    }

    #[test]
    fn every_aux_block_polls_the_header_and_confirms() {
        let mut f = Fake::new();
        set_channels(&mut f, 8).unwrap();
        // write cmd, poll header, read old payload, write payload, confirm read.
        let kinds: Vec<(bool, u16)> = f.ops.iter().map(|o| (o.0, o.1)).collect();
        assert_eq!(
            kinds,
            vec![
                (true, R_AUX),
                (false, R_AUX),
                (false, R_AUX_PAYLOAD),
                (true, R_AUX_PAYLOAD),
                (false, R_AUX_PAYLOAD)
            ]
        );
        assert_eq!(f.payload[0], 0xFF);
    }

    #[test]
    fn normal_pattern_issues_a_reset() {
        let mut f = Fake::new();
        set_pattern(&mut f, Pattern::Inputs).unwrap();
        let w = writes(&f);
        assert_eq!(&w[w.len() - 2..], &[(R_CTRL, CTRL_RST), (R_CTRL, 0)]);
        let mut f = Fake::new();
        set_pattern(&mut f, Pattern::Counter).unwrap();
        assert!(!writes(&f).iter().any(|&(a, _)| a == R_CTRL));
    }

    #[test]
    fn unreachable_samplerate_is_an_error_not_a_hang() {
        let mut f = Fake::new();
        assert!(set_samplerate(&mut f, 3e6).is_err()); // neither 250 nor 800 MHz is a multiple of 3 MHz
        let mut f = Fake::new();
        f.cmd = 0;
        assert!(set_samplerate(&mut f, 0.0).is_err());
    }

    #[test]
    fn mailbox_that_never_gets_ready_fails() {
        struct Dead;
        impl RegIo for Dead {
            fn read32(&mut self, _: u16) -> Result<u32, DeviceError> {
                Ok(0)
            }
            fn write32(&mut self, _: u16, _: u32) -> Result<(), DeviceError> {
                Ok(())
            }
        }
        assert!(set_channels(&mut Dead, 16).is_err());
    }

    #[test]
    fn dac_and_pattern_codes() {
        assert_eq!(vref_code(1.7), 261);
        assert_eq!(vref_code(0.0), 0);
        assert_eq!(vref_code(10.0), 1023);
        assert_eq!(pattern_code(Pattern::Inputs).unwrap(), 0);
        assert_eq!(pattern_code(Pattern::Counter).unwrap(), 1);
        assert_eq!(pattern_code(Pattern::Emulation).unwrap(), 2);
        assert!(pattern_code(Pattern::Generator).is_err());
    }

    #[test]
    fn ring_plan_follows_the_spec() {
        // 16 ch @ 200 MHz = 400 MB/s: 5 ms = 2 MB (62 x 32 KiB), within the 2 MiB cap; 4+ fit the usbfs budget.
        let p = plan_ring(200e6, 16, None);
        assert_eq!(p.xfer, 62 * 32 * KIB);
        assert!(p.xfer <= 2 * MIB && p.count >= 4 && p.count * p.xfer <= 16 * MIB);
        // Above 2 MiB worth of 5 ms the size is capped.
        assert_eq!(plan_ring(800e6, 16, None).xfer, 2 * MIB);
        // Slow captures: the 32 KiB floor, a full ring of 16.
        assert_eq!(
            plan_ring(1e6, 8, None),
            RingPlan {
                xfer: 32 * KIB,
                count: 16
            }
        );
        // Always 32 KiB aligned and at least 4 in flight for unlimited captures.
        for (r, c) in [(10e6, 16), (100e6, 16), (800e6, 4), (50e6, 8)] {
            let p = plan_ring(r, c, None);
            assert_eq!(p.xfer % (32 * KIB), 0);
            assert!(p.count >= 4);
        }
        // A tiny finite capture needs one small transfer.
        let p = plan_ring(10e6, 16, Some(2000));
        assert_eq!(
            p,
            RingPlan {
                xfer: 2048,
                count: 1
            }
        );
        // A finite capture never keeps more transfers than it needs.
        let p = plan_ring(10e6, 16, Some(100_000));
        assert!(p.count * p.xfer >= 100_004 && p.count * p.xfer < 100_004 + p.xfer);
    }

    #[test]
    fn accounting_drops_the_head_across_small_transfers() {
        let mut a = Accounting::new(4, None);
        assert_eq!(a.feed(&[1, 2]).0, &[] as &[u8]);
        assert_eq!(a.feed(&[3, 4, 5, 6]).0, &[5, 6]);
        assert_eq!(a.feed(&[7]).0, &[7]);
    }

    #[test]
    fn accounting_delivers_exactly_the_limit() {
        let mut a = Accounting::new(4, Some(6));
        let (d, done) = a.feed(&[0, 0, 0, 0, 1, 2, 3]);
        assert_eq!((d, done), (&[1u8, 2, 3][..], false));
        let (d, done) = a.feed(&[4, 5, 6, 7, 8]);
        assert_eq!((d, done), (&[4u8, 5, 6][..], true));
    }

    #[test]
    fn usb2_link_caps_the_rate() {
        let c = capabilities(false);
        assert_eq!(c.max_rate(16), Some(20e6));
        assert_eq!(c.max_rate(8), Some(40e6));
        assert_eq!(c.max_rate(4), Some(80e6));
        let c = capabilities(true);
        assert_eq!(c.max_rate(16), Some(200e6));
        assert_eq!(c.max_rate(4), Some(800e6));
    }
}
