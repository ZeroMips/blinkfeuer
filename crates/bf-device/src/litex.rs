//! Sipeed SLogic16U3 running the open LiteX gateware (USB `1d50:605a`, product "SLogic16U3 LiteX").
//!
//! **Not yet run against the hardware**: everything that does not need a USB device is covered by
//! tests on a fake register file, the USB parts follow the transport described below and the
//! backend of the stock firmware.
//!
//! Reimplemented from the register map and the transport of the gateware's own library, libslogic16u3
//! (BSD-2-Clause, Copyright (c) 2026 Enjoy-Digital), which is the authority on the protocol:
//!
//! * **EP0** carries accesses to the SoC bus (LiteX "CommUSB"): `bmRequestType` `0xC3` (read) or `0x43`
//!   (write) with `bRequest` 0, the byte address split into `wValue` (low 16 bits) and `wIndex` (high
//!   16 bits), up to 128 little-endian 32-bit words per transfer. 64-bit registers are two words,
//!   most significant first.
//! * **EP 0x81 (bulk IN)** carries the packed samples, in the same wire format as the stock firmware
//!   (sample-major, first sample in the low bits) but without a dropped head. A transfer shorter than
//!   requested ends the capture; the last word is padded to 8 bytes.
//! * Start sequence: stopped control word, divider, limit, clear halt and queue the transfers, then
//!   set ENABLE. Stop: clear ENABLE.
//!
//! Open points the code is tolerant of rather than hides:
//!
//! * While the bulk pipe streams, EP0 may not answer, so no register is read during a capture; the
//!   stop request is sent after the transfers are cancelled, with bounded retries, and a missing
//!   answer is reported in the stream report instead of hanging.
//! * The gateware can report a FIFO overflow at the end of a capture (see the upstream issue about large
//!   captures); that is surfaced as [`DeviceError::Overflow`] after the data that did arrive. The transfer
//!   geometry can be changed with `BLINKFEUER_LITEX_XFER_KIB` and `BLINKFEUER_LITEX_XFERS`.

use crate::stock::{drain, plan_ring_with, transfer_err, usb_err, Accounting, RingPlan};
use crate::{
    Capabilities, CaptureConfig, ChannelMode, Device, DeviceError, DeviceInfo, Pattern, ReadResult,
    Stream, StreamReport,
};
use nusb::transfer::{Bulk, ControlIn, ControlOut, ControlType, In, Recipient};
use nusb::{Interface, MaybeFuture, Speed};
use std::time::{Duration, Instant};

/// USB vendor id.
pub const VID: u16 = 0x1D50;
/// USB product id.
pub const PID: u16 = 0x605A;
/// Other devices of the USB3-PIPE project may share the vendor and product id: the product string tells.
pub const PRODUCT: &str = "SLogic16U3 LiteX";
const EP_IN: u8 = 0x81;
const MAX_WORDS: usize = 128;
const CONTROL_TIMEOUT: Duration = Duration::from_secs(1);
const IDLE_LIMIT: Duration = Duration::from_secs(1);

/// Register addresses (bytes), the fixed CSR map of the gateware.
pub mod reg {
    /// Reads [`MAGIC`]; version, configuration and base rate follow.
    pub const ANALYZER_MAGIC: u32 = 0x0800;
    /// Enable bit, channel mode and pattern.
    pub const ANALYZER_CONTROL: u32 = 0x0810;
    /// Divider of the base rate.
    pub const ANALYZER_DIVIDER: u32 = 0x0814;
    /// 64 bit.
    pub const ANALYZER_LIMIT: u32 = 0x0818;
    /// The `STATUS_*` bits.
    pub const ANALYZER_STATUS: u32 = 0x0820;
    /// 64 bit.
    pub const ANALYZER_COUNT: u32 = 0x0824;
    /// Identification text, one character per word.
    pub const IDENTIFIER_BASE: u32 = 0x2000;

    /// "SLOG".
    pub const MAGIC: u32 = 0x534c_4f47;

    /// Control: capture.
    pub const CONTROL_ENABLE: u32 = 1 << 0;
    /// Control: the channel mode field (see [`mode_code`](super::mode_code)).
    pub const fn control_mode(m: u32) -> u32 {
        (m & 0x3) << 4
    }
    /// Control: the pattern field (see [`pattern_code`](super::pattern_code)).
    pub const fn control_pattern(p: u32) -> u32 {
        (p & 0x3) << 8
    }
    /// Status: capturing.
    pub const STATUS_RUNNING: u32 = 1 << 0;
    /// Status: the capture is complete.
    pub const STATUS_DONE: u32 = 1 << 1;
    /// Status: samples were lost.
    pub const STATUS_OVERFLOW: u32 = 1 << 2;
    /// Status: error.
    pub const STATUS_ERROR: u32 = 1 << 3;
}

// ------------------------------------------------------------------------------------------------
// The bus.

/// Access to the SoC bus of the gateware.
pub trait Bus {
    /// Read consecutive words from byte address `addr`.
    fn read_burst(&mut self, addr: u32, out: &mut [u32]) -> Result<(), DeviceError>;
    /// Write consecutive words from byte address `addr`.
    fn write_burst(&mut self, addr: u32, vals: &[u32]) -> Result<(), DeviceError>;

    /// Read one word.
    fn read32(&mut self, addr: u32) -> Result<u32, DeviceError> {
        let mut v = [0u32];
        self.read_burst(addr, &mut v)?;
        Ok(v[0])
    }

    /// Write one word.
    fn write32(&mut self, addr: u32, value: u32) -> Result<(), DeviceError> {
        self.write_burst(addr, &[value])
    }

    /// Most significant word first.
    fn read64(&mut self, addr: u32) -> Result<u64, DeviceError> {
        let mut v = [0u32; 2];
        self.read_burst(addr, &mut v)?;
        Ok((v[0] as u64) << 32 | v[1] as u64)
    }

    /// Most significant word first.
    fn write64(&mut self, addr: u32, value: u64) -> Result<(), DeviceError> {
        self.write_burst(addr, &[(value >> 32) as u32, value as u32])
    }
}

/// One EP0 transfer of at most `MAX_WORDS` words.
pub trait Ep0 {
    /// Read `len` bytes (`value` and `index`: see [`control_fields`]).
    fn read(&mut self, value: u16, index: u16, len: usize) -> Result<Vec<u8>, DeviceError>;
    /// Write `data` (`value` and `index`: see [`control_fields`]).
    fn write(&mut self, value: u16, index: u16, data: &[u8]) -> Result<(), DeviceError>;
}

/// `wValue` and `wIndex` of an access at byte address `addr`.
pub fn control_fields(addr: u32) -> (u16, u16) {
    ((addr & 0xffff) as u16, (addr >> 16) as u16)
}

/// Splits bursts into EP0 transfers and converts words from and to bytes.
pub struct Ep0Bus<T: Ep0>(pub T);

impl<T: Ep0> Bus for Ep0Bus<T> {
    fn read_burst(&mut self, mut addr: u32, out: &mut [u32]) -> Result<(), DeviceError> {
        for chunk in out.chunks_mut(MAX_WORDS) {
            let (value, index) = control_fields(addr);
            let data = self.0.read(value, index, 4 * chunk.len())?;
            if data.len() != 4 * chunk.len() {
                return Err(DeviceError::Usb(format!(
                    "register {addr:#x}: short read ({} of {} bytes)",
                    data.len(),
                    4 * chunk.len()
                )));
            }
            for (w, b) in chunk.iter_mut().zip(data.as_chunks::<4>().0) {
                *w = u32::from_le_bytes(*b);
            }
            addr += 4 * chunk.len() as u32;
        }
        Ok(())
    }

    fn write_burst(&mut self, mut addr: u32, vals: &[u32]) -> Result<(), DeviceError> {
        for chunk in vals.chunks(MAX_WORDS) {
            let (value, index) = control_fields(addr);
            let bytes: Vec<u8> = chunk.iter().flat_map(|w| w.to_le_bytes()).collect();
            self.0.write(value, index, &bytes)?;
            addr += 4 * chunk.len() as u32;
        }
        Ok(())
    }
}

struct UsbEp0 {
    iface: Interface,
}

impl Ep0 for UsbEp0 {
    fn read(&mut self, value: u16, index: u16, len: usize) -> Result<Vec<u8>, DeviceError> {
        self.iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Other,
                    request: 0,
                    value,
                    index,
                    length: len as u16,
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(|e| transfer_err(&e))
    }

    fn write(&mut self, value: u16, index: u16, data: &[u8]) -> Result<(), DeviceError> {
        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Other,
                    request: 0,
                    value,
                    index,
                    data,
                },
                CONTROL_TIMEOUT,
            )
            .wait()
            .map_err(|e| transfer_err(&e))
    }
}

// ------------------------------------------------------------------------------------------------
// What the gateware says about itself, and the configuration derived from it.

/// What the gateware reports about itself.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Info {
    /// Version of the register interface (major in the upper 16 bits).
    pub version: u32,
    /// Channels the gateware samples.
    pub nchannels: u32,
    /// Samples per sample-clock cycle at the base rate (8 for 800 MS/s on a 100 MHz cycle).
    pub ratio: u32,
    /// Capture FIFO depth in 64-bit words.
    pub fifo_words: u32,
    /// Sample rate before the divider, in Hz.
    pub base_rate: u32,
    /// Identification text.
    pub ident: String,
}

/// Read and check the identification registers.
pub fn read_info<B: Bus>(bus: &mut B) -> Result<Info, DeviceError> {
    let mut regs = [0u32; 4];
    bus.read_burst(reg::ANALYZER_MAGIC, &mut regs)?;
    let [magic, version, config, base_rate] = regs;
    if magic != reg::MAGIC {
        return Err(DeviceError::Other(format!(
            "not the SLogic16U3 gateware: register magic is {magic:#010x}, expected {:#010x}",
            reg::MAGIC
        )));
    }
    if version >> 16 != 0 {
        return Err(DeviceError::Other(format!(
            "gateware version {version:#x} is newer than this program understands"
        )));
    }
    let ratio = (config >> 8) & 0xff;
    if ratio == 0 || base_rate == 0 {
        return Err(DeviceError::Other(
            "the gateware reports no sample rate".into(),
        ));
    }
    let mut ident = [0u32; 64];
    bus.read_burst(reg::IDENTIFIER_BASE, &mut ident)?;
    let ident: String = ident
        .iter()
        .map(|w| (*w & 0xff) as u8)
        .take_while(|c| *c != 0)
        .map(char::from)
        .collect();
    Ok(Info {
        version,
        nchannels: config & 0xff,
        ratio,
        fifo_words: ((config >> 16) & 0xffff) * 64,
        base_rate,
        ident,
    })
}

/// The gateware turns up to 32 bits into one word per sample-clock cycle: `ceil(ratio/divider)`
/// samples per cycle, so the product with the channel count must not exceed 32.
fn divider_ok(info: &Info, channels: u8, divider: u64) -> bool {
    divider >= 1
        && divider <= u32::MAX as u64
        && (info.ratio as u64).div_ceil(divider) * channels as u64 <= 32
}

/// Highest sample rate for `channels` channels (Hz), 0 if none.
pub fn max_samplerate(info: &Info, channels: u8) -> u64 {
    (1..=info.ratio as u64 * 4)
        .find(|&d| divider_ok(info, channels, d) && (info.base_rate as u64).is_multiple_of(d))
        .map_or(0, |d| info.base_rate as u64 / d)
}

/// Sample rates worth offering: the usual 1-2-2.5-4-5-8 steps that divide the base rate, plus the
/// highest few that are `base / small divider`.
pub fn samplerates(info: &Info, channels: u8) -> Vec<f64> {
    const STEPS: [u64; 6] = [10, 20, 25, 40, 50, 80];
    let base = info.base_rate as u64;
    let max = max_samplerate(info, channels);
    let mut rates: Vec<u64> = Vec::new();
    let mut decade = 100u64;
    while decade <= base {
        for s in STEPS {
            let r = decade * s;
            if r <= max && base.is_multiple_of(r) && r <= base {
                rates.push(r);
            }
        }
        decade *= 10;
    }
    for d in 1..=8u64 {
        let r = base / d;
        if base.is_multiple_of(d) && r <= max {
            rates.push(r);
        }
    }
    rates.sort_unstable();
    rates.dedup();
    rates.into_iter().map(|r| r as f64).collect()
}

/// What a gateware with this identification can do.
pub fn capabilities(info: &Info) -> Capabilities {
    let modes: Vec<ChannelMode> = [4u8, 8, 16]
        .into_iter()
        .filter(|&c| c as u32 <= info.nchannels.max(16))
        .map(|c| ChannelMode {
            channels: c,
            max_rate: max_samplerate(info, c) as f64,
        })
        .filter(|m| m.max_rate > 0.0)
        .collect();
    let rates = samplerates(info, 4);
    Capabilities {
        modes,
        rates,
        patterns: vec![Pattern::Inputs, Pattern::Counter, Pattern::Generator],
        threshold: None,
    }
}

/// The divider for `cfg`, or why the rate cannot be had.
pub fn divider_for(info: &Info, cfg: &CaptureConfig) -> Result<u32, DeviceError> {
    let rate = cfg.sample_rate.round() as u64;
    let base = info.base_rate as u64;
    if rate == 0 || !base.is_multiple_of(rate) {
        return Err(DeviceError::Config(format!(
            "a sample rate of {rate} Hz is not {} MHz / N",
            base as f64 / 1e6
        )));
    }
    let divider = base / rate;
    if !divider_ok(info, cfg.channels.get(), divider) {
        return Err(DeviceError::Config(format!(
            "{} Hz is too fast for {} channels (at most {} MHz)",
            rate,
            cfg.channels.get(),
            max_samplerate(info, cfg.channels.get()) as f64 / 1e6
        )));
    }
    Ok(divider as u32)
}

/// Register code of a channel count.
pub fn mode_code(channels: u8) -> Result<u32, DeviceError> {
    match channels {
        4 => Ok(0),
        8 => Ok(1),
        16 => Ok(2),
        n => Err(DeviceError::Config(format!(
            "{n} channels are not supported (4, 8 or 16)"
        ))),
    }
}

/// Register code of a pattern.
pub fn pattern_code(p: Pattern) -> Result<u32, DeviceError> {
    match p {
        Pattern::Inputs => Ok(0),
        Pattern::Counter => Ok(1),
        Pattern::Generator => Ok(2),
        Pattern::Emulation => Err(DeviceError::Config(
            "the Emulation pattern exists only in the stock firmware".into(),
        )),
    }
}

/// Write the control word (without ENABLE), the divider and the limit. Returns the control word.
pub fn configure<B: Bus>(
    bus: &mut B,
    info: &Info,
    cfg: &CaptureConfig,
) -> Result<u32, DeviceError> {
    let divider = divider_for(info, cfg)?;
    let control = reg::control_mode(mode_code(cfg.channels.get())?)
        | reg::control_pattern(pattern_code(cfg.pattern)?);
    bus.write32(reg::ANALYZER_CONTROL, control)?;
    bus.write32(reg::ANALYZER_DIVIDER, divider)?;
    bus.write64(reg::ANALYZER_LIMIT, cfg.limit_samples.unwrap_or(0))?;
    Ok(control)
}

/// State of the capture.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Status {
    /// Capturing.
    pub running: bool,
    /// The capture is complete.
    pub done: bool,
    /// Samples were lost.
    pub overflow: bool,
    /// The gateware reports an error.
    pub error: bool,
    /// Samples the gateware has captured.
    pub samples: u64,
}

/// Read the state of the capture.
pub fn status<B: Bus>(bus: &mut B) -> Result<Status, DeviceError> {
    let s = bus.read32(reg::ANALYZER_STATUS)?;
    let samples = bus.read64(reg::ANALYZER_COUNT)?;
    Ok(Status {
        running: s & reg::STATUS_RUNNING != 0,
        done: s & reg::STATUS_DONE != 0,
        overflow: s & reg::STATUS_OVERFLOW != 0,
        error: s & reg::STATUS_ERROR != 0,
        samples,
    })
}

/// Clear ENABLE, keeping the rest of the control word.
pub fn disable<B: Bus>(bus: &mut B) -> Result<(), DeviceError> {
    let control = bus.read32(reg::ANALYZER_CONTROL)?;
    bus.write32(reg::ANALYZER_CONTROL, control & !reg::CONTROL_ENABLE)
}

/// Transfer geometry: about 5 ms per transfer like the stock backend, unless overridden for
/// experiments (the gateware has been seen to overflow at transfer boundaries on some hosts).
pub fn plan(rate: f64, channels: u8, limit_bytes: Option<u64>) -> RingPlan {
    let mut p = plan_ring_with(rate, channels, limit_bytes, 0);
    let env = |name: &str| {
        std::env::var(name)
            .ok()
            .and_then(|v| v.trim().parse::<usize>().ok())
            .filter(|v| *v > 0)
    };
    if let Some(kib) = env("BLINKFEUER_LITEX_XFER_KIB") {
        p.xfer = kib.div_ceil(32) * 32 * 1024;
    }
    if let Some(n) = env("BLINKFEUER_LITEX_XFERS") {
        p.count = n.min(64);
    }
    p
}

// ------------------------------------------------------------------------------------------------
// The device.

/// An SLogic16U3 with the LiteX gateware.
pub struct LitexDevice {
    iface: Interface,
    info: DeviceInfo,
    gateware: Info,
    caps: Capabilities,
}

fn describe(d: &nusb::DeviceInfo) -> DeviceInfo {
    let speed = match d.speed() {
        Some(Speed::Super) => "5G",
        Some(Speed::SuperPlus) => "10G",
        Some(Speed::High) => "480M",
        _ => "?",
    };
    DeviceInfo {
        id: format!("litex16u3:{}", d.serial_number().unwrap_or("-")),
        name: format!("({speed}) {PRODUCT}"),
        backend: "litex",
    }
}

fn is_ours(d: &nusb::DeviceInfo) -> bool {
    d.vendor_id() == VID && d.product_id() == PID && d.product_string().is_none_or(|p| p == PRODUCT)
}

impl LitexDevice {
    /// Descriptions of the attached analyzers running the gateware (without opening them).
    pub fn available() -> Vec<DeviceInfo> {
        let Ok(list) = nusb::list_devices().wait() else {
            return Vec::new();
        };
        list.filter(is_ours).map(|d| describe(&d)).collect()
    }

    /// Open the first one, read what the gateware says about itself and bring it to idle.
    pub fn open_first() -> Result<Self, DeviceError> {
        let di = nusb::list_devices()
            .wait()
            .map_err(|e| usb_err("listing USB devices", e))?
            .find(is_ours)
            .ok_or_else(|| {
                DeviceError::Other("no SLogic16U3 with the LiteX gateware (1d50:605a) found".into())
            })?;
        let info = describe(&di);
        let dev = di
            .open()
            .wait()
            .map_err(|e| usb_err("opening the device", e))?;
        let iface = dev
            .claim_interface(0)
            .wait()
            .map_err(|e| usb_err("claiming interface 0", e))?;
        let mut bus = Ep0Bus(UsbEp0 {
            iface: iface.clone(),
        });
        let gateware = read_info(&mut bus)?;
        // A capture left over from a killed program: stop it and throw its data away.
        let _ = disable(&mut bus);
        if let Ok(mut ep) = iface.endpoint::<Bulk, In>(EP_IN) {
            drain(&mut ep);
        }
        let caps = capabilities(&gateware);
        Ok(Self {
            iface,
            info,
            gateware,
            caps,
        })
    }

    /// What the gateware reported.
    pub fn gateware(&self) -> &Info {
        &self.gateware
    }
}

impl Device for LitexDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn capabilities(&self) -> &Capabilities {
        &self.caps
    }

    fn start(&mut self, cfg: &CaptureConfig) -> Result<Box<dyn Stream>, DeviceError> {
        self.caps.validate(cfg)?;
        divider_for(&self.gateware, cfg)?;
        let ep = self
            .iface
            .endpoint::<Bulk, In>(EP_IN)
            .map_err(|e| usb_err("opening bulk IN endpoint 0x81", e))?;
        let limit_bytes = cfg.limit_samples.map(|n| cfg.channels.bytes_for(n));
        let plan = plan(cfg.sample_rate, cfg.channels.get(), limit_bytes);
        let mut s = LitexStream {
            bus: Ep0Bus(UsbEp0 {
                iface: self.iface.clone(),
            }),
            gateware: self.gateware.clone(),
            ep,
            cfg: cfg.clone(),
            plan,
            acc: Accounting::new(0, limit_bytes),
            // The gateware pads the last word to 8 bytes; the padding is asked for but never delivered.
            to_submit: limit_bytes.map(|b| b.div_ceil(8) * 8),
            started: false,
            t_run: Instant::now(),
            t_data: Instant::now(),
            device_ended: false,
            finished: false,
            stopped: false,
            report: StreamReport::default(),
        };
        s.report.note = Some(format!("xfer={}x{}", plan.xfer, plan.count));
        s.begin()?;
        Ok(Box::new(s))
    }
}

struct LitexStream {
    bus: Ep0Bus<UsbEp0>,
    gateware: Info,
    ep: nusb::Endpoint<Bulk, In>,
    cfg: CaptureConfig,
    plan: RingPlan,
    acc: Accounting,
    to_submit: Option<u64>,
    started: bool,
    t_run: Instant,
    t_data: Instant,
    /// A short transfer arrived: the gateware has nothing more to send.
    device_ended: bool,
    finished: bool,
    stopped: bool,
    report: StreamReport,
}

impl LitexStream {
    /// configure -> clear halt -> queue the transfers -> ENABLE.
    fn begin(&mut self) -> Result<(), DeviceError> {
        let control = configure(&mut self.bus, &self.gateware, &self.cfg)?;
        self.ep
            .clear_halt()
            .wait()
            .map_err(|e| usb_err("clearing endpoint halt", e))?;
        self.fill_ring();
        self.bus
            .write32(reg::ANALYZER_CONTROL, control | reg::CONTROL_ENABLE)?;
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

    /// After the stream ended by itself: did the gateware lose data? (EP0 answers again now.)
    fn check_end(&mut self) -> Result<ReadResult, DeviceError> {
        self.finished = true;
        match status(&mut self.bus) {
            Ok(st) if st.overflow || st.error => {
                self.report.overflow = st.overflow;
                Err(if st.overflow {
                    DeviceError::Overflow
                } else {
                    DeviceError::Other("the gateware reports a capture error".into())
                })
            }
            Ok(_) => Ok(ReadResult::Ended),
            Err(e) => {
                self.report.note = Some(format!(
                    "{}; status unreadable: {e}",
                    self.report.note.clone().unwrap_or_default()
                ));
                Ok(ReadResult::Ended)
            }
        }
    }
}

impl Stream for LitexStream {
    fn read(&mut self, timeout: Duration) -> Result<ReadResult, DeviceError> {
        if self.stopped || (self.finished && !self.device_ended) {
            return Ok(ReadResult::Ended);
        }
        if self.device_ended {
            return if self.finished {
                Ok(ReadResult::Ended)
            } else {
                self.check_end()
            };
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
            if !self.started && self.t_run.elapsed() > Duration::from_secs(3) {
                return Err(DeviceError::Stalled("no data 3 s after ENABLE".into()));
            }
            return Ok(ReadResult::Timeout);
        };
        let requested = done.buffer.requested_len();
        let data = done.buffer[..done.actual_len.min(done.buffer.len())].to_vec();
        let status = done.status;
        let mut buf = done.buffer;
        buf.clear();
        buf.set_requested_len(self.plan.xfer);
        let short = data.len() < requested;
        if !short && self.to_submit.is_none_or(|n| n > 0) {
            self.ep.submit(buf);
            if let Some(n) = &mut self.to_submit {
                *n = n.saturating_sub(self.plan.xfer as u64);
            }
        }
        status.map_err(|e| match e {
            nusb::transfer::TransferError::Cancelled => {
                DeviceError::Stalled("transfer cancelled".into())
            }
            other => transfer_err(&other),
        })?;
        if !data.is_empty() {
            self.started = true;
            self.t_data = Instant::now();
        }
        let (out, complete) = self.acc.feed(&data);
        self.report.bytes += out.len() as u64;
        if complete {
            self.finished = true;
        } else if short {
            self.device_ended = true;
        }
        if out.is_empty() {
            return if complete {
                Ok(ReadResult::Ended)
            } else if short {
                self.check_end()
            } else {
                Ok(ReadResult::Timeout)
            };
        }
        Ok(ReadResult::Data(out.to_vec()))
    }

    fn stop(&mut self) -> Result<StreamReport, DeviceError> {
        if !self.stopped {
            self.stopped = true;
            // The bulk pipe first (EP0 may not answer while it streams), then the stop request.
            self.ep.cancel_all();
            let t0 = Instant::now();
            while self.ep.pending() > 0 && t0.elapsed() < Duration::from_secs(2) {
                let _ = self.ep.wait_next_complete(Duration::from_millis(200));
            }
            let mut last = Err(DeviceError::Other("not tried".into()));
            for _ in 0..3 {
                last = disable(&mut self.bus);
                if last.is_ok() {
                    break;
                }
            }
            if let Err(e) = last {
                self.report.note = Some(format!(
                    "{}; the gateware did not answer the stop request ({e}); replug it if it misbehaves",
                    self.report.note.clone().unwrap_or_default()
                ));
            }
        }
        Ok(self.report.clone())
    }
}

impl Drop for LitexStream {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bf_core::ChannelCount;
    use std::collections::HashMap;

    /// A register file with a log of what was written.
    #[derive(Default)]
    struct Fake {
        regs: HashMap<u32, u32>,
        writes: Vec<(u32, u32)>,
        fail: bool,
    }

    impl Fake {
        /// A gateware as built for the SLogic16U3: 16 channels, ratio 8, 800 MS/s, 2048x64-bit FIFO.
        fn gateware() -> Self {
            let mut f = Fake::default();
            f.regs.insert(reg::ANALYZER_MAGIC, reg::MAGIC);
            f.regs.insert(reg::ANALYZER_MAGIC + 4, 0x0001);
            f.regs
                .insert(reg::ANALYZER_MAGIC + 8, 16 | 8 << 8 | 32 << 16);
            f.regs.insert(reg::ANALYZER_MAGIC + 12, 800_000_000);
            for (i, c) in b"SLogic16U3 LiteX\0".iter().enumerate() {
                f.regs
                    .insert(reg::IDENTIFIER_BASE + 4 * i as u32, *c as u32);
            }
            f
        }
    }

    impl Bus for Fake {
        fn read_burst(&mut self, addr: u32, out: &mut [u32]) -> Result<(), DeviceError> {
            if self.fail {
                return Err(DeviceError::Usb("timeout".into()));
            }
            for (i, w) in out.iter_mut().enumerate() {
                *w = self.regs.get(&(addr + 4 * i as u32)).copied().unwrap_or(0);
            }
            Ok(())
        }

        fn write_burst(&mut self, addr: u32, vals: &[u32]) -> Result<(), DeviceError> {
            if self.fail {
                return Err(DeviceError::Usb("timeout".into()));
            }
            for (i, v) in vals.iter().enumerate() {
                self.regs.insert(addr + 4 * i as u32, *v);
                self.writes.push((addr + 4 * i as u32, *v));
            }
            Ok(())
        }
    }

    fn cfg(ch: u8, rate: f64, pattern: Pattern, limit: Option<u64>) -> CaptureConfig {
        CaptureConfig {
            channels: ChannelCount::new(ch).unwrap(),
            sample_rate: rate,
            limit_samples: limit,
            pattern,
            threshold_v: 1.7,
        }
    }

    fn info() -> Info {
        read_info(&mut Fake::gateware()).unwrap()
    }

    #[test]
    fn identification_is_read_and_checked() {
        let i = info();
        assert_eq!(
            (i.nchannels, i.ratio, i.base_rate, i.fifo_words),
            (16, 8, 800_000_000, 2048)
        );
        assert_eq!(i.ident, "SLogic16U3 LiteX");
        // Something else answering at the same address, a newer gateware and a broken one are refused.
        let mut f = Fake::gateware();
        f.regs.insert(reg::ANALYZER_MAGIC, 0x1234_5678);
        assert!(
            matches!(read_info(&mut f), Err(DeviceError::Other(m)) if m.contains("not the SLogic16U3 gateware"))
        );
        let mut f = Fake::gateware();
        f.regs.insert(reg::ANALYZER_MAGIC + 4, 0x0002_0000);
        assert!(read_info(&mut f).is_err());
        let mut f = Fake::gateware();
        f.regs.insert(reg::ANALYZER_MAGIC + 8, 16 | 32 << 16);
        assert!(read_info(&mut f).is_err(), "ratio 0");
        let mut f = Fake::gateware();
        f.fail = true;
        assert!(read_info(&mut f).is_err());
    }

    #[test]
    fn rates_follow_the_datapath_limit_of_32_bits_per_cycle() {
        let i = info();
        assert_eq!(max_samplerate(&i, 4), 800_000_000);
        assert_eq!(max_samplerate(&i, 8), 400_000_000);
        assert_eq!(max_samplerate(&i, 16), 200_000_000);
        let r16 = samplerates(&i, 16);
        assert!(
            r16.contains(&100e6)
                && r16.contains(&200e6)
                && r16.contains(&160e6)
                && r16.contains(&1e6)
        );
        assert!(
            !r16.contains(&250e6) && !r16.contains(&400e6),
            "too fast for 16 channels"
        );
        assert!(r16.windows(2).all(|w| w[0] < w[1]), "ascending and unique");
        // Only divisors of the base rate are offered.
        assert!(r16.iter().all(|r| 800e6 % r == 0.0));
        assert!(samplerates(&i, 4).contains(&800e6));
        let caps = capabilities(&i);
        assert_eq!(caps.max_rate(16), Some(200e6));
        assert_eq!(caps.max_rate(8), Some(400e6));
        assert_eq!(caps.max_rate(4), Some(800e6));
        assert!(caps.threshold.is_none());
        assert!(
            caps.patterns.contains(&Pattern::Generator)
                && !caps.patterns.contains(&Pattern::Emulation)
        );
    }

    #[test]
    fn dividers_and_bad_rates() {
        let i = info();
        assert_eq!(
            divider_for(&i, &cfg(16, 100e6, Pattern::Inputs, None)).unwrap(),
            8
        );
        assert_eq!(
            divider_for(&i, &cfg(4, 800e6, Pattern::Inputs, None)).unwrap(),
            1
        );
        assert_eq!(
            divider_for(&i, &cfg(16, 160e6, Pattern::Inputs, None)).unwrap(),
            5
        );
        assert_eq!(
            divider_for(&i, &cfg(16, 5e6, Pattern::Inputs, None)).unwrap(),
            160
        );
        let e = divider_for(&i, &cfg(16, 123e6, Pattern::Inputs, None))
            .unwrap_err()
            .to_string();
        assert!(e.contains("not 800 MHz / N"), "{e}");
        let e = divider_for(&i, &cfg(16, 400e6, Pattern::Inputs, None))
            .unwrap_err()
            .to_string();
        assert!(e.contains("too fast for 16 channels"), "{e}");
        assert!(divider_for(&i, &cfg(8, 400e6, Pattern::Inputs, None)).is_ok());
    }

    #[test]
    fn configure_writes_control_divider_and_limit_in_order() {
        let i = info();
        let mut f = Fake::gateware();
        let control = configure(
            &mut f,
            &i,
            &cfg(16, 100e6, Pattern::Counter, Some(0x1_0000_0003)),
        )
        .unwrap();
        // 16 channels = mode 2 (bits 5:4), counter = pattern 1 (bits 9:8), not enabled.
        assert_eq!(control, 2 << 4 | 1 << 8);
        assert_eq!(
            f.writes,
            [
                (reg::ANALYZER_CONTROL, 0x120),
                (reg::ANALYZER_DIVIDER, 8),
                // The 64-bit limit: most significant word first.
                (reg::ANALYZER_LIMIT, 1),
                (reg::ANALYZER_LIMIT + 4, 3),
            ]
        );
        // Continuous: limit 0. Test pattern codes of the other modes.
        let mut f = Fake::gateware();
        configure(&mut f, &i, &cfg(4, 800e6, Pattern::Generator, None)).unwrap();
        // The register fields spelled out: 0 at bit 4, 2 at bit 8.
        #[allow(clippy::identity_op)]
        let control = 0 << 4 | 2 << 8;
        assert_eq!(f.writes[0], (reg::ANALYZER_CONTROL, control));
        assert_eq!(
            &f.writes[2..],
            &[(reg::ANALYZER_LIMIT, 0), (reg::ANALYZER_LIMIT + 4, 0)]
        );
        assert!(configure(
            &mut Fake::gateware(),
            &i,
            &cfg(16, 100e6, Pattern::Emulation, None)
        )
        .is_err());
        assert_eq!(mode_code(8).unwrap(), 1);
        assert!(mode_code(2).is_err());
    }

    #[test]
    fn status_and_disable() {
        let mut f = Fake::gateware();
        f.regs.insert(
            reg::ANALYZER_STATUS,
            reg::STATUS_DONE | reg::STATUS_OVERFLOW,
        );
        f.regs.insert(reg::ANALYZER_COUNT, 0x2);
        f.regs.insert(reg::ANALYZER_COUNT + 4, 0x10);
        let st = status(&mut f).unwrap();
        assert_eq!(
            st,
            Status {
                running: false,
                done: true,
                overflow: true,
                error: false,
                samples: 0x2_0000_0010
            }
        );
        // Disabling keeps the mode and pattern bits.
        f.regs.insert(reg::ANALYZER_CONTROL, 0x121);
        disable(&mut f).unwrap();
        assert_eq!(f.writes.last(), Some(&(reg::ANALYZER_CONTROL, 0x120)));
        f.fail = true;
        assert!(
            disable(&mut f).is_err(),
            "a device that does not answer is an error, not a hang"
        );
    }

    /// EP0 stand-in that records the transfers and answers with counting words.
    #[derive(Default)]
    struct FakeEp0 {
        reads: Vec<(u16, u16, usize)>,
        writes: Vec<(u16, u16, Vec<u8>)>,
        short: bool,
    }

    impl Ep0 for FakeEp0 {
        fn read(&mut self, value: u16, index: u16, len: usize) -> Result<Vec<u8>, DeviceError> {
            self.reads.push((value, index, len));
            Ok(vec![0xAB; if self.short { len - 1 } else { len }])
        }

        fn write(&mut self, value: u16, index: u16, data: &[u8]) -> Result<(), DeviceError> {
            self.writes.push((value, index, data.to_vec()));
            Ok(())
        }
    }

    #[test]
    fn addresses_are_split_into_value_and_index() {
        assert_eq!(control_fields(0x0810), (0x0810, 0));
        assert_eq!(control_fields(0x2000_0000), (0x0000, 0x2000));
        assert_eq!(control_fields(0x2000_01fc), (0x01fc, 0x2000));
    }

    #[test]
    fn bursts_are_cut_into_transfers_of_128_words() {
        let mut bus = Ep0Bus(FakeEp0::default());
        let mut out = vec![0u32; 300];
        bus.read_burst(0x1ff8, &mut out).unwrap();
        assert!(out.iter().all(|w| *w == 0xABAB_ABAB));
        assert_eq!(
            bus.0.reads,
            [(0x1ff8, 0, 512), (0x21f8, 0, 512), (0x23f8, 0, 176)]
        );
        // Words travel little-endian, and the write is cut the same way.
        let vals: Vec<u32> = (0..130).collect();
        bus.write_burst(0x0800, &vals).unwrap();
        assert_eq!(bus.0.writes.len(), 2);
        assert_eq!(bus.0.writes[0].2.len(), 512);
        assert_eq!(&bus.0.writes[0].2[4..8], &1u32.to_le_bytes());
        assert_eq!(
            (bus.0.writes[1].0, bus.0.writes[1].2.len()),
            (0x0800 + 512, 8)
        );
        // A short answer is an error.
        let mut bus = Ep0Bus(FakeEp0 {
            short: true,
            ..Default::default()
        });
        assert!(bus.read32(reg::ANALYZER_STATUS).is_err());
    }

    #[test]
    fn transfer_geometry_can_be_overridden() {
        let p = plan(200e6, 16, None);
        assert!(p.xfer >= 32 * 1024 && p.count >= 4);
        // A finite capture needs no head drop here: exactly what the limit asks for.
        let small = plan(10e6, 8, Some(1000));
        assert!(small.xfer <= 1024 && small.count == 1, "{small:?}");
    }
}
