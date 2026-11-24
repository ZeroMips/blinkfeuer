//! Device abstraction: every analyzer backend produces the *raw wire stream* of the SLogic family
//! (sample-major packed samples, see `bf_core::format`), and the capture session unpacks it into the
//! store. Backends handle everything device specific (control protocol, first-sample quirks, transfer
//! rings, stall recovery).

#![warn(missing_docs)]

pub mod fx2lafw;
pub mod litex;
pub mod session;
pub mod sim;
pub mod stock;

use bf_core::ChannelCount;
use std::time::Duration;

/// What the analyzer feeds the capture engine with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pattern {
    /// The real input pins.
    Inputs,
    /// 16-bit counter, sample i = i (end-to-end check without probes).
    Counter,
    /// Structured generator: UART, SPI, I2C, PWM, clock, Gray counter (decoder demo and tests).
    Generator,
    /// The stock firmware's own "Emulation" test pattern (content defined by Sipeed).
    Emulation,
}

impl Pattern {
    /// Name used on the command line and in setups.
    pub fn name(self) -> &'static str {
        match self {
            Pattern::Inputs => "inputs",
            Pattern::Counter => "counter",
            Pattern::Generator => "generator",
            Pattern::Emulation => "emulation",
        }
    }
}

/// What to capture.
#[derive(Clone, Debug)]
pub struct CaptureConfig {
    /// Channels to capture (from CH0 on).
    pub channels: ChannelCount,
    /// Samples per second.
    pub sample_rate: f64,
    /// `None`: continuous until stopped.
    pub limit_samples: Option<u64>,
    /// The inputs, or a test pattern.
    pub pattern: Pattern,
    /// Input threshold in volts (ignored by devices without one).
    pub threshold_v: f64,
}

/// Channel count and the highest samplerate the device sustains with it.
#[derive(Clone, Copy, Debug)]
pub struct ChannelMode {
    /// Channel count.
    pub channels: u8,
    /// Highest samplerate in Hz.
    pub max_rate: f64,
}

/// What a device can do.
#[derive(Clone, Debug)]
pub struct Capabilities {
    /// The channel counts, each with its highest samplerate.
    pub modes: Vec<ChannelMode>,
    /// Selectable samplerates in Hz (ascending).
    pub rates: Vec<f64>,
    /// The patterns it offers.
    pub patterns: Vec<Pattern>,
    /// Threshold range in volts and step.
    pub threshold: Option<(f64, f64, f64)>,
}

impl Capabilities {
    /// Highest samplerate with `channels` channels; `None` if that count is not supported.
    pub fn max_rate(&self, channels: u8) -> Option<f64> {
        self.modes
            .iter()
            .find(|m| m.channels == channels)
            .map(|m| m.max_rate)
    }

    /// Check the channel count, samplerate limit and pattern of `cfg`.
    pub fn validate(&self, cfg: &CaptureConfig) -> Result<(), DeviceError> {
        let max = self.max_rate(cfg.channels.get()).ok_or_else(|| {
            DeviceError::Config(format!("{} channels not supported", cfg.channels.get()))
        })?;
        if cfg.sample_rate > max * 1.000_001 {
            return Err(DeviceError::Config(format!(
                "{} channels support at most {} MHz",
                cfg.channels.get(),
                max / 1e6
            )));
        }
        if !self.patterns.contains(&cfg.pattern) {
            return Err(DeviceError::Config(format!(
                "pattern {} not supported",
                cfg.pattern.name()
            )));
        }
        Ok(())
    }
}

/// Identification of a device.
#[derive(Clone, Debug)]
pub struct DeviceInfo {
    /// Stable identifier (backend + serial or bus path).
    pub id: String,
    /// Name for the user.
    pub name: String,
    /// Name of the backend that drives it.
    pub backend: &'static str,
}

/// What can go wrong with a device.
#[derive(thiserror::Error, Debug, Clone)]
pub enum DeviceError {
    /// The device cannot do what was asked for.
    #[error("invalid configuration: {0}")]
    Config(String),
    /// A USB transfer failed, or the device answered nonsense.
    #[error("USB error: {0}")]
    Usb(String),
    /// The device is gone.
    #[error("device disconnected")]
    Disconnected,
    /// The device ran out of buffer space and lost samples.
    #[error("device FIFO overflow: data lost")]
    Overflow,
    /// The stream stopped delivering data.
    #[error("stream stalled: {0}")]
    Stalled(String),
    /// Anything else (the text is complete).
    #[error("{0}")]
    Other(String),
}

/// Outcome of [`Stream::read`].
pub enum ReadResult {
    /// Raw wire bytes, in order, with arbitrary slicing.
    Data(Vec<u8>),
    /// Nothing to deliver yet.
    Timeout,
    /// The capture reached its limit; everything has been delivered.
    Ended,
}

/// Summary of a stream.
#[derive(Clone, Debug, Default)]
pub struct StreamReport {
    /// Wire bytes delivered.
    pub bytes: u64,
    /// The device reported an overflow (backends whose device cannot tell leave it false).
    pub overflow: bool,
    /// Technical remarks of the backend, e.g. the transfer sizes.
    pub note: Option<String>,
}

/// A running capture. Stop it with [`Stream::stop`].
pub trait Stream: Send {
    /// The next piece of the stream, waiting at most about `timeout` for it.
    fn read(&mut self, timeout: Duration) -> Result<ReadResult, DeviceError>;
    /// Stop the capture; safe to call more than once.
    fn stop(&mut self) -> Result<StreamReport, DeviceError>;
}

/// An opened analyzer.
pub trait Device: Send {
    /// Identification of the device.
    fn info(&self) -> &DeviceInfo;
    /// What the device can do.
    fn capabilities(&self) -> &Capabilities;
    /// Configure the device and start streaming.
    fn start(&mut self, cfg: &CaptureConfig) -> Result<Box<dyn Stream>, DeviceError>;
}
