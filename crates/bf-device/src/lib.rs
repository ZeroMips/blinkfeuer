//! Device abstraction: every analyzer backend produces the *raw wire stream* of the SLogic family
//! (sample-major packed samples, see `bf_core::format`), and the capture session unpacks it into the
//! store. Backends handle everything device specific (control protocol, first-sample quirks, transfer
//! rings, stall recovery).

pub mod sim;
pub mod session;
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
    pub fn name(self) -> &'static str {
        match self {
            Pattern::Inputs => "inputs",
            Pattern::Counter => "counter",
            Pattern::Generator => "generator",
            Pattern::Emulation => "emulation",
        }
    }
}

#[derive(Clone, Debug)]
pub struct CaptureConfig {
    pub channels: ChannelCount,
    pub sample_rate: f64,
    /// `None`: continuous until stopped.
    pub limit_samples: Option<u64>,
    pub pattern: Pattern,
    /// Input threshold in volts (ignored by devices without one).
    pub threshold_v: f64,
}

/// Channel count and the highest samplerate the device sustains with it.
#[derive(Clone, Copy, Debug)]
pub struct ChannelMode {
    pub channels: u8,
    pub max_rate: f64,
}

#[derive(Clone, Debug)]
pub struct Capabilities {
    pub modes: Vec<ChannelMode>,
    /// Selectable samplerates in Hz (ascending).
    pub rates: Vec<f64>,
    pub patterns: Vec<Pattern>,
    /// Threshold range in volts and step.
    pub threshold: Option<(f64, f64, f64)>,
}

impl Capabilities {
    pub fn max_rate(&self, channels: u8) -> Option<f64> {
        self.modes.iter().find(|m| m.channels == channels).map(|m| m.max_rate)
    }

    pub fn validate(&self, cfg: &CaptureConfig) -> Result<(), DeviceError> {
        let max = self
            .max_rate(cfg.channels.get())
            .ok_or_else(|| DeviceError::Config(format!("{} channels not supported", cfg.channels.get())))?;
        if cfg.sample_rate > max * 1.000_001 {
            return Err(DeviceError::Config(format!(
                "{} channels support at most {} MHz",
                cfg.channels.get(),
                max / 1e6
            )));
        }
        if !self.patterns.contains(&cfg.pattern) {
            return Err(DeviceError::Config(format!("pattern {} not supported", cfg.pattern.name())));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct DeviceInfo {
    /// Stable identifier (backend + serial or bus path).
    pub id: String,
    pub name: String,
    pub backend: &'static str,
}

#[derive(thiserror::Error, Debug, Clone)]
pub enum DeviceError {
    #[error("invalid configuration: {0}")]
    Config(String),
    #[error("USB error: {0}")]
    Usb(String),
    #[error("device disconnected")]
    Disconnected,
    #[error("device FIFO overflow: data lost")]
    Overflow,
    #[error("stream stalled: {0}")]
    Stalled(String),
    #[error("{0}")]
    Other(String),
}

pub enum ReadResult {
    /// Raw wire bytes, in order, with arbitrary slicing.
    Data(Vec<u8>),
    Timeout,
    /// The capture reached its limit; everything has been delivered.
    Ended,
}

#[derive(Clone, Debug, Default)]
pub struct StreamReport {
    pub bytes: u64,
    pub overflow: bool,
    pub note: Option<String>,
}

pub trait Stream: Send {
    fn read(&mut self, timeout: Duration) -> Result<ReadResult, DeviceError>;
    /// Stop the capture; safe to call more than once.
    fn stop(&mut self) -> Result<StreamReport, DeviceError>;
}

pub trait Device: Send {
    fn info(&self) -> &DeviceInfo;
    fn capabilities(&self) -> &Capabilities;
    /// Configure the device and start streaming.
    fn start(&mut self, cfg: &CaptureConfig) -> Result<Box<dyn Stream>, DeviceError>;
}
