//! The setup of the instrument: acquisition settings, row layout, protocol decoders. It is kept
//! between runs (in the configuration directory) and saved inside capture files.

use crate::decode_ui::Config;
use crate::rows::Layout;
use bf_core::Cond;
use bf_decode::i2c::I2cConfig;
use bf_decode::spi::SpiConfig;
use bf_decode::trigger::{I2cMatch, ProtocolTrigger, SpiLane, SpiMatch, TriggerAt, UartMatch};
use bf_decode::uart::UartConfig;
use bf_decode::Format;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Clone, Serialize, Deserialize)]
pub struct DecoderSetup {
    /// Id the decoder had when this was saved (the layout refers to decoders by it).
    pub id: u32,
    pub name: String,
    pub format: Format,
    pub color: [u8; 3],
    pub config: Config,
}

/// How the trigger decides.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum TriggerMode {
    /// Levels and edges of channels.
    #[default]
    Pattern,
    Uart,
    Spi,
    I2c,
}

impl TriggerMode {
    pub const ALL: [TriggerMode; 4] = [TriggerMode::Pattern, TriggerMode::Uart, TriggerMode::Spi, TriggerMode::I2c];

    pub fn label(self) -> &'static str {
        match self {
            TriggerMode::Pattern => "Pattern",
            TriggerMode::Uart => "UART",
            TriggerMode::Spi => "SPI",
            TriggerMode::I2c => "I2C",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum UartWhat {
    #[default]
    Bytes,
    Error,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum I2cWhat {
    Start,
    Stop,
    #[default]
    Address,
    Nak,
    Data,
}

impl I2cWhat {
    pub const ALL: [I2cWhat; 5] = [I2cWhat::Start, I2cWhat::Stop, I2cWhat::Address, I2cWhat::Nak, I2cWhat::Data];

    pub fn label(self) -> &'static str {
        match self {
            I2cWhat::Start => "START condition",
            I2cWhat::Stop => "STOP condition",
            I2cWhat::Address => "Address",
            I2cWhat::Nak => "Not acknowledged",
            I2cWhat::Data => "Data byte",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Direction {
    #[default]
    Any,
    Read,
    Write,
}

/// Protocol trigger settings. Values are kept as the text the user typed and parsed when a capture
/// starts, so an unfinished entry is never lost.
#[derive(Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ProtocolSetup {
    pub uart: UartConfig,
    pub uart_what: UartWhat,
    /// Bytes to look for, as hex ("48 65 6C") or, with `uart_ascii`, as text with `\r \n \xNN` escapes.
    pub uart_bytes: String,
    pub uart_ascii: bool,
    pub spi: SpiConfig,
    pub spi_miso: bool,
    /// Word and the bits that have to match (hex).
    pub spi_value: String,
    pub spi_care: String,
    pub i2c: I2cConfig,
    pub i2c_what: I2cWhat,
    /// Address or data byte (hex).
    pub i2c_value: String,
    pub i2c_dir: Direction,
    /// Which sample of the match is the trigger sample.
    pub trigger_at: TriggerAt,
}

impl Default for ProtocolSetup {
    fn default() -> Self {
        Self {
            uart: UartConfig { rx: 0, ..Default::default() },
            uart_what: UartWhat::Bytes,
            uart_bytes: "sim 0".into(),
            uart_ascii: true,
            spi: SpiConfig { cs: Some(1), sck: 2, mosi: Some(3), miso: None, ..Default::default() },
            spi_miso: false,
            spi_value: "A5".into(),
            spi_care: "FF".into(),
            i2c: I2cConfig { scl: 4, sda: 5 },
            i2c_what: I2cWhat::Address,
            i2c_value: "50".into(),
            i2c_dir: Direction::Any,
            trigger_at: TriggerAt::End,
        }
    }
}

/// "48 65 6c", "0x48,0x65", "4865" -> bytes.
pub fn parse_hex_bytes(text: &str) -> Result<Vec<u8>, String> {
    let cleaned: String = text.replace("0x", "").replace("0X", "").chars().filter(|c| !c.is_whitespace() && *c != ',').collect();
    if cleaned.len() % 2 != 0 {
        return Err("hex bytes come in pairs of digits (48 65 6C)".into());
    }
    (0..cleaned.len() / 2)
        .map(|i| u8::from_str_radix(&cleaned[2 * i..2 * i + 2], 16).map_err(|_| format!("'{}' is not hex", &cleaned[2 * i..2 * i + 2])))
        .collect()
}

/// Text with the escapes `\r \n \t \0 \\` and `\xNN` -> bytes.
pub fn parse_ascii_bytes(text: &str) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    let mut it = text.chars().peekable();
    while let Some(c) = it.next() {
        if c != '\\' {
            if !c.is_ascii() {
                return Err(format!("'{c}' is not an ASCII character"));
            }
            out.push(c as u8);
            continue;
        }
        match it.next() {
            Some('r') => out.push(b'\r'),
            Some('n') => out.push(b'\n'),
            Some('t') => out.push(b'\t'),
            Some('0') => out.push(0),
            Some('\\') => out.push(b'\\'),
            Some('x') => {
                let hex: String = it.by_ref().take(2).collect();
                out.push(u8::from_str_radix(&hex, 16).map_err(|_| format!("\\x{hex} is not a byte"))?);
            }
            Some(other) => return Err(format!("unknown escape \\{other}")),
            None => return Err("a backslash at the end".into()),
        }
    }
    Ok(out)
}

pub fn parse_hex_u32(text: &str) -> Result<u32, String> {
    let t = text.trim().trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(t, 16).map_err(|_| format!("'{}' is not a hex number", text.trim()))
}

impl ProtocolSetup {
    pub fn uart_sequence(&self) -> Result<Vec<u8>, String> {
        if self.uart_ascii {
            parse_ascii_bytes(&self.uart_bytes)
        } else {
            parse_hex_bytes(&self.uart_bytes)
        }
    }

    /// The trigger for `mode` (None for the pattern trigger), or what is wrong with its settings.
    pub fn build(&self, mode: TriggerMode) -> Result<Option<ProtocolTrigger>, String> {
        let t = match mode {
            TriggerMode::Pattern => return Ok(None),
            TriggerMode::Uart => {
                let what = match self.uart_what {
                    UartWhat::Bytes => UartMatch::Bytes(self.uart_sequence()?),
                    UartWhat::Error => UartMatch::Error,
                };
                ProtocolTrigger::Uart(self.uart.clone(), what)
            }
            TriggerMode::Spi => {
                let value = parse_hex_u32(&self.spi_value)?;
                let care = parse_hex_u32(&self.spi_care)?;
                let lane = if self.spi_miso { SpiLane::Miso } else { SpiLane::Mosi };
                ProtocolTrigger::Spi(self.spi.clone(), SpiMatch::Word { lane, value, care })
            }
            TriggerMode::I2c => {
                let byte = |s: &str| -> Result<u8, String> {
                    let v = parse_hex_u32(s)?;
                    u8::try_from(v).map_err(|_| format!("{v:X} does not fit in a byte"))
                };
                let what = match self.i2c_what {
                    I2cWhat::Start => I2cMatch::Start,
                    I2cWhat::Stop => I2cMatch::Stop,
                    I2cWhat::Nak => I2cMatch::Nak,
                    I2cWhat::Data => I2cMatch::Data(byte(&self.i2c_value)?),
                    I2cWhat::Address => {
                        let addr = byte(&self.i2c_value)?;
                        if addr > 0x7f {
                            return Err("an I2C address has 7 bits (00 to 7F)".into());
                        }
                        let read = match self.i2c_dir {
                            Direction::Any => None,
                            Direction::Read => Some(true),
                            Direction::Write => Some(false),
                        };
                        I2cMatch::Address { addr, read }
                    }
                };
                ProtocolTrigger::I2c(self.i2c.clone(), what)
            }
        };
        t.check()?;
        Ok(Some(t))
    }

    pub fn summary(&self, mode: TriggerMode) -> String {
        match mode {
            TriggerMode::Pattern => String::new(),
            TriggerMode::Uart => match self.uart_what {
                UartWhat::Bytes => format!("UART \"{}\"", self.uart_bytes),
                UartWhat::Error => "UART error".into(),
            },
            TriggerMode::Spi => format!("SPI {} = {}", if self.spi_miso { "MISO" } else { "MOSI" }, self.spi_value),
            TriggerMode::I2c => match self.i2c_what {
                I2cWhat::Address => format!("I2C address {}{}", self.i2c_value, match self.i2c_dir { Direction::Any => "", Direction::Read => " R", Direction::Write => " W" }),
                I2cWhat::Data => format!("I2C data {}", self.i2c_value),
                other => format!("I2C {}", other.label()),
            },
        }
    }
}

/// Unit of a recording span.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SpanUnit {
    Samples,
    Micros,
    #[default]
    Millis,
    Seconds,
}

impl SpanUnit {
    pub const ALL: [SpanUnit; 4] = [SpanUnit::Samples, SpanUnit::Micros, SpanUnit::Millis, SpanUnit::Seconds];

    pub fn label(self) -> &'static str {
        match self {
            SpanUnit::Samples => "Sa",
            SpanUnit::Micros => "µs",
            SpanUnit::Millis => "ms",
            SpanUnit::Seconds => "s",
        }
    }
}

/// How much to record before or after the trigger: a number of samples or a time.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Span {
    pub value: f64,
    pub unit: SpanUnit,
}

impl Span {
    pub const fn new(value: f64, unit: SpanUnit) -> Self {
        Self { value, unit }
    }

    /// The span in samples at `rate` samples per second (at least 0, rounded).
    pub fn samples(&self, rate: f64) -> u64 {
        let v = if self.value.is_finite() { self.value.max(0.0) } else { 0.0 };
        let s = match self.unit {
            SpanUnit::Samples => v,
            SpanUnit::Micros => v * 1e-6 * rate,
            SpanUnit::Millis => v * 1e-3 * rate,
            SpanUnit::Seconds => v * rate,
        };
        s.round().min(u64::MAX as f64 / 2.0) as u64
    }

    pub fn text(&self) -> String {
        format!("{} {}", self.value, self.unit.label())
    }
}

/// The trigger condition of the Trigger window.
#[derive(Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TriggerSetup {
    pub enabled: bool,
    pub mode: TriggerMode,
    /// One condition per channel (32 entries) for the pattern trigger.
    pub conds: Vec<Cond>,
    pub protocol: ProtocolSetup,
    /// Recorded before the trigger (the trigger arms once this much has been seen).
    pub pre: Span,
    /// Recorded after the trigger in mode Single; mode Continuous records until Stop.
    pub post: Span,
}

impl Default for TriggerSetup {
    fn default() -> Self {
        Self { enabled: false, mode: TriggerMode::Pattern, conds: vec![Cond::X; 32], protocol: ProtocolSetup::default(), pre: Span::new(1.0, SpanUnit::Millis), post: Span::new(4.0, SpanUnit::Millis) }
    }
}

impl TriggerSetup {
    /// Fit a setup read from a file: 32 conditions, finite non-negative spans.
    pub fn sanitized(mut self) -> Self {
        self.conds.resize(32, Cond::X);
        for sp in [&mut self.pre, &mut self.post] {
            if !sp.value.is_finite() || sp.value < 0.0 {
                sp.value = 0.0;
            }
        }
        self
    }

    /// What the trigger waits for, in a few words.
    pub fn summary(&self, names: &[String]) -> String {
        if self.mode != TriggerMode::Pattern {
            return self.protocol.summary(self.mode);
        }
        let set: Vec<String> = self
            .conds
            .iter()
            .enumerate()
            .filter(|(_, c)| **c != Cond::X)
            .map(|(i, c)| format!("{} {}", names.get(i).cloned().unwrap_or_else(|| format!("CH{i}")), c.label()))
            .collect();
        match set.len() {
            0 => "immediately".to_string(),
            1..=3 => set.join(", "),
            n => format!("{}, ... ({n} conditions)", set[..2].join(", ")),
        }
    }
}

/// Everything is optional so that older or hand-edited files still load.
#[derive(Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct Setup {
    pub version: u32,
    pub device: Option<String>,
    pub channels: Option<u8>,
    pub rate: Option<f64>,
    pub pattern: Option<String>,
    pub threshold: Option<f64>,
    pub continuous: Option<bool>,
    pub samples: Option<f64>,
    pub snap: Option<bool>,
    pub row_height: Option<f32>,
    pub row_auto: Option<bool>,
    pub layout: Option<Layout>,
    pub decoders: Vec<DecoderSetup>,
    pub trigger: Option<TriggerSetup>,
    /// Memory the capture may use (MiB).
    pub memory_mib: Option<u64>,
    pub zero_at_trigger: Option<bool>,
    /// Marker positions and the visible window (first sample, samples per pixel): saved with
    /// captures only.
    pub markers: Option<(Option<f64>, Option<f64>)>,
    pub view: Option<(f64, f64)>,
    /// Directory of the last file dialog.
    pub last_dir: Option<String>,
}

pub const VERSION: u32 = 1;

impl Setup {
    pub fn to_json(&self) -> serde_json::Value {
        serde_json::to_value(self).unwrap_or(serde_json::Value::Null)
    }

    pub fn from_json(v: &serde_json::Value) -> Result<Self, String> {
        if v.is_null() {
            return Ok(Self::default());
        }
        let s: Setup = serde_json::from_value(v.clone()).map_err(|e| format!("setup is not readable: {e}"))?;
        if s.version > VERSION {
            return Err(format!("the setup was written by a newer version ({} > {VERSION})", s.version));
        }
        if let (Some(l), Some(n)) = (&s.layout, s.channels) {
            l.check(n as usize)?;
        }
        Ok(s)
    }

    pub fn read(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let v: serde_json::Value = serde_json::from_str(&text).map_err(|e| format!("{}: not a setup file ({e})", path.display()))?;
        Self::from_json(&v)
    }

    /// Write through a temporary file so that a crash never leaves a half-written setup behind.
    pub fn write(&self, path: &Path) -> Result<(), String> {
        let text = serde_json::to_string_pretty(self).map_err(|e| e.to_string())?;
        write_atomic(path, text.as_bytes()).map_err(|e| format!("{}: {e}", path.display()))
    }
}

pub fn write_atomic(path: &Path, data: &[u8]) -> std::io::Result<()> {
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir)?;
    }
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    std::fs::write(&tmp, data)?;
    std::fs::rename(&tmp, path)
}

/// Where the setup is kept between runs.
pub fn autosave_path() -> Option<PathBuf> {
    // `BLINKFEUER_CONFIG_DIR` replaces the user's configuration directory (tests, screenshots).
    let base = std::env::var_os("BLINKFEUER_CONFIG_DIR").map(PathBuf::from).or_else(dirs::config_dir)?;
    Some(base.join("blinkfeuer").join("setup.json"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decode_ui::Config;
    use crate::rows::{Dest, Mover, Radix};

    #[test]
    fn a_setup_survives_json() {
        let mut layout = Layout::generator(16);
        layout.add_group("SPI", vec![1, 2, 3]).unwrap();
        layout.add_decoder(4);
        layout.move_to(Mover::Decoder(4), Dest::Group { item: 1, pos: 0 });
        layout.colors[2] = [9, 8, 7];
        layout.add_bus("X", vec![4, 5], Radix::Bin).unwrap();
        let s = Setup {
            version: VERSION,
            channels: Some(16),
            rate: Some(2e6),
            layout: Some(layout.clone()),
            decoders: vec![DecoderSetup {
                id: 4,
                name: "UART".into(),
                format: Format::Ascii,
                color: [1, 2, 3],
                config: Config::default_for(crate::decode_ui::DecoderKind::Uart),
            }],
            markers: Some((Some(10.5), None)),
            ..Default::default()
        };
        let text = serde_json::to_string(&s.to_json()).unwrap();
        let back = Setup::from_json(&serde_json::from_str(&text).unwrap()).unwrap();
        assert_eq!(back.rate, Some(2e6));
        assert_eq!(back.layout.as_ref().unwrap().items, layout.items);
        assert_eq!(back.layout.as_ref().unwrap().names, layout.names);
        assert_eq!(back.decoders.len(), 1);
        assert!(back.decoders[0].config == s.decoders[0].config);
        assert_eq!(back.markers, Some((Some(10.5), None)));
    }

    #[test]
    fn odd_files_are_handled() {
        // Missing fields are fine, unknown ones are ignored, nonsense is an error, never a panic.
        assert!(Setup::from_json(&serde_json::json!({})).unwrap().layout.is_none());
        assert!(Setup::from_json(&serde_json::json!({"future_field": 1, "rate": 5.0})).unwrap().rate == Some(5.0));
        assert!(Setup::from_json(&serde_json::json!({"rate": "fast"})).is_err());
        assert!(Setup::from_json(&serde_json::json!({"version": 99})).is_err());
        assert!(Setup::from_json(&serde_json::json!([1, 2])).is_err());
        assert!(Setup::from_json(&serde_json::Value::Null).is_ok());
        // A layout for another channel count, or with a channel placed twice, is refused.
        let mut l = Layout::new(4);
        let bad_count = serde_json::json!({"channels": 8, "layout": l});
        assert!(Setup::from_json(&bad_count).is_err());
        l.items.push(crate::rows::Item::Signal(0));
        assert!(Setup::from_json(&serde_json::json!({"channels": 4, "layout": l})).is_err());
    }

    #[test]
    fn decoder_ids_are_renamed_on_load() {
        let mut l = Layout::new(4);
        l.add_group("G", vec![0]).unwrap();
        l.add_decoder(10);
        l.add_decoder(11);
        l.add_decoder(12);
        l.move_to(Mover::Decoder(11), Dest::Group { item: 0, pos: 0 });
        l.set_decoder_shown(12, false);
        let map: std::collections::HashMap<u32, u32> = [(10, 100), (11, 101), (12, 102)].into();
        l.map_decoder_ids(&map);
        assert_eq!(l.group(0).unwrap().decoders, [101]);
        assert!(l.items.contains(&crate::rows::Item::Decoder(100)));
        assert_eq!(l.hidden_decoders, [102]);
        // A decoder the layout does not know gets a place at the end.
        l.place_missing_decoders(&[100, 101, 103]);
        assert_eq!(l.items.last(), Some(&crate::rows::Item::Decoder(103)));
    }

    #[test]
    fn atomic_write_replaces_the_file() {
        let dir = std::env::temp_dir().join(format!("lv-setup-test-{}", std::process::id()));
        let p = dir.join("sub").join("setup.json");
        Setup { rate: Some(1.0), ..Default::default() }.write(&p).unwrap();
        Setup { rate: Some(2.0), ..Default::default() }.write(&p).unwrap();
        assert_eq!(Setup::read(&p).unwrap().rate, Some(2.0));
        assert!(!dir.join("sub").join("setup.json.tmp").exists());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn typed_trigger_values_are_parsed() {
        assert_eq!(parse_hex_bytes("48 65 6c").unwrap(), b"Hel");
        assert_eq!(parse_hex_bytes("0x48,0x65").unwrap(), b"He");
        assert_eq!(parse_hex_bytes("4865").unwrap(), b"He");
        assert!(parse_hex_bytes("486").is_err() && parse_hex_bytes("zz").is_err());
        assert_eq!(parse_hex_bytes("").unwrap(), b"");
        assert_eq!(parse_ascii_bytes("OK\\r\\n").unwrap(), b"OK\r\n");
        assert_eq!(parse_ascii_bytes("a\\x41\\\\b").unwrap(), b"aA\\b");
        assert!(parse_ascii_bytes("bad\\q").is_err() && parse_ascii_bytes("x\\").is_err() && parse_ascii_bytes("\\xZZ").is_err());
        assert!(parse_ascii_bytes("é").is_err());
        assert_eq!(parse_hex_u32(" 0xA5 ").unwrap(), 0xA5);
        assert!(parse_hex_u32("G").is_err() && parse_hex_u32("").is_err());
    }

    #[test]
    fn protocol_settings_build_triggers_or_say_what_is_wrong() {
        let mut p = ProtocolSetup::default();
        assert!(p.build(TriggerMode::Pattern).unwrap().is_none());
        // UART: the default text, then a hex entry, then nothing typed at all.
        assert_eq!(p.build(TriggerMode::Uart).unwrap(), Some(ProtocolTrigger::Uart(p.uart.clone(), UartMatch::Bytes(b"sim 0".to_vec()))));
        p.uart_ascii = false;
        p.uart_bytes = "4F 4B".into();
        assert!(matches!(p.build(TriggerMode::Uart).unwrap(), Some(ProtocolTrigger::Uart(_, UartMatch::Bytes(b))) if b == b"OK"));
        p.uart_bytes.clear();
        assert!(p.build(TriggerMode::Uart).is_err());
        p.uart_what = UartWhat::Error;
        assert!(p.build(TriggerMode::Uart).is_ok());
        // SPI needs the line it looks at.
        assert!(p.build(TriggerMode::Spi).is_ok());
        p.spi_miso = true;
        assert!(p.build(TriggerMode::Spi).is_err(), "MISO has no channel");
        p.spi.miso = Some(4);
        assert!(p.build(TriggerMode::Spi).is_ok());
        p.spi_value = "XYZ".into();
        assert!(p.build(TriggerMode::Spi).is_err());
        // I2C: addresses have 7 bits.
        p.i2c_value = "80".into();
        assert!(p.build(TriggerMode::I2c).is_err());
        p.i2c_value = "7F".into();
        p.i2c_dir = Direction::Read;
        assert_eq!(p.build(TriggerMode::I2c).unwrap(), Some(ProtocolTrigger::I2c(p.i2c.clone(), I2cMatch::Address { addr: 0x7f, read: Some(true) })));
        p.i2c_what = I2cWhat::Data;
        p.i2c_value = "FF".into();
        assert!(p.build(TriggerMode::I2c).is_ok());
        p.i2c_value = "1FF".into();
        assert!(p.build(TriggerMode::I2c).is_err());
    }

    #[test]
    fn trigger_setup_survives_json_with_old_files_too() {
        let mut t = TriggerSetup { enabled: true, mode: TriggerMode::Spi, ..Default::default() };
        t.protocol.spi_value = "3C".into();
        let back: TriggerSetup = serde_json::from_value(serde_json::to_value(&t).unwrap()).unwrap();
        assert!(back == t);
        // A file from before protocol triggers existed has none of the new fields.
        let old: TriggerSetup = serde_json::from_value(serde_json::json!({"enabled": true, "pre_percent": 30.0})).unwrap();
        assert_eq!(old.pre, Span::new(1.0, SpanUnit::Millis));
        assert!(old.enabled && old.mode == TriggerMode::Pattern && old.protocol == ProtocolSetup::default());
        assert!(t.summary(&[]).contains("SPI MOSI = 3C"));
    }

    #[test]
    fn spans_convert_to_samples() {
        let r = 200e6;
        assert_eq!(Span::new(1.0, SpanUnit::Seconds).samples(r), 200_000_000);
        assert_eq!(Span::new(2.5, SpanUnit::Millis).samples(r), 500_000);
        assert_eq!(Span::new(10.0, SpanUnit::Micros).samples(r), 2_000);
        assert_eq!(Span::new(1234.0, SpanUnit::Samples).samples(r), 1234);
        assert_eq!(Span::new(-5.0, SpanUnit::Millis).samples(r), 0);
        assert_eq!(Span::new(f64::NAN, SpanUnit::Seconds).samples(r), 0);
        assert!(Span::new(1e30, SpanUnit::Seconds).samples(r) > 1 << 60);
        // A file with nonsense spans is repaired, not trusted.
        let t = TriggerSetup { pre: Span::new(f64::INFINITY, SpanUnit::Seconds), post: Span::new(-1.0, SpanUnit::Millis), ..Default::default() }.sanitized();
        assert_eq!((t.pre.value, t.post.value), (0.0, 0.0));
    }
}
