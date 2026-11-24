//! The setup of the instrument: acquisition settings, row layout, protocol decoders. It is kept
//! between runs (in the configuration directory) and saved inside capture files.

use crate::decode_ui::Config;
use crate::rows::Layout;
use bf_core::Cond;
use bf_decode::i2c::I2cConfig;
use bf_decode::spi::SpiConfig;
use bf_decode::trigger::{
    I2cMatch, PatItem, ProtocolTrigger, SpiLane, SpiMatch, TriggerAt, UartMatch,
};
use bf_decode::uart::{Protocol, UartConfig};
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
    pub const ALL: [TriggerMode; 4] = [
        TriggerMode::Pattern,
        TriggerMode::Uart,
        TriggerMode::Spi,
        TriggerMode::I2c,
    ];

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

/// What the I2C trigger of older versions looked for (only read from old setups).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum I2cWhat {
    Start,
    Stop,
    #[default]
    Address,
    Nak,
    Data,
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
///
/// Each protocol can take its settings from a decoder (`*_source`: the decoder's id) instead of its
/// own copy; [`ProtocolSetup::resolved`] puts the decoder's settings in. The own settings are kept
/// (and saved with the decoder's settings filled in), so the trigger still works when the decoder is
/// gone.
#[derive(Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ProtocolSetup {
    pub uart: UartConfig,
    /// Take the UART settings from this decoder.
    pub uart_source: Option<u32>,
    pub uart_what: UartWhat,
    /// What to look for: hex ("48 65 6C", with X for free nibbles, 0b... for single bits and * for
    /// anything in between) or, with `uart_ascii`, text with `\r \n \xNN \xXX \*` escapes.
    pub uart_bytes: String,
    pub uart_ascii: bool,
    /// Also fire on any character with a framing or parity error or a break.
    pub uart_or_error: bool,
    pub spi: SpiConfig,
    pub spi_source: Option<u32>,
    pub spi_miso: bool,
    /// Words to look for: hex with X for free nibbles, 0b... for single bits, * for any words in
    /// between.
    pub spi_pattern: String,
    /// Also fire on a word cut short by the end of the transaction.
    pub spi_or_incomplete: bool,
    pub i2c: I2cConfig,
    pub i2c_source: Option<u32>,
    /// What to look for: S, Sr, P, addresses (@50, @50W, @50R), data bytes, ! after an address or
    /// byte for "not acknowledged", X for free nibbles, * for anything in between.
    pub i2c_pattern: String,
    /// Also fire on any byte that was not acknowledged.
    pub i2c_or_nak: bool,
    /// Settings of older versions, turned into the patterns when a setup is read (never written).
    #[serde(skip_serializing)]
    pub spi_value: Option<String>,
    #[serde(skip_serializing)]
    pub spi_care: Option<String>,
    #[serde(skip_serializing)]
    pub i2c_what: Option<I2cWhat>,
    #[serde(skip_serializing)]
    pub i2c_value: Option<String>,
    #[serde(skip_serializing)]
    pub i2c_dir: Option<Direction>,
    /// Which sample of the match is the trigger sample.
    pub trigger_at: TriggerAt,
}

impl Default for ProtocolSetup {
    fn default() -> Self {
        Self {
            uart: UartConfig {
                rx: 0,
                ..Default::default()
            },
            uart_source: None,
            uart_what: UartWhat::Bytes,
            uart_bytes: "sim 0".into(),
            uart_ascii: true,
            uart_or_error: false,
            spi: SpiConfig {
                cs: Some(1),
                sck: 2,
                mosi: Some(3),
                miso: None,
                ..Default::default()
            },
            spi_source: None,
            spi_miso: false,
            spi_pattern: "A5".into(),
            spi_or_incomplete: false,
            i2c: I2cConfig { scl: 4, sda: 5 },
            i2c_source: None,
            i2c_pattern: "@50".into(),
            i2c_or_nak: false,
            spi_value: None,
            spi_care: None,
            i2c_what: None,
            i2c_value: None,
            i2c_dir: None,
            trigger_at: TriggerAt::End,
        }
    }
}

/// One hex digit, or `X` for a free nibble: (value, care).
fn nibble(c: char) -> Option<(u32, u32)> {
    match c {
        'X' | 'x' => Some((0, 0)),
        _ => c.to_digit(16).map(|v| (v, 0xF)),
    }
}

/// A hex UART pattern: bytes as pairs of hex digits ("48 65 6C", "0x48,0x65", "4865") with `X` for a
/// free nibble ("4X", "XX"), `0b...` for single bits with `X` for free ones ("0b1010XXXX"), and `*`
/// for any number of characters in between.
pub fn parse_hex_pattern(text: &str) -> Result<Vec<PatItem>, String> {
    let mut out = Vec::new();
    for token in text
        .split(|c: char| c.is_whitespace() || c == ',')
        .filter(|t| !t.is_empty())
    {
        if token == "*" {
            out.push(PatItem::Gap);
            continue;
        }
        if let Some(b) = token
            .strip_prefix("0b")
            .or_else(|| token.strip_prefix("0B"))
        {
            if b.is_empty() || b.len() > 9 {
                return Err(format!("{token}: 1 to 9 bits after 0b"));
            }
            let (mut value, mut care) = (0u32, 0u32);
            for c in b.chars() {
                let (v, k) = match c {
                    '0' => (0, 1),
                    '1' => (1, 1),
                    'X' | 'x' => (0, 0),
                    _ => return Err(format!("{token}: bits are 0, 1 or X")),
                };
                value = value << 1 | v;
                care = care << 1 | k;
            }
            out.push(PatItem::Char { value, care });
            continue;
        }
        let t = token
            .strip_prefix("0x")
            .or_else(|| token.strip_prefix("0X"))
            .unwrap_or(token);
        let digits: Vec<char> = t.chars().collect();
        if digits.len() % 2 != 0 {
            return Err("hex bytes come in pairs of digits (48 65 6C)".into());
        }
        for pair in digits.chunks(2) {
            match (nibble(pair[0]), nibble(pair[1])) {
                (Some((hv, hc)), Some((lv, lc))) => {
                    let care = hc << 4 | lc;
                    // An exact byte also fixes a 9th data bit to 0, like a plain byte always did.
                    let care = if care == 0xFF { 0x1FF } else { care };
                    out.push(PatItem::Char {
                        value: hv << 4 | lv,
                        care,
                    });
                }
                _ => {
                    return Err(format!(
                        "'{}{}' is not hex (X: any nibble)",
                        pair[0], pair[1]
                    ))
                }
            }
        }
    }
    Ok(out)
}

/// A text UART pattern: characters, the escapes `\r \n \t \0 \\` and `\xNN` (with `X` for a free
/// nibble: `\xXX` is any character), and `\*` for any number of characters in between.
pub fn parse_text_pattern(text: &str) -> Result<Vec<PatItem>, String> {
    let mut out = Vec::new();
    let mut it = text.chars().peekable();
    while let Some(c) = it.next() {
        if c != '\\' {
            if !c.is_ascii() {
                return Err(format!("'{c}' is not an ASCII character"));
            }
            out.push(PatItem::exact(c as u8));
            continue;
        }
        match it.next() {
            Some('r') => out.push(PatItem::exact(b'\r')),
            Some('n') => out.push(PatItem::exact(b'\n')),
            Some('t') => out.push(PatItem::exact(b'\t')),
            Some('0') => out.push(PatItem::exact(0)),
            Some('\\') => out.push(PatItem::exact(b'\\')),
            Some('*') => out.push(PatItem::Gap),
            Some('x') => {
                let hex: String = it.by_ref().take(2).collect();
                let mut cs = hex.chars();
                match (cs.next().and_then(nibble), cs.next().and_then(nibble)) {
                    (Some((hv, hc)), Some((lv, lc))) => {
                        let care = hc << 4 | lc;
                        out.push(PatItem::Char {
                            value: hv << 4 | lv,
                            care: if care == 0xFF { 0x1FF } else { care },
                        });
                    }
                    _ => return Err(format!("\\x{hex} is not a byte (X: any nibble)")),
                }
            }
            Some(other) => return Err(format!("unknown escape \\{other}")),
            None => return Err("a backslash at the end".into()),
        }
    }
    Ok(out)
}

/// A pattern in the hex notation of [`parse_hex_pattern`].
pub fn format_hex_pattern(items: &[PatItem]) -> String {
    items
        .iter()
        .map(|it| match *it {
            // (I2C items do not occur in UART patterns.)
            PatItem::Char { value, care } => {
                let (care8, nine) = (
                    care & 0xFF,
                    value & care & 0x100 != 0 || care & 0x100 != 0 && care & 0xFF != 0xFF,
                );
                let nib = |shift: u32| {
                    if care8 >> shift & 0xF == 0xF {
                        format!("{:X}", value >> shift & 0xF)
                    } else {
                        "X".into()
                    }
                };
                let nibble_aligned = [0, 4].iter().all(|s| matches!(care8 >> s & 0xF, 0 | 0xF));
                if nibble_aligned && !nine {
                    format!("{}{}", nib(4), nib(0))
                } else {
                    let width = if nine { 9 } else { 8 };
                    let bits: String = (0..width)
                        .rev()
                        .map(|i| match (care >> i & 1, value >> i & 1) {
                            (0, _) => 'X',
                            (_, 1) => '1',
                            _ => '0',
                        })
                        .collect();
                    format!("0b{bits}")
                }
            }
            _ => "*".to_string(),
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// A pattern in the text notation of [`parse_text_pattern`]; `None` if it has single-bit wildcards,
/// which only the hex notation can write.
pub fn format_text_pattern(items: &[PatItem]) -> Option<String> {
    let mut s = String::new();
    for it in items {
        match *it {
            PatItem::Gap => s.push_str("\\*"),
            PatItem::Char { value, care } => {
                let care8 = care & 0xFF;
                if care8 == 0xFF {
                    let c = value as u8;
                    match c {
                        b'\\' => s.push_str("\\\\"),
                        b'\r' => s.push_str("\\r"),
                        b'\n' => s.push_str("\\n"),
                        b'\t' => s.push_str("\\t"),
                        0x20..=0x7E => s.push(c as char),
                        _ => s.push_str(&format!("\\x{c:02X}")),
                    }
                } else if [0, 4].iter().all(|sh| matches!(care8 >> sh & 0xF, 0 | 0xF)) {
                    s.push_str("\\x");
                    s.push_str(&format_hex_pattern(&[*it]));
                } else {
                    return None;
                }
            }
            _ => return None,
        }
    }
    Some(s)
}

/// One value of a pattern: hex digits with X for free nibbles (optionally after 0x), or 0b with bits
/// and X for free bits. Returns (value, care, bits typed); the bits above those typed are 0.
fn parse_value(token: &str) -> Result<(u32, u32, u32), String> {
    if let Some(b) = token
        .strip_prefix("0b")
        .or_else(|| token.strip_prefix("0B"))
    {
        if b.is_empty() || b.len() > 32 {
            return Err(format!("{token}: 1 to 32 bits after 0b"));
        }
        let (mut value, mut care) = (0u32, 0u32);
        for c in b.chars() {
            let (v, k) = match c {
                '0' => (0, 1),
                '1' => (1, 1),
                'X' | 'x' => (0, 0),
                _ => return Err(format!("{token}: bits are 0, 1 or X")),
            };
            value = value << 1 | v;
            care = care << 1 | k;
        }
        return Ok((value, care, b.len() as u32));
    }
    let t = token
        .strip_prefix("0x")
        .or_else(|| token.strip_prefix("0X"))
        .unwrap_or(token);
    if t.is_empty() || t.len() > 8 {
        return Err(format!(
            "'{token}' is not a hex value (1 to 8 digits, X: any nibble)"
        ));
    }
    let (mut value, mut care) = (0u32, 0u32);
    for c in t.chars() {
        let (v, k) =
            nibble(c).ok_or_else(|| format!("'{token}' is not a hex value (X: any nibble)"))?;
        value = value << 4 | v;
        care = care << 4 | k;
    }
    Ok((value, care, 4 * t.len() as u32))
}

/// `care` with the bits above the `typed` ones fixed (they are 0), within `width` bits.
fn fix_high_bits(care: u32, typed: u32, width: u32) -> u32 {
    let full = |n: u32| if n >= 32 { u32::MAX } else { (1u32 << n) - 1 };
    care | full(width) & !full(typed)
}

/// An SPI pattern: one word per token (hex with X for free nibbles, or 0b... with X for free bits;
/// the bits above those typed are 0) and * for any words in between.
pub fn parse_word_pattern(text: &str, bits: u32) -> Result<Vec<PatItem>, String> {
    text.split(|c: char| c.is_whitespace() || c == ',')
        .filter(|t| !t.is_empty())
        .map(|token| {
            if token == "*" {
                return Ok(PatItem::Gap);
            }
            let (value, care, typed) = parse_value(token)?;
            Ok(PatItem::Char {
                value,
                care: fix_high_bits(care, typed, bits),
            })
        })
        .collect()
}

/// An I2C pattern: S (any START), Sr (repeated START), P (STOP), addresses @50 / @50W / @50R, data
/// bytes, ! after an address or byte for "not acknowledged", X for free nibbles (0b... for single
/// bits) and * for anything in between. Letters may be lower case.
pub fn parse_i2c_pattern(text: &str) -> Result<Vec<PatItem>, String> {
    let mut out = Vec::new();
    for token in text
        .split(|c: char| c.is_whitespace() || c == ',')
        .filter(|t| !t.is_empty())
    {
        match token.to_ascii_uppercase().as_str() {
            "*" => out.push(PatItem::Gap),
            "S" => out.push(PatItem::Start { repeated: false }),
            "SR" => out.push(PatItem::Start { repeated: true }),
            "P" => out.push(PatItem::Stop),
            _ => {
                let (body, nak) = match token.strip_suffix('!') {
                    Some(b) => (b, true),
                    None => (token, false),
                };
                if let Some(addr) = body.strip_prefix('@') {
                    let (addr, read) = match addr.chars().last() {
                        Some('W' | 'w') => (&addr[..addr.len() - 1], Some(false)),
                        Some('R' | 'r') => (&addr[..addr.len() - 1], Some(true)),
                        _ => (addr, None),
                    };
                    let (value, care, typed) =
                        parse_value(addr).map_err(|e| format!("address: {e}"))?;
                    out.push(PatItem::Address {
                        value,
                        care: fix_high_bits(care, typed, 8),
                        read,
                        nak,
                    });
                } else {
                    let (value, care, typed) = parse_value(body)?;
                    let care = fix_high_bits(care, typed, 8);
                    out.push(if nak {
                        PatItem::CharNak { value, care }
                    } else {
                        PatItem::Char { value, care }
                    });
                }
            }
        }
    }
    Ok(out)
}

/// A value in `width` bits: hex digits with X if every nibble is either free or fixed, else 0b with
/// one character per bit.
fn format_value(value: u32, care: u32, width: u32) -> String {
    let full = if width >= 32 {
        u32::MAX
    } else {
        (1u32 << width) - 1
    };
    let care = care & full;
    let digits = width.div_ceil(4);
    let aligned = (0..digits).all(|d| {
        matches!(care >> (4 * d) & 0xF, 0 | 0xF)
            || (care >> (4 * d) & 0xF) == (full >> (4 * d) & 0xF)
    });
    if aligned {
        (0..digits)
            .rev()
            .map(|d| {
                if care >> (4 * d) & 0xF == 0 {
                    "X".to_string()
                } else {
                    format!("{:X}", value >> (4 * d) & 0xF)
                }
            })
            .collect()
    } else {
        let bits: String = (0..width)
            .rev()
            .map(|i| match (care >> i & 1, value >> i & 1) {
                (0, _) => 'X',
                (_, 1) => '1',
                _ => '0',
            })
            .collect();
        format!("0b{bits}")
    }
}

/// An SPI pattern in the notation of [`parse_word_pattern`] for `bits`-bit words.
pub fn format_word_pattern(items: &[PatItem], bits: u32) -> String {
    items
        .iter()
        .map(|it| match *it {
            PatItem::Char { value, care } => format_value(value, care, bits),
            _ => "*".into(),
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// An I2C pattern in the notation of [`parse_i2c_pattern`].
pub fn format_i2c_pattern(items: &[PatItem]) -> String {
    items
        .iter()
        .map(|it| match *it {
            PatItem::Start { repeated: false } => "S".into(),
            PatItem::Start { repeated: true } => "Sr".into(),
            PatItem::Stop => "P".into(),
            PatItem::Gap => "*".into(),
            PatItem::Char { value, care } => format_value(value, care, 8),
            PatItem::CharNak { value, care } => format!("{}!", format_value(value, care, 8)),
            PatItem::Address {
                value,
                care,
                read,
                nak,
            } => format!(
                "@{}{}{}",
                format_value(value, care, 8),
                match read {
                    Some(false) => "W",
                    Some(true) => "R",
                    None => "",
                },
                if nak { "!" } else { "" }
            ),
        })
        .collect::<Vec<_>>()
        .join(" ")
}

pub fn parse_hex_u32(text: &str) -> Result<u32, String> {
    let t = text
        .trim()
        .trim_start_matches("0x")
        .trim_start_matches("0X");
    u32::from_str_radix(t, 16).map_err(|_| format!("'{}' is not a hex number", text.trim()))
}

impl ProtocolSetup {
    /// Turn the SPI and I2C settings of older versions (one value, a care mask, an I2C "what") into
    /// the patterns that mean the same.
    pub fn upgrade(&mut self) {
        if let Some(v) = self.spi_value.take() {
            let care = self.spi_care.take().unwrap_or_else(|| "FFFFFFFF".into());
            self.spi_pattern = match (parse_hex_u32(&v), parse_hex_u32(&care)) {
                (Ok(value), Ok(care)) => {
                    format_word_pattern(&[PatItem::Char { value, care }], self.spi.bits as u32)
                }
                _ => v,
            };
        }
        if let Some(what) = self.i2c_what.take() {
            let value = self
                .i2c_value
                .take()
                .unwrap_or_default()
                .trim()
                .to_uppercase();
            let dir = match self.i2c_dir.take().unwrap_or_default() {
                Direction::Any => "",
                Direction::Read => "R",
                Direction::Write => "W",
            };
            (self.i2c_pattern, self.i2c_or_nak) = match what {
                I2cWhat::Start => ("S".into(), false),
                I2cWhat::Stop => ("P".into(), false),
                I2cWhat::Nak => (String::new(), true),
                I2cWhat::Data => (value, false),
                I2cWhat::Address => (format!("@{value}{dir}"), false),
            };
        }
    }

    /// The typed UART pattern.
    pub fn uart_pattern(&self) -> Result<Vec<PatItem>, String> {
        if self.uart_ascii {
            parse_text_pattern(&self.uart_bytes)
        } else {
            parse_hex_pattern(&self.uart_bytes)
        }
    }

    /// These settings with the linked decoders' settings put in (`decoders`: id and settings). A link
    /// to a decoder that is missing or of another kind is ignored: the own settings apply.
    pub fn resolved(&self, decoders: &[(u32, Config)]) -> ProtocolSetup {
        let mut p = self.clone();
        let find = |src: Option<u32>| {
            src.and_then(|id| decoders.iter().find(|(d, _)| *d == id).map(|(_, c)| c))
        };
        if let Some(Config::Uart(c)) = find(self.uart_source) {
            p.uart = c.clone();
        }
        if let Some(Config::Spi(c)) = find(self.spi_source) {
            p.spi = c.clone();
        }
        if let Some(Config::I2c(c)) = find(self.i2c_source) {
            p.i2c = c.clone();
        }
        p
    }

    /// Decoder `id` (with settings `cfg`) is going away: links to it end, keeping its settings as the
    /// own ones.
    pub fn unlink(&mut self, id: u32, cfg: &Config) {
        match cfg {
            Config::Uart(c) if self.uart_source == Some(id) => {
                self.uart = c.clone();
                self.uart_source = None;
            }
            Config::Spi(c) if self.spi_source == Some(id) => {
                self.spi = c.clone();
                self.spi_source = None;
            }
            Config::I2c(c) if self.i2c_source == Some(id) => {
                self.i2c = c.clone();
                self.i2c_source = None;
            }
            _ => {}
        }
    }

    /// Decoder ids changed (a decoder was rebuilt, or a setup loaded): links follow; ids not in `map`
    /// end their link.
    pub fn map_sources(&mut self, map: &dyn Fn(u32) -> Option<u32>) {
        for src in [
            &mut self.uart_source,
            &mut self.spi_source,
            &mut self.i2c_source,
        ] {
            *src = src.and_then(map);
        }
    }

    /// The trigger for `mode` (None for the pattern trigger), or what is wrong with its settings.
    pub fn build(&self, mode: TriggerMode) -> Result<Option<ProtocolTrigger>, String> {
        let t = match mode {
            TriggerMode::Pattern => return Ok(None),
            TriggerMode::Uart => {
                let what = match self.uart_what {
                    // With Modbus RTU in the settings the pattern applies to whole frames.
                    UartWhat::Bytes if self.uart.protocol == Protocol::ModbusRtu => {
                        UartMatch::ModbusFrames {
                            items: bf_decode::trigger::normalize(self.uart_pattern()?),
                            or_bad: self.uart_or_error,
                        }
                    }
                    UartWhat::Bytes => UartMatch::pattern(self.uart_pattern()?, self.uart_or_error),
                    UartWhat::Error => UartMatch::Error,
                };
                ProtocolTrigger::Uart(self.uart.clone(), what)
            }
            TriggerMode::Spi => {
                let lane = if self.spi_miso {
                    SpiLane::Miso
                } else {
                    SpiLane::Mosi
                };
                let items = parse_word_pattern(&self.spi_pattern, self.spi.bits as u32)?;
                ProtocolTrigger::Spi(
                    self.spi.clone(),
                    SpiMatch::pattern(lane, items, self.spi_or_incomplete),
                )
            }
            TriggerMode::I2c => ProtocolTrigger::I2c(
                self.i2c.clone(),
                I2cMatch::pattern(parse_i2c_pattern(&self.i2c_pattern)?, self.i2c_or_nak),
            ),
        };
        t.check()?;
        Ok(Some(t))
    }

    pub fn summary(&self, mode: TriggerMode) -> String {
        match mode {
            TriggerMode::Pattern => String::new(),
            TriggerMode::Uart => match self.uart_what {
                UartWhat::Bytes if self.uart.protocol == Protocol::ModbusRtu => format!(
                    "Modbus frame {}{}",
                    self.uart_bytes,
                    if self.uart_or_error {
                        " or bad frame"
                    } else {
                        ""
                    }
                ),
                UartWhat::Bytes => format!(
                    "UART \"{}\"{}",
                    self.uart_bytes,
                    if self.uart_or_error { " or error" } else { "" }
                ),
                UartWhat::Error => "UART error".into(),
            },
            TriggerMode::Spi => format!(
                "SPI {} {}{}",
                if self.spi_miso { "MISO" } else { "MOSI" },
                self.spi_pattern,
                if self.spi_or_incomplete {
                    " or incomplete"
                } else {
                    ""
                }
            ),
            TriggerMode::I2c => match (self.i2c_pattern.trim(), self.i2c_or_nak) {
                ("", true) => "I2C NAK".into(),
                (p, nak) => format!("I2C {p}{}", if nak { " or NAK" } else { "" }),
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
    pub const ALL: [SpanUnit; 4] = [
        SpanUnit::Samples,
        SpanUnit::Micros,
        SpanUnit::Millis,
        SpanUnit::Seconds,
    ];

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
        let v = if self.value.is_finite() {
            self.value.max(0.0)
        } else {
            0.0
        };
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
        Self {
            enabled: false,
            mode: TriggerMode::Pattern,
            conds: vec![Cond::X; 32],
            protocol: ProtocolSetup::default(),
            pre: Span::new(1.0, SpanUnit::Millis),
            post: Span::new(4.0, SpanUnit::Millis),
        }
    }
}

impl TriggerSetup {
    /// Fit a setup read from a file: 32 conditions, finite non-negative spans.
    pub fn sanitized(mut self) -> Self {
        self.conds.resize(32, Cond::X);
        self.protocol.upgrade();
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
            .map(|(i, c)| {
                format!(
                    "{} {}",
                    names.get(i).cloned().unwrap_or_else(|| format!("CH{i}")),
                    c.label()
                )
            })
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
        let s: Setup =
            serde_json::from_value(v.clone()).map_err(|e| format!("setup is not readable: {e}"))?;
        if s.version > VERSION {
            return Err(format!(
                "the setup was written by a newer version ({} > {VERSION})",
                s.version
            ));
        }
        if let (Some(l), Some(n)) = (&s.layout, s.channels) {
            l.check(n as usize)?;
        }
        Ok(s)
    }

    pub fn read(path: &Path) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let v: serde_json::Value = serde_json::from_str(&text)
            .map_err(|e| format!("{}: not a setup file ({e})", path.display()))?;
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
    let base = std::env::var_os("BLINKFEUER_CONFIG_DIR")
        .map(PathBuf::from)
        .or_else(dirs::config_dir)?;
    Some(base.join("blinkfeuer").join("setup.json"))
}

#[cfg(test)]
mod link_tests {
    use super::*;

    fn uart(baud: f64) -> Config {
        Config::Uart(UartConfig {
            baud,
            ..Default::default()
        })
    }

    #[test]
    fn linked_settings_come_from_the_decoder() {
        let mut p = ProtocolSetup::default();
        let decoders = [
            (7, uart(19_200.0)),
            (8, Config::I2c(I2cConfig { scl: 0, sda: 1 })),
        ];
        assert_eq!(p.resolved(&decoders).uart.baud, p.uart.baud, "not linked");
        p.uart_source = Some(7);
        assert_eq!(p.resolved(&decoders).uart.baud, 19_200.0);
        // A link to a missing decoder or one of another kind is ignored.
        p.uart_source = Some(8);
        assert_eq!(p.resolved(&decoders).uart, p.uart);
        p.uart_source = Some(99);
        assert_eq!(p.resolved(&decoders).uart, p.uart);
        p.i2c_source = Some(8);
        assert_eq!(p.resolved(&decoders).i2c, I2cConfig { scl: 0, sda: 1 });
    }

    #[test]
    fn removing_the_decoder_keeps_its_settings() {
        let mut p = ProtocolSetup {
            uart_source: Some(7),
            ..Default::default()
        };
        p.unlink(8, &uart(9_600.0));
        assert_eq!(p.uart_source, Some(7), "another decoder");
        p.unlink(7, &uart(19_200.0));
        assert_eq!((p.uart_source, p.uart.baud), (None, 19_200.0));
    }

    #[test]
    fn links_follow_new_ids_and_old_setups_have_none() {
        let mut p = ProtocolSetup {
            uart_source: Some(3),
            spi_source: Some(4),
            ..Default::default()
        };
        p.map_sources(&|id| (id == 3).then_some(30));
        assert_eq!((p.uart_source, p.spi_source), (Some(30), None));
        let old: ProtocolSetup = serde_json::from_str(r#"{"uart_bytes": "OK"}"#).unwrap();
        assert_eq!(
            (old.uart_source, old.spi_source, old.i2c_source),
            (None, None, None)
        );
    }
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
        assert!(Setup::from_json(&serde_json::json!({}))
            .unwrap()
            .layout
            .is_none());
        assert!(
            Setup::from_json(&serde_json::json!({"future_field": 1, "rate": 5.0}))
                .unwrap()
                .rate
                == Some(5.0)
        );
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
        Setup {
            rate: Some(1.0),
            ..Default::default()
        }
        .write(&p)
        .unwrap();
        Setup {
            rate: Some(2.0),
            ..Default::default()
        }
        .write(&p)
        .unwrap();
        assert_eq!(Setup::read(&p).unwrap().rate, Some(2.0));
        assert!(!dir.join("sub").join("setup.json.tmp").exists());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn typed_trigger_values_are_parsed() {
        let exact = |b: &[u8]| b.iter().map(|&c| PatItem::exact(c)).collect::<Vec<_>>();
        // Plain bytes mean what they always meant.
        assert_eq!(parse_hex_pattern("48 65 6c").unwrap(), exact(b"Hel"));
        assert_eq!(parse_hex_pattern("0x48,0x65").unwrap(), exact(b"He"));
        assert_eq!(parse_hex_pattern("4865").unwrap(), exact(b"He"));
        assert!(parse_hex_pattern("486").is_err() && parse_hex_pattern("zz").is_err());
        assert_eq!(parse_hex_pattern("").unwrap(), vec![]);
        assert_eq!(parse_text_pattern("OK\\r\\n").unwrap(), exact(b"OK\r\n"));
        assert_eq!(parse_text_pattern("a\\x41\\\\b").unwrap(), exact(b"aA\\b"));
        assert!(
            parse_text_pattern("bad\\q").is_err()
                && parse_text_pattern("x\\").is_err()
                && parse_text_pattern("\\xZZ").is_err()
        );
        assert!(parse_text_pattern("é").is_err());
        // Don't-care: X everywhere, in either case.
        let ch = |value, care| PatItem::Char { value, care };
        assert_eq!(
            parse_hex_pattern("XX 4X xf 0b1010XXXX * 0x0d").unwrap(),
            vec![
                ch(0, 0),
                ch(0x40, 0xF0),
                ch(0x0F, 0x0F),
                ch(0b1010_0000, 0b1111_0000),
                PatItem::Gap,
                PatItem::exact(0x0D)
            ]
        );
        assert!(parse_hex_pattern("0b102").is_err() && parse_hex_pattern("0b").is_err());
        assert_eq!(
            parse_text_pattern("OK\\xXX\\x4X\\*\\n").unwrap(),
            vec![
                PatItem::exact(b'O'),
                PatItem::exact(b'K'),
                ch(0, 0),
                ch(0x40, 0xF0),
                PatItem::Gap,
                PatItem::exact(b'\n')
            ]
        );
        // ? and * are plain characters in text.
        assert_eq!(parse_text_pattern("?*").unwrap(), exact(b"?*"));
    }

    #[test]
    fn patterns_convert_between_text_and_hex() {
        for hex in ["4F 4B 0D 0A", "XX 4X * 0A", "0b10X0XXXX 41"] {
            let items = parse_hex_pattern(hex).unwrap();
            assert_eq!(format_hex_pattern(&items), hex);
            if let Some(text) = format_text_pattern(&items) {
                assert_eq!(parse_text_pattern(&text).unwrap(), items, "{hex} -> {text}");
            }
        }
        let ok = parse_hex_pattern("4F 4B 0D 0A").unwrap();
        assert_eq!(format_text_pattern(&ok).unwrap(), "OK\\r\\n");
        let wild = parse_hex_pattern("XX 4X * 5C").unwrap();
        assert_eq!(format_text_pattern(&wild).unwrap(), "\\xXX\\x4X\\*\\\\");
        // Bits on nibble boundaries are written as nibbles.
        assert_eq!(
            format_hex_pattern(&parse_hex_pattern("0b1010XXXX").unwrap()),
            "AX"
        );
        // Single-bit wildcards only exist in hex.
        assert_eq!(
            format_text_pattern(&parse_hex_pattern("0b1X").unwrap()),
            None
        );
        assert_eq!(parse_hex_u32(" 0xA5 ").unwrap(), 0xA5);
        assert!(parse_hex_u32("G").is_err() && parse_hex_u32("").is_err());
    }

    #[test]
    fn protocol_settings_build_triggers_or_say_what_is_wrong() {
        let mut p = ProtocolSetup::default();
        assert!(p.build(TriggerMode::Pattern).unwrap().is_none());
        // UART: the default text, then a hex entry, then nothing typed at all.
        assert_eq!(
            p.build(TriggerMode::Uart).unwrap(),
            Some(ProtocolTrigger::Uart(
                p.uart.clone(),
                UartMatch::bytes(b"sim 0")
            ))
        );
        p.uart_ascii = false;
        p.uart_bytes = "4F 4B".into();
        assert!(
            p.build(TriggerMode::Uart).unwrap()
                == Some(ProtocolTrigger::Uart(
                    p.uart.clone(),
                    UartMatch::bytes(b"OK")
                ))
        );
        p.uart_or_error = true;
        p.uart_bytes = "4F * 4B".into();
        assert_eq!(
            p.build(TriggerMode::Uart).unwrap(),
            Some(ProtocolTrigger::Uart(
                p.uart.clone(),
                UartMatch::pattern(
                    vec![PatItem::exact(b'O'), PatItem::Gap, PatItem::exact(b'K')],
                    true
                )
            ))
        );
        // With Modbus RTU in the settings the pattern applies to whole frames.
        let mut modbus = p.clone();
        modbus.uart.protocol = Protocol::ModbusRtu;
        assert!(matches!(
            modbus.build(TriggerMode::Uart).unwrap(),
            Some(ProtocolTrigger::Uart(
                _,
                UartMatch::ModbusFrames { or_bad: true, .. }
            ))
        ));
        assert!(modbus
            .summary(TriggerMode::Uart)
            .starts_with("Modbus frame"));
        p.uart_bytes.clear();
        assert!(
            p.build(TriggerMode::Uart).is_ok(),
            "nothing typed, but errors"
        );
        p.uart_or_error = false;
        assert!(p.build(TriggerMode::Uart).is_err(), "nothing to look for");
        p.uart_what = UartWhat::Error;
        assert!(p.build(TriggerMode::Uart).is_ok());
        // SPI needs the line it looks at.
        assert!(p.build(TriggerMode::Spi).is_ok());
        p.spi_miso = true;
        assert!(p.build(TriggerMode::Spi).is_err(), "MISO has no channel");
        p.spi.miso = Some(4);
        assert!(p.build(TriggerMode::Spi).is_ok());
        p.spi_pattern = "XYZ".into();
        assert!(p.build(TriggerMode::Spi).is_err());
        p.spi_pattern = "12 * AX".into();
        p.spi_or_incomplete = true;
        assert_eq!(
            p.build(TriggerMode::Spi).unwrap(),
            Some(ProtocolTrigger::Spi(
                p.spi.clone(),
                SpiMatch::pattern(
                    SpiLane::Miso,
                    vec![
                        PatItem::Char {
                            value: 0x12,
                            care: 0xFF
                        },
                        PatItem::Gap,
                        PatItem::Char {
                            value: 0xA0,
                            care: 0xF0
                        }
                    ],
                    true
                )
            ))
        );
        // I2C: addresses have 7 bits, bytes 8.
        p.i2c_pattern = "@80".into();
        assert!(p.build(TriggerMode::I2c).is_err());
        p.i2c_pattern = "@7FR".into();
        assert_eq!(
            p.build(TriggerMode::I2c).unwrap(),
            Some(ProtocolTrigger::I2c(
                p.i2c.clone(),
                I2cMatch::address(0x7f, Some(true))
            ))
        );
        p.i2c_pattern = "FF".into();
        assert!(p.build(TriggerMode::I2c).is_ok());
        p.i2c_pattern = "1FF".into();
        assert!(p.build(TriggerMode::I2c).is_err());
        p.i2c_pattern.clear();
        assert!(p.build(TriggerMode::I2c).is_err(), "nothing to look for");
        p.i2c_or_nak = true;
        assert!(p.build(TriggerMode::I2c).is_ok(), "only NAKs");
    }

    #[test]
    fn spi_and_i2c_patterns_are_parsed() {
        let ch = |value, care| PatItem::Char { value, care };
        // SPI: one word per entry; bits above those typed are 0.
        assert_eq!(
            parse_word_pattern("A5 x3 * 0b1X 12AB", 16).unwrap(),
            vec![
                ch(0xA5, 0xFFFF),
                ch(0x03, 0xFF0F),
                PatItem::Gap,
                ch(0b10, 0xFFFE),
                ch(0x12AB, 0xFFFF)
            ]
        );
        assert!(
            parse_word_pattern("G1", 8).is_err() && parse_word_pattern("123456789", 32).is_err()
        );
        assert_eq!(
            format_word_pattern(&parse_word_pattern("AX 3C * 0b1X0XXXXX", 8).unwrap(), 8),
            "AX 3C * 0b1X0XXXXX"
        );
        assert_eq!(format_word_pattern(&[ch(0x12AB, 0xFF0F)], 16), "12XB");
        // I2C: conditions, addresses with direction, bytes, NAK, wildcards, any case.
        assert_eq!(
            parse_i2c_pattern("S @50W 01 xx! sr @4xr P *").unwrap(),
            vec![
                PatItem::Start { repeated: false },
                PatItem::Address {
                    value: 0x50,
                    care: 0xFF,
                    read: Some(false),
                    nak: false
                },
                ch(0x01, 0xFF),
                PatItem::CharNak { value: 0, care: 0 },
                PatItem::Start { repeated: true },
                PatItem::Address {
                    value: 0x40,
                    care: 0xF0,
                    read: Some(true),
                    nak: false
                },
                PatItem::Stop,
                PatItem::Gap
            ]
        );
        assert_eq!(
            parse_i2c_pattern("@50!").unwrap(),
            vec![PatItem::Address {
                value: 0x50,
                care: 0xFF,
                read: None,
                nak: true
            }]
        );
        assert!(parse_i2c_pattern("@").is_err() && parse_i2c_pattern("Q").is_err());
        for text in ["S @50W 01 XX! Sr @4XR P", "@5X * P", "3C! 0b1X0XXXXX"] {
            assert_eq!(format_i2c_pattern(&parse_i2c_pattern(text).unwrap()), text);
        }
    }

    #[test]
    fn old_spi_and_i2c_settings_become_patterns() {
        let load = |json: serde_json::Value| {
            let t: TriggerSetup =
                serde_json::from_value(serde_json::json!({ "protocol": json })).unwrap();
            t.sanitized().protocol
        };
        let p = load(serde_json::json!({"spi_value": "A5", "spi_care": "F0"}));
        assert_eq!(p.spi_pattern, "AX");
        assert!(p.spi_value.is_none() && p.spi_care.is_none());
        let p = load(serde_json::json!({"spi_value": "A5", "spi_care": "FF"}));
        assert_eq!(p.spi_pattern, "A5");
        for (what, value, dir, pattern, nak) in [
            ("Address", "50", "Any", "@50", false),
            ("Address", "48", "Read", "@48R", false),
            ("Address", "3c", "Write", "@3CW", false),
            ("Data", "7E", "Any", "7E", false),
            ("Start", "50", "Any", "S", false),
            ("Stop", "50", "Any", "P", false),
            ("Nak", "50", "Any", "", true),
        ] {
            let p = load(serde_json::json!({"i2c_what": what, "i2c_value": value, "i2c_dir": dir}));
            assert_eq!(
                (p.i2c_pattern.as_str(), p.i2c_or_nak),
                (pattern, nak),
                "{what}"
            );
        }
        // New setups write no old fields, and keep their patterns.
        let mut p = ProtocolSetup::default();
        p.i2c_pattern = "S @50W".into();
        let json = serde_json::to_value(&p).unwrap();
        assert!(json.get("i2c_what").is_none() && json.get("spi_value").is_none());
        assert_eq!(load(json).i2c_pattern, "S @50W");
    }

    #[test]
    fn trigger_setup_survives_json_with_old_files_too() {
        let mut t = TriggerSetup {
            enabled: true,
            mode: TriggerMode::Spi,
            ..Default::default()
        };
        t.protocol.spi_pattern = "3C".into();
        let back: TriggerSetup = serde_json::from_value(serde_json::to_value(&t).unwrap()).unwrap();
        assert!(back == t);
        // A file from before protocol triggers existed has none of the new fields.
        let old: TriggerSetup =
            serde_json::from_value(serde_json::json!({"enabled": true, "pre_percent": 30.0}))
                .unwrap();
        assert_eq!(old.pre, Span::new(1.0, SpanUnit::Millis));
        assert!(
            old.enabled
                && old.mode == TriggerMode::Pattern
                && old.protocol == ProtocolSetup::default()
        );
        assert!(t.summary(&[]).contains("SPI MOSI 3C"));
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
        let t = TriggerSetup {
            pre: Span::new(f64::INFINITY, SpanUnit::Seconds),
            post: Span::new(-1.0, SpanUnit::Millis),
            ..Default::default()
        }
        .sanitized();
        assert_eq!((t.pre.value, t.post.value), (0.0, 0.0));
    }
}
