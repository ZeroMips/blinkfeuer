//! Protocol decoders for blinkfeuer.
//!
//! Decoders are event driven: they see the capture store's *changes* (a sample index and the full
//! state word), not individual samples, so the cost of decoding is proportional to the number of
//! edges. They are incremental (a decoder can be fed a growing capture slice by slice, which is how
//! live decoding works) and allocation free per edge; annotations are small `Copy` records whose text
//! is only produced when something is drawn.

pub mod engine;
pub mod i2c;
pub mod spi;
pub mod synth;
pub mod trigger;
pub mod uart;

use std::ops::Range;

/// What an annotation is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Kind {
    /// A data word (UART byte, SPI word, I2C data byte).
    Data,
    /// I2C address byte: `value` is the 7-bit address, [`flag::READ`] the direction.
    Address,
    Start,
    Stop,
    /// A whole transaction (SPI chip select assertion): `value` is the number of words.
    Frame,
    /// Consecutive UART characters forming one message: `value` is the number of characters. The text
    /// is the lane-0 annotations inside `start..end` (see [`telegram_chars`]).
    Telegram,
}

/// Bits of [`Annotation::flags`].
pub mod flag {
    pub const FRAMING: u8 = 1;
    pub const PARITY: u8 = 2;
    pub const BREAK: u8 = 4;
    /// A word cut short by the end of a transaction.
    pub const INCOMPLETE: u8 = 8;
    /// I2C: the byte was not acknowledged.
    pub const NAK: u8 = 16;
    /// I2C address: read access.
    pub const READ: u8 = 32;
    /// I2C START in the middle of a transaction.
    pub const REPEATED: u8 = 64;
    /// The flags that mean "this decoded wrongly or incompletely".
    pub const ERROR: u8 = FRAMING | PARITY | BREAK | INCOMPLETE;
}

/// A decoded item covering samples `start..end` on one lane of its decoder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Annotation {
    pub start: u64,
    pub end: u64,
    pub value: u32,
    pub lane: u8,
    pub kind: Kind,
    /// Number of data bits in `value`.
    pub bits: u8,
    pub flags: u8,
}

impl Annotation {
    pub fn is_error(&self) -> bool {
        self.flags & flag::ERROR != 0
    }
}

/// How data values are shown.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Format {
    Hex,
    Dec,
    Bin,
    Ascii,
}

impl Format {
    pub const ALL: [Format; 4] = [Format::Hex, Format::Dec, Format::Bin, Format::Ascii];

    pub fn label(self) -> &'static str {
        match self {
            Format::Hex => "Hex",
            Format::Dec => "Decimal",
            Format::Bin => "Binary",
            Format::Ascii => "ASCII",
        }
    }
}

/// Text of a data value in `fmt` (`bits` is the word width: hex and binary are zero padded to it).
pub fn format_value(value: u32, bits: u8, fmt: Format) -> String {
    match fmt {
        Format::Hex => format!("{:0w$X}", value, w = (bits as usize).div_ceil(4).max(1)),
        Format::Dec => value.to_string(),
        Format::Bin => format!("{:0w$b}", value, w = (bits as usize).max(1)),
        Format::Ascii => match value {
            0x20..=0x7e => char::from(value as u8).to_string(),
            0x0d => "\\r".into(),
            0x0a => "\\n".into(),
            0x09 => "\\t".into(),
            0x00 => "\\0".into(),
            v => format!("{:02X}", v),
        },
    }
}

/// The characters (annotations of the data lane, sorted) inside telegram `t`.
pub fn telegram_chars<'a>(chars: &'a [Annotation], t: &Annotation) -> &'a [Annotation] {
    let lo = chars.partition_point(|a| a.end <= t.start);
    let hi = chars.partition_point(|a| a.start < t.end).max(lo);
    &chars[lo..hi]
}

/// A telegram's characters as one readable string. ASCII shows text (`\r`, `\n`, `\t`, `\xNN` for other
/// control codes); the other formats list the values separated by spaces. At most `max` characters are
/// shown, ending in "..." when there are more.
pub fn telegram_text(chars: &[Annotation], fmt: Format, max: usize) -> String {
    let mut s = String::new();
    for (i, a) in chars.iter().take(max).enumerate() {
        if fmt == Format::Ascii {
            match a.value {
                0x20..=0x7e => s.push(char::from(a.value as u8)),
                0x0d => s.push_str("\\r"),
                0x0a => s.push_str("\\n"),
                0x09 => s.push_str("\\t"),
                v if v < 0x100 => s.push_str(&format!("\\x{v:02X}")),
                v => s.push_str(&format!("<{v:X}>")),
            }
        } else {
            if i > 0 {
                s.push(' ');
            }
            s.push_str(&format_value(a.value, a.bits, fmt));
        }
    }
    if chars.len() > max {
        s.push_str("...");
    }
    s
}

impl Annotation {
    /// Text for the annotation.
    pub fn label(&self, fmt: Format) -> String {
        let mut s = match self.kind {
            Kind::Start => return if self.flags & flag::REPEATED != 0 { "Sr".into() } else { "S".into() },
            Kind::Stop => return "P".into(),
            Kind::Frame => return format!("{} word{}", self.value, if self.value == 1 { "" } else { "s" }),
            Kind::Telegram => return format!("{} byte{}", self.value, if self.value == 1 { "" } else { "s" }),
            Kind::Address => {
                let rw = if self.flags & flag::READ != 0 { "R" } else { "W" };
                format!("{} {}", format_value(self.value, 7, Format::Hex), rw)
            }
            Kind::Data => format_value(self.value, self.bits, fmt),
        };
        if self.flags & flag::BREAK != 0 {
            s.push_str(" BRK");
        } else if self.flags & flag::FRAMING != 0 {
            s.push_str(" FE");
        }
        if self.flags & flag::PARITY != 0 {
            s.push_str(" PE");
        }
        if self.flags & flag::INCOMPLETE != 0 {
            s.push_str(" ...");
        }
        if self.flags & flag::NAK != 0 {
            s.push_str(" NAK");
        }
        s
    }
}

/// Annotations of one decoder, one vector per lane. Within a lane annotations are sorted and do not
/// overlap, so a visible range is found by binary search.
#[derive(Clone, Debug, Default)]
pub struct AnnStore {
    pub lanes: Vec<Vec<Annotation>>,
}

impl AnnStore {
    pub fn new(lanes: usize) -> Self {
        Self { lanes: vec![Vec::new(); lanes] }
    }

    pub fn push(&mut self, a: Annotation) {
        if let Some(l) = self.lanes.get_mut(a.lane as usize) {
            l.push(a);
        }
    }

    pub fn clear(&mut self) {
        self.lanes.iter_mut().for_each(Vec::clear);
    }

    pub fn len(&self) -> usize {
        self.lanes.iter().map(Vec::len).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Annotations of `lane` that overlap `range`.
    pub fn visible(&self, lane: usize, range: Range<u64>) -> &[Annotation] {
        let Some(l) = self.lanes.get(lane) else { return &[] };
        let lo = l.partition_point(|a| a.end <= range.start);
        let hi = l.partition_point(|a| a.start < range.end);
        &l[lo..hi.max(lo)]
    }

    /// Forget annotations that end before `first` (the store evicted those samples).
    pub fn drop_before(&mut self, first: u64) {
        for l in &mut self.lanes {
            let n = l.partition_point(|a| a.end <= first);
            l.drain(..n);
        }
    }
}

/// A protocol decoder.
///
/// Time contract: the decoder is told about every change of its channels in increasing sample order
/// with [`Decoder::change`]; between two changes the levels are constant. A decoder that needs to
/// look at a level at some later instant (UART sampling) keeps that instant pending and resolves it
/// from [`Decoder::change`] / [`Decoder::advance`], which say that nothing changed before `t`.
pub trait Decoder: Send {
    fn name(&self) -> &'static str;
    /// Names of the annotation lanes.
    fn lanes(&self) -> &'static [&'static str];
    /// Whether anything is ever drawn on `lane` with the current configuration.
    fn lane_used(&self, _lane: usize) -> bool {
        true
    }
    /// Channels the decoder reads, as a state-word mask.
    fn mask(&self) -> u32;
    /// Restart at the beginning of a capture with this sample rate.
    fn reset(&mut self, sample_rate: f64);
    /// The channels changed at sample `t`; `state` is the full word from `t` on. The first call after
    /// `reset` carries the initial state (not a change).
    fn change(&mut self, t: u64, state: u32, out: &mut Vec<Annotation>);
    /// The levels stay as they are during `[last change, end)`.
    fn advance(&mut self, _end: u64, _out: &mut Vec<Annotation>) {}
}

#[inline]
pub(crate) fn bit(state: u32, ch: u8) -> bool {
    (state >> ch) & 1 != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ch(start: u64, value: u32) -> Annotation {
        Annotation { start, end: start + 10, value, lane: 0, kind: Kind::Data, bits: 8, flags: 0 }
    }

    #[test]
    fn telegram_text_shows_ascii_with_escapes() {
        let c: Vec<_> = b"Hi!\r\n\x01".iter().enumerate().map(|(i, &b)| ch(i as u64 * 20, b as u32)).collect();
        assert_eq!(telegram_text(&c, Format::Ascii, 100), "Hi!\\r\\n\\x01");
        assert_eq!(telegram_text(&c[..2], Format::Hex, 100), "48 69");
        assert_eq!(telegram_text(&c[..2], Format::Dec, 100), "72 105");
        assert_eq!(telegram_text(&c, Format::Ascii, 3), "Hi!...");
        assert_eq!(telegram_text(&[], Format::Ascii, 3), "");
    }

    #[test]
    fn telegram_chars_are_found_by_time() {
        let c: Vec<_> = (0..10u64).map(|i| ch(i * 20, 65 + i as u32)).collect();
        let t = Annotation { start: 40, end: 130, value: 5, lane: 1, kind: Kind::Telegram, bits: 0, flags: 0 };
        let inside: Vec<u32> = telegram_chars(&c, &t).iter().map(|a| a.value).collect();
        assert_eq!(inside, vec![67, 68, 69, 70, 71]);
        assert_eq!(t.label(Format::Hex), "5 bytes");
    }
}
