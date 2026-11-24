//! Protocol triggers: a decoder runs on the incoming sample stream and the trigger fires when it
//! decodes what was asked for.
//!
//! What to look for is a pattern of items for every protocol: UART characters, SPI words, or I2C
//! conditions, addresses and data bytes, each with don't-care bits, and gaps that match anything in
//! between. As an alternative the trigger can also fire on the protocol's error (a UART framing or
//! parity error or break, an incomplete SPI word, an I2C NAK).
//!
//! The decoders are event driven, so the work is the scan for changes of their channels plus a few
//! operations per edge. The trigger sample is the end (or the start) of the match.

use crate::i2c::{I2c, I2cConfig};
use crate::spi::{Spi, SpiConfig};
use crate::uart::{Uart, UartConfig};
use crate::{flag, Annotation, Decoder, Kind};
use bf_core::EventTrigger;
use serde::{Deserialize, Serialize};

/// One position of a pattern.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PatItem {
    /// A data value (UART character, SPI word, I2C data byte) whose bits set in `care` equal those
    /// of `value` (all bits set: exactly `value`).
    Char {
        /// The bits to compare with.
        value: u32,
        /// Which bits are compared.
        care: u32,
    },
    /// An I2C data byte as `Char` that was not acknowledged.
    CharNak {
        /// The bits to compare with.
        value: u32,
        /// Which bits are compared.
        care: u32,
    },
    /// An I2C address (7 bits) as `Char`; `read`: `Some(true)` only reads, `Some(false)` only
    /// writes; `nak`: it must not have been acknowledged.
    Address {
        /// The bits to compare with.
        value: u32,
        /// Which bits are compared.
        care: u32,
        /// The direction, if it matters: read or write.
        read: Option<bool>,
        /// Only if not acknowledged.
        nak: bool,
    },
    /// An I2C START; `repeated`: only a repeated START.
    Start {
        /// Only a repeated START.
        repeated: bool,
    },
    /// An I2C STOP.
    Stop,
    /// Any number of items, also none.
    Gap,
}

impl PatItem {
    /// Exactly this byte (a 9th data bit must be 0).
    pub fn exact(b: u8) -> Self {
        PatItem::Char {
            value: b as u32,
            care: 0x1FF,
        }
    }

    /// Exactly this word.
    pub fn word(v: u32) -> Self {
        PatItem::Char {
            value: v,
            care: u32::MAX,
        }
    }

    fn matches(&self, a: &Annotation) -> bool {
        let fits = |value: u32, care: u32| (a.value ^ value) & care == 0;
        let nak = a.flags & flag::NAK != 0;
        match *self {
            PatItem::Char { value, care } => a.kind == Kind::Data && fits(value, care),
            PatItem::CharNak { value, care } => a.kind == Kind::Data && fits(value, care) && nak,
            PatItem::Address {
                value,
                care,
                read,
                nak: want_nak,
            } => {
                a.kind == Kind::Address
                    && fits(value, care)
                    && read.is_none_or(|r| r == (a.flags & flag::READ != 0))
                    && (!want_nak || nak)
            }
            PatItem::Start { repeated } => {
                a.kind == Kind::Start && (!repeated || a.flags & flag::REPEATED != 0)
            }
            PatItem::Stop => a.kind == Kind::Stop,
            PatItem::Gap => false,
        }
    }

    /// The value bits that must match (for the width check).
    fn fixed_bits(&self) -> u32 {
        match *self {
            PatItem::Char { value, care }
            | PatItem::CharNak { value, care }
            | PatItem::Address { value, care, .. } => value & care,
            _ => 0,
        }
    }

    fn is_i2c_only(&self) -> bool {
        matches!(
            self,
            PatItem::CharNak { .. }
                | PatItem::Address { .. }
                | PatItem::Start { .. }
                | PatItem::Stop
        )
    }
}

/// A pattern with gaps at its ends removed (they change nothing) and runs of gaps joined.
pub fn normalize(items: Vec<PatItem>) -> Vec<PatItem> {
    let mut out: Vec<PatItem> = Vec::new();
    for it in items {
        if it == PatItem::Gap && out.last().is_none_or(|l| *l == PatItem::Gap) {
            continue;
        }
        out.push(it);
    }
    while out.last() == Some(&PatItem::Gap) {
        out.pop();
    }
    out
}

/// Finds the matches of a pattern in decoded items fed one at a time (the search of the annotation
/// table uses the trigger patterns).
#[derive(Clone, Debug)]
pub struct PatternSearch {
    items: Vec<PatItem>,
    active: Vec<(usize, usize)>,
}

impl PatternSearch {
    /// `None` if the pattern has nothing to look for (only gaps).
    pub fn new(items: Vec<PatItem>) -> Option<Self> {
        let items = normalize(items);
        (!items.is_empty()).then_some(PatternSearch {
            items,
            active: Vec::new(),
        })
    }

    /// Feed item number `index`: the index of the first item of the match it completes, if any.
    /// Matches do not overlap: after one, the search starts afresh (like "find all").
    pub fn feed(&mut self, index: usize, a: &Annotation) -> Option<usize> {
        let hit = step_pattern(&self.items, &mut self.active, a, index, true);
        if hit.is_some() {
            self.active.clear();
        }
        hit
    }

    /// Whether `seq` begins with a match (Modbus frames: the pattern starts at the first byte).
    pub fn found_at_start(&self, seq: &[Annotation]) -> bool {
        matches_from_start(&self.items, seq)
    }

    /// Whether `seq` contains a match (from a fresh start).
    pub fn found_in(&self, seq: &[Annotation]) -> bool {
        let mut s = PatternSearch {
            items: self.items.clone(),
            active: Vec::new(),
        };
        seq.iter().enumerate().any(|(i, a)| s.feed(i, a).is_some())
    }
}

/// What to look for on a UART.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum UartMatch {
    /// Characters matching `items` one after the other (a gap item skips any number of them); with
    /// `or_error` also any character with a framing or parity error or a break.
    Pattern {
        /// The pattern.
        items: Vec<PatItem>,
        /// Also fire on a character error.
        or_error: bool,
    },
    /// A framing error, a parity error or a break.
    Error,
    /// Modbus RTU frames (the UART decoder runs with the Modbus protocol) that begin with bytes
    /// matching `items`; with `or_bad` also any frame with a bad CRC or a character error.
    ModbusFrames {
        /// The bytes a frame begins with.
        items: Vec<PatItem>,
        /// Also fire on a bad frame.
        or_bad: bool,
    },
}

impl UartMatch {
    /// Exactly these characters.
    pub fn bytes(b: &[u8]) -> Self {
        UartMatch::Pattern {
            items: b.iter().map(|&c| PatItem::exact(c)).collect(),
            or_error: false,
        }
    }

    /// A normalised pattern.
    pub fn pattern(items: Vec<PatItem>, or_error: bool) -> Self {
        UartMatch::Pattern {
            items: normalize(items),
            or_error,
        }
    }
}

/// A data line of an SPI bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SpiLane {
    /// Master to slave.
    Mosi,
    /// Slave to master.
    Miso,
}

/// What to look for on an SPI bus: words on one line matching `items`; with `or_incomplete` also a
/// word cut short by the end of the transaction.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SpiMatch {
    /// The line to look at.
    pub lane: SpiLane,
    /// The pattern.
    pub items: Vec<PatItem>,
    /// Also fire on a word cut short.
    pub or_incomplete: bool,
}

impl SpiMatch {
    /// A normalised pattern.
    pub fn pattern(lane: SpiLane, items: Vec<PatItem>, or_incomplete: bool) -> Self {
        SpiMatch {
            lane,
            items: normalize(items),
            or_incomplete,
        }
    }

    /// One word, of which only the bits set in `care` have to match `value`.
    pub fn word(lane: SpiLane, value: u32, care: u32) -> Self {
        Self::pattern(lane, vec![PatItem::Char { value, care }], false)
    }
}

/// What to look for on an I2C bus: conditions, addresses and data bytes matching `items`; with
/// `or_nak` also any byte that was not acknowledged.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct I2cMatch {
    /// The pattern.
    pub items: Vec<PatItem>,
    /// Also fire on any byte that was not acknowledged.
    pub or_nak: bool,
}

impl I2cMatch {
    /// A normalised pattern.
    pub fn pattern(items: Vec<PatItem>, or_nak: bool) -> Self {
        I2cMatch {
            items: normalize(items),
            or_nak,
        }
    }

    /// Any START condition.
    pub fn start() -> Self {
        Self::pattern(vec![PatItem::Start { repeated: false }], false)
    }

    /// Any STOP condition.
    pub fn stop() -> Self {
        Self::pattern(vec![PatItem::Stop], false)
    }

    /// A 7-bit address; `read`: `Some(true)` only reads, `Some(false)` only writes.
    pub fn address(addr: u8, read: Option<bool>) -> Self {
        Self::pattern(
            // All 8 bits are compared, so an address above 7F is caught by the check.
            vec![PatItem::Address {
                value: addr as u32,
                care: 0xFF,
                read,
                nak: false,
            }],
            false,
        )
    }

    /// A data byte (not an address) with this value.
    pub fn data(v: u8) -> Self {
        Self::pattern(
            vec![PatItem::Char {
                value: v as u32,
                care: 0xFF,
            }],
            false,
        )
    }

    /// Any byte that was not acknowledged.
    pub fn nak() -> Self {
        Self::pattern(Vec::new(), true)
    }
}

/// Which sample of the matched item is the trigger sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum TriggerAt {
    /// The first sample of the match (for a sequence: of its first item).
    Start,
    /// The end of the match (the default).
    #[default]
    End,
}

/// A trigger on decoded data: the decoder settings and what to look for.
#[derive(Clone, Debug, PartialEq)]
pub enum ProtocolTrigger {
    /// On a UART.
    Uart(UartConfig, UartMatch),
    /// On an SPI bus.
    Spi(SpiConfig, SpiMatch),
    /// On an I2C bus.
    I2c(I2cConfig, I2cMatch),
}

/// Longest pattern a trigger takes (items and gaps).
pub const MAX_SEQUENCE: usize = 16;

/// The error a pattern trigger can fire on instead.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Alternative {
    None,
    /// A UART framing or parity error or a break.
    UartError,
    /// An SPI word cut short.
    SpiIncomplete,
    /// An I2C byte not acknowledged.
    I2cNak,
}

impl Alternative {
    fn fires(self, a: &Annotation) -> bool {
        match self {
            Alternative::None => false,
            Alternative::UartError => a.is_error(),
            Alternative::SpiIncomplete => a.flags & flag::INCOMPLETE != 0,
            Alternative::I2cNak => a.flags & flag::NAK != 0,
        }
    }
}

/// A pattern (never starting or ending with a gap) on one decoder lane, and its partial matches:
/// (items matched, sample where the match began), at most one per position.
struct Matcher {
    lane: u8,
    items: Vec<PatItem>,
    alternative: Alternative,
    active: Vec<(usize, u64)>,
    /// Modbus mode: the pattern applies to whole frames (lane 1) from their first character; the
    /// characters (lane 0) are kept until their frame is complete.
    frames: Option<Vec<Annotation>>,
}

/// Whether `seq` begins with a match of `items` (the match may end before `seq` does).
fn matches_from_start(items: &[PatItem], seq: &[Annotation]) -> bool {
    if items.is_empty() {
        return true;
    }
    let mut active = Vec::new();
    for (i, c) in seq.iter().enumerate() {
        // Only the first item may begin a match.
        if step_pattern(items, &mut active, c, i, i == 0).is_some() {
            return true;
        }
        if active.is_empty() {
            return false;
        }
    }
    false
}

struct DecoderTrigger {
    dec: Box<dyn Decoder>,
    matcher: Matcher,
    at: TriggerAt,
    out: Vec<Annotation>,
    /// The last sample seen (for the lookback of partial matches).
    now: u64,
    /// Longest item seen so far, in samples: how far back a trigger at the start of an item can reach.
    max_len: u64,
    /// How long before its detection a trigger sample can lie: `lookback_bits` bit times at `bit_rate`.
    lookback_bits: f64,
    bit_rate: f64,
    lookback: u64,
}

impl DecoderTrigger {
    /// The trigger sample if anything decoded since the last call matches.
    fn scan(&mut self) -> Option<u64> {
        let mut hit = None;
        let m = &mut self.matcher;
        for a in self.out.drain(..) {
            if let Some(chars) = &mut m.frames {
                // Modbus: characters wait for their frame; a frame matches as a whole.
                match (a.lane, a.kind) {
                    (0, Kind::Data) => chars.push(a),
                    (1, Kind::ModbusFrame) => {
                        let lo = chars.partition_point(|c| c.start < a.start);
                        let hi = chars.partition_point(|c| c.end <= a.end);
                        let bad = a.is_error();
                        let found = if bad {
                            m.alternative != Alternative::None
                        } else {
                            !m.items.is_empty()
                                && matches_from_start(&m.items, &chars[lo..hi.max(lo)])
                        };
                        chars.drain(..hi.max(lo));
                        self.max_len = self.max_len.max(a.end.saturating_sub(a.start));
                        if found && hit.is_none() {
                            hit = Some(match self.at {
                                TriggerAt::Start => a.start,
                                TriggerAt::End => a.end,
                            });
                        }
                    }
                    _ => {}
                }
                continue;
            }
            if a.lane != m.lane {
                continue;
            }
            self.max_len = self.max_len.max(a.end.saturating_sub(a.start));
            let begins = if m.alternative.fires(&a) {
                m.active.clear();
                Some(a.start)
            } else if m.items.is_empty() {
                None
            } else {
                step_pattern(&m.items, &mut m.active, &a, a.start, true)
            };
            if let (Some(b), None) = (begins, hit) {
                hit = Some(match self.at {
                    TriggerAt::Start => b,
                    TriggerAt::End => a.end,
                });
            }
        }
        hit
    }
}

/// Feed item `c` (which begins at `here`: a sample, or an index) to the partial matches of `items`:
/// where a completed match began (the latest start, i.e. the shortest match) if `c` completes one.
/// With `may_begin` a new match can also begin at `c` (always, except for a match anchored at the
/// first item).
fn step_pattern<T: Copy + Ord>(
    items: &[PatItem],
    active: &mut Vec<(usize, T)>,
    c: &Annotation,
    here: T,
    may_begin: bool,
) -> Option<T> {
    // Add state `i` (and, past a gap that may match nothing, the following ones), keeping the latest
    // start per position.
    fn add<T: Copy + Ord>(items: &[PatItem], next: &mut Vec<(usize, T)>, i: usize, start: T) {
        match next.iter_mut().find(|s| s.0 == i) {
            Some(s) => s.1 = s.1.max(start),
            None => next.push((i, start)),
        }
        if items.get(i) == Some(&PatItem::Gap) {
            add(items, next, i + 1, start);
        }
    }
    let mut current = std::mem::take(active);
    if may_begin {
        add(items, &mut current, 0, here);
    }
    let mut hit = None;
    for (i, start) in current {
        match items[i] {
            PatItem::Gap => add(items, active, i, start),
            ref item if item.matches(c) => {
                if i + 1 == items.len() {
                    hit = Some(hit.map_or(start, |h: T| h.max(start)));
                } else {
                    add(items, active, i + 1, start);
                }
            }
            _ => {}
        }
    }
    hit
}

impl EventTrigger for DecoderTrigger {
    fn mask(&self) -> u32 {
        self.dec.mask()
    }

    fn reset(&mut self, sample_rate: f64) {
        self.dec.reset(sample_rate);
        self.out.clear();
        self.now = 0;
        self.matcher.active.clear();
        if let Some(chars) = &mut self.matcher.frames {
            chars.clear();
        }
        self.max_len = 0;
        self.lookback = (self.lookback_bits * sample_rate / self.bit_rate.max(1.0)) as u64;
    }

    fn change(&mut self, t: u64, state: u32) -> Option<u64> {
        self.now = t;
        self.dec.change(t, state, &mut self.out);
        self.scan()
    }

    fn advance(&mut self, end: u64) -> Option<u64> {
        self.now = self.now.max(end);
        self.dec.advance(end, &mut self.out);
        self.scan()
    }

    fn lookback(&self) -> u64 {
        match self.at {
            TriggerAt::End => self.lookback,
            // The trigger sample is the start of an item that is only recognised at its end, or the
            // start of a pattern match that is still open (a gap can span any time).
            TriggerAt::Start => {
                let open = self
                    .matcher
                    .active
                    .iter()
                    .map(|s| self.now.saturating_sub(s.1))
                    .max()
                    .unwrap_or(0);
                self.lookback.max(2 * self.max_len).max(open)
            }
        }
    }
}

/// What is wrong with a pattern: nothing to look for (and no alternative), too long, an item the
/// protocol does not have (`i2c`), or a value wider than `bits`.
fn check_pattern(
    items: &[PatItem],
    alternative: bool,
    i2c: bool,
    bits: u32,
    what: &str,
) -> Result<(), String> {
    if !alternative && !items.iter().any(|i| *i != PatItem::Gap) {
        return Err(format!("enter at least one {what} to look for"));
    }
    if items.len() > MAX_SEQUENCE {
        return Err(format!("at most {MAX_SEQUENCE} items and gaps"));
    }
    if !i2c && items.iter().any(PatItem::is_i2c_only) {
        return Err("START, STOP, addresses and NAK only exist on I2C".into());
    }
    let full = if bits >= 32 {
        u32::MAX
    } else {
        (1u32 << bits) - 1
    };
    for it in items {
        let limit = if matches!(it, PatItem::Address { .. }) {
            0x7F
        } else {
            full
        };
        if it.fixed_bits() & !limit != 0 {
            return Err(if matches!(it, PatItem::Address { .. }) {
                "an I2C address has 7 bits (00 to 7F)".into()
            } else {
                format!("a {what} does not fit in {bits} bits")
            });
        }
    }
    Ok(())
}

impl ProtocolTrigger {
    /// Problems with the settings that make the trigger useless, if any.
    pub fn check(&self) -> Result<(), String> {
        match self {
            ProtocolTrigger::Uart(c, UartMatch::Pattern { items, or_error }) => {
                check_pattern(items, *or_error, false, c.data_bits as u32, "character")
            }
            ProtocolTrigger::Uart(c, UartMatch::ModbusFrames { items, or_bad }) => {
                if c.protocol != crate::uart::Protocol::ModbusRtu {
                    return Err("frames need the Modbus RTU protocol in the UART settings".into());
                }
                check_pattern(items, *or_bad, false, 8, "byte")
            }
            ProtocolTrigger::Uart(..) => Ok(()),
            ProtocolTrigger::Spi(c, m) => {
                let wired = match m.lane {
                    SpiLane::Mosi => c.mosi,
                    SpiLane::Miso => c.miso,
                };
                if wired.is_none() {
                    return Err("the line to look at has no channel".into());
                }
                check_pattern(&m.items, m.or_incomplete, false, c.bits as u32, "word")
            }
            ProtocolTrigger::I2c(c, m) => {
                if c.scl == c.sda {
                    return Err("SCL and SDA are the same channel".into());
                }
                check_pattern(&m.items, m.or_nak, true, 8, "byte")
            }
        }
    }

    /// The trigger as something the capture thread can run.
    pub fn build(&self, at: TriggerAt) -> Result<Box<dyn EventTrigger>, String> {
        self.check()?;
        let (dec, lane, items, alternative, lookback_bits, bit_rate): (
            Box<dyn Decoder>,
            u8,
            Vec<PatItem>,
            Alternative,
            f64,
            f64,
        ) = match self {
            ProtocolTrigger::Uart(c, m) => {
                let mut cfg = c.clone();
                cfg.telegrams = false;
                if let UartMatch::ModbusFrames { items, or_bad } = m {
                    // A frame is only known once the pause after it has passed: keep the longest
                    // frame (256 characters) and that pause in the window.
                    return Ok(Box::new(DecoderTrigger {
                        dec: Box::new(Uart::new(cfg.clone())),
                        matcher: Matcher {
                            lane: 1,
                            items: normalize(items.clone()),
                            alternative: if *or_bad {
                                Alternative::UartError
                            } else {
                                Alternative::None
                            },
                            active: Vec::new(),
                            frames: Some(Vec::new()),
                        },
                        at,
                        out: Vec::new(),
                        now: 0,
                        max_len: 0,
                        lookback_bits: 262.0 * (cfg.char_bits() + 2.0),
                        bit_rate: cfg.baud,
                        lookback: 0,
                    }));
                }
                let (items, alternative) = match m {
                    UartMatch::Pattern { items, or_error } => (
                        normalize(items.clone()),
                        if *or_error {
                            Alternative::UartError
                        } else {
                            Alternative::None
                        },
                    ),
                    UartMatch::Error => (Vec::new(), Alternative::UartError),
                    UartMatch::ModbusFrames { .. } => unreachable!("handled above"),
                };
                let chars = items.len().max(1) as f64;
                (
                    Box::new(Uart::new(cfg.clone())),
                    0,
                    items,
                    alternative,
                    chars * (cfg.char_bits() + 2.0) + 2.0,
                    cfg.baud,
                )
            }
            ProtocolTrigger::Spi(c, m) => (
                Box::new(Spi::new(c.clone())),
                match m.lane {
                    SpiLane::Mosi => 0,
                    SpiLane::Miso => 1,
                },
                normalize(m.items.clone()),
                if m.or_incomplete {
                    Alternative::SpiIncomplete
                } else {
                    Alternative::None
                },
                0.0,
                1.0,
            ),
            ProtocolTrigger::I2c(c, m) => (
                Box::new(I2c::new(c.clone())),
                0,
                normalize(m.items.clone()),
                if m.or_nak {
                    Alternative::I2cNak
                } else {
                    Alternative::None
                },
                0.0,
                1.0,
            ),
        };
        Ok(Box::new(DecoderTrigger {
            dec,
            matcher: Matcher {
                lane,
                items,
                alternative,
                active: Vec::new(),
                frames: None,
            },
            at,
            out: Vec::new(),
            now: 0,
            max_len: 0,
            lookback_bits,
            bit_rate,
            lookback: 0,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::Wave;

    fn run(t: &ProtocolTrigger, words: &[u32], rate: f64, chunk: usize) -> Option<u64> {
        run_at(t, words, rate, chunk, TriggerAt::End)
    }

    fn run_at(
        t: &ProtocolTrigger,
        words: &[u32],
        rate: f64,
        chunk: usize,
        at: TriggerAt,
    ) -> Option<u64> {
        let mut ev = t.build(at).unwrap();
        ev.reset(rate);
        let mask = ev.mask();
        ev.change(0, words[0]);
        let mut prev = words[0];
        for (k, buf) in words.chunks(chunk).enumerate() {
            let base = (k * chunk) as u64;
            for (i, &w) in buf.iter().enumerate() {
                if (w ^ prev) & mask != 0 {
                    if let Some(t) = ev.change(base + i as u64, w) {
                        return Some(t);
                    }
                }
                prev = w;
            }
            if let Some(t) = ev.advance(base + buf.len() as u64) {
                return Some(t);
            }
        }
        None
    }

    fn uart_wave(bytes: &[u8]) -> (Vec<u32>, UartConfig, u64) {
        let cfg = UartConfig {
            rx: 2,
            baud: 100_000.0,
            ..Default::default()
        };
        let mut w = Wave::new(1e6, 4000, 1 << 2);
        let end = w.uart(2, 100, &cfg, bytes);
        (w.samples.clone(), cfg, end)
    }

    #[test]
    fn uart_byte_sequences_fire_at_the_end_of_the_last_byte() {
        let (words, cfg, _) = uart_wave(b"hello world");
        // 10 samples per bit, 10 bits per frame and two idle bits in between: byte k starts at
        // 100 + 120 k and its stop bit ends 100 samples later.
        let frame_end = |k: u64| 100 + 120 * k + 100;
        for chunk in [1usize, 7, 100, 4096] {
            let t = ProtocolTrigger::Uart(cfg.clone(), UartMatch::bytes(b"lo w"));
            let hit = run(&t, &words, 1e6, chunk).expect("fires");
            // "lo w" ends with the byte at index 6.
            assert!(
                (hit as i64 - frame_end(6) as i64).abs() <= 10,
                "chunk {chunk}: {hit} vs {}",
                frame_end(6)
            );
        }
        // A sequence that is not in the data never fires; a single byte does, at its own end.
        assert_eq!(
            run(
                &ProtocolTrigger::Uart(cfg.clone(), UartMatch::bytes(b"xyz")),
                &words,
                1e6,
                64
            ),
            None
        );
        let one = run(
            &ProtocolTrigger::Uart(cfg.clone(), UartMatch::bytes(b"h")),
            &words,
            1e6,
            64,
        )
        .unwrap();
        assert!((one as i64 - frame_end(0) as i64).abs() <= 10);
        // Bytes must follow each other directly: "h" then "l" is not in "hello world".
        assert_eq!(
            run(
                &ProtocolTrigger::Uart(cfg, UartMatch::bytes(b"hl")),
                &words,
                1e6,
                64
            ),
            None
        );
    }

    /// A character whose cared-for bits are given by a pattern of '0', '1' and 'X', MSB first.
    fn bits(p: &str) -> PatItem {
        let (mut value, mut care) = (0u32, 0u32);
        for c in p.chars() {
            value <<= 1;
            care <<= 1;
            match c {
                '1' => (value, care) = (value | 1, care | 1),
                '0' => care |= 1,
                _ => {}
            }
        }
        PatItem::Char { value, care }
    }

    fn pat(items: Vec<PatItem>) -> UartMatch {
        UartMatch::pattern(items, false)
    }

    #[test]
    fn uart_patterns_with_wildcards_and_gaps() {
        let (words, cfg, _) = uart_wave(b"hello world");
        let frame_end = |k: u64| 100 + 120 * k + 100;
        let frame_start = |k: u64| 100 + 120 * k;
        let fire =
            |m: UartMatch, at| run_at(&ProtocolTrigger::Uart(cfg.clone(), m), &words, 1e6, 64, at);
        let near =
            |hit: Option<u64>, want: u64| hit.is_some_and(|h| (h as i64 - want as i64).abs() <= 10);
        let e = PatItem::exact;
        // Any byte between "l" and " ": "lo " ends at index 5.
        let any = PatItem::Char { value: 0, care: 0 };
        assert!(near(
            fire(pat(vec![e(b'l'), any, e(b' ')]), TriggerAt::End),
            frame_end(5)
        ));
        // One nibble free: 'o' = 0x6F, 'w' = 0x77; "6X 2X 7X" finds "o w" (ends at index 6).
        let nib = |hi: u32| PatItem::Char {
            value: hi << 4,
            care: 0xF0,
        };
        assert!(near(
            fire(pat(vec![nib(6), nib(2), nib(7)]), TriggerAt::End),
            frame_end(6)
        ));
        // Bit by bit: 'w' = 0111 0111.
        assert!(near(
            fire(pat(vec![bits("0111X111")]), TriggerAt::End),
            frame_end(6)
        ));
        assert_eq!(
            fire(pat(vec![bits("1XXXXXXX")]), TriggerAt::End),
            None,
            "no byte >= 0x80"
        );
        // A gap: "h" ... "d" spans the whole text; the trigger at its start is the "h".
        let hd = pat(vec![e(b'h'), PatItem::Gap, e(b'd')]);
        assert!(near(fire(hd.clone(), TriggerAt::End), frame_end(10)));
        assert!(near(fire(hd, TriggerAt::Start), frame_start(0)));
        // A gap may also match nothing: "l * l" finds the "ll" in "hello" (ends at index 3).
        assert!(near(
            fire(pat(vec![e(b'l'), PatItem::Gap, e(b'l')]), TriggerAt::End),
            frame_end(3)
        ));
        // With several possible starts the latest one wins (the shortest match): "o * d" starts at
        // the "o" of "world" (index 7), not of "hello".
        assert!(near(
            fire(pat(vec![e(b'o'), PatItem::Gap, e(b'd')]), TriggerAt::Start),
            frame_start(7)
        ));
    }

    #[test]
    fn gaps_at_the_ends_are_dropped_and_runs_joined() {
        let e = PatItem::exact;
        let g = PatItem::Gap;
        assert_eq!(
            UartMatch::pattern(vec![g, e(1), g, g, e(2), g], true),
            UartMatch::Pattern {
                items: vec![e(1), g, e(2)],
                or_error: true
            }
        );
        // Only gaps: nothing to look for.
        let only = ProtocolTrigger::Uart(UartConfig::default(), UartMatch::pattern(vec![g], false));
        assert!(only.check().is_err());
    }

    #[test]
    fn a_pattern_can_also_fire_on_errors() {
        // "hi" with a broken stop bit in the first character: the sequence is not there, but with
        // or_error the broken character fires.
        let cfg = UartConfig {
            rx: 2,
            baud: 100_000.0,
            ..Default::default()
        };
        let mut w = Wave::new(1e6, 4000, 1 << 2);
        w.uart(2, 100, &cfg, b"hi");
        w.set(2, 100 + 90, 100 + 100, false);
        let m = |or_error| {
            ProtocolTrigger::Uart(
                cfg.clone(),
                UartMatch::pattern(vec![PatItem::exact(b'x')], or_error),
            )
        };
        assert_eq!(run(&m(false), &w.samples, 1e6, 64), None);
        let hit = run(&m(true), &w.samples, 1e6, 64).expect("fires on the error");
        assert!((hit as i64 - 200).abs() <= 10, "{hit}");
    }

    #[test]
    fn an_open_gap_keeps_its_start_in_the_window() {
        // "h" seen, waiting for "#": with the trigger at the start, the look-back reaches back to "h".
        let (words, cfg, end) = uart_wave(b"hello");
        let t = ProtocolTrigger::Uart(
            cfg,
            pat(vec![
                PatItem::exact(b'h'),
                PatItem::Gap,
                PatItem::exact(b'#'),
            ]),
        );
        let mut ev = t.build(TriggerAt::Start).unwrap();
        ev.reset(1e6);
        ev.change(0, words[0]);
        for i in 1..words.len() {
            if words[i] != words[i - 1] {
                assert_eq!(ev.change(i as u64, words[i]), None);
            }
        }
        assert_eq!(ev.advance(3000), None);
        assert!(
            ev.lookback() >= 3000 - 100,
            "{} (h started at 100, end {end})",
            ev.lookback()
        );
    }

    #[test]
    fn uart_errors_are_found() {
        let cfg = UartConfig {
            rx: 2,
            baud: 100_000.0,
            ..Default::default()
        };
        let mut w = Wave::new(1e6, 3000, 1 << 2);
        let a = w.uart(2, 100, &cfg, b"ok");
        // A frame whose stop bit is low: framing error.
        let bad_start = a + 50;
        w.set(2, bad_start, bad_start + 10, false); // start bit
        w.set(2, bad_start + 10, bad_start + 100, false); // data and stop all low
        let words = w.samples.clone();
        let hit = run(
            &ProtocolTrigger::Uart(cfg.clone(), UartMatch::Error),
            &words,
            1e6,
            100,
        )
        .expect("framing error fires");
        assert!(hit > bad_start && hit <= bad_start + 120, "{hit}");
        // Clean data has no errors.
        let (clean, cfg2, _) = uart_wave(b"fine");
        assert_eq!(
            run(
                &ProtocolTrigger::Uart(cfg2, UartMatch::Error),
                &clean,
                1e6,
                100
            ),
            None
        );
    }

    #[test]
    fn spi_words_match_under_a_care_mask() {
        let cfg = SpiConfig {
            sck: 1,
            cs: Some(0),
            mosi: Some(2),
            miso: Some(3),
            ..Default::default()
        };
        let mut w = Wave::new(1e6, 6000, 1);
        w.spi(&cfg, 100, 10, &[0x12, 0xA5, 0x34], &[0xF0, 0x0F, 0x55]);
        let words = w.samples.clone();
        let word = |lane, value, care| {
            ProtocolTrigger::Spi(cfg.clone(), SpiMatch::word(lane, value, care))
        };
        let a5 = run(&word(SpiLane::Mosi, 0xA5, 0xFF), &words, 1e6, 50).expect("A5 on MOSI");
        let low_nibble_5 = run(&word(SpiLane::Mosi, 0x05, 0x0F), &words, 1e6, 50).unwrap();
        assert_eq!(
            a5, low_nibble_5,
            "0xA5 is the first word whose low nibble is 5"
        );
        let first = run(&word(SpiLane::Mosi, 0x12, 0xFF), &words, 1e6, 50).unwrap();
        assert!(first < a5);
        assert_eq!(
            run(&word(SpiLane::Mosi, 0xA5, 0xFF), &words, 1e6, 1).unwrap(),
            a5,
            "buffer size does not matter"
        );
        // The same value on the other line is a different match: MISO sends 0x0F second.
        let m = run(&word(SpiLane::Miso, 0x0F, 0xFF), &words, 1e6, 50).unwrap();
        assert!(m > first && m < run(&word(SpiLane::Miso, 0x55, 0xFF), &words, 1e6, 50).unwrap());
        assert_eq!(run(&word(SpiLane::Mosi, 0x99, 0xFF), &words, 1e6, 50), None);
    }

    #[test]
    fn i2c_conditions_addresses_and_naks() {
        let (scl, sda) = (0u8, 1u8);
        let cfg = I2cConfig { scl, sda };
        let q = 5;
        let mut w = Wave::new(1e6, 4000, 0b11);
        let mut t = 50;
        t = w.i2c_start(scl, sda, t, q, false);
        t = w.i2c_byte(scl, sda, t, q, 0x50 << 1, false); // write to 0x50
        t = w.i2c_byte(scl, sda, t, q, 0x3C, false);
        t = w.i2c_start(scl, sda, t, q, true);
        t = w.i2c_byte(scl, sda, t, q, (0x48 << 1) | 1, false); // read from 0x48
        t = w.i2c_byte(scl, sda, t, q, 0xBE, true); // not acknowledged
        let stop_at = t;
        let _ = w.i2c_stop(scl, sda, t, q);
        let words = w.samples.clone();
        let go = |m| run(&ProtocolTrigger::I2c(cfg.clone(), m), &words, 1e6, 33);
        let start = go(I2cMatch::start()).unwrap();
        let a50 = go(I2cMatch::address(0x50, None)).unwrap();
        let a48 = go(I2cMatch::address(0x48, Some(true))).unwrap();
        let d3c = go(I2cMatch::data(0x3C)).unwrap();
        let nak = go(I2cMatch::nak()).unwrap();
        let stop = go(I2cMatch::stop()).unwrap();
        assert!(
            start < a50 && a50 < d3c && d3c < a48 && a48 < nak && nak <= stop,
            "{start} {a50} {d3c} {a48} {nak} {stop}"
        );
        assert!(stop >= stop_at);
        // Direction is part of the match: 0x50 is only ever written, 0x48 only read.
        assert_eq!(go(I2cMatch::address(0x50, Some(true))), None);
        assert_eq!(go(I2cMatch::address(0x48, Some(false))), None);
        assert_eq!(go(I2cMatch::address(0x77, None)), None);
    }

    #[test]
    fn spi_word_patterns_and_incomplete_words() {
        let cfg = SpiConfig {
            cs: Some(0),
            mosi: Some(2),
            miso: Some(3),
            ..Default::default()
        };
        let mut w = Wave::new(1e6, 6000, 1);
        w.spi(&cfg, 100, 10, &[0x12, 0xA5, 0x34], &[0xF0, 0x0F, 0x55]);
        let words = w.samples.clone();
        let go = |m: SpiMatch| run(&ProtocolTrigger::Spi(cfg.clone(), m), &words, 1e6, 50);
        let ch = |value, care| PatItem::Char { value, care };
        let third = go(SpiMatch::word(SpiLane::Mosi, 0x34, 0xFF)).unwrap();
        // 12, anything, 34: ends with the third word; "AX 3X" too.
        let p = |items| SpiMatch::pattern(SpiLane::Mosi, items, false);
        assert_eq!(
            go(p(vec![ch(0x12, 0xFF), PatItem::Gap, ch(0x34, 0xFF)])),
            Some(third)
        );
        assert_eq!(go(p(vec![ch(0xA0, 0xF0), ch(0x30, 0xF0)])), Some(third));
        assert_eq!(
            go(p(vec![ch(0x34, 0xFF), ch(0x12, 0xFF)])),
            None,
            "wrong order"
        );
        // A word cut short by CS: with or_incomplete it fires although the pattern never matches.
        let mut cut = Wave::new(1e6, 6000, 1);
        cut.spi(&cfg, 100, 10, &[0x12], &[]);
        cut.set(0, 100 + 2 * 10 * 4, 6000, true); // CS released after 4 bits
        let cut = cut.samples.clone();
        let m = |or| SpiMatch::pattern(SpiLane::Mosi, vec![ch(0x99, 0xFF)], or);
        assert_eq!(
            run(&ProtocolTrigger::Spi(cfg.clone(), m(false)), &cut, 1e6, 50),
            None
        );
        assert!(run(&ProtocolTrigger::Spi(cfg.clone(), m(true)), &cut, 1e6, 50).is_some());
    }

    #[test]
    fn i2c_patterns_of_conditions_addresses_and_bytes() {
        let (scl, sda) = (0u8, 1u8);
        let cfg = I2cConfig { scl, sda };
        let q = 5;
        let mut w = Wave::new(1e6, 4000, 0b11);
        let mut t = 50;
        t = w.i2c_start(scl, sda, t, q, false);
        t = w.i2c_byte(scl, sda, t, q, 0x50 << 1, false);
        t = w.i2c_byte(scl, sda, t, q, 0x3C, false);
        t = w.i2c_start(scl, sda, t, q, true);
        t = w.i2c_byte(scl, sda, t, q, (0x48 << 1) | 1, false);
        t = w.i2c_byte(scl, sda, t, q, 0xBE, true);
        let _ = w.i2c_stop(scl, sda, t, q);
        let words = w.samples.clone();
        let go = |items, or_nak| {
            run(
                &ProtocolTrigger::I2c(cfg.clone(), I2cMatch::pattern(items, or_nak)),
                &words,
                1e6,
                33,
            )
        };
        let addr = |value, care, read, nak| PatItem::Address {
            value,
            care,
            read,
            nak,
        };
        let byte = |value, care| PatItem::Char { value, care };
        let nak_byte = |value, care| PatItem::CharNak { value, care };
        let stop = go(vec![PatItem::Stop], false).unwrap();
        let be = go(vec![byte(0xBE, 0xFF)], false).unwrap();
        // The whole transaction, event by event.
        let full = vec![
            PatItem::Start { repeated: false },
            addr(0x50, 0x7F, Some(false), false),
            byte(0x3C, 0xFF),
            PatItem::Start { repeated: true },
            addr(0x48, 0x7F, Some(true), false),
            nak_byte(0xBE, 0xFF),
            PatItem::Stop,
        ];
        assert_eq!(go(full, false), Some(stop));
        // A write to 50, anything, a STOP.
        let w50 = vec![
            addr(0x50, 0x7F, Some(false), false),
            PatItem::Gap,
            PatItem::Stop,
        ];
        assert_eq!(go(w50, false), Some(stop));
        // Wildcards: a read from 4X, then a byte BX that was not acknowledged.
        assert_eq!(
            go(
                vec![addr(0x40, 0x70, Some(true), false), nak_byte(0xB0, 0xF0)],
                false
            ),
            Some(be)
        );
        // 3C was acknowledged; 50 was not NAKed; Sr is only the repeated START.
        assert_eq!(go(vec![nak_byte(0x3C, 0xFF)], false), None);
        assert_eq!(go(vec![addr(0x50, 0x7F, None, true)], false), None);
        let sr = go(vec![PatItem::Start { repeated: true }], false).unwrap();
        assert!(sr > go(vec![PatItem::Start { repeated: false }], false).unwrap());
        // Or any NAK: fires at BE although 77 is never addressed.
        assert_eq!(go(vec![addr(0x77, 0x7F, None, false)], true), Some(be));
    }

    #[test]
    fn pattern_search_reports_where_matches_begin() {
        let ann = |v: u32| Annotation {
            start: 0,
            end: 1,
            value: v,
            lane: 0,
            kind: Kind::Data,
            bits: 8,
            flags: 0,
        };
        let seq: Vec<Annotation> = [1, 2, 3, 1, 9, 3].iter().map(|&v| ann(v)).collect();
        let e = |v: u32| PatItem::Char {
            value: v,
            care: 0xFF,
        };
        // "1 * 3": matches 0..=2 and 3..=5.
        let mut s = PatternSearch::new(vec![e(1), PatItem::Gap, e(3)]).unwrap();
        let ends: Vec<(usize, usize)> = seq
            .iter()
            .enumerate()
            .filter_map(|(i, a)| s.feed(i, a).map(|f| (f, i)))
            .collect();
        assert_eq!(ends, vec![(0, 2), (3, 5)]);
        assert!(s.found_in(&seq[3..]));
        // Matches do not overlap: "1 * 3" in 1 1 3 3 is one match (1..=2, the shortest), not three.
        let seq2: Vec<Annotation> = [1, 1, 3, 3].iter().map(|&v| ann(v)).collect();
        let mut s = PatternSearch::new(vec![e(1), PatItem::Gap, e(3)]).unwrap();
        let found: Vec<(usize, usize)> = seq2
            .iter()
            .enumerate()
            .filter_map(|(i, a)| s.feed(i, a).map(|f| (f, i)))
            .collect();
        assert_eq!(found, vec![(1, 2)]);
        assert!(!PatternSearch::new(vec![e(3), e(1), e(2)])
            .unwrap()
            .found_in(&seq));
        assert!(PatternSearch::new(vec![PatItem::Gap]).is_none());
        // Anchored at the first item (Modbus frames): "1 * 3" begins at 1 2 3 ...; "2 3" does not
        // begin there, although it is inside; a later possible start does not hide the first one.
        let anchored = |items: Vec<PatItem>, seq: &[Annotation]| matches_from_start(&items, seq);
        assert!(anchored(vec![e(1), PatItem::Gap, e(3)], &seq));
        assert!(!anchored(vec![e(2), e(3)], &seq));
        let twice: Vec<Annotation> = [1, 3, 1, 3].iter().map(|&v| ann(v)).collect();
        assert!(anchored(vec![e(1), PatItem::Gap, e(3), e(1)], &twice));
        assert!(anchored(vec![], &seq), "nothing to match: any frame");
    }

    #[test]
    fn modbus_frames_match_from_their_first_byte() {
        use crate::modbus::crc16;
        use crate::uart::{Parity, Protocol};
        let cfg = UartConfig {
            rx: 0,
            baud: 19_200.0,
            parity: Parity::Even,
            protocol: Protocol::ModbusRtu,
            ..Default::default()
        };
        let rate = 1e6;
        let char_samples = rate / 19_200.0 * 11.0;
        let frame = |body: &[u8]| {
            let mut f = body.to_vec();
            f.extend_from_slice(&crc16(body).to_le_bytes());
            f
        };
        // Write 0101 = 0000, then read 0404 x 1, then a frame with a broken CRC.
        let write = frame(&[0x65, 0x06, 0x01, 0x01, 0x00, 0x00]);
        let read = frame(&[0x65, 0x03, 0x04, 0x04, 0x00, 0x01]);
        let mut bad = frame(&[0x11, 0x06, 0x00, 0x01, 0x00, 0x03]);
        bad[3] ^= 0x40;
        let mut w = Wave::new(rate, 60_000, 1);
        let mut t = 1_000;
        let mut ends = Vec::new();
        let mut starts = Vec::new();
        for f in [&write, &read, &bad] {
            starts.push(t);
            t = w.uart(0, t, &cfg, f);
            // The generator leaves two idle bits after each character: the frame ends before.
            ends.push(t - (2.0 * rate / 19_200.0) as u64);
            t += (6.0 * char_samples) as u64;
        }
        let near =
            |hit: Option<u64>, want: u64| hit.is_some_and(|h| (h as i64 - want as i64).abs() <= 5);
        let go = |items: &[PatItem], or_bad, at| {
            run_at(
                &ProtocolTrigger::Uart(
                    cfg.clone(),
                    UartMatch::ModbusFrames {
                        items: items.to_vec(),
                        or_bad,
                    },
                ),
                &w.samples,
                rate,
                64,
                at,
            )
        };
        let e = PatItem::exact;
        // "65 03": the read frame, at its end or its start.
        assert!(near(
            go(&[e(0x65), e(0x03)], false, TriggerAt::End),
            ends[1]
        ));
        assert!(near(
            go(&[e(0x65), e(0x03)], false, TriggerAt::Start),
            starts[1]
        ));
        // Anchored: 03 is the second byte of the read frame, never a first byte.
        assert_eq!(go(&[e(0x03)], false, TriggerAt::End), None);
        // Wildcards and gaps inside a frame: XX 06 * 00 00 is the write frame.
        let any = PatItem::Char { value: 0, care: 0 };
        assert!(near(
            go(
                &[any, e(0x06), PatItem::Gap, e(0x00), e(0x00)],
                false,
                TriggerAt::End
            ),
            ends[0]
        ));
        // The broken frame matches its bytes, but only as a bad frame.
        assert_eq!(go(&[e(0x11)], false, TriggerAt::End), None);
        assert!(near(go(&[e(0x11)], true, TriggerAt::End), ends[2]));
        assert!(near(go(&[], true, TriggerAt::End), ends[2]));
        // Frames need the Modbus protocol in the settings.
        let plain = UartConfig {
            protocol: Protocol::Plain,
            ..cfg.clone()
        };
        let m = UartMatch::ModbusFrames {
            items: vec![e(0x65)],
            or_bad: false,
        };
        assert!(ProtocolTrigger::Uart(plain, m).check().is_err());
    }

    #[test]
    fn settings_are_checked() {
        let u = UartConfig::default();
        assert!(ProtocolTrigger::Uart(u.clone(), UartMatch::bytes(&[]))
            .check()
            .is_err());
        assert!(
            ProtocolTrigger::Uart(u.clone(), UartMatch::bytes(&[0; MAX_SEQUENCE + 1]))
                .check()
                .is_err()
        );
        assert!(ProtocolTrigger::Uart(
            UartConfig {
                data_bits: 7,
                ..u.clone()
            },
            UartMatch::bytes(&[0x80])
        )
        .check()
        .is_err());
        assert!(ProtocolTrigger::Uart(u, UartMatch::Error).check().is_ok());
        let s = SpiConfig {
            miso: None,
            ..Default::default()
        };
        assert!(ProtocolTrigger::Spi(s, SpiMatch::word(SpiLane::Miso, 0, 0))
            .check()
            .is_err());
        assert!(
            ProtocolTrigger::I2c(I2cConfig { scl: 3, sda: 3 }, I2cMatch::start())
                .check()
                .is_err()
        );
        // Items of another protocol, too wide values, nothing to look for.
        let i2c = I2cConfig { scl: 0, sda: 1 };
        assert!(ProtocolTrigger::Uart(
            UartConfig::default(),
            UartMatch::pattern(vec![PatItem::Stop], false)
        )
        .check()
        .is_err());
        assert!(
            ProtocolTrigger::I2c(i2c.clone(), I2cMatch::address(0x80, None))
                .check()
                .is_err()
        );
        assert!(ProtocolTrigger::I2c(
            i2c.clone(),
            I2cMatch::pattern(vec![PatItem::word(0x100)], false)
        )
        .check()
        .is_err());
        assert!(
            ProtocolTrigger::I2c(i2c.clone(), I2cMatch::pattern(vec![], false))
                .check()
                .is_err()
        );
        assert!(ProtocolTrigger::I2c(i2c, I2cMatch::nak()).check().is_ok());
        let spi16 = SpiConfig {
            bits: 16,
            ..Default::default()
        };
        assert!(
            ProtocolTrigger::Spi(spi16.clone(), SpiMatch::word(SpiLane::Mosi, 0xFFFF, 0xFFFF))
                .check()
                .is_ok()
        );
        assert!(
            ProtocolTrigger::Spi(spi16, SpiMatch::word(SpiLane::Mosi, 0x1FFFF, 0x1FFFF))
                .check()
                .is_err()
        );
    }

    #[test]
    fn the_trigger_can_be_the_start_of_the_match() {
        let (words, cfg, _) = uart_wave(b"hello world");
        let seq = ProtocolTrigger::Uart(cfg.clone(), UartMatch::bytes(b"lo w"));
        for chunk in [1usize, 50, 4096] {
            let end = run_at(&seq, &words, 1e6, chunk, TriggerAt::End).unwrap();
            let start = run_at(&seq, &words, 1e6, chunk, TriggerAt::Start).unwrap();
            // The sequence is the characters 3..=6; the first of them starts at 100 + 120 * 3.
            assert!(
                (start as i64 - 460).abs() <= 10,
                "chunk {chunk}: start {start}"
            );
            assert!(end > start + 300, "start {start} end {end}");
        }
        // A single item: the start is where it began, a frame of 100 samples before its end.
        let one = ProtocolTrigger::Uart(cfg, UartMatch::bytes(b"w"));
        let (s, e) = (
            run_at(&one, &words, 1e6, 64, TriggerAt::Start).unwrap(),
            run_at(&one, &words, 1e6, 64, TriggerAt::End).unwrap(),
        );
        assert!((e - s) as i64 >= 90 && (e - s) as i64 <= 110, "{s} {e}");
        // The look-back needed to keep the start in the window grows with the sequence.
        let long = ProtocolTrigger::Uart(UartConfig::default(), UartMatch::bytes(&[0x55; 8]));
        let (mut a, mut b) = (
            long.build(TriggerAt::End).unwrap(),
            long.build(TriggerAt::Start).unwrap(),
        );
        a.reset(1e6);
        b.reset(1e6);
        assert!(b.lookback() >= a.lookback() && b.lookback() > 0);
    }
}
