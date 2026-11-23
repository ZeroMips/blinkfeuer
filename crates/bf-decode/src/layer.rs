//! Byte layers: what the UART decoder does with its characters beyond showing them.
//!
//! The UART decoder turns edges into characters (lane 0) and hands every character to its layer, which
//! adds annotations on lanes 1 and up: joining characters into telegrams, or parsing a protocol such as
//! Modbus RTU. A layer only sees characters and idle time, so it does not depend on how they were
//! decoded.

use crate::{flag, Annotation, Kind};

/// Timing of the character stream a layer is fed with.
#[derive(Clone, Copy, Debug)]
pub struct CharTiming {
    pub sample_rate: f64,
    /// Samples per bit.
    pub bit: f64,
    /// Bits per character on the wire (start, data, parity and stop bits).
    pub char_bits: f64,
}

impl CharTiming {
    pub fn baud(&self) -> f64 {
        self.sample_rate / self.bit
    }

    /// Samples per character.
    pub fn char_samples(&self) -> f64 {
        self.bit * self.char_bits
    }
}

pub trait ByteLayer: Send {
    /// All lanes of the decoder with this layer, lane 0 being the characters.
    fn lanes(&self) -> &'static [&'static str];
    /// Whether anything is ever drawn on `lane` (from 1).
    fn lane_used(&self, _lane: usize) -> bool {
        true
    }
    /// Start over (new capture).
    fn reset(&mut self, timing: CharTiming);
    /// A character was decoded (it is already on lane 0).
    fn char(&mut self, c: &Annotation, out: &mut Vec<Annotation>);
    /// No character starts before `until`: a gap long enough to end a message may be over.
    fn idle(&mut self, until: u64, out: &mut Vec<Annotation>);
}

/// Joins consecutive characters into telegrams (lane 1): a telegram ends when the line has been idle
/// for a number of bit times, and optionally right after a line feed.
pub struct Telegrams {
    enabled: bool,
    gap_bits: f64,
    end_at_lf: bool,
    gap: u64,
    tele: Option<Tele>,
}

/// A telegram being collected.
struct Tele {
    start: u64,
    end: u64,
    count: u32,
    flags: u8,
}

impl Telegrams {
    pub fn new(enabled: bool, gap_bits: f64, end_at_lf: bool) -> Self {
        Self {
            enabled,
            gap_bits,
            end_at_lf,
            gap: 0,
            tele: None,
        }
    }

    fn close(&mut self, out: &mut Vec<Annotation>) {
        if let Some(t) = self.tele.take() {
            out.push(Annotation {
                start: t.start,
                end: t.end,
                value: t.count,
                lane: 1,
                kind: Kind::Telegram,
                bits: 0,
                flags: t.flags,
            });
        }
    }
}

impl ByteLayer for Telegrams {
    fn lanes(&self) -> &'static [&'static str] {
        &["RX", "Telegram"]
    }

    fn lane_used(&self, _lane: usize) -> bool {
        self.enabled
    }

    fn reset(&mut self, timing: CharTiming) {
        self.gap = (self.gap_bits * timing.bit) as u64;
        self.tele = None;
    }

    fn char(&mut self, c: &Annotation, out: &mut Vec<Annotation>) {
        if !self.enabled {
            return;
        }
        if self
            .tele
            .as_ref()
            .is_some_and(|t| c.start.saturating_sub(t.end) >= self.gap)
        {
            self.close(out);
        }
        let flags = c.flags & flag::ERROR;
        match &mut self.tele {
            Some(t) => {
                t.end = c.end;
                t.count += 1;
                t.flags |= flags;
            }
            None => {
                self.tele = Some(Tele {
                    start: c.start,
                    end: c.end,
                    count: 1,
                    flags,
                })
            }
        }
        if self.end_at_lf && c.value == 0x0A {
            self.close(out);
        }
    }

    fn idle(&mut self, until: u64, out: &mut Vec<Annotation>) {
        if self
            .tele
            .as_ref()
            .is_some_and(|t| until.saturating_sub(t.end) >= self.gap)
        {
            self.close(out);
        }
    }
}
