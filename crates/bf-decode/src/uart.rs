//! Asynchronous serial (UART) decoder.
//!
//! A falling edge on an idle line starts a frame; the start bit is verified in its middle and every
//! following bit is sampled in the middle of its slot. Frames are decoded from edges only, so the cost
//! is proportional to the number of frames (a handful of level lookups each), not to the samples.
//!
//! What happens with the characters beyond lane 0 is up to the configured [`ByteLayer`]: telegrams
//! for plain serial data, or a protocol parser such as Modbus RTU.

use crate::layer::{ByteLayer, CharTiming, Telegrams};
use crate::modbus::ModbusRtu;
use crate::{bit, flag, Annotation, Decoder, Kind};

/// The protocol carried by the characters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Protocol {
    /// Plain characters, optionally joined into telegrams.
    #[default]
    Plain,
    ModbusRtu,
}

impl Protocol {
    pub const ALL: [Protocol; 2] = [Protocol::Plain, Protocol::ModbusRtu];

    pub fn label(self) -> &'static str {
        match self {
            Protocol::Plain => "None (characters)",
            Protocol::ModbusRtu => "Modbus RTU",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Parity {
    None,
    Even,
    Odd,
    /// Parity bit always 1.
    Mark,
    /// Parity bit always 0.
    Space,
}

impl Parity {
    pub const ALL: [Parity; 5] = [
        Parity::None,
        Parity::Even,
        Parity::Odd,
        Parity::Mark,
        Parity::Space,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Parity::None => "None",
            Parity::Even => "Even",
            Parity::Odd => "Odd",
            Parity::Mark => "Mark",
            Parity::Space => "Space",
        }
    }
}

/// Unit of the telegram gap.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum GapUnit {
    /// Bit times (setups saved before the unit existed are in bit times).
    #[default]
    Bits,
    /// Character times: start, data, parity and stop bits of the configured format.
    Chars,
}

impl GapUnit {
    pub const ALL: [GapUnit; 2] = [GapUnit::Bits, GapUnit::Chars];

    pub fn label(self) -> &'static str {
        match self {
            GapUnit::Bits => "bit times",
            GapUnit::Chars => "characters",
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct UartConfig {
    pub rx: u8,
    pub baud: f64,
    /// 5 to 9.
    pub data_bits: u8,
    pub parity: Parity,
    /// 1, 1.5 or 2.
    pub stop_bits: f32,
    pub lsb_first: bool,
    /// The idle level is low (inverted signal levels, e.g. RS-232 without a transceiver).
    pub invert: bool,
    /// Group consecutive characters into telegrams (lane 1).
    pub telegrams: bool,
    /// A telegram ends when the line has been idle this long (in `telegram_gap_unit`).
    pub telegram_gap: f64,
    pub telegram_gap_unit: GapUnit,
    /// ... or right after a line feed (0x0A).
    pub telegram_lf: bool,
    /// The protocol on top of the characters (telegram settings apply to `Plain` only).
    pub protocol: Protocol,
}

impl UartConfig {
    /// Bits per character on the wire: start, data, parity and stop bits.
    pub fn char_bits(&self) -> f64 {
        1.0 + self.data_bits as f64
            + (self.parity != Parity::None) as u8 as f64
            + self.stop_bits as f64
    }

    /// The telegram gap in bit times, whatever unit it is set in.
    pub fn telegram_gap_bits(&self) -> f64 {
        match self.telegram_gap_unit {
            GapUnit::Bits => self.telegram_gap,
            GapUnit::Chars => self.telegram_gap * self.char_bits(),
        }
    }
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            rx: 0,
            baud: 115_200.0,
            data_bits: 8,
            parity: Parity::None,
            stop_bits: 1.0,
            lsb_first: true,
            invert: false,
            telegrams: true,
            telegram_gap: 24.0,
            telegram_gap_unit: GapUnit::Bits,
            telegram_lf: true,
            protocol: Protocol::Plain,
        }
    }
}

struct Frame {
    t0: u64,
    /// Index of the next bit to sample: 0 start, 1..=n data, then parity, then stop bit(s).
    j: u32,
    value: u32,
    ones: u32,
    parity_bit: bool,
    flags: u8,
}

pub struct Uart {
    cfg: UartConfig,
    layer: Box<dyn ByteLayer>,
    /// Samples per bit.
    bit: f64,
    primed: bool,
    /// Logical level (true = mark / idle).
    level: bool,
    frame: Option<Frame>,
}

impl Uart {
    pub fn new(cfg: UartConfig) -> Self {
        let layer: Box<dyn ByteLayer> = match cfg.protocol {
            Protocol::Plain => Box::new(Telegrams::new(
                cfg.telegrams,
                cfg.telegram_gap_bits(),
                cfg.telegram_lf,
            )),
            Protocol::ModbusRtu => Box::new(ModbusRtu::new()),
        };
        Self {
            cfg,
            layer,
            bit: 0.0,
            primed: false,
            level: true,
            frame: None,
        }
    }

    pub fn config(&self) -> &UartConfig {
        &self.cfg
    }

    fn parity_bits(&self) -> u32 {
        (self.cfg.parity != Parity::None) as u32
    }

    fn stop_samples(&self) -> u32 {
        // 1.5 stop bits are checked like one (the level is sampled in the first stop bit).
        if self.cfg.stop_bits >= 2.0 {
            2
        } else {
            1
        }
    }
}

impl Decoder for Uart {
    fn name(&self) -> &'static str {
        "UART"
    }

    fn lanes(&self) -> &'static [&'static str] {
        self.layer.lanes()
    }

    fn lane_used(&self, lane: usize) -> bool {
        lane == 0 || self.layer.lane_used(lane)
    }

    fn mask(&self) -> u32 {
        1 << self.cfg.rx
    }

    fn reset(&mut self, sample_rate: f64) {
        self.bit = sample_rate / self.cfg.baud.max(1.0);
        self.primed = false;
        self.level = true;
        self.frame = None;
        self.layer.reset(CharTiming {
            sample_rate,
            bit: self.bit,
            char_bits: self.cfg.char_bits(),
        });
    }

    fn change(&mut self, t: u64, state: u32, out: &mut Vec<Annotation>) {
        let lvl = bit(state, self.cfg.rx) ^ self.cfg.invert;
        if !self.primed {
            self.primed = true;
            self.level = lvl;
            return;
        }
        self.advance(t, out);
        if self.frame.is_none() && self.level && !lvl {
            self.frame = Some(Frame {
                t0: t,
                j: 0,
                value: 0,
                ones: 0,
                parity_bit: false,
                flags: 0,
            });
        }
        self.level = lvl;
    }

    fn advance(&mut self, end: u64, out: &mut Vec<Annotation>) {
        self.advance_frames(end, out);
        // The line is idle up to `end`, or up to the start of the frame in progress.
        let until = self.frame.as_ref().map_or(end, |f| f.t0.min(end));
        self.layer.idle(until, out);
    }
}

impl Uart {
    fn advance_frames(&mut self, end: u64, out: &mut Vec<Annotation>) {
        let n = self.cfg.data_bits as u32;
        let parity = self.parity_bits();
        let stops = self.stop_samples();
        loop {
            let Some(f) = self.frame.as_mut() else { return };
            let when = f.t0 as f64 + (f.j as f64 + 0.5) * self.bit;
            if when.round() as u64 >= end {
                return;
            }
            let lvl = self.level;
            let j = f.j;
            if j == 0 {
                if lvl {
                    // The start bit did not last: a glitch.
                    self.frame = None;
                    return;
                }
            } else if j <= n {
                let k = j - 1;
                if self.cfg.lsb_first {
                    f.value |= (lvl as u32) << k;
                } else {
                    f.value = (f.value << 1) | lvl as u32;
                }
                f.ones += lvl as u32;
            } else if parity == 1 && j == n + 1 {
                f.parity_bit = lvl;
            } else {
                if !lvl {
                    f.flags |= flag::FRAMING;
                }
                let first_stop = n + 1 + parity;
                if j + 1 >= first_stop + stops {
                    // Last stop bit: the frame is complete.
                    let mut flags = f.flags;
                    let ok = match self.cfg.parity {
                        Parity::None => true,
                        Parity::Even => (f.ones + f.parity_bit as u32) % 2 == 0,
                        Parity::Odd => (f.ones + f.parity_bit as u32) % 2 == 1,
                        Parity::Mark => f.parity_bit,
                        Parity::Space => !f.parity_bit,
                    };
                    if !ok {
                        flags |= flag::PARITY;
                    }
                    if flags & flag::FRAMING != 0 && f.value == 0 {
                        flags |= flag::BREAK;
                    }
                    let slots = 1.0 + n as f64 + parity as f64 + self.cfg.stop_bits as f64;
                    let (start, stop, value) = (
                        f.t0,
                        (f.t0 as f64 + slots * self.bit).round() as u64,
                        f.value,
                    );
                    let c = Annotation {
                        start,
                        end: stop,
                        value,
                        lane: 0,
                        kind: Kind::Data,
                        bits: self.cfg.data_bits,
                        flags,
                    };
                    out.push(c);
                    self.frame = None;
                    self.layer.char(&c, out);
                    return;
                }
            }
            f.j += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::decode_all;
    use crate::synth::Wave;
    use bf_core::{CaptureStore, ChannelCount, StoreConfig};

    const RATE: f64 = 10e6;

    fn store(w: &Wave) -> CaptureStore {
        let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(8).unwrap(), RATE));
        s.push_samples(&w.samples);
        s
    }

    /// The characters (lane 0).
    fn decode(cfg: UartConfig, w: &Wave) -> Vec<Annotation> {
        decode_lanes(cfg, w)
            .into_iter()
            .filter(|a| a.lane == 0)
            .collect()
    }

    fn decode_lanes(cfg: UartConfig, w: &Wave) -> Vec<Annotation> {
        decode_all(Box::new(Uart::new(cfg)), &store(w))
    }

    fn telegrams(a: &[Annotation]) -> Vec<&Annotation> {
        a.iter().filter(|a| a.kind == Kind::Telegram).collect()
    }

    fn text(a: &[Annotation], t: &Annotation) -> String {
        let chars: Vec<Annotation> = a.iter().filter(|a| a.lane == 0).copied().collect();
        crate::telegram_text(crate::telegram_chars(&chars, t), crate::Format::Ascii, 1000)
    }

    fn values(a: &[Annotation]) -> Vec<u32> {
        a.iter().map(|a| a.value).collect()
    }

    #[test]
    fn decodes_8n1_text() {
        let mut w = Wave::new(RATE, 40_000, 0b1);
        let cfg = UartConfig::default();
        w.uart(0, 100, &cfg, b"Hello, UART!");
        let a = decode(cfg, &w);
        assert_eq!(
            values(&a),
            b"Hello, UART!"
                .iter()
                .map(|&b| b as u32)
                .collect::<Vec<_>>()
        );
        assert!(a.iter().all(|a| a.flags == 0 && a.lane == 0 && a.bits == 8));
        // The first frame spans start bit to stop bit: 10 bits of 86.8 samples.
        assert_eq!(a[0].start, 100);
        assert!((a[0].end as i64 - (100.0 + 10.0 * RATE / 115_200.0) as i64).abs() <= 1);
    }

    #[test]
    fn other_word_formats() {
        for (bits, parity, stop, lsb, invert) in [
            (7, Parity::Even, 1.0, true, false),
            (8, Parity::Odd, 2.0, true, false),
            (5, Parity::None, 1.5, true, false),
            (8, Parity::Mark, 1.0, false, false),
            (9, Parity::Space, 1.0, true, true),
        ] {
            let cfg = UartConfig {
                data_bits: bits,
                parity,
                stop_bits: stop,
                lsb_first: lsb,
                invert,
                baud: 250_000.0,
                ..Default::default()
            };
            let mask = (1u32 << bits) - 1;
            let data: Vec<u8> = (0..20u32).map(|i| ((i * 37 + 5) & mask) as u8).collect();
            let mut w = Wave::new(RATE, 20_000, if invert { 0 } else { 1 });
            w.uart(0, 50, &cfg, &data);
            let a = decode(cfg.clone(), &w);
            assert_eq!(
                values(&a),
                data.iter().map(|&b| b as u32).collect::<Vec<_>>(),
                "{cfg:?}"
            );
            assert!(a.iter().all(|a| a.flags == 0), "{cfg:?}: {a:?}");
        }
    }

    #[test]
    fn parity_error_is_flagged_but_the_byte_is_kept() {
        let cfg = UartConfig {
            parity: Parity::Even,
            ..Default::default()
        };
        let mut w = Wave::new(RATE, 10_000, 1);
        w.uart(0, 100, &cfg, &[0x41, 0x42]);
        // Flip the parity bit of the first frame (bit slot 9, i.e. 1 + 8).
        let t = (100.0 + 9.5 * RATE / cfg.baud) as u64;
        let lvl = w.samples[t as usize] & 1 != 0;
        let slot = (RATE / cfg.baud) as u64;
        w.set(0, t - slot / 3, t + slot / 3, !lvl);
        let a = decode(cfg, &w);
        assert_eq!(values(&a), vec![0x41, 0x42]);
        assert_ne!(a[0].flags & flag::PARITY, 0);
        assert_eq!(a[1].flags, 0);
    }

    #[test]
    fn framing_error_and_break() {
        let cfg = UartConfig::default();
        let bit = (RATE / cfg.baud) as u64;
        // A frame whose stop bit is low, then a long break (line held low), then a normal frame.
        let mut w = Wave::new(RATE, 30_000, 1);
        w.uart(0, 100, &cfg, &[0x55]);
        let stop_start = 100 + 9 * bit + 2;
        w.set(0, stop_start, stop_start + bit, false);
        w.set(0, 2_000, 3_000, false); // break: low for ~11 bit times
        w.uart(0, 5_000, &cfg, &[0x33]);
        let a = decode(cfg, &w);
        assert_eq!(a.len(), 3, "{a:?}");
        assert_eq!(a[0].value, 0x55);
        assert_ne!(a[0].flags & flag::FRAMING, 0);
        assert_eq!(a[0].flags & flag::BREAK, 0);
        assert_eq!(a[1].value, 0);
        assert_ne!(a[1].flags & flag::BREAK, 0);
        assert_eq!(a[2].value, 0x33);
        assert_eq!(a[2].flags, 0);
    }

    #[test]
    fn glitches_on_an_idle_line_are_ignored() {
        let cfg = UartConfig::default();
        let mut w = Wave::new(RATE, 10_000, 1);
        w.set(0, 500, 520, false); // 2 us glitch, far shorter than half a bit
        w.uart(0, 2_000, &cfg, &[0xA5]);
        assert_eq!(values(&decode(cfg, &w)), vec![0xA5]);
    }

    #[test]
    fn incremental_decoding_matches_decoding_in_one_go() {
        // Feed the same capture in awkward slices: a frame must be decoded exactly once, even when a
        // slice boundary falls inside it.
        use crate::engine::Runner;
        let cfg = UartConfig::default();
        let mut w = Wave::new(RATE, 60_000, 1);
        w.uart(0, 100, &cfg, b"incremental decoding test");
        let whole = decode_lanes(cfg.clone(), &w);
        for chunk in [1usize, 7, 100, 333, 1000, 4096] {
            let mut store =
                CaptureStore::new(StoreConfig::new(ChannelCount::new(8).unwrap(), RATE));
            let mut runner = Runner::new(Box::new(Uart::new(cfg.clone())));
            let mut got = Vec::new();
            for part in w.samples.chunks(chunk) {
                store.push_samples(part);
                while runner.pump(&store, 77, &mut got).is_some() {}
            }
            assert_eq!(got, whole, "chunk {chunk}");
        }
    }

    #[test]
    fn characters_separated_by_a_long_idle_time_form_separate_telegrams() {
        let cfg = UartConfig::default();
        let bit = RATE / cfg.baud;
        let mut w = Wave::new(RATE, 120_000, 1);
        let end = w.uart(0, 100, &cfg, b"HELLO");
        // 60 idle bit times between the telegrams (the gap limit is 24).
        let start2 = end + (60.0 * bit) as u64;
        w.uart(0, start2, &cfg, b"WORLD");
        let a = decode_lanes(cfg, &w);
        let t = telegrams(&a);
        assert_eq!(t.len(), 2, "{a:?}");
        assert_eq!(text(&a, t[0]), "HELLO");
        assert_eq!(text(&a, t[1]), "WORLD");
        assert_eq!((t[0].value, t[1].value), (5, 5));
        let chars: Vec<_> = a.iter().filter(|a| a.lane == 0).collect();
        // A telegram spans its first character start to its last character end.
        assert_eq!((t[0].start, t[0].end), (chars[0].start, chars[4].end));
        assert_eq!((t[1].start, t[1].end), (chars[5].start, chars[9].end));
        // Sorted and not overlapping.
        assert!(t[0].end <= t[1].start);
        assert_eq!(t[0].lane, 1);
    }

    #[test]
    fn a_line_feed_ends_a_telegram_even_without_a_gap() {
        let cfg = UartConfig::default();
        let mut w = Wave::new(RATE, 60_000, 1);
        w.uart(0, 100, &cfg, b"AB\nCD\r\n");
        let a = decode_lanes(cfg.clone(), &w);
        let t = telegrams(&a);
        assert_eq!(
            t.iter().map(|t| text(&a, t)).collect::<Vec<_>>(),
            vec!["AB\\n".to_string(), "CD\\r\\n".to_string()]
        );
        // Without the option the idle gap decides, and these characters follow closely.
        let a = decode_lanes(
            UartConfig {
                telegram_lf: false,
                ..cfg
            },
            &w,
        );
        let t = telegrams(&a);
        assert_eq!(t.len(), 1);
        assert_eq!(text(&a, t[0]), "AB\\nCD\\r\\n");
    }

    #[test]
    fn the_gap_can_be_given_in_characters() {
        assert_eq!(UartConfig::default().char_bits(), 10.0);
        let chars = UartConfig {
            telegram_gap: 2.4,
            telegram_gap_unit: GapUnit::Chars,
            ..Default::default()
        };
        assert!((chars.telegram_gap_bits() - 24.0).abs() < 1e-9);
        let e81 = UartConfig {
            parity: Parity::Even,
            ..chars.clone()
        };
        assert!((e81.telegram_gap_bits() - 2.4 * 11.0).abs() < 1e-9);
        // 30 idle bit times between two words: split at 24 bits (2.4 characters), not at 4 characters.
        let plain = UartConfig::default();
        let bit = RATE / plain.baud;
        let mut w = Wave::new(RATE, 120_000, 1);
        let end = w.uart(0, 100, &plain, b"HELLO");
        w.uart(0, end + (30.0 * bit) as u64, &plain, b"WORLD");
        let n = |c: UartConfig| telegrams(&decode_lanes(c, &w)).len();
        assert_eq!(n(plain), 2);
        assert_eq!(n(chars.clone()), 2);
        assert_eq!(
            n(UartConfig {
                telegram_gap: 4.0,
                ..chars
            }),
            1
        );
    }

    #[test]
    fn telegrams_can_be_switched_off() {
        let cfg = UartConfig {
            telegrams: false,
            ..Default::default()
        };
        let mut w = Wave::new(RATE, 20_000, 1);
        w.uart(0, 100, &cfg, b"abc");
        assert!(telegrams(&decode_lanes(cfg.clone(), &w)).is_empty());
        assert!(!Uart::new(cfg).lane_used(1));
        assert!(Uart::new(UartConfig::default()).lane_used(1));
    }

    #[test]
    fn a_telegram_is_complete_only_once_the_gap_has_passed() {
        let cfg = UartConfig::default();
        let bit = RATE / cfg.baud;
        let mut w = Wave::new(RATE, 100_000, 1);
        let end = w.uart(0, 100, &cfg, b"XY") as usize;
        // The capture ends 10 bit times after the last character: the telegram may still continue.
        let early = Wave {
            rate: RATE,
            samples: w.samples[..end + (10.0 * bit) as usize].to_vec(),
        };
        assert!(telegrams(&decode_lanes(cfg.clone(), &early)).is_empty());
        // 40 bit times of idle line: complete.
        let late = Wave {
            rate: RATE,
            samples: w.samples[..end + (40.0 * bit) as usize].to_vec(),
        };
        assert_eq!(telegrams(&decode_lanes(cfg, &late)).len(), 1);
    }

    #[test]
    fn a_character_error_marks_its_telegram() {
        let cfg = UartConfig {
            parity: Parity::Even,
            ..Default::default()
        };
        let mut w = Wave::new(RATE, 40_000, 1);
        w.uart(0, 100, &cfg, b"OK");
        let slot = (RATE / cfg.baud) as u64;
        let t = (100.0 + 9.5 * RATE / cfg.baud) as u64;
        let lvl = w.samples[t as usize] & 1 != 0;
        w.set(0, t - slot / 3, t + slot / 3, !lvl); // wrong parity bit in the first character
        let a = decode_lanes(cfg, &w);
        let t = telegrams(&a);
        assert_eq!(t.len(), 1);
        assert_ne!(t[0].flags & flag::PARITY, 0);
        assert!(t[0].is_error());
    }
}
