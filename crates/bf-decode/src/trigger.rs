//! Protocol triggers: a decoder runs on the incoming sample stream and the trigger fires when it
//! decodes what was asked for (a UART byte sequence, an SPI word, an I2C address, ...).
//!
//! The decoders are event driven, so the work is the scan for changes of their channels plus a few
//! operations per edge. The trigger sample is the end of the matched item.

use crate::i2c::{I2c, I2cConfig};
use crate::spi::{Spi, SpiConfig};
use crate::uart::{Uart, UartConfig};
use crate::{flag, Annotation, Decoder, Kind};
use bf_core::EventTrigger;
use serde::{Deserialize, Serialize};

/// What to look for on a UART.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum UartMatch {
    /// These characters, one right after the other.
    Bytes(Vec<u8>),
    /// A framing error, a parity error or a break.
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SpiLane {
    Mosi,
    Miso,
}

/// What to look for on an SPI bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SpiMatch {
    /// A word on the given line; only the bits set in `care` have to match `value`.
    Word {
        lane: SpiLane,
        value: u32,
        care: u32,
    },
}

/// What to look for on an I2C bus.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum I2cMatch {
    Start,
    Stop,
    /// A 7-bit address; `read`: `Some(true)` only reads, `Some(false)` only writes.
    Address {
        addr: u8,
        read: Option<bool>,
    },
    /// Any byte that was not acknowledged.
    Nak,
    /// A data byte (not an address) with this value.
    Data(u8),
}

/// Which sample of the matched item is the trigger sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum TriggerAt {
    /// The first sample of the match (for a byte sequence: of its first byte).
    Start,
    /// The end of the match (the default).
    #[default]
    End,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ProtocolTrigger {
    Uart(UartConfig, UartMatch),
    Spi(SpiConfig, SpiMatch),
    I2c(I2cConfig, I2cMatch),
}

/// Longest byte sequence a UART trigger takes.
pub const MAX_SEQUENCE: usize = 16;

enum Matcher {
    UartBytes(Vec<u8>),
    UartError,
    SpiWord { lane: u8, value: u32, care: u32 },
    I2c(I2cMatch),
}

struct DecoderTrigger {
    dec: Box<dyn Decoder>,
    matcher: Matcher,
    at: TriggerAt,
    out: Vec<Annotation>,
    /// The last characters decoded with the sample each started at (UART sequences).
    recent: Vec<(u8, u64)>,
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
        for a in self.out.drain(..) {
            self.max_len = self.max_len.max(a.end.saturating_sub(a.start));
            // The match begins at the first character of a sequence, at the item itself otherwise.
            let mut begins = a.start;
            let m = match &self.matcher {
                Matcher::UartBytes(seq) => {
                    if a.lane != 0 || a.kind != Kind::Data {
                        false
                    } else {
                        self.recent.push((a.value as u8, a.start));
                        if self.recent.len() > seq.len() {
                            self.recent.remove(0);
                        }
                        let hit = self.recent.len() == seq.len()
                            && self.recent.iter().map(|c| c.0).eq(seq.iter().copied());
                        if hit {
                            begins = self.recent[0].1;
                        }
                        hit
                    }
                }
                Matcher::UartError => a.lane == 0 && a.kind == Kind::Data && a.is_error(),
                Matcher::SpiWord { lane, value, care } => {
                    a.kind == Kind::Data && a.lane == *lane && (a.value ^ value) & care == 0
                }
                Matcher::I2c(m) => match (m, a.kind) {
                    (I2cMatch::Start, Kind::Start) | (I2cMatch::Stop, Kind::Stop) => true,
                    (I2cMatch::Address { addr, read }, Kind::Address) => {
                        a.value == *addr as u32
                            && read.is_none_or(|r| r == (a.flags & flag::READ != 0))
                    }
                    (I2cMatch::Nak, Kind::Address | Kind::Data) => a.flags & flag::NAK != 0,
                    (I2cMatch::Data(v), Kind::Data) => a.value == *v as u32,
                    _ => false,
                },
            };
            if m && hit.is_none() {
                hit = Some(match self.at {
                    TriggerAt::Start => begins,
                    TriggerAt::End => a.end,
                });
            }
        }
        hit
    }
}

impl EventTrigger for DecoderTrigger {
    fn mask(&self) -> u32 {
        self.dec.mask()
    }

    fn reset(&mut self, sample_rate: f64) {
        self.dec.reset(sample_rate);
        self.out.clear();
        self.recent.clear();
        self.max_len = 0;
        self.lookback = (self.lookback_bits * sample_rate / self.bit_rate.max(1.0)) as u64;
    }

    fn change(&mut self, t: u64, state: u32) -> Option<u64> {
        self.dec.change(t, state, &mut self.out);
        self.scan()
    }

    fn advance(&mut self, end: u64) -> Option<u64> {
        self.dec.advance(end, &mut self.out);
        self.scan()
    }

    fn lookback(&self) -> u64 {
        match self.at {
            TriggerAt::End => self.lookback,
            // The trigger sample is the start of an item that is only recognised at its end.
            TriggerAt::Start => self.lookback.max(2 * self.max_len),
        }
    }
}

impl ProtocolTrigger {
    /// Problems with the settings that make the trigger useless, if any.
    pub fn check(&self) -> Result<(), String> {
        match self {
            ProtocolTrigger::Uart(c, UartMatch::Bytes(b)) => {
                if b.is_empty() {
                    return Err("enter at least one byte to look for".into());
                }
                if b.len() > MAX_SEQUENCE {
                    return Err(format!("at most {MAX_SEQUENCE} bytes"));
                }
                if c.data_bits < 8 && b.iter().any(|v| *v >> c.data_bits != 0) {
                    return Err(format!("a byte does not fit in {} data bits", c.data_bits));
                }
                Ok(())
            }
            ProtocolTrigger::Uart(..) => Ok(()),
            ProtocolTrigger::Spi(c, SpiMatch::Word { lane, .. }) => {
                let wired = match lane {
                    SpiLane::Mosi => c.mosi,
                    SpiLane::Miso => c.miso,
                };
                if wired.is_none() {
                    return Err("the line to look at has no channel".into());
                }
                Ok(())
            }
            ProtocolTrigger::I2c(c, _) => {
                if c.scl == c.sda {
                    return Err("SCL and SDA are the same channel".into());
                }
                Ok(())
            }
        }
    }

    /// The trigger as something the capture thread can run.
    pub fn build(&self, at: TriggerAt) -> Result<Box<dyn EventTrigger>, String> {
        self.check()?;
        let (dec, matcher, lookback_bits, bit_rate): (Box<dyn Decoder>, Matcher, f64, f64) =
            match self {
                ProtocolTrigger::Uart(c, m) => {
                    let mut cfg = c.clone();
                    cfg.telegrams = false;
                    let frame_bits = 1.0
                        + cfg.data_bits as f64
                        + if cfg.parity == crate::uart::Parity::None {
                            0.0
                        } else {
                            1.0
                        }
                        + cfg.stop_bits as f64;
                    let matcher = match m {
                        UartMatch::Bytes(b) => Matcher::UartBytes(b.clone()),
                        UartMatch::Error => Matcher::UartError,
                    };
                    let chars = if let UartMatch::Bytes(b) = m {
                        b.len() as f64
                    } else {
                        1.0
                    };
                    (
                        Box::new(Uart::new(cfg.clone())),
                        matcher,
                        chars * (frame_bits + 2.0) + 2.0,
                        cfg.baud,
                    )
                }
                ProtocolTrigger::Spi(c, SpiMatch::Word { lane, value, care }) => {
                    let lane = match lane {
                        SpiLane::Mosi => 0,
                        SpiLane::Miso => 1,
                    };
                    (
                        Box::new(Spi::new(c.clone())),
                        Matcher::SpiWord {
                            lane,
                            value: *value,
                            care: *care,
                        },
                        0.0,
                        1.0,
                    )
                }
                ProtocolTrigger::I2c(c, m) => {
                    (Box::new(I2c::new(c.clone())), Matcher::I2c(*m), 0.0, 1.0)
                }
            };
        Ok(Box::new(DecoderTrigger {
            dec,
            matcher,
            at,
            out: Vec::new(),
            recent: Vec::new(),
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
            let t = ProtocolTrigger::Uart(cfg.clone(), UartMatch::Bytes(b"lo w".to_vec()));
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
                &ProtocolTrigger::Uart(cfg.clone(), UartMatch::Bytes(b"xyz".to_vec())),
                &words,
                1e6,
                64
            ),
            None
        );
        let one = run(
            &ProtocolTrigger::Uart(cfg.clone(), UartMatch::Bytes(b"h".to_vec())),
            &words,
            1e6,
            64,
        )
        .unwrap();
        assert!((one as i64 - frame_end(0) as i64).abs() <= 10);
        // Bytes must follow each other directly: "h" then "l" is not in "hello world".
        assert_eq!(
            run(
                &ProtocolTrigger::Uart(cfg, UartMatch::Bytes(b"hl".to_vec())),
                &words,
                1e6,
                64
            ),
            None
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
            ProtocolTrigger::Spi(cfg.clone(), SpiMatch::Word { lane, value, care })
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
        let start = go(I2cMatch::Start).unwrap();
        let a50 = go(I2cMatch::Address {
            addr: 0x50,
            read: None,
        })
        .unwrap();
        let a48 = go(I2cMatch::Address {
            addr: 0x48,
            read: Some(true),
        })
        .unwrap();
        let d3c = go(I2cMatch::Data(0x3C)).unwrap();
        let nak = go(I2cMatch::Nak).unwrap();
        let stop = go(I2cMatch::Stop).unwrap();
        assert!(
            start < a50 && a50 < d3c && d3c < a48 && a48 < nak && nak <= stop,
            "{start} {a50} {d3c} {a48} {nak} {stop}"
        );
        assert!(stop >= stop_at);
        // Direction is part of the match: 0x50 is only ever written, 0x48 only read.
        assert_eq!(
            go(I2cMatch::Address {
                addr: 0x50,
                read: Some(true)
            }),
            None
        );
        assert_eq!(
            go(I2cMatch::Address {
                addr: 0x48,
                read: Some(false)
            }),
            None
        );
        assert_eq!(
            go(I2cMatch::Address {
                addr: 0x77,
                read: None
            }),
            None
        );
    }

    #[test]
    fn settings_are_checked() {
        let u = UartConfig::default();
        assert!(ProtocolTrigger::Uart(u.clone(), UartMatch::Bytes(vec![]))
            .check()
            .is_err());
        assert!(
            ProtocolTrigger::Uart(u.clone(), UartMatch::Bytes(vec![0; MAX_SEQUENCE + 1]))
                .check()
                .is_err()
        );
        assert!(ProtocolTrigger::Uart(
            UartConfig {
                data_bits: 7,
                ..u.clone()
            },
            UartMatch::Bytes(vec![0x80])
        )
        .check()
        .is_err());
        assert!(ProtocolTrigger::Uart(u, UartMatch::Error).check().is_ok());
        let s = SpiConfig {
            miso: None,
            ..Default::default()
        };
        assert!(ProtocolTrigger::Spi(
            s,
            SpiMatch::Word {
                lane: SpiLane::Miso,
                value: 0,
                care: 0
            }
        )
        .check()
        .is_err());
        assert!(
            ProtocolTrigger::I2c(I2cConfig { scl: 3, sda: 3 }, I2cMatch::Start)
                .check()
                .is_err()
        );
    }

    #[test]
    fn the_trigger_can_be_the_start_of_the_match() {
        let (words, cfg, _) = uart_wave(b"hello world");
        let seq = ProtocolTrigger::Uart(cfg.clone(), UartMatch::Bytes(b"lo w".to_vec()));
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
        let one = ProtocolTrigger::Uart(cfg, UartMatch::Bytes(b"w".to_vec()));
        let (s, e) = (
            run_at(&one, &words, 1e6, 64, TriggerAt::Start).unwrap(),
            run_at(&one, &words, 1e6, 64, TriggerAt::End).unwrap(),
        );
        assert!((e - s) as i64 >= 90 && (e - s) as i64 <= 110, "{s} {e}");
        // The look-back needed to keep the start in the window grows with the sequence.
        let long = ProtocolTrigger::Uart(UartConfig::default(), UartMatch::Bytes(vec![0x55; 8]));
        let (mut a, mut b) = (
            long.build(TriggerAt::End).unwrap(),
            long.build(TriggerAt::Start).unwrap(),
        );
        a.reset(1e6);
        b.reset(1e6);
        assert!(b.lookback() >= a.lookback() && b.lookback() > 0);
    }
}
