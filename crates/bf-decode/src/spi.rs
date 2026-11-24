//! SPI decoder: words on MOSI and/or MISO, framed by chip select when it is wired.

use crate::{bit, flag, Annotation, Decoder, Kind};

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SpiConfig {
    pub sck: u8,
    pub cs: Option<u8>,
    pub mosi: Option<u8>,
    pub miso: Option<u8>,
    /// Clock idles high.
    pub cpol: bool,
    /// Data is sampled on the second (trailing) clock edge.
    pub cpha: bool,
    pub msb_first: bool,
    /// Bits per word, 2 to 32.
    pub bits: u8,
    pub cs_active_low: bool,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            sck: 1,
            cs: Some(0),
            mosi: Some(2),
            miso: None,
            cpol: false,
            cpha: false,
            msb_first: true,
            bits: 8,
            cs_active_low: true,
        }
    }
}

pub struct Spi {
    cfg: SpiConfig,
    primed: bool,
    prev: u32,
    in_txn: bool,
    txn_start: u64,
    words: u32,
    /// Bits collected in the current word.
    n: u8,
    mosi: u32,
    miso: u32,
    first_t: u64,
    last_t: u64,
}

impl Spi {
    pub fn new(cfg: SpiConfig) -> Self {
        Self {
            cfg,
            primed: false,
            prev: 0,
            in_txn: false,
            txn_start: 0,
            words: 0,
            n: 0,
            mosi: 0,
            miso: 0,
            first_t: 0,
            last_t: 0,
        }
    }

    pub fn config(&self) -> &SpiConfig {
        &self.cfg
    }

    fn cs_active(&self, state: u32) -> bool {
        self.cfg
            .cs
            .is_none_or(|c| bit(state, c) != self.cfg.cs_active_low)
    }

    fn emit_word(&mut self, flags: u8, out: &mut Vec<Annotation>) {
        let bits = self.n;
        // Words are drawn from half a clock before the first sampling edge to half a clock after the
        // last, so that consecutive words touch.
        let period = if bits > 1 {
            (self.last_t - self.first_t) / (bits as u64 - 1)
        } else {
            2
        };
        let (start, end) = (
            self.first_t.saturating_sub(period / 2).max(self.txn_start),
            self.last_t + period / 2 + 1,
        );
        let mut emit = |lane: u8, value: u32| {
            out.push(Annotation {
                start,
                end,
                value,
                lane,
                kind: Kind::Data,
                bits,
                flags,
            });
        };
        if self.cfg.mosi.is_some() {
            emit(0, self.mosi);
        }
        if self.cfg.miso.is_some() {
            emit(1, self.miso);
        }
        self.n = 0;
        self.mosi = 0;
        self.miso = 0;
        self.words += 1;
    }

    fn end_txn(&mut self, t: u64, out: &mut Vec<Annotation>) {
        if self.n > 0 {
            self.emit_word(flag::INCOMPLETE, out);
        }
        if self.cfg.cs.is_some() && self.words > 0 {
            out.push(Annotation {
                start: self.txn_start,
                end: t.max(self.txn_start + 1),
                value: self.words,
                lane: 2,
                kind: Kind::Frame,
                bits: 0,
                flags: 0,
            });
        }
        self.in_txn = false;
    }

    fn begin_txn(&mut self, t: u64) {
        self.in_txn = true;
        self.txn_start = t;
        self.words = 0;
        self.n = 0;
        self.mosi = 0;
        self.miso = 0;
    }
}

impl Decoder for Spi {
    fn name(&self) -> &'static str {
        "SPI"
    }

    fn lanes(&self) -> &'static [&'static str] {
        &["MOSI", "MISO", "CS"]
    }

    fn lane_used(&self, lane: usize) -> bool {
        match lane {
            0 => self.cfg.mosi.is_some(),
            1 => self.cfg.miso.is_some(),
            _ => self.cfg.cs.is_some(),
        }
    }

    fn mask(&self) -> u32 {
        let mut m = 1 << self.cfg.sck;
        for c in [self.cfg.cs, self.cfg.mosi, self.cfg.miso]
            .into_iter()
            .flatten()
        {
            m |= 1 << c;
        }
        // MOSI and MISO are only sampled on clock edges: changes of the data lines alone are not
        // events for the decoder.
        m & !(self.cfg.mosi.map_or(0, |c| 1 << c) | self.cfg.miso.map_or(0, |c| 1 << c))
    }

    fn reset(&mut self, _rate: f64) {
        self.primed = false;
        self.in_txn = false;
        self.n = 0;
        self.words = 0;
    }

    fn change(&mut self, t: u64, state: u32, out: &mut Vec<Annotation>) {
        if !self.primed {
            self.primed = true;
            self.prev = state;
            if self.cs_active(state) {
                self.begin_txn(t);
            }
            return;
        }
        if self.cfg.cs.is_some() {
            let now = self.cs_active(state);
            if now && !self.in_txn {
                self.begin_txn(t);
            } else if !now && self.in_txn {
                self.end_txn(t, out);
            }
        } else {
            self.in_txn = true;
        }
        let (was, now) = (bit(self.prev, self.cfg.sck), bit(state, self.cfg.sck));
        if self.in_txn && was != now && now == (self.cfg.cpol == self.cfg.cpha) {
            // Sampling edge.
            if self.n == 0 {
                self.first_t = t;
            }
            let (mi, mo) = (
                self.cfg.mosi.is_some_and(|c| bit(state, c)),
                self.cfg.miso.is_some_and(|c| bit(state, c)),
            );
            if self.cfg.msb_first {
                self.mosi = (self.mosi << 1) | mi as u32;
                self.miso = (self.miso << 1) | mo as u32;
            } else {
                self.mosi |= (mi as u32) << self.n;
                self.miso |= (mo as u32) << self.n;
            }
            self.n += 1;
            self.last_t = t;
            if self.n == self.cfg.bits {
                self.emit_word(0, out);
            }
        }
        self.prev = state;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::decode_all;
    use crate::synth::Wave;
    use bf_core::{CaptureStore, ChannelCount, StoreConfig};

    const RATE: f64 = 100e6;

    fn run(cfg: &SpiConfig, w: &Wave) -> Vec<Annotation> {
        let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(8).unwrap(), RATE));
        s.push_samples(&w.samples);
        decode_all(Box::new(Spi::new(cfg.clone())), &s)
    }

    fn words(a: &[Annotation], lane: u8) -> Vec<u32> {
        a.iter()
            .filter(|a| a.lane == lane && a.kind == Kind::Data)
            .map(|a| a.value)
            .collect()
    }

    fn init(cfg: &SpiConfig) -> u32 {
        // Clock idles at CPOL, CS idles inactive.
        (cfg.cpol as u32) << cfg.sck | cfg.cs.map_or(0, |c| (cfg.cs_active_low as u32) << c)
    }

    #[test]
    fn all_four_modes_decode_the_same_bytes() {
        for (cpol, cpha) in [(false, false), (false, true), (true, false), (true, true)] {
            let cfg = SpiConfig {
                cpol,
                cpha,
                miso: Some(3),
                ..Default::default()
            };
            let mut w = Wave::new(RATE, 6_000, init(&cfg));
            let mosi = [0xA5, 0x3C, 0xFF, 0x00];
            let miso = [0x12, 0x34, 0x56, 0x78];
            w.spi(&cfg, 200, 20, &mosi, &miso);
            let a = run(&cfg, &w);
            assert_eq!(words(&a, 0), mosi, "mode cpol={cpol} cpha={cpha}");
            assert_eq!(words(&a, 1), miso, "mode cpol={cpol} cpha={cpha}");
            let frames: Vec<_> = a.iter().filter(|a| a.kind == Kind::Frame).collect();
            assert_eq!(frames.len(), 1);
            assert_eq!(frames[0].value, 4);
            assert!(a.iter().all(|a| a.flags == 0));
        }
    }

    #[test]
    fn lsb_first_and_word_sizes() {
        for (bits, msb) in [(8u8, false), (12, true), (16, false), (32, true), (5, true)] {
            let cfg = SpiConfig {
                bits,
                msb_first: msb,
                ..Default::default()
            };
            let mask = if bits == 32 {
                u32::MAX
            } else {
                (1u32 << bits) - 1
            };
            let data: Vec<u32> = [0x1234_5678u32, 0x9ABC_DEF0, 0x0F0F_0F0F]
                .iter()
                .map(|v| v & mask)
                .collect();
            let mut w = Wave::new(RATE, 12_000, init(&cfg));
            w.spi(&cfg, 100, 10, &data, &[]);
            let a = run(&cfg, &w);
            assert_eq!(words(&a, 0), data, "bits={bits} msb={msb}");
            assert!(words(&a, 1).is_empty());
        }
    }

    #[test]
    fn several_transactions_are_separate_frames() {
        let cfg = SpiConfig::default();
        let mut w = Wave::new(RATE, 10_000, init(&cfg));
        let e1 = w.spi(&cfg, 100, 10, &[0x11, 0x22], &[]);
        w.spi(&cfg, e1 + 50, 10, &[0x33], &[]);
        let a = run(&cfg, &w);
        assert_eq!(words(&a, 0), vec![0x11, 0x22, 0x33]);
        let frames: Vec<u32> = a
            .iter()
            .filter(|a| a.kind == Kind::Frame)
            .map(|a| a.value)
            .collect();
        assert_eq!(frames, vec![2, 1]);
    }

    #[test]
    fn without_chip_select_words_follow_the_clock() {
        let cfg = SpiConfig {
            cs: None,
            ..Default::default()
        };
        let mut w = Wave::new(RATE, 6_000, init(&cfg));
        w.spi(&cfg, 100, 10, &[0xDE, 0xAD, 0xBE, 0xEF], &[]);
        let a = run(&cfg, &w);
        assert_eq!(words(&a, 0), vec![0xDE, 0xAD, 0xBE, 0xEF]);
        assert!(a.iter().all(|a| a.kind != Kind::Frame));
    }

    #[test]
    fn a_word_cut_short_by_chip_select_is_marked_incomplete() {
        let cfg = SpiConfig::default();
        let mut w = Wave::new(RATE, 4_000, init(&cfg));
        w.spi(&cfg, 100, 10, &[0xAB, 0xCD], &[]);
        // Release CS in the middle of the second byte.
        let cs_off = 100 + 10 + 2 * 10 * 12;
        w.set(0, cs_off, 4_000, true);
        let a = run(&cfg, &w);
        let data: Vec<_> = a.iter().filter(|a| a.kind == Kind::Data).collect();
        assert_eq!(data.len(), 2);
        assert_eq!(data[0].flags, 0);
        assert_ne!(data[1].flags & flag::INCOMPLETE, 0);
        assert!(data[1].bits < 8);
    }

    #[test]
    fn words_touch_and_never_overlap() {
        let cfg = SpiConfig::default();
        let mut w = Wave::new(RATE, 6_000, init(&cfg));
        w.spi(&cfg, 100, 10, &[1, 2, 3, 4], &[]);
        let a = run(&cfg, &w);
        let d: Vec<_> = a.iter().filter(|a| a.kind == Kind::Data).collect();
        for p in d.windows(2) {
            assert!(
                p[0].end <= p[1].start + 1 && p[0].end + 3 >= p[1].start,
                "{p:?}"
            );
        }
    }
}
