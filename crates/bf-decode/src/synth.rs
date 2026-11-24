//! Synthetic waveforms with exactly known content, used by the decoder tests and benchmarks.

use crate::spi::SpiConfig;
use crate::uart::{Parity, UartConfig};

/// A multi-channel waveform as one state word per sample.
pub struct Wave {
    /// Samples per second.
    pub rate: f64,
    /// State word of each sample.
    pub samples: Vec<u32>,
}

impl Wave {
    /// A waveform of `len` samples that all have the state `init`.
    pub fn new(rate: f64, len: usize, init: u32) -> Self {
        Self {
            rate,
            samples: vec![init; len],
        }
    }

    /// Set channel `ch` to `level` for samples `from..to` (clamped to the waveform).
    pub fn set(&mut self, ch: u8, from: u64, to: u64, level: bool) {
        let n = self.samples.len() as u64;
        for s in &mut self.samples[from.min(n) as usize..to.min(n) as usize] {
            if level {
                *s |= 1 << ch;
            } else {
                *s &= !(1 << ch);
            }
        }
    }

    /// Draw UART frames starting at sample `start`, with two idle bit times between frames. Returns the
    /// sample after the last frame. The idle level is high (low if `cfg.invert`).
    pub fn uart(&mut self, ch: u8, start: u64, cfg: &UartConfig, bytes: &[u8]) -> u64 {
        let t = self.rate / cfg.baud;
        let mut pos = start as f64;
        for &b in bytes {
            let mut slots: Vec<(bool, f64)> = vec![(false, 1.0)];
            let mut ones = 0;
            let n = cfg.data_bits as u32;
            for k in 0..n {
                let shift = if cfg.lsb_first { k } else { n - 1 - k };
                let v = (b as u32 >> shift) & 1 != 0;
                ones += v as u32;
                slots.push((v, 1.0));
            }
            match cfg.parity {
                Parity::None => {}
                Parity::Even => slots.push((ones % 2 == 1, 1.0)),
                Parity::Odd => slots.push((ones % 2 == 0, 1.0)),
                Parity::Mark => slots.push((true, 1.0)),
                Parity::Space => slots.push((false, 1.0)),
            }
            slots.push((true, cfg.stop_bits as f64));
            for (level, len) in slots {
                let (a, z) = (pos.round() as u64, (pos + len * t).round() as u64);
                self.set(ch, a, z, level ^ cfg.invert);
                pos += len * t;
            }
            pos += 2.0 * t;
        }
        pos.round() as u64
    }

    /// One SPI transaction: CS asserted, `words` shifted out on MOSI (and `miso` shifted in, if not
    /// empty), CS released. `half` is half a clock period in samples. Returns the end sample.
    pub fn spi(
        &mut self,
        cfg: &SpiConfig,
        start: u64,
        half: u64,
        mosi: &[u32],
        miso: &[u32],
    ) -> u64 {
        let idle = cfg.cpol;
        let cs_on = !cfg.cs_active_low;
        let words = mosi.len().max(miso.len());
        let bits = cfg.bits as u64;
        let mut t = start;
        if let Some(cs) = cfg.cs {
            self.set(cs, t, t + 2 * half * bits * words as u64 + 8 * half, cs_on);
        }
        t += half;
        for w in 0..words {
            for i in 0..bits {
                let shift = if cfg.msb_first { bits - 1 - i } else { i };
                let p = t + (w as u64 * bits + i) * 2 * half;
                let (d0, d1) = if cfg.cpha {
                    (p + half, p + 3 * half)
                } else {
                    (p, p + 2 * half)
                };
                if let Some(c) = cfg.mosi {
                    let v = mosi.get(w).is_some_and(|m| (m >> shift) & 1 != 0);
                    self.set(c, d0, d1, v);
                }
                if let Some(c) = cfg.miso {
                    let v = miso.get(w).is_some_and(|m| (m >> shift) & 1 != 0);
                    self.set(c, d0, d1, v);
                }
                // Clock: leading edge in the middle of the bit period, trailing edge half a period later.
                let lead = p + half;
                self.set(cfg.sck, lead, lead + half, !idle);
            }
        }
        // Chip select is released well after the last clock edge.
        start + 2 * half * bits * words as u64 + 8 * half
    }

    /// I2C START (a repeated START first brings SDA high while SCL is low).
    pub fn i2c_start(&mut self, scl: u8, sda: u8, t: u64, q: u64, repeated: bool) -> u64 {
        let mut t = t;
        if repeated {
            self.set(scl, t, t + q, false);
            self.set(sda, t, t + q, true);
            self.set(scl, t + q, t + 2 * q, true);
            self.set(sda, t + q, t + 2 * q, true);
            t += 2 * q;
        }
        self.set(scl, t, t + q, true);
        self.set(sda, t, t + q, false);
        t + q
    }

    /// One byte plus acknowledge bit (`nak`: the slave leaves SDA high).
    pub fn i2c_byte(&mut self, scl: u8, sda: u8, t: u64, q: u64, byte: u8, nak: bool) -> u64 {
        let mut t = t;
        for k in (0..9).rev() {
            let d = if k == 0 {
                nak
            } else {
                (byte >> (k - 1)) & 1 != 0
            };
            self.set(scl, t, t + q, false);
            self.set(sda, t, t + 4 * q, d);
            self.set(scl, t + q, t + 3 * q, true);
            self.set(scl, t + 3 * q, t + 4 * q, false);
            t += 4 * q;
        }
        t
    }

    /// I2C STOP at sample `t` (`q`: a quarter of a bit time in samples). Returns the sample after
    /// it.
    pub fn i2c_stop(&mut self, scl: u8, sda: u8, t: u64, q: u64) -> u64 {
        self.set(scl, t, t + q, false);
        self.set(sda, t, t + 2 * q, false);
        self.set(scl, t + q, t + 3 * q, true);
        self.set(sda, t + 2 * q, t + 3 * q, true);
        t + 3 * q
    }
}
