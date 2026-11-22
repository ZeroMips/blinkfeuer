//! I2C decoder: START, address, data bytes with acknowledge, STOP.

use crate::{bit, flag, Annotation, Decoder, Kind};

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct I2cConfig {
    pub scl: u8,
    pub sda: u8,
}

impl Default for I2cConfig {
    fn default() -> Self {
        Self { scl: 0, sda: 1 }
    }
}

struct Pending {
    start: u64,
    value: u8,
    nak: bool,
    address: bool,
}

pub struct I2c {
    cfg: I2cConfig,
    primed: bool,
    scl: bool,
    sda: bool,
    in_txn: bool,
    /// Bits of the current byte received so far (8 means the next SCL rise is the acknowledge).
    bit_n: u8,
    byte: u8,
    first_byte: bool,
    byte_start: u64,
    pending: Option<Pending>,
}

impl I2c {
    pub fn new(cfg: I2cConfig) -> Self {
        Self { cfg, primed: false, scl: true, sda: true, in_txn: false, bit_n: 0, byte: 0, first_byte: true, byte_start: 0, pending: None }
    }

    pub fn config(&self) -> &I2cConfig {
        &self.cfg
    }

    /// A finished byte is reported when the SCL falls after its acknowledge bit.
    fn flush(&mut self, end: u64, out: &mut Vec<Annotation>) {
        if let Some(p) = self.pending.take() {
            let nak = if p.nak { flag::NAK } else { 0 };
            out.push(if p.address {
                Annotation { start: p.start, end, value: (p.value >> 1) as u32, lane: 0, kind: Kind::Address, bits: 7, flags: nak | if p.value & 1 != 0 { flag::READ } else { 0 } }
            } else {
                Annotation { start: p.start, end, value: p.value as u32, lane: 0, kind: Kind::Data, bits: 8, flags: nak }
            });
        }
    }
}

impl Decoder for I2c {
    fn name(&self) -> &'static str {
        "I2C"
    }

    fn lanes(&self) -> &'static [&'static str] {
        &["I2C"]
    }

    fn mask(&self) -> u32 {
        (1 << self.cfg.scl) | (1 << self.cfg.sda)
    }

    fn reset(&mut self, _rate: f64) {
        self.primed = false;
        self.in_txn = false;
        self.bit_n = 0;
        self.pending = None;
    }

    fn change(&mut self, t: u64, state: u32, out: &mut Vec<Annotation>) {
        let (scl, sda) = (bit(state, self.cfg.scl), bit(state, self.cfg.sda));
        if !self.primed {
            self.primed = true;
            (self.scl, self.sda) = (scl, sda);
            return;
        }
        if scl && self.scl {
            // SCL steady high: a change of SDA is a START or STOP.
            if sda != self.sda {
                self.flush(t, out);
                if !sda {
                    out.push(Annotation {
                        start: t,
                        end: t + 1,
                        value: 0,
                        lane: 0,
                        kind: Kind::Start,
                        bits: 0,
                        flags: if self.in_txn { flag::REPEATED } else { 0 },
                    });
                    self.in_txn = true;
                    self.bit_n = 0;
                    self.byte = 0;
                    self.first_byte = true;
                    self.byte_start = t + 1;
                } else if self.in_txn {
                    out.push(Annotation { start: t, end: t + 1, value: 0, lane: 0, kind: Kind::Stop, bits: 0, flags: 0 });
                    self.in_txn = false;
                }
            }
        } else if scl && !self.scl {
            // SCL rises: SDA is valid.
            if self.in_txn {
                if self.bit_n < 8 {
                    self.byte = (self.byte << 1) | sda as u8;
                    self.bit_n += 1;
                } else {
                    self.pending = Some(Pending { start: self.byte_start, value: self.byte, nak: sda, address: self.first_byte });
                    self.first_byte = false;
                    self.bit_n = 0;
                    self.byte = 0;
                }
            }
        } else if !scl && self.scl {
            // SCL falls: the byte before it is complete; the next one begins.
            self.flush(t, out);
            if self.bit_n == 0 {
                self.byte_start = t;
            }
        }
        (self.scl, self.sda) = (scl, sda);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::decode_all;
    use crate::synth::Wave;
    use bf_core::{CaptureStore, ChannelCount, StoreConfig};

    const RATE: f64 = 10e6;
    const Q: u64 = 25; // quarter of a 100 kHz bit

    fn decode(w: &Wave) -> Vec<Annotation> {
        let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(8).unwrap(), RATE));
        s.push_samples(&w.samples);
        decode_all(Box::new(I2c::new(I2cConfig::default())), &s)
    }

    #[test]
    fn write_transaction_with_two_data_bytes() {
        let mut w = Wave::new(RATE, 6_000, 0b11);
        let mut t = w.i2c_start(0, 1, 100, Q, false);
        t = w.i2c_byte(0, 1, t, Q, 0xA0, false);
        t = w.i2c_byte(0, 1, t, Q, 0x3C, false);
        t = w.i2c_byte(0, 1, t, Q, 0x5A, false);
        w.i2c_stop(0, 1, t, Q);
        let a = decode(&w);
        let kinds: Vec<_> = a.iter().map(|a| (a.kind, a.value, a.flags)).collect();
        assert_eq!(
            kinds,
            vec![
                (Kind::Start, 0, 0),
                (Kind::Address, 0x50, 0),
                (Kind::Data, 0x3C, 0),
                (Kind::Data, 0x5A, 0),
                (Kind::Stop, 0, 0),
            ]
        );
        // Sorted and not overlapping.
        assert!(a.windows(2).all(|p| p[0].end <= p[1].start), "{a:?}");
        assert_eq!(a[0].start, 100);
        assert_eq!(a[1].label(crate::Format::Hex), "50 W");
    }

    #[test]
    fn read_with_nak_and_repeated_start() {
        let mut w = Wave::new(RATE, 12_000, 0b11);
        let mut t = w.i2c_start(0, 1, 100, Q, false);
        t = w.i2c_byte(0, 1, t, Q, 0x90, false); // address 0x48 write
        t = w.i2c_byte(0, 1, t, Q, 0x01, false);
        t = w.i2c_start(0, 1, t, Q, true);
        t = w.i2c_byte(0, 1, t, Q, 0x91, false); // address 0x48 read
        t = w.i2c_byte(0, 1, t, Q, 0xBE, false);
        t = w.i2c_byte(0, 1, t, Q, 0xEF, true); // master NAKs the last byte
        w.i2c_stop(0, 1, t, Q);
        let a = decode(&w);
        let seq: Vec<_> = a.iter().map(|a| (a.kind, a.value, a.flags)).collect();
        assert_eq!(
            seq,
            vec![
                (Kind::Start, 0, 0),
                (Kind::Address, 0x48, 0),
                (Kind::Data, 0x01, 0),
                (Kind::Start, 0, flag::REPEATED),
                (Kind::Address, 0x48, flag::READ),
                (Kind::Data, 0xBE, 0),
                (Kind::Data, 0xEF, flag::NAK),
                (Kind::Stop, 0, 0),
            ]
        );
        assert_eq!(a[4].label(crate::Format::Hex), "48 R");
        assert_eq!(a[6].label(crate::Format::Hex), "EF NAK");
    }

    #[test]
    fn address_nak_is_reported() {
        let mut w = Wave::new(RATE, 4_000, 0b11);
        let mut t = w.i2c_start(0, 1, 100, Q, false);
        t = w.i2c_byte(0, 1, t, Q, 0xA0, true); // nobody answers
        w.i2c_stop(0, 1, t, Q);
        let a = decode(&w);
        assert_eq!(a[1].kind, Kind::Address);
        assert_ne!(a[1].flags & flag::NAK, 0);
    }

    #[test]
    fn idle_bus_and_lone_clock_produce_nothing() {
        let w = Wave::new(RATE, 1_000, 0b11);
        assert!(decode(&w).is_empty());
        // SCL toggling without a START is not a transaction.
        let mut w = Wave::new(RATE, 1_000, 0b11);
        for k in 0..10 {
            w.set(0, 100 + k * 40, 120 + k * 40, false);
        }
        assert!(decode(&w).is_empty());
    }

    #[test]
    fn different_pins_work() {
        let mut w = Wave::new(RATE, 4_000, 0b1100_0000);
        let mut t = w.i2c_start(6, 7, 100, Q, false);
        t = w.i2c_byte(6, 7, t, Q, 0x42, false);
        w.i2c_stop(6, 7, t, Q);
        let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(8).unwrap(), RATE));
        s.push_samples(&w.samples);
        let a = decode_all(Box::new(I2c::new(I2cConfig { scl: 6, sda: 7 })), &s);
        assert_eq!(a.iter().filter(|a| a.kind == Kind::Address).count(), 1);
        assert_eq!(a[1].value, 0x21);
    }
}
