//! Simulated analyzer: deterministic patterns as a pure function of the sample index, so tests and
//! decoders know exactly what to expect. Optionally paced in real time.

use crate::*;
use bf_core::{pack, pack_alignment};
use std::time::{Duration, Instant};

/// Message sent by the UART generator on CH0 (115200 baud, 8N1).
pub const UART_MESSAGE: &[u8] = b"blinkfeuer sim 0123456789\r\n";
pub const UART_BAUD: f64 = 115_200.0;
/// UART slots (bit times) per character and idle slots between messages.
const UART_SLOTS_PER_CHAR: u64 = 12; // start + 8 data + stop + 2 idle
const UART_GAP_SLOTS: u64 = 200;

pub const SPI_PERIOD_NS: u64 = 50_000;
pub const I2C_PERIOD_NS: u64 = 500_000;
pub const I2C_BYTES: [u8; 3] = [0xA0, 0x3C, 0x5A];
const I2C_SLOT_NS: u64 = 2_500; // quarter of a 100 kHz bit
pub const PWM_PERIOD_NS: u64 = 100_000;

/// Channel assignment of the generator pattern.
pub mod ch {
    pub const UART: u8 = 0;
    pub const SPI_CS: u8 = 1;
    pub const SPI_SCK: u8 = 2;
    pub const SPI_MOSI: u8 = 3;
    pub const I2C_SCL: u8 = 4;
    pub const I2C_SDA: u8 = 5;
    pub const PWM: u8 = 6;
    pub const CLOCK_1MHZ: u8 = 7;
    /// CH8..CH15: Gray counter (8 bit), +1 every microsecond.
    pub const GRAY_LSB: u8 = 8;
}

/// Pure generator state (the I2C slot table).
#[derive(Clone)]
pub struct Generator {
    i2c: Vec<(bool, bool)>,
}

impl Default for Generator {
    fn default() -> Self {
        Self::new()
    }
}

impl Generator {
    pub fn new() -> Self {
        let mut t: Vec<(bool, bool)> = Vec::new();
        let mut push = |scl, sda, n| t.extend(std::iter::repeat_n((scl, sda), n));
        push(true, true, 4); // idle
        push(true, false, 2); // START: SDA falls while SCL is high
        for &byte in &I2C_BYTES {
            for k in (0..8).rev() {
                let d = (byte >> k) & 1 != 0;
                push(false, d, 1);
                push(true, d, 2);
                push(false, d, 1);
            }
            // ACK: the slave pulls SDA low.
            push(false, false, 1);
            push(true, false, 2);
            push(false, false, 1);
        }
        push(false, false, 1); // STOP: SDA rises while SCL is high
        push(true, false, 1);
        push(true, true, 2);
        let slots = (I2C_PERIOD_NS / I2C_SLOT_NS) as usize;
        assert!(t.len() <= slots);
        t.resize(slots, (true, true));
        Self { i2c: t }
    }

    /// State word of the generator at time `t_ns`.
    pub fn at_ns(&self, t: u64) -> u32 {
        let mut w = 0u32;
        let mut set = |c: u8, v: bool| w |= (v as u32) << c;

        // UART.
        let slot = (t as f64 * UART_BAUD / 1e9) as u64;
        let period = UART_MESSAGE.len() as u64 * UART_SLOTS_PER_CHAR + UART_GAP_SLOTS;
        let p = slot % period;
        let uart = if p >= UART_MESSAGE.len() as u64 * UART_SLOTS_PER_CHAR {
            true
        } else {
            let (c, s) = (
                UART_MESSAGE[(p / UART_SLOTS_PER_CHAR) as usize],
                p % UART_SLOTS_PER_CHAR,
            );
            match s {
                0 => false,                       // start bit
                1..=8 => (c >> (s - 1)) & 1 != 0, // data, LSB first
                _ => true,                        // stop and idle
            }
        };
        set(ch::UART, uart);

        // SPI mode 0, 1 MHz, two bytes per transaction, CS# low during the transfer.
        let (txn, local) = (t / SPI_PERIOD_NS, t % SPI_PERIOD_NS);
        let (cs_start, cs_len) = (5_000u64, 16_000u64);
        let active = local >= cs_start && local < cs_start + cs_len;
        set(ch::SPI_CS, !active);
        if active {
            let rel = local - cs_start;
            let (bit, frac) = (rel / 1000, rel % 1000);
            let byte = if bit < 8 { 0xA5u8 } else { txn as u8 };
            set(ch::SPI_MOSI, (byte >> (7 - bit % 8)) & 1 != 0);
            set(ch::SPI_SCK, frac >= 500);
        }

        // I2C.
        let (scl, sda) = self.i2c[((t % I2C_PERIOD_NS) / I2C_SLOT_NS) as usize];
        set(ch::I2C_SCL, scl);
        set(ch::I2C_SDA, sda);

        // PWM with a duty cycle sweeping 10..90 %.
        let duty = 10 + 80 * ((t / PWM_PERIOD_NS) % 100) / 100;
        set(ch::PWM, (t % PWM_PERIOD_NS) < duty * (PWM_PERIOD_NS / 100));

        // 1 MHz clock and Gray counter.
        set(ch::CLOCK_1MHZ, (t / 500) % 2 == 1);
        let n = t / 1000;
        let gray = (n ^ (n >> 1)) as u32 & 0xff;
        w | gray << ch::GRAY_LSB
    }
}

/// Time of sample `index` in nanoseconds.
pub fn time_ns(index: u64, rate: f64) -> u64 {
    (index as f64 * 1e9 / rate) as u64
}

/// State word of sample `index` for `pattern` (masked to the channel count by the caller).
pub fn pattern_sample(g: &Generator, pattern: Pattern, index: u64, rate: f64) -> u32 {
    match pattern {
        Pattern::Inputs | Pattern::Emulation => 0,
        Pattern::Counter => index as u32 & 0xffff,
        Pattern::Generator => g.at_ns(time_ns(index, rate)),
    }
}

pub struct SimDevice {
    info: DeviceInfo,
    caps: Capabilities,
    realtime: bool,
}

impl SimDevice {
    /// `realtime`: pace the stream at the configured samplerate (as far as the CPU allows).
    pub fn new(realtime: bool) -> Self {
        let rates: Vec<f64> = [5.0, 10.0, 20.0, 25.0, 40.0, 50.0, 80.0, 100.0, 160.0, 200.0]
            .iter()
            .map(|m| m * 1e6)
            .collect();
        Self {
            info: DeviceInfo {
                id: "sim:0".into(),
                name: "Simulated SLogic16U3".into(),
                backend: "sim",
            },
            caps: Capabilities {
                modes: vec![
                    ChannelMode {
                        channels: 4,
                        max_rate: 800e6,
                    },
                    ChannelMode {
                        channels: 8,
                        max_rate: 400e6,
                    },
                    ChannelMode {
                        channels: 16,
                        max_rate: 200e6,
                    },
                ],
                rates,
                patterns: vec![Pattern::Inputs, Pattern::Counter, Pattern::Generator],
                threshold: Some((0.0, 6.0, 0.1)),
            },
            realtime,
        }
    }
}

impl Device for SimDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn capabilities(&self) -> &Capabilities {
        &self.caps
    }

    fn start(&mut self, cfg: &CaptureConfig) -> Result<Box<dyn Stream>, DeviceError> {
        self.caps.validate(cfg)?;
        let align = pack_alignment(cfg.channels) as u64;
        let limit = cfg.limit_samples.map(|l| l.div_ceil(align) * align);
        Ok(Box::new(SimStream {
            cfg: cfg.clone(),
            gen: Generator::new(),
            produced: 0,
            limit,
            started: Instant::now(),
            realtime: self.realtime,
            bytes: 0,
            stopped: false,
        }))
    }
}

struct SimStream {
    cfg: CaptureConfig,
    gen: Generator,
    produced: u64,
    limit: Option<u64>,
    started: Instant,
    realtime: bool,
    bytes: u64,
    stopped: bool,
}

const BUFFER_SAMPLES: u64 = 1 << 16;

impl Stream for SimStream {
    fn read(&mut self, timeout: Duration) -> Result<ReadResult, DeviceError> {
        if self.stopped || self.limit == Some(self.produced) {
            return Ok(ReadResult::Ended);
        }
        let align = pack_alignment(self.cfg.channels) as u64;
        let deadline = Instant::now() + timeout;
        let n = loop {
            let mut avail = BUFFER_SAMPLES;
            if self.realtime {
                let due = (self.started.elapsed().as_secs_f64() * self.cfg.sample_rate) as u64;
                avail = avail.min(due.saturating_sub(self.produced));
            }
            if let Some(l) = self.limit {
                avail = avail.min(l - self.produced);
            }
            // In real time wait for ~1 ms worth of samples to keep the buffer rate sane.
            let min = if self.realtime {
                ((self.cfg.sample_rate / 1000.0) as u64).min(BUFFER_SAMPLES)
            } else {
                1
            };
            if avail >= min.max(align) || (avail > 0 && Some(self.produced + avail) == self.limit) {
                break avail / align * align;
            }
            if Instant::now() >= deadline {
                return Ok(ReadResult::Timeout);
            }
            std::thread::sleep(Duration::from_micros(300));
        };
        let mask = self.cfg.channels.mask();
        let samples: Vec<u32> = (self.produced..self.produced + n)
            .map(|i| pattern_sample(&self.gen, self.cfg.pattern, i, self.cfg.sample_rate) & mask)
            .collect();
        let mut bytes = Vec::with_capacity(self.cfg.channels.bytes_for(n) as usize);
        pack(self.cfg.channels, &samples, &mut bytes);
        self.produced += n;
        self.bytes += bytes.len() as u64;
        Ok(ReadResult::Data(bytes))
    }

    fn stop(&mut self) -> Result<StreamReport, DeviceError> {
        self.stopped = true;
        Ok(StreamReport {
            bytes: self.bytes,
            overflow: false,
            note: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bf_core::Unpacker;

    fn collect(cfg: &CaptureConfig) -> Vec<u32> {
        let mut dev = SimDevice::new(false);
        let mut st = dev.start(cfg).unwrap();
        let mut un = Unpacker::new(cfg.channels);
        let mut out = Vec::new();
        loop {
            match st.read(Duration::from_millis(100)).unwrap() {
                ReadResult::Data(b) => {
                    un.unpack(&b, &mut out);
                }
                ReadResult::Timeout => panic!("timeout"),
                ReadResult::Ended => break,
            }
        }
        out
    }

    fn cfg(channels: u8, rate: f64, n: u64, pattern: Pattern) -> CaptureConfig {
        CaptureConfig {
            channels: ChannelCount::new(channels).unwrap(),
            sample_rate: rate,
            limit_samples: Some(n),
            pattern,
            threshold_v: 1.7,
        }
    }

    #[test]
    fn counter_pattern_counts() {
        let v = collect(&cfg(16, 100e6, 200_000, Pattern::Counter));
        assert_eq!(v.len(), 200_000);
        assert!(v.iter().enumerate().all(|(i, &s)| s == i as u32 & 0xffff));
        let v = collect(&cfg(8, 100e6, 1000, Pattern::Counter));
        assert!(v.iter().enumerate().all(|(i, &s)| s == i as u32 & 0xff));
    }

    #[test]
    fn limit_is_exact_for_aligned_modes() {
        for ch in [4u8, 8, 16] {
            assert_eq!(collect(&cfg(ch, 10e6, 1000, Pattern::Counter)).len(), 1000);
        }
    }

    #[test]
    fn generator_uart_start_bit_and_idle_levels() {
        let g = Generator::new();
        // Before the first start bit the line idles... the message starts at t=0 with a start bit.
        assert_eq!(g.at_ns(0) >> ch::UART & 1, 0);
        // Data bits of the first character, least significant first, in slots 1 to 8.
        let slot = |s: u64| (s as f64 * 1e9 / UART_BAUD) as u64 + 100;
        for k in 0..8u64 {
            assert_eq!(
                (g.at_ns(slot(1 + k)) >> ch::UART & 1),
                u32::from((UART_MESSAGE[0] >> k) & 1),
                "data bit {k}"
            );
        }
        assert_eq!(g.at_ns(slot(10)) >> ch::UART & 1, 1); // stop bit
    }

    #[test]
    fn generator_spi_chip_select_and_bits() {
        let g = Generator::new();
        assert_eq!(g.at_ns(1_000) >> ch::SPI_CS & 1, 1); // idle
        assert_eq!(g.at_ns(6_000) >> ch::SPI_CS & 1, 0); // active
                                                         // First bit of 0xA5 is 1, second 0.
        assert_eq!(g.at_ns(5_100) >> ch::SPI_MOSI & 1, 1);
        assert_eq!(g.at_ns(6_100) >> ch::SPI_MOSI & 1, 0);
        // SCK low in the first half of a bit, high in the second.
        assert_eq!(g.at_ns(5_100) >> ch::SPI_SCK & 1, 0);
        assert_eq!(g.at_ns(5_600) >> ch::SPI_SCK & 1, 1);
    }

    #[test]
    fn generator_i2c_has_start_and_stop() {
        let g = Generator::new();
        let (mut starts, mut stops) = (0, 0);
        let mut prev = g.at_ns(0);
        for t in (0..I2C_PERIOD_NS).step_by(100) {
            let w = g.at_ns(t);
            let (scl, sda, psda) = (
                w >> ch::I2C_SCL & 1,
                w >> ch::I2C_SDA & 1,
                prev >> ch::I2C_SDA & 1,
            );
            if scl == 1 && psda == 1 && sda == 0 {
                starts += 1;
            }
            if scl == 1 && psda == 0 && sda == 1 {
                stops += 1;
            }
            prev = w;
        }
        assert_eq!((starts, stops), (1, 1));
    }

    #[test]
    fn rejects_rates_above_the_channel_mode_limit() {
        let mut dev = SimDevice::new(false);
        assert!(dev.start(&cfg(16, 400e6, 100, Pattern::Counter)).is_err());
        assert!(dev.start(&cfg(4, 800e6, 100, Pattern::Counter)).is_ok());
    }

    #[test]
    fn realtime_delivers_data_above_the_buffer_size_rate() {
        // 100 MHz > BUFFER_SAMPLES per millisecond: must not wait for a buffer that can never fill.
        let mut dev = SimDevice::new(true);
        let mut st = dev
            .start(&cfg(16, 100e6, 1_000_000, Pattern::Counter))
            .unwrap();
        let t0 = Instant::now();
        let mut got = 0usize;
        while got < 100_000 && t0.elapsed() < Duration::from_secs(2) {
            if let ReadResult::Data(b) = st.read(Duration::from_millis(50)).unwrap() {
                got += b.len() / 2;
            }
        }
        assert!(got >= 100_000, "only {got} samples in 2 s");
    }
}
