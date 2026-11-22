//! A protocol trigger in a running capture: the capture is cut around the end of the first item that
//! matches, found independently by decoding the same simulated stream completely.

use bf_core::{CaptureStore, ChannelCount, StoreConfig};
use bf_decode::engine::decode_all;
use bf_decode::i2c::{I2c, I2cConfig};
use bf_decode::spi::{Spi, SpiConfig};
use bf_decode::trigger::{I2cMatch, ProtocolTrigger, SpiLane, SpiMatch, TriggerAt, UartMatch};
use bf_decode::uart::{Uart, UartConfig};
use bf_decode::{flag, Kind};
use bf_device::session::{Session, TriggerKind, TriggerPlan};
use bf_device::sim::{ch, time_ns, Generator, SimDevice, UART_MESSAGE};
use bf_device::{CaptureConfig, Pattern};

const RATE: f64 = 20e6;

fn reference(ms: u64) -> (Vec<u32>, CaptureStore) {
    let g = Generator::new();
    let n = (ms as f64 * 1e-3 * RATE) as u64;
    let words: Vec<u32> = (0..n).map(|i| g.at_ns(time_ns(i, RATE))).collect();
    let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(16).unwrap(), RATE));
    s.push_samples(&words);
    (words, s)
}

/// Run a triggered capture of the simulator's generator pattern; returns its store.
fn capture(trigger: ProtocolTrigger, at: TriggerAt, pre: u64, post: u64) -> bf_core::SharedStore {
    let channels = ChannelCount::new(16).unwrap();
    let store = CaptureStore::shared(StoreConfig::new(channels, RATE));
    let cfg = CaptureConfig { channels, sample_rate: RATE, limit_samples: None, pattern: Pattern::Generator, threshold_v: 1.7 };
    let plan = TriggerPlan { kind: TriggerKind::Event(trigger.build(at).unwrap()), pre, post: Some(post) };
    let mut dev = SimDevice::new(false);
    let res = Session::start_triggered(&mut dev, &cfg, store.clone(), Some(plan)).unwrap().join();
    assert!(res.error.is_none(), "{:?}", res.error);
    store
}

fn check_window(store: &bf_core::SharedStore, words: &[u32], expect: u64, pre: u64, post: u64) {
    let s = store.read();
    assert_eq!(s.trigger(), Some(expect));
    assert_eq!(s.range(), (expect - pre)..(expect + post));
    // The data in the window is the generator's, sample for sample.
    for p in (expect - pre)..(expect + post) {
        if let Some(w) = words.get(p as usize) {
            assert_eq!(s.state_at(p), Some(*w), "sample {p}");
        }
    }
}

#[test]
fn uart_text_triggers_at_the_end_of_its_last_character() {
    let (words, full) = reference(40);
    let cfg = UartConfig { rx: ch::UART, ..Default::default() };
    let chars: Vec<_> = decode_all(Box::new(Uart::new(cfg.clone())), &full).into_iter().filter(|a| a.lane == 0).collect();
    // "sim 0" (the text after "blinkfeuer ") in the second message of the capture: with 100 000
    // samples of pre-trigger the first message cannot be used.
    let needle = b"sim 0";
    let (pre, post) = (100_000u64, 20_000u64);
    let expect = chars
        .windows(needle.len())
        .filter(|w| w.iter().map(|a| a.value as u8).eq(needle.iter().copied()))
        .map(|w| w[needle.len() - 1].end)
        .find(|end| *end >= pre)
        .expect("in the reference decode");
    assert!(UART_MESSAGE.windows(needle.len()).any(|w| w == needle));
    let store = capture(ProtocolTrigger::Uart(cfg, UartMatch::Bytes(needle.to_vec())), TriggerAt::End, pre, post);
    check_window(&store, &words, expect, pre, post);
}

#[test]
fn spi_word_triggers_at_the_end_of_the_word() {
    let (words, full) = reference(10);
    let cfg = SpiConfig { cs: Some(ch::SPI_CS), sck: ch::SPI_SCK, mosi: Some(ch::SPI_MOSI), ..Default::default() };
    let all = decode_all(Box::new(Spi::new(cfg.clone())), &full);
    let a5 = all.iter().filter(|a| a.kind == Kind::Data && a.lane == 0 && a.value == 0xA5).collect::<Vec<_>>();
    let (pre, post) = (2_000u64, 3_000u64);
    let expect = a5.iter().map(|a| a.end).find(|e| *e >= pre).expect("A5 in the reference decode");
    let store = capture(ProtocolTrigger::Spi(cfg, SpiMatch::Word { lane: SpiLane::Mosi, value: 0xA5, care: 0xFF }), TriggerAt::End, pre, post);
    check_window(&store, &words, expect, pre, post);
}

#[test]
fn i2c_address_and_direction_trigger_at_the_address_byte() {
    let (words, full) = reference(10);
    let cfg = I2cConfig { scl: ch::I2C_SCL, sda: ch::I2C_SDA };
    let all = decode_all(Box::new(I2c::new(cfg.clone())), &full);
    // The generator writes 0xA0 0x3C 0x5A: address 0x50, write.
    let addr = all.iter().find(|a| a.kind == Kind::Address).expect("address");
    assert_eq!((addr.value, addr.flags & flag::READ), (0x50, 0));
    let (pre, post) = (500u64, 4_000u64);
    let expect = all.iter().filter(|a| a.kind == Kind::Address && a.value == 0x50 && a.end >= pre).map(|a| a.end).next().unwrap();
    let store = capture(ProtocolTrigger::I2c(cfg.clone(), I2cMatch::Address { addr: 0x50, read: Some(false) }), TriggerAt::End, pre, post);
    check_window(&store, &words, expect, pre, post);
    // The same address as a read never happens in the generator's traffic: the capture would wait
    // forever, so only check that the trigger refuses nothing and builds.
    assert!(ProtocolTrigger::I2c(cfg, I2cMatch::Address { addr: 0x50, read: Some(true) }).build(TriggerAt::End).is_ok());
}

#[test]
fn the_start_of_the_match_can_be_the_trigger_instead_of_its_end() {
    let (words, full) = reference(40);
    let (pre, post) = (100_000u64, 20_000u64);

    // UART text: the start of the first character of "sim 0".
    let cfg = UartConfig { rx: ch::UART, ..Default::default() };
    let chars: Vec<_> = decode_all(Box::new(Uart::new(cfg.clone())), &full).into_iter().filter(|a| a.lane == 0).collect();
    let needle = b"sim 0";
    let windows: Vec<_> = chars.windows(needle.len()).filter(|w| w.iter().map(|a| a.value as u8).eq(needle.iter().copied())).collect();
    let expect = windows.iter().map(|w| w[0].start).find(|s| *s >= pre).expect("in the reference decode");
    let store = capture(ProtocolTrigger::Uart(cfg, UartMatch::Bytes(needle.to_vec())), TriggerAt::Start, pre, post);
    check_window(&store, &words, expect, pre, post);

    // SPI: the start of the word.
    let spi = SpiConfig { cs: Some(ch::SPI_CS), sck: ch::SPI_SCK, mosi: Some(ch::SPI_MOSI), ..Default::default() };
    let all = decode_all(Box::new(Spi::new(spi.clone())), &full);
    let expect = all.iter().filter(|a| a.kind == Kind::Data && a.lane == 0 && a.value == 0xA5).map(|a| a.start).find(|s| *s >= 2_000).unwrap();
    let store = capture(ProtocolTrigger::Spi(spi, SpiMatch::Word { lane: SpiLane::Mosi, value: 0xA5, care: 0xFF }), TriggerAt::Start, 2_000, 3_000);
    check_window(&store, &words, expect, 2_000, 3_000);

    // I2C: the start of the address byte, a few hundred samples before its end.
    let i2c = I2cConfig { scl: ch::I2C_SCL, sda: ch::I2C_SDA };
    let all = decode_all(Box::new(I2c::new(i2c.clone())), &full);
    let addr = all.iter().find(|a| a.kind == Kind::Address && a.value == 0x50 && a.start >= 500).unwrap();
    let store = capture(ProtocolTrigger::I2c(i2c, I2cMatch::Address { addr: 0x50, read: None }), TriggerAt::Start, 500, 4_000);
    check_window(&store, &words, addr.start, 500, 4_000);
    assert!(addr.end > addr.start + 100, "an address byte lasts many samples, so start and end differ");
}
