//! The simulator's generator pattern carries a UART message, SPI transactions and an I2C write;
//! the decoders must recover exactly that content.

use bf_core::{CaptureStore, ChannelCount, StoreConfig};
use bf_decode::engine::decode_all;
use bf_decode::i2c::{I2c, I2cConfig};
use bf_decode::spi::{Spi, SpiConfig};
use bf_decode::uart::{Uart, UartConfig};
use bf_decode::{flag, Kind};
use bf_device::sim::{ch, time_ns, Generator, I2C_BYTES, UART_MESSAGE};

const RATE: f64 = 20e6;

fn capture(ms: u64) -> CaptureStore {
    let g = Generator::new();
    let n = (ms as f64 * 1e-3 * RATE) as u64;
    let samples: Vec<u32> = (0..n).map(|i| g.at_ns(time_ns(i, RATE))).collect();
    let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(16).unwrap(), RATE));
    s.push_samples(&samples);
    s
}

#[test]
fn uart_message_is_recovered_repeatedly() {
    let s = capture(20);
    let all = decode_all(
        Box::new(Uart::new(UartConfig {
            rx: ch::UART,
            ..Default::default()
        })),
        &s,
    );
    let a: Vec<_> = all.iter().filter(|a| a.lane == 0).copied().collect();
    let bytes: Vec<u8> = a.iter().map(|a| a.value as u8).collect();
    // 20 ms hold four complete messages (4.44 ms each) and a partial fifth. The capture starts in the
    // middle of the first start bit, so the first message may be misframed (the decoder cannot know it
    // joined a frame in progress); from the first idle gap on every message must be exact.
    let msg = UART_MESSAGE;
    let exact = bytes.windows(msg.len()).filter(|w| *w == msg).count();
    assert!(
        exact >= 3,
        "found {exact} exact messages in {} bytes",
        bytes.len()
    );
    // Exact messages follow each other: the byte after the first exact one starts the next.
    let first = bytes.windows(msg.len()).position(|w| w == msg).unwrap();
    assert!(
        first <= msg.len(),
        "resynchronised late: message found at byte {first}"
    );
    for (i, b) in bytes[first..].iter().enumerate() {
        assert_eq!(*b, msg[i % msg.len()], "byte {i} after resync");
    }
    // The only errors are in the characters before the decoder found its footing.
    assert!(
        a[first..].iter().all(|a| a.flags == 0),
        "no errors expected after resync"
    );
}

#[test]
fn uart_messages_become_telegrams_of_the_exact_text() {
    use bf_decode::{telegram_chars, telegram_text, Format};
    let s = capture(30);
    let all = decode_all(
        Box::new(Uart::new(UartConfig {
            rx: ch::UART,
            ..Default::default()
        })),
        &s,
    );
    let chars: Vec<_> = all.iter().filter(|a| a.lane == 0).copied().collect();
    let texts: Vec<String> = all
        .iter()
        .filter(|a| a.kind == Kind::Telegram)
        .map(|t| telegram_text(telegram_chars(&chars, t), Format::Ascii, 1000))
        .collect();
    let want = String::from_utf8_lossy(UART_MESSAGE)
        .replace('\r', "\\r")
        .replace('\n', "\\n");
    // The first telegram may be damaged (the capture starts inside a frame); the next ones are exact.
    let exact = texts.iter().filter(|t| **t == want).count();
    assert!(exact >= 5, "{exact} exact telegrams in {texts:?}");
    assert!(texts.len() <= exact + 2);
}

#[test]
fn spi_transactions_carry_a5_and_the_transaction_counter() {
    let s = capture(10); // 200 transactions of 50 us
    let cfg = SpiConfig {
        cs: Some(ch::SPI_CS),
        sck: ch::SPI_SCK,
        mosi: Some(ch::SPI_MOSI),
        miso: None,
        ..Default::default()
    };
    let a = decode_all(Box::new(Spi::new(cfg)), &s);
    let words: Vec<u32> = a
        .iter()
        .filter(|a| a.kind == Kind::Data)
        .map(|a| a.value)
        .collect();
    let frames: Vec<u32> = a
        .iter()
        .filter(|a| a.kind == Kind::Frame)
        .map(|a| a.value)
        .collect();
    assert_eq!(frames.len(), 200);
    assert!(frames.iter().all(|&w| w == 2));
    assert_eq!(words.len(), 400);
    for txn in 0..200usize {
        assert_eq!(words[2 * txn], 0xA5, "txn {txn}");
        assert_eq!(words[2 * txn + 1], txn as u32 & 0xff, "txn {txn}");
    }
    assert!(a.iter().all(|a| a.flags == 0));
}

#[test]
fn i2c_write_of_a0_3c_5a_with_acks() {
    let s = capture(5); // ten 500 us periods
    let a = decode_all(
        Box::new(I2c::new(I2cConfig {
            scl: ch::I2C_SCL,
            sda: ch::I2C_SDA,
        })),
        &s,
    );
    // Per period: S, address 0x50 W, 0x3C, 0x5A, P.
    assert_eq!(a.len(), 10 * 5);
    for period in a.chunks(5) {
        assert_eq!(period[0].kind, Kind::Start);
        assert_eq!(period[1].kind, Kind::Address);
        assert_eq!(period[1].value, (I2C_BYTES[0] >> 1) as u32);
        assert_eq!(period[1].flags & flag::READ, 0);
        assert_eq!(period[2].value, I2C_BYTES[1] as u32);
        assert_eq!(period[3].value, I2C_BYTES[2] as u32);
        assert_eq!(period[4].kind, Kind::Stop);
        assert!(
            period.iter().all(|a| a.flags == 0),
            "all bytes acknowledged: {period:?}"
        );
    }
}

#[test]
fn engine_thread_decodes_a_live_capture() {
    use bf_decode::engine::Engine;
    use std::time::{Duration, Instant};
    let g = Generator::new();
    let n = (10e-3 * RATE) as u64;
    let samples: Vec<u32> = (0..n).map(|i| g.at_ns(time_ns(i, RATE))).collect();
    let store = CaptureStore::shared(StoreConfig::new(ChannelCount::new(16).unwrap(), RATE));
    let engine = Engine::new();
    engine.set_store(store.clone());
    let h = engine.add(Box::new(Uart::new(UartConfig {
        rx: ch::UART,
        ..Default::default()
    })));
    // The capture grows while the engine works.
    for part in samples.chunks(10_000) {
        store.write().push_samples(part);
        std::thread::sleep(Duration::from_millis(1));
    }
    let t0 = Instant::now();
    while engine.busy() && t0.elapsed() < Duration::from_secs(5) {
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(h.decoded_to(), n);
    let anns = h.anns.read();
    let bytes: Vec<u8> = anns.lanes[0].iter().map(|a| a.value as u8).collect();
    assert!(bytes.len() >= 2 * UART_MESSAGE.len());
    assert!(bytes.windows(UART_MESSAGE.len()).any(|w| w == UART_MESSAGE));
    // A new capture restarts the decoder and drops the old annotations.
    drop(anns);
    let fresh = CaptureStore::shared(StoreConfig::new(ChannelCount::new(16).unwrap(), RATE));
    engine.set_store(fresh.clone());
    std::thread::sleep(Duration::from_millis(50));
    fresh.write().push_samples(&samples[..4_000]);
    let t0 = Instant::now();
    while h.decoded_to() < 4_000 && t0.elapsed() < Duration::from_secs(5) {
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(h.decoded_to(), 4_000);
    assert!(h.anns.read().lanes[0].iter().all(|a| a.end <= 4_000 + 200));
}
