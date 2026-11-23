//! Decoder throughput on the simulator's generator pattern.
//! `cargo run --release -p bf-decode --example decode_bench [milliseconds] [rate_MHz]`

use bf_core::{CaptureStore, ChannelCount, StoreConfig};
use bf_decode::engine::decode_all;
use bf_decode::i2c::{I2c, I2cConfig};
use bf_decode::spi::{Spi, SpiConfig};
use bf_decode::uart::{Uart, UartConfig};
use bf_decode::Decoder;
use bf_device::sim::{ch, time_ns, Generator};
use std::time::Instant;

fn main() {
    let ms: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(1000.0);
    let rate: f64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(100.0)
        * 1e6;
    let n = (ms * 1e-3 * rate) as u64;
    let g = Generator::new();
    let t0 = Instant::now();
    let mut store = CaptureStore::new(StoreConfig::new(ChannelCount::new(16).unwrap(), rate));
    let mut buf = Vec::with_capacity(1 << 16);
    let mut i = 0;
    while i < n {
        buf.clear();
        buf.extend((i..(i + (1 << 16)).min(n)).map(|k| g.at_ns(time_ns(k, rate))));
        store.push_samples(&buf);
        i += buf.len() as u64;
    }
    println!(
        "capture: {} samples ({:.2} s at {} MHz), {} events, built in {:.2} s",
        n,
        n as f64 / rate,
        rate / 1e6,
        store.event_count(),
        t0.elapsed().as_secs_f64()
    );

    let decoders: Vec<(&str, Box<dyn Decoder>)> = vec![
        (
            "UART",
            Box::new(Uart::new(UartConfig {
                rx: ch::UART,
                ..Default::default()
            })),
        ),
        (
            "SPI",
            Box::new(Spi::new(SpiConfig {
                cs: Some(ch::SPI_CS),
                sck: ch::SPI_SCK,
                mosi: Some(ch::SPI_MOSI),
                ..Default::default()
            })),
        ),
        (
            "I2C",
            Box::new(I2c::new(I2cConfig {
                scl: ch::I2C_SCL,
                sda: ch::I2C_SDA,
            })),
        ),
    ];
    for (name, d) in decoders {
        let t = Instant::now();
        let a = decode_all(d, &store);
        let dt = t.elapsed().as_secs_f64();
        println!(
            "{name:5}: {:>9} annotations in {:>8.2} ms  = {:>9.0} Msamples/s of capture",
            a.len(),
            dt * 1e3,
            n as f64 / dt / 1e6
        );
    }
}
