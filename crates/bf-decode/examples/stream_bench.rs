//! What a protocol trigger would cost: a decoder run directly on the incoming sample stream (find the
//! samples where its channels change, feed those to the decoder), before anything is stored.
//! `cargo run --release -p bf-decode --example stream_bench [milliseconds] [rate_MHz]`

use bf_decode::i2c::{I2c, I2cConfig};
use bf_decode::spi::{Spi, SpiConfig};
use bf_decode::uart::{Uart, UartConfig};
use bf_decode::Decoder;
use bf_device::sim::{ch, time_ns, Generator};
use std::time::Instant;

fn run(d: &mut dyn Decoder, words: &[u32], rate: f64) -> (usize, usize) {
    d.reset(rate);
    let mask = d.mask();
    let mut out = Vec::new();
    let (mut prev, mut changes, mut base) = (words[0], 0usize, 0u64);
    d.change(0, words[0], &mut out);
    for buf in words.chunks(1 << 16) {
        // The scan every trigger needs: where does a watched channel change?
        for (i, &w) in buf.iter().enumerate() {
            if (w ^ prev) & mask != 0 {
                d.change(base + i as u64, w, &mut out);
                changes += 1;
            }
            prev = w;
        }
        base += buf.len() as u64;
        d.advance(base, &mut out);
    }
    (out.len(), changes)
}

fn main() {
    let ms: f64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(200.0);
    let rate: f64 = std::env::args().nth(2).and_then(|s| s.parse::<f64>().ok()).unwrap_or(100.0) * 1e6;
    let n = (ms * 1e-3 * rate) as u64;
    let g = Generator::new();
    let words: Vec<u32> = (0..n).map(|k| g.at_ns(time_ns(k, rate))).collect();
    println!("{} samples ({:.0} ms at {} MHz)", n, ms, rate / 1e6);
    let mut decoders: Vec<(&str, Box<dyn Decoder>)> = vec![
        ("UART", Box::new(Uart::new(UartConfig { rx: ch::UART, ..Default::default() }))),
        ("SPI", Box::new(Spi::new(SpiConfig { cs: Some(ch::SPI_CS), sck: ch::SPI_SCK, mosi: Some(ch::SPI_MOSI), ..Default::default() }))),
        ("I2C", Box::new(I2c::new(I2cConfig { scl: ch::I2C_SCL, sda: ch::I2C_SDA }))),
    ];
    for (name, d) in decoders.iter_mut() {
        let t = Instant::now();
        let (anns, changes) = run(d.as_mut(), &words, rate);
        let dt = t.elapsed().as_secs_f64();
        println!("{name:5}: {anns:>7} annotations, {changes:>8} watched changes, {:>7.2} ms = {:>6.0} MSa/s", dt * 1e3, n as f64 / dt / 1e6);
    }
}
