//! Stage timing of the capture path on the simulator: device read, unpack, store push.
use bf_core::{CaptureStore, ChannelCount, StoreConfig, Unpacker};
use bf_device::sim::SimDevice;
use bf_device::{CaptureConfig, Device, Pattern, ReadResult};
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let pattern = match args.get(1).map(String::as_str) {
        Some("counter") => Pattern::Counter,
        _ => Pattern::Generator,
    };
    let samples: u64 = args
        .get(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(100_000_000);
    let channels = ChannelCount::new(16).unwrap();
    let cfg = CaptureConfig {
        channels,
        sample_rate: 200e6,
        limit_samples: Some(samples),
        pattern,
        threshold_v: 1.8,
    };
    let mut dev = SimDevice::new(false);
    let mut stream = dev.start(&cfg).unwrap();
    // Stage 1: device read only (buffers kept).
    let t0 = Instant::now();
    let mut bufs: Vec<Vec<u8>> = Vec::new();
    loop {
        match stream.read(Duration::from_millis(50)).unwrap() {
            ReadResult::Data(b) => bufs.push(b.to_vec()),
            ReadResult::Ended => break,
            ReadResult::Timeout => {}
        }
    }
    let t_read = t0.elapsed().as_secs_f64();
    let bytes: usize = bufs.iter().map(|b| b.len()).sum();
    // Stage 2: unpack into one reused vector (as the capture session does).
    let mut up = Unpacker::new(channels);
    let mut scratch: Vec<u32> = Vec::new();
    let t1 = Instant::now();
    for b in &bufs {
        scratch.clear();
        up.unpack(b, &mut scratch);
    }
    let t_unpack = t1.elapsed().as_secs_f64();
    // Words kept for stage 3.
    let mut up = Unpacker::new(channels);
    let mut words: Vec<Vec<u32>> = Vec::with_capacity(bufs.len());
    for b in &bufs {
        let mut w = Vec::new();
        up.unpack(b, &mut w);
        words.push(w);
    }
    // Stage 3: store push.
    let mut sc = StoreConfig::new(channels, cfg.sample_rate);
    sc.max_bytes = 1 << 30;
    let mut store = CaptureStore::new(sc);
    let t2 = Instant::now();
    for w in &words {
        store.push_samples(w);
    }
    let t_push = t2.elapsed().as_secs_f64();
    let mb = bytes as f64 / 1e6;
    println!(
        "{} bytes in {} buffers | device read {:.0} MB/s | unpack {:.0} MB/s | push {:.0} MB/s | stored {:.2} bytes/sample",
        bytes,
        bufs.len(),
        mb / t_read,
        mb / t_unpack,
        mb / t_push,
        store.memory_bytes() as f64 / samples as f64
    );
}
