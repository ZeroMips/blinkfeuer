//! Ingest throughput of the capture store on synthetic data of different densities.
use bf_core::{CaptureStore, ChannelCount, StoreConfig};
use std::time::Instant;

fn lcg(s: &mut u64) -> u64 {
    *s = s
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    *s >> 33
}

fn make(n: usize, run: u64) -> Vec<u32> {
    let mut seed = 1u64;
    let mut v = Vec::with_capacity(n);
    let mut cur = 0u32;
    while v.len() < n {
        let len = if run == 1 {
            1
        } else {
            1 + lcg(&mut seed) % (2 * run)
        };
        cur = (lcg(&mut seed) as u32) & 0xffff;
        for _ in 0..len.min((n - v.len()) as u64) {
            v.push(cur);
        }
    }
    let _ = cur;
    v
}

fn main() {
    let n = 100_000_000usize;
    println!(
        "{:>34} {:>12} {:>10} {:>10}",
        "data", "Msamples/s", "MB/s(16ch)", "bytes/smp"
    );
    for (name, run) in [
        ("sparse (~1 change / 400 samples)", 200u64),
        ("medium (~1 change / 20)", 10),
        ("dense (~1 change / 4)", 2),
        ("worst (every sample)", 1),
    ] {
        let data = make(n, run);
        let mut cfg = StoreConfig::new(ChannelCount::new(16).unwrap(), 200e6);
        cfg.max_bytes = 256 << 20;
        let mut store = CaptureStore::new(cfg);
        let t0 = Instant::now();
        for part in data.chunks(1 << 16) {
            store.push_samples(part);
        }
        let dt = t0.elapsed().as_secs_f64();
        println!(
            "{:>34} {:>12.0} {:>10.0} {:>10.3}",
            name,
            n as f64 / dt / 1e6,
            n as f64 * 2.0 / dt / 1e6,
            store.memory_bytes() as f64 / (store.end() - store.first_sample()) as f64
        );
    }
}
