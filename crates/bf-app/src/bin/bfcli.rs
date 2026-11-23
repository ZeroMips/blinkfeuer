//! Headless capture tool: runs a capture into the store and prints statistics.

use bf_core::{CaptureStore, ChannelCount, StoreConfig};
use bf_device::litex::LitexDevice;
use bf_device::session::Session;
use bf_device::sim::SimDevice;
use bf_device::stock::StockDevice;
use bf_device::{CaptureConfig, Device, Pattern, ReadResult};
use clap::{Parser, Subcommand, ValueEnum};
use std::time::{Duration, Instant};

#[derive(Parser)]
#[command(name = "bfcli", about = "blinkfeuer command line tool")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Clone, Copy, ValueEnum)]
enum PatternArg {
    Inputs,
    Counter,
    Generator,
    Emulation,
}

#[derive(Subcommand)]
enum Cmd {
    /// List the available devices.
    Devices,
    /// Read a sigrok session file (.sr) and print what is in it; with `--out`, convert it to a Blinkfeuer capture.
    Import {
        input: String,
        /// Write the capture here (.bfc).
        #[arg(long)]
        out: Option<String>,
    },
    /// Capture into memory and print statistics.
    Capture {
        /// Device: `sim` (simulator, as fast as possible), `sim-rt` (paced in real time) or `stock` (SLogic16U3, stock firmware) or `litex` (SLogic16U3, open gateware).
        #[arg(long, default_value = "sim")]
        device: String,
        /// Channels (4, 8 or 16).
        #[arg(long, default_value_t = 16)]
        channels: u8,
        /// Samplerate, e.g. 100M.
        #[arg(long, default_value = "100M", value_parser = parse_si)]
        rate: f64,
        /// Samples to capture, e.g. 10M (0: until --seconds elapsed).
        #[arg(long, default_value = "10M", value_parser = parse_si)]
        samples: f64,
        /// Stop a continuous capture after this many seconds.
        #[arg(long, default_value_t = 5.0)]
        seconds: f64,
        #[arg(long, value_enum, default_value = "generator")]
        pattern: PatternArg,
        /// Store memory cap, e.g. 256M.
        #[arg(long, default_value = "1G", value_parser = parse_si)]
        max_mem: f64,
    },
    /// Read the raw wire stream without storing it (throughput and integrity test of a backend).
    Stream {
        #[arg(long, default_value = "stock")]
        device: String,
        #[arg(long, default_value_t = 16)]
        channels: u8,
        #[arg(long, default_value = "100M", value_parser = parse_si)]
        rate: f64,
        /// Samples to capture (0: until --seconds elapsed).
        #[arg(long, default_value = "10M", value_parser = parse_si)]
        samples: f64,
        #[arg(long, default_value_t = 5.0)]
        seconds: f64,
        #[arg(long, value_enum, default_value = "counter")]
        pattern: PatternArg,
        /// Check that the data is a gap-free counter (8 or 16 channels, counter pattern).
        #[arg(long)]
        verify: bool,
    },
}

/// Streaming check of a counter: every sample must be the previous one plus one.
struct CounterCheck {
    width: usize,
    carry: Vec<u8>,
    prev: Option<u32>,
    first: Option<u32>,
    samples: u64,
    breaks: u64,
    first_break: Option<(u64, u32, u32)>,
}

impl CounterCheck {
    fn new(width: usize) -> Self {
        Self {
            width,
            carry: Vec::new(),
            prev: None,
            first: None,
            samples: 0,
            breaks: 0,
            first_break: None,
        }
    }

    fn feed(&mut self, data: &[u8]) {
        let mask = if self.width == 1 { 0xff } else { 0xffff };
        self.carry.extend_from_slice(data);
        let whole = self.carry.len() / self.width * self.width;
        for c in self.carry[..whole].chunks_exact(self.width) {
            let v = if self.width == 1 {
                c[0] as u32
            } else {
                u16::from_le_bytes([c[0], c[1]]) as u32
            };
            if let Some(p) = self.prev {
                if v != (p + 1) & mask {
                    self.breaks += 1;
                    self.first_break.get_or_insert((self.samples, p, v));
                }
            } else {
                self.first = Some(v);
            }
            self.prev = Some(v);
            self.samples += 1;
        }
        self.carry.drain(..whole);
    }
}

fn parse_si(s: &str) -> Result<f64, String> {
    let s = s.trim();
    let (num, mul) = match s.chars().last() {
        Some('k') | Some('K') => (&s[..s.len() - 1], 1e3),
        Some('M') => (&s[..s.len() - 1], 1e6),
        Some('G') => (&s[..s.len() - 1], 1e9),
        _ => (s, 1.0),
    };
    num.parse::<f64>()
        .map(|v| v * mul)
        .map_err(|e| format!("invalid number '{s}': {e}"))
}

fn open_device(name: &str) -> Result<Box<dyn Device>, String> {
    match name {
        "sim" => Ok(Box::new(SimDevice::new(false))),
        "sim-rt" => Ok(Box::new(SimDevice::new(true))),
        "stock" => StockDevice::open_first()
            .map(|d| Box::new(d) as Box<dyn Device>)
            .map_err(|e| e.to_string()),
        "litex" => LitexDevice::open_first()
            .map(|d| Box::new(d) as Box<dyn Device>)
            .map_err(|e| e.to_string()),
        other => Err(format!(
            "unknown device '{other}' (available: sim, sim-rt, stock, litex)"
        )),
    }
}

fn main() {
    let cli = Cli::parse();
    if let Err(e) = run(cli) {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

fn run(cli: Cli) -> Result<(), String> {
    match cli.cmd {
        Cmd::Devices => {
            for name in ["sim", "sim-rt"] {
                let d = open_device(name)?;
                println!("{:8} {} ({})", name, d.info().name, d.info().backend);
            }
            for d in StockDevice::available() {
                println!("{:8} {} ({}, {})", "stock", d.name, d.backend, d.id);
            }
            for d in LitexDevice::available() {
                println!("{:8} {} ({}, {})", "litex", d.name, d.backend, d.id);
            }
            Ok(())
        }
        Cmd::Import { input, out } => {
            let f = std::fs::File::open(&input).map_err(|e| format!("{input}: {e}"))?;
            let t0 = std::time::Instant::now();
            let c =
                bf_io::read_sr(std::io::BufReader::new(f)).map_err(|e| format!("{input}: {e}"))?;
            let s = &c.store;
            println!(
                "{} probes ({} channels in the store), {} MHz, {} samples ({:.3} s), {} events, read in {:.2} s",
                c.probes,
                s.channels(),
                c.sample_rate / 1e6,
                s.end(),
                s.end() as f64 / c.sample_rate,
                s.event_count(),
                t0.elapsed().as_secs_f64()
            );
            println!("names: {}", c.names.join(", "));
            if c.analog > 0 {
                println!("note: {} analog channels were not imported", c.analog);
            }
            if let Some(out) = out {
                let mut f = std::fs::File::create(&out).map_err(|e| format!("{out}: {e}"))?;
                bf_io::write_capture(&mut f, s, &serde_json::Value::Null)
                    .map_err(|e| format!("{out}: {e}"))?;
                println!("written to {out}");
            }
            Ok(())
        }
        Cmd::Stream {
            device,
            channels,
            rate,
            samples,
            seconds,
            pattern,
            verify,
        } => {
            let ch = ChannelCount::new(channels).ok_or("channels must be 2, 4, 8, 16 or 32")?;
            let mut dev = open_device(&device)?;
            let cfg = CaptureConfig {
                channels: ch,
                sample_rate: rate,
                limit_samples: (samples > 0.0).then_some(samples as u64),
                pattern: match pattern {
                    PatternArg::Inputs => Pattern::Inputs,
                    PatternArg::Counter => Pattern::Counter,
                    PatternArg::Generator => Pattern::Generator,
                    PatternArg::Emulation => Pattern::Emulation,
                },
                threshold_v: 1.7,
            };
            // The firmware's test pattern is a 16-bit counter only in 16 channel mode (the narrow modes
            // show a different repeating pattern), so continuity can only be checked there.
            let width = if channels == 16 { 2 } else { 0 };
            if verify && (width == 0 || cfg.pattern != Pattern::Counter) {
                return Err("--verify needs --pattern counter with 16 channels".into());
            }
            let mut check = CounterCheck::new(width.max(1));
            let mut st = dev.start(&cfg).map_err(|e| e.to_string())?;
            let t0 = Instant::now();
            let mut bytes = 0u64;
            let mut error = None;
            let mut head: Vec<u8> = Vec::new();
            loop {
                match st.read(Duration::from_millis(100)) {
                    Ok(ReadResult::Data(b)) => {
                        if head.len() < 48 {
                            head.extend_from_slice(&b[..b.len().min(48 - head.len())]);
                        }
                        bytes += b.len() as u64;
                        if verify {
                            check.feed(&b);
                        }
                    }
                    Ok(ReadResult::Timeout) => {}
                    Ok(ReadResult::Ended) => break,
                    Err(e) => {
                        error = Some(e);
                        break;
                    }
                }
                if cfg.limit_samples.is_none() && t0.elapsed().as_secs_f64() >= seconds {
                    break;
                }
            }
            let wall = t0.elapsed().as_secs_f64();
            let report = st.stop().map_err(|e| e.to_string())?;
            println!("device     : {}", dev.info().name);
            println!(
                "config     : {} ch @ {} MHz, pattern {}",
                channels,
                rate / 1e6,
                cfg.pattern.name()
            );
            let expect = cfg.limit_samples.map(|n| ch.bytes_for(n));
            println!(
                "wire bytes : {} ({:.1} MB/s over {:.2} s){}",
                bytes,
                bytes as f64 / wall / 1e6,
                wall,
                expect.map_or(String::new(), |e| format!(
                    ", expected {e}: {}",
                    if e == bytes { "exact" } else { "MISMATCH" }
                ))
            );
            println!(
                "head       : {}",
                head.iter()
                    .map(|b| format!("{b:02x}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            );
            if let Some(n) = &report.note {
                println!("transfers  : {n}");
            }
            let mut bad = expect.is_some_and(|e| e != bytes);
            if verify {
                println!(
                    "counter    : {} samples checked, first value {}, {} discontinuities{}",
                    check.samples,
                    check.first.unwrap_or(0),
                    check.breaks,
                    check.first_break.map_or(String::new(), |(i, p, v)| format!(
                        " (first at sample {i}: {p} -> {v})"
                    ))
                );
                bad |= check.breaks > 0;
            }
            if let Some(e) = error {
                return Err(format!("stream error after {bytes} bytes: {e}"));
            }
            if bad {
                return Err("verification failed".into());
            }
            Ok(())
        }
        Cmd::Capture {
            device,
            channels,
            rate,
            samples,
            seconds,
            pattern,
            max_mem,
        } => {
            let channels =
                ChannelCount::new(channels).ok_or("channels must be 2, 4, 8, 16 or 32")?;
            let mut dev = open_device(&device)?;
            let mut scfg = StoreConfig::new(channels, rate);
            scfg.max_bytes = max_mem as usize;
            let store = CaptureStore::shared(scfg);
            let cfg = CaptureConfig {
                channels,
                sample_rate: rate,
                limit_samples: (samples > 0.0).then_some(samples as u64),
                pattern: match pattern {
                    PatternArg::Inputs => Pattern::Inputs,
                    PatternArg::Counter => Pattern::Counter,
                    PatternArg::Generator => Pattern::Generator,
                    PatternArg::Emulation => Pattern::Emulation,
                },
                threshold_v: 1.7,
            };
            let t0 = Instant::now();
            let session =
                Session::start(dev.as_mut(), &cfg, store.clone()).map_err(|e| e.to_string())?;
            if cfg.limit_samples.is_none() {
                std::thread::sleep(Duration::from_secs_f64(seconds));
                session.request_stop();
            }
            let res = session.join();
            let wall = t0.elapsed().as_secs_f64();
            let s = store.read();
            println!("device     : {}", dev.info().name);
            println!(
                "config     : {} ch @ {} MHz, pattern {}",
                channels.get(),
                rate / 1e6,
                cfg.pattern.name()
            );
            println!(
                "samples    : {} ({:.3} s of signal)",
                res.samples,
                res.samples as f64 / rate
            );
            println!(
                "wire bytes : {} ({:.1} MB/s over {:.2} s)",
                res.bytes,
                res.bytes as f64 / wall / 1e6,
                wall
            );
            println!(
                "stored     : {:?} (first sample {})",
                s.range(),
                s.first_sample()
            );
            println!(
                "memory     : {:.2} MB for {} events ({:.4} bytes/sample)",
                s.memory_bytes() as f64 / 1e6,
                s.event_count(),
                s.memory_bytes() as f64 / res.samples.max(1) as f64
            );
            let range = s.range();
            print!("toggles/ch :");
            for ch in 0..channels.get().min(16) {
                print!(" {}", s.count_toggles(ch, range.clone()));
            }
            println!();
            if let Some(e) = res.error {
                return Err(format!("capture error: {e}"));
            }
            Ok(())
        }
    }
}
