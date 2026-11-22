//! Import of sigrok session files (`.sr`), the format of PulseView and sigrok-cli.
//!
//! A session file is a zip archive with a `version` file, a `metadata` file in INI syntax and the
//! samples in chunk files `logic-1-1`, `logic-1-2`, ... of `unitsize` bytes per sample (little
//! endian, bit n is probe n+1). Analog channels are not imported.

use crate::capture::Error;
use bf_core::{CaptureStore, ChannelCount, StoreConfig};
use std::collections::HashMap;
use std::io::{Read, Seek};
use zip::ZipArchive;

/// A capture read from a session file.
pub struct SigrokCapture {
    pub store: CaptureStore,
    /// One name per channel of the store (sigrok's probe names, `CHn` for the unused ones).
    pub names: Vec<String>,
    pub sample_rate: f64,
    /// Probes in the file (the store has the next supported channel count, 2, 4, 8, 16 or 32).
    pub probes: usize,
    /// Analog channels in the file, which are not imported.
    pub analog: usize,
}

fn bad<T>(msg: impl Into<String>) -> Result<T, Error> {
    Err(Error::Format(msg.into()))
}

/// "1 MHz", "200 kHz", "20MHz", "500 Hz", "1000000" -> Hz.
pub fn parse_rate(text: &str) -> Option<f64> {
    let t = text.trim();
    let split = t.find(|c: char| !(c.is_ascii_digit() || c == '.')).unwrap_or(t.len());
    let value: f64 = t[..split].parse().ok()?;
    let mult = match t[split..].trim().to_ascii_lowercase().as_str() {
        "" | "hz" => 1.0,
        "khz" => 1e3,
        "mhz" => 1e6,
        "ghz" => 1e9,
        _ => return None,
    };
    let r = value * mult;
    (r.is_finite() && r > 0.0).then_some(r)
}

/// `[section]` / `key=value` lines.
fn parse_ini(text: &str) -> Vec<(String, HashMap<String, String>)> {
    let mut out: Vec<(String, HashMap<String, String>)> = Vec::new();
    for line in text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with([';', '#'])) {
        if let Some(name) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            out.push((name.trim().to_string(), HashMap::new()));
        } else if let (Some((k, v)), Some(sec)) = (line.split_once('='), out.last_mut()) {
            sec.1.insert(k.trim().to_ascii_lowercase(), v.trim().to_string());
        }
    }
    out
}

/// Read a session file. Everything is checked: a damaged or unsupported file is an error.
pub fn read_sr<R: Read + Seek>(r: R) -> Result<SigrokCapture, Error> {
    let mut zip = ZipArchive::new(r).map_err(|_| Error::Format("not a sigrok session file (not a zip archive)".into()))?;
    let read_text = |zip: &mut ZipArchive<R>, name: &str| -> Result<Option<String>, Error> {
        match zip.by_name(name) {
            Ok(f) => {
                let mut s = String::new();
                f.take(1 << 20).read_to_string(&mut s).map_err(|e| Error::Format(format!("{name}: {e}")))?;
                Ok(Some(s))
            }
            Err(_) => Ok(None),
        }
    };
    if let Some(v) = read_text(&mut zip, "version")? {
        if !matches!(v.trim().chars().next(), Some('1' | '2')) {
            return bad(format!("sigrok session version '{}' is not supported", v.trim()));
        }
    }
    let Some(meta) = read_text(&mut zip, "metadata")? else { return bad("not a sigrok session file (no metadata)") };
    let sections = parse_ini(&meta);
    let Some((_, dev)) = sections.iter().find(|(n, k)| n.starts_with("device") && k.contains_key("capturefile")) else {
        return bad("the session has no logic data (no device with a capture file)");
    };
    let num = |key: &str| dev.get(key).and_then(|v| v.parse::<usize>().ok());
    let probes = num("total probes").unwrap_or(0);
    if probes == 0 {
        return bad("the session has no logic probes");
    }
    if probes > 32 {
        return bad(format!("{probes} channels are more than the 32 Blinkfeuer handles"));
    }
    let unitsize = num("unitsize").unwrap_or(probes.div_ceil(8));
    if unitsize == 0 || unitsize > 4 || unitsize < probes.div_ceil(8) {
        return bad(format!("unit size {unitsize} does not fit {probes} probes"));
    }
    let rate_text = dev.get("samplerate").ok_or_else(|| Error::Format("the session has no sample rate".into()))?;
    let sample_rate = parse_rate(rate_text).ok_or_else(|| Error::Format(format!("sample rate '{rate_text}' is not understood")))?;
    let channels = [2u8, 4, 8, 16, 32].into_iter().find(|c| *c as usize >= probes).and_then(ChannelCount::new).expect("probes <= 32");
    let mut names: Vec<String> = (0..channels.get() as usize).map(|i| format!("CH{i}")).collect();
    for (i, n) in names.iter_mut().enumerate().take(probes) {
        if let Some(name) = dev.get(&format!("probe{}", i + 1)).filter(|v| !v.is_empty()) {
            *n = name.clone();
        }
    }
    let capturefile = dev["capturefile"].clone();

    // The chunks, in order of their number.
    let prefix = format!("{capturefile}-");
    let mut chunks: Vec<(u64, String)> = zip
        .file_names()
        .filter_map(|n| n.strip_prefix(&prefix).and_then(|s| s.parse::<u64>().ok()).map(|k| (k, n.to_string())))
        .collect();
    chunks.sort();
    if chunks.is_empty() {
        return bad(format!("the session has no sample data ('{capturefile}-1' is missing)"));
    }

    let mut cfg = StoreConfig::new(channels, sample_rate);
    cfg.max_bytes = usize::MAX / 2;
    let mut store = CaptureStore::new(cfg);
    let mask = if probes == 32 { u32::MAX } else { (1u32 << probes) - 1 };
    let mut buf = vec![0u8; unitsize * (1 << 16)];
    let mut words: Vec<u32> = Vec::with_capacity(1 << 16);
    for (_, name) in &chunks {
        let mut f = zip.by_name(name).map_err(|e| Error::Format(format!("{name}: {e}")))?;
        let mut have = 0usize;
        loop {
            let n = f.read(&mut buf[have..]).map_err(|e| Error::Format(format!("{name}: {e}")))?;
            if n == 0 {
                break;
            }
            have += n;
            let whole = have / unitsize * unitsize;
            words.clear();
            words.extend(buf[..whole].chunks_exact(unitsize).map(|u| u.iter().rev().fold(0u32, |a, b| a << 8 | *b as u32) & mask));
            store.push_samples(&words);
            buf.copy_within(whole..have, 0);
            have -= whole;
        }
        if have != 0 {
            return bad(format!("{name} ends in the middle of a sample"));
        }
    }
    if store.is_empty() {
        return bad("the session holds no samples");
    }
    let analog = num("total analog").unwrap_or(0);
    Ok(SigrokCapture { store, names, sample_rate, probes, analog })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Cursor, Write};
    use zip::write::SimpleFileOptions;

    fn session(meta: &str, chunks: &[(&str, Vec<u8>)], version: Option<&str>) -> Vec<u8> {
        let mut z = zip::ZipWriter::new(Cursor::new(Vec::new()));
        let o = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Stored);
        if let Some(v) = version {
            z.start_file("version", o).unwrap();
            z.write_all(v.as_bytes()).unwrap();
        }
        z.start_file("metadata", o).unwrap();
        z.write_all(meta.as_bytes()).unwrap();
        for (name, data) in chunks {
            z.start_file(*name, o).unwrap();
            z.write_all(data).unwrap();
        }
        z.finish().unwrap().into_inner()
    }

    const META8: &str = "[global]\nsigrok version=0.5.1\n\n[device 1]\ncapturefile=logic-1\ntotal probes=8\nsamplerate=2 MHz\ntotal analog=0\nprobe1=SCL\nprobe2=SDA\nunitsize=1\n";

    #[test]
    fn rates_are_parsed() {
        assert_eq!(parse_rate("1 MHz"), Some(1e6));
        assert_eq!(parse_rate("200 kHz"), Some(200e3));
        assert_eq!(parse_rate("20MHz"), Some(20e6));
        assert_eq!(parse_rate("1.5 GHz"), Some(1.5e9));
        assert_eq!(parse_rate("500 Hz"), Some(500.0));
        assert_eq!(parse_rate("1000000"), Some(1e6));
        assert_eq!(parse_rate("fast"), None);
        assert_eq!(parse_rate("0 Hz"), None);
        assert_eq!(parse_rate("1 THz"), None);
        assert_eq!(parse_rate(""), None);
    }

    #[test]
    fn a_session_is_imported_with_names_rate_and_samples() {
        // Two chunks: the samples continue across them.
        let a: Vec<u8> = (0..100u8).collect();
        let b: Vec<u8> = (100..200u8).collect();
        let file = session(META8, &[("logic-1-2", b), ("logic-1-1", a)], Some("2"));
        let c = read_sr(Cursor::new(file)).unwrap();
        assert_eq!((c.probes, c.sample_rate, c.analog), (8, 2e6, 0));
        assert_eq!(c.store.channels(), 8);
        assert_eq!(c.store.range(), 0..200, "chunks are put in numeric order");
        for s in [0u64, 1, 99, 100, 199] {
            assert_eq!(c.store.state_at(s), Some(s as u32), "sample {s}");
        }
        assert_eq!(&c.names[..3], ["SCL", "SDA", "CH2"]);
        assert_eq!(c.names.len(), 8);
    }

    #[test]
    fn wide_units_and_odd_probe_counts() {
        // 12 probes: 2 bytes per sample, little endian, stored in a 16 channel store; bits above the probes are cut.
        let mut data = Vec::new();
        for v in [0x0123u16, 0xF456, 0x0FFF] {
            data.extend_from_slice(&v.to_le_bytes());
        }
        let meta = "[device 1]\ncapturefile=logic-1\ntotal probes=12\nsamplerate=100 kHz\nunitsize=2\n";
        let c = read_sr(Cursor::new(session(meta, &[("logic-1-1", data)], Some("2")))).unwrap();
        assert_eq!(c.store.channels(), 16);
        assert_eq!(c.store.range(), 0..3);
        assert_eq!([0, 1, 2].map(|s| c.store.state_at(s).unwrap()), [0x0123, 0x0456, 0x0FFF]);
        // 3 probes need a 4 channel store; unitsize defaults to what the probes need.
        let meta = "[device 1]\ncapturefile=logic-1\ntotal probes=3\nsamplerate=1 MHz\n";
        let c = read_sr(Cursor::new(session(meta, &[("logic-1-1", vec![5, 2, 7])], None))).unwrap();
        assert_eq!(c.store.channels(), 4);
        assert_eq!([0, 1, 2].map(|s| c.store.state_at(s).unwrap()), [5, 2, 7]);
    }

    #[test]
    fn broken_sessions_are_refused_with_a_reason() {
        let ok = vec![("logic-1-1", vec![1u8, 2, 3])];
        let err = |bytes: Vec<u8>| read_sr(Cursor::new(bytes)).err().map(|e| e.to_string()).unwrap_or_default();
        assert!(err(b"hello".to_vec()).contains("not a zip"));
        assert!(err(session(META8, &ok, Some("3"))).contains("version"));
        let no_meta = {
            let mut z = zip::ZipWriter::new(Cursor::new(Vec::new()));
            z.start_file("logic-1-1", SimpleFileOptions::default()).unwrap();
            z.write_all(&[1, 2]).unwrap();
            z.finish().unwrap().into_inner()
        };
        assert!(err(no_meta).contains("metadata"));
        assert!(err(session("[global]\nx=1\n", &ok, None)).contains("no logic data"));
        assert!(err(session(&META8.replace("total probes=8", "total probes=40"), &ok, None)).contains("more than the 32"));
        assert!(err(session(&META8.replace("samplerate=2 MHz\n", ""), &ok, None)).contains("sample rate"));
        assert!(err(session(&META8.replace("2 MHz", "fast"), &ok, None)).contains("not understood"));
        assert!(err(session(&META8.replace("unitsize=1", "unitsize=9"), &ok, None)).contains("unit size"));
        assert!(err(session(META8, &[], None)).contains("no sample data"));
        assert!(err(session(META8, &[("logic-1-1", vec![])], None)).contains("no samples"));
        let two = "[device 1]\ncapturefile=logic-1\ntotal probes=16\nsamplerate=1 MHz\nunitsize=2\n";
        assert!(err(session(two, &[("logic-1-1", vec![1, 2, 3])], None)).contains("middle of a sample"));
        // Analog-only sessions have no logic device.
        assert!(err(session("[device 1]\ncapturefile=analog-1\ntotal analog=1\nsamplerate=1 MHz\n", &[], None)).contains("no logic probes"));
    }

    #[test]
    fn a_large_session_is_read_in_pieces() {
        // More than one read buffer (64 Ki samples), an edge every 1000 samples.
        let data: Vec<u8> = (0..300_000u32).map(|i| (i / 1000 % 2) as u8).collect();
        let c = read_sr(Cursor::new(session(META8, &[("logic-1-1", data)], Some("2")))).unwrap();
        assert_eq!(c.store.range(), 0..300_000);
        assert_eq!(c.store.count_toggles(0, 0..300_000), 299);
    }
}
