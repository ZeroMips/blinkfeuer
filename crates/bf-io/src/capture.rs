//! The capture file.
//!
//! ```text
//! 0   8  magic "BFCAP\r\n\x1a"
//! 8   4  format version (1)
//! 12  4  length of the JSON header
//! 16  8  number of events
//! 24  4  CRC-32 of the uncompressed event stream
//! 28  4  reserved (0)
//! 32  n  JSON header: sample rate, channels, first/end sample, state of the first sample, setup
//! ..     deflate stream of the events
//! ```
//! An event is a sample where the state changed: `varint(sample - previous event sample)` (the
//! first one counts from the first stored sample) and `varint(state ^ previous state)`. This is the
//! store's own representation, so the file is as small as the capture is quiet, and loading costs
//! time proportional to the number of events.

use bf_core::{CaptureStore, ChannelCount, StoreConfig};
use flate2::read::DeflateDecoder;
use flate2::write::DeflateEncoder;
use flate2::{Compression, Crc};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::io::{self, BufReader, Read, Seek, SeekFrom, Write};

const MAGIC: &[u8; 8] = b"BFCAP\r\n\x1a";
const VERSION: u32 = 1;
const MAX_HEADER: usize = 64 << 20;

#[derive(Debug)]
pub enum Error {
    Io(io::Error),
    /// Not a capture file, damaged, or from a newer version.
    Format(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "{e}"),
            Error::Format(m) => write!(f, "{m}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        // A stream that ends early is a damaged file, not an I/O problem.
        if e.kind() == io::ErrorKind::UnexpectedEof {
            Error::Format("the file is truncated".into())
        } else {
            Error::Io(e)
        }
    }
}

fn bad<T>(msg: impl Into<String>) -> Result<T, Error> {
    Err(Error::Format(msg.into()))
}

#[derive(Serialize, Deserialize)]
struct Header {
    sample_rate: f64,
    channels: u8,
    first_sample: u64,
    end: u64,
    initial_state: u32,
    /// Sample at which the trigger fired.
    #[serde(default)]
    trigger: Option<u64>,
    generator: String,
    #[serde(default)]
    setup: serde_json::Value,
}

/// A capture read from a file.
pub struct Loaded {
    pub store: CaptureStore,
    /// The application setup saved with the capture (`null` if there was none).
    pub setup: serde_json::Value,
}

fn put_varint(buf: &mut Vec<u8>, mut v: u64) {
    while v >= 0x80 {
        buf.push((v as u8) | 0x80);
        v >>= 7;
    }
    buf.push(v as u8);
}

/// Write the stored part of `store` and the application `setup` to `w`.
pub fn write_capture<W: Write + Seek>(
    w: &mut W,
    store: &CaptureStore,
    setup: &serde_json::Value,
) -> Result<(), Error> {
    let first = store.first_sample();
    let header = Header {
        sample_rate: store.sample_rate(),
        channels: store.channels(),
        first_sample: first,
        end: store.end(),
        initial_state: store.state_at(first).unwrap_or(0),
        trigger: store.trigger(),
        generator: format!("blinkfeuer {}", env!("CARGO_PKG_VERSION")),
        setup: setup.clone(),
    };
    let json = serde_json::to_vec(&header).map_err(|e| Error::Format(e.to_string()))?;
    w.write_all(MAGIC)?;
    w.write_all(&VERSION.to_le_bytes())?;
    w.write_all(&(json.len() as u32).to_le_bytes())?;
    w.write_all(&[0u8; 16])?; // event count and CRC, patched below
    w.write_all(&json)?;

    let mut enc = DeflateEncoder::new(&mut *w, Compression::fast());
    let mut crc = Crc::new();
    let mut buf: Vec<u8> = Vec::with_capacity(1 << 17);
    let mut failure: Option<io::Error> = None;
    let mut count = 0u64;
    let (mut prev_pos, mut prev_state) = (first, header.initial_state);
    let mask = ChannelCount::new(store.channels()).map_or(u32::MAX, |c| c.mask());
    store.changes(mask, first..store.end(), |p, state| {
        if failure.is_some() {
            return;
        }
        put_varint(&mut buf, p - prev_pos);
        put_varint(&mut buf, (state ^ prev_state) as u64);
        (prev_pos, prev_state) = (p, state);
        count += 1;
        if buf.len() >= 1 << 16 {
            crc.update(&buf);
            if let Err(e) = enc.write_all(&buf) {
                failure = Some(e);
            }
            buf.clear();
        }
    });
    if let Some(e) = failure {
        return Err(e.into());
    }
    crc.update(&buf);
    enc.write_all(&buf)?;
    enc.finish()?;

    let end = w.stream_position()?;
    w.seek(SeekFrom::Start(16))?;
    w.write_all(&count.to_le_bytes())?;
    w.write_all(&crc.sum().to_le_bytes())?;
    w.seek(SeekFrom::Start(end))?;
    w.flush()?;
    Ok(())
}

struct CrcReader<R> {
    inner: R,
    crc: Crc,
}

impl<R: Read> Read for CrcReader<R> {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        let n = self.inner.read(out)?;
        self.crc.update(&out[..n]);
        Ok(n)
    }
}

fn get_varint(r: &mut impl Read) -> Result<u64, Error> {
    let (mut v, mut shift) = (0u64, 0u32);
    loop {
        let mut b = [0u8; 1];
        r.read_exact(&mut b)?;
        if shift >= 64 || (shift == 63 && b[0] > 1) {
            return bad("damaged event stream (number too large)");
        }
        v |= ((b[0] & 0x7f) as u64) << shift;
        if b[0] & 0x80 == 0 {
            return Ok(v);
        }
        shift += 7;
    }
}

/// Append `n` samples of constant `state`.
fn push_run(store: &mut CaptureStore, state: u32, mut n: u64, buf: &mut [u32]) {
    while n > 0 {
        let k = (n as usize).min(buf.len());
        buf[..k].fill(state);
        store.push_samples(&buf[..k]);
        n -= k as u64;
    }
}

/// Read a capture file. Everything is checked, a damaged file gives an error, never a panic.
pub fn read_capture<R: Read>(r: R) -> Result<Loaded, Error> {
    let mut r = BufReader::with_capacity(1 << 16, r);
    let mut head = [0u8; 32];
    r.read_exact(&mut head)
        .map_err(|_| Error::Format("not a blinkfeuer capture file".into()))?;
    if &head[..8] != MAGIC {
        return bad("not a blinkfeuer capture file");
    }
    let version = u32::from_le_bytes(head[8..12].try_into().unwrap());
    if version != VERSION {
        return bad(format!("capture file version {version} is not supported (this program reads version {VERSION})"));
    }
    let json_len = u32::from_le_bytes(head[12..16].try_into().unwrap()) as usize;
    let count = u64::from_le_bytes(head[16..24].try_into().unwrap());
    let crc_expected = u32::from_le_bytes(head[24..28].try_into().unwrap());
    if json_len > MAX_HEADER {
        return bad("damaged file header");
    }
    let mut json = vec![0u8; json_len];
    r.read_exact(&mut json)?;
    let h: Header = serde_json::from_slice(&json)
        .map_err(|e| Error::Format(format!("damaged file header: {e}")))?;

    let Some(channels) = ChannelCount::new(h.channels) else {
        return bad(format!("{} channels is not supported", h.channels));
    };
    if !(h.sample_rate.is_finite() && h.sample_rate > 0.0) {
        return bad("damaged file header (sample rate)");
    }
    if h.end < h.first_sample || h.initial_state & !channels.mask() != 0 {
        return bad("damaged file header");
    }
    let mut cfg = StoreConfig::new(channels, h.sample_rate);
    cfg.max_bytes = usize::MAX / 2; // a saved capture is loaded completely
    let mut store = CaptureStore::new_at(cfg, h.first_sample);

    let mut ev = CrcReader {
        inner: BufReader::with_capacity(1 << 16, DeflateDecoder::new(r)),
        crc: Crc::new(),
    };
    let mut buf = vec![0u32; 1 << 16];
    let (mut pos, mut state) = (h.first_sample, h.initial_state);
    for _ in 0..count {
        let delta = get_varint(&mut ev)?;
        let x = get_varint(&mut ev)?;
        let Some(p) = pos.checked_add(delta).filter(|p| delta > 0 && *p < h.end) else {
            return bad("damaged event stream (position out of range)");
        };
        if x == 0 || x > channels.mask() as u64 {
            return bad("damaged event stream (state out of range)");
        }
        push_run(&mut store, state, p - pos, &mut buf);
        state ^= x as u32;
        pos = p;
    }
    push_run(&mut store, state, h.end - pos, &mut buf);
    store.set_trigger(h.trigger.filter(|t| (h.first_sample..h.end).contains(t)));
    if ev.crc.sum() != crc_expected {
        return bad("the capture data is damaged (checksum mismatch)");
    }
    Ok(Loaded {
        store,
        setup: h.setup,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn store_from(samples: &[u32], channels: u8, start: u64) -> CaptureStore {
        let mut cfg = StoreConfig::new(ChannelCount::new(channels).unwrap(), 25e6);
        cfg.block_samples = 64;
        cfg.chunk_samples = 256;
        let mut s = CaptureStore::new_at(cfg, start);
        for c in samples.chunks(100) {
            s.push_samples(c);
        }
        s
    }

    fn roundtrip(s: &CaptureStore, setup: serde_json::Value) -> Loaded {
        let mut file = Cursor::new(Vec::new());
        write_capture(&mut file, s, &setup).unwrap();
        read_capture(&file.get_ref()[..]).unwrap()
    }

    fn same(a: &CaptureStore, b: &CaptureStore) {
        assert_eq!(a.range(), b.range());
        assert_eq!(a.channels(), b.channels());
        assert_eq!(a.sample_rate(), b.sample_rate());
        for s in a.range() {
            assert_eq!(a.state_at(s), b.state_at(s), "sample {s}");
        }
        let (mut ea, mut eb) = (Vec::new(), Vec::new());
        a.changes(u32::MAX, a.range(), |p, s| ea.push((p, s)));
        b.changes(u32::MAX, b.range(), |p, s| eb.push((p, s)));
        assert_eq!(ea, eb);
    }

    fn pseudo(n: usize, seed: u32, quiet: u32) -> Vec<u32> {
        let mut x = seed;
        let mut v = 0u32;
        (0..n)
            .map(|_| {
                x = x.wrapping_mul(1664525).wrapping_add(1013904223);
                if (x >> 8) % quiet == 0 {
                    v ^= 1 << ((x >> 20) % 16);
                }
                v
            })
            .collect()
    }

    #[test]
    fn captures_survive_a_round_trip() {
        for (n, quiet) in [
            (1usize, 1u32),
            (2, 1),
            (1000, 1),
            (5000, 7),
            (20_000, 400),
            (9000, 1_000_000),
        ] {
            let src = store_from(&pseudo(n, n as u32, quiet), 16, 0);
            let l = roundtrip(&src, serde_json::json!({"hello": [1, 2, 3]}));
            same(&src, &l.store);
            assert_eq!(l.setup["hello"][1], 2);
        }
    }

    #[test]
    fn evicted_captures_keep_their_sample_numbers() {
        let samples: Vec<u32> = pseudo(3000, 9, 5).into_iter().map(|w| w & 0xff).collect();
        let src = store_from(&samples, 8, 1_000_003);
        assert_eq!(src.first_sample(), 1_000_003);
        let l = roundtrip(&src, serde_json::Value::Null);
        assert_eq!(l.store.first_sample(), 1_000_003);
        assert!(l.setup.is_null());
        same(&src, &l.store);
    }

    #[test]
    fn empty_captures_round_trip() {
        let src = store_from(&[], 4, 0);
        let l = roundtrip(&src, serde_json::Value::Null);
        assert!(l.store.is_empty());
        assert_eq!(l.store.channels(), 4);
    }

    #[test]
    fn quiet_captures_are_small() {
        // 10 M samples with a few edges: the file must not grow with the sample count.
        let mut cfg = StoreConfig::new(ChannelCount::new(16).unwrap(), 100e6);
        cfg.chunk_samples = 1 << 20;
        let mut s = CaptureStore::new(cfg);
        let block = vec![0u32; 1 << 16];
        let high = vec![1u32; 1 << 16];
        for i in 0..160 {
            s.push_samples(if i % 40 == 0 { &high } else { &block });
        }
        let mut file = Cursor::new(Vec::new());
        write_capture(&mut file, &s, &serde_json::Value::Null).unwrap();
        assert!(
            file.get_ref().len() < 2000,
            "{} bytes",
            file.get_ref().len()
        );
        same(&s, &read_capture(&file.get_ref()[..]).unwrap().store);
    }

    #[test]
    fn damaged_files_are_refused() {
        let src = store_from(&pseudo(5000, 3, 3), 16, 0);
        let mut file = Cursor::new(Vec::new());
        write_capture(&mut file, &src, &serde_json::Value::Null).unwrap();
        let good = file.into_inner();
        assert!(read_capture(&good[..]).is_ok());
        // Wrong magic, wrong version, truncation at every kind of place, a flipped payload byte.
        let mut b = good.clone();
        b[0] = b'X';
        assert!(read_capture(&b[..]).is_err());
        let mut b = good.clone();
        b[8] = 99;
        assert!(matches!(read_capture(&b[..]), Err(Error::Format(m)) if m.contains("version")));
        for cut in [0, 7, 31, 40, good.len() / 4] {
            assert!(read_capture(&good[..cut]).is_err(), "cut at {cut}");
        }
        // Cut inside the end of the stream: either refused, or (all events present, checksum fine) intact.
        for cut in [good.len() / 2, good.len() - 1] {
            if let Ok(l) = read_capture(&good[..cut]) {
                same(&src, &l.store);
            }
        }
        for at in [good.len() - 5, good.len() / 2 + 7] {
            let mut b = good.clone();
            b[at] ^= 0x55;
            assert!(read_capture(&b[..]).is_err(), "flip at {at}");
        }
        // Absurd header length and garbage.
        let mut b = good.clone();
        b[12..16].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(read_capture(&b[..]).is_err());
        assert!(read_capture(&b"hello"[..]).is_err());
        assert!(read_capture(&[][..]).is_err());
    }

    #[test]
    fn the_trigger_sample_and_a_cut_start_survive() {
        let mut src = store_from(&pseudo(4000, 5, 6), 16, 0);
        src.set_floor(1234);
        src.set_trigger(Some(1500));
        let l = roundtrip(&src, serde_json::Value::Null);
        assert_eq!(l.store.trigger(), Some(1500));
        assert_eq!(l.store.first_sample(), 1234);
        same(&src, &l.store);
        // Without a trigger there is none after loading either.
        let plain = store_from(&pseudo(100, 1, 2), 16, 0);
        assert_eq!(
            roundtrip(&plain, serde_json::Value::Null).store.trigger(),
            None
        );
    }
}
