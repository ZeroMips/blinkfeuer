//! Example traces for the decoder setup: a synthetic waveform generated for the *current settings* and
//! decoded by the real decoder, so the picture shows what the decoder expects and what it makes of it.
//!
//! The trigger setup gets the same kind of example for a protocol trigger: a waveform that contains
//! what the trigger looks for, decoded by the real decoder, with the trigger sample found by running
//! the real trigger over it.

use crate::decode_ui::Config;
use crate::theme;
use crate::view::{format_duration, View};
use crate::waveform::draw_annotations;
use bf_core::{CaptureStore, ChannelCount, StoreConfig};
use bf_decode::engine::decode_all;
use bf_decode::i2c::I2c;
use bf_decode::spi::Spi;
use bf_decode::synth::Wave;
use bf_decode::trigger::{
    I2cMatch, PatItem, ProtocolTrigger, SpiLane, SpiMatch, TriggerAt, UartMatch,
};
use bf_decode::uart::{Uart, UartConfig};
use bf_decode::{Annotation, Decoder, Format};
use egui::{pos2, vec2, Align2, Color32, FontId, Rect, Sense, Shape, Stroke};

/// One signal of the example.
pub struct SigRow {
    pub label: String,
    /// Level per sample.
    pub levels: Vec<bool>,
}

pub struct Preview {
    pub len: usize,
    pub rows: Vec<SigRow>,
    /// Sampling instants: (row, sample).
    pub marks: Vec<(usize, f64)>,
    /// Bit names along the top: (from sample, to sample, text).
    pub bit_labels: Vec<(f64, f64, String)>,
    /// Decoded lanes: (name, annotations).
    pub lanes: Vec<(String, Vec<Annotation>)>,
    pub caption: Vec<String>,
    /// Where a trigger fires (trigger examples only).
    pub trigger: Option<f64>,
}

fn chan(names: &[String], ch: u8) -> String {
    format!(
        "CH{ch} {}",
        names.get(ch as usize).map_or("", |s| s.as_str())
    )
    .trim()
    .to_string()
}

fn levels_of(w: &Wave, ch: u8) -> Vec<bool> {
    w.samples.iter().map(|s| (s >> ch) & 1 != 0).collect()
}

fn decode(w: &Wave, d: Box<dyn Decoder>) -> Vec<Vec<Annotation>> {
    let lanes = d.lanes().len();
    let mut store = CaptureStore::new(StoreConfig::new(ChannelCount::new(32).unwrap(), w.rate));
    store.push_samples(&w.samples);
    let anns = decode_all(d, &store);
    let mut out = vec![Vec::new(); lanes];
    for a in anns {
        out[a.lane as usize].push(a);
    }
    out
}

fn edges(levels: &[bool], rising: bool) -> Vec<f64> {
    (1..levels.len())
        .filter(|&i| levels[i] != levels[i - 1] && levels[i] == rising)
        .map(|i| i as f64)
        .collect()
}

/// Build the example for `cfg`. Channel names label the rows.
pub fn build(cfg: &Config, names: &[String]) -> Preview {
    match cfg {
        Config::Uart(c) => uart(c, names),
        Config::Spi(c) => spi(c, names),
        Config::I2c(c) => i2c(c, names),
    }
}

/// The byte shown by the UART example (with 8 data bits: "K").
pub const UART_EXAMPLE: u32 = 0x4B;

fn uart(cfg: &bf_decode::uart::UartConfig, names: &[String]) -> Preview {
    use bf_decode::uart::Parity;
    const BIT: usize = 16;
    let n = cfg.data_bits as usize;
    let byte = UART_EXAMPLE & ((1u32 << n) - 1);
    let parity = (cfg.parity != Parity::None) as usize;
    let idle = 2 * BIT;
    let frame = ((1 + n + parity) as f64 + cfg.stop_bits as f64) * BIT as f64;
    let len = idle + frame.ceil() as usize + idle;
    let rate = cfg.baud * BIT as f64;
    let mut w = Wave::new(rate, len, if cfg.invert { 0 } else { 1 << cfg.rx });
    w.uart(cfg.rx, idle as u64, cfg, &[byte as u8]);

    let mut bit_labels = Vec::new();
    let mut marks = Vec::new();
    let mut slot = |i: f64, width: f64, text: String| {
        let s0 = idle as f64 + i * BIT as f64;
        bit_labels.push((s0, s0 + width * BIT as f64, text));
        marks.push((0usize, s0 + width * BIT as f64 / 2.0));
    };
    slot(0.0, 1.0, "Start".into());
    for j in 0..n {
        let k = if cfg.lsb_first { j } else { n - 1 - j };
        slot(1.0 + j as f64, 1.0, format!("D{k}"));
    }
    if parity == 1 {
        slot(1.0 + n as f64, 1.0, "P".into());
    }
    let stop_at = (1 + n + parity) as f64;
    {
        let s0 = idle as f64 + stop_at * BIT as f64;
        bit_labels.push((s0, s0 + cfg.stop_bits as f64 * BIT as f64, "Stop".into()));
        marks.push((0, s0 + BIT as f64 / 2.0));
    }
    let lanes = decode(&w, Box::new(Uart::new(cfg.clone())));
    let ascii = if (0x20..0x7f).contains(&byte) {
        format!(" ('{}')", byte as u8 as char)
    } else {
        String::new()
    };
    let mut caption = vec![format!(
        "One frame carrying 0x{byte:02X}{ascii}: {} data bit{} sent {} first, bit time {} ({} baud).",
        n,
        if n == 1 { "" } else { "s" },
        if cfg.lsb_first { "LSB" } else { "MSB" },
        format_duration(1.0 / cfg.baud),
        cfg.baud
    )];
    caption.push(format!(
        "The line idles {}. A start bit ({}) begins the frame; each bit is sampled in its middle (dots).",
        if cfg.invert { "low (inverted)" } else { "high" },
        if cfg.invert { "high" } else { "low" }
    ));
    Preview {
        len,
        rows: vec![SigRow {
            label: format!("RX  {}", chan(names, cfg.rx)),
            levels: levels_of(&w, cfg.rx),
        }],
        marks,
        bit_labels,
        lanes: vec![("decoded".into(), lanes[0].clone())],
        caption,
        trigger: None,
    }
}

fn spi(cfg: &bf_decode::spi::SpiConfig, names: &[String]) -> Preview {
    const HALF: u64 = 8;
    let bits = cfg.bits as u64;
    let lead = 2 * HALF;
    let len = (lead + 2 * HALF * bits + 10 * HALF + lead) as usize;
    let top = |v: u32| if bits >= 32 { v } else { v >> (32 - bits) };
    let (mosi, miso) = ([top(0x96C5_A53B)], [top(0x3A5C_69D1)]);
    let init = (cfg.cpol as u32) << cfg.sck | cfg.cs.map_or(0, |c| (cfg.cs_active_low as u32) << c);
    let mut w = Wave::new(1e6, len, init);
    w.spi(
        cfg,
        lead,
        HALF,
        if cfg.mosi.is_some() { &mosi } else { &[] },
        if cfg.miso.is_some() { &miso } else { &[] },
    );
    let mut rows = Vec::new();
    let mut idx = |label: &str, ch: u8| {
        rows.push(SigRow {
            label: format!("{label:<4} {}", chan(names, ch)),
            levels: levels_of(&w, ch),
        });
        rows.len() - 1
    };
    if let Some(c) = cfg.cs {
        idx(if cfg.cs_active_low { "CS#" } else { "CS" }, c);
    }
    let sck_row = idx("SCK", cfg.sck);
    let mosi_row = cfg.mosi.map(|c| idx("MOSI", c));
    let miso_row = cfg.miso.map(|c| idx("MISO", c));

    // Sampling edges of the clock and the data lines at those instants.
    let sample_rising = cfg.cpol == cfg.cpha;
    let mut marks = Vec::new();
    for e in edges(&rows[sck_row].levels, sample_rising) {
        marks.push((sck_row, e));
        for r in [mosi_row, miso_row].into_iter().flatten() {
            marks.push((r, e));
        }
    }
    let bit_labels = (0..bits)
        .map(|i| {
            let s0 = (lead + HALF + i * 2 * HALF) as f64;
            let k = if cfg.msb_first { bits - 1 - i } else { i };
            (s0, s0 + 2.0 * HALF as f64, k.to_string())
        })
        .collect();
    let decoded = decode(&w, Box::new(Spi::new(cfg.clone())));
    let mut lanes = Vec::new();
    if cfg.mosi.is_some() {
        lanes.push(("MOSI word".to_string(), decoded[0].clone()));
    }
    if cfg.miso.is_some() {
        lanes.push(("MISO word".to_string(), decoded[1].clone()));
    }
    let mode = (cfg.cpol as u8) << 1 | cfg.cpha as u8;
    let mut caption = vec![format!(
        "SPI mode {mode} (CPOL={}, CPHA={}): the clock idles {}; data is sampled on the {} edge (dots).",
        cfg.cpol as u8,
        cfg.cpha as u8,
        if cfg.cpol { "high" } else { "low" },
        if sample_rising { "rising" } else { "falling" },
    )];
    caption.push(format!(
        "{} bits per word, {} bit first{}. Bit numbers are shown above the traces.",
        cfg.bits,
        if cfg.msb_first {
            "most significant"
        } else {
            "least significant"
        },
        match cfg.cs {
            Some(_) => format!(
                "; words are only decoded while CS is {}",
                if cfg.cs_active_low { "low" } else { "high" }
            ),
            None => "; without CS, words follow the clock count".into(),
        }
    ));
    if cfg.mosi.is_none() && cfg.miso.is_none() {
        caption.push("Select MOSI and/or MISO to see data.".into());
    }
    Preview {
        len,
        rows,
        marks,
        bit_labels,
        lanes,
        caption,
        trigger: None,
    }
}

fn i2c(cfg: &bf_decode::i2c::I2cConfig, names: &[String]) -> Preview {
    const Q: u64 = 8;
    let lead = 3 * Q;
    let (scl, sda) = (cfg.scl, cfg.sda);
    let mut w = Wave::new(
        1e6,
        (lead + Q + 2 * 36 * Q + 3 * Q + lead) as usize,
        (1 << scl) | (1 << sda),
    );
    let mut t = w.i2c_start(scl, sda, lead, Q, false);
    let start_end = t;
    let mut labels: Vec<(f64, f64, String)> = vec![(lead as f64, start_end as f64, "S".into())];
    for (byte_names, byte) in [(0u8, 0xA0u8), (1, 0x3C)] {
        for k in 0..9u64 {
            let text = if byte_names == 0 {
                match k {
                    0..=6 => format!("A{}", 6 - k),
                    7 => "W".to_string(),
                    _ => "ACK".to_string(),
                }
            } else if k < 8 {
                format!("D{}", 7 - k)
            } else {
                "ACK".to_string()
            };
            labels.push(((t + k * 4 * Q) as f64, (t + (k + 1) * 4 * Q) as f64, text));
        }
        t = w.i2c_byte(scl, sda, t, Q, byte, false);
    }
    let stop_start = t;
    t = w.i2c_stop(scl, sda, t, Q);
    labels.push((stop_start as f64, t as f64, "P".into()));
    let len = (t + lead) as usize;
    w.samples.truncate(len);
    let rows = vec![
        SigRow {
            label: format!("SCL {}", chan(names, scl)),
            levels: levels_of(&w, scl),
        },
        SigRow {
            label: format!("SDA {}", chan(names, sda)),
            levels: levels_of(&w, sda),
        },
    ];
    // Dots on the SCL rises that carry bits; the rise inside the STOP condition is not one.
    let marks: Vec<(usize, f64)> = edges(&rows[0].levels, true)
        .into_iter()
        .filter(|&e| e < stop_start as f64)
        .map(|e| (1usize, e))
        .collect();
    let decoded = decode(&w, Box::new(I2c::new(cfg.clone())));
    Preview {
        len,
        rows,
        marks,
        bit_labels: labels,
        lanes: vec![("decoded".into(), decoded[0].clone())],
        caption: vec![
            "Both lines idle high. START: SDA falls while SCL is high. STOP: SDA rises while SCL is high.".into(),
            "Each bit is valid while SCL is high (dots on SDA); SDA changes only while SCL is low. The 9th clock is ACK: the receiver pulls SDA low.".into(),
        ],
        trigger: None,
    }
}

/// The example for a protocol trigger: what it looks for, after something it does not, with the
/// trigger sample marked. `None` when the trigger settings are not usable.
pub fn build_trigger(t: &ProtocolTrigger, at: TriggerAt, names: &[String]) -> Option<Preview> {
    t.check().ok()?;
    let mut p = match t {
        ProtocolTrigger::Uart(cfg, UartMatch::ModbusFrames { items, or_bad }) => {
            modbus_trigger(cfg, items, *or_bad, at, names)
        }
        ProtocolTrigger::Uart(cfg, m) => uart_trigger(cfg, m, at, names),
        ProtocolTrigger::Spi(cfg, m) => spi_trigger(cfg, m, names),
        ProtocolTrigger::I2c(cfg, m) => i2c_trigger(cfg, m, names),
    };
    p.0.trigger = trigger_sample(t, at, &p.1);
    if p.0.trigger.is_none() {
        p.0.caption
            .push("The trigger does not fire on this example.".into());
    }
    Some(p.0)
}

/// Run the real trigger over `w`: the sample it reports.
fn trigger_sample(t: &ProtocolTrigger, at: TriggerAt, w: &Wave) -> Option<f64> {
    let mut trg = t.build(at).ok()?;
    trg.reset(w.rate);
    let mask = trg.mask();
    let first = *w.samples.first()?;
    if let Some(s) = trg.change(0, first) {
        return Some(s as f64);
    }
    for i in 1..w.samples.len() {
        if (w.samples[i] ^ w.samples[i - 1]) & mask != 0 {
            if let Some(s) = trg.change(i as u64, w.samples[i]) {
                return Some(s as f64);
            }
        }
    }
    trg.advance(w.samples.len() as u64).map(|s| s as f64)
}

fn at_text(at: TriggerAt) -> &'static str {
    match at {
        TriggerAt::Start => "start",
        TriggerAt::End => "end",
    }
}

/// A character as a label: the character itself if printable, else its hex value.
fn char_label(b: u8) -> String {
    if (0x21..0x7f).contains(&b) {
        format!("'{}'", b as char)
    } else {
        format!("{b:02X}")
    }
}

fn uart_trigger(
    cfg: &UartConfig,
    m: &UartMatch,
    at: TriggerAt,
    names: &[String],
) -> (Preview, Wave) {
    const BIT: usize = 16;
    let cfg = UartConfig {
        telegrams: false,
        protocol: Default::default(),
        ..cfg.clone()
    };
    let mask = ((1u32 << cfg.data_bits.min(8)) - 1) as u8;
    // A character the trigger does not look for, then what it looks for.
    let fits = |c: u8, item: &PatItem| match *item {
        PatItem::Char { value, care } => (c as u32 ^ value) & care & 0xFF == 0,
        _ => true,
    };
    let (chars, broken): (Vec<u8>, bool) = match m {
        UartMatch::Pattern { items, .. } => {
            let other = [0x2E, 0x20, 0x55, 0x00]
                .into_iter()
                .map(|b: u8| b & mask)
                .find(|&b| items.first().map_or(true, |i| !fits(b, i)))
                .unwrap_or(0);
            let mut chars = vec![other];
            for it in items {
                match *it {
                    // Free bits get a 0101 pattern, so the example shows that they do not matter.
                    PatItem::Char { value, care } => {
                        chars.push(((value & care | 0x55 & !care) as u8) & mask)
                    }
                    // Anything in between: two characters.
                    _ => chars.extend([0x2E & mask, 0x2E & mask]),
                }
            }
            (chars, false)
        }
        UartMatch::Error => (vec![0x4B & mask, 0x4B & mask], true),
        UartMatch::ModbusFrames { .. } => unreachable!("modbus_trigger"),
    };
    let frame = cfg.char_bits() * BIT as f64;
    let idle = 2 * BIT;
    let len = idle + (chars.len() as f64 * frame).ceil() as usize + 3 * idle;
    let mut w = Wave::new(
        cfg.baud * BIT as f64,
        len,
        if cfg.invert { 0 } else { 1 << cfg.rx },
    );
    let mut labels = Vec::new();
    for (i, &c) in chars.iter().enumerate() {
        let s0 = idle as f64 + i as f64 * frame;
        w.uart(cfg.rx, s0.round() as u64, &cfg, &[c]);
        labels.push((s0, s0 + frame, char_label(c)));
    }
    if broken {
        // The second character's stop bit is held at the start bit level: a framing error.
        let stop = idle as f64 + frame + (cfg.char_bits() - cfg.stop_bits as f64) * BIT as f64;
        w.set(
            cfg.rx,
            stop.round() as u64,
            (stop + BIT as f64).round() as u64,
            cfg.invert,
        );
    }
    let lanes = decode(&w, Box::new(Uart::new(cfg.clone())));
    let what = match m {
        UartMatch::Pattern { items, or_error } => format!(
            "{}{}",
            crate::setup::format_hex_pattern(items),
            if *or_error {
                " (or a character with an error)"
            } else {
                ""
            }
        ),
        UartMatch::Error => "a framing error (here: a stop bit that is low)".into(),
        UartMatch::ModbusFrames { .. } => unreachable!("modbus_trigger"),
    };
    let caption = vec![
        format!(
            "The trigger looks for {what}; the character before it does not match. Free bits are \
             shown as 0101, anything in between as '.'."
        ),
        format!(
            "T marks the trigger sample: the {} of the match.",
            at_text(at)
        ),
    ];
    let p = Preview {
        len,
        rows: vec![SigRow {
            label: format!("RX  {}", chan(names, cfg.rx)),
            levels: levels_of(&w, cfg.rx),
        }],
        marks: Vec::new(),
        bit_labels: labels,
        lanes: vec![("decoded".into(), lanes[0].clone())],
        caption,
        trigger: None,
    };
    (p, w)
}

/// A concrete value for a pattern value: the fixed bits, and 0101... for the free ones.
fn concrete(value: u32, care: u32, full: u32) -> u32 {
    (value & care | 0x5555_5555 & !care) & full
}

/// Modbus: a frame that does not begin with the pattern, then one that does (or, with only the
/// alternative, one with a bad CRC), each with its CRC.
fn modbus_trigger(
    cfg: &UartConfig,
    items: &[PatItem],
    or_bad: bool,
    at: TriggerAt,
    names: &[String],
) -> (Preview, Wave) {
    use bf_decode::modbus::crc16;
    const BIT: usize = 16;
    let cfg = UartConfig {
        telegrams: false,
        ..cfg.clone()
    };
    let as_chars = |bytes: &[u8]| -> Vec<Annotation> {
        bytes
            .iter()
            .map(|&b| Annotation {
                start: 0,
                end: 1,
                value: b as u32,
                lane: 0,
                kind: bf_decode::Kind::Data,
                bits: 8,
                flags: 0,
            })
            .collect()
    };
    let search = bf_decode::trigger::PatternSearch::new(items.to_vec());
    let begins = |body: &[u8]| {
        search
            .as_ref()
            .is_some_and(|p| p.found_at_start(&as_chars(body)))
    };
    let other: Vec<u8> = [
        vec![0x01, 0x03, 0x00, 0x00, 0x00, 0x01],
        vec![0x7F, 0x10, 0x12, 0x34, 0x00, 0x02],
    ]
    .into_iter()
    .find(|b| !begins(b))
    .unwrap_or_default();
    let mut body: Vec<u8> = Vec::new();
    for it in items {
        match *it {
            PatItem::Char { value, care } => body.push(concrete(value, care, 0xFF) as u8),
            _ => body.extend([0x00, 0x00]),
        }
    }
    let bad = items.is_empty();
    if bad {
        body = vec![0x11, 0x06, 0x00, 0x01, 0x00, 0x03];
    }
    while body.len() < 2 {
        body.push(0x03);
    }
    let with_crc = |b: &[u8], broken: bool| {
        let mut f = b.to_vec();
        let crc = crc16(b) ^ if broken { 0x00FF } else { 0 };
        f.extend_from_slice(&crc.to_le_bytes());
        f
    };
    let frames = [with_crc(&other, false), with_crc(&body, bad)];
    let ch = cfg.char_bits() * BIT as f64;
    let (lead, gap) = (2.0 * ch, 5.0 * ch);
    let total: f64 = frames.iter().map(|f| f.len() as f64 * ch).sum();
    let len = (lead + total + 2.0 * gap) as usize;
    let mut w = Wave::new(
        cfg.baud * BIT as f64,
        len,
        if cfg.invert { 0 } else { 1 << cfg.rx },
    );
    let mut labels = Vec::new();
    let mut t = lead;
    for f in &frames {
        for &b in f {
            w.uart(cfg.rx, t.round() as u64, &cfg, &[b]);
            labels.push((t, t + ch, format!("{b:02X}")));
            t += ch;
        }
        t += gap;
    }
    let lanes = decode(&w, Box::new(Uart::new(cfg.clone())));
    let what = if bad {
        "a frame with a bad CRC".to_string()
    } else {
        format!(
            "a Modbus frame beginning with {}{}",
            crate::setup::format_hex_pattern(items),
            if or_bad { " (or a bad frame)" } else { "" }
        )
    };
    let caption = vec![
        format!(
            "The trigger looks for {what}; the frame before it does not match. Free bits are \
             shown as 0101, anything in between as 00."
        ),
        format!(
            "T marks the trigger sample: the {} of the frame.",
            at_text(at)
        ),
    ];
    let p = Preview {
        len,
        rows: vec![SigRow {
            label: format!("RX  {}", chan(names, cfg.rx)),
            levels: levels_of(&w, cfg.rx),
        }],
        marks: Vec::new(),
        bit_labels: labels,
        lanes: vec![
            ("bytes".into(), lanes[0].clone()),
            ("frames".into(), lanes.get(1).cloned().unwrap_or_default()),
        ],
        caption,
        trigger: None,
    };
    (p, w)
}

fn spi_trigger(cfg: &bf_decode::spi::SpiConfig, m: &SpiMatch, names: &[String]) -> (Preview, Wave) {
    const HALF: u64 = 8;
    let bits = cfg.bits as u64;
    let full = if bits >= 32 {
        u32::MAX
    } else {
        (1u32 << bits) - 1
    };
    let filler = 0x3A5C_69D1 & full;
    // A word the pattern does not start with, then the pattern's words (gaps: two filler words).
    let first_fits = |w: u32| match m.items.first() {
        Some(PatItem::Char { value, care }) => (w ^ value) & care == 0,
        _ => false,
    };
    let other = [0, full, filler, 0x5555_5555 & full]
        .into_iter()
        .find(|&w| !first_fits(w))
        .unwrap_or(0);
    let mut words = vec![other];
    for it in &m.items {
        match *it {
            PatItem::Char { value, care } => words.push(concrete(value, care, full)),
            _ => words.extend([filler, filler]),
        }
    }
    // Only the alternative: a word, then one cut short.
    let cut = m.items.is_empty();
    if cut {
        words.push(filler);
    }
    let other_line: Vec<u32> = words.iter().map(|_| 0x2468_ACE1 & full).collect();
    let (mosi, miso) = match m.lane {
        SpiLane::Mosi => (words.clone(), other_line),
        SpiLane::Miso => (other_line, words.clone()),
    };
    let lead = 2 * HALF;
    let len = (lead + 2 * HALF * bits * words.len() as u64 + 10 * HALF + lead) as usize;
    let init = (cfg.cpol as u32) << cfg.sck | cfg.cs.map_or(0, |c| (cfg.cs_active_low as u32) << c);
    let mut w = Wave::new(1e6, len, init);
    w.spi(
        cfg,
        lead,
        HALF,
        if cfg.mosi.is_some() { &mosi } else { &[] },
        if cfg.miso.is_some() { &miso } else { &[] },
    );
    if cut {
        // CS goes away in the middle of the last word.
        if let Some(cs) = cfg.cs {
            let at = lead + HALF + 2 * HALF * bits * (words.len() as u64 - 1) + bits * HALF;
            w.set(cs, at, len as u64, cfg.cs_active_low);
        }
    }
    let mut rows = Vec::new();
    let mut add = |label: &str, ch: u8| {
        rows.push(SigRow {
            label: format!("{label:<4} {}", chan(names, ch)),
            levels: levels_of(&w, ch),
        })
    };
    if let Some(c) = cfg.cs {
        add(if cfg.cs_active_low { "CS#" } else { "CS" }, c);
    }
    add("SCK", cfg.sck);
    if let Some(c) = cfg.mosi {
        add("MOSI", c);
    }
    if let Some(c) = cfg.miso {
        add("MISO", c);
    }
    let digits = (bits as usize).div_ceil(4);
    let labels = words
        .iter()
        .enumerate()
        .map(|(i, v)| {
            let s0 = (lead + HALF + i as u64 * bits * 2 * HALF) as f64;
            (s0, s0 + (bits * 2 * HALF) as f64, format!("{v:0digits$X}"))
        })
        .collect();
    let decoded = decode(&w, Box::new(Spi::new(cfg.clone())));
    let mut lanes = Vec::new();
    if cfg.mosi.is_some() {
        lanes.push(("MOSI word".to_string(), decoded[0].clone()));
    }
    if cfg.miso.is_some() {
        lanes.push(("MISO word".to_string(), decoded[1].clone()));
    }
    let line = match m.lane {
        SpiLane::Mosi => "MOSI",
        SpiLane::Miso => "MISO",
    };
    let what = match (m.items.is_empty(), m.or_incomplete) {
        (true, _) => "a word cut short by the end of the transaction".to_string(),
        (false, alt) => format!(
            "{} on {line}{}",
            crate::setup::format_word_pattern(&m.items, bits as u32),
            if alt { " (or an incomplete word)" } else { "" }
        ),
    };
    let caption = vec![
        format!(
            "The trigger looks for {what}; the word before it does not match. Free bits are \
             shown as 0101."
        ),
        "T marks the trigger sample.".into(),
    ];
    let p = Preview {
        len,
        rows,
        marks: Vec::new(),
        bit_labels: labels,
        lanes,
        caption,
        trigger: None,
    };
    (p, w)
}

/// I2C traffic for an example, built item by item: START, address and STOP are added where the
/// pattern leaves them out, so the bus sequence is always valid.
struct I2cExample {
    w: Wave,
    scl: u8,
    sda: u8,
    t: u64,
    in_txn: bool,
    /// The last thing sent was a START (an address may follow directly).
    after_start: bool,
    labels: Vec<(f64, f64, String)>,
}

impl I2cExample {
    const Q: u64 = 8;

    fn start(&mut self) {
        let t0 = self.t;
        self.t = self
            .w
            .i2c_start(self.scl, self.sda, self.t, Self::Q, self.in_txn);
        let label = if self.in_txn { "Sr" } else { "S" };
        self.labels.push((t0 as f64, self.t as f64, label.into()));
        self.in_txn = true;
        self.after_start = true;
    }

    fn byte(&mut self, b: u8, nak: bool, label: String) {
        let t0 = self.t;
        self.t = self.w.i2c_byte(self.scl, self.sda, self.t, Self::Q, b, nak);
        let label = if nak { format!("{label} NAK") } else { label };
        self.labels.push((t0 as f64, self.t as f64, label));
        self.after_start = false;
    }

    fn address(&mut self, addr: u8, read: bool, nak: bool) {
        if !self.after_start {
            self.start();
        }
        let dir = if read { "R" } else { "W" };
        self.byte(addr << 1 | read as u8, nak, format!("@{addr:02X} {dir}"));
    }

    /// Inside a transaction, after an address.
    fn ensure_txn(&mut self) {
        if !self.in_txn {
            self.start();
            self.address(0x50, false, false);
        } else if self.after_start {
            self.address(0x50, false, false);
        }
    }

    fn stop(&mut self) {
        let t0 = self.t;
        self.t = self.w.i2c_stop(self.scl, self.sda, self.t, Self::Q);
        self.labels.push((t0 as f64, self.t as f64, "P".into()));
        self.in_txn = false;
        self.after_start = false;
    }
}

fn i2c_trigger(cfg: &bf_decode::i2c::I2cConfig, m: &I2cMatch, names: &[String]) -> (Preview, Wave) {
    let q = I2cExample::Q;
    let lead = 3 * q;
    let (scl, sda) = (cfg.scl, cfg.sda);
    let room = lead + (m.items.len() as u64 * 3 + 6) * 40 * q;
    let mut e = I2cExample {
        w: Wave::new(1e6, room as usize, (1 << scl) | (1 << sda)),
        scl,
        sda,
        t: lead,
        in_txn: false,
        after_start: false,
        labels: Vec::new(),
    };
    for it in &m.items {
        match *it {
            PatItem::Start { repeated } => {
                if repeated && !e.in_txn {
                    e.ensure_txn();
                }
                e.start();
            }
            PatItem::Stop => {
                e.ensure_txn();
                e.stop();
            }
            PatItem::Address {
                value,
                care,
                read,
                nak,
            } => {
                let addr = concrete(value, care, 0x7F) as u8;
                e.address(addr, read.unwrap_or(false), nak);
            }
            PatItem::Char { value, care } | PatItem::CharNak { value, care } => {
                e.ensure_txn();
                let b = concrete(value, care, 0xFF) as u8;
                e.byte(b, matches!(it, PatItem::CharNak { .. }), format!("{b:02X}"));
            }
            PatItem::Gap => {
                e.ensure_txn();
                e.byte(0x2E, false, "2E".into());
            }
        }
    }
    // Only the alternative: a byte that is not acknowledged.
    if m.items.is_empty() {
        e.ensure_txn();
        e.byte(0x3C, true, "3C".into());
    }
    if e.in_txn {
        e.stop();
    }
    let len = (e.t + lead) as usize;
    let mut w = e.w;
    w.samples.truncate(len);
    let rows = vec![
        SigRow {
            label: format!("SCL {}", chan(names, scl)),
            levels: levels_of(&w, scl),
        },
        SigRow {
            label: format!("SDA {}", chan(names, sda)),
            levels: levels_of(&w, sda),
        },
    ];
    let decoded = decode(&w, Box::new(I2c::new(cfg.clone())));
    let what = match (m.items.is_empty(), m.or_nak) {
        (true, _) => "a byte that is not acknowledged".to_string(),
        (false, nak) => format!(
            "{}{}",
            crate::setup::format_i2c_pattern(&m.items),
            if nak { " (or any NAK)" } else { "" }
        ),
    };
    let p = Preview {
        len,
        rows,
        marks: Vec::new(),
        bit_labels: e.labels,
        lanes: vec![("decoded".into(), decoded[0].clone())],
        caption: vec![
            format!(
                "The trigger looks for {what}. START, address and STOP are added where the \
                 pattern leaves them out; free bits are shown as 0101."
            ),
            "T marks the trigger sample.".into(),
        ],
        trigger: None,
    };
    (p, w)
}

/// Draw `p` into the available width.
pub fn draw(ui: &mut egui::Ui, p: &Preview, fmt: Format) {
    const GUTTER: f32 = 150.0;
    const ROW: f32 = 38.0;
    const LANE: f32 = 32.0;
    const TOP: f32 = 20.0;
    let height = TOP + p.rows.len() as f32 * ROW + p.lanes.len() as f32 * LANE + 6.0;
    let (rect, _) = ui.allocate_exact_size(vec2(ui.available_width(), height), Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 0.0, theme::PLOT_BG);
    painter.rect_stroke(rect, 0.0, Stroke::new(1.0_f32, theme::BORDER));
    let plot = Rect::from_min_max(
        pos2(rect.left() + GUTTER, rect.top()),
        pos2(rect.right() - 8.0, rect.bottom()),
    );
    let x_of = |s: f64| plot.left() + (s / p.len as f64) as f32 * plot.width();
    let font = FontId::monospace(10.0);

    // Bit names across the top, with faint separators.
    for (s0, s1, text) in &p.bit_labels {
        let (x0, x1) = (x_of(*s0), x_of(*s1));
        painter.line_segment(
            [
                pos2(x0, rect.top() + TOP - 4.0),
                pos2(x0, rect.bottom() - 4.0),
            ],
            Stroke::new(1.0_f32, theme::GRID),
        );
        if x1 - x0 > 9.0 * text.len() as f32 * 0.7 {
            painter.text(
                pos2((x0 + x1) / 2.0, rect.top() + 9.0),
                Align2::CENTER_CENTER,
                text,
                font.clone(),
                theme::TEXT_DIM,
            );
        }
    }
    if let Some((_, s1, _)) = p.bit_labels.last() {
        let x = x_of(*s1);
        painter.line_segment(
            [
                pos2(x, rect.top() + TOP - 4.0),
                pos2(x, rect.bottom() - 4.0),
            ],
            Stroke::new(1.0_f32, theme::GRID),
        );
    }

    if let Some(t) = p.trigger {
        // The same marker as in the waveform: an orange line with a "T" tab.
        let x = x_of(t);
        painter.line_segment(
            [pos2(x, rect.top() + 2.0), pos2(x, rect.bottom() - 2.0)],
            Stroke::new(1.5_f32, theme::DEC_WARN),
        );
        let tab = Rect::from_min_size(pos2(x - 7.0, rect.top() + 2.0), vec2(14.0, 13.0));
        painter.rect_filled(tab, 0.0, theme::DEC_WARN);
        painter.text(
            tab.center(),
            Align2::CENTER_CENTER,
            "T",
            FontId::monospace(10.0),
            Color32::BLACK,
        );
    }

    let row_top = |i: usize| rect.top() + TOP + i as f32 * ROW;
    let y_of = |i: usize, level: bool| {
        if level {
            row_top(i) + 9.0
        } else {
            row_top(i) + ROW - 9.0
        }
    };
    for (i, row) in p.rows.iter().enumerate() {
        painter.text(
            pos2(rect.left() + 8.0, row_top(i) + ROW / 2.0),
            Align2::LEFT_CENTER,
            &row.label,
            FontId::monospace(11.0),
            theme::TEXT,
        );
        let Some(&first) = row.levels.first() else {
            continue;
        };
        let mut pts = vec![pos2(x_of(0.0), y_of(i, first))];
        let mut prev = first;
        for (s, &l) in row.levels.iter().enumerate().skip(1) {
            if l != prev {
                pts.push(pos2(x_of(s as f64), y_of(i, prev)));
                pts.push(pos2(x_of(s as f64), y_of(i, l)));
                prev = l;
            }
        }
        pts.push(pos2(x_of(p.len as f64), y_of(i, prev)));
        painter.add(Shape::line(pts, Stroke::new(1.5_f32, theme::TRACE)));
    }
    for &(row, s) in &p.marks {
        let level = p.rows[row].levels.get(s as usize).copied().unwrap_or(false);
        painter.circle_filled(pos2(x_of(s), y_of(row, level)), 3.5, theme::SNAP);
    }
    let view = View {
        start: 0.0,
        spp: p.len as f64 / plot.width() as f64,
    };
    for (k, (name, anns)) in p.lanes.iter().enumerate() {
        let top = row_top(p.rows.len()) + k as f32 * LANE;
        let lane_rect = Rect::from_min_size(pos2(plot.left(), top), vec2(plot.width(), LANE));
        painter.text(
            pos2(rect.left() + 8.0, top + LANE / 2.0),
            Align2::LEFT_CENTER,
            name,
            FontId::monospace(11.0),
            theme::DEC_ADDR,
        );
        draw_annotations(
            &painter.with_clip_rect(lane_rect),
            anns,
            lane_rect,
            &view,
            fmt,
            Color32::from_rgb(0xe2, 0xe6, 0xe9),
            None,
            None,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bf_decode::spi::SpiConfig;
    use bf_decode::uart::{Parity, UartConfig};
    use bf_decode::{flag, Kind};

    fn words(p: &Preview, lane: usize) -> Vec<u32> {
        p.lanes[lane]
            .1
            .iter()
            .filter(|a| a.kind == Kind::Data)
            .map(|a| a.value)
            .collect()
    }

    fn uart_cfg() -> UartConfig {
        UartConfig {
            parity: Parity::Even,
            ..Default::default()
        }
    }

    #[test]
    fn trigger_example_marks_the_start_or_end_of_the_sequence() {
        let t = ProtocolTrigger::Uart(uart_cfg(), UartMatch::bytes(b"OK"));
        for at in [TriggerAt::Start, TriggerAt::End] {
            let p = build_trigger(&t, at, &[]).unwrap();
            // One other character, then "OK", decoded without errors.
            assert_eq!(words(&p, 0), vec![0x2E, b'O' as u32, b'K' as u32]);
            let chars = &p.lanes[0].1;
            let want = match at {
                TriggerAt::Start => chars[1].start,
                TriggerAt::End => chars[2].end,
            };
            assert_eq!(p.trigger, Some(want as f64), "{at:?}");
        }
    }

    #[test]
    fn trigger_example_for_a_pattern_fires_where_the_pattern_ends() {
        let items = crate::setup::parse_hex_pattern("4X * 0A").unwrap();
        let t = ProtocolTrigger::Uart(uart_cfg(), UartMatch::pattern(items, true));
        let p = build_trigger(&t, TriggerAt::End, &[]).unwrap();
        // Other character, 0x45 (4X with the free nibble 0101), two in between, 0x0A.
        assert_eq!(words(&p, 0), vec![0x2E, 0x45, 0x2E, 0x2E, 0x0A]);
        assert_eq!(p.trigger, Some(p.lanes[0].1[4].end as f64));
        assert!(p.caption[0].contains("4X * 0A") && p.caption[0].contains("error"));
    }

    #[test]
    fn modbus_trigger_example_fires_at_the_matching_frame() {
        let cfg = UartConfig {
            baud: 19_200.0,
            parity: Parity::Even,
            protocol: bf_decode::uart::Protocol::ModbusRtu,
            ..Default::default()
        };
        let items = crate::setup::parse_hex_pattern("65 06 * XX").unwrap();
        for at in [TriggerAt::Start, TriggerAt::End] {
            let t = ProtocolTrigger::Uart(
                cfg.clone(),
                UartMatch::ModbusFrames {
                    items: items.clone(),
                    or_bad: false,
                },
            );
            let p = build_trigger(&t, at, &[]).unwrap();
            let frames = &p.lanes[1].1;
            assert_eq!(frames.len(), 2, "{frames:?}");
            assert!(frames.iter().all(|f| !f.is_error()), "both CRCs are right");
            let want = match at {
                TriggerAt::Start => frames[1].start,
                TriggerAt::End => frames[1].end,
            };
            assert_eq!(p.trigger, Some(want as f64), "{at:?}");
        }
        // Only bad frames: the second frame has a broken CRC and is the one that fires.
        let t = ProtocolTrigger::Uart(
            cfg,
            UartMatch::ModbusFrames {
                items: vec![],
                or_bad: true,
            },
        );
        let p = build_trigger(&t, TriggerAt::End, &[]).unwrap();
        let frames = &p.lanes[1].1;
        assert!(frames[1].is_error());
        assert_eq!(p.trigger, Some(frames[1].end as f64));
    }

    #[test]
    fn trigger_examples_for_spi_and_i2c_patterns() {
        // SPI: 12, anything, AX on MOSI; the example ends with the pattern and the trigger fires.
        let spi = SpiConfig::default();
        let items = crate::setup::parse_word_pattern("12 * AX", 8).unwrap();
        let t = ProtocolTrigger::Spi(spi, SpiMatch::pattern(SpiLane::Mosi, items, false));
        let p = build_trigger(&t, TriggerAt::End, &[]).unwrap();
        let w = words(&p, 0);
        assert_eq!(&w[1..], &[0x12, 0xD1, 0xD1, 0xA5]);
        assert_eq!(p.trigger, Some(p.lanes[0].1.last().unwrap().end as f64));
        // I2C: a register write whose value is not acknowledged; START and STOP are added.
        let i2c = bf_decode::i2c::I2cConfig::default();
        for text in ["@50W 01 XX!", "S @50W 01 XX! P", "Sr @48R", "3C *  P"] {
            let items = crate::setup::parse_i2c_pattern(text).unwrap();
            let t = ProtocolTrigger::I2c(i2c.clone(), I2cMatch::pattern(items, false));
            let p = build_trigger(&t, TriggerAt::End, &[]).unwrap();
            assert!(p.trigger.is_some(), "{text}: {:?}", p.caption);
            assert_eq!(p.bit_labels.first().unwrap().2, "S", "{text}");
            assert_eq!(p.bit_labels.last().unwrap().2, "P", "{text}");
        }
        // Only the alternative: a NAKed byte.
        let p = build_trigger(
            &ProtocolTrigger::I2c(i2c, I2cMatch::nak()),
            TriggerAt::End,
            &[],
        )
        .unwrap();
        assert!(p.trigger.is_some());
    }

    #[test]
    fn trigger_example_for_uart_errors_spi_words_and_i2c() {
        let t = ProtocolTrigger::Uart(uart_cfg(), UartMatch::Error);
        let p = build_trigger(&t, TriggerAt::End, &[]).unwrap();
        let chars = &p.lanes[0].1;
        assert!(chars[0].flags & flag::FRAMING == 0 && chars[1].flags & flag::FRAMING != 0);
        assert_eq!(p.trigger, Some(chars[1].end as f64));

        let spi = SpiConfig::default();
        let t = ProtocolTrigger::Spi(spi.clone(), SpiMatch::word(SpiLane::Mosi, 0xA5, 0xF0));
        let p = build_trigger(&t, TriggerAt::End, &[]).unwrap();
        let w = words(&p, 0);
        assert_eq!(w.len(), 2);
        assert_ne!(w[0] & 0xF0, 0xA0);
        assert_eq!(w[1] & 0xF0, 0xA0);
        assert_eq!(p.trigger, Some(p.lanes[0].1.last().unwrap().end as f64));

        let i2c = bf_decode::i2c::I2cConfig::default();
        for m in [
            I2cMatch::start(),
            I2cMatch::stop(),
            I2cMatch::nak(),
            I2cMatch::data(0x7E),
            I2cMatch::address(0x23, Some(true)),
        ] {
            let p = build_trigger(
                &ProtocolTrigger::I2c(i2c.clone(), m.clone()),
                TriggerAt::End,
                &[],
            )
            .unwrap();
            assert!(p.trigger.is_some(), "{m:?}");
        }
        // Unusable settings: no example.
        let same = bf_decode::i2c::I2cConfig { scl: 1, sda: 1 };
        assert!(build_trigger(
            &ProtocolTrigger::I2c(same, I2cMatch::start()),
            TriggerAt::End,
            &[]
        )
        .is_none());
    }

    #[test]
    fn uart_example_decodes_to_the_example_byte_for_every_format() {
        for bits in 5..=9u8 {
            for parity in Parity::ALL {
                for stop in [1.0f32, 1.5, 2.0] {
                    for (lsb, invert) in
                        [(true, false), (false, false), (true, true), (false, true)]
                    {
                        let cfg = UartConfig {
                            data_bits: bits,
                            parity,
                            stop_bits: stop,
                            lsb_first: lsb,
                            invert,
                            ..Default::default()
                        };
                        let p = build(&Config::Uart(cfg.clone()), &[]);
                        assert_eq!(
                            words(&p, 0),
                            vec![UART_EXAMPLE & ((1 << bits) - 1)],
                            "{cfg:?}"
                        );
                        assert!(
                            p.lanes[0].1.iter().all(|a| a.flags & flag::ERROR == 0),
                            "{cfg:?}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn uart_example_marks_every_bit_in_its_middle() {
        let p = build(&Config::Uart(UartConfig::default()), &[]);
        // start + 8 data + stop
        assert_eq!(p.marks.len(), 10);
        assert_eq!(p.bit_labels.len(), 10);
        assert_eq!(p.bit_labels[1].2, "D0");
        let msb = build(
            &Config::Uart(UartConfig {
                lsb_first: false,
                ..Default::default()
            }),
            &[],
        );
        assert_eq!(msb.bit_labels[1].2, "D7");
        // The line level at each dot is what the decoder reads: reassemble the byte from the dots.
        let lv = &p.rows[0].levels;
        let byte: u32 = (1..=8)
            .map(|j| (lv[p.marks[j].1 as usize] as u32) << (j - 1))
            .sum();
        assert_eq!(byte, UART_EXAMPLE);
    }

    #[test]
    fn spi_example_decodes_in_all_modes_and_word_sizes() {
        for (cpol, cpha) in [(false, false), (false, true), (true, false), (true, true)] {
            for (bits, msb) in [(8u8, true), (8, false), (12, true), (16, false), (32, true)] {
                let cfg = SpiConfig {
                    cpol,
                    cpha,
                    msb_first: msb,
                    bits,
                    miso: Some(3),
                    mosi: Some(2),
                    ..Default::default()
                };
                let p = build(&Config::Spi(cfg.clone()), &[]);
                let top = |v: u32| if bits >= 32 { v } else { v >> (32 - bits) };
                assert_eq!(words(&p, 0), vec![top(0x96C5_A53B)], "{cfg:?}");
                assert_eq!(words(&p, 1), vec![top(0x3A5C_69D1)], "{cfg:?}");
                // One dot per word bit on the clock.
                let sck_dots = p.marks.iter().filter(|m| m.0 == 1).count();
                assert_eq!(sck_dots, bits as usize, "{cfg:?}");
            }
        }
    }

    #[test]
    fn spi_example_rows_follow_the_wiring() {
        let cfg = SpiConfig {
            cs: None,
            mosi: Some(2),
            miso: None,
            ..Default::default()
        };
        let p = build(&Config::Spi(cfg), &[]);
        assert_eq!(p.rows.len(), 2); // SCK, MOSI
        assert_eq!(p.lanes.len(), 1);
        let cfg = SpiConfig {
            mosi: None,
            miso: None,
            ..Default::default()
        };
        let p = build(&Config::Spi(cfg), &[]);
        assert_eq!(p.rows.len(), 2); // CS, SCK
        assert!(p.lanes.is_empty());
        assert!(p.caption.iter().any(|c| c.contains("Select MOSI")));
    }

    #[test]
    fn i2c_example_shows_address_data_and_acks() {
        let p = build(
            &Config::I2c(bf_decode::i2c::I2cConfig { scl: 4, sda: 5 }),
            &[],
        );
        let a = &p.lanes[0].1;
        let kinds: Vec<(Kind, u32)> = a.iter().map(|a| (a.kind, a.value)).collect();
        assert_eq!(
            kinds,
            vec![
                (Kind::Start, 0),
                (Kind::Address, 0x50),
                (Kind::Data, 0x3C),
                (Kind::Stop, 0)
            ]
        );
        assert!(a.iter().all(|a| a.flags & flag::NAK == 0));
        // 16 data bits of the two bytes get a dot, plus the two ACK clocks.
        assert_eq!(p.marks.len(), 18);
        assert_eq!(p.bit_labels.iter().filter(|l| l.2 == "ACK").count(), 2);
    }

    #[test]
    fn row_labels_use_the_channel_names() {
        let names = vec!["a".to_string(), "b".to_string(), "clock".to_string()];
        let p = build(
            &Config::Spi(SpiConfig {
                sck: 2,
                ..Default::default()
            }),
            &names,
        );
        assert!(p.rows.iter().any(|r| r.label.contains("CH2 clock")));
    }
}
