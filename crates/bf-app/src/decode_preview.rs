//! Example traces for the decoder setup: a synthetic waveform generated for the *current settings* and
//! decoded by the real decoder, so the picture shows what the decoder expects and what it makes of it.

use crate::decode_ui::Config;
use crate::theme;
use crate::view::{format_duration, View};
use crate::waveform::draw_annotations;
use bf_core::{CaptureStore, ChannelCount, StoreConfig};
use bf_decode::engine::decode_all;
use bf_decode::i2c::I2c;
use bf_decode::spi::Spi;
use bf_decode::synth::Wave;
use bf_decode::uart::Uart;
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
    }
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
