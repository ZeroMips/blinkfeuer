//! The Measurements window: edges, frequency, period, duty cycle and pulse widths of every shown
//! channel, over the visible window, between the markers, or over the whole capture.

use crate::markers::Markers;
use crate::rows::Layout;
use crate::theme;
use crate::view::{format_duration, format_hz, format_split, View};
use bf_core::measure::{measure, Measurement, Stat};
use bf_core::CaptureStore;
use egui::RichText;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Scope {
    #[default]
    Visible,
    Markers,
    All,
}

impl Scope {
    fn label(self) -> &'static str {
        match self {
            Scope::Visible => "Visible window",
            Scope::Markers => "Between M1 and M2",
            Scope::All => "Whole capture",
        }
    }
}

struct Cache {
    key: (Scope, u64, u64, u64, Vec<bool>),
    rows: Vec<(usize, Option<Measurement>)>,
}

#[derive(Default)]
pub struct MeasureState {
    pub scope: Scope,
    cache: Option<Cache>,
    computed: Option<Instant>,
}

/// The range to measure, in samples.
fn range_of(scope: Scope, store: &CaptureStore, view: &View, plot_width: f64, markers: &Markers) -> Option<(u64, u64)> {
    let all = store.range();
    match scope {
        Scope::All => Some((all.start, all.end)),
        Scope::Visible => {
            let a = view.start.max(all.start as f64).floor() as u64;
            let b = (view.end(plot_width).min(all.end as f64)).ceil() as u64;
            (b > a).then_some((a, b))
        }
        Scope::Markers => {
            let (a, b) = (markers.m1?, markers.m2?);
            let (lo, hi) = (a.min(b).max(all.start as f64), a.max(b).min(all.end as f64));
            (hi > lo).then_some((lo.floor() as u64, hi.ceil() as u64))
        }
    }
}

fn stat_cells(s: &Stat, rate: f64) -> [String; 3] {
    if s.count == 0 {
        return ["-".into(), "-".into(), "-".into()];
    }
    let t = |samples: f64| format_duration(samples / rate);
    [t(s.min as f64), t(s.mean().unwrap_or(0.0)), t(s.max as f64)]
}

/// The table as CSV (seconds and hertz as plain numbers).
fn csv(rows: &[(String, Option<Measurement>)], rate: f64) -> String {
    let mut out = String::from("channel,edges,rising,falling,frequency_hz,period_s,duty,high_min_s,high_mean_s,high_max_s,low_min_s,low_mean_s,low_max_s\n");
    for (name, m) in rows {
        let name = if name.contains([',', '"']) { format!("\"{}\"", name.replace('"', "\"\"")) } else { name.clone() };
        let Some(m) = m else { continue };
        let num = |v: Option<f64>| v.map_or(String::new(), |v| format!("{v:.9e}"));
        let d = m.detail;
        let period = m.period_samples().map(|p| p / rate);
        let stat = |s: Option<&Stat>| -> [String; 3] {
            match s.filter(|s| s.count > 0) {
                Some(s) => [num(Some(s.min as f64 / rate)), num(s.mean().map(|v| v / rate)), num(Some(s.max as f64 / rate))],
                None => [String::new(), String::new(), String::new()],
            }
        };
        let (h, l) = (stat(d.as_ref().map(|d| &d.high)), stat(d.as_ref().map(|d| &d.low)));
        out.push_str(&format!(
            "{name},{},{},{},{},{},{},{},{},{},{},{},{}\n",
            m.edges,
            d.map_or(String::new(), |d| d.rising.to_string()),
            d.map_or(String::new(), |d| d.falling.to_string()),
            num(period.map(|p| 1.0 / p)),
            num(period),
            num(m.duty()),
            h[0],
            h[1],
            h[2],
            l[0],
            l[1],
            l[2],
        ));
    }
    out
}

#[allow(clippy::too_many_arguments)]
pub fn window(
    ctx: &egui::Context,
    open: &mut bool,
    st: &mut MeasureState,
    store: &CaptureStore,
    layout: &Layout,
    view: &View,
    plot_width: f64,
    markers: &Markers,
    origin: f64,
    running: bool,
) {
    let mut is_open = *open;
    egui::Window::new("Measurements").open(&mut is_open).default_width(900.0).resizable(true).show(ctx, |ui| {
        let rate = store.sample_rate();
        ui.horizontal(|ui| {
            for s in [Scope::Visible, Scope::Markers, Scope::All] {
                let enabled = s != Scope::Markers || (markers.m1.is_some() && markers.m2.is_some());
                if ui.add_enabled(enabled, egui::SelectableLabel::new(st.scope == s, s.label())).clicked() {
                    st.scope = s;
                }
            }
        });
        let Some((a, b)) = range_of(st.scope, store, view, plot_width, markers) else {
            ui.label(RichText::new("Nothing to measure: no data in that range.").color(theme::TEXT_DIM));
            return;
        };
        ui.label(
            RichText::new(format!(
                "{} .. {}   ({}, {} Sa)",
                format_split((a as f64 - origin) / rate, 1.0 / rate),
                format_split((b as f64 - origin) / rate, 1.0 / rate),
                format_duration((b - a) as f64 / rate),
                b - a
            ))
            .color(theme::TEXT_DIM),
        );
        // Recompute when the range, the data or the shown channels change (while capturing at most a few times a second).
        let key = (st.scope, a, b, store.generation(), layout.visible.clone());
        let stale = st.cache.as_ref().is_none_or(|c| c.key != key);
        let throttled = running && st.computed.is_some_and(|t| t.elapsed() < Duration::from_millis(300));
        if stale && !throttled {
            let rows = (0..layout.names.len().min(store.channels() as usize))
                .filter(|&c| layout.visible[c])
                .map(|c| (c, measure(store, c as u8, a..b)))
                .collect();
            st.cache = Some(Cache { key, rows });
            st.computed = Some(Instant::now());
        }
        if running {
            ui.ctx().request_repaint_after(Duration::from_millis(300));
        }
        let Some(cache) = &st.cache else { return };
        let named: Vec<(String, Option<Measurement>)> = cache.rows.iter().map(|(c, m)| (layout.names[*c].clone(), *m)).collect();

        egui::ScrollArea::both().show(ui, |ui| {
            egui::Grid::new("measure_grid").num_columns(12).spacing([14.0, 3.0]).striped(true).show(ui, |ui| {
                for h in ["Channel", "Edges", "Frequency", "Period", "Duty", "High min", "mean", "max", "Low min", "mean", "max"] {
                    ui.label(RichText::new(h).color(theme::TEXT_DIM));
                }
                ui.end_row();
                for (name, m) in &named {
                    ui.label(name);
                    let Some(m) = m else {
                        ui.label("-");
                        ui.end_row();
                        continue;
                    };
                    match &m.detail {
                        Some(d) => {
                            ui.label(format!("{} ({}↑ {}↓)", m.edges, d.rising, d.falling));
                            let p = m.period_samples().map(|p| p / rate);
                            ui.label(p.map_or("-".into(), |p| format_hz(1.0 / p)));
                            ui.label(p.map_or("-".into(), format_duration));
                            ui.label(m.duty().map_or("-".into(), |d| format!("{:.1} %", d * 100.0)));
                            for c in stat_cells(&d.high, rate).into_iter().chain(stat_cells(&d.low, rate)) {
                                ui.label(c);
                            }
                        }
                        None => {
                            // Too many edges to walk: the count and the rate they imply.
                            ui.label(format!("{}", m.edges));
                            let f = m.edges as f64 / 2.0 / (m.samples() as f64 / rate);
                            ui.label(RichText::new(format!("~{}", format_hz(f))).color(theme::DEC_WARN));
                            ui.label(RichText::new("too many edges: narrow the range for exact values").color(theme::TEXT_DIM));
                        }
                    }
                    ui.end_row();
                }
            });
        });
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Copy as CSV").on_hover_text("Seconds and hertz as plain numbers").clicked() {
                ui.ctx().copy_text(csv(&named, rate));
            }
            ui.label(
                RichText::new("Pulse widths count only pulses that start and end inside the range; the period is between rising edges.")
                    .color(theme::TEXT_DIM),
            );
        });
    });
    *open = is_open;
}

#[cfg(test)]
mod tests {
    use super::*;
    use bf_core::{ChannelCount, StoreConfig};

    fn store() -> CaptureStore {
        let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(4).unwrap(), 1e6));
        // CH0: 25 high / 75 low, CH1 constant high.
        let words: Vec<u32> = (0..1000).map(|i| (i % 100 < 25) as u32 | 2).collect();
        s.push_samples(&words);
        s
    }

    #[test]
    fn scopes_pick_the_range() {
        let s = store();
        let v = View { start: 100.0, spp: 1.0 };
        let m = Markers { m1: Some(300.5), m2: Some(200.0) };
        assert_eq!(range_of(Scope::All, &s, &v, 400.0, &m), Some((0, 1000)));
        assert_eq!(range_of(Scope::Visible, &s, &v, 400.0, &m), Some((100, 500)));
        assert_eq!(range_of(Scope::Markers, &s, &v, 400.0, &m), Some((200, 301)), "either order, rounded outwards");
        assert_eq!(range_of(Scope::Markers, &s, &v, 400.0, &Markers::default()), None);
        // A window scrolled past the data has nothing to measure.
        assert_eq!(range_of(Scope::Visible, &s, &View { start: 5000.0, spp: 1.0 }, 400.0, &m), None);
        // The window is clipped to the capture.
        assert_eq!(range_of(Scope::Visible, &s, &View { start: -50.0, spp: 1.0 }, 100.0, &m), Some((0, 50)));
    }

    #[test]
    fn csv_has_plain_numbers_and_quotes_names() {
        let s = store();
        let m0 = measure(&s, 0, 0..1000);
        let rows = vec![("A,B".to_string(), m0), ("C".to_string(), measure(&s, 1, 0..1000))];
        let text = csv(&rows, 1e6);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 3);
        assert!(lines[0].starts_with("channel,edges,"));
        assert!(lines[1].starts_with("\"A,B\",19"), "{}", lines[1]);
        // 25 % duty, 100 µs period = 10 kHz.
        let cols: Vec<&str> = lines[1].rsplitn(13, ',').collect();
        let fields: Vec<&str> = cols.into_iter().rev().collect();
        assert!(fields[4].starts_with("1.0000"), "frequency {}", fields[4]);
        assert!(fields[6].starts_with("2.5000"), "duty {}", fields[6]);
        // The constant channel has an edge count and no frequency.
        assert!(lines[2].starts_with("C,0,0,0,,,"), "{}", lines[2]);
    }
}
