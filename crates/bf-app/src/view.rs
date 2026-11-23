//! Pure view math: sample/pixel mapping, zoom and pan, time ruler ticks. No egui here.

use std::ops::Range;

/// Smallest zoom: 64 pixels per sample.
pub const MIN_SPP: f64 = 1.0 / 64.0;

/// Horizontal view onto the capture: `start` is the (fractional) sample at the left edge of the
/// plot and `spp` the number of samples per pixel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct View {
    pub start: f64,
    pub spp: f64,
}

impl Default for View {
    fn default() -> Self {
        Self {
            start: 0.0,
            spp: 1.0,
        }
    }
}

impl View {
    pub fn sample_at(&self, x: f64) -> f64 {
        self.start + x * self.spp
    }

    pub fn x_of(&self, sample: f64) -> f64 {
        (sample - self.start) / self.spp
    }

    pub fn end(&self, width: f64) -> f64 {
        self.sample_at(width)
    }

    /// Show the whole `range` (at least a few samples).
    pub fn fit(range: Range<u64>, width: f64) -> View {
        let len = (range.end.saturating_sub(range.start)).max(8) as f64;
        View {
            start: range.start as f64,
            spp: (len / width.max(1.0)).max(MIN_SPP),
        }
    }

    /// Zoom by `factor` (> 1 zooms in) keeping the sample under pixel `anchor_x` fixed.
    pub fn zoom_at(&mut self, anchor_x: f64, factor: f64) {
        let anchor = self.sample_at(anchor_x);
        self.spp = (self.spp / factor).max(MIN_SPP);
        self.start = anchor - anchor_x * self.spp;
    }

    pub fn pan_px(&mut self, dx: f64) {
        self.start -= dx * self.spp;
    }

    /// Pin the newest sample to the right edge.
    pub fn follow(&mut self, end: u64, width: f64) {
        self.start = end as f64 - width * self.spp;
    }

    /// Keep the view meaningful: bounded zoom-out and at least part of the data in view.
    pub fn clamp(&mut self, range: Range<u64>, width: f64) {
        let len = (range.end.saturating_sub(range.start)).max(8) as f64;
        let max_spp = (len * 4.0 / width.max(1.0)).max(1.0);
        self.spp = self.spp.clamp(MIN_SPP, max_spp);
        let span = width * self.spp;
        self.start = self.start.clamp(
            range.start as f64 - span * 0.9,
            range.end as f64 - span * 0.1,
        );
    }
}

/// 1-2-5 step (seconds) that is at least `min`.
pub fn nice_step(min: f64) -> f64 {
    let min = min.max(1e-15);
    let p = 10f64.powf(min.log10().floor());
    for m in [1.0, 2.0, 5.0, 10.0] {
        if m * p >= min * (1.0 - 1e-9) {
            return m * p;
        }
    }
    10.0 * p
}

/// Format an absolute time in seconds for a ruler whose ticks are `step` seconds apart. The unit
/// follows the magnitude of the time and enough decimals are shown to resolve `step`
/// ("1.518460 s" at a 1 µs step, "250 ns" at a 50 ns step).
pub fn format_time(t: f64, step: f64) -> String {
    let (unit, scale) = unit_for(t.abs().max(step));
    let step_u = step * scale;
    let decimals = if step_u >= 1.0 {
        0
    } else {
        (-step_u.log10()).ceil().clamp(0.0, 9.0) as usize
    };
    format!("{:.*} {}", decimals, t * scale, unit)
}

fn unit_for(t: f64) -> (&'static str, f64) {
    if t < 1e-6 {
        ("ns", 1e9)
    } else if t < 1e-3 {
        ("µs", 1e6)
    } else if t < 1.0 {
        ("ms", 1e3)
    } else {
        ("s", 1.0)
    }
}

/// Exact, easy to read time: groups of three digits per unit, like a timecode
/// ("1 s 518 ms 460 µs 120 ns"). `res` is the sample period; digits finer than it are not shown.
pub fn format_split(t: f64, res: f64) -> String {
    const UNITS: [(&str, i128); 5] = [
        ("s", 1_000_000_000_000),
        ("ms", 1_000_000_000),
        ("µs", 1_000_000),
        ("ns", 1_000),
        ("ps", 1),
    ];
    let res_ps = (res * 1e12).round().max(1.0) as i128;
    // Lowest unit that still shows the sample period exactly.
    let low = UNITS
        .iter()
        .position(|&(_, u)| res_ps % u == 0)
        .unwrap_or(UNITS.len() - 1);
    let step = UNITS[low].1;
    let ps = (t.abs() * 1e12 / step as f64).round() as i128 * step;
    let mut rest = ps;
    let mut groups: Vec<(i128, &str)> = Vec::new();
    for &(name, u) in &UNITS[..=low] {
        groups.push((rest / u, name));
        rest %= u;
    }
    // Drop leading zero groups but keep the last one ("0 ns").
    let first = groups
        .iter()
        .position(|g| g.0 != 0)
        .unwrap_or(groups.len() - 1);
    let mut out = String::from(if t < 0.0 && ps != 0 { "-" } else { "" });
    for (k, (value, name)) in groups[first..].iter().enumerate() {
        if k > 0 {
            out.push(' ');
            out.push_str(&format!("{value:03} {name}"));
        } else {
            out.push_str(&format!("{value} {name}"));
        }
    }
    out
}

/// Grid step shown as "5 ns/div": no trailing zeros ("500 ns", "2.5 µs").
pub fn format_scale(step: f64) -> String {
    let (unit, scale) = unit_for(step);
    let s = format!("{:.3}", step * scale);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    format!("{s} {unit}")
}

/// Parse a time such as "5ns", "2.5 us", "1 ms/div" or a plain number of seconds.
pub fn parse_time(text: &str) -> Option<f64> {
    let t = text.trim().to_lowercase().replace(['µ', 'μ'], "u");
    let t = t.strip_suffix("/div").unwrap_or(&t).trim();
    let (num, mult) = if let Some(n) = t.strip_suffix("ns") {
        (n, 1e-9)
    } else if let Some(n) = t.strip_suffix("us") {
        (n, 1e-6)
    } else if let Some(n) = t.strip_suffix("ms") {
        (n, 1e-3)
    } else if let Some(n) = t.strip_suffix('s') {
        (n, 1.0)
    } else {
        (t, 1.0)
    };
    let v: f64 = num.trim().parse().ok()?;
    (v.is_finite() && v > 0.0).then_some(v * mult)
}

/// Format a duration with three significant digits ("12.5 ns", "3.20 ms").
pub fn format_duration(t: f64) -> String {
    let (unit, scale) = if t < 1e-6 {
        ("ns", 1e9)
    } else if t < 1e-3 {
        ("µs", 1e6)
    } else if t < 1.0 {
        ("ms", 1e3)
    } else {
        ("s", 1.0)
    };
    let v = t * scale;
    let decimals = if v >= 100.0 {
        0
    } else if v >= 10.0 {
        1
    } else {
        2
    };
    format!("{:.*} {}", decimals, v, unit)
}

/// Format a sample rate or frequency.
pub fn format_hz(f: f64) -> String {
    if f >= 1e9 {
        format!("{:.3} GHz", f / 1e9)
    } else if f >= 1e6 {
        format!("{:.3} MHz", f / 1e6)
    } else if f >= 1e3 {
        format!("{:.3} kHz", f / 1e3)
    } else {
        format!("{:.1} Hz", f)
    }
}

/// Pixels per major grid division.
pub const PX_PER_DIV: f64 = 80.0;

/// Major grid step (seconds) at the current zoom: the "scale/div" of the display.
pub fn scale_step(view: &View, rate: f64) -> f64 {
    nice_step(PX_PER_DIV * view.spp / rate)
}

/// Samples per pixel that make `step` seconds the major grid step.
pub fn spp_for_step(step: f64, rate: f64) -> f64 {
    // A division of `step` spans 100 px: >= PX_PER_DIV, and the next smaller 1-2-5 step does not.
    (step * rate / (PX_PER_DIV * 1.25)).max(MIN_SPP)
}

/// Major ticks: every `scale_step` seconds counted from `origin` (the sample that is time zero).
/// Returns the step and `(sample, time in seconds)` pairs; times before the origin are negative and
/// samples before the first sample of the capture are left out.
pub fn ticks(view: &View, width: f64, rate: f64, origin: f64) -> (f64, Vec<(u64, f64)>) {
    let step = scale_step(view, rate);
    let step_samples = step * rate;
    let first = ((view.start - origin) / step_samples).ceil() as i64;
    let last = ((view.end(width) - origin) / step_samples).floor() as i64;
    let v = (first..=last)
        .take(10_000)
        .map(|k| (origin + k as f64 * step_samples, k as f64 * step))
        .filter(|(s, _)| *s >= -0.5)
        .map(|(s, t)| (s.round() as u64, t))
        .collect();
    (step, v)
}

/// Minor subdivisions per major step: 10 for 1-steps, 4 for 2-steps, 5 for 5-steps.
pub fn minor_divisions(step: f64) -> u32 {
    let m = step / 10f64.powf(step.log10().floor());
    match m.round() as u32 {
        2 => 4,
        5 => 5,
        _ => 10,
    }
}

/// Minor tick positions (samples) between the major ticks, excluding the major ticks themselves.
pub fn minor_ticks(view: &View, width: f64, rate: f64, origin: f64) -> Vec<u64> {
    let step = scale_step(view, rate);
    let n = minor_divisions(step) as i64;
    let minor = step * rate / n as f64;
    let first = ((view.start - origin) / minor).ceil() as i64;
    let last = ((view.end(width) - origin) / minor).floor() as i64;
    (first..=last)
        .filter(|k| k.rem_euclid(n) != 0)
        .take(20_000)
        .map(|k| origin + k as f64 * minor)
        .filter(|s| *s >= -0.5)
        .map(|s| s.round() as u64)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zoom_keeps_the_anchor_sample_fixed() {
        let mut v = View {
            start: 1000.0,
            spp: 10.0,
        };
        let anchor = v.sample_at(300.0);
        v.zoom_at(300.0, 2.0);
        assert!((v.sample_at(300.0) - anchor).abs() < 1e-9);
        assert!((v.spp - 5.0).abs() < 1e-12);
        v.zoom_at(0.0, 0.5);
        assert!((v.spp - 10.0).abs() < 1e-12);
    }

    #[test]
    fn zoom_is_bounded() {
        let mut v = View {
            start: 0.0,
            spp: 0.02,
        };
        v.zoom_at(10.0, 100.0);
        assert_eq!(v.spp, MIN_SPP);
    }

    #[test]
    fn fit_and_follow() {
        let v = View::fit(100..1100, 500.0);
        assert_eq!(v.start, 100.0);
        assert!((v.spp - 2.0).abs() < 1e-12);
        assert!((v.end(500.0) - 1100.0).abs() < 1e-9);
        let mut f = View {
            start: 0.0,
            spp: 4.0,
        };
        f.follow(10_000, 250.0);
        assert!((f.end(250.0) - 10_000.0).abs() < 1e-9);
    }

    #[test]
    fn clamp_keeps_data_in_view() {
        let mut v = View {
            start: 1e9,
            spp: 1.0,
        };
        v.clamp(0..1000, 500.0);
        assert!(v.start <= 1000.0 - 50.0);
        let mut v = View {
            start: -1e9,
            spp: 1e9,
        };
        v.clamp(0..1000, 500.0);
        assert!(v.spp <= 8.0 + 1e-9);
        assert!(v.start >= -(500.0 * v.spp));
    }

    #[test]
    fn nice_steps() {
        assert_eq!(nice_step(0.7), 1.0);
        assert_eq!(nice_step(1.0), 1.0);
        assert_eq!(nice_step(1.1), 2.0);
        assert_eq!(nice_step(3.0), 5.0);
        assert_eq!(nice_step(5.1), 10.0);
        assert!((nice_step(0.0003) - 0.0005).abs() < 1e-15);
    }

    #[test]
    fn time_formatting() {
        assert_eq!(format_time(1.5e-3, 5e-4), "1.5 ms");
        assert_eq!(format_time(2.0e-6, 1e-6), "2 µs");
        assert_eq!(format_time(250e-9, 50e-9), "250 ns");
        assert_eq!(format_time(3.0, 1.0), "3 s");
    }

    #[test]
    fn duration_formatting() {
        assert_eq!(format_duration(10e-9), "10.0 ns");
        assert_eq!(format_duration(1.5e-6), "1.50 µs");
        assert_eq!(format_duration(250e-6), "250 µs");
        assert_eq!(format_duration(2.0), "2.00 s");
    }

    #[test]
    fn ticks_are_spaced_and_inside_the_view() {
        let v = View {
            start: 0.0,
            spp: 100.0,
        };
        let (step, t) = ticks(&v, 1000.0, 100e6, 0.0);
        assert!(step >= 80.0 * 100.0 / 100e6);
        assert!(!t.is_empty());
        assert!(t.iter().all(|&(s, _)| (s as f64) <= v.end(1000.0) + 1.0));
        let px: Vec<f64> = t.iter().map(|&(s, _)| v.x_of(s as f64)).collect();
        assert!(px.windows(2).all(|w| w[1] - w[0] >= 79.0));
    }

    #[test]
    fn ticks_count_from_the_origin() {
        // 1 MHz, 100 samples per pixel: a 10 ms step is 10 000 samples; the origin sits at sample 25 000.
        let v = View {
            start: 0.0,
            spp: 125.0,
        };
        let (step, t) = ticks(&v, 1000.0, 1e6, 25_000.0);
        assert!((step - 0.01).abs() < 1e-12, "{step}");
        // Ticks at samples 5000, 15000, 25000, ... are 10 ms steps counted from the origin.
        assert_eq!(t.first(), Some(&(5_000, -0.02)));
        assert!(t.contains(&(25_000, 0.0)) && t.contains(&(35_000, 0.01)));
        // The ruler never labels samples before the capture starts.
        let v = View {
            start: -50_000.0,
            spp: 500.0,
        };
        let (_, t) = ticks(&v, 1000.0, 1e6, 125_000.0);
        assert!(t.iter().all(|&(s, _)| s as f64 >= 0.0) && t.iter().any(|&(_, time)| time < 0.0));
        // Minor ticks follow the same grid and skip the major ticks.
        let minor = minor_ticks(&v, 1000.0, 1e6, 125_000.0);
        assert!(minor.iter().all(|m| t.iter().all(|&(s, _)| s != *m)));
    }

    #[test]
    fn time_labels_resolve_the_step_at_large_times() {
        assert_eq!(format_time(1.51846, 1e-6), "1.518460 s");
        assert_eq!(format_time(0.0, 5e-9), "0 ns");
        assert_eq!(format_time(1.005e-6, 5e-9), "1.005 µs");
    }

    #[test]
    fn split_times_group_by_unit() {
        assert_eq!(format_split(10.0004e-3, 10e-9), "10 ms 000 µs 400 ns");
        assert_eq!(format_split(1.51846012, 10e-9), "1 s 518 ms 460 µs 120 ns");
        assert_eq!(format_split(0.0, 10e-9), "0 ns");
        assert_eq!(format_split(250e-9, 10e-9), "250 ns");
        assert_eq!(format_split(200.4e-6, 1e-9), "200 µs 400 ns");
        assert_eq!(format_split(5.0, 1e-3), "5 s 000 ms");
        assert_eq!(format_split(1.5e-3, 1e-6), "1 ms 500 µs");
        // 400 MHz: the 2.5 ns sample period needs the ps group.
        assert_eq!(format_split(3.75e-9, 2.5e-9), "3 ns 750 ps");
        assert_eq!(format_split(-600e-9, 10e-9), "-600 ns");
    }

    #[test]
    fn scale_text_roundtrip() {
        assert_eq!(format_scale(5e-9), "5 ns");
        assert_eq!(format_scale(500e-9), "500 ns");
        assert_eq!(format_scale(2.5e-6), "2.5 µs");
        assert_eq!(format_scale(1e-3), "1 ms");
        assert_eq!(format_scale(2.0), "2 s");
        for t in [5e-9, 2.5e-6, 1e-3, 2.0] {
            assert!((parse_time(&format_scale(t)).unwrap() / t - 1.0).abs() < 1e-9);
        }
        assert_eq!(parse_time("5ns/div"), Some(5e-9));
        assert!((parse_time(" 10 US ").unwrap() / 10e-6 - 1.0).abs() < 1e-12);
        assert_eq!(parse_time("1e-3"), Some(1e-3));
        assert_eq!(parse_time("µs"), None);
        assert_eq!(parse_time("-5ns"), None);
        assert_eq!(parse_time("abc"), None);
    }

    #[test]
    fn entered_scale_becomes_the_grid_step() {
        let rate = 100e6;
        for step in [1e-9, 2e-9, 5e-9, 1e-6, 2e-6, 5e-6, 1e-3, 0.5] {
            let v = View {
                start: 0.0,
                spp: spp_for_step(step, rate).max(MIN_SPP),
            };
            if spp_for_step(step, rate) > MIN_SPP {
                assert!(
                    (scale_step(&v, rate) / step - 1.0).abs() < 1e-6,
                    "step {step}"
                );
            }
        }
    }

    #[test]
    fn minor_ticks_skip_the_major_ones() {
        let v = View {
            start: 0.0,
            spp: 1.0,
        };
        let (step, major) = ticks(&v, 1000.0, 1e6, 0.0);
        let major: Vec<u64> = major.into_iter().map(|(s, _)| s).collect();
        let minor = minor_ticks(&v, 1000.0, 1e6, 0.0);
        assert!(!minor.is_empty());
        assert!(minor.iter().all(|m| !major.contains(m)));
        assert_eq!(
            minor_divisions(step),
            minor_divisions(1e-4 * 1.0).max(minor_divisions(step))
        );
        assert_eq!(minor_divisions(2e-6), 4);
        assert_eq!(minor_divisions(5e-3), 5);
        assert_eq!(minor_divisions(1e-9), 10);
    }
}
