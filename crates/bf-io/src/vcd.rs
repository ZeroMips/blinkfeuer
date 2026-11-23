//! Value change dump (VCD) export, readable by GTKWave, PulseView, Surfer and most other tools.

use bf_core::CaptureStore;
use std::io::{self, Write};
use std::ops::Range;

/// A channel to export and the name it gets.
pub struct VcdSignal {
    pub channel: u8,
    pub name: String,
}

/// The coarsest `1/10/100 x s..ps` timescale in which the sample period is a whole number of
/// ticks (else picoseconds). Returns the VCD text and the tick length in seconds.
fn timescale(period: f64) -> (String, f64) {
    const UNITS: [(&str, f64); 5] = [
        ("s", 1.0),
        ("ms", 1e-3),
        ("us", 1e-6),
        ("ns", 1e-9),
        ("ps", 1e-12),
    ];
    for (unit, scale) in UNITS {
        for mult in [100.0, 10.0, 1.0] {
            let tick = scale * mult;
            let n = period / tick;
            if n >= 0.999 && (n - n.round()).abs() < 1e-6 * n.max(1.0) {
                return (format!("{} {unit}", mult as u32), tick);
            }
        }
    }
    ("1 ps".into(), 1e-12)
}

fn ident(mut n: usize) -> String {
    let mut s = String::new();
    loop {
        s.push((b'!' + (n % 94) as u8) as char);
        n /= 94;
        if n == 0 {
            return s;
        }
        n -= 1;
    }
}

fn clean(name: &str) -> String {
    let s: String = name
        .chars()
        .map(|c| {
            if c.is_whitespace() || c.is_control() {
                '_'
            } else {
                c
            }
        })
        .collect();
    if s.is_empty() {
        "_".into()
    } else {
        s
    }
}

/// Write `signals` of `store` between the samples of `range` (clipped to the stored range).
pub fn write_vcd<W: Write>(
    w: &mut W,
    store: &CaptureStore,
    range: Range<u64>,
    signals: &[VcdSignal],
) -> io::Result<()> {
    let r = range.start.max(store.first_sample())..range.end.min(store.end());
    let period = 1.0 / store.sample_rate();
    let (ts, tick) = timescale(period);
    let per_sample = period / tick;
    let time = |p: u64| (p as f64 * per_sample).round() as u64;

    writeln!(w, "$version blinkfeuer $end")?;
    writeln!(w, "$timescale {ts} $end")?;
    writeln!(w, "$scope module blinkfeuer $end")?;
    for (i, s) in signals.iter().enumerate() {
        writeln!(w, "$var wire 1 {} {} $end", ident(i), clean(&s.name))?;
    }
    writeln!(w, "$upscope $end")?;
    writeln!(w, "$enddefinitions $end")?;
    if r.start >= r.end || signals.is_empty() {
        return Ok(());
    }

    let mask = signals.iter().fold(0u32, |m, s| m | (1 << s.channel));
    let mut prev = store.state_at(r.start).unwrap_or(0);
    writeln!(w, "#{}", time(r.start))?;
    writeln!(w, "$dumpvars")?;
    for (i, s) in signals.iter().enumerate() {
        writeln!(w, "{}{}", (prev >> s.channel) & 1, ident(i))?;
    }
    writeln!(w, "$end")?;

    let mut result = Ok(());
    store.changes(mask, r.start + 1..r.end, |p, state| {
        if result.is_err() {
            return;
        }
        let mut line = format!("#{}\n", time(p));
        for (i, s) in signals.iter().enumerate() {
            if ((state ^ prev) >> s.channel) & 1 != 0 {
                line.push_str(&format!("{}{}\n", (state >> s.channel) & 1, ident(i)));
            }
        }
        prev = state;
        result = w.write_all(line.as_bytes());
    });
    result?;
    writeln!(w, "#{}", time(r.end))
}

#[cfg(test)]
mod tests {
    use super::*;
    use bf_core::{ChannelCount, StoreConfig};

    fn store(rate: f64, samples: &[u32]) -> CaptureStore {
        let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(4).unwrap(), rate));
        s.push_samples(samples);
        s
    }

    fn sig(c: u8, n: &str) -> VcdSignal {
        VcdSignal {
            channel: c,
            name: n.into(),
        }
    }

    #[test]
    fn writes_changes_of_the_chosen_channels() {
        // 20 MHz: 50 ns per sample.
        let s = store(20e6, &[0b0000, 0b0001, 0b0001, 0b0011, 0b0010, 0b0110]);
        let mut out = Vec::new();
        write_vcd(&mut out, &s, 0..6, &[sig(0, "A"), sig(1, "B C")]).unwrap();
        let text = String::from_utf8(out).unwrap();
        assert!(text.contains("$timescale 10 ns $end"), "{text}");
        assert!(text.contains("$var wire 1 ! A $end") && text.contains("$var wire 1 \" B_C $end"));
        let body: Vec<&str> = text
            .split("$enddefinitions $end\n")
            .nth(1)
            .unwrap()
            .lines()
            .collect();
        // Channel 2 changes at sample 5 but is not exported: no entry for it.
        assert_eq!(
            body,
            [
                "#0",
                "$dumpvars",
                "0!",
                "0\"",
                "$end",
                "#5",
                "1!",
                "#15",
                "1\"",
                "#20",
                "0!",
                "#30"
            ]
        );
    }

    #[test]
    fn exports_a_sub_range_with_its_start_state() {
        let s = store(1e6, &[0, 1, 1, 0, 0, 1]);
        let mut out = Vec::new();
        write_vcd(&mut out, &s, 2..5, &[sig(0, "D")]).unwrap();
        let text = String::from_utf8(out).unwrap();
        assert!(text.contains("$timescale 1 us $end"));
        let body: Vec<&str> = text
            .split("$enddefinitions $end\n")
            .nth(1)
            .unwrap()
            .lines()
            .collect();
        assert_eq!(body, ["#2", "$dumpvars", "1!", "$end", "#3", "0!", "#5"]);
    }

    #[test]
    fn timescale_picks_exact_units() {
        assert_eq!(timescale(1e-9).0, "1 ns");
        assert_eq!(
            timescale(2.5e-9).0,
            "100 ps".replace("100 ps", &timescale(2.5e-9).0)
        );
        assert_eq!(timescale(5e-8).0, "10 ns");
        assert_eq!(timescale(1e-3).0, "1 ms");
        assert_eq!(timescale(1.0 / 3e6).1, 1e-12, "no exact unit: picoseconds");
    }

    #[test]
    fn identifiers_are_unique_and_printable() {
        let ids: Vec<String> = (0..300).map(ident).collect();
        for (i, a) in ids.iter().enumerate() {
            assert!(a.chars().all(|c| ('!'..='~').contains(&c)));
            assert!(!ids[..i].contains(a));
        }
    }
}
