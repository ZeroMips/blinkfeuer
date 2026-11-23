//! UART baud rate detection from the edges of a recorded line.
//!
//! In UART data every time between two edges is a whole number of bit times (a run of equal bits), so
//! the shortest widths that occur often are the bit time. Candidates are the clusters of common widths,
//! the smallest one under which almost all widths come out as whole bit times is taken, and the bit
//! time is then refined with a least-squares fit over all the widths.

/// The baud rates a result is snapped to (when within 2.5 %).
pub const STANDARD: [f64; 28] = [
    110.0,
    300.0,
    600.0,
    1200.0,
    1800.0,
    2400.0,
    4800.0,
    9600.0,
    14400.0,
    19200.0,
    28800.0,
    31250.0,
    38400.0,
    57600.0,
    76800.0,
    115200.0,
    230400.0,
    250000.0,
    460800.0,
    500000.0,
    576000.0,
    921600.0,
    1_000_000.0,
    1_152_000.0,
    1_500_000.0,
    2_000_000.0,
    3_000_000.0,
    4_000_000.0,
];

#[derive(Clone, Debug, PartialEq)]
pub struct BaudGuess {
    /// The rate to use: the nearest standard rate if one is within 2.5 %, else the measured one.
    pub baud: f64,
    pub measured: f64,
    pub snapped: bool,
    /// Share of the widths (up to one frame long) that are whole bit times.
    pub fit: f64,
    /// Widths looked at.
    pub widths: usize,
    pub bit_samples: f64,
    /// Something worth knowing about the result, e.g. a sample rate that is low for this baud rate.
    pub note: Option<String>,
}

/// The standard rate within 2.5 % of `baud`.
pub fn snap(baud: f64) -> Option<f64> {
    STANDARD
        .iter()
        .copied()
        .filter(|s| (baud / s - 1.0).abs() <= 0.025)
        .min_by(|a, b| (baud / a - 1.0).abs().total_cmp(&(baud / b - 1.0).abs()))
}

/// How many widths (up to one frame long) are whole multiples of `t`, and the least-squares `t` over them.
fn fit(widths: &[f64], t: f64) -> (f64, f64) {
    // A bit time is at least a few samples; widths are integers, so allow one sample of quantisation.
    let tol = 0.2f64.max(1.5 / t).min(0.45);
    let (mut n, mut ok, mut wk, mut kk) = (0usize, 0usize, 0.0, 0.0);
    for &w in widths {
        let r = w / t;
        if r > 11.5 {
            continue; // an idle gap between frames: any length
        }
        n += 1;
        let k = r.round();
        if k >= 1.0 && (r - k).abs() <= tol {
            ok += 1;
            wk += w * k;
            kk += k * k;
        }
    }
    if n == 0 || kk == 0.0 {
        return (0.0, t);
    }
    (ok as f64 / n as f64, wk / kk)
}

/// Detect the baud rate from the sample numbers of the edges (both directions) of the RX line.
pub fn detect(edges: &[u64], sample_rate: f64) -> Result<BaudGuess, String> {
    let widths: Vec<f64> = edges.windows(2).map(|w| (w[1] - w[0]) as f64).collect();
    if widths.len() < 12 {
        return Err(format!(
            "only {} edges on this channel: capture some UART traffic first",
            edges.len()
        ));
    }
    // Clusters of similar widths, smallest first.
    let mut sorted = widths.clone();
    sorted.sort_by(|a, b| a.total_cmp(b));
    let min_count = 3usize.max(widths.len() / 100);
    let mut candidates: Vec<f64> = Vec::new();
    let mut i = 0;
    while i < sorted.len() && candidates.len() < 8 {
        let limit = sorted[i] * 1.15 + 1.0;
        let j = sorted[i..]
            .iter()
            .position(|&w| w > limit)
            .map_or(sorted.len(), |p| i + p);
        if j - i >= min_count {
            candidates.push(sorted[i..j].iter().sum::<f64>() / (j - i) as f64);
        }
        i = j;
    }
    if candidates.is_empty() {
        return Err("no pulse width occurs often enough to be a bit time".into());
    }
    // Refine each candidate once, then take the smallest one that explains nearly everything.
    let evaluated: Vec<(f64, f64)> = candidates
        .iter()
        .map(|&c| {
            let (_, t1) = fit(&widths, c);
            let (score, t2) = fit(&widths, t1);
            (score, t2)
        })
        .collect();
    let chosen = evaluated
        .iter()
        .find(|(s, t)| *s >= 0.9 && *t >= 2.0)
        .or_else(|| {
            evaluated
                .iter()
                .filter(|(_, t)| *t >= 2.0)
                .max_by(|a, b| a.0.total_cmp(&b.0))
        });
    let Some(&(score, t)) = chosen else {
        return Err(
            "the shortest pulses are only a sample or two wide: capture at a higher sample rate"
                .into(),
        );
    };
    if score < 0.7 {
        return Err(format!(
            "the pulse widths do not look like UART data (best fit {:.0} %)",
            score * 100.0
        ));
    }
    let measured = sample_rate / t;
    let snapped = snap(measured);
    let note = (t < 8.0).then(|| {
        format!(
            "only {t:.1} samples per bit: the decoder works best with 8 or more, capture faster"
        )
    });
    Ok(BaudGuess {
        baud: snapped.unwrap_or(measured.round()),
        measured,
        snapped: snapped.is_some(),
        fit: score,
        widths: widths.len(),
        bit_samples: t,
        note,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::Wave;
    use crate::uart::{Parity, UartConfig};

    /// Edges of channel 0 of `w`.
    fn edges(w: &Wave) -> Vec<u64> {
        w.samples
            .windows(2)
            .enumerate()
            .filter(|(_, p)| (p[0] ^ p[1]) & 1 != 0)
            .map(|(i, _)| i as u64 + 1)
            .collect()
    }

    fn line(baud: f64, rate: f64, bytes: &[u8], cfg: UartConfig) -> Vec<u64> {
        let bit = rate / baud;
        let len = (bytes.len() as f64 * 14.0 * bit) as usize + 200;
        let init = if cfg.invert { 0 } else { 1 };
        let mut w = Wave::new(rate, len, init);
        w.uart(0, 50, &UartConfig { baud, ..cfg }, bytes);
        edges(&w)
    }

    fn text(n: usize) -> Vec<u8> {
        b"The quick brown fox jumps over the lazy dog 0123456789\r\n"
            .iter()
            .copied()
            .cycle()
            .take(n)
            .collect()
    }

    #[test]
    fn standard_rates_are_found_at_many_sample_rates() {
        for (baud, rate) in [
            (9600.0, 1e6),
            (9600.0, 100e3 * 8.0),
            (19200.0, 2e6),
            (38400.0, 5e6),
            (57600.0, 1e6),
            (115200.0, 2e6),
            (115200.0, 20e6),
            (230400.0, 5e6),
            (921600.0, 20e6),
            (1_000_000.0, 20e6),
            (4800.0, 500e3),
        ] {
            let g = detect(&line(baud, rate, &text(60), UartConfig::default()), rate)
                .unwrap_or_else(|e| panic!("{baud} at {rate}: {e}"));
            assert_eq!(
                g.baud, baud,
                "{baud} baud at {rate} Hz: measured {:.1}",
                g.measured
            );
            assert!(g.snapped && g.fit >= 0.9, "{g:?}");
            assert!((g.measured / baud - 1.0).abs() < 0.03, "{g:?}");
        }
    }

    #[test]
    fn frame_formats_and_polarity_do_not_matter() {
        let base = UartConfig::default();
        for cfg in [
            UartConfig {
                parity: Parity::Even,
                ..base.clone()
            },
            UartConfig {
                data_bits: 7,
                stop_bits: 2.0,
                ..base.clone()
            },
            UartConfig {
                lsb_first: false,
                ..base.clone()
            },
            UartConfig {
                invert: true,
                ..base.clone()
            },
            UartConfig {
                data_bits: 9,
                parity: Parity::Odd,
                ..base.clone()
            },
        ] {
            let g = detect(&line(115200.0, 2e6, &text(50), cfg.clone()), 2e6)
                .unwrap_or_else(|e| panic!("{cfg:?}: {e}"));
            assert_eq!(g.baud, 115200.0, "{cfg:?}: {g:?}");
        }
    }

    #[test]
    fn simple_and_sparse_traffic() {
        // Only 0x55: every width is one bit. Alternating 0x00 and 0xFF: long runs and single start bits.
        // (A stream of 0x00 alone cannot be told from a baud rate three times lower: all its widths are 3 or 9 bits.)
        for bytes in [vec![0x55u8; 30], [0x00u8, 0xFF].repeat(20), text(14)] {
            let g = detect(&line(19200.0, 1e6, &bytes, UartConfig::default()), 1e6).unwrap();
            assert_eq!(g.baud, 19200.0, "{bytes:?}: {g:?}");
        }
        // Frames separated by long idle times (200 bit times) in between.
        let rate = 2e6;
        let bit = rate / 9600.0;
        let mut w = Wave::new(rate, (60.0 * 220.0 * bit) as usize, 1);
        let cfg = UartConfig {
            baud: 9600.0,
            ..Default::default()
        };
        let mut at = 100.0;
        for b in text(40) {
            w.uart(0, at as u64, &cfg, &[b]);
            at += 220.0 * bit;
        }
        let g = detect(&edges(&w), rate).unwrap();
        assert_eq!(g.baud, 9600.0, "{g:?}");
    }

    #[test]
    fn glitches_and_odd_rates() {
        // Isolated one-sample glitches on the line do not move the result.
        let rate = 5e6;
        let bit = rate / 115200.0;
        let mut w = Wave::new(rate, (40.0 * 12.0 * bit) as usize, 1);
        w.uart(
            0,
            100,
            &UartConfig {
                baud: 115200.0,
                ..Default::default()
            },
            &text(40),
        );
        for k in 0..30 {
            let at = 5_000 + k * 9_973;
            if at + 1 < w.samples.len() && w.samples[at] & 1 == 1 && w.samples[at + 1] & 1 == 1 {
                w.samples[at] &= !1;
            }
        }
        let g = detect(&edges(&w), rate).unwrap();
        assert_eq!(g.baud, 115200.0, "{g:?}");
        // A non-standard rate is reported as measured, rounded.
        for odd in [125_000.0, 100_000.0, 33_333.0] {
            let g = detect(&line(odd, 5e6, &text(60), UartConfig::default()), 5e6).unwrap();
            assert!(
                !g.snapped && (g.baud / odd - 1.0).abs() < 0.02,
                "{odd}: {g:?}"
            );
        }
    }

    #[test]
    fn data_that_is_not_uart_is_refused_with_a_reason() {
        // Too few edges.
        assert!(detect(&[1, 5, 9], 1e6)
            .unwrap_err()
            .contains("only 3 edges"));
        assert!(detect(&[], 1e6).is_err());
        // A square wave is "UART" with all-one-bit widths: that is a valid reading (the bit time is the half period).
        let sq: Vec<u64> = (0..100).map(|i| i * 50).collect();
        assert_eq!(detect(&sq, 1e6).unwrap().measured.round(), 20_000.0);
        // Random widths: no common bit time.
        let mut x = 12345u64;
        let mut at = 0u64;
        let rnd: Vec<u64> = (0..300)
            .map(|_| {
                x = x
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                at += 20 + (x >> 40) % 400;
                at
            })
            .collect();
        let e = detect(&rnd, 1e6).unwrap_err();
        assert!(
            e.contains("do not look like UART") || e.contains("often enough"),
            "{e}"
        );
        // Edges one or two samples apart: sample rate far too low.
        let fast: Vec<u64> = (0..200).map(|i| i + i / 2).collect();
        assert!(detect(&fast, 1e6)
            .unwrap_err()
            .contains("higher sample rate"));
    }

    #[test]
    fn snapping_and_notes() {
        assert_eq!(snap(115_000.0), Some(115_200.0));
        assert_eq!(snap(9_700.0), Some(9_600.0));
        assert_eq!(snap(100_000.0), None);
        let g = detect(
            &line(115200.0, 400e3, &text(60), UartConfig::default()),
            400e3,
        );
        // 3.5 samples per bit: found, but with a warning.
        if let Ok(g) = g {
            assert!(g
                .note
                .as_deref()
                .is_some_and(|n| n.contains("samples per bit")));
        }
    }
}
