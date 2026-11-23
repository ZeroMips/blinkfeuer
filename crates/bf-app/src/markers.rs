//! Time markers M1/M2 with delta readout and snap-to-edge, as on a bench logic analyzer.

use crate::view::{format_duration, format_hz};
use bf_core::CaptureStore;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Which {
    M1,
    M2,
}

/// Marker positions in (possibly fractional) sample numbers.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Markers {
    pub m1: Option<f64>,
    pub m2: Option<f64>,
}

impl Markers {
    pub fn get(&self, w: Which) -> Option<f64> {
        match w {
            Which::M1 => self.m1,
            Which::M2 => self.m2,
        }
    }

    pub fn set(&mut self, w: Which, s: f64) {
        match w {
            Which::M1 => self.m1 = Some(s),
            Which::M2 => self.m2 = Some(s),
        }
    }

    /// The marker closer than `tol` (in samples) to `s`, nearest first.
    pub fn hit(&self, s: f64, tol: f64) -> Option<Which> {
        [Which::M1, Which::M2]
            .into_iter()
            .filter_map(|w| self.get(w).map(|p| (w, (p - s).abs())))
            .filter(|&(_, d)| d <= tol)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(w, _)| w)
    }

    /// Distance M1 to M2 in samples (signed: M2 - M1).
    pub fn delta_samples(&self) -> Option<f64> {
        Some(self.m2? - self.m1?)
    }
}

/// "M1 to M2 = 525 ns" and "1/dt = 1.905 MHz" parts of the readout.
pub fn delta_text(m: &Markers, rate: f64) -> Option<(String, Option<String>)> {
    let d = m.delta_samples()?;
    let t = d.abs() / rate;
    let freq = (t > 0.0).then(|| format_hz(1.0 / t));
    let sign = if d < 0.0 { "-" } else { "" };
    Some((format!("{sign}{}", format_duration(t)), freq))
}

/// The edge of any of `channels` nearest to sample `at` within `tol` samples.
pub fn snap_to_edge(store: &CaptureStore, channels: &[u8], at: f64, tol: f64) -> Option<u64> {
    let lo = (at - tol).floor().max(store.first_sample() as f64) as u64;
    // Range ends are exclusive: include an edge sitting exactly at `at + tol`.
    let hi = ((at + tol).floor().max(0.0) as u64 + 1).min(store.end());
    if lo >= hi {
        return None;
    }
    let mut best: Option<(f64, u64)> = None;
    for &ch in channels {
        for (p, _) in store.edges(ch, lo..hi, usize::MAX)? {
            let d = (p as f64 - at).abs();
            if d <= tol && best.is_none_or(|(bd, _)| d < bd) {
                best = Some((d, p));
            }
        }
    }
    best.map(|(_, p)| p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bf_core::{ChannelCount, StoreConfig};

    fn store(samples: &[u32]) -> CaptureStore {
        let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(8).unwrap(), 100e6));
        s.push_samples(samples);
        s
    }

    #[test]
    fn delta_readout() {
        let m = Markers {
            m1: Some(100.0),
            m2: Some(152.5),
        };
        let (t, f) = delta_text(&m, 100e6).unwrap();
        assert_eq!(t, "525 ns");
        assert_eq!(f.unwrap(), "1.905 MHz");
        let m = Markers {
            m1: Some(200.0),
            m2: Some(100.0),
        };
        assert_eq!(delta_text(&m, 100e6).unwrap().0, "-1.00 µs");
        assert!(delta_text(
            &Markers {
                m1: Some(1.0),
                m2: None
            },
            100e6
        )
        .is_none());
        let same = Markers {
            m1: Some(5.0),
            m2: Some(5.0),
        };
        assert_eq!(delta_text(&same, 1e6).unwrap().1, None);
    }

    #[test]
    fn hit_picks_the_nearest_marker() {
        let m = Markers {
            m1: Some(100.0),
            m2: Some(110.0),
        };
        assert_eq!(m.hit(103.0, 5.0), Some(Which::M1));
        assert_eq!(m.hit(108.0, 5.0), Some(Which::M2));
        assert_eq!(m.hit(200.0, 5.0), None);
        assert_eq!(Markers::default().hit(0.0, 100.0), None);
    }

    #[test]
    fn snapping_finds_the_nearest_edge_in_tolerance() {
        // channel 0 rises at 10 and falls at 20, channel 1 rises at 14.
        let mut v = vec![0u32; 30];
        v[10..20].fill(1);
        v[14..].iter_mut().for_each(|x| *x |= 2);
        let s = store(&v);
        assert_eq!(snap_to_edge(&s, &[0], 12.0, 3.0), Some(10));
        assert_eq!(snap_to_edge(&s, &[0], 17.0, 3.0), Some(20));
        assert_eq!(snap_to_edge(&s, &[0, 1], 13.0, 3.0), Some(14));
        assert_eq!(snap_to_edge(&s, &[0], 15.0, 2.0), None);
        assert_eq!(snap_to_edge(&s, &[3], 15.0, 20.0), None);
    }
}
