//! Measurements on one channel over a range of samples: edges, frequency, duty cycle, pulse widths.
//!
//! The numbers come from the edges in the range, so the cost follows the number of edges, not the
//! number of samples. Beyond [`MAX_EDGES`] only the (fast) edge count is given.

use crate::store::CaptureStore;
use std::ops::Range;

/// Most edges that are walked; more are counted only.
pub const MAX_EDGES: u64 = 2_000_000;

/// Count, smallest, largest and total of some durations (in samples).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stat {
    /// How many durations were added.
    pub count: u64,
    /// The shortest (0 if there were none).
    pub min: u64,
    /// The longest (0 if there were none).
    pub max: u64,
    /// Their total.
    pub sum: u64,
}

impl Stat {
    fn add(&mut self, v: u64) {
        if self.count == 0 {
            self.min = v;
            self.max = v;
        } else {
            self.min = self.min.min(v);
            self.max = self.max.max(v);
        }
        self.count += 1;
        self.sum += v;
    }

    /// Mean in samples.
    pub fn mean(&self) -> Option<f64> {
        (self.count > 0).then(|| self.sum as f64 / self.count as f64)
    }
}

/// What the edges of the range say.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Detail {
    /// Rising edges.
    pub rising: u64,
    /// Falling edges.
    pub falling: u64,
    /// Samples spent high and low (the partial segments at both ends count).
    pub high_samples: u64,
    /// See `high_samples`.
    pub low_samples: u64,
    /// Time between consecutive rising edges (falling edges if there are fewer than two rising ones).
    pub period: Stat,
    /// Complete high pulses (rising to falling edge, both inside the range).
    pub high: Stat,
    /// Complete low pulses.
    pub low: Stat,
}

/// The result of [`measure`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Measurement {
    /// The range measured (clipped to the stored samples).
    pub start: u64,
    /// One past the last sample measured.
    pub end: u64,
    /// Level of the first sample.
    pub initial_high: bool,
    /// Edges strictly inside the range.
    pub edges: u64,
    /// `None` when there were more than [`MAX_EDGES`] edges.
    pub detail: Option<Detail>,
}

impl Measurement {
    /// Length of the range in samples.
    pub fn samples(&self) -> u64 {
        self.end - self.start
    }

    /// Duty cycle, 0..1.
    pub fn duty(&self) -> Option<f64> {
        let d = self.detail?;
        let total = d.high_samples + d.low_samples;
        (total > 0).then(|| d.high_samples as f64 / total as f64)
    }

    /// Mean period in samples.
    pub fn period_samples(&self) -> Option<f64> {
        self.detail?.period.mean()
    }
}

/// Measure channel `ch` between the samples of `range`. `None` if the range holds no samples.
pub fn measure(store: &CaptureStore, ch: u8, range: Range<u64>) -> Option<Measurement> {
    let a = range.start.max(store.first_sample());
    let b = range.end.min(store.end());
    if a >= b {
        return None;
    }
    let initial_high = store.level(ch, a)?;
    // An edge at `a` itself belongs to the sample before the range: edges are those in (a, b).
    let edges_range = (a + 1)..b;
    let edges = store.count_toggles(ch, edges_range.clone());
    if edges > MAX_EDGES {
        return Some(Measurement {
            start: a,
            end: b,
            initial_high,
            edges,
            detail: None,
        });
    }
    let mut d = Detail::default();
    let mut level = initial_high;
    let mut seg_start = a;
    let mut last_rise: Option<u64> = None;
    let mut last_fall: Option<u64> = None;
    let mut fall_period = Stat::default();
    store.toggles(ch, edges_range, |p, new_level| {
        let len = p - seg_start;
        if level {
            d.high_samples += len;
        } else {
            d.low_samples += len;
        }
        if new_level {
            d.rising += 1;
            if let Some(f) = last_fall {
                d.low.add(p - f);
            }
            if let Some(r) = last_rise {
                d.period.add(p - r);
            }
            last_rise = Some(p);
        } else {
            d.falling += 1;
            if let Some(r) = last_rise {
                d.high.add(p - r);
            }
            if let Some(f) = last_fall {
                fall_period.add(p - f);
            }
            last_fall = Some(p);
        }
        level = new_level;
        seg_start = p;
    });
    let len = b - seg_start;
    if level {
        d.high_samples += len;
    } else {
        d.low_samples += len;
    }
    if d.period.count == 0 {
        d.period = fall_period;
    }
    Some(Measurement {
        start: a,
        end: b,
        initial_high,
        edges,
        detail: Some(d),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::ChannelCount;
    use crate::store::StoreConfig;
    use proptest::prelude::*;

    fn store(words: &[u32]) -> CaptureStore {
        let mut cfg = StoreConfig::new(ChannelCount::new(4).unwrap(), 1e6);
        cfg.block_samples = 16;
        cfg.chunk_samples = 64;
        let mut s = CaptureStore::new(cfg);
        s.push_samples(words);
        s
    }

    fn square(high: usize, low: usize, periods: usize) -> Vec<u32> {
        (0..periods)
            .flat_map(|_| std::iter::repeat_n(1u32, high).chain(std::iter::repeat_n(0u32, low)))
            .collect()
    }

    #[test]
    fn a_square_wave_has_the_expected_numbers() {
        // 30 high, 70 low, 10 periods: 100 samples per period.
        let s = store(&square(30, 70, 10));
        let m = measure(&s, 0, 0..1000).unwrap();
        let d = m.detail.unwrap();
        assert!(m.initial_high);
        assert_eq!((m.edges, d.rising, d.falling), (19, 9, 10));
        assert_eq!(
            d.period,
            Stat {
                count: 8,
                min: 100,
                max: 100,
                sum: 800
            }
        );
        assert_eq!(
            d.high,
            Stat {
                count: 9,
                min: 30,
                max: 30,
                sum: 270
            }
        );
        assert_eq!(
            d.low,
            Stat {
                count: 9,
                min: 70,
                max: 70,
                sum: 630
            }
        );
        assert_eq!((d.high_samples, d.low_samples), (300, 700));
        assert!((m.duty().unwrap() - 0.3).abs() < 1e-12);
        assert_eq!(m.period_samples(), Some(100.0));
    }

    #[test]
    fn a_sub_range_counts_only_what_is_inside() {
        let s = store(&square(30, 70, 10));
        // From the middle of the first high pulse to the middle of the third.
        let m = measure(&s, 0, 15..215).unwrap();
        let d = m.detail.unwrap();
        assert!(m.initial_high);
        // Edges at 30 (fall), 100 (rise), 130 (fall), 200 (rise).
        assert_eq!((m.edges, d.rising, d.falling), (4, 2, 2));
        assert_eq!(
            d.high,
            Stat {
                count: 1,
                min: 30,
                max: 30,
                sum: 30
            },
            "only the pulse that starts and ends inside"
        );
        assert_eq!(
            d.low,
            Stat {
                count: 2,
                min: 70,
                max: 70,
                sum: 140
            }
        );
        assert_eq!(
            d.period,
            Stat {
                count: 1,
                min: 100,
                max: 100,
                sum: 100
            }
        );
        // High time includes the partial pulses at both ends: 15 + 30 + 15.
        assert_eq!(d.high_samples, 15 + 30 + 15);
        assert_eq!(d.high_samples + d.low_samples, 200);
        // An edge exactly at the start of the range is not inside it.
        let m = measure(&s, 0, 30..100).unwrap();
        assert_eq!((m.edges, m.initial_high), (0, false));
        assert_eq!(m.detail.unwrap().low_samples, 70);
    }

    #[test]
    fn degenerate_ranges() {
        let s = store(&square(3, 3, 4));
        assert!(measure(&s, 0, 5..5).is_none());
        assert!(measure(&s, 0, 100..200).is_none(), "outside the capture");
        let m = measure(&s, 0, 0..1000).unwrap();
        assert_eq!((m.start, m.end), (0, 24), "clipped to what is stored");
        // A constant signal: no edges, no period, duty 1.
        let m = measure(&store(&[1; 50]), 0, 0..50).unwrap();
        assert_eq!(
            (m.edges, m.duty(), m.period_samples()),
            (0, Some(1.0), None)
        );
        // One edge: a single transition has no pulse and no period.
        let mut w = vec![0u32; 20];
        w.extend(vec![1u32; 20]);
        let m = measure(&store(&w), 0, 0..40).unwrap();
        let d = m.detail.unwrap();
        assert_eq!(
            (m.edges, d.rising, d.high.count, d.low.count, d.period.count),
            (1, 1, 0, 0, 0)
        );
    }

    #[test]
    fn falling_edges_give_the_period_when_there_is_one_rising_edge() {
        let mut w = vec![1u32; 5];
        w.extend(vec![0u32; 10]); // fall at 5
        w.extend(vec![1u32; 10]); // rise at 15
        w.extend(vec![0u32; 10]); // fall at 25
        let m = measure(&store(&w), 0, 0..w.len() as u64).unwrap();
        let d = m.detail.unwrap();
        assert_eq!((d.rising, d.falling), (1, 2));
        assert_eq!(
            d.period,
            Stat {
                count: 1,
                min: 20,
                max: 20,
                sum: 20
            }
        );
    }

    #[test]
    fn too_many_edges_are_only_counted() {
        let n = (MAX_EDGES + 1000) as usize;
        let words: Vec<u32> = (0..n).map(|i| (i & 1) as u32).collect();
        let mut cfg = StoreConfig::new(ChannelCount::new(2).unwrap(), 1e6);
        cfg.max_bytes = usize::MAX / 2;
        let mut s = CaptureStore::new(cfg);
        s.push_samples(&words);
        let m = measure(&s, 0, 0..n as u64).unwrap();
        assert!(m.edges > MAX_EDGES && m.detail.is_none());
        assert_eq!(m.duty(), None);
    }

    proptest! {
        #[test]
        fn agrees_with_a_naive_scan(
            bits in prop::collection::vec(any::<bool>(), 2..400),
            a in 0usize..400,
            len in 1usize..400,
        ) {
            let words: Vec<u32> = bits.iter().map(|&b| b as u32).collect();
            let s = store(&words);
            let a = a.min(words.len() - 1);
            let b = (a + len).min(words.len());
            let m = measure(&s, 0, a as u64..b as u64).unwrap();
            let d = m.detail.unwrap();
            let edges: Vec<(usize, bool)> = ((a + 1)..b).filter(|&p| bits[p] != bits[p - 1]).map(|p| (p, bits[p])).collect();
            prop_assert_eq!(m.edges, edges.len() as u64);
            prop_assert_eq!(d.rising, edges.iter().filter(|e| e.1).count() as u64);
            prop_assert_eq!(d.high_samples, bits[a..b].iter().filter(|&&x| x).count() as u64);
            prop_assert_eq!(d.low_samples, bits[a..b].iter().filter(|&&x| !x).count() as u64);
            let rises: Vec<usize> = edges.iter().filter(|e| e.1).map(|e| e.0).collect();
            let falls: Vec<usize> = edges.iter().filter(|e| !e.1).map(|e| e.0).collect();
            if rises.len() >= 2 {
                let gaps: Vec<u64> = rises.windows(2).map(|w| (w[1] - w[0]) as u64).collect();
                prop_assert_eq!(d.period.count, gaps.len() as u64);
                prop_assert_eq!(d.period.min, *gaps.iter().min().unwrap());
                prop_assert_eq!(d.period.max, *gaps.iter().max().unwrap());
            }
            let pulses: Vec<u64> = rises.iter().filter_map(|&r| falls.iter().find(|&&f| f > r).map(|&f| (f - r) as u64)).collect();
            prop_assert_eq!(d.high.count, pulses.len() as u64);
            prop_assert_eq!(d.high.sum, pulses.iter().sum::<u64>());
        }
    }
}
