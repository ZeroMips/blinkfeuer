//! Trigger condition, evaluated on the host over the sample stream (neither backend has a hardware
//! trigger).
//!
//! One stage: every channel has a condition and the trigger fires on the first sample where *all*
//! of them hold. Level conditions (0, 1) look at the sample itself, edge conditions compare it with
//! the sample before.

use serde::{Deserialize, Serialize};

/// A trigger that is not a bit pattern but a state machine over the *changes* of some channels, such
/// as a protocol decoder looking for a byte. It sees only the samples where a watched channel
/// changes, so its cost follows the number of edges, not the number of samples.
pub trait EventTrigger: Send {
    /// The channels it watches (bit mask of the sample word).
    fn mask(&self) -> u32;
    /// Start over at the beginning of a capture.
    fn reset(&mut self, sample_rate: f64);
    /// A watched channel changed at sample `t`; `state` is the full word from `t` on. The first call
    /// after `reset` carries the initial state. Returns the trigger sample when the trigger fires.
    fn change(&mut self, t: u64, state: u32) -> Option<u64>;
    /// Nothing changed before sample `end` (lets pending items complete).
    fn advance(&mut self, end: u64) -> Option<u64>;
    /// How many samples before the sample where it is *detected* the trigger sample can lie.
    fn lookback(&self) -> u64 {
        0
    }
}

/// What one channel has to do for the trigger to fire.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum Cond {
    /// Don't care.
    #[default]
    X,
    /// The channel is low.
    Low,
    /// The channel is high.
    High,
    /// The channel goes from low to high.
    Rising,
    /// The channel goes from high to low.
    Falling,
    /// Any edge.
    Either,
}

impl Cond {
    /// All conditions, in the order the user interface offers them.
    pub const ALL: [Cond; 6] = [
        Cond::X,
        Cond::Low,
        Cond::High,
        Cond::Rising,
        Cond::Falling,
        Cond::Either,
    ];

    /// Short label for the user interface.
    pub fn label(self) -> &'static str {
        match self {
            Cond::X => "X",
            Cond::Low => "0",
            Cond::High => "1",
            Cond::Rising => "↑",
            Cond::Falling => "↓",
            Cond::Either => "↕",
        }
    }

    /// What the condition means, in words.
    pub fn describe(self) -> &'static str {
        match self {
            Cond::X => "don't care",
            Cond::Low => "low",
            Cond::High => "high",
            Cond::Rising => "rising edge",
            Cond::Falling => "falling edge",
            Cond::Either => "any edge",
        }
    }
}

/// The conditions in a form that is checked with a few word operations per sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Trigger {
    /// Channels that must be 0 or 1 (`level_mask`) and the value they must have.
    level_mask: u32,
    level_value: u32,
    rising: u32,
    falling: u32,
    either: u32,
}

impl Trigger {
    /// `conds[c]` is the condition of channel `c` (missing channels are don't-care).
    pub fn new(conds: &[Cond]) -> Self {
        let mut t = Self {
            level_mask: 0,
            level_value: 0,
            rising: 0,
            falling: 0,
            either: 0,
        };
        for (c, cond) in conds.iter().enumerate().take(32) {
            let bit = 1u32 << c;
            match cond {
                Cond::X => {}
                Cond::Low => t.level_mask |= bit,
                Cond::High => {
                    t.level_mask |= bit;
                    t.level_value |= bit;
                }
                Cond::Rising => t.rising |= bit,
                Cond::Falling => t.falling |= bit,
                Cond::Either => t.either |= bit,
            }
        }
        t
    }

    /// Nothing is required: the trigger fires on the first sample it may fire on.
    pub fn is_empty(&self) -> bool {
        (self.level_mask | self.rising | self.falling | self.either) == 0
    }

    /// Does sample `cur` (after `prev`) satisfy the trigger?
    #[inline(always)]
    pub fn matches(&self, prev: u32, cur: u32) -> bool {
        (cur & self.level_mask) == self.level_value
            && (!prev & cur & self.rising) == self.rising
            && (prev & !cur & self.falling) == self.falling
            && ((prev ^ cur) & self.either) == self.either
    }

    /// Index of the first sample of `words` that satisfies the trigger; `prev` is the sample before
    /// `words[0]`.
    pub fn find(&self, prev: u32, words: &[u32]) -> Option<usize> {
        let &first = words.first()?;
        if self.matches(prev, first) {
            return Some(0);
        }
        // Level-only triggers need no previous sample: a tight loop the compiler can unroll.
        if (self.rising | self.falling | self.either) == 0 {
            return words
                .iter()
                .position(|&w| (w & self.level_mask) == self.level_value);
        }
        words
            .windows(2)
            .position(|w| self.matches(w[0], w[1]))
            .map(|i| i + 1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn conds(spec: &[(usize, Cond)]) -> Vec<Cond> {
        let mut v = vec![Cond::X; 8];
        for (c, k) in spec {
            v[*c] = *k;
        }
        v
    }

    #[test]
    fn levels_and_edges() {
        let t = Trigger::new(&conds(&[(0, Cond::High), (1, Cond::Low)]));
        assert!(t.matches(0, 0b01) && t.matches(0b11, 0b101));
        assert!(!t.matches(0, 0b11) && !t.matches(0, 0b10) && !t.matches(0, 0));

        let r = Trigger::new(&conds(&[(2, Cond::Rising)]));
        assert!(r.matches(0b000, 0b100) && r.matches(0b011, 0b111));
        assert!(!r.matches(0b100, 0b100) && !r.matches(0b100, 0b000) && !r.matches(0, 0));

        let f = Trigger::new(&conds(&[(2, Cond::Falling)]));
        assert!(f.matches(0b100, 0b000) && !f.matches(0b000, 0b100));

        let e = Trigger::new(&conds(&[(2, Cond::Either)]));
        assert!(e.matches(0b100, 0b000) && e.matches(0b000, 0b100) && !e.matches(0b100, 0b100));

        // All conditions together: CH0 rising while CH1 is high and CH2 falls.
        let all = Trigger::new(&conds(&[
            (0, Cond::Rising),
            (1, Cond::High),
            (2, Cond::Falling),
        ]));
        assert!(all.matches(0b100, 0b011));
        assert!(
            !all.matches(0b100, 0b001) && !all.matches(0b000, 0b011) && !all.matches(0b101, 0b011)
        );
    }

    #[test]
    fn empty_trigger_fires_at_once() {
        let t = Trigger::new(&[]);
        assert!(t.is_empty());
        assert_eq!(t.find(5, &[1, 2, 3]), Some(0));
        assert_eq!(t.find(5, &[]), None);
        assert!(!Trigger::new(&conds(&[(0, Cond::X), (1, Cond::High)])).is_empty());
    }

    #[test]
    fn find_uses_the_sample_before_the_buffer() {
        let t = Trigger::new(&conds(&[(0, Cond::Rising)]));
        // The edge is between the previous buffer and this one.
        assert_eq!(t.find(0, &[1, 1, 0, 1]), Some(0));
        assert_eq!(t.find(1, &[1, 1, 0, 1]), Some(3));
        assert_eq!(t.find(1, &[1, 1, 1]), None);
    }

    fn cond() -> impl Strategy<Value = Cond> {
        prop::sample::select(Cond::ALL.to_vec())
    }

    proptest! {
        #[test]
        fn find_agrees_with_a_sample_by_sample_scan(
            cs in prop::collection::vec(cond(), 8),
            prev in any::<u8>(),
            words in prop::collection::vec(any::<u8>(), 0..64),
        ) {
            let t = Trigger::new(&cs);
            let words: Vec<u32> = words.into_iter().map(u32::from).collect();
            let mut p = prev as u32;
            let mut expect = None;
            for (i, &w) in words.iter().enumerate() {
                let ok = cs.iter().enumerate().all(|(c, k)| {
                    let (a, b) = ((p >> c) & 1, (w >> c) & 1);
                    match k {
                        Cond::X => true,
                        Cond::Low => b == 0,
                        Cond::High => b == 1,
                        Cond::Rising => a == 0 && b == 1,
                        Cond::Falling => a == 1 && b == 0,
                        Cond::Either => a != b,
                    }
                });
                if ok {
                    expect = Some(i);
                    break;
                }
                p = w;
            }
            prop_assert_eq!(t.find(prev as u32, &words), expect);
        }
    }

    #[test]
    #[ignore = "benchmark: cargo test -p bf-core --release -- --ignored --nocapture"]
    fn scan_speed() {
        let words: Vec<u32> = (0..(1u32 << 24)).map(|i| i & 0x7fff).collect();
        for (name, spec) in [
            ("level", vec![(15, Cond::High)]),
            ("edge", vec![(15, Cond::Rising)]),
            (
                "mixed",
                vec![(14, Cond::High), (3, Cond::Falling), (15, Cond::Either)],
            ),
        ] {
            let mut v = vec![Cond::X; 16];
            for (c, k) in spec {
                v[c] = k;
            }
            let t = Trigger::new(&v);
            let t0 = std::time::Instant::now();
            let mut hits = 0u64;
            for _ in 0..8 {
                hits += t.find(0, &words).is_some() as u64;
            }
            let s = t0.elapsed().as_secs_f64();
            println!(
                "{name}: {:.2} GSa/s ({hits})",
                8.0 * words.len() as f64 / s / 1e9
            );
        }
    }
}
