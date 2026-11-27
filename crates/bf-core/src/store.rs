//! Capture store: sparse state-change events plus per-block transition counts.
//!
//! Real signals are mostly constant, so the store keeps only the samples where *any* channel
//! changes (an [`Event`]: sample offset and the new full state word). On top of that every block of
//! `block_samples` samples keeps its starting state and a saturating per-channel transition count.
//! That lets the UI draw zoomed-out views from the block counts (cost proportional to the number of
//! blocks in view, not to the number of samples) and exact edges when zoomed in.
//!
//! The store is append-only with a memory cap: the oldest chunks are dropped first (rolling capture).
//!
//! Conventions used by every query:
//! * Sample `s` has state `state_at(s)`. The first sample of a capture has no transition.
//! * A *toggle* of channel `c` "at `p`" means sample `p-1` and sample `p` differ in bit `c`.
//!   `p` is the first sample with the new level.

use crate::format::ChannelCount;
use parking_lot::RwLock;
use std::collections::VecDeque;
use std::ops::Range;
use std::sync::Arc;

/// A change of the sample state: `state` is the full state of sample `off` (relative to its chunk).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    /// Sample offset from the start of the chunk.
    pub off: u32,
    /// State word from this sample on.
    pub state: u32,
}

/// Store parameters.
#[derive(Clone, Debug)]
pub struct StoreConfig {
    /// Channels per sample.
    pub channels: ChannelCount,
    /// Samples per second (used for time conversions only).
    pub sample_rate: f64,
    /// Samples per summary block (power of two).
    pub block_samples: u32,
    /// A chunk is sealed after this many samples (multiple of `block_samples`).
    pub chunk_samples: u32,
    /// ... or after this many events (checked at block boundaries).
    pub chunk_max_events: u32,
    /// Memory cap in bytes for stored chunks; the oldest are dropped beyond it.
    pub max_bytes: usize,
}

impl StoreConfig {
    /// The default block and chunk sizes, with a cap of 1 GiB.
    pub fn new(channels: ChannelCount, sample_rate: f64) -> Self {
        Self {
            channels,
            sample_rate,
            block_samples: 4096,
            chunk_samples: 1 << 20,
            chunk_max_events: 1 << 15,
            max_bytes: 1 << 30,
        }
    }
}

#[derive(Clone, Debug)]
struct Block {
    /// Index in `Chunk::events` of the first event with `off >= block start`.
    ev_idx: u32,
    /// State of the sample just before the block (the chunk's `prev_state` for block 0).
    start_state: u32,
}

#[derive(Clone, Debug)]
struct Chunk {
    start: u64,
    len: u32,
    /// State of the sample before `start` (the first sample itself for the very first chunk).
    prev_state: u32,
    last_state: u32,
    events: Vec<Event>,
    blocks: Vec<Block>,
    /// `blocks.len() * channels` saturating transition counts.
    counts: Vec<u16>,
}

impl Chunk {
    fn new(start: u64, prev_state: u32) -> Self {
        Self {
            start,
            len: 0,
            prev_state,
            last_state: prev_state,
            events: Vec::new(),
            blocks: Vec::new(),
            counts: Vec::new(),
        }
    }

    fn bytes(&self) -> usize {
        self.events.len() * std::mem::size_of::<Event>()
            + self.blocks.len() * std::mem::size_of::<Block>()
            + self.counts.len() * 2
            + std::mem::size_of::<Self>()
    }

    fn end(&self) -> u64 {
        self.start + self.len as u64
    }

    fn block_events(&self, b: usize) -> &[Event] {
        let lo = self.blocks[b].ev_idx as usize;
        let hi = self
            .blocks
            .get(b + 1)
            .map_or(self.events.len(), |n| n.ev_idx as usize);
        &self.events[lo..hi]
    }

    /// State of the sample at chunk offset `off` (`off < len`).
    fn state_at(&self, off: u32, block: u32) -> u32 {
        let b = (off / block) as usize;
        let evs = self.block_events(b);
        // Last event at or before `off`; the events of a block are sorted by offset.
        match evs.partition_point(|e| e.off <= off) {
            0 => self.blocks[b].start_state,
            i => evs[i - 1].state,
        }
    }

    /// State just before offset `a` (`a <= len`).
    fn state_before(&self, a: u32, block: u32) -> u32 {
        if a == 0 {
            self.prev_state
        } else {
            self.state_at(a - 1, block)
        }
    }
}

/// One horizontal pixel column of a channel row.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Column {
    /// The column lies inside the captured range.
    pub valid: bool,
    /// Level just before the column (for the first column of a capture: its own level).
    pub before: bool,
    /// Level of the last sample of the column.
    pub after: bool,
    /// Number of toggles inside the column (saturating).
    pub edges: u32,
}

/// A store shared between the capture thread (writing) and its readers.
pub type SharedStore = Arc<RwLock<CaptureStore>>;

/// See the module documentation.
pub struct CaptureStore {
    cfg: StoreConfig,
    sealed: VecDeque<Chunk>,
    open: Chunk,
    started: bool,
    sealed_bytes: usize,
    generation: u64,
    /// Samples before this one are hidden (a capture cut to its pre-trigger window).
    floor: u64,
    trigger: Option<u64>,
    /// Scratch for the ingest of one block: events found and their xor with the previous state.
    scratch_ev: Vec<Event>,
    scratch_x: Vec<u32>,
}

impl CaptureStore {
    /// An empty store. Panics if the block or chunk size does not meet the rules of
    /// [`StoreConfig`].
    pub fn new(cfg: StoreConfig) -> Self {
        assert!(
            cfg.block_samples.is_power_of_two(),
            "block_samples must be a power of two"
        );
        assert!(
            cfg.chunk_samples.is_multiple_of(cfg.block_samples),
            "chunk_samples must be a multiple of the block size"
        );
        Self {
            cfg,
            sealed: VecDeque::new(),
            open: Chunk::new(0, 0),
            started: false,
            sealed_bytes: 0,
            generation: 0,
            floor: 0,
            trigger: None,
            scratch_ev: Vec::new(),
            scratch_x: Vec::new(),
        }
    }

    /// An empty store whose first sample will be sample number `start` (for loading a capture
    /// whose beginning was evicted).
    pub fn new_at(cfg: StoreConfig, start: u64) -> Self {
        let mut s = Self::new(cfg);
        s.open = Chunk::new(start, 0);
        s
    }

    /// Move an existing store into the shared form the application uses.
    pub fn into_shared(self) -> SharedStore {
        Arc::new(RwLock::new(self))
    }

    /// An empty store in the shared form.
    pub fn shared(cfg: StoreConfig) -> SharedStore {
        Arc::new(RwLock::new(Self::new(cfg)))
    }

    /// The parameters the store was made with.
    pub fn config(&self) -> &StoreConfig {
        &self.cfg
    }

    /// Channels per sample.
    pub fn channels(&self) -> u8 {
        self.cfg.channels.get()
    }

    /// Samples per second.
    pub fn sample_rate(&self) -> f64 {
        self.cfg.sample_rate
    }

    /// Increments on every `push_samples` (cheap change detection for the UI).
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// First sample still stored (older ones were evicted).
    pub fn first_sample(&self) -> u64 {
        self.floor
            .max(self.sealed.front().map_or(self.open.start, |c| c.start))
    }

    /// Sample at which the trigger fired, if this capture was triggered.
    pub fn trigger(&self) -> Option<u64> {
        self.trigger
    }

    /// Note the sample at which the trigger fired.
    pub fn set_trigger(&mut self, sample: Option<u64>) {
        self.trigger = sample;
        self.generation += 1;
    }

    /// Free whole chunks that end before `sample` (memory only: the visible range changes in steps
    /// of a chunk, so decoders are not restarted for every new sample while waiting for a trigger).
    pub fn trim_chunks_before(&mut self, sample: u64) {
        while self.sealed.front().is_some_and(|c| c.end() <= sample) {
            if let Some(c) = self.sealed.pop_front() {
                self.sealed_bytes -= c.bytes();
            }
        }
    }

    /// Hide everything before `sample` (exactly) and free what that makes unreachable.
    pub fn set_floor(&mut self, sample: u64) {
        self.floor = self.floor.max(sample.min(self.end()));
        let f = self.floor;
        self.trim_chunks_before(f);
        self.generation += 1;
    }

    /// One past the last captured sample.
    pub fn end(&self) -> u64 {
        self.open.end()
    }

    /// Whether no samples are stored.
    pub fn is_empty(&self) -> bool {
        self.end() == self.first_sample()
    }

    /// The stored samples.
    pub fn range(&self) -> Range<u64> {
        self.first_sample()..self.end()
    }

    /// Memory used by the chunks.
    pub fn memory_bytes(&self) -> usize {
        self.sealed_bytes + self.open.bytes()
    }

    /// Total events stored (statistics).
    pub fn event_count(&self) -> usize {
        self.sealed.iter().map(|c| c.events.len()).sum::<usize>() + self.open.events.len()
    }

    /// Time of `sample` in seconds, counted from sample 0.
    pub fn time_of(&self, sample: u64) -> f64 {
        sample as f64 / self.cfg.sample_rate
    }

    /// Append samples (already unpacked, bits above the channel count must be zero).
    pub fn push_samples(&mut self, mut s: &[u32]) {
        if s.is_empty() {
            return;
        }
        let block = self.cfg.block_samples;
        let nch = self.cfg.channels.get() as usize;
        let chunk_samples = self.cfg.chunk_samples;
        let chunk_max_events = self.cfg.chunk_max_events;
        if !self.started {
            self.started = true;
            self.open.prev_state = s[0];
            self.open.last_state = s[0];
        }
        while !s.is_empty() {
            let c = &mut self.open;
            let off = c.len;
            if off.is_multiple_of(block) {
                c.blocks.push(Block {
                    ev_idx: c.events.len() as u32,
                    start_state: c.last_state,
                });
                c.counts.resize(c.counts.len() + nch, 0);
            }
            let room = (block - off % block) as usize;
            let n = room.min(s.len());
            let base = (c.blocks.len() - 1) * nch;
            // Pass 1: the events of this part of the block, branch-free (every sample is written
            // at the next free slot, which advances only when the state changed), with runs of
            // unchanged samples skipped a group at a time.
            if self.scratch_ev.len() < block as usize {
                self.scratch_ev
                    .resize(block as usize, Event { off: 0, state: 0 });
                self.scratch_x.resize(block as usize, 0);
            }
            let (ev, xs) = (&mut self.scratch_ev, &mut self.scratch_x);
            let mut last = c.last_state;
            let mut k = 0usize;
            for (g, grp) in s[..n].chunks(16).enumerate() {
                // Unchanged group: a reduction (vectorizes), not an early-exit comparison.
                if grp.iter().fold(0u32, |a, &w| a | (w ^ last)) == 0 {
                    continue;
                }
                let at = off + (g * 16) as u32;
                for (j, &w) in grp.iter().enumerate() {
                    let x = w ^ last;
                    ev[k] = Event {
                        off: at + j as u32,
                        state: w,
                    };
                    xs[k] = x;
                    k += (x != 0) as usize;
                    last = w;
                }
            }
            c.events.extend_from_slice(&ev[..k]);
            // Pass 2: per-channel transition counts from the xors.
            let acc = &mut c.counts[base..base + nch];
            match nch {
                0..=8 => count_transitions::<8>(&xs[..k], acc),
                9..=16 => count_transitions::<16>(&xs[..k], acc),
                _ => count_transitions::<32>(&xs[..k], acc),
            }
            c.last_state = last;
            c.len += n as u32;
            s = &s[n..];
            if c.len.is_multiple_of(block)
                && (c.len >= chunk_samples || c.events.len() as u32 >= chunk_max_events)
            {
                self.seal();
            }
        }
        self.evict();
        self.generation += 1;
    }

    fn seal(&mut self) {
        let next = Chunk::new(self.open.end(), self.open.last_state);
        let old = std::mem::replace(&mut self.open, next);
        self.sealed_bytes += old.bytes();
        self.sealed.push_back(old);
    }

    fn evict(&mut self) {
        while self.sealed_bytes + self.open.bytes() > self.cfg.max_bytes {
            match self.sealed.pop_front() {
                Some(c) => self.sealed_bytes -= c.bytes(),
                None => break,
            }
        }
    }

    fn chunk_at(&self, s: u64) -> Option<&Chunk> {
        if s >= self.end() || s < self.first_sample() {
            return None;
        }
        if s >= self.open.start {
            return Some(&self.open);
        }
        let i = self.sealed.partition_point(|c| c.end() <= s);
        self.sealed.get(i)
    }

    fn chunks(&self) -> impl Iterator<Item = &Chunk> {
        self.sealed
            .iter()
            .chain(std::iter::once(&self.open))
            .filter(|c| c.len > 0)
    }

    /// State word of sample `s`, `None` if it is not (or no longer) stored.
    pub fn state_at(&self, s: u64) -> Option<u32> {
        let c = self.chunk_at(s)?;
        Some(c.state_at((s - c.start) as u32, self.cfg.block_samples))
    }

    /// Level of channel `ch` at sample `s`.
    pub fn level(&self, ch: u8, s: u64) -> Option<bool> {
        self.state_at(s).map(|w| (w >> ch) & 1 != 0)
    }

    /// Visit the toggles of `ch` in `range` in order: `f(p, new_level)`.
    pub fn toggles(&self, ch: u8, range: Range<u64>, mut f: impl FnMut(u64, bool)) {
        let first = self.first_sample();
        let block = self.cfg.block_samples;
        for c in self.chunks() {
            let lo = range.start.max(c.start).max(first + 1);
            let hi = range.end.min(c.end());
            if lo >= hi {
                continue;
            }
            let (a, b) = ((lo - c.start) as u32, (hi - c.start) as u32);
            Self::walk(c, ch, a, b, block, &mut |off, lvl| {
                f(c.start + off as u64, lvl)
            });
        }
    }

    /// Walk toggles of `ch` at chunk offsets in `[a, b)`.
    fn walk(c: &Chunk, ch: u8, a: u32, b: u32, block: u32, f: &mut dyn FnMut(u32, bool)) {
        if a >= b {
            return;
        }
        let mut cur = c.state_before(a, block);
        let bi = (a / block) as usize;
        let lo = c.blocks[bi].ev_idx as usize;
        let hi = c
            .blocks
            .get(bi + 1)
            .map_or(c.events.len(), |n| n.ev_idx as usize);
        // First event at or after `a` (binary search inside the block, no scan from its start).
        let start = lo + c.events[lo..hi].partition_point(|e| e.off < a);
        for e in &c.events[start..] {
            if e.off >= b {
                break;
            }
            if ((e.state ^ cur) >> ch) & 1 != 0 {
                f(e.off, (e.state >> ch) & 1 != 0);
            }
            cur = e.state;
        }
    }

    /// Visit the changes that affect any channel in `mask` within `range`, in order:
    /// `f(p, state)` with `p` the first sample of the new state and `state` its full word.
    /// Cost is proportional to the stored events in the range, not to its length. The first stored
    /// sample is never reported (it has no predecessor); use [`Self::state_at`] for the start state.
    pub fn changes(&self, mask: u32, range: Range<u64>, mut f: impl FnMut(u64, u32)) {
        let first = self.first_sample();
        let block = self.cfg.block_samples;
        for c in self.chunks() {
            let lo = range.start.max(c.start).max(first + 1);
            let hi = range.end.min(c.end());
            if lo >= hi {
                continue;
            }
            let (a, b) = ((lo - c.start) as u32, (hi - c.start) as u32);
            let mut cur = c.state_before(a, block);
            let bi = (a / block) as usize;
            let (blo, bhi) = (
                c.blocks[bi].ev_idx as usize,
                c.blocks
                    .get(bi + 1)
                    .map_or(c.events.len(), |n| n.ev_idx as usize),
            );
            let start = blo + c.events[blo..bhi].partition_point(|e| e.off < a);
            for e in &c.events[start..] {
                if e.off >= b {
                    break;
                }
                if (e.state ^ cur) & mask != 0 {
                    f(c.start + e.off as u64, e.state);
                }
                cur = e.state;
            }
        }
    }

    /// The first change of any channel in `mask` after sample `s` (a toggle at `p > s`), for jumping
    /// from edge to edge. Cost is proportional to the stored events skipped, not to the samples.
    pub fn next_change(&self, mask: u32, s: u64) -> Option<u64> {
        let from = s.saturating_add(1).max(self.first_sample() + 1);
        for c in self.chunks() {
            if c.end() <= from {
                continue;
            }
            let off = from.saturating_sub(c.start) as u32;
            let i0 = c.events.partition_point(|e| e.off < off);
            for i in i0..c.events.len() {
                let prev = if i > 0 {
                    c.events[i - 1].state
                } else {
                    c.prev_state
                };
                if (c.events[i].state ^ prev) & mask != 0 {
                    return Some(c.start + c.events[i].off as u64);
                }
            }
        }
        None
    }

    /// The last change of any channel in `mask` before sample `s` (a toggle at `p < s`).
    pub fn prev_change(&self, mask: u32, s: u64) -> Option<u64> {
        let first = self.first_sample();
        let chunks: Vec<&Chunk> = self.chunks().collect();
        for c in chunks.into_iter().rev() {
            if c.start >= s {
                continue;
            }
            let off = (s.min(c.end()) - c.start) as u32;
            let end = c.events.partition_point(|e| e.off < off);
            for i in (0..end).rev() {
                let p = c.start + c.events[i].off as u64;
                if p <= first {
                    return None;
                }
                let prev = if i > 0 {
                    c.events[i - 1].state
                } else {
                    c.prev_state
                };
                if (c.events[i].state ^ prev) & mask != 0 {
                    return Some(p);
                }
            }
        }
        None
    }

    /// Number of toggles of `ch` in `range`; whole blocks are answered from the summaries.
    pub fn count_toggles(&self, ch: u8, range: Range<u64>) -> u64 {
        let first = self.first_sample();
        let block = self.cfg.block_samples;
        let nch = self.cfg.channels.get() as usize;
        let mut total = 0u64;
        for c in self.chunks() {
            let lo = range.start.max(c.start).max(first + 1);
            let hi = range.end.min(c.end());
            if lo >= hi {
                continue;
            }
            let (a, b) = ((lo - c.start) as u32, (hi - c.start) as u32);
            // First and last whole block inside [a, b) that is complete in the chunk.
            let first_full = a.div_ceil(block);
            let last_full = (b / block).min(c.len / block); // exclusive
            if first_full >= last_full {
                Self::walk(c, ch, a, b, block, &mut |_, _| total += 1);
                continue;
            }
            Self::walk(c, ch, a, first_full * block, block, &mut |_, _| total += 1);
            for bi in first_full as usize..last_full as usize {
                total += c.counts[bi * nch + ch as usize] as u64;
            }
            Self::walk(c, ch, last_full * block, b, block, &mut |_, _| total += 1);
        }
        total
    }

    /// Exact toggles of `ch` in `range`, `None` if there are more than `max`.
    pub fn edges(&self, ch: u8, range: Range<u64>, max: usize) -> Option<Vec<(u64, bool)>> {
        if self.count_toggles(ch, range.clone()) > max as u64 {
            return None;
        }
        let mut v = Vec::new();
        self.toggles(ch, range, |p, l| v.push((p, l)));
        Some(v)
    }

    /// Fill `n` pixel columns for channel `ch`. Column `i` covers samples
    /// `[start + i*spp, start + (i+1)*spp)` (`spp` is clamped to >= 1). `start` may be negative.
    pub fn columns(&self, ch: u8, start: i64, spp: f64, n: usize, out: &mut Vec<Column>) {
        out.clear();
        let spp = spp.max(1.0);
        let (first, end) = (self.first_sample() as i64, self.end() as i64);
        for i in 0..n {
            let s0 = start + (i as f64 * spp).floor() as i64;
            let s1 = (start + ((i + 1) as f64 * spp).floor() as i64).max(s0 + 1);
            let (c0, c1) = (s0.max(first), s1.min(end));
            if c0 >= c1 {
                out.push(Column::default());
                continue;
            }
            let (c0, c1) = (c0 as u64, c1 as u64);
            let before = if c0 > first as u64 {
                self.level(ch, c0 - 1)
            } else {
                self.level(ch, c0)
            };
            let after = self.level(ch, c1 - 1);
            out.push(Column {
                valid: true,
                before: before.unwrap_or(false),
                after: after.unwrap_or(false),
                edges: self.count_toggles(ch, c0..c1).min(u32::MAX as u64) as u32,
            });
        }
    }
}

/// Add, per channel, the number of `xs` words having that bit set to `acc` (saturating). `N` is a
/// lane count of at least `acc.len()`, fixed so that the loop vectorizes.
fn count_transitions<const N: usize>(xs: &[u32], acc: &mut [u16]) {
    let mut lanes = [0u16; N];
    for (l, a) in lanes.iter_mut().zip(acc.iter()) {
        *l = *a;
    }
    for &x in xs {
        for (ch, l) in lanes.iter_mut().enumerate() {
            *l = l.saturating_add(((x >> ch) & 1) as u16);
        }
    }
    for (a, l) in acc.iter_mut().zip(lanes.iter()) {
        *a = *l;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn cfg(block: u32, chunk: u32, max_events: u32) -> StoreConfig {
        let mut c = StoreConfig::new(ChannelCount::new(8).unwrap(), 1e6);
        c.block_samples = block;
        c.chunk_samples = chunk;
        c.chunk_max_events = max_events;
        c
    }

    fn naive_toggles(v: &[u32], ch: u8, r: Range<u64>, first: u64) -> Vec<(u64, bool)> {
        let mut out = Vec::new();
        for p in r.start.max(first + 1)..r.end.min(v.len() as u64) {
            let (a, b) = (v[p as usize - 1], v[p as usize]);
            if ((a ^ b) >> ch) & 1 != 0 {
                out.push((p, (b >> ch) & 1 != 0));
            }
        }
        out
    }

    /// Samples made of runs, so that both sparse and dense stretches occur.
    fn samples() -> impl Strategy<Value = Vec<u32>> {
        prop::collection::vec((0u32..256, 1usize..40), 1..60).prop_map(|runs| {
            runs.into_iter()
                .flat_map(|(v, n)| std::iter::repeat_n(v, n))
                .collect()
        })
    }

    fn fill(store: &mut CaptureStore, v: &[u32], splits: &[usize]) {
        let mut pos = 0;
        for &s in splits {
            let n = (s % 97 + 1).min(v.len() - pos);
            store.push_samples(&v[pos..pos + n]);
            pos += n;
        }
        store.push_samples(&v[pos..]);
    }

    proptest! {
        #[test]
        fn changes_match_a_naive_scan(v in samples(), mask in 1u32..256, a in 0u64..2500, len in 0u64..2500,
                                       splits in prop::collection::vec(0usize..1000, 0..10)) {
            let mut s = CaptureStore::new(cfg(8, 32, 8));
            fill(&mut s, &v, &splits);
            let r = a..a + len;
            let mut got = Vec::new();
            s.changes(mask, r.clone(), |p, st| got.push((p, st)));
            let mut want = Vec::new();
            for p in r.start.max(1)..r.end.min(v.len() as u64) {
                if (v[p as usize - 1] ^ v[p as usize]) & mask != 0 {
                    want.push((p, v[p as usize]));
                }
            }
            prop_assert_eq!(got, want);
        }

        #[test]
        fn next_and_prev_change_match_a_naive_scan(v in samples(), mask in 1u32..256, at in 0u64..2500,
                                                   splits in prop::collection::vec(0usize..1000, 0..10)) {
            let mut s = CaptureStore::new(cfg(8, 32, 8));
            fill(&mut s, &v, &splits);
            let toggles: Vec<u64> = (1..v.len() as u64)
                .filter(|&p| (v[p as usize - 1] ^ v[p as usize]) & mask != 0)
                .collect();
            prop_assert_eq!(s.next_change(mask, at), toggles.iter().copied().find(|&p| p > at));
            prop_assert_eq!(s.prev_change(mask, at), toggles.iter().copied().rev().find(|&p| p < at));
        }
    }

    #[test]
    fn empty_and_single_sample() {
        let mut s = CaptureStore::new(cfg(8, 32, 16));
        assert!(s.is_empty());
        assert_eq!(s.state_at(0), None);
        s.push_samples(&[5]);
        assert_eq!(s.range(), 0..1);
        assert_eq!(s.state_at(0), Some(5));
        assert_eq!(s.count_toggles(0, 0..1), 0);
    }

    #[test]
    fn constant_signal_has_no_events() {
        let mut s = CaptureStore::new(cfg(8, 32, 16));
        s.push_samples(&vec![0xa5; 1000]);
        assert_eq!(s.event_count(), 0);
        assert_eq!(s.state_at(999), Some(0xa5));
        for ch in 0..8 {
            assert_eq!(s.count_toggles(ch, 0..1000), 0);
        }
    }

    #[test]
    fn simple_square_wave() {
        let mut s = CaptureStore::new(cfg(8, 32, 16));
        let v: Vec<u32> = (0..100).map(|i| (i / 5) as u32 & 1).collect();
        s.push_samples(&v);
        let e = s.edges(0, 0..100, 100).unwrap();
        assert_eq!(e.len(), 19);
        assert_eq!(e[0], (5, true));
        assert_eq!(e[1], (10, false));
        assert_eq!(s.count_toggles(0, 0..100), 19);
        assert_eq!(s.count_toggles(1, 0..100), 0);
        assert!(s.edges(0, 0..100, 18).is_none());
    }

    #[test]
    fn columns_cover_range_and_report_edges() {
        let mut s = CaptureStore::new(cfg(8, 32, 16));
        let v: Vec<u32> = (0..64).map(|i| (i / 4) as u32 & 1).collect();
        s.push_samples(&v);
        let mut cols = Vec::new();
        s.columns(0, -8, 8.0, 10, &mut cols);
        assert!(!cols[0].valid);
        assert!(cols[1].valid);
        assert_eq!(cols[1].edges, 1); // samples 0..8: toggle at 4
        assert!(!cols[9].valid); // 64..72 is past the end
        assert_eq!(cols.len(), 10);
    }

    #[test]
    fn eviction_keeps_a_consistent_suffix() {
        let mut c = cfg(8, 32, 16);
        c.max_bytes = 4096;
        let mut s = CaptureStore::new(c);
        let v: Vec<u32> = (0..20_000u32).map(|i| (i / 3) & 0xff).collect();
        for part in v.chunks(1000) {
            s.push_samples(part);
        }
        assert!(s.first_sample() > 0);
        assert!(s.memory_bytes() <= 4096 + 2048);
        let first = s.first_sample();
        for p in [first, first + 1, (first + s.end()) / 2, s.end() - 1] {
            assert_eq!(s.state_at(p), Some(v[p as usize]), "p={p}");
        }
        assert_eq!(
            s.state_at(first.saturating_sub(1)),
            if first == 0 { Some(v[0]) } else { None }
        );
    }

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(200))]

        #[test]
        fn state_at_matches_naive(v in samples(), splits in prop::collection::vec(0usize..1000, 0..10)) {
            let mut s = CaptureStore::new(cfg(8, 32, 16));
            fill(&mut s, &v, &splits);
            prop_assert_eq!(s.range(), 0..v.len() as u64);
            for (i, &w) in v.iter().enumerate() {
                prop_assert_eq!(s.state_at(i as u64), Some(w), "i={}", i);
            }
            prop_assert_eq!(s.state_at(v.len() as u64), None);
        }

        #[test]
        fn toggles_and_counts_match_naive(
            v in samples(),
            splits in prop::collection::vec(0usize..1000, 0..10),
            ch in 0u8..8,
            a in 0u64..2400,
            len in 0u64..2400,
        ) {
            let mut s = CaptureStore::new(cfg(8, 32, 16));
            fill(&mut s, &v, &splits);
            let r = a..a + len;
            let want = naive_toggles(&v, ch, r.clone(), 0);
            let mut got = Vec::new();
            s.toggles(ch, r.clone(), |p, l| got.push((p, l)));
            prop_assert_eq!(&got, &want);
            prop_assert_eq!(s.count_toggles(ch, r), want.len() as u64);
        }

        #[test]
        fn columns_match_naive(
            v in samples(),
            ch in 0u8..8,
            start in -40i64..200,
            spp in prop::sample::select(vec![1.0f64, 2.5, 8.0, 33.0]),
        ) {
            let mut s = CaptureStore::new(cfg(8, 32, 16));
            s.push_samples(&v);
            let mut cols = Vec::new();
            s.columns(ch, start, spp, 30, &mut cols);
            for (i, col) in cols.iter().enumerate() {
                let s0 = start + (i as f64 * spp).floor() as i64;
                let s1 = (start + ((i + 1) as f64 * spp).floor() as i64).max(s0 + 1);
                let (c0, c1) = (s0.max(0), s1.min(v.len() as i64));
                if c0 >= c1 {
                    prop_assert!(!col.valid);
                    continue;
                }
                prop_assert!(col.valid);
                let before = if c0 > 0 { v[c0 as usize - 1] } else { v[0] };
                prop_assert_eq!(col.before, (before >> ch) & 1 != 0);
                prop_assert_eq!(col.after, (v[c1 as usize - 1] >> ch) & 1 != 0);
                let want = naive_toggles(&v, ch, c0 as u64..c1 as u64, 0).len() as u32;
                prop_assert_eq!(col.edges, want);
            }
        }
    }

    #[test]
    fn a_floor_hides_the_samples_before_it() {
        let mut cfg = StoreConfig::new(ChannelCount::new(4).unwrap(), 1e6);
        cfg.block_samples = 16;
        cfg.chunk_samples = 64;
        let mut s = CaptureStore::new(cfg);
        let words: Vec<u32> = (0..1000u32).map(|i| (i / 3) & 15).collect();
        s.push_samples(&words);
        let before = s.memory_bytes();
        s.set_floor(333);
        assert_eq!(s.first_sample(), 333);
        assert_eq!(s.range(), 333..1000);
        assert_eq!(s.state_at(332), None);
        assert_eq!(s.state_at(333), Some(words[333]));
        assert!(
            s.memory_bytes() < before,
            "chunks before the floor are freed"
        );
        // Queries over the visible range agree with the data, the first sample is not an event.
        let mut got = Vec::new();
        s.changes(15, s.range(), |p, st| got.push((p, st)));
        let want: Vec<(u64, u32)> = (334..1000u64)
            .filter(|&p| words[p as usize] != words[p as usize - 1])
            .map(|p| (p, words[p as usize]))
            .collect();
        assert!(got.iter().all(|e| want.contains(e)), "{got:?}");
        // The floor never moves back, and never past the end.
        s.set_floor(10);
        assert_eq!(s.first_sample(), 333);
        s.set_floor(5000);
        assert_eq!(s.first_sample(), 1000);
        assert!(s.is_empty());
    }
}
