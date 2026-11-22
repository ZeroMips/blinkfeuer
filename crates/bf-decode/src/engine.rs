//! Feeding decoders from the capture store: a synchronous [`Runner`] and a background [`Engine`]
//! that decodes live data in short slices so it never holds the store's read lock for long (the
//! capture thread needs the write lock for every USB buffer).

use crate::{AnnStore, Annotation, Decoder};
use bf_core::{CaptureStore, SharedStore};
use parking_lot::{Mutex, RwLock};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

/// Samples decoded per slice (bounds how long the store is locked: a dense slice is a few ms).
pub const SLICE_SAMPLES: u64 = 1 << 21;

/// A decoder together with its progress through one capture.
pub struct Runner {
    decoder: Box<dyn Decoder>,
    decoded_to: u64,
    primed: bool,
}

impl Runner {
    pub fn new(decoder: Box<dyn Decoder>) -> Self {
        Self { decoder, decoded_to: 0, primed: false }
    }

    pub fn decoder(&self) -> &dyn Decoder {
        self.decoder.as_ref()
    }

    /// All samples before this one are decoded.
    pub fn decoded_to(&self) -> u64 {
        self.decoded_to
    }

    /// Start over (a new capture was assigned).
    pub fn restart(&mut self) {
        self.primed = false;
        self.decoded_to = 0;
    }

    /// Decode up to `budget` further samples. `None`: nothing new. `Some(restarted)`: progress was
    /// made; when `restarted` the decoder began again (new capture, or its data was evicted) and
    /// earlier annotations are stale.
    pub fn pump(&mut self, store: &CaptureStore, budget: u64, out: &mut Vec<Annotation>) -> Option<bool> {
        let (first, end) = (store.first_sample(), store.end());
        if end == first {
            return None;
        }
        let mut restarted = false;
        if !self.primed || self.decoded_to < first {
            self.decoder.reset(store.sample_rate());
            self.primed = true;
            self.decoded_to = first;
            restarted = true;
            // The first sample is the initial state, not a change.
            self.decoder.change(first, store.state_at(first).unwrap_or(0), out);
        }
        if self.decoded_to >= end {
            return restarted.then_some(true);
        }
        let to = end.min(self.decoded_to.saturating_add(budget.max(1)));
        let dec = &mut self.decoder;
        store.changes(dec.mask(), self.decoded_to..to, |p, state| dec.change(p, state, out));
        dec.advance(to, out);
        self.decoded_to = to;
        Some(restarted)
    }
}

/// Decode a whole capture in one go.
pub fn decode_all(decoder: Box<dyn Decoder>, store: &CaptureStore) -> Vec<Annotation> {
    let mut runner = Runner::new(decoder);
    let mut out = Vec::new();
    while runner.pump(store, u64::MAX, &mut out).is_some() {}
    out
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct DecoderId(pub u32);

/// What the UI keeps for a decoder: its annotations and how far it has come.
#[derive(Clone)]
pub struct DecoderHandle {
    pub id: DecoderId,
    pub name: &'static str,
    pub lanes: &'static [&'static str],
    pub lane_used: Vec<bool>,
    pub anns: Arc<RwLock<AnnStore>>,
    progress: Arc<AtomicU64>,
}

impl DecoderHandle {
    /// Samples decoded so far.
    pub fn decoded_to(&self) -> u64 {
        self.progress.load(Ordering::Relaxed)
    }
}

struct Slot {
    id: DecoderId,
    runner: Runner,
    anns: Arc<RwLock<AnnStore>>,
    progress: Arc<AtomicU64>,
}

struct Shared {
    store: Mutex<Option<SharedStore>>,
    slots: Mutex<Vec<Slot>>,
    stop: AtomicBool,
    next_id: AtomicU32,
}

/// Runs the registered decoders over a (growing) capture on a background thread.
pub struct Engine {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

impl Engine {
    pub fn new() -> Self {
        let shared = Arc::new(Shared {
            store: Mutex::new(None),
            slots: Mutex::new(Vec::new()),
            stop: AtomicBool::new(false),
            next_id: AtomicU32::new(1),
        });
        let s2 = shared.clone();
        let thread = std::thread::Builder::new().name("decode".into()).spawn(move || worker(s2)).ok();
        Self { shared, thread }
    }

    /// Decode this capture from now on (all decoders start over).
    pub fn set_store(&self, store: SharedStore) {
        let mut slots = self.shared.slots.lock();
        *self.shared.store.lock() = Some(store);
        for s in slots.iter_mut() {
            s.runner.restart();
            s.progress.store(0, Ordering::Relaxed);
        }
    }

    pub fn add(&self, decoder: Box<dyn Decoder>) -> DecoderHandle {
        let id = DecoderId(self.shared.next_id.fetch_add(1, Ordering::Relaxed));
        let lanes = decoder.lanes();
        let lane_used = (0..lanes.len()).map(|l| decoder.lane_used(l)).collect();
        let anns = Arc::new(RwLock::new(AnnStore::new(lanes.len())));
        let progress = Arc::new(AtomicU64::new(0));
        let handle = DecoderHandle { id, name: decoder.name(), lanes, lane_used, anns: anns.clone(), progress: progress.clone() };
        self.shared.slots.lock().push(Slot { id, runner: Runner::new(decoder), anns, progress });
        handle
    }

    pub fn remove(&self, id: DecoderId) {
        self.shared.slots.lock().retain(|s| s.id != id);
    }

    /// True while some decoder has not caught up with the capture.
    pub fn busy(&self) -> bool {
        let Some(store) = self.shared.store.lock().clone() else { return false };
        let end = store.read().end();
        self.shared.slots.lock().iter().any(|s| s.progress.load(Ordering::Relaxed) < end)
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn worker(shared: Arc<Shared>) {
    let mut out = Vec::new();
    while !shared.stop.load(Ordering::Relaxed) {
        let mut worked = false;
        if let Some(store) = shared.store.lock().clone() {
            let mut slots = shared.slots.lock();
            for slot in slots.iter_mut() {
                out.clear();
                let guard = store.read();
                let result = slot.runner.pump(&guard, SLICE_SAMPLES, &mut out);
                let first = guard.first_sample();
                drop(guard);
                if let Some(restarted) = result {
                    worked = true;
                    let mut anns = slot.anns.write();
                    if restarted {
                        anns.clear();
                    }
                    for a in out.drain(..) {
                        anns.push(a);
                    }
                    anns.drop_before(first);
                    slot.progress.store(slot.runner.decoded_to(), Ordering::Relaxed);
                }
            }
        }
        if !worked {
            std::thread::sleep(Duration::from_millis(8));
        }
    }
}
