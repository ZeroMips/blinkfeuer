//! Capture session: a thread that reads a [`Stream`], unpacks the wire bytes and appends them to the
//! shared capture store.

use crate::{CaptureConfig, Device, DeviceError, ReadResult, StreamReport};
use bf_core::{EventTrigger, SharedStore, Trigger, Unpacker};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

/// Where a triggered capture is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// No trigger: samples go straight into the store.
    Free,
    /// Collecting the pre-trigger window; the trigger is not armed yet.
    Filling,
    /// Armed: waiting for the trigger condition.
    Waiting,
    /// The trigger has fired; the post-trigger part is being recorded.
    Triggered,
}

/// What fires the trigger.
pub enum TriggerKind {
    /// Levels and edges of channels, checked on every sample.
    Pattern(Trigger),
    /// A state machine over the channel changes (a protocol decoder looking for something).
    Event(Box<dyn EventTrigger>),
}

/// A trigger and the window around it, for [`Session::start_triggered`].
pub struct TriggerPlan {
    pub kind: TriggerKind,
    /// Samples kept before the trigger sample (the trigger is armed once they are there).
    pub pre: u64,
    /// Samples recorded from the trigger sample on, then the capture ends; `None` records until stopped.
    pub post: Option<u64>,
}

impl TriggerPlan {
    pub fn pattern(trigger: Trigger, pre: u64, post: Option<u64>) -> Self {
        Self {
            kind: TriggerKind::Pattern(trigger),
            pre,
            post,
        }
    }
}

#[derive(Default)]
pub struct SessionStats {
    pub bytes: AtomicU64,
    pub samples: AtomicU64,
    pub running: AtomicBool,
    phase: AtomicU8,
    force: AtomicBool,
    /// The capture ended because the memory limit was reached (after the trigger, data is never
    /// dropped to make room: the pre-trigger part and the trigger itself must stay).
    pub memory_full: AtomicBool,
}

impl SessionStats {
    pub fn phase(&self) -> Phase {
        match self.phase.load(Ordering::Relaxed) {
            1 => Phase::Filling,
            2 => Phase::Waiting,
            3 => Phase::Triggered,
            _ => Phase::Free,
        }
    }

    fn set_phase(&self, p: Phase) {
        self.phase.store(p as u8, Ordering::Relaxed);
    }
}

/// The capture thread's trigger logic: feeds samples to the store, holds back nothing, but keeps
/// only the pre-trigger window while it waits, and cuts the capture at the end of the post part.
struct Gate {
    plan: TriggerPlan,
    rate: f64,
    prev: Option<u32>,
    triggered: Option<u64>,
    /// Longest buffer seen: a protocol trigger is detected a little after its trigger sample, so the
    /// window kept while waiting is a few buffers longer than the pre-trigger part.
    slack: u64,
}

enum Flow {
    Continue,
    /// The capture is complete.
    Done,
}

impl Gate {
    fn new(plan: TriggerPlan, rate: f64) -> Self {
        Self {
            plan,
            rate,
            prev: None,
            triggered: None,
            slack: 0,
        }
    }

    /// The trigger sample, if the trigger fires in `words` (which start at sample `base`).
    fn detect(&mut self, base: u64, words: &[u32], stats: &SessionStats) -> Option<u64> {
        if stats.force.swap(false, Ordering::SeqCst) {
            return Some(base);
        }
        let pre = self.plan.pre;
        match &mut self.plan.kind {
            TriggerKind::Pattern(trig) => {
                // Samples before `pre` cannot be trigger points: the window before them would be short.
                let from = (pre.saturating_sub(base) as usize).min(words.len());
                // The very first sample of a capture has no predecessor, so it cannot be an edge.
                let prev = if from > 0 {
                    words[from - 1]
                } else {
                    self.prev.unwrap_or(words[0])
                };
                trig.find(prev, &words[from..])
                    .map(|i| base + (i + from) as u64)
            }
            TriggerKind::Event(ev) => {
                let mask = ev.mask();
                if self.prev.is_none() {
                    ev.reset(self.rate);
                    ev.change(base, words[0]); // the initial state, not a change
                }
                // The decoder follows the whole stream, but a match before `pre` does not count.
                let mut prev = self.prev.unwrap_or(words[0]);
                for (i, &w) in words.iter().enumerate() {
                    if (w ^ prev) & mask != 0 {
                        if let Some(t) = ev.change(base + i as u64, w).filter(|t| *t >= pre) {
                            return Some(t);
                        }
                    }
                    prev = w;
                }
                ev.advance(base + words.len() as u64).filter(|t| *t >= pre)
            }
        }
    }

    fn feed(&mut self, store: &SharedStore, words: &[u32], stats: &SessionStats) -> Flow {
        let Some(&last) = words.last() else {
            return Flow::Continue;
        };
        // Only this thread writes, so the end of the store is the index of `words[0]`.
        let base = store.read().end();
        self.slack = self.slack.max(4 * words.len() as u64);
        let hit = if self.triggered.is_none() {
            self.detect(base, words, stats)
        } else {
            None
        };
        let mut st = store.write();
        let mut take = words.len();
        match hit {
            Some(t) => {
                self.triggered = Some(t);
                st.set_trigger(Some(t));
                stats.set_phase(Phase::Triggered);
            }
            None if self.triggered.is_none() => {
                stats.set_phase(if base + words.len() as u64 >= self.plan.pre {
                    Phase::Waiting
                } else {
                    Phase::Filling
                });
            }
            None => {}
        }
        if let (Some(t), Some(post)) = (self.triggered, self.plan.post) {
            take = take.min((t + post).saturating_sub(base) as usize);
        }
        // Past the trigger nothing may be evicted to make room. A sample costs at most one 8-byte
        // event, so only push what is sure to fit under 95 % of the limit.
        let mut full = false;
        if self.triggered.is_some() {
            let room = (0.95 * st.config().max_bytes as f64 - st.memory_bytes() as f64).max(0.0);
            let fits = (room / 8.5) as usize;
            if fits < take {
                take = fits;
                full = true;
            }
        }
        st.push_samples(&words[..take]);
        if full {
            stats.memory_full.store(true, Ordering::SeqCst);
            return Flow::Done;
        }
        match self.triggered {
            Some(t) => {
                // Cut the capture to its pre-trigger window (once; later calls change nothing).
                st.set_floor(t.saturating_sub(self.plan.pre));
                if let Some(post) = self.plan.post {
                    if st.end() >= t + post {
                        return Flow::Done;
                    }
                }
                // Past the trigger nothing may be evicted to make room: stop a little before the limit.
                if st.memory_bytes() as f64 > 0.95 * st.config().max_bytes as f64 {
                    stats.memory_full.store(true, Ordering::SeqCst);
                    return Flow::Done;
                }
            }
            None => {
                let end = st.end();
                let lookback = match &self.plan.kind {
                    TriggerKind::Event(ev) => ev.lookback(),
                    TriggerKind::Pattern(_) => 0,
                };
                st.trim_chunks_before(end.saturating_sub(self.plan.pre + self.slack + lookback));
            }
        }
        self.prev = Some(last);
        Flow::Continue
    }
}

#[derive(Debug)]
pub struct SessionResult {
    pub samples: u64,
    pub bytes: u64,
    pub elapsed: Duration,
    pub report: StreamReport,
    pub error: Option<DeviceError>,
}

pub struct Session {
    stop: Arc<AtomicBool>,
    stats: Arc<SessionStats>,
    handle: Option<JoinHandle<SessionResult>>,
}

impl Session {
    /// Start capturing into `store` (the store must have been created for the same channel count).
    pub fn start(
        device: &mut dyn Device,
        cfg: &CaptureConfig,
        store: SharedStore,
    ) -> Result<Session, DeviceError> {
        Self::start_triggered(device, cfg, store, None)
    }

    /// Like [`Session::start`], with a trigger: the capture waits for the trigger condition, keeps
    /// the `pre` samples before it and ends `post` samples after it. The device must be started
    /// without a sample limit then (the end is decided here).
    pub fn start_triggered(
        device: &mut dyn Device,
        cfg: &CaptureConfig,
        store: SharedStore,
        plan: Option<TriggerPlan>,
    ) -> Result<Session, DeviceError> {
        let mut stream = device.start(cfg)?;
        let stop = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(SessionStats::default());
        stats.running.store(true, Ordering::SeqCst);
        if plan.is_some() {
            stats.set_phase(Phase::Filling);
        }
        let (stop2, stats2, channels, rate) =
            (stop.clone(), stats.clone(), cfg.channels, cfg.sample_rate);
        let handle = std::thread::Builder::new()
            .name("capture".into())
            .spawn(move || {
                let t0 = Instant::now();
                let mut unpacker = Unpacker::new(channels);
                let mut words = Vec::new();
                let mut error = None;
                let mut gate = plan.map(|p| Gate::new(p, rate));
                while !stop2.load(Ordering::Relaxed) {
                    match stream.read(Duration::from_millis(50)) {
                        Ok(ReadResult::Data(buf)) => {
                            stats2.bytes.fetch_add(buf.len() as u64, Ordering::Relaxed);
                            unpacker.unpack(&buf, &mut words);
                            stats2
                                .samples
                                .fetch_add(words.len() as u64, Ordering::Relaxed);
                            let done = match &mut gate {
                                Some(g) => matches!(g.feed(&store, &words, &stats2), Flow::Done),
                                None => {
                                    store.write().push_samples(&words);
                                    false
                                }
                            };
                            words.clear();
                            if done {
                                break;
                            }
                        }
                        Ok(ReadResult::Timeout) => {}
                        Ok(ReadResult::Ended) => break,
                        Err(e) => {
                            error = Some(e);
                            break;
                        }
                    }
                }
                let report = stream.stop().unwrap_or_default();
                stats2.running.store(false, Ordering::SeqCst);
                SessionResult {
                    samples: stats2.samples.load(Ordering::Relaxed),
                    bytes: stats2.bytes.load(Ordering::Relaxed),
                    elapsed: t0.elapsed(),
                    report,
                    error,
                }
            })
            .map_err(|e| DeviceError::Other(e.to_string()))?;
        Ok(Session {
            stop,
            stats,
            handle: Some(handle),
        })
    }

    pub fn stats(&self) -> &SessionStats {
        &self.stats
    }

    pub fn is_running(&self) -> bool {
        self.stats.running.load(Ordering::SeqCst)
    }

    /// Fire the trigger now (while waiting for it).
    pub fn force_trigger(&self) {
        self.stats.force.store(true, Ordering::SeqCst);
    }

    /// Ask the capture to stop (does not wait).
    pub fn request_stop(&self) {
        self.stop.store(true, Ordering::SeqCst);
    }

    /// Wait for the capture thread and return its result.
    pub fn join(mut self) -> SessionResult {
        self.handle
            .take()
            .expect("joined twice")
            .join()
            .expect("capture thread panicked")
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        self.request_stop();
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SimDevice;
    use crate::Pattern;
    use bf_core::{CaptureStore, ChannelCount, StoreConfig};

    #[test]
    fn simulated_capture_fills_the_store_exactly() {
        let channels = ChannelCount::new(16).unwrap();
        let rate = 50e6;
        let store = CaptureStore::shared(StoreConfig::new(channels, rate));
        let cfg = CaptureConfig {
            channels,
            sample_rate: rate,
            limit_samples: Some(500_000),
            pattern: Pattern::Counter,
            threshold_v: 1.7,
        };
        let mut dev = SimDevice::new(false);
        let res = Session::start(&mut dev, &cfg, store.clone())
            .unwrap()
            .join();
        assert!(res.error.is_none());
        assert_eq!(res.samples, 500_000);
        assert_eq!(res.bytes, 1_000_000);
        let s = store.read();
        assert_eq!(s.range(), 0..500_000);
        for p in [0u64, 1, 4095, 4096, 65_535, 65_536, 499_999] {
            assert_eq!(s.state_at(p), Some(p as u32 & 0xffff), "p={p}");
        }
        // Counter bit 0 toggles on every sample.
        assert_eq!(s.count_toggles(0, 0..500_000), 499_999);
    }

    #[test]
    fn stop_ends_a_continuous_capture() {
        let channels = ChannelCount::new(8).unwrap();
        let store = CaptureStore::shared(StoreConfig::new(channels, 10e6));
        let cfg = CaptureConfig {
            channels,
            sample_rate: 10e6,
            limit_samples: None,
            pattern: Pattern::Generator,
            threshold_v: 1.7,
        };
        let mut dev = SimDevice::new(true);
        let session = Session::start(&mut dev, &cfg, store.clone()).unwrap();
        std::thread::sleep(Duration::from_millis(200));
        session.request_stop();
        let res = session.join();
        assert!(res.error.is_none());
        assert!(
            res.samples > 500_000 && res.samples < 5_000_000,
            "samples {}",
            res.samples
        );
        assert_eq!(store.read().end(), res.samples);
    }

    fn counter_capture(
        conds: &[(usize, bf_core::Cond)],
        pre: u64,
        post: u64,
    ) -> (SharedStore, SessionResult) {
        let channels = ChannelCount::new(16).unwrap();
        let store = CaptureStore::shared(StoreConfig::new(channels, 50e6));
        let mut v = vec![bf_core::Cond::X; 16];
        for (c, k) in conds {
            v[*c] = *k;
        }
        let cfg = CaptureConfig {
            channels,
            sample_rate: 50e6,
            limit_samples: None,
            pattern: Pattern::Counter,
            threshold_v: 1.7,
        };
        let plan = TriggerPlan::pattern(Trigger::new(&v), pre, Some(post));
        let mut dev = SimDevice::new(false);
        let res = Session::start_triggered(&mut dev, &cfg, store.clone(), Some(plan))
            .unwrap()
            .join();
        (store, res)
    }

    #[test]
    fn trigger_cuts_the_capture_around_the_trigger_sample() {
        use bf_core::Cond::*;
        // The counter's bit 15 rises (and is first high) at sample 0x8000.
        for conds in [
            vec![(15, Rising)],
            vec![(15, High)],
            vec![(15, High), (14, Low), (0, Low)],
        ] {
            let (store, res) = counter_capture(&conds, 1000, 5000);
            assert!(res.error.is_none());
            let s = store.read();
            assert_eq!(s.trigger(), Some(0x8000), "{conds:?}");
            assert_eq!(s.range(), (0x8000 - 1000)..(0x8000 + 5000), "{conds:?}");
            assert_eq!(s.state_at(0x8000), Some(0x8000));
            assert_eq!(s.state_at(0x7fff), Some(0x7fff));
            assert_eq!(
                s.state_at(0x8000 - 1001),
                None,
                "hidden below the pre-trigger window"
            );
        }
    }

    #[test]
    fn the_trigger_arms_only_after_the_pre_trigger_window_is_full() {
        use bf_core::Cond::*;
        // Bit 0 rises at every odd sample; with 1000 samples of pre-trigger the first usable one is 1001.
        let (store, _) = counter_capture(&[(0, Rising)], 1000, 100);
        let s = store.read();
        assert_eq!(s.trigger(), Some(1001));
        assert_eq!(s.range(), 1..1101);
        // No condition at all fires on the first sample that may.
        let (store, _) = counter_capture(&[], 500, 100);
        let s = store.read();
        assert_eq!((s.trigger(), s.range()), (Some(500), 0..600));
        // Pre-trigger 0: a level condition may fire on the very first sample, an edge cannot.
        let (store, _) = counter_capture(&[(0, Low)], 0, 10);
        assert_eq!(store.read().trigger(), Some(0));
        let (store, _) = counter_capture(&[(0, Falling)], 0, 10);
        assert_eq!(store.read().trigger(), Some(2));
    }

    #[test]
    fn the_capture_length_is_pre_plus_post_whatever_the_buffer_sizes() {
        use bf_core::Cond::*;
        for (pre, post) in [(0u64, 1u64), (1, 1), (4095, 4097), (70_000, 3)] {
            let (store, _) = counter_capture(&[(15, Rising)], pre, post);
            let s = store.read();
            // With 70000 samples of pre-trigger the first bit-15 edge (32768) comes too early; the
            // next one, one counter period (65536) later, is the first that can fire.
            let t = if pre <= 0x8000 {
                0x8000
            } else {
                0x8000 + 0x10000
            };
            assert_eq!(s.trigger(), Some(t));
            assert_eq!(s.range(), (t - pre)..(t + post), "pre {pre} post {post}");
        }
    }

    #[test]
    fn waiting_keeps_only_the_pre_trigger_window_and_can_be_forced() {
        use bf_core::Cond::*;
        let channels = ChannelCount::new(16).unwrap();
        let rate = 20e6;
        let store = CaptureStore::shared(StoreConfig::new(channels, rate));
        // An increment makes exactly one bit rise, so two rising bits at once never happen.
        let mut c = vec![X; 16];
        c[14] = Rising;
        c[15] = Rising;
        let cfg = CaptureConfig {
            channels,
            sample_rate: rate,
            limit_samples: None,
            pattern: Pattern::Counter,
            threshold_v: 1.7,
        };
        let plan = TriggerPlan::pattern(Trigger::new(&c), 100_000, None);
        let mut dev = SimDevice::new(true);
        let session = Session::start_triggered(&mut dev, &cfg, store.clone(), Some(plan)).unwrap();
        // Wait for data instead of for a fixed time: the machine may be busy with other tests.
        let t0 = Instant::now();
        while store.read().end() < 2_500_000 && t0.elapsed() < Duration::from_secs(10) {
            std::thread::sleep(Duration::from_millis(20));
        }
        assert_eq!(session.stats().phase(), Phase::Waiting);
        {
            let s = store.read();
            assert!(s.end() > 2_000_000, "end {}", s.end());
            assert!(s.trigger().is_none());
            // Whole chunks of a million samples are freed; the window itself is kept.
            let kept = s.end() - s.first_sample();
            assert!(
                (100_000..100_000 + 2_000_000).contains(&kept),
                "{:?}",
                s.range()
            );
        }
        session.force_trigger();
        std::thread::sleep(Duration::from_millis(100));
        assert_eq!(session.stats().phase(), Phase::Triggered);
        session.request_stop();
        session.join();
        let s = store.read();
        let t = s.trigger().expect("forced trigger");
        assert_eq!(s.first_sample(), t - 100_000);
        assert!(s.end() > t);
    }

    #[test]
    fn after_the_trigger_a_full_memory_ends_the_capture_instead_of_dropping_data() {
        use bf_core::Cond::*;
        let channels = ChannelCount::new(16).unwrap();
        // The counter changes on every sample: 8 bytes each, so 2 MiB fill up after a quarter million samples.
        let mut cfg = StoreConfig::new(channels, 50e6);
        cfg.max_bytes = 2 << 20;
        let store = CaptureStore::shared(cfg);
        let mut c = vec![X; 16];
        c[15] = Rising;
        let capture = CaptureConfig {
            channels,
            sample_rate: 50e6,
            limit_samples: None,
            pattern: Pattern::Counter,
            threshold_v: 1.7,
        };
        let plan = TriggerPlan::pattern(Trigger::new(&c), 1000, None);
        let mut dev = SimDevice::new(false);
        let session =
            Session::start_triggered(&mut dev, &capture, store.clone(), Some(plan)).unwrap();
        // Stopped by itself: wait for it.
        let t0 = Instant::now();
        while session.is_running() && t0.elapsed() < Duration::from_secs(20) {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(!session.is_running(), "the capture must stop on its own");
        assert!(session.stats().memory_full.load(Ordering::SeqCst));
        session.join();
        let s = store.read();
        assert_eq!(s.trigger(), Some(0x8000));
        assert_eq!(
            s.first_sample(),
            0x8000 - 1000,
            "nothing before the trigger was evicted"
        );
        assert!(
            s.end() > 0x8000 + 100_000,
            "recorded well past the trigger: {}",
            s.end()
        );
        assert!(s.memory_bytes() <= 2 << 20);
    }
}
