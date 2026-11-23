//! Files: capture files, setup files, exports, and the setup that is kept between runs.
//!
//! File dialogs are blinkfeuer's own (`crate::file_dialog`), drawn in the window, so they work the same
//! everywhere (no desktop portal needed). Reading or writing a capture runs on a background thread,
//! so a large capture never freezes the UI.

use super::*;
use crate::file_dialog::{FileDialog, Outcome};
use crate::setup::{self, DecoderSetup, Setup};
use bf_io::{read_capture, read_sr, write_capture, write_vcd, VcdSignal};
use std::io::{BufReader, BufWriter};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{channel, Receiver, TryRecvError};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum Purpose {
    OpenCapture,
    SaveCapture,
    OpenSetup,
    SaveSetup,
    ExportVcd,
    ExportCsv,
}

impl Purpose {
    fn saves(self) -> bool {
        !matches!(self, Purpose::OpenCapture | Purpose::OpenSetup)
    }

    fn filter(self) -> (&'static str, &'static str) {
        match self {
            Purpose::OpenCapture | Purpose::SaveCapture => ("Blinkfeuer capture", "bfc"),
            Purpose::OpenSetup | Purpose::SaveSetup => ("Blinkfeuer setup", "json"),
            Purpose::ExportVcd => ("Value change dump", "vcd"),
            Purpose::ExportCsv => ("CSV", "csv"),
        }
    }

    fn title(self) -> &'static str {
        match self {
            Purpose::OpenCapture => "Open capture",
            Purpose::SaveCapture => "Save capture",
            Purpose::OpenSetup => "Open setup",
            Purpose::SaveSetup => "Save setup",
            Purpose::ExportVcd => "Export VCD",
            Purpose::ExportCsv => "Export decoded items",
        }
    }

    fn default_name(self) -> &'static str {
        match self {
            Purpose::OpenCapture | Purpose::SaveCapture => "capture.bfc",
            Purpose::OpenSetup | Purpose::SaveSetup => "setup.json",
            Purpose::ExportVcd => "capture.vcd",
            Purpose::ExportCsv => "decoded.csv",
        }
    }
}

pub(super) enum JobResult {
    Done(String),
    Loaded {
        path: PathBuf,
        store: Box<CaptureStore>,
        setup: serde_json::Value,
    },
    Failed(String),
}

pub(super) struct Job {
    label: String,
    rx: Receiver<JobResult>,
}

/// Which parts of a setup to apply.
#[derive(Clone, Copy)]
pub(super) struct Apply {
    /// Device, channels, rate, pattern, threshold, run mode (not for a loaded capture, which brings its own).
    pub acquisition: bool,
    /// Markers and the visible window.
    pub view: bool,
}

impl DeviceChoice {
    fn key(self) -> &'static str {
        match self {
            DeviceChoice::Sim => "sim",
            DeviceChoice::SimRealtime => "sim-realtime",
            DeviceChoice::Stock => "stock",
            DeviceChoice::Litex => "litex",
            DeviceChoice::Fx2lafw => "fx2lafw",
        }
    }

    fn from_key(k: &str) -> Option<Self> {
        [
            DeviceChoice::Sim,
            DeviceChoice::SimRealtime,
            DeviceChoice::Stock,
            DeviceChoice::Litex,
            DeviceChoice::Fx2lafw,
        ]
        .into_iter()
        .find(|d| d.key() == k)
    }
}

fn with_extension(path: PathBuf, ext: &str) -> PathBuf {
    if path.extension().is_some() {
        path
    } else {
        path.with_extension(ext)
    }
}

/// Write to `path.tmp` and rename, so that an error never destroys an existing file.
fn write_via_temp(
    path: &Path,
    f: impl FnOnce(&mut BufWriter<std::fs::File>) -> Result<(), String>,
) -> Result<(), String> {
    let mut tmp = path.as_os_str().to_owned();
    tmp.push(".tmp");
    let tmp = PathBuf::from(tmp);
    let result = (|| {
        let file = std::fs::File::create(&tmp).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut w = BufWriter::with_capacity(1 << 20, file);
        f(&mut w)?;
        w.into_inner()
            .map_err(|e| e.to_string())?
            .sync_all()
            .map_err(|e| e.to_string())?;
        std::fs::rename(&tmp, path).map_err(|e| format!("{}: {e}", path.display()))
    })();
    if result.is_err() {
        let _ = std::fs::remove_file(&tmp);
    }
    result
}

/// Read a capture file: a Blinkfeuer capture, or a sigrok session (recognised by being a zip archive),
/// whose probe names become the channel names.
fn load_capture_file(path: &Path) -> Result<(CaptureStore, serde_json::Value), String> {
    let mut file = std::fs::File::open(path).map_err(|e| e.to_string())?;
    let mut magic = [0u8; 4];
    let zip = std::io::Read::read_exact(&mut file, &mut magic).is_ok() && &magic == b"PK\x03\x04";
    std::io::Seek::rewind(&mut file).map_err(|e| e.to_string())?;
    if zip {
        let c = read_sr(BufReader::with_capacity(1 << 20, file)).map_err(|e| e.to_string())?;
        let n = c.store.channels();
        let mut layout = Layout::new(n as usize);
        layout.names = c.names;
        let setup = Setup {
            version: setup::VERSION,
            channels: Some(n),
            rate: Some(c.sample_rate),
            layout: Some(layout),
            ..Default::default()
        };
        Ok((c.store, setup.to_json()))
    } else {
        let l = read_capture(BufReader::with_capacity(1 << 20, file)).map_err(|e| e.to_string())?;
        Ok((l.store, l.setup))
    }
}

impl App {
    pub(super) fn notify(&mut self, msg: impl Into<String>, error: bool) {
        self.notice = Some((msg.into(), Instant::now(), error));
    }

    // ------------------------------------------------------------ setup

    /// The setup as it is now. `layout`: include the row layout (always for files; for the automatic
    /// save only once the user has changed it, so a default layout still follows the test pattern).
    pub(super) fn current_setup(&self, layout: bool, with_view: bool) -> Setup {
        Setup {
            version: setup::VERSION,
            device: Some(self.device.key().into()),
            channels: Some(self.channels_n),
            rate: Some(self.rate),
            pattern: Some(self.pattern.name().into()),
            threshold: Some(self.threshold),
            continuous: Some(self.continuous),
            samples: Some(self.samples),
            snap: Some(self.snap),
            row_height: Some(self.row_h),
            row_auto: Some(self.row_auto),
            layout: layout.then(|| self.layout.clone()),
            decoders: self
                .decoders
                .iter()
                .map(|e| DecoderSetup {
                    id: e.handle.id.0,
                    name: e.name.clone(),
                    format: e.format,
                    color: e.color,
                    config: e.cfg.clone(),
                })
                .collect(),
            trigger: Some(self.trigger.clone()),
            zero_at_trigger: Some(self.zero_at_trigger),
            memory_mib: Some(self.memory_mib),
            markers: with_view.then_some((self.markers.m1, self.markers.m2)),
            view: with_view.then_some((self.view.start, self.view.spp)),
            last_dir: self.last_dir.as_ref().map(|d| d.display().to_string()),
        }
    }

    /// Apply a setup. Problems with parts of it are collected into the error; the rest is applied.
    pub(super) fn apply_setup(&mut self, s: Setup, what: Apply) -> Result<(), String> {
        let mut problems: Vec<String> = Vec::new();
        if what.acquisition {
            if let Some(d) = s.device.as_deref().and_then(DeviceChoice::from_key) {
                self.device = d;
            }
            if let Some(n) = s.channels.filter(|n| ChannelCount::new(*n).is_some()) {
                self.channels_n = n;
            }
            if let Some(r) = s.rate.filter(|r| r.is_finite() && *r > 0.0) {
                self.rate = r;
            }
            self.pattern = s
                .pattern
                .as_deref()
                .and_then(|p| {
                    self.device
                        .patterns()
                        .iter()
                        .find(|x| x.name() == p)
                        .copied()
                })
                .unwrap_or(self.device.patterns()[0]);
            if let Some(t) = s.threshold.filter(|t| t.is_finite()) {
                self.threshold = t.clamp(0.0, 3.3);
            }
            if let Some(c) = s.continuous {
                self.continuous = c;
            }
            if let Some(n) = s.samples.filter(|n| n.is_finite() && *n >= 1.0) {
                self.samples = n;
            }
        }
        if what.acquisition {
            if let Some(t) = s.trigger {
                self.trigger = t.sanitized();
            }
            if let Some(m) = s.memory_mib.filter(|m| *m >= 64) {
                self.memory_mib = m.min(1 << 21);
            }
        }
        if let Some(v) = s.zero_at_trigger {
            self.zero_at_trigger = v;
        }
        if let Some(v) = s.snap {
            self.snap = v;
        }
        if let Some(h) = s.row_height.filter(|h| h.is_finite()) {
            self.row_h = h.clamp(14.0, 160.0);
        }
        if let Some(v) = s.row_auto {
            self.row_auto = v;
        }
        if let Some(d) = s
            .last_dir
            .as_deref()
            .map(PathBuf::from)
            .filter(|d| d.is_dir())
        {
            self.last_dir = Some(d);
        }

        // Decoders: the old ones go, the saved ones are started again.
        let n = self.channels_n;
        for e in self.decoders.drain(..) {
            self.layout.remove_decoder(e.handle.id.0);
            self.engine.remove(e.handle.id);
        }
        self.selected_ann = None;
        self.table = decode_ui::TableState::default();
        self.setup.selected = 0;

        let mut layout = None;
        if let Some(l) = s.layout {
            match l.check(n as usize) {
                Ok(()) => layout = Some(l),
                Err(e) => problems.push(format!("row layout not applied: {e}")),
            }
        }
        let mut map = std::collections::HashMap::new();
        for mut d in s.decoders {
            crate::decode_ui::lf_only_for_ascii(&mut d.config, d.format);
            if let Err(e) = d.config.check_file_values(n) {
                problems.push(format!("decoder '{}' skipped: {e}", d.name));
                continue;
            }
            let handle = self.engine.add(d.config.build());
            map.insert(d.id, handle.id.0);
            self.decoders.push(Entry {
                handle,
                cfg: d.config,
                name: d.name,
                format: d.format,
                color: d.color,
            });
        }
        if let Some(mut l) = layout {
            l.map_decoder_ids(&map);
            self.layout = l;
            self.layout_touched = true;
            self.selected = None;
        }
        let ids: Vec<u32> = self.decoders.iter().map(|e| e.handle.id.0).collect();
        self.layout.place_missing_decoders(&ids);

        if what.view {
            if let Some((a, b)) = s.markers {
                self.markers = Markers {
                    m1: a.filter(|v| v.is_finite()),
                    m2: b.filter(|v| v.is_finite()),
                };
            }
            if let Some((start, spp)) = s
                .view
                .filter(|(a, b)| a.is_finite() && b.is_finite() && *b > 0.0)
            {
                self.view = View { start, spp };
                self.follow = false;
            }
        }
        if problems.is_empty() {
            Ok(())
        } else {
            Err(problems.join("; "))
        }
    }

    pub(super) fn autosave_enabled(&self) -> bool {
        std::env::var_os("BLINKFEUER_CONFIG_DIR").is_some()
            || !(self.opts.demo || self.opts.screenshot.is_some())
    }

    /// Take over the setup of the last run.
    pub(super) fn restore_autosave(&mut self) {
        if !self.autosave_enabled() {
            return;
        }
        let Some(path) = setup::autosave_path().filter(|p| p.exists()) else {
            return;
        };
        match Setup::read(&path) {
            Ok(s) => {
                if let Err(e) = self.apply_setup(
                    s,
                    Apply {
                        acquisition: true,
                        view: false,
                    },
                ) {
                    eprintln!("blinkfeuer: saved setup only partly applied: {e}");
                }
            }
            Err(e) => eprintln!("blinkfeuer: ignoring the saved setup: {e}"),
        }
        self.last_autosave = self.autosave_text();
    }

    fn autosave_text(&self) -> String {
        serde_json::to_string_pretty(&self.current_setup(self.layout_touched, false))
            .unwrap_or_default()
    }

    /// Keep the setup on disk (checked every couple of seconds, written when it changed).
    pub(super) fn autosave(&mut self, force: bool) {
        if !self.autosave_enabled()
            || (!force && self.autosave_at.elapsed() < Duration::from_secs(2))
        {
            return;
        }
        self.autosave_at = Instant::now();
        let text = self.autosave_text();
        if text == self.last_autosave {
            return;
        }
        if let Some(path) = setup::autosave_path() {
            match setup::write_atomic(&path, text.as_bytes()) {
                Ok(()) => self.last_autosave = text,
                Err(e) => eprintln!(
                    "blinkfeuer: cannot save the setup to {}: {e}",
                    path.display()
                ),
            }
        }
    }

    // ------------------------------------------------------------ dialogs and jobs

    /// Filters (label, extensions) and the proposed file name (when saving).
    fn dialog_spec(&self, p: Purpose) -> (Vec<(String, Vec<String>)>, Option<String>) {
        let (label, ext) = p.filter();
        let mut filters: Vec<(&str, Vec<&str>)> = if p == Purpose::OpenCapture {
            // Blinkfeuer captures and sigrok sessions (PulseView, sigrok-cli).
            vec![
                ("Captures (.bfc, .sr)", vec!["bfc", "sr"]),
                (label, vec![ext]),
                ("sigrok session", vec!["sr"]),
            ]
        } else {
            vec![(label, vec![ext])]
        };
        filters.push(("All files", vec!["*"]));
        let filters = filters
            .into_iter()
            .map(|(l, e)| (l.to_string(), e.into_iter().map(String::from).collect()))
            .collect();
        let name = p.saves().then(|| {
            self.file
                .as_ref()
                .and_then(|f| f.file_stem())
                .map(|s| format!("{}.{}", s.to_string_lossy(), ext))
                .unwrap_or_else(|| p.default_name().to_string())
        });
        (filters, name)
    }

    pub(super) fn ask_file(&mut self, p: Purpose) {
        let (filters, name) = self.dialog_spec(p);
        let dialog = FileDialog::new(p.title(), p.saves(), self.last_dir.clone(), name, filters);
        self.dialog = Some((p, Box::new(dialog)));
    }

    /// A file dialog is open: the window's own shortcuts are off meanwhile.
    pub(super) fn file_dialog_open(&self) -> bool {
        self.dialog.is_some()
    }

    fn spawn_job(&mut self, label: String, f: impl FnOnce() -> JobResult + Send + 'static) {
        let (tx, rx) = channel();
        std::thread::spawn(move || {
            let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f))
                .unwrap_or_else(|_| JobResult::Failed("internal error".into()));
            let _ = tx.send(r);
        });
        self.job = Some(Job { label, rx });
    }

    /// Check on the open dialog and the running job.
    pub(super) fn poll_files(&mut self, ctx: &egui::Context) {
        if let Some((p, dialog)) = &mut self.dialog {
            let p = *p;
            match dialog.show(ctx) {
                Some(Outcome::Chosen(path)) => {
                    self.dialog = None;
                    self.file_chosen(p, path);
                }
                Some(Outcome::Cancelled) => self.dialog = None,
                None => {}
            }
        }
        if let Some(job) = &self.job {
            match job.rx.try_recv() {
                Ok(r) => {
                    self.job = None;
                    match r {
                        JobResult::Done(msg) => self.notify(msg, false),
                        JobResult::Failed(e) => self.notify(e, true),
                        JobResult::Loaded { path, store, setup } => {
                            self.finish_load(path, *store, setup)
                        }
                    }
                }
                Err(TryRecvError::Empty) => ctx.request_repaint_after(Duration::from_millis(100)),
                Err(TryRecvError::Disconnected) => {
                    self.job = None;
                    self.notify("the background task ended unexpectedly", true);
                }
            }
        }
    }

    pub(super) fn file_chosen(&mut self, p: Purpose, path: PathBuf) {
        let path = if p.saves() {
            with_extension(path, p.filter().1)
        } else {
            path
        };
        if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
            self.last_dir = Some(dir.to_path_buf());
        }
        let name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        match p {
            Purpose::OpenCapture => {
                let path2 = path.clone();
                self.spawn_job(format!("Loading {name}"), move || {
                    match load_capture_file(&path2) {
                        Ok((store, setup)) => JobResult::Loaded {
                            path: path2,
                            store: Box::new(store),
                            setup,
                        },
                        Err(e) => JobResult::Failed(format!("{name}: {e}")),
                    }
                });
            }
            Purpose::SaveCapture => {
                let store = self.store.clone();
                let setup = self.current_setup(true, true).to_json();
                self.file = Some(path.clone());
                self.spawn_job(format!("Saving {name}"), move || {
                    let r = write_via_temp(&path, |w| {
                        write_capture(w, &store.read(), &setup).map_err(|e| e.to_string())
                    });
                    match r {
                        Ok(()) => JobResult::Done(format!("saved {}", path.display())),
                        Err(e) => JobResult::Failed(e),
                    }
                });
            }
            Purpose::OpenSetup => match Setup::read(&path).and_then(|s| {
                self.apply_setup(
                    s,
                    Apply {
                        acquisition: true,
                        view: false,
                    },
                )
            }) {
                Ok(()) => self.notify(format!("loaded setup {name}"), false),
                Err(e) => self.notify(format!("setup {name}: {e}"), true),
            },
            Purpose::SaveSetup => match self.current_setup(true, false).write(&path) {
                Ok(()) => self.notify(format!("saved setup {}", path.display()), false),
                Err(e) => self.notify(e, true),
            },
            Purpose::ExportVcd => {
                let store = self.store.clone();
                let (range, signals) = self.vcd_selection();
                self.spawn_job(format!("Exporting {name}"), move || {
                    let r = write_via_temp(&path, |w| {
                        write_vcd(w, &store.read(), range, &signals).map_err(|e| e.to_string())
                    });
                    match r {
                        Ok(()) => JobResult::Done(format!("exported {}", path.display())),
                        Err(e) => JobResult::Failed(e),
                    }
                });
            }
            Purpose::ExportCsv => {
                let Some(e) = self.decoders.get(self.table.entry) else {
                    return;
                };
                let (anns, lane, fmt) = (e.handle.anns.clone(), self.table.lane, e.format);
                let rate = self.store.read().sample_rate();
                self.spawn_job(format!("Exporting {name}"), move || {
                    let r = write_via_temp(&path, |w| {
                        decode_ui::write_csv(w, &anns.read().lanes, lane, rate, fmt)
                            .map_err(|e| e.to_string())
                    });
                    match r {
                        Ok(()) => JobResult::Done(format!("exported {}", path.display())),
                        Err(e) => JobResult::Failed(e),
                    }
                });
            }
        }
    }

    /// Range (between M1 and M2 when both are set, else everything) and channels for a VCD export.
    pub(super) fn vcd_selection(&self) -> (std::ops::Range<u64>, Vec<VcdSignal>) {
        let all = self.store.read().range();
        let range = match (self.markers.m1, self.markers.m2) {
            (Some(a), Some(b)) => {
                let (lo, hi) = (
                    a.min(b).max(0.0).floor() as u64,
                    b.max(a).max(0.0).ceil() as u64 + 1,
                );
                lo.max(all.start)..hi.min(all.end)
            }
            _ => all,
        };
        let n = self.layout.names.len();
        let mut chans: Vec<usize> = (0..n).filter(|&c| self.layout.visible[c]).collect();
        if chans.is_empty() {
            chans = (0..n).collect();
        }
        let signals = chans
            .into_iter()
            .map(|c| VcdSignal {
                channel: c as u8,
                name: self.layout.names[c].clone(),
            })
            .collect();
        (range, signals)
    }

    /// A capture came in from a file: it replaces the current one.
    fn finish_load(&mut self, path: PathBuf, store: CaptureStore, setup_json: serde_json::Value) {
        let n = store.channels();
        self.rate = store.sample_rate();
        self.channels_n = n;
        self.store = store.into_shared();
        self.engine.set_store(self.store.clone());
        self.session = None;
        self.last_result = None;
        self.selected_ann = None;
        self.selected = None;
        self.markers = Markers::default();
        self.follow = false;
        self.error = None;
        self.layout = Layout::new(n as usize);
        self.layout_touched = false;
        match Setup::from_json(&setup_json) {
            Ok(s) => {
                if let Err(e) = self.apply_setup(
                    s,
                    Apply {
                        acquisition: false,
                        view: true,
                    },
                ) {
                    self.notify(format!("setup of the file only partly applied: {e}"), true);
                }
            }
            Err(e) => self.notify(
                format!("the setup stored in the file was ignored: {e}"),
                true,
            ),
        }
        // An opened capture is shown whole (the view saved with it is not restored).
        self.fit();
        self.file = Some(path);
        let r = self.store.read().range();
        self.view.clamp(r, self.plot_width);
    }

    pub(super) fn save_capture(&mut self) {
        match self.file.clone() {
            Some(p) => self.file_chosen(Purpose::SaveCapture, p),
            None => self.ask_file(Purpose::SaveCapture),
        }
    }

    pub(super) fn busy_files(&self) -> bool {
        self.job.is_some() || self.dialog.is_some()
    }

    pub(super) fn job_label(&self) -> Option<&str> {
        self.job.as_ref().map(|j| j.label.as_str())
    }

    pub(super) fn file_menu(&mut self, ui: &mut egui::Ui) {
        let idle = !self.running() && !self.busy_files();
        let have_data = !self.store.read().is_empty();
        let mut ask: Option<Purpose> = None;
        if ui
            .add_enabled(
                idle,
                egui::Button::new("Open capture...").shortcut_text("Ctrl+O"),
            )
            .clicked()
        {
            ask = Some(Purpose::OpenCapture);
        }
        if ui
            .add_enabled(
                idle && have_data,
                egui::Button::new("Save capture").shortcut_text("Ctrl+S"),
            )
            .clicked()
        {
            self.save_capture();
            ui.close_menu();
        }
        if ui
            .add_enabled(idle && have_data, egui::Button::new("Save capture as..."))
            .clicked()
        {
            ask = Some(Purpose::SaveCapture);
        }
        ui.separator();
        if ui
            .add_enabled(!self.busy_files(), egui::Button::new("Open setup..."))
            .clicked()
        {
            ask = Some(Purpose::OpenSetup);
        }
        if ui
            .add_enabled(!self.busy_files(), egui::Button::new("Save setup as..."))
            .on_hover_text(
                "Channels, colors, groups and decoders; also kept automatically between runs",
            )
            .clicked()
        {
            ask = Some(Purpose::SaveSetup);
        }
        ui.separator();
        let between = self.markers.m1.is_some() && self.markers.m2.is_some();
        let vcd = if between {
            "Export VCD (M1 to M2)..."
        } else {
            "Export VCD..."
        };
        if ui
            .add_enabled(idle && have_data, egui::Button::new(vcd))
            .clicked()
        {
            ask = Some(Purpose::ExportVcd);
        }
        if ui
            .add_enabled(
                idle && !self.decoders.is_empty(),
                egui::Button::new("Export decoded items (CSV)..."),
            )
            .on_hover_text("The decoder and lane shown in the annotation table")
            .clicked()
        {
            ask = Some(Purpose::ExportCsv);
        }
        ui.separator();
        if ui.button("Quit").clicked() {
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
        if let Some(p) = ask {
            self.ask_file(p);
            ui.close_menu();
        }
    }

    /// Window title: the file of the capture on screen.
    pub(super) fn update_title(&mut self, ctx: &egui::Context) {
        let t = match self.file.as_ref().and_then(|f| f.file_name()) {
            Some(n) => format!("Blinkfeuer - {}", n.to_string_lossy()),
            None => "Blinkfeuer".to_string(),
        };
        if t != self.title {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(t.clone()));
            self.title = t;
        }
    }
}
