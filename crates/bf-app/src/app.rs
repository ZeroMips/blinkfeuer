//! The blinkfeuer application window, laid out like a bench logic analyzer: menu bar, run
//! controls, marker readout, waveform with a Bus/Signal column, overview strip and status bar.

mod files;

use crate::decode_ui::{self, Config, DecoderKind, Entry};
use crate::markers::{delta_text, snap_to_edge, Markers, Which};
use crate::rows::{
    bus_value, format_bus, parse_channels, Dest, Item, Layout, Mover, Radix, Row, RowKind,
};
use crate::theme;
use crate::view::{
    format_duration, format_hz, format_scale, format_split, parse_time, scale_step, spp_for_step,
    View,
};
use crate::waveform::{
    draw_annotations, draw_bus, draw_grid, draw_markers, draw_ruler, draw_signal, draw_trigger,
};
use crate::Opts;
use bf_core::{CaptureStore, ChannelCount, Cond, SharedStore, StoreConfig, Trigger};
use bf_decode::engine::Engine;
use bf_decode::Format;
use bf_device::fx2lafw::{self, Fx2lafwDevice};
use bf_device::litex::LitexDevice;
use bf_device::session::{Phase, Session, SessionResult, TriggerKind, TriggerPlan};
use bf_device::sim::SimDevice;
use bf_device::stock::StockDevice;
use bf_device::{CaptureConfig, Device, Pattern};
use egui::{
    pos2, vec2, Align2, Color32, CursorIcon, FontId, Key, Margin, Rect, RichText, Sense, Stroke,
};
use std::time::{Duration, Instant};

const NAME_W: f32 = 140.0;
const VAL_W: f32 = 52.0;
const GUTTER: f32 = NAME_W + 2.0 * VAL_W;
const RULER_H: f32 = 36.0;
const OVERVIEW_H: f32 = 46.0;

#[derive(Clone, Copy, PartialEq, Eq)]
enum DeviceChoice {
    Sim,
    SimRealtime,
    Stock,
    Litex,
    Fx2lafw,
}

/// Samplerates offered for the SLogic16U3 (each limited by the channel count).
const SLOGIC_RATES: [f64; 8] = [5e6, 10e6, 20e6, 50e6, 100e6, 200e6, 400e6, 800e6];

impl DeviceChoice {
    fn label(self) -> &'static str {
        match self {
            DeviceChoice::Sim => "Simulator (fast)",
            DeviceChoice::SimRealtime => "Simulator (real time)",
            DeviceChoice::Stock => "SLogic16U3 (stock firmware)",
            DeviceChoice::Litex => "SLogic16U3 (LiteX gateware)",
            DeviceChoice::Fx2lafw => "FX2 / Saleae Logic clone (fx2lafw)",
        }
    }

    fn open(self) -> Result<Box<dyn Device>, String> {
        match self {
            DeviceChoice::Sim => Ok(Box::new(SimDevice::new(false))),
            DeviceChoice::SimRealtime => Ok(Box::new(SimDevice::new(true))),
            DeviceChoice::Stock => StockDevice::open_first()
                .map(|d| Box::new(d) as Box<dyn Device>)
                .map_err(|e| e.to_string()),
            DeviceChoice::Litex => LitexDevice::open_first()
                .map(|d| Box::new(d) as Box<dyn Device>)
                .map_err(|e| e.to_string()),
            DeviceChoice::Fx2lafw => Fx2lafwDevice::open_first()
                .map(|d| Box::new(d) as Box<dyn Device>)
                .map_err(|e| e.to_string()),
        }
    }

    /// Test patterns the backend offers; the first is the default.
    fn patterns(self) -> &'static [Pattern] {
        match self {
            DeviceChoice::Sim | DeviceChoice::SimRealtime => {
                &[Pattern::Generator, Pattern::Counter, Pattern::Inputs]
            }
            DeviceChoice::Stock => &[Pattern::Inputs, Pattern::Counter, Pattern::Emulation],
            DeviceChoice::Litex => &[Pattern::Inputs, Pattern::Counter, Pattern::Generator],
            DeviceChoice::Fx2lafw => &[Pattern::Inputs],
        }
    }

    /// Selectable channel counts.
    fn channel_options(self) -> &'static [u8] {
        match self {
            DeviceChoice::Fx2lafw => &[8],
            _ => &[4, 8, 16],
        }
    }

    /// Selectable samplerates (ascending) and the highest one usable with `channels`.
    fn rates(self, channels: u8) -> (&'static [f64], f64) {
        match self {
            DeviceChoice::Fx2lafw => (&fx2lafw::RATES, 24e6),
            _ => (
                &SLOGIC_RATES,
                match channels {
                    4 => 800e6,
                    8 => 400e6,
                    _ => 200e6,
                },
            ),
        }
    }

    /// A real analyzer (not the simulator).
    fn is_hardware(self) -> bool {
        matches!(
            self,
            DeviceChoice::Stock | DeviceChoice::Litex | DeviceChoice::Fx2lafw
        )
    }

    fn has_threshold(self) -> bool {
        self == DeviceChoice::Stock
    }
}

/// A UART decoder showing plain characters (text), as opposed to a binary protocol on top.
fn is_text_uart(cfg: &Config) -> bool {
    matches!(cfg, Config::Uart(c) if c.protocol == bf_decode::uart::Protocol::Plain)
}

/// A quarter of the machine's memory (at least 1 GiB, at most 32 GiB) is a sensible default for the
/// capture store.
fn default_memory_mib() -> u64 {
    let total_kib = std::fs::read_to_string("/proc/meminfo")
        .ok()
        .and_then(|t| {
            t.lines()
                .find(|l| l.starts_with("MemTotal:"))
                .and_then(|l| l.split_whitespace().nth(1)?.parse::<u64>().ok())
        })
        .unwrap_or(4 << 20);
    (total_kib / 1024 / 4).clamp(1024, 32 * 1024)
}

fn rgb(c: [u8; 3]) -> Color32 {
    Color32::from_rgb(c[0], c[1], c[2])
}

/// Name field at the top of a row's context menu. True if the name was edited.
fn rename_menu(ui: &mut egui::Ui, name: &mut String) -> bool {
    ui.label(RichText::new("Name").color(theme::TEXT_DIM));
    let changed = ui
        .add(egui::TextEdit::singleline(name).desired_width(130.0))
        .changed();
    ui.separator();
    changed
}

/// "Move up/down" and "Move to group" entries for the context menu of a row. True if the layout changed.
fn order_menu(ui: &mut egui::Ui, layout: &mut Layout, row: &Row) -> bool {
    let mut changed = false;
    {
        ui.separator();
        if ui
            .button("Hide")
            .on_hover_text("Show it again in Setup > Bus/Signal")
            .clicked()
        {
            match row.kind {
                RowKind::Signal(ch) => {
                    if let Some(v) = layout.visible.get_mut(ch as usize) {
                        *v = false;
                    }
                }
                RowKind::Decode { id, .. } => layout.set_decoder_shown(id, false),
                RowKind::Bus(i) => {
                    if let Some(Item::Bus(b)) = layout.items.get_mut(i) {
                        b.shown = false;
                    }
                }
                RowKind::Group(i) => {
                    if let Some(Item::Group(g)) = layout.items.get_mut(i) {
                        g.shown = false;
                    }
                }
            }
            changed = true;
            ui.close_menu();
        }
    }
    if let Some(item) = layout.item_of_row(row) {
        ui.separator();
        if ui.button("Move up").clicked() {
            layout.move_item(item, -1);
            changed = true;
            ui.close_menu();
        }
        if ui.button("Move down").clicked() {
            layout.move_item(item, 1);
            changed = true;
            ui.close_menu();
        }
    }
    if let Some(m) = layout
        .mover_of(row)
        .filter(|m| !matches!(m, Mover::Group(_)))
    {
        let current = layout.group_of(m);
        let groups: Vec<(usize, String)> = layout
            .items
            .iter()
            .enumerate()
            .filter_map(|(i, it)| {
                if let Item::Group(g) = it {
                    Some((i, g.name.clone()))
                } else {
                    None
                }
            })
            .collect();
        {
            ui.menu_button("Add to group", |ui| {
                if ui.button("New group").clicked() {
                    changed |= layout.move_to_new_group(m);
                    ui.close_menu();
                }
                if let Some(g) = current {
                    if ui.button("Remove from group").clicked() {
                        changed |= layout.move_to(m, Dest::Top(g + 1));
                        ui.close_menu();
                    }
                }
                if !groups.is_empty() {
                    ui.separator();
                }
                for (i, name) in groups {
                    if Some(i) != current && ui.button(name).clicked() {
                        changed |= layout.move_to(
                            m,
                            Dest::Group {
                                item: i,
                                pos: usize::MAX,
                            },
                        );
                        ui.close_menu();
                    }
                }
            });
        }
    }
    changed
}

/// Palette grid plus a free-form picker. Returns true when the colour changed.
fn color_menu(ui: &mut egui::Ui, c: &mut [u8; 3]) -> bool {
    let mut changed = false;
    ui.label(RichText::new("Trace color").color(theme::TEXT_DIM));
    for row in crate::rows::PALETTE.chunks(8) {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing = vec2(3.0, 3.0);
            for p in row {
                let (r, resp) = ui.allocate_exact_size(vec2(20.0, 20.0), Sense::click());
                ui.painter().rect_filled(r, 0.0, rgb(*p));
                let stroke = if *c == *p {
                    Stroke::new(2.0_f32, Color32::WHITE)
                } else if resp.hovered() {
                    Stroke::new(1.0_f32, theme::TEXT)
                } else {
                    Stroke::new(1.0_f32, theme::BORDER)
                };
                ui.painter().rect_stroke(r, 0.0, stroke);
                if resp
                    .on_hover_text(format!("#{:02x}{:02x}{:02x}", p[0], p[1], p[2]))
                    .clicked()
                {
                    *c = *p;
                    changed = true;
                }
            }
        });
    }
    ui.horizontal(|ui| {
        ui.label(RichText::new("Custom").color(theme::TEXT_DIM));
        changed |= ui.color_edit_button_srgb(c).changed();
    });
    changed
}

/// A colour swatch button that opens the palette menu.
fn color_button(ui: &mut egui::Ui, c: &mut [u8; 3]) -> bool {
    let mut changed = false;
    ui.menu_button(RichText::new("████").color(rgb(*c)), |ui| {
        changed = color_menu(ui, c)
    });
    changed
}

#[derive(Default)]
struct NewBus {
    name: String,
    channels: String,
    radix: Option<Radix>,
    error: Option<String>,
}

pub struct App {
    store: SharedStore,
    session: Option<Session>,
    last_result: Option<SessionResult>,
    error: Option<String>,

    device: DeviceChoice,
    channels_n: u8,
    rate: f64,
    pattern: Pattern,
    continuous: bool,
    samples: f64,
    threshold: f64,
    /// When the attached analyzers were last looked for, and whether a stock / LiteX one is there.
    stock_seen: (Instant, bool, bool, bool),

    layout: Layout,
    layout_touched: bool,
    selected: Option<RowKind>,
    scroll_y: f32,
    /// Row height in points; `row_auto` fits the rows to the window instead.
    row_h: f32,
    row_auto: bool,
    show_setup: bool,
    show_keys: bool,
    show_about: bool,
    /// The application icon as a texture (for the About window), made when first needed.
    icon_texture: Option<egui::TextureHandle>,
    show_decoders: bool,
    show_table: bool,
    show_trigger: bool,
    show_measure: bool,
    /// Baud rate detection for the UART settings of the trigger window.
    trigger_probe: decode_ui::BaudProbe,
    measure: crate::measure_ui::MeasureState,
    /// Times are counted from the trigger sample (when the capture has one) instead of its start.
    zero_at_trigger: bool,
    /// Memory the capture may use, in MiB (a capture with a trigger stops when it is full).
    memory_mib: u64,
    /// Samples after the trigger the running capture is meant to record (progress display).
    post_target: Option<u64>,
    trigger: crate::setup::TriggerSetup,
    engine: Engine,
    decoders: Vec<Entry>,
    setup: decode_ui::SetupState,
    table: decode_ui::TableState,
    selected_ann: Option<decode_ui::Selected>,
    new_bus: NewBus,

    view: View,
    follow: bool,
    plot_width: f64,
    plot_height: f32,
    pending_fit: bool,
    scale_text: String,

    markers: Markers,
    snap: bool,
    drag_marker: Option<Which>,
    /// A row picked up in the gutter, being dragged to a new place.
    row_drag: Option<Mover>,
    hover_sample: Option<f64>,
    /// Edge the yellow snap target currently sits on.
    hover_edge: Option<u64>,

    rate_stat: (Instant, u64, f64),

    opts: Opts,
    t0: Instant,
    autostarted: bool,
    screenshot_requested: bool,
    zoom_applied: bool,
    goto_done: bool,
    open_done: bool,
    detect_pressed: bool,
    save_done: bool,

    // files
    /// Capture file of the data on screen.
    file: Option<std::path::PathBuf>,
    last_dir: Option<std::path::PathBuf>,
    dialog: Option<(files::Purpose, Box<crate::file_dialog::FileDialog>)>,
    job: Option<files::Job>,
    /// A message for the status bar: text, when, is an error.
    notice: Option<(String, Instant, bool)>,
    last_autosave: String,
    autosave_at: Instant,
    title: String,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>, opts: Opts) -> Self {
        theme::apply(&cc.egui_ctx);
        let ch = ChannelCount::new(16).unwrap();
        let mut app = Self {
            store: CaptureStore::shared(StoreConfig::new(ch, 100e6)),
            session: None,
            last_result: None,
            error: None,
            device: DeviceChoice::SimRealtime,
            channels_n: 16,
            rate: 100e6,
            pattern: Pattern::Generator,
            continuous: true,
            samples: 10e6,
            threshold: 1.7,
            stock_seen: (
                Instant::now() - Duration::from_secs(10),
                false,
                false,
                false,
            ),
            layout: Layout::generator(16),
            layout_touched: false,
            selected: None,
            scroll_y: 0.0,
            row_h: 30.0,
            row_auto: true,
            show_setup: false,
            show_keys: false,
            show_about: false,
            icon_texture: None,
            show_decoders: false,
            show_table: false,
            show_trigger: false,
            show_measure: false,
            trigger_probe: Default::default(),
            measure: Default::default(),
            zero_at_trigger: true,
            memory_mib: default_memory_mib(),
            post_target: None,
            trigger: crate::setup::TriggerSetup::default(),
            engine: Engine::new(),
            decoders: Vec::new(),
            setup: decode_ui::SetupState::default(),
            table: decode_ui::TableState::default(),
            selected_ann: None,
            new_bus: NewBus::default(),
            view: View::default(),
            follow: true,
            plot_width: 800.0,
            plot_height: 400.0,
            pending_fit: false,
            scale_text: String::new(),
            markers: Markers::default(),
            snap: true,
            drag_marker: None,
            row_drag: None,
            hover_sample: None,
            hover_edge: None,
            rate_stat: (Instant::now(), 0, 0.0),
            opts,
            t0: Instant::now(),
            autostarted: false,
            screenshot_requested: false,
            zoom_applied: false,
            goto_done: false,
            open_done: false,
            detect_pressed: false,
            save_done: false,
            file: None,
            last_dir: None,
            dialog: None,
            job: None,
            notice: None,
            last_autosave: String::new(),
            autosave_at: Instant::now(),
            title: String::new(),
        };
        app.engine.set_store(app.store.clone());
        app.restore_autosave();
        app
    }

    /// The sample that is time zero: the trigger if the capture has one and the user wants that.
    fn origin_of(&self, store: &CaptureStore) -> f64 {
        match store.trigger() {
            Some(t) if self.zero_at_trigger => t as f64,
            _ => 0.0,
        }
    }

    fn running(&self) -> bool {
        self.session.as_ref().is_some_and(|s| s.is_running())
    }

    /// The default channel setup for `n` channels of the selected device and source: the generator's
    /// signal names and buses, else CH0, CH1, ... The decoders keep their settings. Until the setup
    /// is changed again it follows the device and source.
    fn reset_layout(&mut self, n: usize) {
        self.layout = match (self.pattern, self.device) {
            (Pattern::Generator, DeviceChoice::Litex) => Layout::litex_generator(n),
            (Pattern::Generator, _) => Layout::generator(n),
            _ => Layout::new(n),
        };
        self.layout_touched = false;
        self.selected = None;
        let ids: Vec<u32> = self.decoders.iter().map(|e| e.handle.id.0).collect();
        self.layout.place_missing_decoders(&ids);
    }

    fn start(&mut self) {
        self.error = None;
        let Some(channels) = ChannelCount::new(self.channels_n) else {
            return;
        };
        let n = self.channels_n as usize;
        if !self.layout_touched || self.layout.names.len() != n {
            self.reset_layout(n);
        }

        // With a trigger the capture length is decided here (pre + post around the trigger), the
        // device just streams.
        let plan = match self.trigger_plan(n) {
            Ok(p) => p,
            Err(e) => {
                self.error = Some(e);
                return;
            }
        };
        let triggered = plan.is_some();
        self.post_target = plan.as_ref().and_then(|p| p.post);
        self.file = None;
        let mut scfg = StoreConfig::new(channels, self.rate);
        scfg.max_bytes = (self.memory_mib as usize) << 20;
        self.store = CaptureStore::shared(scfg);
        self.engine.set_store(self.store.clone());
        self.selected_ann = None;
        let cfg = CaptureConfig {
            channels,
            sample_rate: self.rate,
            limit_samples: (!self.continuous && !triggered).then_some(self.samples as u64),
            pattern: self.pattern,
            threshold_v: self.threshold,
        };
        let mut dev = match self.device.open() {
            Ok(d) => d,
            Err(e) => {
                self.error = Some(e);
                return;
            }
        };
        match Session::start_triggered(dev.as_mut(), &cfg, self.store.clone(), plan) {
            Ok(s) => {
                self.session = Some(s);
                self.last_result = None;
                self.follow = self.continuous || triggered;
                self.pending_fit = !self.continuous;
                self.markers = Markers::default();
                self.view = View {
                    start: 0.0,
                    spp: (self.rate / 1e3 / self.plot_width.max(1.0)).max(1.0 / 64.0),
                };
            }
            Err(e) => self.error = Some(e.to_string()),
        }
    }

    /// The trigger of the next capture, or what is wrong with its settings.
    fn trigger_plan(&self, channels: usize) -> Result<Option<TriggerPlan>, String> {
        if !self.trigger.enabled {
            return Ok(None);
        }
        let pre = self.trigger.pre.samples(self.rate);
        let post = (!self.continuous).then(|| self.trigger.post.samples(self.rate).max(1));
        let kind = match self
            .trigger_protocol()
            .build(self.trigger.mode)
            .map_err(|e| format!("trigger: {e}"))?
        {
            Some(p) => {
                let cfg = match &p {
                    bf_decode::trigger::ProtocolTrigger::Uart(c, _) => Config::Uart(c.clone()),
                    bf_decode::trigger::ProtocolTrigger::Spi(c, _) => Config::Spi(c.clone()),
                    bf_decode::trigger::ProtocolTrigger::I2c(c, _) => Config::I2c(c.clone()),
                };
                cfg.check_file_values(channels as u8)
                    .map_err(|e| format!("trigger: {e}"))?;
                TriggerKind::Event(
                    p.build(self.trigger.protocol.trigger_at)
                        .map_err(|e| format!("trigger: {e}"))?,
                )
            }
            None => TriggerKind::Pattern(Trigger::new(&self.trigger.conds[..channels.min(32)])),
        };
        Ok(Some(TriggerPlan { kind, pre, post }))
    }

    fn stop(&mut self) {
        if let Some(s) = &self.session {
            s.request_stop();
        }
    }

    fn fit(&mut self) {
        let r = self.store.read().range();
        self.view = View::fit(r, self.plot_width);
        self.follow = false;
    }

    fn poll_session(&mut self) {
        if self.session.as_ref().is_some_and(|s| !s.is_running()) {
            if let Some(s) = self.session.take() {
                let memory_full = s
                    .stats()
                    .memory_full
                    .load(std::sync::atomic::Ordering::SeqCst);
                let res = s.join();
                if memory_full {
                    self.notify(format!("stopped: the memory limit of {} MiB was reached (raise it in Setup > Trigger)", self.memory_mib), true);
                }
                if let Some(e) = &res.error {
                    self.error = Some(e.to_string());
                }
                if self.pending_fit {
                    self.fit();
                    self.pending_fit = false;
                }
                self.last_result = Some(res);
            }
        }
    }

    // ---------------------------------------------------------------- menus and bars

    fn menu_bar(&mut self, ui: &mut egui::Ui) {
        egui::menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| self.file_menu(ui));
            ui.menu_button("Setup", |ui| {
                if ui.button("Bus/Signal...").clicked() {
                    self.show_setup = true;
                    ui.close_menu();
                }
                if ui.button("Trigger...").clicked() {
                    self.show_trigger = true;
                    ui.close_menu();
                }
                if ui.button("Protocol decoders...").clicked() {
                    self.show_decoders = true;
                    ui.close_menu();
                }
                ui.separator();
                if ui
                    .button("Reset channel setup")
                    .on_hover_text("Default names, colors, buses and groups for the selected device and source (the signal names of the simulator's generator, else CH0, CH1, ...). Decoders stay. Save the setup first to keep the current one.")
                    .clicked()
                {
                    self.reset_layout(self.layout.names.len());
                    ui.close_menu();
                }
            });
            ui.menu_button("Markers", |ui| {
                let center = self.view.sample_at(self.plot_width / 2.0);
                if ui.button("Place M1 at view center").clicked() {
                    self.markers.set(Which::M1, center);
                    ui.close_menu();
                }
                if ui.button("Place M2 at view center").clicked() {
                    self.markers.set(Which::M2, center);
                    ui.close_menu();
                }
                if ui
                    .add_enabled(self.markers.m1.is_some(), egui::Button::new("Go to M1"))
                    .clicked()
                {
                    self.center_on(self.markers.m1.unwrap_or(0.0));
                    ui.close_menu();
                }
                if ui
                    .add_enabled(self.markers.m2.is_some(), egui::Button::new("Go to M2"))
                    .clicked()
                {
                    self.center_on(self.markers.m2.unwrap_or(0.0));
                    ui.close_menu();
                }
                let trig = self.store.read().trigger();
                if ui
                    .add_enabled(trig.is_some(), egui::Button::new("Go to trigger"))
                    .clicked()
                {
                    self.center_on(trig.unwrap_or(0) as f64);
                    ui.close_menu();
                }
                ui.separator();
                ui.checkbox(&mut self.snap, "Snap to edge");
                if ui.button("Clear markers").clicked() {
                    self.markers = Markers::default();
                    ui.close_menu();
                }
            });
            ui.menu_button("Run/Stop", |ui| {
                let running = self.running();
                if ui
                    .add_enabled(!running, egui::Button::new("Run").shortcut_text("F5"))
                    .clicked()
                {
                    self.start();
                    ui.close_menu();
                }
                if ui
                    .add_enabled(
                        running,
                        egui::Button::new("Stop").shortcut_text("F8 / Shift+F5"),
                    )
                    .clicked()
                {
                    self.stop();
                    ui.close_menu();
                }
            });
            ui.menu_button("View", |ui| {
                if ui.checkbox(&mut self.follow, "Follow live data").clicked() {
                    ui.close_menu();
                }
                if ui
                    .add(egui::Button::new("Fit capture").shortcut_text("Home / F"))
                    .clicked()
                {
                    self.fit();
                    ui.close_menu();
                }
                if ui.button("Zoom in").clicked() {
                    self.zoom_center(2.0);
                    ui.close_menu();
                }
                if ui.button("Zoom out").clicked() {
                    self.zoom_center(0.5);
                    ui.close_menu();
                }
                ui.separator();
                if ui
                    .add(egui::Button::new("Taller rows").shortcut_text("Ctrl+Up"))
                    .clicked()
                {
                    self.resize_rows(6.0);
                    ui.close_menu();
                }
                if ui
                    .add(egui::Button::new("Shorter rows").shortcut_text("Ctrl+Down"))
                    .clicked()
                {
                    self.resize_rows(-6.0);
                    ui.close_menu();
                }
                if ui.checkbox(&mut self.row_auto, "Auto row height").clicked() {
                    ui.close_menu();
                }
                ui.separator();
                if ui.checkbox(&mut self.show_table, "Decode table").clicked() {
                    ui.close_menu();
                }
                if ui
                    .checkbox(&mut self.show_measure, "Measurements")
                    .clicked()
                {
                    ui.close_menu();
                }
            });
            ui.menu_button("Help", |ui| {
                if ui.button("Mouse and keys...").clicked() {
                    self.show_keys = true;
                    ui.close_menu();
                }
                ui.separator();
                if ui.button("About Blinkfeuer...").clicked() {
                    self.show_about = true;
                    ui.close_menu();
                }
            });
        });
    }

    fn resize_rows(&mut self, delta: f32) {
        self.row_auto = false;
        self.row_h = (self.row_h + delta).clamp(14.0, 160.0);
    }

    fn center_on(&mut self, sample: f64) {
        self.view.start = sample - self.plot_width * self.view.spp / 2.0;
        self.follow = false;
    }

    /// Centre on `sample` only if it is outside the visible window.
    fn ensure_visible(&mut self, sample: f64) {
        if sample < self.view.start || sample > self.view.end(self.plot_width) {
            self.center_on(sample);
        }
    }

    fn view_center(&self) -> f64 {
        self.view.sample_at(self.plot_width / 2.0)
    }

    fn current_rows(&self) -> Vec<Row> {
        self.layout.rows_with(&|id| self.decoder_lanes(id))
    }

    /// Up/Down: select the row above or below (the row the cursor keys work on), scrolling it into view.
    fn select_row_step(&mut self, dir: i32) {
        let rows = self.current_rows();
        if rows.is_empty() {
            return;
        }
        let cur = self
            .selected
            .and_then(|k| rows.iter().position(|r| r.kind == k));
        let idx = match cur {
            Some(i) => (i as i64 + dir as i64).clamp(0, rows.len() as i64 - 1) as usize,
            None if dir > 0 => 0,
            None => rows.len() - 1,
        };
        self.selected = Some(rows[idx].kind);
        let (top, h) = (idx as f32 * self.row_h, self.row_h);
        if top < self.scroll_y {
            self.scroll_y = top;
        } else if top + h > self.scroll_y + self.plot_height {
            self.scroll_y = top + h - self.plot_height;
        }
    }

    /// Left/Right: on a decoder row go to the previous/next decoded item; on a signal, bus or group row
    /// (or with no row selected: any channel) move the marker to the previous/next edge.
    fn step(&mut self, dir: i32, which: Which) {
        let rows = self.current_rows();
        let row = self
            .selected
            .and_then(|k| rows.iter().find(|r| r.kind == k));
        if let Some(RowKind::Decode { id, lane }) = row.map(|r| r.kind) {
            self.step_item(id, lane, dir);
            return;
        }
        let mask = match row {
            Some(r) => self
                .layout
                .row_channels(r)
                .iter()
                .fold(0u32, |m, &c| m | 1 << c),
            None => self.store.read().config().channels.mask(),
        };
        let from = self
            .markers
            .get(which)
            .unwrap_or_else(|| self.view_center());
        let found = {
            let store = self.store.read();
            if dir > 0 {
                store.next_change(mask, from.max(0.0).floor() as u64)
            } else {
                store.prev_change(mask, from.max(0.0).ceil() as u64)
            }
        };
        match found {
            Some(p) => {
                self.markers.set(which, p as f64);
                self.selected_ann = None;
                self.ensure_visible(p as f64);
            }
            None => self.notify(
                if dir > 0 {
                    "no further edge"
                } else {
                    "no earlier edge"
                },
                false,
            ),
        }
    }

    /// The previous/next item of a decoder lane: after the selected item if it is on this lane, else
    /// after M1 (or the view centre).
    fn step_item(&mut self, id: u32, lane: u8, dir: i32) {
        let Some(anns) = self.entry(id).map(|e| e.handle.anns.clone()) else {
            return;
        };
        let pos = self.markers.m1.unwrap_or_else(|| self.view_center());
        let sel = self
            .selected_ann
            .filter(|s| s.id == id && s.lane == lane)
            .map(|s| s.start);
        let target = {
            let anns = anns.read();
            let Some(l) = anns.lanes.get(lane as usize) else {
                return;
            };
            let idx = match sel {
                Some(start) => {
                    let i = l.partition_point(|a| a.start < start);
                    if dir > 0 {
                        Some(i + 1)
                    } else {
                        i.checked_sub(1)
                    }
                }
                None if dir > 0 => Some(l.partition_point(|a| a.start as f64 <= pos)),
                None => l.partition_point(|a| (a.start as f64) < pos).checked_sub(1),
            };
            idx.and_then(|i| l.get(i).map(|a| (i, *a)))
        };
        match target {
            Some((i, a)) => {
                self.select_annotation(id, lane, a, i, false, false);
                self.ensure_visible((a.start + a.end) as f64 / 2.0);
            }
            None => self.notify(
                if dir > 0 {
                    "no further item"
                } else {
                    "no earlier item"
                },
                false,
            ),
        }
    }

    fn zoom_center(&mut self, factor: f64) {
        let anchor = if self.follow {
            self.plot_width
        } else {
            self.plot_width / 2.0
        };
        self.view.zoom_at(anchor, factor);
    }

    fn run_bar(&mut self, ui: &mut egui::Ui) {
        let running = self.running();
        // Look for an attached analyzer once a second while idle.
        if !running && self.stock_seen.0.elapsed() > Duration::from_secs(1) {
            self.stock_seen = (
                Instant::now(),
                !StockDevice::available().is_empty(),
                !LitexDevice::available().is_empty(),
                !Fx2lafwDevice::available().is_empty(),
            );
        }
        let (stock_present, litex_present, fx2_present) =
            (self.stock_seen.1, self.stock_seen.2, self.stock_seen.3);
        let dim = |t: &str| RichText::new(t.to_string()).color(theme::TEXT_DIM);
        ui.horizontal_wrapped(|ui| {
            let run = egui::Button::new(RichText::new("Run  F5").color(if running {
                theme::TEXT_DIM
            } else {
                theme::RUN
            }));
            if ui.add_enabled(!running, run).clicked() {
                self.start();
            }
            let stop = egui::Button::new(RichText::new("Stop F8").color(if running {
                theme::STOP
            } else {
                theme::TEXT_DIM
            }));
            if ui.add_enabled(running, stop).clicked() {
                self.stop();
            }
            ui.separator();
            ui.add_enabled_ui(!running, |ui| {
                ui.label(dim("Device"));
                let device_before = self.device;
                egui::ComboBox::from_id_salt("device")
                    .selected_text(self.device.label())
                    .show_ui(ui, |ui| {
                        for d in [
                            DeviceChoice::Stock,
                            DeviceChoice::Litex,
                            DeviceChoice::Fx2lafw,
                            DeviceChoice::SimRealtime,
                            DeviceChoice::Sim,
                        ] {
                            let present = match d {
                                DeviceChoice::Stock => stock_present,
                                DeviceChoice::Litex => litex_present,
                                DeviceChoice::Fx2lafw => fx2_present,
                                _ => true,
                            };
                            if present || self.device == d {
                                ui.selectable_value(&mut self.device, d, d.label());
                            }
                        }
                    });
                // Another device starts with its own default pattern: the simulator with the generator
                // (its "inputs" are idle lines), an analyzer with its inputs.
                if self.device != device_before || !self.device.patterns().contains(&self.pattern) {
                    self.pattern = self.device.patterns()[0];
                }
                let channel_options = self.device.channel_options();
                if !channel_options.contains(&self.channels_n) {
                    self.channels_n = channel_options[0];
                }
                ui.label(dim("Channels"));
                egui::ComboBox::from_id_salt("channels")
                    .selected_text(self.channels_n.to_string())
                    .show_ui(ui, |ui| {
                        for &n in channel_options {
                            ui.selectable_value(&mut self.channels_n, n, n.to_string());
                        }
                    });
                let (rates, max) = self.device.rates(self.channels_n);
                let unreachable = self.device == DeviceChoice::Fx2lafw
                    && fx2lafw::start_command(self.rate).is_err();
                if self.rate > max || unreachable {
                    // The fastest offered rate not above the current one (or the slowest).
                    self.rate = rates
                        .iter()
                        .copied()
                        .rfind(|&r| r <= self.rate.min(max))
                        .unwrap_or(rates[0]);
                }
                ui.label(dim("Sample rate"));
                egui::ComboBox::from_id_salt("rate")
                    .selected_text(format_hz(self.rate))
                    .show_ui(ui, |ui| {
                        for &r in rates {
                            if r <= max {
                                ui.selectable_value(&mut self.rate, r, format_hz(r));
                            }
                        }
                    });
                ui.label(dim("Source"));
                egui::ComboBox::from_id_salt("pattern")
                    .selected_text(self.pattern.name())
                    .show_ui(ui, |ui| {
                        for &p in self.device.patterns() {
                            ui.selectable_value(&mut self.pattern, p, p.name());
                        }
                    });
                if self.device.has_threshold() {
                    ui.label(dim("Threshold"));
                    ui.add(
                        egui::DragValue::new(&mut self.threshold)
                            .speed(0.02)
                            .range(0.0..=3.3)
                            .suffix(" V")
                            .fixed_decimals(2),
                    );
                }
                ui.label(dim("Mode"));
                egui::ComboBox::from_id_salt("mode")
                    .selected_text(if self.continuous {
                        "Continuous"
                    } else {
                        "Single"
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.continuous, true, "Continuous");
                        ui.selectable_value(&mut self.continuous, false, "Single");
                    });
                if !self.continuous && !self.trigger.enabled {
                    ui.add(
                        egui::DragValue::new(&mut self.samples)
                            .speed(1e5)
                            .range(1e3..=4e9)
                            .suffix(" Sa"),
                    )
                    .on_hover_text("Samples to capture");
                }
                ui.separator();
                ui.checkbox(&mut self.trigger.enabled, "Trigger")
                    .on_hover_text("Wait for a trigger condition (Setup > Trigger...)");
                if ui.button("...").on_hover_text("Trigger setup").clicked() {
                    self.show_trigger = true;
                }
                if self.trigger.enabled {
                    let names = self.layout.names.clone();
                    let window = if self.continuous {
                        format!("{} before, until Stop", self.trigger.pre.text())
                    } else {
                        format!(
                            "{} before, {} after",
                            self.trigger.pre.text(),
                            self.trigger.post.text()
                        )
                    };
                    ui.label(RichText::new(self.trigger.summary(&names)).color(theme::DEC_WARN));
                    ui.label(RichText::new(window).color(theme::TEXT_DIM));
                }
            });
            if let Some(s) = self
                .session
                .as_ref()
                .filter(|s| matches!(s.stats().phase(), Phase::Filling | Phase::Waiting))
            {
                if ui
                    .button(RichText::new("Force trigger").color(theme::DEC_WARN))
                    .on_hover_text("Trigger now, whatever the condition")
                    .clicked()
                {
                    s.force_trigger();
                }
            }
        });
    }

    fn marker_bar(&mut self, ui: &mut egui::Ui) {
        let (rate, origin) = {
            let s = self.store.read();
            (s.sample_rate(), self.origin_of(&s))
        };
        let chip = |ui: &mut egui::Ui, name: &str, color: Color32| {
            ui.label(
                RichText::new(format!(" {name} "))
                    .color(Color32::BLACK)
                    .background_color(color)
                    .monospace(),
            );
        };
        let pos_text = |p: Option<f64>| match p {
            Some(s) => format_split((s - origin) / rate, 1.0 / rate),
            None => "----".to_string(),
        };
        ui.horizontal(|ui| {
            chip(ui, "M1", theme::M1);
            ui.label(format!("= {}", pos_text(self.markers.m1)));
            ui.add_space(8.0);
            chip(ui, "M2", theme::M2);
            ui.label(format!("= {}", pos_text(self.markers.m2)));
            ui.add_space(8.0);
            chip(ui, "M1", theme::M1);
            ui.label("to");
            chip(ui, "M2", theme::M2);
            match delta_text(&self.markers, rate) {
                Some((t, f)) => {
                    ui.label(format!("= {t}"));
                    if let Some(f) = f {
                        ui.label(RichText::new(format!("(1/dt = {f})")).color(theme::TEXT_DIM));
                    }
                    if let Some(d) = self.markers.delta_samples() {
                        ui.label(
                            RichText::new(format!("{:.0} Sa", d.abs())).color(theme::TEXT_DIM),
                        );
                    }
                }
                None => {
                    ui.label("= ----");
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("Clear").clicked() {
                    self.markers = Markers::default();
                }
                ui.checkbox(&mut self.snap, "Snap to edge");
            });
        });
    }

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        let selected_text = self.selected_text();
        let job_label = self.job_label().map(str::to_string);
        let notice = self.notice.clone();
        let store = self.store.read();
        let origin = self.origin_of(&store);
        ui.horizontal_centered(|ui| {
            if let Some(s) = &self.session {
                let bytes = s.stats().bytes.load(std::sync::atomic::Ordering::Relaxed);
                let now = Instant::now();
                if now.duration_since(self.rate_stat.0) > Duration::from_millis(500) {
                    let dt = now.duration_since(self.rate_stat.0).as_secs_f64();
                    self.rate_stat = (
                        now,
                        bytes,
                        bytes.saturating_sub(self.rate_stat.1) as f64 / dt / 1e6,
                    );
                }
                match s.stats().phase() {
                    Phase::Filling => {
                        ui.label(RichText::new("PRE-TRIGGER").color(theme::DEC_WARN).strong())
                    }
                    Phase::Waiting => ui.label(
                        RichText::new("WAITING FOR TRIGGER")
                            .color(theme::DEC_WARN)
                            .strong(),
                    ),
                    Phase::Triggered => {
                        let done = store.trigger().zip(self.post_target).map(|(t, post)| {
                            100.0 * store.end().saturating_sub(t) as f64 / post.max(1) as f64
                        });
                        ui.label(
                            RichText::new(match done {
                                Some(p) => format!("TRIGGERED {:.0} %", p.min(100.0)),
                                None => "TRIGGERED".to_string(),
                            })
                            .color(theme::RUN)
                            .strong(),
                        )
                    }
                    Phase::Free => ui.label(RichText::new("RUNNING").color(theme::RUN).strong()),
                };
                ui.label(format!("{} Sa   {:.1} MB/s", store.end(), self.rate_stat.2));
            } else if let Some(r) = &self.last_result {
                ui.label(RichText::new("STOPPED").color(theme::TEXT_DIM).strong());
                ui.label(format!(
                    "{} Sa in {:.2} s",
                    r.samples,
                    r.elapsed.as_secs_f64()
                ));
            } else if let Some(f) = self.file.as_ref().and_then(|f| f.file_name()) {
                ui.label(RichText::new("FILE").color(theme::TEXT_DIM).strong());
                ui.label(f.to_string_lossy().into_owned());
            } else {
                ui.label(RichText::new("IDLE").color(theme::TEXT_DIM).strong());
            }
            ui.separator();
            ui.label(format!(
                "{}  {} ch",
                format_hz(store.sample_rate()),
                store.channels()
            ));
            if let Some(t) = store.trigger() {
                ui.separator();
                let rate = store.sample_rate();
                ui.label(
                    RichText::new(format!(
                        "trigger at {} from start",
                        format_split(t as f64 / rate, 1.0 / rate)
                    ))
                    .color(theme::DEC_WARN),
                );
            }
            ui.separator();
            ui.label(format!(
                "memory {:.1} MB ({} events)",
                store.memory_bytes() as f64 / 1e6,
                store.event_count()
            ));
            if let Some(e) = &self.error {
                ui.separator();
                ui.colored_label(theme::STOP, e);
            }
            if let Some(label) = &job_label {
                ui.separator();
                ui.spinner();
                ui.label(RichText::new(format!("{label}...")).color(theme::SNAP));
            } else if let Some((msg, at, err)) = &notice {
                if at.elapsed() < Duration::from_secs(12) {
                    ui.separator();
                    ui.add(
                        egui::Label::new(RichText::new(msg.as_str()).color(if *err {
                            theme::STOP
                        } else {
                            theme::RUN
                        }))
                        .truncate(),
                    );
                }
            }
            if let Some(text) = &selected_text {
                ui.separator();
                ui.add(
                    egui::Label::new(RichText::new(text.as_str()).color(theme::DEC_ADDR))
                        .truncate(),
                );
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let rate = store.sample_rate();
                if let Some(s) = self.hover_sample {
                    ui.label(format!(
                        "cursor {}",
                        format_split((s - origin) / rate, 1.0 / rate)
                    ));
                }
                if let Some(e) = self.hover_edge {
                    ui.separator();
                    ui.label(
                        RichText::new(format!(
                            "edge {}",
                            format_split((e as f64 - origin) / rate, 1.0 / rate)
                        ))
                        .color(theme::SNAP),
                    );
                }
            });
        });
    }

    fn overview(&mut self, ui: &mut egui::Ui) {
        let (rect, resp) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());
        let painter = ui.painter_at(rect);
        let store = self.store.read();
        let rate = store.sample_rate();
        let range = store.range();
        let strip = Rect::from_min_max(
            pos2(rect.left() + GUTTER, rect.top() + 7.0),
            pos2(rect.right() - 6.0, rect.bottom() - 7.0),
        );
        painter.text(
            pos2(rect.left() + 8.0, rect.top() + 6.0),
            Align2::LEFT_TOP,
            "Overview",
            FontId::monospace(11.0),
            theme::TEXT,
        );
        if !store.is_empty() {
            painter.text(
                pos2(rect.left() + 8.0, rect.top() + 22.0),
                Align2::LEFT_TOP,
                format_split((range.end - range.start) as f64 / rate, 1.0 / rate),
                FontId::monospace(10.0),
                theme::TEXT_DIM,
            );
        }
        painter.rect_filled(strip, 0.0, Color32::BLACK);
        painter.rect_stroke(strip, 0.0, Stroke::new(1.0_f32, theme::BORDER));
        let len = (range.end.saturating_sub(range.start)).max(1) as f64;
        let x_of = |s: f64| strip.left() + ((s - range.start as f64) / len) as f32 * strip.width();
        if !store.is_empty() {
            painter.rect_filled(strip.shrink(1.0), 0.0, Color32::from_gray(0x16));
            // The part of the capture shown in the waveform window.
            let w0 = x_of(self.view.start).clamp(strip.left(), strip.right());
            let w1 = x_of(self.view.end(self.plot_width))
                .clamp(strip.left(), strip.right())
                .max(w0 + 2.0);
            let win =
                Rect::from_min_max(pos2(w0, strip.top() + 1.0), pos2(w1, strip.bottom() - 1.0));
            painter.rect_filled(win, 0.0, Color32::from_white_alpha(0x28));
            painter.rect_stroke(win, 0.0, Stroke::new(1.0_f32, theme::TRACE));
            for (p, c) in [
                (store.trigger().map(|t| t as f64), theme::DEC_WARN),
                (self.markers.m1, theme::M1),
                (self.markers.m2, theme::M2),
            ] {
                if let Some(p) = p {
                    let x = x_of(p).clamp(strip.left(), strip.right());
                    painter.line_segment(
                        [pos2(x, strip.top()), pos2(x, strip.bottom())],
                        Stroke::new(2.0_f32, c),
                    );
                }
            }
        }
        if let Some(p) = resp.interact_pointer_pos() {
            if (resp.clicked() || resp.dragged())
                && strip.expand2(vec2(200.0, 10.0)).contains(p)
                && !store.is_empty()
            {
                let f = ((p.x - strip.left()) / strip.width()).clamp(0.0, 1.0) as f64;
                let s = range.start as f64 + f * len;
                self.view.start = s - self.plot_width * self.view.spp / 2.0;
                self.follow = false;
            }
        }
    }

    // ---------------------------------------------------------------- waveform window

    fn scale_header(&mut self, ui: &mut egui::Ui, rate: f64, origin: f64, has_trigger: bool) {
        egui::Frame::none().fill(theme::CHROME).inner_margin(Margin::symmetric(6.0, 3.0)).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.horizontal(|ui| {
                ui.label(RichText::new("Scale").color(theme::TEXT_DIM));
                let r = ui.add(egui::TextEdit::singleline(&mut self.scale_text).desired_width(74.0));
                ui.label(RichText::new("/div").color(theme::TEXT_DIM));
                if !r.has_focus() {
                    self.scale_text = format_scale(scale_step(&self.view, rate));
                }
                if r.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter)) {
                    if let Some(step) = parse_time(&self.scale_text) {
                        let spp = spp_for_step(step, rate);
                        let anchor = if self.follow { self.plot_width } else { self.plot_width / 2.0 };
                        self.view.zoom_at(anchor, self.view.spp / spp);
                    }
                }
                if ui.button("-").on_hover_text("Zoom out").clicked() {
                    self.zoom_center(0.5);
                }
                if ui.button("+").on_hover_text("Zoom in").clicked() {
                    self.zoom_center(2.0);
                }
                if ui.button("Fit").on_hover_text("Show the whole capture (Home)").clicked() {
                    self.fit();
                }
                if ui.selectable_label(self.follow, "Follow").on_hover_text("Keep the newest data at the right edge").clicked() {
                    self.follow = !self.follow;
                }
                ui.separator();
                ui.label(RichText::new("Height").color(theme::TEXT_DIM));
                if ui.selectable_label(self.row_auto, "Auto").on_hover_text("Fit the rows to the window").clicked() {
                    self.row_auto = !self.row_auto;
                }
                let r = ui.add(egui::DragValue::new(&mut self.row_h).speed(0.5).range(14.0..=160.0).suffix(" px").fixed_decimals(0));
                if r.changed() {
                    self.row_auto = false;
                }
                ui.separator();
                let t0 = (self.view.start.max(0.0) - origin) / rate;
                let span = self.plot_width * self.view.spp / rate;
                ui.label(
                    RichText::new(format!(
                        "window {} .. {}",
                        format_split(t0, 1.0 / rate),
                        format_split(t0 + span, 1.0 / rate)
                    ))
                    .color(theme::TEXT_DIM),
                );
                ui.separator();
                ui.add_enabled(has_trigger, egui::Checkbox::new(&mut self.zero_at_trigger, "t = 0 at trigger"))
                    .on_hover_text("Count all times from the trigger sample (negative before it) instead of the start of the capture");
            });
        });
    }

    fn value_text(&self, store: &CaptureStore, row: &Row, at: Option<f64>) -> String {
        let Some(s) = at else { return "-".into() };
        let sample = s.round().max(0.0) as u64;
        if let RowKind::Decode { id, lane } = row.kind {
            return self.decoder_text_at(id, lane, sample);
        }
        let Some(state) = store.state_at(sample) else {
            return "-".into();
        };
        match row.kind {
            RowKind::Signal(ch) => ((state >> ch) & 1).to_string(),
            RowKind::Bus(i) => match self.layout.bus(i) {
                Some(b) => format_bus(bus_value(state, &b.members), b.members.len(), b.radix),
                None => "-".into(),
            },
            RowKind::Decode { .. } | RowKind::Group(_) => "-".into(),
        }
    }

    /// `UART Telegram: <text>` for the selected decoded item.
    fn selected_text(&self) -> Option<String> {
        let sel = self.selected_ann?;
        let e = self.entry(sel.id)?;
        let anns = e.handle.anns.read();
        let lane = anns.lanes.get(sel.lane as usize)?;
        let a = lane.get(lane.partition_point(|a| a.start < sel.start))?;
        if a.start != sel.start {
            return None;
        }
        let text = if a.kind == bf_decode::Kind::Telegram {
            let chars = anns.lanes.first().map_or(&[][..], |v| v.as_slice());
            bf_decode::telegram_text(bf_decode::telegram_chars(chars, a), e.format, 200)
        } else {
            a.label(e.format)
        };
        Some(format!(
            "{} {}: {}",
            e.name, e.handle.lanes[sel.lane as usize], text
        ))
    }

    fn entry(&self, id: u32) -> Option<&Entry> {
        self.decoders.iter().find(|e| e.handle.id.0 == id)
    }

    /// The lanes of decoder `id` that have a row (the used ones).
    fn decoder_lanes(&self, id: u32) -> Vec<u8> {
        self.entry(id)
            .map(|e| {
                (0..e.handle.lanes.len())
                    .filter(|&l| e.handle.lane_used.get(l).copied().unwrap_or(false))
                    .map(|l| l as u8)
                    .collect()
            })
            .unwrap_or_default()
    }

    fn row_label(&self, row: &Row) -> String {
        match row.kind {
            RowKind::Decode { id, lane } => self
                .entry(id)
                .map(|e| match e.handle.lanes {
                    [_] => e.name.clone(),
                    lanes => format!("{} {}", e.name, lanes[lane as usize]),
                })
                .unwrap_or_default(),
            _ => self.layout.row_label(row),
        }
    }

    /// Id and settings of every decoder (what a trigger can take its settings from).
    fn decoder_configs(&self) -> Vec<(u32, Config)> {
        self.decoders
            .iter()
            .map(|e| (e.handle.id.0, e.cfg.clone()))
            .collect()
    }

    /// The protocol trigger settings with the linked decoders' settings put in.
    fn trigger_protocol(&self) -> crate::setup::ProtocolSetup {
        self.trigger.protocol.resolved(&self.decoder_configs())
    }

    /// Text of the annotation covering `sample` in a decoder lane (for the marker columns).
    fn decoder_text_at(&self, id: u32, lane: u8, sample: u64) -> String {
        let Some(e) = self.entry(id) else {
            return "-".into();
        };
        let anns = e.handle.anns.read();
        let l = anns.visible(lane as usize, sample..sample + 1);
        l.first().map_or("-".into(), |a| {
            if a.kind == bf_decode::Kind::Telegram {
                let chars = anns.lanes.first().map_or(&[][..], |v| v.as_slice());
                bf_decode::telegram_text(bf_decode::telegram_chars(chars, a), e.format, 60)
            } else {
                a.label(e.format)
            }
        })
    }

    fn add_decoder(&mut self, cfg: Config) {
        let handle = self.engine.add(cfg.build());
        let name = cfg.kind().label().to_string();
        // UART output is mostly text: show characters and telegrams as ASCII by default (binary
        // protocols such as Modbus in hex).
        let format = if is_text_uart(&cfg) {
            Format::Ascii
        } else {
            Format::Hex
        };
        self.layout.add_decoder(handle.id.0);
        self.decoders.push(Entry {
            handle,
            cfg,
            name,
            format,
            color: crate::rows::DEFAULT_COLOR,
        });
    }

    /// Apply changed settings: the decoder starts over with its new configuration.
    fn rebuild_decoder(&mut self, i: usize) {
        let old = self.decoders[i].handle.id;
        self.engine.remove(old);
        self.decoders[i].handle = self.engine.add(self.decoders[i].cfg.build());
        // A UART switched to a binary protocol: ASCII would show its bytes as garbage.
        if !is_text_uart(&self.decoders[i].cfg) && self.decoders[i].format == Format::Ascii {
            self.decoders[i].format = Format::Hex;
        }
        let new = self.decoders[i].handle.id.0;
        self.layout.replace_decoder(old.0, new);
        self.trigger
            .protocol
            .map_sources(&|id| Some(if id == old.0 { new } else { id }));
        self.selected_ann = None;
    }

    /// Select the decoded item under a click in a decoder row. False if the click was elsewhere.
    fn click_annotation(&mut self, p: egui::Pos2, plot: Rect, rows: &[Row], row_h: f32) -> bool {
        if p.y < plot.top() {
            return false;
        }
        let idx = ((p.y - plot.top() + self.scroll_y) / row_h).floor();
        let Some(RowKind::Decode { id, lane }) = (idx >= 0.0)
            .then(|| rows.get(idx as usize).map(|r| r.kind))
            .flatten()
        else {
            return false;
        };
        let s = self.view.sample_at((p.x - plot.left()) as f64);
        let tol = 3.0 * self.view.spp;
        let found = self.entry(id).and_then(|e| {
            let anns = e.handle.anns.read();
            let l = anns.lanes.get(lane as usize)?;
            let at = s.max(0.0) as u64;
            let i = l.partition_point(|a| a.end <= at);
            // The item containing the click, else the nearest one within a few pixels.
            let dist = |a: &bf_decode::Annotation| {
                if (a.start as f64) > s {
                    a.start as f64 - s
                } else if (a.end as f64) <= s {
                    s - a.end as f64
                } else {
                    0.0
                }
            };
            [i.checked_sub(1), Some(i)]
                .into_iter()
                .flatten()
                .filter_map(|k| l.get(k).map(|a| (k, *a)))
                .map(|(k, a)| (dist(&a), k, a))
                .filter(|(d, _, _)| *d <= tol)
                .min_by(|x, y| x.0.total_cmp(&y.0))
                .map(|(_, k, a)| (k, a))
        });
        match found {
            Some((i, a)) => {
                self.select_annotation(id, lane, a, i, false, false);
                true
            }
            None => false,
        }
    }

    /// Make `a` (item `idx` of a decoder lane) the selection: highlight it, put the markers on it and
    /// show it in the table. `recenter` also moves the waveform to it; `from_table` means the table
    /// already shows it (no scrolling needed).
    fn select_annotation(
        &mut self,
        id: u32,
        lane: u8,
        a: bf_decode::Annotation,
        idx: usize,
        recenter: bool,
        from_table: bool,
    ) {
        self.selected_ann = Some(decode_ui::Selected {
            id,
            lane,
            start: a.start,
        });
        self.markers = Markers {
            m1: Some(a.start as f64),
            m2: Some(a.end as f64),
        };
        if let Some(ti) = self.decoders.iter().position(|e| e.handle.id.0 == id) {
            self.table.entry = ti;
            self.table.lane = lane as usize;
        }
        if !from_table {
            self.table.scroll_to = Some(idx);
        }
        if recenter {
            self.focus_annotation(&a);
        }
    }

    /// Bring `a` into view by centring it. The zoom is left alone.
    fn focus_annotation(&mut self, a: &bf_decode::Annotation) {
        self.center_on((a.start + a.end) as f64 / 2.0);
    }

    /// Sample to snap to near `s` for the row at screen height `y` (or the selected row).
    fn snap_target(
        &self,
        store: &CaptureStore,
        s: f64,
        y: f32,
        plot: Rect,
        rows: &[Row],
        row_h: f32,
    ) -> Option<(u64, usize)> {
        let idx = if y >= plot.top() {
            let i = ((y - plot.top() + self.scroll_y) / row_h).floor();
            (i >= 0.0).then_some(i as usize)
        } else {
            self.selected
                .and_then(|k| rows.iter().position(|r| r.kind == k))
        }?;
        let row = rows.get(idx)?;
        let p = snap_to_edge(
            store,
            &self.layout.row_channels(row),
            s,
            8.0 * self.view.spp,
        )?;
        Some((p, idx))
    }

    #[allow(clippy::too_many_arguments)]
    fn place_marker(
        &mut self,
        store: &CaptureStore,
        w: Which,
        x: f32,
        y: f32,
        plot: Rect,
        rows: &[Row],
        row_h: f32,
    ) {
        let mut s = self.view.sample_at((x - plot.left()) as f64);
        if self.snap && self.row_drag.is_none() && self.opts.dragging.is_none() {
            if let Some((p, _)) = self.snap_target(store, s, y, plot, rows, row_h) {
                s = p as f64;
            }
        }
        self.markers
            .set(w, s.clamp(store.first_sample() as f64, store.end() as f64));
    }

    fn waveform_panel(&mut self, ui: &mut egui::Ui) {
        let shared = self.store.clone();
        let store = shared.read();
        let rate = store.sample_rate();
        let (origin, has_trigger) = (self.origin_of(&store), store.trigger().is_some());
        self.scale_header(ui, rate, origin, has_trigger);

        let (rect, _) = ui.allocate_exact_size(ui.available_size(), Sense::hover());
        let painter = ui.painter_at(rect);
        let ruler = Rect::from_min_size(
            pos2(rect.left() + GUTTER, rect.top()),
            vec2(rect.width() - GUTTER, RULER_H),
        );
        let plot = Rect::from_min_max(
            pos2(rect.left() + GUTTER, rect.top() + RULER_H),
            rect.right_bottom(),
        );
        let gutter = Rect::from_min_max(
            pos2(rect.left(), plot.top()),
            pos2(rect.left() + GUTTER, rect.bottom()),
        );
        self.plot_width = plot.width() as f64;
        self.plot_height = plot.height();

        let rows = self.layout.rows_with(&|id| self.decoder_lanes(id));
        let row_h = if self.row_auto {
            let auto = if rows.is_empty() {
                24.0
            } else {
                (plot.height() / rows.len() as f32).clamp(18.0, 30.0)
            };
            self.row_h = auto; // shown in the header, and the starting point for manual adjustment
            auto
        } else {
            self.row_h
        };
        let max_scroll = (row_h * rows.len() as f32 - plot.height()).max(0.0);
        let pointer = self
            .opts
            .hover
            .map(|(x, y)| pos2(plot.left() + x, plot.top() + y))
            .or(ui.input(|i| i.pointer.hover_pos()));
        if pointer.is_some_and(|p| gutter.contains(p)) {
            self.scroll_y -= ui.input(|i| i.smooth_scroll_delta.y);
        }
        self.scroll_y = self.scroll_y.clamp(0.0, max_scroll);

        // ---- interaction
        let plot_resp = ui.interact(plot, ui.id().with("plot"), Sense::click_and_drag());
        let ruler_resp = ui.interact(ruler, ui.id().with("ruler"), Sense::click_and_drag());
        let hovered = plot_resp.hovered() || ruler_resp.hovered();
        if hovered {
            let (scroll, zoom_delta, shift) =
                ui.input(|i| (i.smooth_scroll_delta, i.zoom_delta(), i.modifiers.shift));
            let hover_x = pointer.map_or(0.0, |p| (p.x - plot.left()) as f64);
            // Zoom around the pointer, except while following live data: then the newest sample
            // stays at the right edge.
            let live = self.follow && self.running();
            let anchor = if live { self.plot_width } else { hover_x };
            let mut factor = zoom_delta as f64;
            if shift || scroll.x.abs() > scroll.y.abs() {
                let dx = if scroll.x.abs() > scroll.y.abs() {
                    scroll.x
                } else {
                    scroll.y
                };
                if dx != 0.0 {
                    self.view.pan_px(dx as f64);
                    self.follow = false;
                }
            } else if scroll.y != 0.0 {
                factor *= (scroll.y as f64 * 0.004).exp();
            }
            if (factor - 1.0).abs() > 1e-9 {
                self.view.zoom_at(anchor, factor);
                if !live {
                    // Nothing new to follow: keep the view where the zoom put it.
                    self.follow = false;
                }
            }
        }
        let shift = ui.input(|i| i.modifiers.shift);
        for (resp, in_ruler) in [(&ruler_resp, true), (&plot_resp, false)] {
            if resp.drag_started() {
                if let Some(p) = resp.interact_pointer_pos() {
                    let s = self.view.sample_at((p.x - plot.left()) as f64);
                    self.drag_marker = self.markers.hit(s, 7.0 * self.view.spp);
                    if self.drag_marker.is_none() && in_ruler {
                        self.drag_marker = Some(if shift { Which::M2 } else { Which::M1 });
                    }
                }
            }
            if resp.dragged() {
                match (self.drag_marker, resp.interact_pointer_pos()) {
                    (Some(w), Some(p)) => {
                        self.place_marker(&store, w, p.x, p.y, plot, &rows, row_h)
                    }
                    _ => {
                        self.view.pan_px(resp.drag_delta().x as f64);
                        self.follow = false;
                    }
                }
            }
            if resp.drag_stopped() {
                self.drag_marker = None;
            }
            if let Some(p) = resp.interact_pointer_pos() {
                if resp.clicked() {
                    // The clicked row becomes the one the cursor keys work on.
                    if !in_ruler {
                        let i = ((p.y - plot.top() + self.scroll_y) / row_h).floor();
                        if let Some(r) = (i >= 0.0).then(|| rows.get(i as usize)).flatten() {
                            self.selected = Some(r.kind);
                        }
                    }
                    // A click on a decoded item selects it (and shows it in the table); elsewhere it places a marker.
                    let picked = !in_ruler && self.click_annotation(p, plot, &rows, row_h);
                    if !picked {
                        self.place_marker(
                            &store,
                            if shift { Which::M2 } else { Which::M1 },
                            p.x,
                            p.y,
                            plot,
                            &rows,
                            row_h,
                        );
                    }
                } else if resp.secondary_clicked() {
                    self.place_marker(&store, Which::M2, p.x, p.y, plot, &rows, row_h);
                }
            }
        }
        if plot_resp.double_clicked() {
            self.fit();
        }
        if let Some(p) = pointer.filter(|p| plot.contains(*p) || ruler.contains(*p)) {
            let s = self.view.sample_at((p.x - plot.left()) as f64);
            if self.markers.hit(s, 7.0 * self.view.spp).is_some() || self.drag_marker.is_some() {
                ui.ctx().set_cursor_icon(CursorIcon::ResizeHorizontal);
            }
            self.hover_sample = Some(s).filter(|s| *s >= 0.0 && *s <= store.end() as f64);
        } else {
            self.hover_sample = None;
        }
        if self.follow && !store.is_empty() {
            self.view.follow(store.end(), self.plot_width);
        }
        self.view.clamp(store.range(), self.plot_width);

        // ---- drawing
        painter.rect_filled(rect, 0.0, theme::CHROME_DARK);
        painter.rect_filled(plot, 0.0, theme::PLOT_BG);
        painter.rect_filled(ruler, 0.0, theme::PLOT_BG);
        let plot_p = painter.with_clip_rect(plot);
        draw_grid(&plot_p, plot, &self.view, rate, origin);
        draw_ruler(
            &painter.with_clip_rect(ruler),
            ruler,
            &self.view,
            rate,
            origin,
        );

        // Column headers.
        let head = Rect::from_min_size(rect.left_top(), vec2(GUTTER, RULER_H));
        painter.text(
            pos2(head.left() + 8.0, head.center().y),
            Align2::LEFT_CENTER,
            "Bus/Signal",
            FontId::monospace(11.0),
            theme::TEXT_DIM,
        );
        for (k, (name, color)) in [("M1", theme::M1), ("M2", theme::M2)]
            .into_iter()
            .enumerate()
        {
            painter.text(
                pos2(
                    head.left() + NAME_W + VAL_W * (k as f32 + 1.0) - 6.0,
                    head.center().y,
                ),
                Align2::RIGHT_CENTER,
                name,
                FontId::monospace(11.0),
                color,
            );
        }
        painter.line_segment(
            [
                pos2(rect.left(), ruler.bottom()),
                pos2(ruler.left(), ruler.bottom()),
            ],
            Stroke::new(1.0_f32, theme::BORDER),
        );
        painter.line_segment(
            [
                pos2(plot.left(), rect.top()),
                pos2(plot.left(), rect.bottom()),
            ],
            Stroke::new(1.0_f32, theme::BORDER),
        );

        let gutter_p = painter.with_clip_rect(gutter);
        let mut toggle: Option<usize> = None;
        let mut select: Option<RowKind> = None;
        let font = FontId::monospace(12.0);
        let small = FontId::monospace(11.0);
        for (i, row) in rows.iter().enumerate() {
            let top = plot.top() - self.scroll_y + i as f32 * row_h;
            if top + row_h < plot.top() || top > plot.bottom() {
                continue;
            }
            let r = Rect::from_min_size(pos2(plot.left(), top), vec2(plot.width(), row_h));
            let label = Rect::from_min_size(pos2(rect.left(), top), vec2(GUTTER, row_h));
            let is_sel = self.selected == Some(row.kind);
            if is_sel {
                gutter_p.rect_filled(label, 0.0, theme::SELECT);
                plot_p.rect_filled(r, 0.0, Color32::from_rgb(0x07, 0x10, 0x1c));
            }
            if matches!(row.kind, RowKind::Group(_)) && !is_sel {
                gutter_p.rect_filled(label, 0.0, theme::GROUP_BG);
                plot_p.rect_filled(r, 0.0, Color32::from_rgb(0x0a, 0x0c, 0x0f));
            }
            let indent = row.depth as f32 * 16.0;
            if let RowKind::Bus(item) | RowKind::Group(item) = row.kind {
                let expanded = self.layout.is_expanded(item);
                let box_r =
                    Rect::from_center_size(pos2(label.left() + 12.0, r.center().y), vec2(9.0, 9.0));
                gutter_p.rect_stroke(box_r, 0.0, Stroke::new(1.0_f32, theme::TEXT_DIM));
                gutter_p.line_segment(
                    [
                        box_r.left_center() + vec2(2.0, 0.0),
                        box_r.right_center() - vec2(2.0, 0.0),
                    ],
                    Stroke::new(1.0_f32, theme::TEXT),
                );
                if !expanded {
                    gutter_p.line_segment(
                        [
                            box_r.center_top() + vec2(0.0, 2.0),
                            box_r.center_bottom() - vec2(0.0, 2.0),
                        ],
                        Stroke::new(1.0_f32, theme::TEXT),
                    );
                }
            }
            gutter_p.text(
                pos2(label.left() + 24.0 + indent, r.center().y),
                Align2::LEFT_CENTER,
                self.row_label(row),
                font.clone(),
                theme::TEXT,
            );
            for (k, at) in [self.markers.m1, self.markers.m2].into_iter().enumerate() {
                // Short values are right aligned; long ones (a decoded telegram) start at the left of
                // their cell and are cut off at its edge instead of running over the row name.
                let cell = Rect::from_min_size(
                    pos2(label.left() + NAME_W + VAL_W * k as f32, r.top()),
                    vec2(VAL_W, row_h),
                );
                let color = if k == 0 { theme::M1 } else { theme::M2 };
                let galley =
                    gutter_p.layout_no_wrap(self.value_text(&store, row, at), small.clone(), color);
                let x = if galley.size().x > VAL_W - 8.0 {
                    cell.left() + 3.0
                } else {
                    cell.right() - 6.0 - galley.size().x
                };
                gutter_p.with_clip_rect(cell).galley(
                    pos2(x, r.center().y - galley.size().y / 2.0),
                    galley,
                    color,
                );
            }
            match row.kind {
                RowKind::Signal(ch) => {
                    let c = self
                        .layout
                        .colors
                        .get(ch as usize)
                        .copied()
                        .unwrap_or(crate::rows::DEFAULT_COLOR);
                    draw_signal(
                        &plot_p.with_clip_rect(r.intersect(plot)),
                        &store,
                        ch,
                        r,
                        &self.view,
                        rgb(c),
                    );
                }
                RowKind::Bus(item) => {
                    if let Some(b) = self.layout.bus(item) {
                        draw_bus(
                            &plot_p.with_clip_rect(r.intersect(plot)),
                            &store,
                            &b.members,
                            b.radix,
                            r,
                            &self.view,
                            rgb(b.color),
                        );
                    }
                }
                RowKind::Group(_) => {}
                RowKind::Decode { id, lane } => {
                    if let Some(e) = self.entry(id) {
                        let anns = e.handle.anns.read();
                        if let Some(l) = anns.lanes.get(lane as usize) {
                            let sel = self
                                .selected_ann
                                .filter(|s| s.id == id && s.lane == lane)
                                .map(|s| s.start);
                            draw_annotations(
                                &plot_p.with_clip_rect(r.intersect(plot)),
                                l,
                                r,
                                &self.view,
                                e.format,
                                rgb(e.color),
                                sel,
                                anns.lanes.first().map(|v| v.as_slice()),
                            );
                        }
                    }
                }
            }
            plot_p.line_segment(
                [pos2(rect.left(), r.bottom()), pos2(r.right(), r.bottom())],
                Stroke::new(1.0_f32, theme::ROW_SEP),
            );

            let hit = label.intersect(gutter);
            if hit.height() > 2.0 {
                let resp = ui.interact(
                    Rect::from_min_size(hit.left_top(), vec2(NAME_W, hit.height())),
                    ui.id().with(("row", i)),
                    Sense::click_and_drag(),
                );
                if resp.drag_started() {
                    self.row_drag = self.layout.mover_of(row);
                }
                resp.context_menu(|ui| {
                    if let RowKind::Decode { id, .. } = row.kind {
                        if let Some(e) = self.decoders.iter_mut().find(|e| e.handle.id.0 == id) {
                            rename_menu(ui, &mut e.name);
                            ui.label(RichText::new("Show values as").color(theme::TEXT_DIM));
                            ui.horizontal(|ui| {
                                for f in Format::ALL {
                                    ui.selectable_value(&mut e.format, f, f.label());
                                }
                            });
                            ui.separator();
                            color_menu(ui, &mut e.color);
                        }
                        if order_menu(ui, &mut self.layout, row) {
                            self.layout_touched = true;
                        }
                        return;
                    }
                    let name = match row.kind {
                        RowKind::Signal(ch) => self.layout.names.get_mut(ch as usize),
                        RowKind::Bus(i) => match self.layout.items.get_mut(i) {
                            Some(Item::Bus(b)) => Some(&mut b.name),
                            _ => None,
                        },
                        RowKind::Group(i) => match self.layout.items.get_mut(i) {
                            Some(Item::Group(g)) => Some(&mut g.name),
                            _ => None,
                        },
                        RowKind::Decode { .. } => None,
                    };
                    if let Some(n) = name {
                        if rename_menu(ui, n) {
                            self.layout_touched = true;
                        }
                    }
                    let slot = match row.kind {
                        RowKind::Signal(ch) => self.layout.colors.get_mut(ch as usize),
                        RowKind::Bus(item) => match self.layout.items.get_mut(item) {
                            Some(Item::Bus(b)) => Some(&mut b.color),
                            _ => None,
                        },
                        RowKind::Group(_) | RowKind::Decode { .. } => None,
                    };
                    if let Some(c) = slot {
                        if color_menu(ui, c) {
                            self.layout_touched = true;
                        }
                    }
                    if order_menu(ui, &mut self.layout, row) {
                        self.layout_touched = true;
                    }
                });
                if resp.clicked() {
                    select = Some(row.kind);
                    if let (RowKind::Bus(item) | RowKind::Group(item), Some(p)) =
                        (row.kind, resp.interact_pointer_pos())
                    {
                        if p.x < label.left() + 22.0 {
                            toggle = Some(item);
                        }
                    }
                }
            }
        }
        if let Some(k) = select {
            self.selected = Some(k);
        }
        if let Some(item) = toggle {
            self.layout.toggle_expanded(item);
        }
        self.row_drag_ui(ui, &painter, rect, plot, &rows, row_h, pointer);

        if let Some(t) = store.trigger() {
            draw_trigger(&painter.with_clip_rect(rect), plot, ruler, &self.view, t);
        }
        draw_markers(
            &painter.with_clip_rect(rect),
            plot,
            ruler,
            &self.view,
            &self.markers,
        );
        // Yellow target on the edge a click would snap the marker to, with its timestamp.
        self.hover_edge = None;
        if self.snap && self.row_drag.is_none() && self.opts.dragging.is_none() {
            if let Some(p) = pointer.filter(|p| plot.contains(*p)) {
                let s = self.view.sample_at((p.x - plot.left()) as f64);
                if let Some((edge, idx)) = self.snap_target(&store, s, p.y, plot, &rows, row_h) {
                    self.hover_edge = Some(edge);
                    let c = pos2(
                        plot.left() + self.view.x_of(edge as f64) as f32,
                        plot.top() - self.scroll_y + (idx as f32 + 0.5) * row_h,
                    );
                    plot_p.circle_stroke(c, 5.0, Stroke::new(1.5_f32, theme::SNAP));
                    let mut text = format_split((edge as f64 - origin) / rate, 1.0 / rate);
                    if let Some(m1) = self.markers.m1 {
                        let d = (edge as f64 - m1) / rate;
                        text.push_str(&format!(
                            "   {}M1 {}",
                            if d < 0.0 { "-" } else { "+" },
                            format_duration(d.abs())
                        ));
                    }
                    let galley = plot_p.layout_no_wrap(text, FontId::monospace(11.0), theme::SNAP);
                    let size = galley.size() + vec2(8.0, 4.0);
                    let x = if c.x + 10.0 + size.x > plot.right() {
                        c.x - 10.0 - size.x
                    } else {
                        c.x + 10.0
                    };
                    let y = if c.y - size.y - 4.0 < plot.top() {
                        c.y + 6.0
                    } else {
                        c.y - size.y - 4.0
                    };
                    let tag = Rect::from_min_size(pos2(x, y), size);
                    plot_p.rect_filled(tag, 0.0, Color32::BLACK);
                    plot_p.rect_stroke(tag, 0.0, Stroke::new(1.0_f32, theme::SNAP));
                    plot_p.galley(tag.min + vec2(4.0, 2.0), galley, theme::SNAP);
                }
            }
        }
        if store.is_empty() {
            painter.text(
                plot.center(),
                Align2::CENTER_CENTER,
                "No data.  Run (F5) to acquire.",
                FontId::monospace(13.0),
                theme::TEXT_DIM,
            );
        }
    }

    // ---------------------------------------------------------------- dialogs

    fn setup_window(&mut self, ctx: &egui::Context) {
        let mut open = self.show_setup;
        egui::Window::new("Bus/Signal Setup")
            .open(&mut open)
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                let n = self.layout.names.len();
                let mut changed = false;
                ui.label(RichText::new("Signals").color(theme::TEXT_DIM));
                for ch in 0..n {
                    ui.horizontal(|ui| {
                        ui.label(format!("{ch:>2}"));
                        changed |= color_button(ui, &mut self.layout.colors[ch]);
                        let name = egui::TextEdit::singleline(&mut self.layout.names[ch]);
                        changed |= ui.add_sized([130.0, 18.0], name).changed();
                        changed |= ui.checkbox(&mut self.layout.visible[ch], "show").changed();
                    });
                }
                if !self.decoders.is_empty() {
                    ui.separator();
                    ui.label(RichText::new("Decoders").color(theme::TEXT_DIM));
                    for e in &self.decoders {
                        let id = e.handle.id.0;
                        let mut shown = !self.layout.hidden_decoders.contains(&id);
                        if ui
                            .checkbox(&mut shown, format!("show {}", e.name))
                            .changed()
                        {
                            self.layout.set_decoder_shown(id, shown);
                            changed = true;
                        }
                    }
                }
                ui.separator();
                ui.label(RichText::new("Buses").color(theme::TEXT_DIM));
                let mut remove = None;
                for (i, item) in self.layout.items.iter_mut().enumerate() {
                    if let Item::Bus(b) = item {
                        ui.horizontal(|ui| {
                            changed |= color_button(ui, &mut b.color);
                            changed |= ui
                                .add(egui::TextEdit::singleline(&mut b.name).desired_width(90.0))
                                .changed();
                            ui.label(crate::rows::bus_range(&b.members));
                            changed |= ui.checkbox(&mut b.shown, "show").changed();
                            egui::ComboBox::from_id_salt(("radix", i))
                                .selected_text(b.radix.label())
                                .show_ui(ui, |ui| {
                                    for r in Radix::ALL {
                                        changed |= ui
                                            .selectable_value(&mut b.radix, r, r.label())
                                            .changed();
                                    }
                                });
                            if ui.button("Remove").clicked() {
                                remove = Some(i);
                            }
                        });
                    } else if let Item::Group(g) = item {
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("group").color(theme::TEXT_DIM));
                            changed |= ui
                                .add(egui::TextEdit::singleline(&mut g.name).desired_width(90.0))
                                .changed();
                            ui.label(crate::rows::bus_range(&g.members));
                            changed |= ui.checkbox(&mut g.shown, "show").changed();
                            if ui.button("Remove").clicked() {
                                remove = Some(i);
                            }
                        });
                    }
                }
                if let Some(i) = remove {
                    self.layout.remove_bus(i);
                    self.selected = None;
                    changed = true;
                }
                ui.separator();
                ui.horizontal(|ui| {
                    ui.add(
                        egui::TextEdit::singleline(&mut self.new_bus.name)
                            .hint_text("name")
                            .desired_width(80.0),
                    );
                    ui.add(
                        egui::TextEdit::singleline(&mut self.new_bus.channels)
                            .hint_text("e.g. 15-8")
                            .desired_width(80.0),
                    );
                    let radix = self.new_bus.radix.get_or_insert(Radix::Hex);
                    egui::ComboBox::from_id_salt("new_radix")
                        .selected_text(radix.label())
                        .show_ui(ui, |ui| {
                            for r in Radix::ALL {
                                ui.selectable_value(radix, r, r.label());
                            }
                        });
                    if ui
                        .button("Add group")
                        .on_hover_text("A collapsible folder of signals, without a combined value")
                        .clicked()
                    {
                        match parse_channels(&self.new_bus.channels, n)
                            .and_then(|m| self.layout.add_group(&self.new_bus.name, m))
                        {
                            Ok(()) => {
                                self.new_bus = NewBus::default();
                                self.selected = None;
                                changed = true;
                            }
                            Err(e) => self.new_bus.error = Some(e),
                        }
                    }
                    if ui.button("Add bus").clicked() {
                        let radix = self.new_bus.radix.unwrap_or(Radix::Hex);
                        let res = parse_channels(&self.new_bus.channels, n)
                            .and_then(|m| self.layout.add_bus(&self.new_bus.name, m, radix));
                        match res {
                            Ok(()) => {
                                self.new_bus = NewBus::default();
                                self.selected = None;
                                changed = true;
                            }
                            Err(e) => self.new_bus.error = Some(e),
                        }
                    }
                });
                if let Some(e) = &self.new_bus.error {
                    ui.colored_label(theme::STOP, e);
                }
                ui.separator();
                ui.label(RichText::new("Row order").color(theme::TEXT_DIM));
                let mut mv: Option<(usize, i32)> = None;
                for (i, item) in self.layout.items.iter().enumerate() {
                    let name = match item {
                        Item::Signal(ch) => self
                            .layout
                            .names
                            .get(*ch as usize)
                            .cloned()
                            .unwrap_or_default(),
                        Item::Bus(b) => format!("{} (bus)", b.name),
                        Item::Group(g) => format!("{} (group)", g.name),
                        Item::Decoder(id) => format!(
                            "{} (decoder)",
                            self.entry(*id).map(|e| e.name.as_str()).unwrap_or("?")
                        ),
                    };
                    ui.horizontal(|ui| {
                        ui.add_sized([150.0, 16.0], egui::Label::new(name));
                        if ui.small_button("Up").clicked() {
                            mv = Some((i, -1));
                        }
                        if ui.small_button("Down").clicked() {
                            mv = Some((i, 1));
                        }
                    });
                }
                if let Some((i, d)) = mv {
                    self.layout.move_item(i, d);
                    changed = true;
                }
                if changed {
                    self.layout_touched = true;
                }
            });
        self.show_setup = open;
    }

    /// Drag and drop of rows in the gutter: a yellow line shows the gap the row would land in, a
    /// highlighted group header means "into the group". Released on a valid target, the row moves.
    #[allow(clippy::too_many_arguments)]
    fn row_drag_ui(
        &mut self,
        ui: &egui::Ui,
        painter: &egui::Painter,
        rect: Rect,
        plot: Rect,
        rows: &[Row],
        row_h: f32,
        pointer: Option<egui::Pos2>,
    ) {
        let demo = self
            .opts
            .dragging
            .and_then(|i| rows.get(i))
            .and_then(|r| self.layout.mover_of(r));
        let Some(m) = self.row_drag.or(demo) else {
            return;
        };
        let Some(p) = pointer else { return };
        let (released, escape) = ui.input(|i| {
            (
                i.pointer.any_released() || !i.pointer.primary_down(),
                i.key_pressed(Key::Escape),
            )
        });
        let rel = (p.y - plot.top() + self.scroll_y) / row_h;
        let idx = rel.floor().max(0.0) as usize;
        let frac = rel - rel.floor();
        let onto = rows
            .get(idx)
            .filter(|_| rel >= 0.0 && (0.25..0.75).contains(&frac))
            .and_then(|r| self.layout.drop_onto(r, m).map(|d| (d, idx)));
        let (dest, mark) = match onto {
            Some((d, row)) => (Some(d), Err(row)),
            None => {
                let gap = (idx + usize::from(frac >= 0.5)).min(rows.len());
                (self.layout.drop_gap(rows, m, gap), Ok(gap))
            }
        };
        // The row being carried, as a small tag at the pointer.
        let label = rows
            .iter()
            .find(|r| self.layout.mover_of(r) == Some(m))
            .map(|r| self.row_label(r))
            .unwrap_or_default();
        let top = |row: usize| plot.top() - self.scroll_y + row as f32 * row_h;
        let clip = painter.with_clip_rect(Rect::from_min_max(rect.left_top(), rect.right_bottom()));
        if let (Some(_), true) = (dest, p.y >= plot.top() - 4.0) {
            match mark {
                Ok(gap) => {
                    let y = top(gap).clamp(plot.top(), plot.bottom());
                    clip.line_segment(
                        [pos2(rect.left(), y), pos2(rect.right(), y)],
                        Stroke::new(2.0_f32, theme::SNAP),
                    );
                }
                Err(row) => {
                    let r =
                        Rect::from_min_size(pos2(rect.left(), top(row)), vec2(rect.width(), row_h));
                    clip.rect_stroke(r.shrink(1.0), 0.0, Stroke::new(2.0_f32, theme::SNAP));
                }
            }
        } else {
            ui.ctx().set_cursor_icon(CursorIcon::NotAllowed);
        }
        let tag = clip.layout_no_wrap(label, FontId::monospace(12.0), theme::TEXT);
        let tag_r = Rect::from_min_size(p + vec2(12.0, -tag.size().y / 2.0), tag.size())
            .expand2(vec2(5.0, 2.0));
        clip.rect_filled(tag_r, 0.0, theme::SELECT);
        clip.rect_stroke(tag_r, 0.0, Stroke::new(1.0_f32, theme::SNAP));
        clip.galley(tag_r.min + vec2(5.0, 2.0), tag, theme::TEXT);
        // Scroll while carrying a row to the edge of the list.
        if p.y < plot.top() + 14.0 {
            self.scroll_y = (self.scroll_y - 6.0).max(0.0);
        } else if p.y > plot.bottom() - 14.0 {
            self.scroll_y += 6.0;
        }
        ui.ctx().request_repaint();
        if self.row_drag.is_some() && (released || escape) {
            if let (Some(d), false) = (dest, escape) {
                if p.y >= plot.top() - 4.0 && self.layout.move_to(m, d) {
                    self.layout_touched = true;
                }
            }
            self.row_drag = None;
        }
    }

    fn trigger_window(&mut self, ctx: &egui::Context) {
        let mut open = self.show_trigger;
        let names = self.layout.names.clone();
        let n = self.channels_n as usize;
        let (rate, continuous) = (self.rate, self.continuous);
        let decoders: Vec<crate::trigger_ui::DecoderChoice> = self
            .decoders
            .iter()
            .map(|e| crate::trigger_ui::DecoderChoice {
                id: e.handle.id.0,
                label: format!("{} ({})", e.name, decode_ui::short_summary(&e.cfg)),
                cfg: e.cfg.clone(),
            })
            .collect();
        let mut edit = None;
        egui::Window::new("Trigger")
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_width(1000.0)
            .vscroll(true)
            .show(ctx, |ui| {
                edit = crate::trigger_ui::contents(
                    ui,
                    &mut self.trigger,
                    &mut self.trigger_probe,
                    &mut self.memory_mib,
                    &names,
                    &decoders,
                    n,
                    rate,
                    continuous,
                );
            });
        self.show_trigger = open;
        // "Edit in decoder setup": open the decoder window on that decoder.
        if let Some(i) = edit.and_then(|id| self.decoders.iter().position(|e| e.handle.id.0 == id))
        {
            self.setup.selected = i;
            self.show_decoders = true;
        }
        if let Some(rx) = self.trigger_probe.request.take() {
            self.trigger_probe.result = Some((rx, self.detect_baud(rx)));
        }
        if let Some(c) = self.trigger_probe.format_request.take() {
            let r = self.detect_frame_format(&c);
            self.trigger_probe.format_result = Some((c, r));
        }
    }

    /// Detect the baud rate of the UART on channel `rx` over the whole capture.
    fn detect_baud(&self, rx: u8) -> Result<bf_decode::baud::BaudGuess, String> {
        const MAX_EDGES: usize = 300_000;
        let store = self.store.read();
        if store.is_empty() {
            return Err("no capture yet: run one or open a file first".into());
        }
        if rx >= store.channels() {
            return Err(format!("channel {rx} is not in the capture"));
        }
        let mut edges: Vec<u64> = Vec::new();
        store.toggles(rx, store.range(), |p, _| {
            if edges.len() < MAX_EDGES {
                edges.push(p);
            }
        });
        bf_decode::baud::detect(&edges, store.sample_rate())
    }

    /// Detect the frame format of the UART `cfg` (its channel, baud rate and polarity) over the whole
    /// capture.
    fn detect_frame_format(
        &self,
        cfg: &bf_decode::uart::UartConfig,
    ) -> Result<bf_decode::framing::FormatGuess, String> {
        let store = self.store.read();
        if store.is_empty() {
            return Err("no capture yet: run one or open a file first".into());
        }
        if cfg.rx >= store.channels() {
            return Err(format!("channel {} is not in the capture", cfg.rx));
        }
        bf_decode::framing::detect_format(&store, cfg)
    }

    fn decoders_window(&mut self, ctx: &egui::Context) {
        let mut open = self.show_decoders;
        let names = self.layout.names.clone();
        let end = self.store.read().end().max(1);
        let mut window = egui::Window::new("Protocol Decoders")
            .open(&mut open)
            .collapsible(false)
            .resizable(true)
            .default_size([1240.0, 700.0])
            .min_size([640.0, 420.0]);
        if let Some([x, y, w, h]) = self.opts.decoders_rect {
            // Screenshots: a fixed place and size.
            window = window.current_pos([x, y]).fixed_size([w, h]);
        }
        window.show(ctx, |ui| {
            let actions = decode_ui::setup_ui(ui, &mut self.setup, &mut self.decoders, &names, end);
            for a in actions {
                match a {
                    decode_ui::Action::Add(kind) => {
                        self.add_decoder(Config::default_for(kind));
                        self.setup.selected = self.decoders.len() - 1;
                    }
                    decode_ui::Action::Remove(i) => {
                        let e = self.decoders.remove(i);
                        self.trigger.protocol.unlink(e.handle.id.0, &e.cfg);
                        self.layout.remove_decoder(e.handle.id.0);
                        self.engine.remove(e.handle.id);
                        self.selected_ann = None;
                        self.setup.selected = self.setup.selected.saturating_sub(1);
                    }
                    decode_ui::Action::Rebuild(i) => self.rebuild_decoder(i),
                }
            }
        });
        self.show_decoders = open;
        if let Some(rx) = self.setup.probe.request.take() {
            self.setup.probe.result = Some((rx, self.detect_baud(rx)));
        }
        if let Some(c) = self.setup.probe.format_request.take() {
            let r = self.detect_frame_format(&c);
            self.setup.probe.format_result = Some((c, r));
        }
    }

    fn table_panel(&mut self, ui: &mut egui::Ui) {
        let (rate, origin) = {
            let s = self.store.read();
            (s.sample_rate(), self.origin_of(&s))
        };
        let selected = self.selected_ann;
        if let Some((sel, a, idx)) =
            decode_ui::table_ui(ui, &mut self.table, &self.decoders, rate, origin, selected)
        {
            self.select_annotation(sel.id, sel.lane, a, idx, true, true);
        }
    }

    fn about_window(&mut self, ctx: &egui::Context) {
        if !self.show_about {
            return;
        }
        let icon = self
            .icon_texture
            .get_or_insert_with(|| {
                let size = 192;
                ctx.load_texture(
                    "blinkfeuer-icon",
                    egui::ColorImage::from_rgba_unmultiplied(
                        [size, size],
                        &crate::icon::render(size as u32),
                    ),
                    egui::TextureOptions::LINEAR,
                )
            })
            .clone();
        let dim = |t: &str| RichText::new(t.to_string()).color(theme::TEXT_DIM);
        egui::Window::new("About Blinkfeuer")
            .open(&mut self.show_about)
            .collapsible(false)
            .resizable(false)
            .anchor(Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                ui.set_max_width(420.0);
                ui.horizontal(|ui| {
                    ui.add(egui::Image::new((icon.id(), vec2(96.0, 96.0))));
                    ui.vertical(|ui| {
                        ui.add_space(14.0);
                        ui.label(RichText::new("Blinkfeuer").size(24.0).strong());
                        ui.label(format!("Version {}", env!("CARGO_PKG_VERSION")));
                        ui.label(dim("A logic analyzer for Linux"));
                    });
                });
                ui.add_space(8.0);
                ui.label(
                    "Blinkfeuer is the old German word for a flashing beacon, the light that tells \
                     you something is out there.",
                );
                ui.add_space(8.0);
                ui.label(dim("Analyzers"));
                ui.label("Sipeed SLogic16U3 (stock firmware or LiteX gateware)");
                ui.label(
                    "FX2 analyzers such as the Saleae Logic clones (sigrok's fx2lafw firmware)",
                );
                ui.label("Built-in simulator");
                ui.add_space(8.0);
                ui.horizontal(|ui| {
                    ui.label(dim("Licence"));
                    ui.label(env!("CARGO_PKG_LICENSE"));
                });
                ui.horizontal(|ui| {
                    ui.label(dim("Source"));
                    ui.hyperlink(env!("CARGO_PKG_REPOSITORY"));
                });
                ui.add_space(8.0);
                ui.label(dim(
                    "The fx2lafw firmware is part of sigrok and licensed under the GPL; it is not \
                     shipped with Blinkfeuer but loaded from the system. Blinkfeuer is built with \
                     egui.",
                ));
            });
    }

    fn keys_window(&mut self, ctx: &egui::Context) {
        egui::Window::new("Mouse and keys")
            .open(&mut self.show_keys)
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                egui::Grid::new("keys")
                    .num_columns(2)
                    .spacing([16.0, 3.0])
                    .show(ui, |ui| {
                        for (k, d) in [
                            ("Wheel", "zoom around the pointer"),
                            ("Shift+wheel, drag", "pan"),
                            ("Click / right-click", "place M1 / M2 (snapped to an edge)"),
                            ("Drag a marker", "move it (also from the ruler)"),
                            ("Double-click", "fit the whole capture"),
                            ("Click overview", "jump to that part of the capture"),
                            ("Click +/- on a bus", "expand to its signals"),
                            ("Right-click a row", "trace color"),
                            ("F5 / F8 or Shift+F5", "run / stop"),
                            ("Ctrl+O / Ctrl+S", "open / save a capture"),
                            (
                                "View > Measurements",
                                "frequency, duty and pulse widths per channel",
                            ),
                            ("Home or F", "fit the whole capture"),
                            (
                                "Left / Right",
                                "M1 to the previous / next edge of the selected row; on a decoder row the previous / next item",
                            ),
                            ("Shift+Left / Right", "the same for M2"),
                            ("Up / Down", "select the row above / below"),
                            ("Ctrl+Up / Down", "taller / shorter rows"),
                            (
                                "Click a decoded item",
                                "select it, mark its span, show it in the decode table",
                            ),
                            (
                                "Ctrl+F, F3 / Shift+F3",
                                "search the decode table, next / previous row",
                            ),
                        ] {
                            ui.label(RichText::new(k).color(theme::TEXT_DIM));
                            ui.label(d);
                            ui.end_row();
                        }
                    });
            });
    }

    fn handle_demo_and_screenshot(&mut self, ctx: &egui::Context) {
        self.show_measure |= self.opts.measure_window;
        if self.opts.detect_baud
            && !self.detect_pressed
            && self.autostarted
            && !self.running()
            && self.last_result.is_some()
        {
            self.detect_pressed = true;
            self.setup.probe.request = Some(0);
        }
        if let (Some(p), false) = (self.opts.open.clone(), self.open_done) {
            self.open_done = true;
            self.file_chosen(files::Purpose::OpenCapture, p.into());
        }
        if let Some(p) = self.opts.save_capture.clone() {
            ctx.request_repaint_after(Duration::from_millis(100));
            if !self.save_done
                && self.autostarted
                && !self.running()
                && self.last_result.is_some()
                && !self.engine.busy()
            {
                self.save_done = true;
                self.file_chosen(files::Purpose::SaveCapture, p.into());
            }
            if self.save_done && self.job.is_none() && self.opts.screenshot.is_none() {
                if let Some((msg, _, err)) = &self.notice {
                    eprintln!("{msg}");
                    std::process::exit(i32::from(*err));
                }
            }
        }
        if self.opts.demo && !self.autostarted {
            self.autostarted = true;
            if let Some(r) = self.opts.rate {
                self.rate = r;
            }
            if let Some(c) = self.opts.channels {
                self.channels_n = c;
            }
            if !self.device.is_hardware() {
                if let Some(p) = self.opts.pattern_enum() {
                    self.pattern = p;
                }
            }
            match self.opts.device.as_deref() {
                Some("stock") => {
                    self.device = DeviceChoice::Stock;
                    self.pattern = self.opts.pattern_enum().unwrap_or(Pattern::Counter);
                }
                Some("litex") => {
                    self.device = DeviceChoice::Litex;
                    self.pattern = self.opts.pattern_enum().unwrap_or(Pattern::Generator);
                }
                Some("fx2lafw") => {
                    self.device = DeviceChoice::Fx2lafw;
                    self.pattern = Pattern::Inputs;
                    self.channels_n = 8;
                    self.rate = self.opts.rate.unwrap_or(1e6).min(24e6);
                }
                _ => {}
            }
            if let Some(spec) = self.opts.trigger.clone() {
                self.trigger.enabled = true;
                for part in spec.split(',').filter(|p| !p.is_empty()) {
                    let (ch, cond) = part.split_once(':').expect("--trigger CH:COND,...");
                    let cond = match cond {
                        "low" => Cond::Low,
                        "high" => Cond::High,
                        "rise" => Cond::Rising,
                        "fall" => Cond::Falling,
                        "either" => Cond::Either,
                        _ => Cond::X,
                    };
                    self.trigger.conds[ch.parse::<usize>().expect("channel")] = cond;
                }
            }
            if let Some(mode) = self.opts.trigger_mode.clone() {
                self.trigger.enabled = true;
                self.trigger.mode = match mode.as_str() {
                    "uart" => crate::setup::TriggerMode::Uart,
                    "spi" => crate::setup::TriggerMode::Spi,
                    "i2c" => crate::setup::TriggerMode::I2c,
                    _ => crate::setup::TriggerMode::Pattern,
                };
            }
            if let Some(ms) = self.opts.pre_ms {
                self.trigger.pre = crate::setup::Span::new(ms, crate::setup::SpanUnit::Millis);
            }
            if let Some(ms) = self.opts.post_ms {
                self.trigger.post = crate::setup::Span::new(ms, crate::setup::SpanUnit::Millis);
            }
            if let Some(m) = self.opts.memory_mib {
                self.memory_mib = m;
            }
            self.show_trigger |= self.opts.trigger_window;
            self.show_measure |= self.opts.measure_window;
            if let Some(n) = self.opts.samples {
                self.continuous = false;
                self.samples = n;
                if !self.device.is_hardware() {
                    self.device = DeviceChoice::Sim;
                }
            }
            if let Some(h) = self.opts.row_height {
                self.row_h = h;
                self.row_auto = false;
            }
            self.start();
            for d in self.opts.decode.clone() {
                let kind = match d.as_str() {
                    "uart" => DecoderKind::Uart,
                    "spi" => DecoderKind::Spi,
                    "i2c" => DecoderKind::I2c,
                    other => {
                        eprintln!("unknown decoder '{other}'");
                        continue;
                    }
                };
                self.add_decoder(Config::default_for(kind));
            }
            self.show_table |= self.opts.table;
            if let Some(l) = self.opts.table_lane {
                self.table.lane = l;
            }
            if let Some(q) = self.opts.search.clone() {
                self.table.filter = q;
                self.show_table = true;
            }
            if let Some(i) = self.opts.decoders_window {
                self.show_decoders = true;
                self.setup.selected = i;
            }
            if self.opts.groups && self.layout.names.len() == 16 {
                let _ = self.layout.add_group("SPI", vec![1, 2, 3]);
                let _ = self.layout.add_group("I2C", vec![4, 5]);
                if let Some(i) = self
                    .layout
                    .items
                    .iter()
                    .position(|it| matches!(it, Item::Group(g) if g.name == "I2C"))
                {
                    self.layout.toggle_expanded(i); // show one group collapsed
                }
                self.layout_touched = true;
            }
            if let Some(name) = self.opts.decoder_group.clone() {
                let gi = self
                    .layout
                    .items
                    .iter()
                    .position(|it| matches!(it, Item::Group(g) if g.name == name));
                if let Some(i) = gi {
                    if let Some(e) = self.decoders.first() {
                        self.layout.move_to(
                            Mover::Decoder(e.handle.id.0),
                            Dest::Group {
                                item: i,
                                pos: usize::MAX,
                            },
                        );
                    }
                    if let Some(b) = self
                        .layout
                        .items
                        .iter()
                        .position(|it| matches!(it, Item::Bus(_)))
                    {
                        self.layout.move_to(
                            Mover::Bus(b),
                            Dest::Group {
                                item: i,
                                pos: usize::MAX,
                            },
                        ); // screenshots: a bus in the group
                    }
                }
            }
        }
        if let (Some(n), false, false) = (self.opts.goto, self.goto_done, self.engine.busy()) {
            if self.autostarted
                && !self.running()
                && self.last_result.is_some()
                && !self.decoders.is_empty()
            {
                let found = {
                    let anns = self.decoders[0].handle.anns.read();
                    let lane = self.opts.table_lane.unwrap_or_else(|| {
                        self.decoders[0]
                            .handle
                            .lane_used
                            .iter()
                            .position(|u| *u)
                            .unwrap_or(0)
                    });
                    anns.lanes
                        .get(lane)
                        .and_then(|l| l.get(n).map(|a| (lane, *a)))
                };
                if let Some((lane, a)) = found {
                    self.goto_done = true;
                    let id = self.decoders[0].handle.id.0;
                    self.select_annotation(id, lane as u8, a, n, true, false);
                }
            }
        }
        if !self.running() && self.autostarted && !self.zoom_applied && self.last_result.is_some() {
            self.zoom_applied = true;
            if let (Some(s), Some(p)) = (self.opts.zoom_start, self.opts.zoom_spp) {
                self.view = View { start: s, spp: p };
                self.follow = false;
            }
            if let Some((a, b)) = self.opts.markers {
                self.markers = Markers {
                    m1: Some(a),
                    m2: Some(b),
                };
            }
            if self.opts.expand {
                for it in &mut self.layout.items {
                    if let Item::Bus(b) = it {
                        b.expanded = true;
                    }
                }
            }
            self.show_setup |= self.opts.setup;
        }
        if let Some(path) = self.opts.screenshot.clone() {
            if !self.screenshot_requested && self.t0.elapsed().as_secs_f64() >= self.opts.after {
                self.screenshot_requested = true;
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
            }
            let shot = ctx.input(|i| {
                i.events.iter().find_map(|e| match e {
                    egui::Event::Screenshot { image, .. } => Some(image.clone()),
                    _ => None,
                })
            });
            if let Some(img) = shot {
                let [w, h] = img.size;
                let mut raw = Vec::with_capacity(w * h * 4);
                for p in &img.pixels {
                    raw.extend_from_slice(&p.to_array());
                }
                match image::save_buffer(&path, &raw, w as u32, h as u32, image::ColorType::Rgba8) {
                    Ok(()) => eprintln!("screenshot saved to {path} ({w}x{h})"),
                    Err(e) => eprintln!("screenshot failed: {e}"),
                }
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            ctx.request_repaint_after(Duration::from_millis(50));
        }
    }
}

impl eframe::App for App {
    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        self.autosave(true);
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.handle_demo_and_screenshot(ctx);
        self.poll_session();
        self.poll_files(ctx);
        self.autosave(false);
        self.update_title(ctx);
        let (open, save) = ctx.input(|i| {
            (
                i.modifiers.command && i.key_pressed(Key::O),
                i.modifiers.command && i.key_pressed(Key::S),
            )
        });
        if open && !self.running() && !self.busy_files() {
            self.ask_file(files::Purpose::OpenCapture);
        }
        if save && !self.running() && !self.busy_files() && !self.store.read().is_empty() {
            self.save_capture();
        }

        // While a file dialog is open the keyboard belongs to it (its text fields, Escape); keys that
        // also edit text (Home, arrows, letters) are left to a focused text field.
        let keys = !self.file_dialog_open();
        let nav = keys && !ctx.wants_keyboard_input();
        let (left, right, up, down, fit_key, shift_nav) = ctx.input(|i| {
            let plain = !i.modifiers.command && !i.modifiers.alt;
            (
                nav && plain && i.key_pressed(Key::ArrowLeft),
                nav && plain && i.key_pressed(Key::ArrowRight),
                nav && plain && !i.modifiers.shift && i.key_pressed(Key::ArrowUp),
                nav && plain && !i.modifiers.shift && i.key_pressed(Key::ArrowDown),
                nav && plain && !i.modifiers.shift && i.key_pressed(Key::F),
                i.modifiers.shift,
            )
        });
        let marker = if shift_nav { Which::M2 } else { Which::M1 };
        if left {
            self.step(-1, marker);
        }
        if right {
            self.step(1, marker);
        }
        if up {
            self.select_row_step(-1);
        }
        if down {
            self.select_row_step(1);
        }
        if fit_key {
            self.fit();
        }
        let (f5, f8, home) = ctx.input(|i| {
            (
                keys && !i.modifiers.shift && i.key_pressed(Key::F5),
                // Shift+F5 as well: F8 can be taken by a global hotkey of another program.
                keys && (i.key_pressed(Key::F8) || i.modifiers.shift && i.key_pressed(Key::F5)),
                nav && i.key_pressed(Key::Home),
            )
        });
        let (taller, shorter) = ctx.input(|i| {
            (
                keys && i.modifiers.command && i.key_pressed(Key::ArrowUp),
                keys && i.modifiers.command && i.key_pressed(Key::ArrowDown),
            )
        });
        let (find, f3, shift_held) = ctx.input(|i| {
            (
                keys && i.modifiers.command && i.key_pressed(Key::F),
                keys && i.key_pressed(Key::F3),
                i.modifiers.shift,
            )
        });
        if find {
            self.show_table = true;
            self.table.focus_search = true;
        }
        if f3 {
            self.show_table = true;
            self.table.nav = Some(if shift_held { -1 } else { 1 });
        }
        if taller {
            self.resize_rows(6.0);
        }
        if shorter {
            self.resize_rows(-6.0);
        }
        if f5 && !self.running() {
            self.start();
        }
        if f8 {
            self.stop();
        }
        if home {
            self.fit();
        }

        let bar = |fill| {
            egui::Frame::none()
                .fill(fill)
                .inner_margin(Margin::symmetric(6.0, 3.0))
        };
        egui::TopBottomPanel::top("menu")
            .frame(bar(theme::CHROME_DARK))
            .show(ctx, |ui| self.menu_bar(ui));
        egui::TopBottomPanel::top("run")
            .frame(bar(theme::CHROME))
            .show(ctx, |ui| self.run_bar(ui));
        egui::TopBottomPanel::top("markers")
            .frame(bar(theme::CHROME_DARK))
            .show(ctx, |ui| self.marker_bar(ui));
        egui::TopBottomPanel::bottom("status")
            .exact_height(22.0)
            .frame(bar(theme::CHROME_DARK))
            .show(ctx, |ui| self.status_bar(ui));
        egui::TopBottomPanel::bottom("overview")
            .exact_height(OVERVIEW_H)
            .frame(egui::Frame::none().fill(theme::CHROME))
            .show(ctx, |ui| self.overview(ui));
        if self.show_table {
            egui::TopBottomPanel::bottom("table")
                .resizable(true)
                .default_height(190.0)
                .frame(
                    egui::Frame::none()
                        .fill(theme::CHROME_DARK)
                        .inner_margin(Margin::symmetric(6.0, 3.0)),
                )
                .show(ctx, |ui| self.table_panel(ui));
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::none())
            .show(ctx, |ui| self.waveform_panel(ui));

        self.setup_window(ctx);
        self.decoders_window(ctx);
        self.trigger_window(ctx);
        if self.show_measure {
            let store = self.store.clone();
            let guard = store.read();
            let origin = self.origin_of(&guard);
            let running = self.running();
            crate::measure_ui::window(
                ctx,
                &mut self.show_measure,
                &mut self.measure,
                &guard,
                &self.layout,
                &self.view,
                self.plot_width,
                &self.markers,
                origin,
                running,
            );
        }
        self.keys_window(ctx);
        self.about_window(ctx);
        if self.opts.color_demo {
            egui::Window::new("Color picker").show(ctx, |ui| {
                let mut c = crate::rows::PALETTE[3];
                color_menu(ui, &mut c);
            });
        }
        if self.running() || self.engine.busy() {
            ctx.request_repaint_after(Duration::from_millis(16));
        }
    }
}
