mod app;
mod decode_preview;
mod decode_ui;
mod markers;
mod measure_ui;
mod rows;
mod setup;
mod theme;
mod trigger_ui;
mod view;
mod waveform;

/// Command line options (mostly for demos and automated screenshots).
#[derive(Default, Clone)]
pub struct Opts {
    /// Start a capture on the simulator immediately.
    pub demo: bool,
    /// Finite capture of this many samples (fitted when done) instead of a live one.
    pub samples: Option<f64>,
    /// Sample rate in Hz.
    pub rate: Option<f64>,
    /// Save a screenshot of the window to this path after `after` seconds and exit.
    pub screenshot: Option<String>,
    pub after: f64,
    /// Initial view after the capture ended: left sample and samples per pixel.
    pub zoom_start: Option<f64>,
    pub zoom_spp: Option<f64>,
    pub pattern: Option<String>,
    /// `sim` (default) or `stock`.
    pub device: Option<String>,
    pub channels: Option<u8>,
    /// Markers M1/M2 (sample numbers) placed after the capture ended.
    pub markers: Option<(f64, f64)>,
    /// Expand all buses / open the Bus/Signal setup window (for screenshots).
    pub expand: bool,
    pub setup: bool,
    pub color_demo: bool,
    /// Fake pointer position (points from the plot's top-left) for screenshots.
    pub hover: Option<(f32, f32)>,
    /// Fixed row height in points.
    pub row_height: Option<f32>,
    /// Decoders to add in demo mode (`uart,spi,i2c`).
    pub decode: Vec<String>,
    /// Show the annotation table (screenshots).
    pub table: bool,
    /// Group the SPI and I2C signals of the generator pattern (screenshots).
    pub groups: bool,
    /// Open the decoder setup window and select this decoder (index) in demo mode.
    pub decoders_window: Option<usize>,
    /// Search text for the annotation table, and a decoded item (index in the first decoder's first lane) to select.
    pub search: Option<String>,
    pub goto: Option<usize>,
    /// Decoder lane for the table and `--goto`.
    pub table_lane: Option<usize>,
    /// Put the first decoder into the group of this name (screenshots; needs `--groups`).
    pub decoder_group: Option<String>,
    /// Show the drag indicator as if the row with this index were being carried to `--hover`.
    pub dragging: Option<usize>,
    /// Open this capture file at start / save the finished demo capture to this file and exit.
    pub open: Option<String>,
    /// Trigger for the demo capture: `CH:COND,...` (low, high, rise, fall, either), pre-trigger percent, show the window.
    pub trigger: Option<String>,
    pub pre_ms: Option<f64>,
    pub post_ms: Option<f64>,
    pub memory_mib: Option<u64>,
    /// Protocol trigger for the demo capture: uart, spi or i2c (default settings fit the generator pattern).
    pub trigger_mode: Option<String>,
    pub trigger_window: bool,
    pub measure_window: bool,
    pub save_capture: Option<String>,
}

impl Opts {
    pub fn pattern_enum(&self) -> Option<bf_device::Pattern> {
        use bf_device::Pattern;
        match self.pattern.as_deref()? {
            "counter" => Some(Pattern::Counter),
            "inputs" => Some(Pattern::Inputs),
            "generator" => Some(Pattern::Generator),
            "emulation" => Some(Pattern::Emulation),
            _ => None,
        }
    }
}

fn parse_opts() -> Opts {
    let mut o = Opts { after: 2.0, ..Default::default() };
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = || args.next().unwrap_or_else(|| panic!("{a} needs a value"));
        match a.as_str() {
            "--demo" => o.demo = true,
            "--samples" => o.samples = Some(val().parse().expect("--samples N")),
            "--rate" => o.rate = Some(val().parse().expect("--rate HZ")),
            "--screenshot" => o.screenshot = Some(val()),
            "--after" => o.after = val().parse().expect("--after SECONDS"),
            "--zoom-start" => o.zoom_start = Some(val().parse().expect("--zoom-start SAMPLE")),
            "--zoom-spp" => o.zoom_spp = Some(val().parse().expect("--zoom-spp SAMPLES_PER_PIXEL")),
            "--pattern" => o.pattern = Some(val()),
            "--device" => o.device = Some(val()),
            "--markers" => {
                let v = val();
                let (a, b) = v.split_once(',').expect("--markers S1,S2");
                o.markers = Some((a.parse().expect("S1"), b.parse().expect("S2")));
            }
            "--expand" => o.expand = true,
            "--setup" => o.setup = true,
            "--color-demo" => o.color_demo = true,
            "--decode" => o.decode = val().split(',').map(str::to_string).collect(),
            "--table" => o.table = true,
            "--groups" => o.groups = true,
            "--decoder-group" => o.decoder_group = Some(val()),
            "--trigger" => o.trigger = Some(val()),
            "--trigger-mode" => o.trigger_mode = Some(val()),
            "--pre-ms" => o.pre_ms = Some(val().parse().expect("--pre-ms MS")),
            "--post-ms" => o.post_ms = Some(val().parse().expect("--post-ms MS")),
            "--memory-mib" => o.memory_mib = Some(val().parse().expect("--memory-mib N")),
            "--trigger-window" => o.trigger_window = true,
            "--measure-window" => o.measure_window = true,
            "--open" => o.open = Some(val()),
            "--save-capture" => o.save_capture = Some(val()),
            "--dragging" => o.dragging = Some(val().parse().expect("--dragging ROW")),
            "--search" => o.search = Some(val()),
            "--table-lane" => o.table_lane = Some(val().parse().expect("--table-lane N")),
            "--goto" => o.goto = Some(val().parse().expect("--goto INDEX")),
            "--decoder-setup" => o.decoders_window = Some(val().parse().expect("--decoder-setup INDEX")),
            "--row-height" => o.row_height = Some(val().parse().expect("--row-height POINTS")),
            "--hover" => {
                let v = val();
                let (a, b) = v.split_once(',').expect("--hover X,Y");
                o.hover = Some((a.parse().expect("X"), b.parse().expect("Y")));
            }
            "--channels" => o.channels = Some(val().parse().expect("--channels N")),
            "-h" | "--help" => {
                println!("blinkfeuer [--demo] [--samples N] [--rate HZ] [--channels N] [--device sim|stock] [--pattern generator|counter|inputs|emulation]");
                println!("          [--screenshot PATH [--after SECS]] [--zoom-start S --zoom-spp P]");
                println!("          [--markers S1,S2] [--expand] [--setup]");
                std::process::exit(0);
            }
            other => panic!("unknown option {other}"),
        }
    }
    o
}

fn main() -> eframe::Result {
    let opts = parse_opts();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1280.0, 800.0]).with_title("Blinkfeuer"),
        ..Default::default()
    };
    eframe::run_native("Blinkfeuer", options, Box::new(move |cc| Ok(Box::new(app::App::new(cc, opts.clone())))))
}
