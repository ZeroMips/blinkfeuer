//! The Blinkfeuer application: command line options and the start of the user interface.

mod app;
mod decode_preview;
mod decode_ui;
mod file_dialog;
mod icon;
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
    /// Write the application icon as a PNG file (and its size in pixels) and exit.
    pub write_icon: Option<(String, u32)>,
    /// Install the icon and a desktop entry and exit: below this folder, or for the user when empty.
    pub install_desktop: Option<String>,
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
    /// Where the decoder setup window opens: x, y, width, height in points (screenshots).
    pub decoders_rect: Option<[f32; 4]>,
    /// Size of the application window in points (screenshots).
    pub window: Option<[f32; 2]>,
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
    /// Press "Detect" for the UART baud rate once the demo capture is done (screenshots).
    pub detect_baud: bool,
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
    let mut o = Opts {
        after: 2.0,
        ..Default::default()
    };
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = || args.next().unwrap_or_else(|| panic!("{a} needs a value"));
        match a.as_str() {
            "--demo" => o.demo = true,
            "--samples" => o.samples = Some(val().parse().expect("--samples N")),
            "--rate" => o.rate = Some(val().parse().expect("--rate HZ")),
            "--screenshot" => o.screenshot = Some(val()),
            "--install-desktop" => o.install_desktop = Some(String::new()),
            a if a.starts_with("--install-desktop=") => {
                o.install_desktop = Some(a["--install-desktop=".len()..].to_string());
            }
            "--write-icon" => {
                let v = val();
                o.write_icon = Some(match v.split_once(',') {
                    Some((path, size)) => (
                        path.to_string(),
                        size.parse().expect("--write-icon PATH[,SIZE]"),
                    ),
                    None => (v, 256),
                });
            }
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
            "--detect-baud" => o.detect_baud = true,
            "--open" => o.open = Some(val()),
            "--save-capture" => o.save_capture = Some(val()),
            "--dragging" => o.dragging = Some(val().parse().expect("--dragging ROW")),
            "--search" => o.search = Some(val()),
            "--table-lane" => o.table_lane = Some(val().parse().expect("--table-lane N")),
            "--goto" => o.goto = Some(val().parse().expect("--goto INDEX")),
            "--decoder-setup" => {
                o.decoders_window = Some(val().parse().expect("--decoder-setup INDEX"))
            }
            "--decoder-setup-rect" => {
                let v: Vec<f32> = val()
                    .split(',')
                    .map(|x| x.trim().parse().expect("--decoder-setup-rect X,Y,W,H"))
                    .collect();
                let [x, y, w, h] = v[..] else {
                    panic!("--decoder-setup-rect X,Y,W,H")
                };
                o.decoders_rect = Some([x, y, w, h]);
            }
            "--window" => {
                let v = val();
                let (w, h) = v.split_once(['x', 'X']).expect("--window WxH");
                o.window = Some([
                    w.trim().parse().expect("--window WxH"),
                    h.trim().parse().expect("--window WxH"),
                ]);
            }
            "--row-height" => o.row_height = Some(val().parse().expect("--row-height POINTS")),
            "--hover" => {
                let v = val();
                let (a, b) = v.split_once(',').expect("--hover X,Y");
                o.hover = Some((a.parse().expect("X"), b.parse().expect("Y")));
            }
            "--channels" => o.channels = Some(val().parse().expect("--channels N")),
            "-h" | "--help" => {
                println!("blinkfeuer [--demo] [--samples N] [--rate HZ] [--channels N] [--device sim|stock|litex|fx2lafw] [--pattern generator|counter|inputs|emulation]");
                println!(
                    "          [--screenshot PATH [--after SECS]] [--zoom-start S --zoom-spp P]"
                );
                println!(
                    "          [--markers S1,S2] [--expand] [--setup] [--write-icon PATH[,SIZE]] [--install-desktop[=DIR]]"
                );
                println!(
                    "          [--window WxH] [--decoder-setup INDEX [--decoder-setup-rect X,Y,W,H]]"
                );
                std::process::exit(0);
            }
            other => panic!("unknown option {other}"),
        }
    }
    o
}

/// Under WSLg, use X11 instead of Wayland: the window shows its own icon there (a Wayland window
/// cannot), and WSLg's compositor (weston 1.0.7x) crashed with SIGSEGV now and then when it matched a
/// Wayland window to the installed desktop entry. `BLINKFEUER_WAYLAND=1` keeps Wayland.
fn prefer_x11_under_wslg() {
    let wsl = std::env::var_os("WSL_DISTRO_NAME").is_some();
    let keep = std::env::var_os("BLINKFEUER_WAYLAND").is_some_and(|v| v == "1");
    if wsl && !keep && std::env::var_os("DISPLAY").is_some() {
        // Still single-threaded here, before the event loop starts.
        std::env::remove_var("WAYLAND_DISPLAY");
        std::env::remove_var("WAYLAND_SOCKET");
    }
}

fn main() -> eframe::Result {
    let opts = parse_opts();
    if let Some((path, size)) = &opts.write_icon {
        if let Err(e) = icon::write_png(path, *size) {
            eprintln!("blinkfeuer: {e}");
            std::process::exit(1);
        }
        return Ok(());
    }
    if let Some(prefix) = &opts.install_desktop {
        let data_home = if prefix.is_empty() {
            std::env::var_os("XDG_DATA_HOME")
                .map(std::path::PathBuf::from)
                .or_else(|| dirs::home_dir().map(|h| h.join(".local/share")))
        } else {
            Some(std::path::PathBuf::from(prefix))
        };
        let exec = std::env::current_exe()
            .and_then(|p| p.canonicalize())
            .map(|p| p.to_string_lossy().into_owned());
        match (data_home, exec) {
            (Some(dir), Ok(exec)) => match icon::install_desktop(&dir, &exec) {
                Ok(files) => {
                    for f in files {
                        println!("{f}");
                    }
                }
                Err(e) => {
                    eprintln!("blinkfeuer: {e}");
                    std::process::exit(1);
                }
            },
            _ => {
                eprintln!("blinkfeuer: cannot find the home directory or this program's path");
                std::process::exit(1);
            }
        }
        return Ok(());
    }
    prefer_x11_under_wslg();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size(opts.window.unwrap_or([1280.0, 800.0]))
            .with_title("Blinkfeuer")
            .with_app_id(icon::APP_ID)
            .with_icon(std::sync::Arc::new(icon::icon_data())),
        ..Default::default()
    };
    eframe::run_native(
        "Blinkfeuer",
        options,
        Box::new(move |cc| Ok(Box::new(app::App::new(cc, opts.clone())))),
    )
}
