//! Decoder configuration, the Decoders setup window and the annotation table.

use crate::theme;
use crate::view::format_split;
use bf_decode::engine::DecoderHandle;
use bf_decode::i2c::{I2c, I2cConfig};
use bf_decode::spi::{Spi, SpiConfig};
use bf_decode::trigger::PatItem;
use bf_decode::uart::{GapUnit, Parity, Protocol, Uart, UartConfig};
use bf_decode::{Annotation, Decoder, Format, Kind};
use egui::{vec2, Color32, Rect, RichText, Sense};

/// A decoder's settings, editable in the UI.
#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Config {
    Uart(UartConfig),
    Spi(SpiConfig),
    I2c(I2cConfig),
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DecoderKind {
    Uart,
    Spi,
    I2c,
}

impl DecoderKind {
    pub const ALL: [DecoderKind; 3] = [DecoderKind::Uart, DecoderKind::Spi, DecoderKind::I2c];

    pub fn label(self) -> &'static str {
        match self {
            DecoderKind::Uart => "UART",
            DecoderKind::Spi => "SPI",
            DecoderKind::I2c => "I2C",
        }
    }
}

impl Config {
    /// Default settings with channels that fit the simulator's generator pattern.
    pub fn default_for(kind: DecoderKind) -> Config {
        match kind {
            DecoderKind::Uart => Config::Uart(UartConfig {
                rx: 0,
                ..Default::default()
            }),
            DecoderKind::Spi => Config::Spi(SpiConfig {
                cs: Some(1),
                sck: 2,
                mosi: Some(3),
                miso: None,
                ..Default::default()
            }),
            DecoderKind::I2c => Config::I2c(I2cConfig { scl: 4, sda: 5 }),
        }
    }

    pub fn kind(&self) -> DecoderKind {
        match self {
            Config::Uart(_) => DecoderKind::Uart,
            Config::Spi(_) => DecoderKind::Spi,
            Config::I2c(_) => DecoderKind::I2c,
        }
    }

    pub fn build(&self) -> Box<dyn Decoder> {
        match self {
            Config::Uart(c) => Box::new(Uart::new(c.clone())),
            Config::Spi(c) => Box::new(Spi::new(c.clone())),
            Config::I2c(c) => Box::new(I2c::new(c.clone())),
        }
    }
}

/// One decoder in use.
pub struct Entry {
    pub handle: DecoderHandle,
    pub cfg: Config,
    pub name: String,
    pub format: Format,
    /// Base colour of the annotations (sRGB).
    pub color: [u8; 3],
}

/// Hover help for the parameters of each decoder.
mod help {
    pub const NAME: &str = "Name shown in the waveform rows and the decode table.";
    pub const FORMAT: &str = "How data values are displayed: hexadecimal, decimal, binary or ASCII (characters; control codes as \\r, \\n ...). Only changes the display, not the decoding.";

    pub const UART_RX: &str = "Channel that carries the serial data. Connect it to the TX pin of the device under test. The line idles high (low if 'Inverted' is on).";
    pub const UART_DETECT: &str = "Measures the pulse widths on the RX channel over the whole capture and suggests the baud rate. It needs UART traffic in the capture and a sample rate of about 8 times the baud rate or more.";
    pub const UART_BAUD: &str = "Bit rate in bits per second (9600, 115200, ...). The decoder uses sample rate / baud samples per bit, so a wrong value gives garbage or framing errors (FE). Capture at no less than about 8 times the baud rate.";
    pub const UART_DETECT_FORMAT: &str = "Finds data bits, parity and stop bits by decoding the whole capture with each common format at the baud rate in the settings below (detect or set that first). The shortest frame that decodes without errors wins; parity is only chosen when it checks out in every character. 2 stop bits cannot be told from 1 when there are pauses between the characters, and 1 stop bit reads such traffic correctly.";
    pub const UART_BITS: &str = "Data bits per frame, 5 to 9 (usually 8).";
    pub const UART_PARITY: &str = "Extra bit after the data bits. Even/Odd: the number of 1 bits including the parity bit is even/odd. Mark: always 1. Space: always 0. A wrong parity bit is flagged PE; the byte is still shown.";
    pub const UART_STOP: &str = "Length of the idle period that ends a frame: 1, 1.5 or 2 bit times. The decoder checks the first stop bit: a low level there is a framing error (FE).";
    pub const UART_LSB: &str = "UART sends the least significant bit first. Turn this off for the rare links that send the most significant bit first.";
    pub const UART_TELEGRAMS: &str = "Characters that follow each other closely are joined into one telegram: a bar on the 'Telegram' row showing the whole text, and one line in the decode table (searchable by text).";
    pub const UART_GAP: &str = "A telegram ends when the line has been idle this long, in bit times or in characters (a character is start, data, parity and stop bits: 10 bit times for 8N1, 11 for 8E1). Changing the unit converts the value. The default of 24 bit times is about 2.4 characters; use a larger gap for slow senders. Click the number to type it, or drag it.";
    pub const UART_PROTOCOL: &str = "What the characters carry. None: plain characters, optionally joined into telegrams. Modbus RTU: frames are found by the pause between them (3.5 characters, 1.75 ms above 19200 baud) and checked by their CRC; each frame is shown on the 'Frame' row and split into address, function, registers, values and CRC on the 'Fields' row. Modbus RTU usually runs 8E1 (even parity) or 8N2.";
    pub const UART_LF: &str = "Only offered while the values are shown as ASCII. Also end a telegram right after a line feed (0x0A), as in line-based text protocols such as NMEA or console output.";
    pub const UART_INVERT: &str = "Use when the line idles low and a start bit is a high level (inverted logic, e.g. RS-232 seen without a level shifter).";

    pub const SPI_SCK: &str =
        "Serial clock. Data is shifted in on its sampling edge (see CPOL and CPHA).";
    pub const SPI_CS: &str = "Chip select. While it is active the clock is decoded; when it ends the transaction (frame) is closed and a partly received word is flagged. 'none' decodes continuously, counting clock edges.";
    pub const SPI_MOSI: &str =
        "Master Out, Slave In: data sent by the master. 'none' ignores this line.";
    pub const SPI_MISO: &str =
        "Master In, Slave Out: data sent by the slave. 'none' ignores this line.";
    pub const SPI_CPOL: &str = "Clock polarity: the level of SCK while idle. Off = idle low (modes 0 and 1), on = idle high (modes 2 and 3).";
    pub const SPI_CPHA: &str = "Clock phase: which edge samples data. Off = the first (leading) edge, on = the second (trailing) edge. With CPOL off, mode 0 samples on the rising edge and mode 1 on the falling edge.";
    pub const SPI_MSB: &str =
        "Bit order within a word. Most devices send the most significant bit first.";
    pub const SPI_CS_LOW: &str = "Chip select is active at a low level (the usual case, written CS# or nCS). Turn off for active-high chip selects.";
    pub const SPI_BITS: &str =
        "Bits per word, 2 to 32 (usually 8). A word is complete after this many sampling edges.";

    pub const I2C_SCL: &str =
        "Clock line. Open-drain with a pull-up: idles high. Bits are valid while it is high.";
    pub const I2C_SDA: &str = "Data line. Open-drain with a pull-up: idles high. It changes only while SCL is low, except for START (SDA falls while SCL is high) and STOP (SDA rises while SCL is high).";
}

/// Channel chooser. Returns true when the selection changed.
fn channel_combo(ui: &mut egui::Ui, id: &str, value: &mut u8, names: &[String]) -> egui::Response {
    let mut changed = false;
    let mut r = egui::ComboBox::from_id_salt(id)
        .selected_text(format!(
            "{value:>2} {}",
            names.get(*value as usize).map_or("?", |s| s.as_str())
        ))
        .width(150.0)
        .show_ui(ui, |ui| {
            for (i, n) in names.iter().enumerate() {
                changed |= ui
                    .selectable_value(value, i as u8, format!("{i:>2} {n}"))
                    .changed();
            }
        })
        .response;
    if changed {
        r.mark_changed();
    }
    r
}

fn optional_channel_combo(
    ui: &mut egui::Ui,
    id: &str,
    value: &mut Option<u8>,
    names: &[String],
) -> egui::Response {
    let mut changed = false;
    let text = value.map_or("none".to_string(), |v| {
        format!(
            "{v:>2} {}",
            names.get(v as usize).cloned().unwrap_or_default()
        )
    });
    let mut r = egui::ComboBox::from_id_salt(id)
        .selected_text(text)
        .width(150.0)
        .show_ui(ui, |ui| {
            changed |= ui.selectable_value(value, None, "none").changed();
            for (i, n) in names.iter().enumerate() {
                changed |= ui
                    .selectable_value(value, Some(i as u8), format!("{i:>2} {n}"))
                    .changed();
            }
        })
        .response;
    if changed {
        r.mark_changed();
    }
    r
}

/// One labelled parameter row with hover help on both the label and the control.
/// The telegram gap: a number (click to type, drag to change) and its unit. Changing the unit converts
/// the number, so the gap stays the same.
fn gap_editor(ui: &mut egui::Ui, id: &str, c: &mut UartConfig) -> egui::Response {
    let mut unit_changed = false;
    let mut r = ui
        .horizontal(|ui| {
            let speed = match c.telegram_gap_unit {
                GapUnit::Bits => 0.5,
                GapUnit::Chars => 0.05,
            };
            let num = ui.add(
                egui::DragValue::new(&mut c.telegram_gap)
                    .speed(speed)
                    .range(0.1..=10_000.0)
                    .max_decimals(2),
            );
            let before = c.telegram_gap_unit;
            egui::ComboBox::from_id_salt(id)
                .selected_text(c.telegram_gap_unit.label())
                .width(90.0)
                .show_ui(ui, |ui| {
                    for u in GapUnit::ALL {
                        ui.selectable_value(&mut c.telegram_gap_unit, u, u.label());
                    }
                });
            if c.telegram_gap_unit != before {
                c.telegram_gap =
                    convert_gap(c.telegram_gap, before, c.telegram_gap_unit, c.char_bits());
                unit_changed = true;
            }
            num
        })
        .inner;
    if unit_changed {
        r.mark_changed();
    }
    r
}

/// `value` in unit `from` expressed in unit `to` (rounded to 2 decimals).
fn convert_gap(value: f64, from: GapUnit, to: GapUnit, char_bits: f64) -> f64 {
    let bits = match from {
        GapUnit::Bits => value,
        GapUnit::Chars => value * char_bits,
    };
    let v = match to {
        GapUnit::Bits => bits,
        GapUnit::Chars => bits / char_bits,
    };
    (v * 100.0).round() / 100.0
}

fn param(
    ui: &mut egui::Ui,
    label: &str,
    help: &str,
    add: impl FnOnce(&mut egui::Ui) -> egui::Response,
) -> bool {
    ui.label(RichText::new(label).color(theme::TEXT_DIM))
        .on_hover_text(help);
    let r = add(ui).on_hover_text(help);
    ui.end_row();
    r.changed()
}

fn section(ui: &mut egui::Ui, title: &str, body: impl FnOnce(&mut egui::Ui)) {
    ui.add_space(6.0);
    ui.label(RichText::new(title).color(theme::TEXT).strong());
    ui.add_space(2.0);
    body(ui);
}

fn grid(ui: &mut egui::Ui, id: &str, body: impl FnOnce(&mut egui::Ui)) {
    egui::Grid::new(id)
        .num_columns(2)
        .spacing([14.0, 6.0])
        .min_col_width(110.0)
        .show(ui, body);
}

/// A problem with the settings that makes the decoder useless, if any.
pub fn validate(cfg: &Config) -> Option<String> {
    match cfg {
        Config::Uart(_) => None,
        Config::Spi(c) => {
            let mut used: Vec<(u8, &str)> = vec![(c.sck, "SCK")];
            for (v, n) in [(c.cs, "CS"), (c.mosi, "MOSI"), (c.miso, "MISO")] {
                if let Some(v) = v {
                    used.push((v, n));
                }
            }
            for (k, (ch, name)) in used.iter().enumerate() {
                if let Some((_, other)) = used[..k].iter().find(|(c2, _)| c2 == ch) {
                    return Some(format!("CH{ch} is used for both {other} and {name}."));
                }
            }
            if c.mosi.is_none() && c.miso.is_none() {
                return Some("Select MOSI and/or MISO: nothing would be decoded.".into());
            }
            None
        }
        Config::I2c(c) => (c.scl == c.sda).then(|| format!("SCL and SDA are both CH{}.", c.scl)),
    }
}

/// Baud rate detection between the settings and the application, which owns the capture: the settings
/// ask by setting `request`, the application answers in `result`.
#[derive(Default)]
pub struct BaudProbe {
    /// Detect the baud rate on this channel.
    pub request: Option<u8>,
    /// The last answer and the channel it was for.
    pub result: Option<(u8, Result<bf_decode::baud::BaudGuess, String>)>,
    /// Detect the frame format of this UART (channel, baud rate and polarity are used).
    pub format_request: Option<UartConfig>,
    /// The last answer and the settings it was for.
    pub format_result: Option<(UartConfig, Result<bf_decode::framing::FormatGuess, String>)>,
}

/// The settings that tell decoders of one kind apart, e.g. "CH0, 19200 8E1".
pub fn short_summary(cfg: &Config) -> String {
    match cfg {
        Config::Uart(c) => {
            let parity = match c.parity {
                Parity::None => 'N',
                Parity::Even => 'E',
                Parity::Odd => 'O',
                Parity::Mark => 'M',
                Parity::Space => 'S',
            };
            format!(
                "CH{}, {} {}{parity}{}",
                c.rx, c.baud, c.data_bits, c.stop_bits
            )
        }
        Config::Spi(c) => {
            let mut parts = vec![format!("SCK CH{}", c.sck)];
            for (name, ch) in [("CS", c.cs), ("MOSI", c.mosi), ("MISO", c.miso)] {
                if let Some(ch) = ch {
                    parts.push(format!("{name} CH{ch}"));
                }
            }
            parts.join(", ")
        }
        Config::I2c(c) => format!("SCL CH{}, SDA CH{}", c.scl, c.sda),
    }
}

/// Ending UART telegrams at a line feed is for text only: with values not shown as ASCII the option is
/// switched off. True when that changed the settings.
pub fn lf_only_for_ascii(cfg: &mut Config, fmt: Format) -> bool {
    match cfg {
        Config::Uart(c) if fmt != Format::Ascii && c.telegram_lf => {
            c.telegram_lf = false;
            true
        }
        _ => false,
    }
}

/// The settings a frame format answer depends on.
fn same_line(a: &UartConfig, b: &UartConfig) -> bool {
    a.rx == b.rx && a.baud == b.baud && a.invert == b.invert && a.lsb_first == b.lsb_first
}

/// Edit a decoder's settings; true when anything changed. `display` is the format values are shown in;
/// `None` leaves out what only matters for display (a trigger does not need it). A `probe` adds baud
/// rate and frame format detection to UART settings.
pub fn config_ui_with(
    ui: &mut egui::Ui,
    id: &str,
    cfg: &mut Config,
    names: &[String],
    display: Option<Format>,
    probe: Option<&mut BaudProbe>,
) -> bool {
    let mut changed = false;
    match cfg {
        Config::Uart(c) => {
            section(ui, "Channel", |ui| {
                grid(ui, &format!("{id}g1"), |ui| {
                    changed |= param(ui, "RX", help::UART_RX, |ui| {
                        channel_combo(ui, &format!("{id}rx"), &mut c.rx, names)
                    });
                });
            });
            if let Some(probe) = probe {
                section(ui, "Detect", |ui| {
                    grid(ui, &format!("{id}g5"), |ui| {
                        changed |= param(ui, "Baud rate", help::UART_DETECT, |ui| {
                            let mut used = false;
                            let mut r = ui
                                .horizontal(|ui| {
                                    if ui.button("Detect").clicked() {
                                        probe.request = Some(c.rx);
                                    }
                                    match &probe.result {
                                        Some((rx, Ok(g))) if *rx == c.rx => {
                                            let text = if g.snapped {
                                                format!("{} baud", g.baud)
                                            } else {
                                                format!("{} baud (not a standard rate)", g.baud)
                                            };
                                            ui.label(text);
                                            ui.label(
                                                RichText::new(format!(
                                                    "fit {:.0} %, {} pulses",
                                                    g.fit * 100.0,
                                                    g.widths
                                                ))
                                                .color(theme::TEXT_DIM),
                                            );
                                            if ui.button("Use").clicked() {
                                                c.baud = g.baud;
                                                used = true;
                                            }
                                            if let Some(n) = &g.note {
                                                ui.label(
                                                    RichText::new(n.as_str())
                                                        .color(theme::DEC_WARN),
                                                );
                                            }
                                        }
                                        Some((rx, Err(e))) if *rx == c.rx => {
                                            ui.colored_label(theme::DEC_ERR, e);
                                        }
                                        _ => {}
                                    }
                                })
                                .response;
                            if used {
                                r.mark_changed();
                            }
                            r
                        });
                        changed |= param(ui, "Frame format", help::UART_DETECT_FORMAT, |ui| {
                            let mut used = false;
                            let mut r = ui
                                .horizontal(|ui| {
                                    if ui.button("Detect").clicked() {
                                        probe.format_request = Some(c.clone());
                                    }
                                    match &probe.format_result {
                                        Some((for_cfg, Ok(g))) if same_line(for_cfg, c) => {
                                            ui.label(g.short());
                                            ui.label(
                                                RichText::new(format!(
                                                    "{} characters, {} with errors",
                                                    g.chars, g.errors
                                                ))
                                                .color(theme::TEXT_DIM),
                                            );
                                            if ui.button("Use").clicked() {
                                                c.data_bits = g.data_bits;
                                                c.parity = g.parity;
                                                c.stop_bits = g.stop_bits;
                                                used = true;
                                            }
                                            if let Some(n) = &g.note {
                                                ui.label(
                                                    RichText::new(n.as_str())
                                                        .color(theme::TEXT_DIM),
                                                );
                                            }
                                        }
                                        Some((for_cfg, Err(e))) if same_line(for_cfg, c) => {
                                            ui.colored_label(theme::DEC_ERR, e);
                                        }
                                        _ => {}
                                    }
                                })
                                .response;
                            if used {
                                r.mark_changed();
                            }
                            r
                        });
                    });
                });
            }
            section(ui, "Settings", |ui| {
                grid(ui, &format!("{id}g2"), |ui| {
                    changed |= param(ui, "Baud rate", help::UART_BAUD, |ui| {
                        ui.add(
                            egui::DragValue::new(&mut c.baud)
                                .speed(100.0)
                                .range(300.0..=20e6)
                                .suffix(" baud"),
                        )
                    });
                    changed |= param(ui, "Data bits", help::UART_BITS, |ui| {
                        ui.add(egui::DragValue::new(&mut c.data_bits).range(5..=9))
                    });
                    changed |= param(ui, "Parity", help::UART_PARITY, |ui| {
                        let mut ch = false;
                        let mut r = egui::ComboBox::from_id_salt(format!("{id}par"))
                            .selected_text(c.parity.label())
                            .width(100.0)
                            .show_ui(ui, |ui| {
                                for p in Parity::ALL {
                                    ch |=
                                        ui.selectable_value(&mut c.parity, p, p.label()).changed();
                                }
                            })
                            .response;
                        if ch {
                            r.mark_changed();
                        }
                        r
                    });
                    changed |= param(ui, "Stop bits", help::UART_STOP, |ui| {
                        let mut ch = false;
                        let mut r = egui::ComboBox::from_id_salt(format!("{id}stop"))
                            .selected_text(format!("{}", c.stop_bits))
                            .width(100.0)
                            .show_ui(ui, |ui| {
                                for s in [1.0f32, 1.5, 2.0] {
                                    ch |= ui
                                        .selectable_value(&mut c.stop_bits, s, format!("{s}"))
                                        .changed();
                                }
                            })
                            .response;
                        if ch {
                            r.mark_changed();
                        }
                        r
                    });
                    changed |= param(ui, "Bit order", help::UART_LSB, |ui| {
                        ui.checkbox(&mut c.lsb_first, "LSB first")
                    });
                    changed |= param(ui, "Polarity", help::UART_INVERT, |ui| {
                        ui.checkbox(&mut c.invert, "Inverted (idle low)")
                    });
                });
            });
            if display.is_some() {
                section(ui, "High-level protocol", |ui| {
                    grid(ui, &format!("{id}g4"), |ui| {
                        changed |= param(ui, "Protocol", help::UART_PROTOCOL, |ui| {
                            let mut ch = false;
                            let mut r = egui::ComboBox::from_id_salt(format!("{id}proto"))
                                .selected_text(c.protocol.label())
                                .width(140.0)
                                .show_ui(ui, |ui| {
                                    for p in Protocol::ALL {
                                        ch |= ui
                                            .selectable_value(&mut c.protocol, p, p.label())
                                            .changed();
                                    }
                                })
                                .response;
                            if ch {
                                r.mark_changed();
                            }
                            r
                        });
                    });
                });
            }
            if display.is_some() && c.protocol == Protocol::Plain {
                section(ui, "Telegrams", |ui| {
                    grid(ui, &format!("{id}g3"), |ui| {
                        changed |= param(ui, "Join characters", help::UART_TELEGRAMS, |ui| {
                            ui.checkbox(
                                &mut c.telegrams,
                                "Show consecutive characters as telegrams",
                            )
                        });
                        if c.telegrams {
                            changed |= param(ui, "Idle gap", help::UART_GAP, |ui| {
                                gap_editor(ui, &format!("{id}gapunit"), c)
                            });
                            // Line feeds only mean something in text.
                            if display == Some(Format::Ascii) {
                                changed |= param(ui, "Line feed", help::UART_LF, |ui| {
                                    ui.checkbox(&mut c.telegram_lf, "End a telegram after LF (\\n)")
                                });
                            }
                        }
                    });
                });
            }
        }
        Config::Spi(c) => {
            section(ui, "Channels", |ui| {
                grid(ui, &format!("{id}g1"), |ui| {
                    changed |= param(ui, "SCK", help::SPI_SCK, |ui| {
                        channel_combo(ui, &format!("{id}sck"), &mut c.sck, names)
                    });
                    changed |= param(ui, "CS", help::SPI_CS, |ui| {
                        optional_channel_combo(ui, &format!("{id}cs"), &mut c.cs, names)
                    });
                    changed |= param(ui, "MOSI", help::SPI_MOSI, |ui| {
                        optional_channel_combo(ui, &format!("{id}mosi"), &mut c.mosi, names)
                    });
                    changed |= param(ui, "MISO", help::SPI_MISO, |ui| {
                        optional_channel_combo(ui, &format!("{id}miso"), &mut c.miso, names)
                    });
                });
            });
            section(
                ui,
                &format!(
                    "Mode {} and word format",
                    (c.cpol as u8) << 1 | c.cpha as u8
                ),
                |ui| {
                    grid(ui, &format!("{id}g2"), |ui| {
                        changed |= param(ui, "Clock polarity", help::SPI_CPOL, |ui| {
                            ui.checkbox(&mut c.cpol, "CPOL: clock idles high")
                        });
                        changed |= param(ui, "Clock phase", help::SPI_CPHA, |ui| {
                            ui.checkbox(&mut c.cpha, "CPHA: sample on the trailing edge")
                        });
                        changed |= param(ui, "Bit order", help::SPI_MSB, |ui| {
                            ui.checkbox(&mut c.msb_first, "MSB first")
                        });
                        changed |= param(ui, "CS level", help::SPI_CS_LOW, |ui| {
                            ui.checkbox(&mut c.cs_active_low, "CS active low")
                        });
                        changed |= param(ui, "Bits per word", help::SPI_BITS, |ui| {
                            ui.add(egui::DragValue::new(&mut c.bits).range(2..=32))
                        });
                    });
                },
            );
        }
        Config::I2c(c) => {
            section(ui, "Channels", |ui| {
                grid(ui, &format!("{id}g1"), |ui| {
                    changed |= param(ui, "SCL", help::I2C_SCL, |ui| {
                        channel_combo(ui, &format!("{id}scl"), &mut c.scl, names)
                    });
                    changed |= param(ui, "SDA", help::I2C_SDA, |ui| {
                        channel_combo(ui, &format!("{id}sda"), &mut c.sda, names)
                    });
                });
            });
        }
    }
    changed
}

/// Things the setup screen asks the application to do.
pub enum Action {
    Add(DecoderKind),
    Remove(usize),
    /// The settings of this decoder changed: it has to start over.
    Rebuild(usize),
}

#[derive(Default)]
pub struct SetupState {
    pub selected: usize,
    pub probe: BaudProbe,
    cache: Option<(Config, Vec<String>, crate::decode_preview::Preview)>,
}

fn kind_color(k: DecoderKind) -> Color32 {
    match k {
        DecoderKind::Uart => theme::DEC_ADDR,
        DecoderKind::Spi => theme::DEC_MARK,
        DecoderKind::I2c => theme::DEC_WARN,
    }
}

const KIND_BLURB: [(DecoderKind, &str); 3] = [
    (DecoderKind::Uart, "Asynchronous serial: one data line, no clock. Frames of start bit, data, optional parity and stop bits."),
    (DecoderKind::Spi, "Synchronous serial with a clock: MOSI/MISO data words, framed by chip select. Four modes (CPOL/CPHA)."),
    (DecoderKind::I2c, "Two-wire bus: SCL clock and SDA data with START/STOP conditions, a 7-bit address and acknowledged bytes."),
];

/// The decoder setup screen: the list on the left, the selected decoder's settings and an example
/// trace on the right. `end` is the capture length (progress display).
pub fn setup_ui(
    ui: &mut egui::Ui,
    st: &mut SetupState,
    entries: &mut [Entry],
    names: &[String],
    end: u64,
) -> Vec<Action> {
    let mut actions = Vec::new();
    st.selected = st.selected.min(entries.len().saturating_sub(1));
    let height = ui.available_height().max(300.0);
    ui.horizontal_top(|ui| {
        // ---- left: the list
        ui.allocate_ui_with_layout(
            vec2(210.0, height),
            egui::Layout::top_down(egui::Align::Min),
            |ui| {
                ui.set_min_size(vec2(210.0, height));
                ui.label(RichText::new("Decoders").color(theme::TEXT_DIM));
                egui::ScrollArea::vertical()
                    .max_height(height - 110.0)
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        if entries.is_empty() {
                            ui.label(RichText::new("none yet").color(theme::TEXT_DIM));
                        }
                        for (i, e) in entries.iter().enumerate() {
                            let sel = st.selected == i;
                            let bad = validate(&e.cfg).is_some();
                            let (rect, resp) = ui.allocate_exact_size(
                                vec2(ui.available_width(), 38.0),
                                Sense::click(),
                            );
                            let bg = if sel {
                                theme::SELECT
                            } else if resp.hovered() {
                                Color32::from_white_alpha(0x10)
                            } else {
                                Color32::TRANSPARENT
                            };
                            ui.painter().rect_filled(rect, 0.0, bg);
                            ui.painter().rect_filled(
                                Rect::from_min_size(rect.left_top(), vec2(3.0, rect.height())),
                                0.0,
                                kind_color(e.cfg.kind()),
                            );
                            ui.painter().text(
                                rect.left_top() + vec2(12.0, 8.0),
                                egui::Align2::LEFT_CENTER,
                                &e.name,
                                egui::FontId::monospace(12.0),
                                theme::TEXT,
                            );
                            let done = (e.handle.decoded_to() as f64 / end.max(1) as f64 * 100.0)
                                .min(100.0);
                            let status = if bad {
                                "check settings".to_string()
                            } else {
                                format!("{} items, {done:.0}%", e.handle.anns.read().len())
                            };
                            ui.painter().text(
                                rect.left_top() + vec2(12.0, 26.0),
                                egui::Align2::LEFT_CENTER,
                                format!("{}  {status}", e.cfg.kind().label()),
                                egui::FontId::monospace(10.0),
                                if bad { theme::DEC_ERR } else { theme::TEXT_DIM },
                            );
                            if resp.clicked() {
                                st.selected = i;
                            }
                        }
                    });
                ui.add_space(6.0);
                ui.separator();
                ui.label(RichText::new("Add decoder").color(theme::TEXT_DIM));
                ui.horizontal(|ui| {
                    for k in DecoderKind::ALL {
                        if ui
                            .button(format!("+ {}", k.label()))
                            .on_hover_text(KIND_BLURB.iter().find(|b| b.0 == k).map_or("", |b| b.1))
                            .clicked()
                        {
                            actions.push(Action::Add(k));
                        }
                    }
                });
            },
        );
        ui.separator();

        // ---- right: settings and example
        ui.allocate_ui_with_layout(
            vec2(ui.available_width(), height),
            egui::Layout::top_down(egui::Align::Min),
            |ui| {
                if entries.is_empty() {
                    ui.add_space(30.0);
                    ui.label(RichText::new("Add a decoder on the left.").size(15.0));
                    ui.add_space(10.0);
                    for (k, text) in KIND_BLURB {
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(format!("{:<5}", k.label()))
                                    .color(kind_color(k))
                                    .strong(),
                            );
                            ui.label(text);
                        });
                    }
                    return;
                }
                let i = st.selected;
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        let e = &mut entries[i];
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(e.cfg.kind().label())
                                    .color(Color32::BLACK)
                                    .background_color(kind_color(e.cfg.kind()))
                                    .strong(),
                            );
                            ui.add(egui::TextEdit::singleline(&mut e.name).desired_width(160.0))
                                .on_hover_text(help::NAME);
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if ui.button("Remove decoder").clicked() {
                                        actions.push(Action::Remove(i));
                                    }
                                },
                            );
                        });
                        if ui.available_width() >= 820.0 {
                            // Settings on the left, the example next to them.
                            ui.columns(2, |cols| {
                                let (left, right) = cols.split_at_mut(1);
                                decoder_settings(&mut left[0], e, i, st, names, &mut actions);
                                decoder_example(&mut right[0], e, st, names);
                            });
                        } else {
                            decoder_settings(ui, e, i, st, names, &mut actions);
                            decoder_example(ui, e, st, names);
                        }
                    });
            },
        );
    });
    actions
}

/// Display format and decoder settings of entry `i`.
fn decoder_settings(
    ui: &mut egui::Ui,
    e: &mut Entry,
    i: usize,
    st: &mut SetupState,
    names: &[String],
    actions: &mut Vec<Action>,
) {
    section(ui, "Display", |ui| {
        grid(ui, "disp", |ui| {
            param(ui, "Show values as", help::FORMAT, |ui| {
                let mut ch = false;
                let mut r = egui::ComboBox::from_id_salt("fmt")
                    .selected_text(e.format.label())
                    .width(100.0)
                    .show_ui(ui, |ui| {
                        for f in Format::ALL {
                            ch |= ui.selectable_value(&mut e.format, f, f.label()).changed();
                        }
                    })
                    .response;
                if ch {
                    r.mark_changed();
                }
                r
            });
        });
    });
    if lf_only_for_ascii(&mut e.cfg, e.format) {
        actions.push(Action::Rebuild(i));
    }
    let mut cfg = e.cfg.clone();
    if config_ui_with(
        ui,
        &format!("d{}", e.handle.id.0),
        &mut cfg,
        names,
        Some(e.format),
        Some(&mut st.probe),
    ) {
        e.cfg = cfg;
        actions.push(Action::Rebuild(i));
    }
    if let Some(msg) = validate(&e.cfg) {
        ui.add_space(6.0);
        ui.colored_label(theme::DEC_ERR, format!("! {msg}"));
    }
}

/// The example traces for an entry's settings (built again when they change).
fn decoder_example(ui: &mut egui::Ui, e: &Entry, st: &mut SetupState, names: &[String]) {
    section(ui, "Example: what the decoder expects", |ui| {
        let stale = st
            .cache
            .as_ref()
            .is_none_or(|(c, n, _)| *c != e.cfg || n != names);
        if stale {
            st.cache = Some((
                e.cfg.clone(),
                names.to_vec(),
                crate::decode_preview::build(&e.cfg, names),
            ));
        }
        if let Some((_, _, p)) = &st.cache {
            if validate(&e.cfg).is_some_and(|_| matches!(e.cfg, Config::I2c(_))) {
                ui.label(
                    RichText::new("Give SCL and SDA different channels to see the example.")
                        .color(theme::TEXT_DIM),
                );
            } else {
                crate::decode_preview::draw(ui, p, e.format);
                ui.add_space(4.0);
                for line in &p.caption {
                    ui.label(RichText::new(line).color(theme::TEXT_DIM).size(11.0));
                }
                ui.label(RichText::new("Dots mark where the decoder samples; the lane below shows what it decodes.").color(theme::TEXT_DIM).size(11.0));
            }
        }
    });
}

/// The annotation the user picked, in the table or in the waveform.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Selected {
    pub id: u32,
    pub lane: u8,
    pub start: u64,
}

fn info_text(a: &Annotation) -> String {
    use bf_decode::flag;
    let mut v: Vec<&str> = Vec::new();
    for (f, t) in [
        (flag::FRAMING, "framing error"),
        (flag::PARITY, "parity error"),
        (flag::BREAK, "break"),
        (flag::INCOMPLETE, "incomplete word"),
        (flag::NAK, "not acknowledged"),
        (flag::REPEATED, "repeated start"),
        (flag::CHECKSUM, "CRC error"),
    ] {
        if a.flags & f != 0 {
            v.push(t);
        }
    }
    if a.kind == Kind::Address {
        v.push(if a.flags & flag::READ != 0 {
            "read"
        } else {
            "write"
        });
    }
    v.join(", ")
}

/// The info column: a Modbus frame lists its bytes (in hex) before the info words.
fn info_with_bytes(a: &Annotation, chars: &[Annotation]) -> String {
    let info = info_text(a);
    if a.kind != Kind::ModbusFrame {
        return info;
    }
    let bytes = bf_decode::telegram_text(bf_decode::telegram_chars(chars, a), Format::Hex, 300);
    if info.is_empty() {
        bytes
    } else {
        format!("{bytes}  ({info})")
    }
}

#[derive(Clone, PartialEq)]
struct SearchKey {
    id: u32,
    lane: usize,
    /// Lower case, for the text search.
    query: String,
    /// As typed, for the pattern.
    raw: String,
    fmt: Format,
    errors_only: bool,
    /// Start of the first annotation: changes when old data is evicted (indices shift).
    first: u64,
}

/// An incremental search over one lane: annotations are scanned a slice per frame, new ones as they
/// arrive, so even millions of items never freeze the UI.
struct Search {
    key: SearchKey,
    scanned: usize,
    /// Sorted, without duplicates.
    hits: Vec<u32>,
    /// The query as a pattern of the decoder's trigger language, and where it applies.
    pattern: Option<bf_decode::trigger::PatternSearch>,
    scope: PatternScope,
}

/// How a pattern search applies to the rows of a lane.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PatternScope {
    /// The rows are the items (UART characters, SPI words, I2C events): every row of a match is a hit.
    Items,
    /// The rows hold characters of lane 0 (telegrams, Modbus frames): a row containing a match is a hit.
    Contents,
    None,
}

fn pattern_scope(cfg: &Config, lane: usize) -> PatternScope {
    match (cfg, lane) {
        (Config::Uart(_), 0) | (Config::Spi(_), 0 | 1) | (Config::I2c(_), 0) => PatternScope::Items,
        (Config::Uart(_), 1) => PatternScope::Contents,
        _ => PatternScope::None,
    }
}

/// The query in the trigger pattern language of the decoder (UART: hex, or text when it has a
/// backslash escape; SPI: words; I2C: events). `None` if it is not a pattern.
fn query_pattern(cfg: &Config, raw: &str) -> Option<Vec<PatItem>> {
    let raw = raw.trim();
    let items = match cfg {
        Config::Uart(_) if raw.contains('\\') => crate::setup::parse_text_pattern(raw),
        Config::Uart(_) => crate::setup::parse_hex_pattern(raw),
        Config::Spi(c) => crate::setup::parse_word_pattern(raw, c.bits as u32),
        Config::I2c(_) => crate::setup::parse_i2c_pattern(raw),
    }
    .ok()?;
    items.iter().any(|i| *i != PatItem::Gap).then_some(items)
}

/// Add row `i` to the sorted hits (rows of a pattern match can come after later text hits).
fn add_hit(hits: &mut Vec<u32>, i: usize) {
    let i = i as u32;
    match hits.last() {
        Some(&last) if last >= i => {
            if let Err(at) = hits.binary_search(&i) {
                hits.insert(at, i);
            }
        }
        _ => hits.push(i),
    }
}

/// Search rows `s.scanned..end` of `lane`: text matches, and matches of the pattern.
fn search_slice(
    s: &mut Search,
    lane: &[Annotation],
    end: usize,
    lane_name: &str,
    chars: &[Annotation],
) {
    let (query, fmt, errors_only) = (s.key.query.clone(), s.key.fmt, s.key.errors_only);
    let ok = |a: &Annotation| !errors_only || a.is_error();
    // On telegram and frame rows a query that is a pattern follows the pattern's rules only (Modbus
    // frames from their first byte, as in the trigger); words still search the text.
    let text = !(s.scope == PatternScope::Contents && s.pattern.is_some());
    for (i, a) in lane.iter().enumerate().take(end).skip(s.scanned) {
        let mut hit =
            text && ok(a) && !query.is_empty() && matches(a, lane_name, &query, fmt, chars);
        hit |= errors_only && query.is_empty() && a.is_error();
        match (&mut s.pattern, s.scope) {
            (Some(p), PatternScope::Contents) => {
                // A Modbus frame matches from its first byte, as in the trigger.
                let inside = bf_decode::telegram_chars(chars, a);
                hit |= ok(a)
                    && if a.kind == Kind::ModbusFrame {
                        p.found_at_start(inside)
                    } else {
                        p.found_in(inside)
                    };
            }
            (Some(p), PatternScope::Items) => {
                if let Some(first) = p.feed(i, a) {
                    for (j, b) in lane.iter().enumerate().take(i + 1).skip(first) {
                        if ok(b) {
                            add_hit(&mut s.hits, j);
                        }
                    }
                }
            }
            _ => {}
        }
        if hit {
            add_hit(&mut s.hits, i);
        }
    }
    s.scanned = end;
}

/// Case-insensitive match of the lower-case `q` against the value (in the display format and in hex,
/// so "A5" finds 0xA5 whatever the format), the lane name and the info words ("nak", "framing", ...).
fn matches(a: &Annotation, lane: &str, q: &str, fmt: Format, chars: &[Annotation]) -> bool {
    let q = q.strip_prefix("0x").unwrap_or(q);
    if q.is_empty() {
        return true;
    }
    if a.kind == Kind::Telegram {
        // A telegram matches on its text (in the display format and in hex) and its lane name (byte
        // patterns are looked for by the search itself).
        let cs = bf_decode::telegram_chars(chars, a);
        let mut t = bf_decode::telegram_text(cs, fmt, usize::MAX);
        t.make_ascii_lowercase();
        if t.contains(q) {
            return true;
        }
        let mut h = bf_decode::telegram_text(cs, Format::Hex, usize::MAX);
        h.make_ascii_lowercase();
        return h.contains(q) || lane.to_ascii_lowercase().contains(q);
    }
    let mut label = a.label(fmt);
    label.make_ascii_lowercase();
    if label.contains(q) {
        return true;
    }
    if fmt != Format::Hex {
        let mut hex = a.label(Format::Hex);
        hex.make_ascii_lowercase();
        if hex.contains(q) {
            return true;
        }
    }
    lane.to_ascii_lowercase().contains(q)
        || (a.flags != 0 || a.kind == Kind::Address || a.kind == Kind::ModbusFrame)
            && info_with_bytes(a, chars).to_ascii_lowercase().contains(q)
}

/// The row to jump to from the selection (the row whose annotation starts at `sel`) in direction `dir`
/// (+1 next, -1 previous). With no selection: the first (next) or last (previous) row. Rows are sorted
/// by start; the ends are not wrapped.
pub fn neighbour_row(
    total: usize,
    start_of: impl Fn(usize) -> u64,
    sel: Option<u64>,
    dir: i32,
) -> usize {
    debug_assert!(total > 0);
    let Some(s) = sel else {
        return if dir >= 0 { 0 } else { total - 1 };
    };
    // First row starting at or after the selection.
    let (mut lo, mut hi) = (0usize, total);
    while lo < hi {
        let mid = (lo + hi) / 2;
        if start_of(mid) < s {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    if dir >= 0 {
        let on_selection = lo < total && start_of(lo) == s;
        (lo + on_selection as usize).min(total - 1)
    } else {
        lo.saturating_sub(1)
    }
}

/// The time column: of a row starting at `start`, or with `delta` since the row before it (`before`; the
/// first row has none).
fn row_time(start: u64, before: Option<u64>, delta: bool, origin: f64, rate: f64) -> String {
    let t = match (delta, before) {
        (false, _) => start as f64 - origin,
        (true, Some(b)) => start as f64 - b as f64,
        (true, None) => return "-".into(),
    };
    format_split(t / rate, 1.0 / rate)
}

/// State of the annotation table.
#[derive(Default)]
pub struct TableState {
    /// Index into the decoder list.
    pub entry: usize,
    pub lane: usize,
    pub filter: String,
    pub errors_only: bool,
    /// Show the time since the previous row instead of the time in the capture.
    pub delta: bool,
    search: Option<Search>,
    /// Reveal this annotation (index in the lane) on the next frame.
    pub scroll_to: Option<usize>,
    /// Jump to the next (+1) or previous (-1) row after the selection on the next frame.
    pub nav: Option<i32>,
    pub focus_search: bool,
    columns: Columns,
}

type ColumnsKey = (u32, usize, Format, bool);

/// Widths of the table's columns except the last, which takes the rest. They fit the rows seen so far
/// and never shrink while the same lane is shown, so that scrolling does not make them jump.
#[derive(Default)]
struct Columns {
    /// Decoder, lane, format and delta times: what the widths are for.
    key: Option<ColumnsKey>,
    widths: [f32; 3],
}

impl Columns {
    /// Space between columns.
    const GAP: f32 = 16.0;
    /// Wider texts are cut off.
    const MAX: f32 = 420.0;

    /// Start over with `min` widths if something else is shown than before.
    fn show(&mut self, key: ColumnsKey, min: [f32; 3]) {
        if self.key != Some(key) {
            *self = Self {
                key: Some(key),
                widths: min,
            };
        }
    }

    /// Make room for a text of this width in column `i`.
    fn fit(&mut self, i: usize, width: f32) {
        if let Some(w) = self.widths.get_mut(i) {
            *w = w.max(width.ceil() + Self::GAP).min(Self::MAX);
        }
    }

    /// Left and right edge of the text in column `i` of a row `total` wide.
    fn span(&self, i: usize, total: f32) -> (f32, f32) {
        let left = 4.0 + self.widths[..i.min(3)].iter().sum::<f32>();
        match self.widths.get(i) {
            // Half the gap stays free of text that is cut off.
            Some(w) => (left, left + w - Self::GAP / 2.0),
            None => (left, total.max(left)),
        }
    }

    fn cell(&self, i: usize, row: Rect) -> Rect {
        let (left, right) = self.span(i, row.width());
        Rect::from_min_max(
            row.left_top() + vec2(left, 0.0),
            egui::pos2(row.left() + right, row.bottom()),
        )
    }
}

const ROW_H: f32 = 16.0;
/// Annotations examined per frame while searching.
const SEARCH_SLICE: usize = 150_000;

/// The annotation table: every annotation of one decoder lane over the whole capture, newest data
/// included, optionally narrowed by a search. Returns the annotation the user selected (clicked, or
/// reached with Prev/Next), with its index in the lane.
pub fn table_ui(
    ui: &mut egui::Ui,
    st: &mut TableState,
    entries: &[Entry],
    rate: f64,
    origin: f64,
    selected: Option<Selected>,
) -> Option<(Selected, Annotation, usize)> {
    let mut picked: Option<(Selected, Annotation, usize)> = None;
    if entries.is_empty() {
        ui.label(
            RichText::new("No decoders yet. Use Setup > Protocol decoders... to add one.")
                .color(theme::TEXT_DIM),
        );
        return None;
    }
    st.entry = st.entry.min(entries.len() - 1);
    let entry = &entries[st.entry];
    let used: Vec<usize> = (0..entry.handle.lanes.len())
        .filter(|&l| entry.handle.lane_used.get(l).copied().unwrap_or(false))
        .collect();
    if !used.contains(&st.lane) {
        st.lane = used.first().copied().unwrap_or(0);
    }
    let anns = entry.handle.anns.read();
    let lane: &[Annotation] = anns.lanes.get(st.lane).map_or(&[], |v| v.as_slice());
    let lane_name = entry.handle.lanes.get(st.lane).copied().unwrap_or("");
    let chars: &[Annotation] = anns.lanes.first().map_or(&[], |v| v.as_slice());
    let is_telegram_lane = lane_name == "Telegram";
    let fmt = entry.format;

    // ---- search state
    let query = st.filter.trim().to_lowercase();
    let wanted = !query.is_empty() || st.errors_only;
    let key = SearchKey {
        id: entry.handle.id.0,
        lane: st.lane,
        query: query.clone(),
        raw: st.filter.trim().to_string(),
        fmt,
        errors_only: st.errors_only,
        first: lane.first().map_or(0, |a| a.start),
    };
    if !wanted {
        st.search = None;
    } else {
        if st.search.as_ref().is_none_or(|s| s.key != key) {
            let scope = pattern_scope(&entry.cfg, st.lane);
            let pattern = (scope != PatternScope::None)
                .then(|| query_pattern(&entry.cfg, &key.raw))
                .flatten()
                .and_then(bf_decode::trigger::PatternSearch::new);
            st.search = Some(Search {
                key,
                scanned: 0,
                hits: Vec::new(),
                pattern,
                scope,
            });
        }
        if let Some(s) = st.search.as_mut() {
            let end = (s.scanned + SEARCH_SLICE).min(lane.len());
            search_slice(s, lane, end, lane_name, chars);
            if s.scanned < lane.len() {
                ui.ctx().request_repaint();
            }
        }
    }
    let search = st.search.as_ref();
    let total = search.map_or(lane.len(), |s| s.hits.len());
    let ann_index = |row: usize| search.map_or(row, |s| s.hits[row] as usize);

    // ---- toolbar
    ui.horizontal(|ui| {
        egui::ComboBox::from_id_salt("tabledec").selected_text(entry.name.clone()).width(90.0).show_ui(ui, |ui| {
            for (i, e) in entries.iter().enumerate() {
                ui.selectable_value(&mut st.entry, i, e.name.clone());
            }
        });
        if used.len() > 1 {
            egui::ComboBox::from_id_salt("tablelane").selected_text(lane_name).width(80.0).show_ui(ui, |ui| {
                for &l in &used {
                    ui.selectable_value(&mut st.lane, l, entry.handle.lanes[l]);
                }
            });
        }
        let r = ui
            .add(egui::TextEdit::singleline(&mut st.filter).hint_text("search (Ctrl+F)").desired_width(190.0))
            .on_hover_text("Text or values, or a pattern as in the trigger: UART bytes (65 06, 4X, XX, 0b1010XXXX, * for anything in between; text with \\xXX and \\*), SPI words (A5 * 3X), I2C events (S @50W 01 XX! P). On the characters, words or events every item of a match is listed; telegrams and Modbus frames are listed when they contain one.");
        r.clone().on_hover_text("Finds values (in the display format or hex), lane names and words like nak, framing, parity, break, read, write.\nSearches the whole capture, not only what is visible.");
        if st.focus_search {
            r.request_focus();
            st.focus_search = false;
        }
        if !st.filter.is_empty() && ui.small_button("x").on_hover_text("Clear the search").clicked() {
            st.filter.clear();
        }
        ui.checkbox(&mut st.errors_only, "errors only").on_hover_text("Only annotations flagged as errors (framing, parity, break, incomplete word).");
        ui.checkbox(&mut st.delta, "delta time").on_hover_text("Show the time from the start of the previous row to the start of this one, instead of the time in the capture. While searching the previous row is the previous hit.");
        let prev = ui.button("Prev").on_hover_text("Previous row (Shift+F3)").clicked();
        let next = ui.button("Next").on_hover_text("Next row (F3)").clicked();
        if prev {
            st.nav = Some(-1);
        }
        if next {
            st.nav = Some(1);
        }
        let progress = search.filter(|s| s.scanned < lane.len()).map(|s| format!("  searching {}%", s.scanned * 100 / lane.len().max(1)));
        let count = if wanted { format!("{total} of {}", lane.len()) } else { format!("{total} items") };
        ui.label(RichText::new(format!("{count}{}", progress.unwrap_or_default())).color(theme::TEXT_DIM));
    });

    // ---- navigation with Prev/Next relative to the selection
    if let Some(dir) = st.nav.take() {
        if total > 0 {
            let sel_start = selected
                .filter(|s| s.id == entry.handle.id.0 && s.lane as usize == st.lane)
                .map(|s| s.start);
            let row = neighbour_row(total, |r| lane[ann_index(r)].start, sel_start, dir);
            let i = ann_index(row);
            let a = lane[i];
            picked = Some((
                Selected {
                    id: entry.handle.id.0,
                    lane: st.lane as u8,
                    start: a.start,
                },
                a,
                i,
            ));
            st.scroll_to = Some(i);
        }
    }

    // ---- header
    let delta = st.delta;
    let time_title = if delta { "Delta" } else { "Start" };
    let titles = if is_telegram_lane {
        [time_title, "Duration", "Bytes", "Text"]
    } else {
        [time_title, "Duration", "Value", "Info"]
    };
    let min = [140.0, 90.0, if is_telegram_lane { 70.0 } else { 110.0 }];
    let font = egui::FontId::monospace(11.0);
    let time_text = |row: usize| {
        let start = lane[ann_index(row)].start;
        let before = row.checked_sub(1).map(|r| lane[ann_index(r)].start);
        row_time(start, before, delta, origin, rate)
    };
    let cols = &mut st.columns;
    cols.show((entry.handle.id.0, st.lane, fmt, delta), min);
    // The last row has the longest time, or (before the trigger) the first one.
    if !delta && total > 0 {
        for row in [0, total - 1] {
            let g = ui
                .painter()
                .layout_no_wrap(time_text(row), font.clone(), theme::TEXT);
            cols.fit(0, g.size().x);
        }
    }
    // Drawn after the rows, which determine the widths.
    let (head, _) = ui.allocate_exact_size(vec2(ui.available_width(), ROW_H), Sense::hover());

    ui.painter().line_segment(
        [head.left_bottom(), head.right_bottom()],
        egui::Stroke::new(1.0_f32, theme::BORDER),
    );
    ui.add_space(2.0);

    // ---- rows
    let mut area = egui::ScrollArea::vertical().auto_shrink([false, false]);
    if let Some(idx) = st.scroll_to.take() {
        // Row of annotation `idx` (or the nearest hit after it), a few rows from the top.
        let row = match search {
            Some(s) => s.hits.partition_point(|&h| (h as usize) < idx),
            None => idx,
        };
        area = area.vertical_scroll_offset((row.saturating_sub(3)) as f32 * ROW_H);
    }
    area.show_rows(ui, ROW_H, total, |ui, range| {
        let mut rows = Vec::with_capacity(range.len());
        for row in range {
            let i = ann_index(row);
            let a = &lane[i];
            let color = if a.is_error() {
                theme::DEC_ERR
            } else {
                theme::TEXT
            };
            let (third, fourth) = if is_telegram_lane {
                let text = bf_decode::telegram_text(bf_decode::telegram_chars(chars, a), fmt, 400);
                (a.value.to_string(), (text, color))
            } else {
                let info_color = if a.flags == 0 && a.kind != Kind::Address {
                    theme::TEXT_DIM
                } else {
                    theme::DEC_WARN
                };
                (a.label(fmt), (info_with_bytes(a, chars), info_color))
            };
            let third_color = if is_telegram_lane {
                theme::TEXT_DIM
            } else {
                color
            };
            let duration = crate::view::format_duration((a.end - a.start) as f64 / rate);
            let cells = [
                (time_text(row), theme::TEXT),
                (duration, theme::TEXT_DIM),
                (third, third_color),
                fourth,
            ]
            .map(|(text, color)| ui.painter().layout_no_wrap(text, font.clone(), color));
            for (c, g) in cells.iter().enumerate() {
                cols.fit(c, g.size().x);
            }
            rows.push((i, cells));
        }
        for (i, cells) in rows {
            let a = &lane[i];
            let is_sel = selected.is_some_and(|s| {
                s.id == entry.handle.id.0 && s.lane as usize == st.lane && s.start == a.start
            });
            let (rect, resp) =
                ui.allocate_exact_size(vec2(ui.available_width(), ROW_H), Sense::click());
            if is_sel {
                ui.painter().rect_filled(rect, 0.0, theme::SELECT);
            } else if resp.hovered() {
                ui.painter()
                    .rect_filled(rect, 0.0, Color32::from_white_alpha(0x10));
            }
            for (c, g) in cells.into_iter().enumerate() {
                let cell = cols.cell(c, rect);
                let at = egui::pos2(cell.left(), rect.center().y - g.size().y / 2.0);
                ui.painter()
                    .with_clip_rect(cell.intersect(ui.clip_rect()))
                    .galley(at, g, theme::TEXT);
            }
            if resp.clicked() {
                picked = Some((
                    Selected {
                        id: entry.handle.id.0,
                        lane: st.lane as u8,
                        start: a.start,
                    },
                    *a,
                    i,
                ));
            }
        }
    });
    for (c, title) in titles.into_iter().enumerate() {
        let cell = cols.cell(c, head);
        ui.painter().with_clip_rect(cell).text(
            cell.left_center(),
            egui::Align2::LEFT_CENTER,
            title,
            egui::FontId::monospace(11.0),
            theme::TEXT_DIM,
        );
    }
    picked
}

impl Config {
    /// Check settings read from a file (the UI itself only offers valid ones): a decoder with a
    /// channel beyond the captured ones, or an impossible frame format, is refused instead of run.
    pub fn check_file_values(&self, channels: u8) -> Result<(), String> {
        let (used, ok): (Vec<u8>, bool) = match self {
            Config::Uart(c) => (
                vec![c.rx],
                c.baud.is_finite()
                    && c.baud > 0.0
                    && (5..=9).contains(&c.data_bits)
                    && c.stop_bits.is_finite()
                    && (0.5..=2.0).contains(&c.stop_bits)
                    && c.telegram_gap.is_finite()
                    && c.telegram_gap >= 0.0,
            ),
            Config::Spi(c) => (
                [Some(c.sck), c.cs, c.mosi, c.miso]
                    .into_iter()
                    .flatten()
                    .collect(),
                (2..=32).contains(&c.bits),
            ),
            Config::I2c(c) => (vec![c.scl, c.sda], true),
        };
        if !ok {
            return Err("impossible settings".into());
        }
        match used.iter().find(|c| **c >= channels) {
            Some(c) => Err(format!(
                "uses channel {c}, but only {channels} are captured"
            )),
            None => Ok(()),
        }
    }
}

/// Quote a CSV field when it needs it.
fn csv_field(s: &str) -> String {
    if s.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

/// The decoded items of one lane as CSV: start and duration in seconds, sample numbers, kind,
/// text (as shown in the table) and notes (errors, direction).
pub fn write_csv(
    w: &mut impl std::io::Write,
    lanes: &[Vec<Annotation>],
    lane: usize,
    rate: f64,
    fmt: Format,
) -> std::io::Result<()> {
    writeln!(
        w,
        "start_s,duration_s,start_sample,end_sample,kind,text,notes"
    )?;
    let empty = Vec::new();
    let items = lanes.get(lane).unwrap_or(&empty);
    let chars = lanes.first().map_or(&[][..], |v| v.as_slice());
    for a in items {
        let text = if a.kind == Kind::Telegram {
            bf_decode::telegram_text(bf_decode::telegram_chars(chars, a), fmt, usize::MAX)
        } else {
            a.label(fmt)
        };
        writeln!(
            w,
            "{:.12},{:.12},{},{},{:?},{},{}",
            a.start as f64 / rate,
            (a.end - a.start) as f64 / rate,
            a.start,
            a.end,
            a.kind,
            csv_field(&text),
            csv_field(&info_with_bytes(a, chars))
        )?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use bf_decode::flag;

    fn ann(kind: Kind, value: u32, flags: u8) -> Annotation {
        Annotation {
            start: 0,
            end: 10,
            value,
            lane: 0,
            kind,
            bits: 8,
            flags,
        }
    }

    #[test]
    fn row_times_are_absolute_or_since_the_row_before() {
        let t = |before, delta| row_time(5_000, before, delta, 1_000.0, 1e6);
        assert_eq!(t(Some(3_500), false), "4 ms 000 µs");
        assert_eq!(t(Some(3_500), true), "1 ms 500 µs");
        assert_eq!(t(None, false), "4 ms 000 µs");
        assert_eq!(t(None, true), "-");
    }

    #[test]
    fn table_columns_do_not_overlap() {
        let mut c = Columns::default();
        c.show((1, 0, Format::Hex, false), [140.0, 90.0, 110.0]);
        for (i, w) in [(0, 230.0), (1, 60.0), (2, 9_000.0), (3, 9_000.0)] {
            c.fit(i, w);
        }
        // Wide enough for the text, up to the limit.
        let (l, r) = c.span(0, 800.0);
        assert!(r - l >= 230.0);
        assert!(c.widths[2] <= Columns::MAX);
        for i in 0..3 {
            assert!(c.span(i, 800.0).1 < c.span(i + 1, 800.0).0);
        }
        // Narrower rows do not shrink them, another lane does.
        c.fit(0, 10.0);
        assert!(c.widths[0] > 230.0);
        c.show((1, 1, Format::Hex, false), [140.0, 90.0, 110.0]);
        assert_eq!(c.widths, [140.0, 90.0, 110.0]);
    }

    #[test]
    fn search_finds_values_in_any_format_and_case() {
        let a = ann(Kind::Data, 0xA5, 0);
        assert!(matches(&a, "RX", "a5", Format::Hex, &[]));
        assert!(
            matches(&a, "RX", "a5", Format::Dec, &[]),
            "hex form is searched whatever the display format"
        );
        assert!(
            matches(&a, "RX", "0xa5", Format::Hex, &[]),
            "a 0x prefix is accepted"
        );
        assert!(matches(&a, "RX", "165", Format::Dec, &[]));
        assert!(!matches(&a, "RX", "a6", Format::Hex, &[]));
        assert!(matches(
            &ann(Kind::Data, b'K' as u32, 0),
            "RX",
            "k",
            Format::Ascii,
            &[]
        ));
        assert!(
            matches(&a, "RX", "rx", Format::Hex, &[]),
            "lane names match"
        );
    }

    #[test]
    fn search_finds_flags_by_their_words() {
        let nak = ann(Kind::Data, 0x3C, flag::NAK);
        assert!(matches(&nak, "I2C", "not ack", Format::Hex, &[]));
        assert!(!matches(
            &ann(Kind::Data, 0x3C, 0),
            "I2C",
            "not ack",
            Format::Hex,
            &[]
        ));
        let fe = ann(Kind::Data, 0x00, flag::FRAMING | flag::BREAK);
        assert!(matches(&fe, "RX", "framing", Format::Hex, &[]));
        assert!(matches(&fe, "RX", "break", Format::Hex, &[]));
        let addr = ann(Kind::Address, 0x50, flag::READ);
        assert!(matches(&addr, "I2C", "read", Format::Hex, &[]));
        assert!(!matches(&addr, "I2C", "write", Format::Hex, &[]));
        assert!(
            matches(&addr, "I2C", "50 r", Format::Hex, &[]),
            "the label text is searched too"
        );
    }

    #[test]
    fn telegrams_are_searched_by_their_text() {
        let chars: Vec<Annotation> = b"Hello World"
            .iter()
            .enumerate()
            .map(|(i, &b)| Annotation {
                start: i as u64 * 20,
                end: i as u64 * 20 + 10,
                value: b as u32,
                lane: 0,
                kind: Kind::Data,
                bits: 8,
                flags: 0,
            })
            .collect();
        let t = Annotation {
            start: 0,
            end: 210,
            value: 11,
            lane: 1,
            kind: Kind::Telegram,
            bits: 0,
            flags: 0,
        };
        assert!(
            matches(&t, "Telegram", "lo wor", Format::Ascii, &chars),
            "a phrase spanning several characters"
        );
        assert!(matches(&t, "Telegram", "hello", Format::Ascii, &chars));
        assert!(
            matches(&t, "Telegram", "48 65", Format::Ascii, &chars),
            "hex bytes also match"
        );
        assert!(!matches(&t, "Telegram", "goodbye", Format::Ascii, &chars));
        // Other telegrams' characters do not count.
        let far = Annotation {
            start: 1000,
            end: 1100,
            ..t
        };
        assert!(!matches(&far, "Telegram", "hello", Format::Ascii, &chars));
    }

    #[test]
    fn modbus_frames_are_found_by_their_bytes_and_crc_errors() {
        let chars: Vec<Annotation> = [0x65u32, 0x06, 0x01, 0x01]
            .iter()
            .enumerate()
            .map(|(i, &v)| Annotation {
                start: i as u64 * 10,
                end: i as u64 * 10 + 10,
                ..ann(Kind::Data, v, 0)
            })
            .collect();
        let frame = Annotation {
            start: 0,
            end: 40,
            ..ann(Kind::ModbusFrame, 0x6506, bf_decode::flag::CHECKSUM)
        };
        assert!(matches(&frame, "Frame", "65 06 01", Format::Hex, &chars));
        assert!(matches(
            &frame,
            "Frame",
            "write single",
            Format::Hex,
            &chars
        ));
        assert!(matches(&frame, "Frame", "crc", Format::Hex, &chars));
        assert!(!matches(&frame, "Frame", "65 03", Format::Hex, &chars));
        assert_eq!(info_with_bytes(&frame, &chars), "65 06 01 01  (CRC error)");
    }

    /// A search of `query` over `lane` of a decoder with settings `cfg`.
    fn search(cfg: &Config, lane_no: usize, query: &str, errors_only: bool) -> Search {
        let scope = pattern_scope(cfg, lane_no);
        Search {
            key: SearchKey {
                id: 1,
                lane: lane_no,
                query: query.trim().to_lowercase(),
                raw: query.trim().to_string(),
                fmt: Format::Hex,
                errors_only,
                first: 0,
            },
            scanned: 0,
            hits: Vec::new(),
            pattern: query_pattern(cfg, query).and_then(bf_decode::trigger::PatternSearch::new),
            scope,
        }
    }

    fn text_lane(text: &[u8]) -> Vec<Annotation> {
        text.iter()
            .enumerate()
            .map(|(i, &v)| Annotation {
                start: i as u64 * 10,
                end: i as u64 * 10 + 10,
                ..ann(Kind::Data, v as u32, 0)
            })
            .collect()
    }

    fn hits(
        cfg: &Config,
        lane: &[Annotation],
        chars: &[Annotation],
        lane_no: usize,
        q: &str,
    ) -> Vec<u32> {
        let mut s = search(cfg, lane_no, q, false);
        search_slice(&mut s, lane, lane.len(), "RX", chars);
        s.hits
    }

    #[test]
    fn search_finds_trigger_patterns_on_characters() {
        let uart = Config::default_for(DecoderKind::Uart);
        let chars = text_lane(b"hello world");
        // l, any, space: "lo " at 3..=5; every row of the match is listed.
        assert_eq!(hits(&uart, &chars, &chars, 0, "6C XX 20"), vec![3, 4, 5]);
        // A gap: h ... o matches "hello" (0..=4, the shortest match ending at the first o).
        assert_eq!(
            hits(&uart, &chars, &chars, 0, "68 * 6F"),
            vec![0, 1, 2, 3, 4]
        );
        // Multi-byte in any of the old notations.
        assert_eq!(hits(&uart, &chars, &chars, 0, "6c6c"), vec![2, 3]);
        assert_eq!(hits(&uart, &chars, &chars, 0, "0x6C,0x6F"), vec![3, 4]);
        // Text with escapes is a text pattern (case sensitive). Spaces at the ends of the query are
        // dropped like in any search field: write \x20 there.
        assert_eq!(hits(&uart, &chars, &chars, 0, "l\\xXX\\x20"), vec![3, 4, 5]);
        assert_eq!(
            hits(&uart, &chars, &chars, 0, "L\\xXX\\x20"),
            Vec::<u32>::new()
        );
        // A match spread over two slices is found the same way.
        let mut s = search(&uart, 0, "6C XX 20", false);
        search_slice(&mut s, &chars, 4, "RX", &chars);
        search_slice(&mut s, &chars, chars.len(), "RX", &chars);
        assert_eq!(s.hits, vec![3, 4, 5]);
        // Not a pattern: the plain text search as before.
        assert_eq!(hits(&uart, &chars, &chars, 0, "nothing"), Vec::<u32>::new());
    }

    #[test]
    fn pattern_hits_and_text_hits_merge_in_order() {
        let mut v = vec![1, 5];
        add_hit(&mut v, 3);
        add_hit(&mut v, 5);
        add_hit(&mut v, 9);
        assert_eq!(v, vec![1, 3, 5, 9]);
        // Errors only: rows of a match without an error are left out.
        let uart = Config::default_for(DecoderKind::Uart);
        let mut chars = text_lane(b"abc");
        chars[1].flags = bf_decode::flag::FRAMING;
        let mut s = search(&uart, 0, "61 62 63", true);
        search_slice(&mut s, &chars, 3, "RX", &chars);
        assert_eq!(s.hits, vec![1]);
    }

    #[test]
    fn search_matches_modbus_frames_from_their_first_byte() {
        let modbus = Config::Uart(UartConfig {
            protocol: Protocol::ModbusRtu,
            ..Default::default()
        });
        let chars = text_lane(&[0x65, 0x06, 0x01, 0x01, 0x11, 0x03, 0x65, 0x06]);
        let frame = |start: u64, end: u64, value: u32| Annotation {
            start,
            end,
            lane: 1,
            ..ann(Kind::ModbusFrame, value, 0)
        };
        let frames = vec![frame(0, 40, 0x6506), frame(40, 80, 0x1103)];
        assert_eq!(hits(&modbus, &frames, &chars, 1, "65 06"), vec![0]);
        // 65 06 is also inside the second frame, but not at its start.
        assert_eq!(hits(&modbus, &frames, &chars, 1, "XX 03 * 06"), vec![1]);
        assert_eq!(
            hits(&modbus, &frames, &chars, 1, "06 01"),
            Vec::<u32>::new()
        );
    }

    #[test]
    fn search_finds_patterns_in_telegrams_and_i2c_events() {
        let uart = Config::default_for(DecoderKind::Uart);
        let chars = text_lane(b"AT+X\r\nOK\r\n");
        let tele = |start: u64, end: u64, n: u32| Annotation {
            start,
            end,
            lane: 1,
            ..ann(Kind::Telegram, n, 0)
        };
        let telegrams = vec![tele(0, 60, 6), tele(60, 100, 4)];
        // "4F 4B" (OK) is in the second telegram only; "XX 0D 0A" in both.
        assert_eq!(hits(&uart, &telegrams, &chars, 1, "4F 4B"), vec![1]);
        assert_eq!(hits(&uart, &telegrams, &chars, 1, "XX 0D 0A"), vec![0, 1]);
        // I2C: S @50W 01 3C P as annotations on one lane.
        let i2c = Config::default_for(DecoderKind::I2c);
        let ev = |i: u64, kind, value, flags| Annotation {
            start: i * 10,
            end: i * 10 + 10,
            value,
            lane: 0,
            kind,
            bits: 8,
            flags,
        };
        let lane = vec![
            ev(0, Kind::Start, 0, 0),
            ev(1, Kind::Address, 0x50, 0),
            ev(2, Kind::Data, 0x01, 0),
            ev(3, Kind::Data, 0x3C, bf_decode::flag::NAK),
            ev(4, Kind::Stop, 0, 0),
        ];
        assert_eq!(hits(&i2c, &lane, &lane, 0, "@50W 01 XX!"), vec![1, 2, 3]);
        assert_eq!(hits(&i2c, &lane, &lane, 0, "S * P"), vec![0, 1, 2, 3, 4]);
        assert_eq!(hits(&i2c, &lane, &lane, 0, "@50R"), Vec::<u32>::new());
    }

    #[test]
    fn empty_query_matches_everything() {
        assert!(matches(&ann(Kind::Stop, 0, 0), "I2C", "", Format::Hex, &[]));
    }

    #[test]
    fn prev_next_move_through_sorted_rows() {
        let starts = [10u64, 20, 30, 40];
        let at = |r: usize| starts[r];
        assert_eq!(neighbour_row(4, at, None, 1), 0);
        assert_eq!(neighbour_row(4, at, None, -1), 3);
        assert_eq!(neighbour_row(4, at, Some(20), 1), 2);
        assert_eq!(neighbour_row(4, at, Some(20), -1), 0);
        // A selection that is not a row of this (filtered) list: next/previous around it.
        assert_eq!(neighbour_row(4, at, Some(25), 1), 2);
        assert_eq!(neighbour_row(4, at, Some(25), -1), 1);
        // Ends stay put.
        assert_eq!(neighbour_row(4, at, Some(40), 1), 3);
        assert_eq!(neighbour_row(4, at, Some(10), -1), 0);
        assert_eq!(neighbour_row(4, at, Some(99), 1), 3);
        assert_eq!(neighbour_row(1, |_| 5, Some(5), 1), 0);
    }

    #[test]
    fn csv_has_one_line_per_item_and_quotes_text() {
        use bf_decode::{Annotation, Format, Kind};
        let ann = |start, end, value, kind| Annotation {
            start,
            end,
            value,
            lane: 0,
            kind,
            bits: 8,
            flags: 0,
        };
        let lanes = vec![vec![
            ann(10, 20, b',' as u32, Kind::Data),
            ann(30, 40, b'"' as u32, Kind::Data),
        ]];
        let mut out = Vec::new();
        write_csv(&mut out, &lanes, 0, 1000.0, Format::Ascii).unwrap();
        let text = String::from_utf8(out).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines.len(), 3);
        assert!(lines[0].starts_with("start_s,duration_s"));
        assert!(
            lines[1].starts_with("0.010000000000,0.010000000000,10,20,Data,\",\","),
            "{}",
            lines[1]
        );
        assert!(
            lines[2].contains(",\"\"\"\"") || lines[2].contains("\"\"\"\""),
            "{}",
            lines[2]
        );
        // A lane that does not exist is just the header.
        let mut out = Vec::new();
        write_csv(&mut out, &lanes, 5, 1000.0, Format::Hex).unwrap();
        assert_eq!(String::from_utf8(out).unwrap().lines().count(), 1);
    }

    #[test]
    fn the_gap_unit_converts_and_old_setups_are_in_bit_times() {
        assert_eq!(convert_gap(24.0, GapUnit::Bits, GapUnit::Chars, 10.0), 2.4);
        assert_eq!(convert_gap(3.5, GapUnit::Chars, GapUnit::Bits, 11.0), 38.5);
        assert_eq!(convert_gap(38.5, GapUnit::Bits, GapUnit::Chars, 11.0), 3.5);
        assert_eq!(convert_gap(7.0, GapUnit::Bits, GapUnit::Bits, 10.0), 7.0);
        let old: UartConfig = serde_json::from_str(r#"{"telegram_gap": 38.0}"#).unwrap();
        assert_eq!(old.telegram_gap_unit, GapUnit::Bits);
        assert_eq!(old.telegram_gap_bits(), 38.0);
    }

    #[test]
    fn short_summaries_tell_decoders_apart() {
        let u = Config::Uart(UartConfig {
            baud: 19_200.0,
            parity: Parity::Even,
            ..Default::default()
        });
        assert_eq!(short_summary(&u), "CH0, 19200 8E1");
        let i = Config::I2c(bf_decode::i2c::I2cConfig { scl: 4, sda: 5 });
        assert_eq!(short_summary(&i), "SCL CH4, SDA CH5");
        assert!(short_summary(&Config::default_for(DecoderKind::Spi)).starts_with("SCK CH"));
    }

    #[test]
    fn line_feeds_end_telegrams_only_for_ascii() {
        assert!(!UartConfig::default().telegram_lf);
        let lf = || {
            Config::Uart(UartConfig {
                telegram_lf: true,
                ..Default::default()
            })
        };
        let mut c = lf();
        assert!(!lf_only_for_ascii(&mut c, Format::Ascii));
        assert!(c == lf(), "ASCII keeps the option");
        for fmt in [Format::Hex, Format::Dec, Format::Bin] {
            let mut c = lf();
            assert!(lf_only_for_ascii(&mut c, fmt));
            assert!(matches!(&c, Config::Uart(u) if !u.telegram_lf));
            assert!(
                !lf_only_for_ascii(&mut c, fmt),
                "only reports a change once"
            );
        }
    }

    #[test]
    fn settings_from_files_are_checked() {
        let mut c = Config::default_for(DecoderKind::Uart);
        assert!(c.check_file_values(16).is_ok());
        assert!(
            c.check_file_values(0).is_err(),
            "channel 0 beyond zero channels"
        );
        if let Config::Uart(u) = &mut c {
            u.baud = 0.0;
        }
        assert!(c.check_file_values(16).is_err());
        if let Config::Uart(u) = &mut c {
            u.baud = f64::NAN;
        }
        assert!(c.check_file_values(16).is_err());
        let mut s = Config::default_for(DecoderKind::Spi);
        assert!(s.check_file_values(4).is_ok());
        assert!(s.check_file_values(3).is_err(), "MOSI on channel 3");
        if let Config::Spi(c) = &mut s {
            c.bits = 40;
        }
        assert!(s.check_file_values(16).is_err());
    }
}
