//! The contents of the Trigger window: pattern conditions per channel, or a protocol to look for.

use crate::decode_ui::{self, BaudProbe, Config};
use crate::setup::{
    Direction, I2cWhat, ProtocolSetup, Span, SpanUnit, TriggerMode, TriggerSetup, UartWhat,
};
use crate::theme;
use crate::view::format_split;
use bf_core::Cond;
use bf_decode::trigger::{PatItem, ProtocolTrigger, TriggerAt};
use bf_decode::Format;
use egui::RichText;

/// A decoder the protocol trigger can take its settings from.
pub struct DecoderChoice {
    pub id: u32,
    pub label: String,
    pub cfg: Config,
}

fn dim(ui: &mut egui::Ui, text: impl Into<String>) {
    ui.label(RichText::new(text.into()).color(theme::TEXT_DIM));
}

/// A line of hex digits or text; red when `error` says it is not usable.
fn entry(ui: &mut egui::Ui, text: &mut String, width: f32, error: Option<&str>) -> bool {
    let r = ui.add(
        egui::TextEdit::singleline(text)
            .desired_width(width)
            .font(egui::TextStyle::Monospace),
    );
    if let Some(e) = error {
        ui.colored_label(theme::STOP, e);
    }
    r.changed()
}

/// A number and its unit.
fn span_ui(ui: &mut egui::Ui, id: &str, span: &mut Span) {
    ui.add(
        egui::DragValue::new(&mut span.value)
            .speed(if span.unit == SpanUnit::Samples {
                100.0
            } else {
                0.1
            })
            .range(0.0..=1e12)
            .max_decimals(6),
    );
    egui::ComboBox::from_id_salt(id)
        .width(46.0)
        .selected_text(span.unit.label())
        .show_ui(ui, |ui| {
            for u in SpanUnit::ALL {
                ui.selectable_value(&mut span.unit, u, u.label());
            }
        });
}

/// The Trigger window. Returns the id of a decoder whose settings the user wants to edit.
///
/// With room for it the window has two columns: what to trigger on on the left; the example, how
/// much to record and the warnings on the right.
#[allow(clippy::too_many_arguments)]
pub fn contents(
    ui: &mut egui::Ui,
    t: &mut TriggerSetup,
    probe: &mut BaudProbe,
    memory_mib: &mut u64,
    names: &[String],
    decoders: &[DecoderChoice],
    n: usize,
    rate: f64,
    continuous: bool,
) -> Option<u32> {
    let mut edit = None;
    // What the trigger will really use: linked decoders' settings put in.
    let configs: Vec<(u32, Config)> = decoders.iter().map(|d| (d.id, d.cfg.clone())).collect();
    if ui.available_width() >= TWO_COLUMNS {
        ui.columns(2, |cols| {
            let (left, right) = cols.split_at_mut(1);
            what_to_look_for(&mut left[0], t, probe, names, decoders, n, &mut edit);
            let protocol = t.protocol.resolved(&configs);
            let right = &mut right[0];
            if example(right, &protocol, t.mode, names) {
                right.separator();
            }
            how_much_to_record(right, t, &protocol, memory_mib, n, rate, continuous);
        });
    } else {
        what_to_look_for(ui, t, probe, names, decoders, n, &mut edit);
        let protocol = t.protocol.resolved(&configs);
        example(ui, &protocol, t.mode, names);
        ui.separator();
        how_much_to_record(ui, t, &protocol, memory_mib, n, rate, continuous);
    }
    edit
}

/// Below this width the window has one column.
const TWO_COLUMNS: f32 = 820.0;

/// Enabling, the mode and its settings.
fn what_to_look_for(
    ui: &mut egui::Ui,
    t: &mut TriggerSetup,
    probe: &mut BaudProbe,
    names: &[String],
    decoders: &[DecoderChoice],
    n: usize,
    edit: &mut Option<u32>,
) {
    ui.checkbox(
        &mut t.enabled,
        "Wait for the trigger before the capture ends",
    );
    ui.horizontal(|ui| {
        dim(ui, "Trigger on");
        for m in TriggerMode::ALL {
            ui.selectable_value(&mut t.mode, m, m.label());
        }
    });
    ui.separator();
    match t.mode {
        TriggerMode::Pattern => pattern(ui, t, names, n),
        TriggerMode::Uart => uart(ui, t, probe, names, decoders, edit),
        TriggerMode::Spi => spi(ui, t, names, decoders, edit),
        TriggerMode::I2c => i2c(ui, t, names, decoders, edit),
    }
}

/// Pre- and post-trigger spans, the memory limit, and what keeps the trigger from working.
fn how_much_to_record(
    ui: &mut egui::Ui,
    t: &mut TriggerSetup,
    protocol: &ProtocolSetup,
    memory_mib: &mut u64,
    n: usize,
    rate: f64,
    continuous: bool,
) {
    ui.label(
        RichText::new("How much to record")
            .color(theme::TEXT)
            .strong(),
    );
    let time = |s: u64| format_split(s as f64 / rate, 1.0 / rate);
    egui::Grid::new("trigger_span")
        .num_columns(2)
        .spacing([10.0, 4.0])
        .show(ui, |ui| {
            ui.label("Before the trigger");
            ui.horizontal(|ui| {
                span_ui(ui, "pre_unit", &mut t.pre);
                dim(
                    ui,
                    format!(
                        "= {} Sa, {}",
                        t.pre.samples(rate),
                        time(t.pre.samples(rate))
                    ),
                );
            });
            ui.end_row();
            ui.label("After the trigger");
            ui.horizontal(|ui| {
                ui.add_enabled_ui(!continuous, |ui| span_ui(ui, "post_unit", &mut t.post));
                if continuous {
                    dim(ui, "until Stop (mode Continuous)");
                } else {
                    dim(
                        ui,
                        format!(
                            "= {} Sa, {}",
                            t.post.samples(rate),
                            time(t.post.samples(rate))
                        ),
                    );
                }
            });
            ui.end_row();
            ui.label("Memory limit");
            ui.horizontal(|ui| {
                let mut gib = *memory_mib as f64 / 1024.0;
                if ui
                    .add(
                        egui::DragValue::new(&mut gib)
                            .speed(0.1)
                            .range(0.0625..=2048.0)
                            .suffix(" GiB")
                            .max_decimals(2),
                    )
                    .changed()
                {
                    *memory_mib = (gib * 1024.0).round().max(64.0) as u64;
                }
                dim(ui, "the capture stops when it is full");
            });
            ui.end_row();
        });
    // The store keeps one 8-byte event per sample where anything changes, so quiet signals need far less.
    let total = t.pre.samples(rate) + if continuous { 0 } else { t.post.samples(rate) };
    let worst = total as f64 * 8.0 / (1u64 << 20) as f64;
    dim(ui, format!("Memory: at most {:.0} MiB for this window if every sample changes; idle signals need very little.", worst));
    dim(ui, "Samples are kept from before the trigger on, so the pre-trigger part is never dropped to make room.");
    if t.enabled {
        if t.mode == TriggerMode::Pattern && t.conds.iter().take(n).all(|c| *c == Cond::X) {
            ui.colored_label(
                theme::DEC_WARN,
                "No condition set: the trigger fires as soon as the pre-trigger window is full.",
            );
        }
        if let Err(e) = protocol.build(t.mode) {
            ui.colored_label(theme::STOP, format!("This trigger cannot start: {e}"));
        }
        if t.pre.samples(rate) as f64 * 8.0 > *memory_mib as f64 * 1048576.0 {
            ui.colored_label(
                theme::DEC_WARN,
                "The pre-trigger window may not fit in the memory limit.",
            );
        }
    }
}

/// For a protocol trigger: an example of what it looks for, with the trigger sample marked.
/// True when an example was drawn.
fn example(
    ui: &mut egui::Ui,
    protocol: &ProtocolSetup,
    mode: TriggerMode,
    names: &[String],
) -> bool {
    let Ok(Some(trigger)) = protocol.build(mode) else {
        return false;
    };
    let Some(p) = crate::decode_preview::build_trigger(&trigger, protocol.trigger_at, names) else {
        return false;
    };
    let fmt = match trigger {
        ProtocolTrigger::Uart(..) if protocol.uart_ascii => Format::Ascii,
        _ => Format::Hex,
    };
    ui.add_space(6.0);
    ui.label(RichText::new("Example").color(theme::TEXT).strong());
    crate::decode_preview::draw(ui, &p, fmt);
    ui.add_space(2.0);
    for line in &p.caption {
        ui.label(RichText::new(line).color(theme::TEXT_DIM).size(11.0));
    }
    true
}

/// The decoder settings of a protocol trigger: its own (editable), or those of a decoder of the same
/// kind (shown, edited in the decoder setup). Returns changed own settings.
#[allow(clippy::too_many_arguments)]
fn settings(
    ui: &mut egui::Ui,
    id: &str,
    source: &mut Option<u32>,
    own: Config,
    names: &[String],
    decoders: &[DecoderChoice],
    probe: Option<&mut BaudProbe>,
    edit: &mut Option<u32>,
) -> Option<Config> {
    let choices: Vec<&DecoderChoice> = decoders
        .iter()
        .filter(|d| d.cfg.kind() == own.kind())
        .collect();
    let linked = source.and_then(|s| choices.iter().find(|d| d.id == s).copied());
    if !choices.is_empty() {
        ui.horizontal(|ui| {
            dim(ui, "Settings from");
            egui::ComboBox::from_id_salt(format!("{id}_source"))
                .selected_text(linked.map_or("Own settings", |d| d.label.as_str()))
                .width(240.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(source, None, "Own settings");
                    for d in &choices {
                        ui.selectable_value(source, Some(d.id), d.label.as_str());
                    }
                });
        });
    }
    match linked {
        Some(d) if *source == Some(d.id) => {
            // The decoder's settings, shown but not editable here.
            let mut cfg = d.cfg.clone();
            ui.add_enabled_ui(false, |ui| {
                decode_ui::config_ui_with(ui, id, &mut cfg, names, None, None);
            });
            ui.horizontal(|ui| {
                dim(ui, "The trigger follows this decoder's settings.");
                if ui.button("Edit in decoder setup").clicked() {
                    *edit = Some(d.id);
                }
            });
            None
        }
        _ => {
            let mut cfg = own;
            decode_ui::config_ui_with(ui, id, &mut cfg, names, None, probe).then_some(cfg)
        }
    }
}

fn pattern(ui: &mut egui::Ui, t: &mut TriggerSetup, names: &[String], n: usize) {
    dim(
        ui,
        "Fires on the first sample where every condition holds at once.",
    );
    dim(
        ui,
        "X = don't care, 0 / 1 = level, arrows = rising / falling / any edge.",
    );
    ui.add_space(4.0);
    egui::Grid::new("trigger_conds")
        .num_columns(3)
        .spacing([10.0, 3.0])
        .show(ui, |ui| {
            for ch in 0..n.min(32) {
                ui.label(format!("{ch:>2}"));
                ui.label(names.get(ch).cloned().unwrap_or_default());
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 2.0;
                    for c in Cond::ALL {
                        if ui
                            .selectable_label(t.conds[ch] == c, c.label())
                            .on_hover_text(c.describe())
                            .clicked()
                        {
                            t.conds[ch] = c;
                        }
                    }
                });
                ui.end_row();
            }
        });
    if ui.button("Clear conditions").clicked() {
        t.conds.iter_mut().for_each(|c| *c = Cond::X);
    }
}

/// Which sample of the match is the trigger.
fn trigger_point(ui: &mut egui::Ui, p: &mut ProtocolSetup) {
    ui.add_space(6.0);
    ui.label(RichText::new("Trigger point").color(theme::TEXT).strong());
    ui.horizontal(|ui| {
        ui.selectable_value(&mut p.trigger_at, TriggerAt::Start, "Start of the match")
            .on_hover_text(
                "The first sample of the matched item (of a byte sequence: of its first byte)",
            );
        ui.selectable_value(&mut p.trigger_at, TriggerAt::End, "End of the match")
            .on_hover_text("The last sample of the matched item");
    });
}

fn protocol_note(ui: &mut egui::Ui) {
    dim(
        ui,
        "A decoder runs on the live data; the trigger is the end of the first match.",
    );
    dim(
        ui,
        "Set the same channels and frame format as the signal has.",
    );
}

fn uart(
    ui: &mut egui::Ui,
    t: &mut TriggerSetup,
    probe: &mut BaudProbe,
    names: &[String],
    decoders: &[DecoderChoice],
    edit: &mut Option<u32>,
) {
    protocol_note(ui);
    let p = &mut t.protocol;
    let own = Config::Uart(p.uart.clone());
    if let Some(Config::Uart(c)) = settings(
        ui,
        "trg_uart",
        &mut p.uart_source,
        own,
        names,
        decoders,
        Some(probe),
        edit,
    ) {
        p.uart = c;
    }
    ui.add_space(6.0);
    ui.label(RichText::new("Look for").color(theme::TEXT).strong());
    ui.horizontal(|ui| {
        ui.selectable_value(&mut p.uart_what, UartWhat::Bytes, "Characters");
        ui.selectable_value(
            &mut p.uart_what,
            UartWhat::Error,
            "Framing / parity error or break",
        );
    });
    if p.uart_what == UartWhat::Bytes {
        ui.horizontal(|ui| {
            let before = p.uart_ascii;
            ui.selectable_value(&mut p.uart_ascii, true, "Text");
            ui.selectable_value(&mut p.uart_ascii, false, "Hex");
            if before != p.uart_ascii {
                // Keep what is typed meaningful when the notation changes. Single-bit wildcards
                // only exist in hex: then the notation stays hex.
                let items = if before {
                    crate::setup::parse_text_pattern(&p.uart_bytes)
                } else {
                    crate::setup::parse_hex_pattern(&p.uart_bytes)
                };
                if let Ok(items) = items {
                    if p.uart_ascii {
                        match crate::setup::format_text_pattern(&items) {
                            Some(text) => p.uart_bytes = text,
                            None => p.uart_ascii = false,
                        }
                    } else {
                        p.uart_bytes = crate::setup::format_hex_pattern(&items);
                    }
                }
            }
        });
        let err = p
            .uart_pattern()
            .and_then(|items| {
                if items.iter().any(|i| matches!(i, PatItem::Char { .. })) {
                    Ok(())
                } else {
                    Err("enter at least one character".into())
                }
            })
            .err();
        entry(ui, &mut p.uart_bytes, 260.0, err.as_deref());
        dim(
            ui,
            if p.uart_ascii {
                "Escapes: \\r \\n \\t \\0 \\\\ \\xNN; X is a free nibble (\\xXX: any character), \\* any characters in between."
            } else {
                "Hex bytes, e.g. 4F 4B 0D 0A; X is a free nibble (XX: any byte, 4X), 0b1010XXXX single bits, * any characters in between."
            },
        );
        ui.checkbox(
            &mut p.uart_or_error,
            "or a character with a framing / parity error or break",
        );
    }
    trigger_point(ui, p);
}

fn spi(
    ui: &mut egui::Ui,
    t: &mut TriggerSetup,
    names: &[String],
    decoders: &[DecoderChoice],
    edit: &mut Option<u32>,
) {
    protocol_note(ui);
    let p = &mut t.protocol;
    let own = Config::Spi(p.spi.clone());
    if let Some(Config::Spi(c)) = settings(
        ui,
        "trg_spi",
        &mut p.spi_source,
        own,
        names,
        decoders,
        None,
        edit,
    ) {
        p.spi = c;
    }
    ui.add_space(6.0);
    ui.label(RichText::new("Look for a word").color(theme::TEXT).strong());
    ui.horizontal(|ui| {
        ui.selectable_value(&mut p.spi_miso, false, "on MOSI");
        ui.selectable_value(&mut p.spi_miso, true, "on MISO");
    });
    egui::Grid::new("trg_spi_word")
        .num_columns(2)
        .spacing([10.0, 4.0])
        .show(ui, |ui| {
            ui.label("Value (hex)");
            let err = crate::setup::parse_hex_u32(&p.spi_value).err();
            entry(ui, &mut p.spi_value, 90.0, err.as_deref());
            ui.end_row();
            ui.label("Bits that must match (hex)");
            let err = crate::setup::parse_hex_u32(&p.spi_care).err();
            entry(ui, &mut p.spi_care, 90.0, err.as_deref());
            ui.end_row();
        });
    dim(
        ui,
        "Bits that are 0 in the second field are ignored (FF = all bits, 0F = low nibble only).",
    );
    trigger_point(ui, p);
}

fn i2c(
    ui: &mut egui::Ui,
    t: &mut TriggerSetup,
    names: &[String],
    decoders: &[DecoderChoice],
    edit: &mut Option<u32>,
) {
    protocol_note(ui);
    let p = &mut t.protocol;
    let own = Config::I2c(p.i2c.clone());
    if let Some(Config::I2c(c)) = settings(
        ui,
        "trg_i2c",
        &mut p.i2c_source,
        own,
        names,
        decoders,
        None,
        edit,
    ) {
        p.i2c = c;
    }
    ui.add_space(6.0);
    ui.label(RichText::new("Look for").color(theme::TEXT).strong());
    egui::ComboBox::from_id_salt("trg_i2c_what")
        .selected_text(p.i2c_what.label())
        .show_ui(ui, |ui| {
            for w in I2cWhat::ALL {
                ui.selectable_value(&mut p.i2c_what, w, w.label());
            }
        });
    match p.i2c_what {
        I2cWhat::Address | I2cWhat::Data => {
            ui.horizontal(|ui| {
                ui.label(if p.i2c_what == I2cWhat::Address {
                    "Address (hex, 7 bit)"
                } else {
                    "Byte (hex)"
                });
                let err = crate::setup::parse_hex_u32(&p.i2c_value).err();
                entry(ui, &mut p.i2c_value, 70.0, err.as_deref());
            });
            if p.i2c_what == I2cWhat::Address {
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut p.i2c_dir, Direction::Any, "Read or write");
                    ui.selectable_value(&mut p.i2c_dir, Direction::Read, "Read");
                    ui.selectable_value(&mut p.i2c_dir, Direction::Write, "Write");
                });
            }
        }
        _ => {}
    }
    trigger_point(ui, p);
}
