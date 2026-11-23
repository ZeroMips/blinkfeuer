//! Instrument look: flat, square, dense, monospace; a black plot on neutral dark chrome.

use egui::{vec2, Color32, FontFamily, FontId, Rounding, Shadow, Stroke, TextStyle};

pub const CHROME: Color32 = Color32::from_rgb(0x23, 0x25, 0x28);
pub const CHROME_DARK: Color32 = Color32::from_rgb(0x1a, 0x1c, 0x1f);
pub const BORDER: Color32 = Color32::from_rgb(0x3d, 0x41, 0x46);
pub const TEXT: Color32 = Color32::from_rgb(0xd6, 0xd9, 0xdc);
pub const TEXT_DIM: Color32 = Color32::from_rgb(0x8b, 0x91, 0x98);

pub const PLOT_BG: Color32 = Color32::BLACK;
pub const GRID: Color32 = Color32::from_rgb(0x4a, 0x4f, 0x55);
pub const GRID_MINOR: Color32 = Color32::from_rgb(0x6a, 0x70, 0x77);
pub const TRACE: Color32 = Color32::from_rgb(0xe2, 0xe6, 0xe9);
pub const ROW_SEP: Color32 = Color32::from_rgb(0x1c, 0x1f, 0x22);
pub const GROUP_BG: Color32 = Color32::from_rgb(0x2b, 0x2f, 0x34);
pub const SELECT: Color32 = Color32::from_rgb(0x1d, 0x3f, 0x66);

pub const DEC_ADDR: Color32 = Color32::from_rgb(0x3f, 0xd5, 0xff);
pub const DEC_WARN: Color32 = Color32::from_rgb(0xff, 0x9a, 0x3c);
pub const DEC_ERR: Color32 = Color32::from_rgb(0xff, 0x5f, 0x56);
pub const DEC_MARK: Color32 = Color32::from_rgb(0xb5, 0xe8, 0x53);

pub const M1: Color32 = Color32::from_rgb(0x2f, 0xe0, 0x4a);
pub const M2: Color32 = Color32::from_rgb(0xff, 0x5c, 0xf0);
pub const SNAP: Color32 = Color32::from_rgb(0xff, 0xe0, 0x30);
pub const RUN: Color32 = Color32::from_rgb(0x3f, 0xc0, 0x55);
pub const STOP: Color32 = Color32::from_rgb(0xd9, 0x4f, 0x45);

pub fn apply(ctx: &egui::Context) {
    let mut v = egui::Visuals::dark();
    v.panel_fill = CHROME;
    v.window_fill = CHROME;
    v.window_rounding = Rounding::ZERO;
    v.menu_rounding = Rounding::ZERO;
    v.window_shadow = Shadow::NONE;
    v.popup_shadow = Shadow::NONE;
    v.window_stroke = Stroke::new(1.0_f32, BORDER);
    v.extreme_bg_color = CHROME_DARK;
    v.faint_bg_color = CHROME_DARK;
    v.override_text_color = Some(TEXT);
    v.selection.bg_fill = SELECT;
    v.selection.stroke = Stroke::new(1.0_f32, Color32::WHITE);
    let w = &mut v.widgets;
    for (ws, fill) in [
        (&mut w.noninteractive, CHROME),
        (&mut w.inactive, Color32::from_rgb(0x30, 0x33, 0x37)),
        (&mut w.hovered, Color32::from_rgb(0x3b, 0x3f, 0x44)),
        (&mut w.active, Color32::from_rgb(0x2a, 0x4a, 0x70)),
        (&mut w.open, Color32::from_rgb(0x2a, 0x4a, 0x70)),
    ] {
        ws.rounding = Rounding::ZERO;
        ws.bg_fill = fill;
        ws.weak_bg_fill = fill;
        ws.bg_stroke = Stroke::new(1.0_f32, BORDER);
        ws.expansion = 0.0;
    }
    w.noninteractive.bg_stroke = Stroke::new(1.0_f32, Color32::from_rgb(0x32, 0x36, 0x3a));
    ctx.set_visuals(v);

    ctx.style_mut(|s| {
        s.spacing.item_spacing = vec2(6.0, 3.0);
        s.spacing.button_padding = vec2(7.0, 2.0);
        s.spacing.interact_size = vec2(24.0, 20.0);
        s.spacing.menu_margin = egui::Margin::symmetric(4.0, 3.0);
        s.spacing.window_margin = egui::Margin::same(6.0);
        s.spacing.combo_width = 60.0;
        let mono = |size| FontId::new(size, FontFamily::Monospace);
        s.text_styles.insert(TextStyle::Body, mono(12.0));
        s.text_styles.insert(TextStyle::Button, mono(12.0));
        s.text_styles.insert(TextStyle::Monospace, mono(12.0));
        s.text_styles.insert(TextStyle::Small, mono(10.0));
        s.text_styles.insert(TextStyle::Heading, mono(13.0));
    });
}
