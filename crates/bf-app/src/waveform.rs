//! Drawing of signal rows, bus rows, the time ruler, grid and marker lines.

use crate::markers::Markers;
use crate::rows::{bus_value, format_bus, Radix};
use crate::theme;
use crate::view::{format_time, minor_ticks, ticks, View};
use egui::{pos2, Align2, Color32, FontId, Painter, Rect, Shape, Stroke};
use bf_core::{CaptureStore, Column};
use bf_decode::{flag, telegram_chars, telegram_text, Annotation, Format, Kind};

/// Space above and below the trace inside a row: grows with the row height.
fn trace_pad(row: Rect) -> f32 {
    (row.height() * 0.17).clamp(4.0, 14.0)
}

/// Visible sample range `[r0, r1)` clamped to the data, or `None` when nothing is in view.
fn visible_range(store: &CaptureStore, view: &View, width: f64) -> Option<(u64, u64)> {
    if store.is_empty() {
        return None;
    }
    let (first, end) = (store.first_sample() as f64, store.end() as f64);
    let (vis0, vis1) = (view.start.max(first), view.end(width).min(end));
    if vis0 >= vis1 {
        return None;
    }
    Some((vis0.floor() as u64, (vis1.ceil() as u64).min(store.end())))
}

/// Draw one digital signal into `rect` (its row; `rect.left()` is `view.start`).
///
/// Zoomed in far enough that every edge gets its own pixel, the exact edges are drawn. Otherwise the
/// per-pixel summary is used: flat line, single edge, or a filled bar where the signal toggles
/// several times inside one pixel.
pub fn draw_signal(painter: &Painter, store: &CaptureStore, ch: u8, rect: Rect, view: &View, color: Color32) {
    let n = rect.width().floor() as usize;
    let width = rect.width() as f64;
    let Some((r0, r1)) = visible_range(store, view, width) else { return };
    if n == 0 {
        return;
    }
    let pad = trace_pad(rect);
    let (top, bottom) = (rect.top() + pad, rect.bottom() - pad);
    let y = |level: bool| if level { top } else { bottom };
    let x_of = |s: f64| rect.left() + view.x_of(s) as f32;
    let stroke = Stroke::new(1.0_f32, color);

    if store.count_toggles(ch, r0..r1) <= 3 * n as u64 {
        let Some(edges) = store.edges(ch, r0..r1, usize::MAX) else { return };
        let before = if r0 > store.first_sample() { store.level(ch, r0 - 1) } else { store.level(ch, r0) };
        let mut level = before.unwrap_or(false);
        let mut pts = Vec::with_capacity(edges.len() * 2 + 2);
        pts.push(pos2(x_of(r0 as f64).max(rect.left() - 1.0), y(level)));
        for (p, new_level) in edges {
            let x = x_of(p as f64);
            pts.push(pos2(x, y(level)));
            pts.push(pos2(x, y(new_level)));
            level = new_level;
        }
        pts.push(pos2(x_of(r1 as f64).min(rect.right() + 1.0), y(level)));
        painter.add(Shape::line(pts, stroke));
        return;
    }

    let mut cols: Vec<Column> = Vec::with_capacity(n);
    store.columns(ch, view.start.floor() as i64, view.spp, n, &mut cols);
    let mut pts: Vec<egui::Pos2> = Vec::new();
    let flush = |pts: &mut Vec<egui::Pos2>| {
        if pts.len() >= 2 {
            painter.add(Shape::line(std::mem::take(pts), stroke));
        } else {
            pts.clear();
        }
    };
    for (i, c) in cols.iter().enumerate() {
        let x = rect.left() + i as f32;
        if !c.valid {
            flush(&mut pts);
            continue;
        }
        match c.edges {
            0 => {
                if pts.is_empty() {
                    pts.push(pos2(x, y(c.before)));
                }
                pts.push(pos2(x + 1.0, y(c.after)));
            }
            1 => {
                if pts.is_empty() {
                    pts.push(pos2(x, y(c.before)));
                }
                pts.push(pos2(x + 0.5, y(c.before)));
                pts.push(pos2(x + 0.5, y(c.after)));
                pts.push(pos2(x + 1.0, y(c.after)));
            }
            _ => {
                flush(&mut pts);
                painter.rect_filled(Rect::from_min_max(pos2(x, top), pos2(x + 1.0, bottom)), 0.0, color.gamma_multiply(0.45));
                pts.push(pos2(x + 1.0, y(c.after)));
            }
        }
    }
    flush(&mut pts);
}

/// Draw a bus as value "hexagons" between its transitions. Too many transitions for the pixels
/// available are shown as a shaded band.
pub fn draw_bus(painter: &Painter, store: &CaptureStore, members: &[u8], radix: Radix, rect: Rect, view: &View, color: Color32) {
    let width = rect.width() as f64;
    let Some((r0, r1)) = visible_range(store, view, width) else { return };
    let pad = trace_pad(rect);
    let (top, bottom) = (rect.top() + pad, rect.bottom() - pad);
    let mid = (top + bottom) / 2.0;
    let x_of = |s: f64| rect.left() + view.x_of(s) as f32;
    let stroke = Stroke::new(1.0_f32, color);

    let raw: u64 = members.iter().map(|&c| store.count_toggles(c, r0..r1)).sum();
    if raw > width as u64 {
        // Dense: filled band with the bus outline.
        let x0 = x_of(r0 as f64).max(rect.left());
        let x1 = x_of(r1 as f64).min(rect.right());
        painter.rect_filled(Rect::from_min_max(pos2(x0, top), pos2(x1, bottom)), 0.0, color.gamma_multiply(0.3));
        painter.line_segment([pos2(x0, top), pos2(x1, top)], stroke);
        painter.line_segment([pos2(x0, bottom), pos2(x1, bottom)], stroke);
        return;
    }

    let mut starts: Vec<u64> = vec![r0];
    for &c in members {
        if let Some(e) = store.edges(c, r0..r1, usize::MAX) {
            starts.extend(e.into_iter().map(|(p, _)| p));
        }
    }
    starts.sort_unstable();
    starts.dedup();

    let font = FontId::monospace(11.0);
    for (k, &s) in starts.iter().enumerate() {
        let e = starts.get(k + 1).copied().unwrap_or(r1);
        let (x0, x1) = (x_of(s as f64), x_of(e as f64));
        if x1 < rect.left() - 4.0 || x0 > rect.right() + 4.0 {
            continue;
        }
        let slant = ((x1 - x0) / 2.0).min(3.0);
        painter.add(Shape::closed_line(
            vec![
                pos2(x0, mid),
                pos2(x0 + slant, top),
                pos2(x1 - slant, top),
                pos2(x1, mid),
                pos2(x1 - slant, bottom),
                pos2(x0 + slant, bottom),
            ],
            stroke,
        ));
        // Centre the value on the part of the segment that is on screen.
        let (vx0, vx1) = (x0.max(rect.left()), x1.min(rect.right()));
        if vx1 - vx0 > 18.0 {
            let value = bus_value(store.state_at(s).unwrap_or(0), members);
            let galley = painter.layout_no_wrap(format_bus(value, members.len(), radix), font.clone(), color);
            let size = galley.size();
            if size.x + 8.0 <= vx1 - vx0 {
                painter.galley(pos2((vx0 + vx1) / 2.0 - size.x / 2.0, mid - size.y / 2.0), galley, color);
            }
        }
    }
}

/// Draw the time ruler into `rect`: labelled major ticks and short minor ticks.
pub fn draw_ruler(painter: &Painter, rect: Rect, view: &View, rate: f64, origin: f64) {
    let width = rect.width() as f64;
    let (step, major) = ticks(view, width, rate, origin);
    painter.line_segment([pos2(rect.left(), rect.bottom()), pos2(rect.right(), rect.bottom())], Stroke::new(1.0_f32, theme::GRID_MINOR));
    for s in minor_ticks(view, width, rate, origin) {
        let x = rect.left() + view.x_of(s as f64) as f32;
        painter.line_segment([pos2(x, rect.bottom() - 3.0), pos2(x, rect.bottom())], Stroke::new(1.0_f32, theme::GRID_MINOR));
    }
    let font = FontId::monospace(11.0);
    for (s, t) in major {
        let x = rect.left() + view.x_of(s as f64) as f32;
        if x < rect.left() || x > rect.right() {
            continue;
        }
        painter.line_segment([pos2(x, rect.bottom() - 7.0), pos2(x, rect.bottom())], Stroke::new(1.0_f32, theme::TEXT));
        painter.text(
            pos2(x + 3.0, rect.top() + 2.0),
            Align2::LEFT_TOP,
            format_time(t, step),
            font.clone(),
            theme::TEXT,
        );
    }
}

/// Dotted vertical grid lines at the major ticks.
pub fn draw_grid(painter: &Painter, rect: Rect, view: &View, rate: f64, origin: f64) {
    let (_, major) = ticks(view, rect.width() as f64, rate, origin);
    for (s, _) in major {
        let x = rect.left() + view.x_of(s as f64) as f32;
        if x >= rect.left() && x <= rect.right() {
            painter.extend(Shape::dashed_line(&[pos2(x, rect.top()), pos2(x, rect.bottom())], Stroke::new(1.0_f32, theme::GRID), 1.0, 3.0));
        }
    }
}

/// Marker lines through the plot with a labelled tab on the ruler.
pub fn draw_markers(painter: &Painter, plot: Rect, ruler: Rect, view: &View, m: &Markers) {
    for (pos, color, name) in [(m.m1, theme::M1, "M1"), (m.m2, theme::M2, "M2")] {
        let Some(p) = pos else { continue };
        let x = plot.left() + view.x_of(p) as f32;
        if x < plot.left() - 20.0 || x > plot.right() + 20.0 {
            continue;
        }
        painter.extend(Shape::dashed_line(&[pos2(x, ruler.bottom()), pos2(x, plot.bottom())], Stroke::new(1.0_f32, color), 4.0, 3.0));
        let tab = Rect::from_min_size(pos2(x - 11.0, ruler.bottom() - 15.0), egui::vec2(22.0, 13.0));
        painter.rect_filled(tab, 0.0, color);
        painter.text(tab.center(), Align2::CENTER_CENTER, name, FontId::monospace(10.0), Color32::BLACK);
    }
}

/// The trigger point: a solid line through the plot and a "T" tab on the ruler.
pub fn draw_trigger(painter: &Painter, plot: Rect, ruler: Rect, view: &View, sample: u64) {
    let x = plot.left() + view.x_of(sample as f64) as f32;
    if x < plot.left() - 20.0 || x > plot.right() + 20.0 {
        return;
    }
    painter.line_segment([pos2(x, ruler.bottom()), pos2(x, plot.bottom())], Stroke::new(1.5_f32, theme::DEC_WARN));
    let tab = Rect::from_min_size(pos2(x - 7.0, ruler.bottom() - 15.0), egui::vec2(14.0, 13.0));
    painter.rect_filled(tab, 0.0, theme::DEC_WARN);
    painter.text(tab.center(), Align2::CENTER_CENTER, "T", FontId::monospace(10.0), Color32::BLACK);
}

/// Colour of an annotation: errors red, NAK orange, addresses cyan, START/STOP green, data as `base`.
fn annotation_color(a: &Annotation, base: Color32) -> Color32 {
    if a.is_error() {
        theme::DEC_ERR
    } else if a.flags & flag::NAK != 0 {
        theme::DEC_WARN
    } else {
        match a.kind {
            Kind::Address => theme::DEC_ADDR,
            Kind::Start | Kind::Stop => theme::DEC_MARK,
            _ => base,
        }
    }
}

/// Draw the annotations of one decoder lane (`lane` is sorted and non-overlapping). Zoomed out far
/// enough that annotations are narrower than a pixel, one summary bar per pixel column is drawn
/// instead, found by binary search, so the cost is bounded by the window width.
#[allow(clippy::too_many_arguments)]
pub fn draw_annotations(
    painter: &Painter,
    lane: &[Annotation],
    rect: Rect,
    view: &View,
    fmt: Format,
    base: Color32,
    selected: Option<u64>,
    companion: Option<&[Annotation]>,
) {
    let width = rect.width() as f64;
    let (a, b) = (view.start.max(0.0) as u64, view.end(width).ceil().max(0.0) as u64);
    let lo = lane.partition_point(|x| x.end <= a);
    let hi = lane.partition_point(|x| x.start < b).max(lo);
    let vis = &lane[lo..hi];
    if vis.is_empty() {
        return;
    }
    let pad = trace_pad(rect);
    let (top, bottom) = (rect.top() + pad, rect.bottom() - pad);
    let mid = (top + bottom) / 2.0;
    let x_of = |s: f64| rect.left() + view.x_of(s) as f32;

    if vis.len() as f64 > width * 3.0 {
        // One bar per pixel column that contains at least one annotation.
        for c in 0..rect.width() as usize {
            let s0 = view.sample_at(c as f64).max(0.0) as u64;
            let s1 = view.sample_at(c as f64 + 1.0).max(0.0) as u64;
            let i = vis.partition_point(|x| x.end <= s0);
            if let Some(x) = vis.get(i).filter(|x| x.start < s1.max(s0 + 1)) {
                let col = annotation_color(x, base).gamma_multiply(0.6);
                let px = rect.left() + c as f32;
                painter.rect_filled(Rect::from_min_max(pos2(px, top), pos2(px + 1.0, bottom)), 0.0, col);
            }
        }
        return;
    }

    let font = FontId::monospace(11.0);
    for ann in vis {
        let (x0, x1) = (x_of(ann.start as f64), x_of(ann.end as f64));
        let color = annotation_color(ann, base);
        let is_sel = selected == Some(ann.start);
        let stroke = Stroke::new(if is_sel { 2.0 } else { 1.0_f32 }, if is_sel { Color32::WHITE } else { color });
        match ann.kind {
            Kind::Start | Kind::Stop => {
                let w = (x1 - x0).max(3.0);
                painter.rect_filled(Rect::from_min_max(pos2(x0, top), pos2(x0 + w, bottom)), 0.0, color.gamma_multiply(0.85));
                if w > 11.0 {
                    painter.text(pos2(x0 + w / 2.0, mid), Align2::CENTER_CENTER, ann.label(fmt), font.clone(), Color32::BLACK);
                } else if x0 + w + 12.0 < rect.right() {
                    painter.text(pos2(x0 + w + 2.0, mid), Align2::LEFT_CENTER, ann.label(fmt), font.clone(), color);
                }
            }
            Kind::Telegram => {
                let (xa, xb) = (x0, x1.max(x0 + 1.0));
                if xb - xa < 2.0 {
                    painter.rect_filled(Rect::from_min_max(pos2(xa, top), pos2(xa + 1.0, bottom)), 0.0, color.gamma_multiply(0.6));
                    continue;
                }
                let bar = Rect::from_min_max(pos2(xa, top), pos2(xb, bottom));
                painter.rect_filled(bar, 0.0, color.gamma_multiply(0.14));
                painter.rect_stroke(bar, 0.0, stroke);
                // The text stays at the left edge of the visible part, so a long telegram can be read
                // while scrolling through it.
                let (vx0, vx1) = (xa.max(rect.left()), xb.min(rect.right()));
                if let Some(chars) = companion.filter(|_| vx1 - vx0 > 14.0) {
                    let max_chars = ((vx1 - vx0) / 6.0) as usize + 4;
                    let text = telegram_text(telegram_chars(chars, ann), fmt, max_chars);
                    let clip = painter.with_clip_rect(Rect::from_min_max(pos2(vx0 + 1.0, top), pos2(vx1 - 1.0, bottom)));
                    clip.text(pos2(vx0 + 5.0, mid), Align2::LEFT_CENTER, text, font.clone(), color);
                }
            }
            _ => {
                let (xa, xb) = (x0, x1.max(x0 + 1.0));
                if xb - xa < 2.0 {
                    painter.rect_filled(Rect::from_min_max(pos2(xa, top), pos2(xa + 1.0, bottom)), 0.0, color.gamma_multiply(0.6));
                    continue;
                }
                let (top, bottom) = if ann.kind == Kind::Frame { (mid - 2.0, mid + 2.0) } else { (top, bottom) };
                let slant = ((xb - xa) / 2.0).min(3.0);
                let mid_y = (top + bottom) / 2.0;
                painter.add(Shape::closed_line(
                    vec![
                        pos2(xa, mid_y),
                        pos2(xa + slant, top),
                        pos2(xb - slant, top),
                        pos2(xb, mid_y),
                        pos2(xb - slant, bottom),
                        pos2(xa + slant, bottom),
                    ],
                    stroke,
                ));
                let (vx0, vx1) = (xa.max(rect.left()), xb.min(rect.right()));
                if vx1 - vx0 > 18.0 && ann.kind != Kind::Frame {
                    let galley = painter.layout_no_wrap(ann.label(fmt), font.clone(), color);
                    let size = galley.size();
                    if size.x + 8.0 <= vx1 - vx0 {
                        painter.galley(pos2((vx0 + vx1) / 2.0 - size.x / 2.0, mid_y - size.y / 2.0), galley, color);
                    }
                }
            }
        }
    }
}
