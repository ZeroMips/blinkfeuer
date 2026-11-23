//! The application icon, drawn in code so it needs no image files: a flashing beacon (the "Blinkfeuer")
//! above a logic trace, on a dark tile in the colours of the application.
//!
//! The drawing is defined on a 256 x 256 canvas and rendered at any size with 4 x 4 supersampling.

/// Straight (not premultiplied) RGBA colour.
#[derive(Clone, Copy)]
struct Rgba([f32; 4]);

const TILE: Rgba = Rgba([
    0x1a as f32 / 255.0,
    0x1c as f32 / 255.0,
    0x1f as f32 / 255.0,
    1.0,
]);
const TILE_EDGE: Rgba = Rgba([
    0x3d as f32 / 255.0,
    0x41 as f32 / 255.0,
    0x46 as f32 / 255.0,
    1.0,
]);
const TRACE: Rgba = Rgba([
    0xe2 as f32 / 255.0,
    0xe6 as f32 / 255.0,
    0xe9 as f32 / 255.0,
    1.0,
]);
const BEACON: Rgba = Rgba([1.0, 0xe0 as f32 / 255.0, 0x30 as f32 / 255.0, 1.0]);
const GLOW: Rgba = Rgba([1.0, 0x9a as f32 / 255.0, 0x3c as f32 / 255.0, 1.0]);
const BASE: Rgba = Rgba([
    0x8b as f32 / 255.0,
    0x91 as f32 / 255.0,
    0x98 as f32 / 255.0,
    1.0,
]);

/// Centre of the lamp's dome (the light sits on a base below it).
const BEACON_AT: (f32, f32) = (128.0, 108.0);
const DOME_R: f32 = 32.0;

/// The logic trace: low, high, low, high, low across the lower part of the tile.
const WAVE: [(f32, f32); 10] = [
    (40.0, 200.0),
    (76.0, 200.0),
    (76.0, 164.0),
    (112.0, 164.0),
    (112.0, 200.0),
    (144.0, 200.0),
    (144.0, 164.0),
    (180.0, 164.0),
    (180.0, 200.0),
    (216.0, 200.0),
];

fn over(dst: Rgba, src: Rgba, coverage: f32) -> Rgba {
    let a = src.0[3] * coverage;
    let out_a = a + dst.0[3] * (1.0 - a);
    if out_a <= 0.0 {
        return Rgba([0.0; 4]);
    }
    let mut c = [0.0; 4];
    for i in 0..3 {
        c[i] = (src.0[i] * a + dst.0[i] * dst.0[3] * (1.0 - a)) / out_a;
    }
    c[3] = out_a;
    Rgba(c)
}

/// Inside a square with rounded corners (side `size` from `inset`, corner radius `r`).
fn in_rounded_square(x: f32, y: f32, inset: f32, r: f32) -> bool {
    let (lo, hi) = (inset, 256.0 - inset);
    let cx = x.clamp(lo + r, hi - r);
    let cy = y.clamp(lo + r, hi - r);
    (x >= lo && x <= hi && y >= lo && y <= hi) && (x - cx).powi(2) + (y - cy).powi(2) <= r * r
}

/// Distance from (x, y) to the segment a-b.
fn segment_distance(x: f32, y: f32, a: (f32, f32), b: (f32, f32)) -> f32 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((x - a.0) * dx + (y - a.1) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    ((x - a.0 - t * dx).powi(2) + (y - a.1 - t * dy).powi(2)).sqrt()
}

/// The rays of the beacon: short strokes pointing away from it.
fn rays() -> impl Iterator<Item = ((f32, f32), (f32, f32))> {
    [8.0f32, 42.0, 138.0, 172.0].into_iter().map(|deg| {
        let (s, c) = deg.to_radians().sin_cos();
        let p = |r: f32| (BEACON_AT.0 + c * r, BEACON_AT.1 - 10.0 - s * r);
        (p(48.0), p(74.0))
    })
}

/// Colour of one point of the canvas.
fn shade(x: f32, y: f32) -> Rgba {
    let mut px = Rgba([0.0; 4]);
    if !in_rounded_square(x, y, 8.0, 44.0) {
        return px;
    }
    // Tile with a thin lighter edge.
    px = over(px, TILE_EDGE, 1.0);
    if in_rounded_square(x, y, 13.0, 39.0) {
        px = over(px, TILE, 1.0);
    }
    // Glow around the beacon, fading out.
    let d = ((x - BEACON_AT.0).powi(2) + (y - BEACON_AT.1).powi(2)).sqrt();
    if d < 84.0 {
        let g = (1.0 - d / 84.0).powi(2) * 0.55;
        px = over(px, GLOW, g);
    }
    for (a, b) in rays() {
        if segment_distance(x, y, a, b) <= 6.0 {
            px = over(px, BEACON, 1.0);
        }
    }
    // The lamp: a dome on a base.
    let dome = d <= DOME_R && y <= BEACON_AT.1;
    let base = (BEACON_AT.0 - 44.0..=BEACON_AT.0 + 44.0).contains(&x)
        && (BEACON_AT.1 + 2.0..=BEACON_AT.1 + 18.0).contains(&y);
    if dome || (y > BEACON_AT.1 && y < BEACON_AT.1 + 2.0 && (x - BEACON_AT.0).abs() <= DOME_R) {
        px = over(px, BEACON, 1.0);
    }
    if base {
        px = over(px, BASE, 1.0);
    }
    if WAVE
        .windows(2)
        .any(|s| segment_distance(x, y, s[0], s[1]) <= 7.0)
    {
        px = over(px, TRACE, 1.0);
    }
    px
}

/// The icon as `size` x `size` RGBA pixels (straight alpha), row by row.
pub fn render(size: u32) -> Vec<u8> {
    const SS: u32 = 4;
    let scale = 256.0 / size as f32;
    let mut out = Vec::with_capacity((size * size * 4) as usize);
    for py in 0..size {
        for px in 0..size {
            // Average the subsamples with premultiplied colour, so edges blend correctly.
            let mut acc = [0.0f32; 4];
            for sy in 0..SS {
                for sx in 0..SS {
                    let x = (px as f32 + (sx as f32 + 0.5) / SS as f32) * scale;
                    let y = (py as f32 + (sy as f32 + 0.5) / SS as f32) * scale;
                    let c = shade(x, y).0;
                    for i in 0..3 {
                        acc[i] += c[i] * c[3];
                    }
                    acc[3] += c[3];
                }
            }
            let n = (SS * SS) as f32;
            let a = acc[3] / n;
            for sum in &acc[..3] {
                let v = if a > 0.0 { sum / n / a } else { 0.0 };
                out.push((v * 255.0).round() as u8);
            }
            out.push((a * 255.0).round() as u8);
        }
    }
    out
}

/// The icon for the window manager.
pub fn icon_data() -> egui::IconData {
    let size = 128;
    egui::IconData {
        rgba: render(size),
        width: size,
        height: size,
    }
}

/// Save the icon as a PNG file.
pub fn write_png(path: &str, size: u32) -> Result<(), String> {
    image::save_buffer(path, &render(size), size, size, image::ColorType::Rgba8)
        .map_err(|e| format!("{path}: {e}"))
}

/// The application ID: names the desktop entry, and Wayland compositors (and WSLg) match the window to
/// that entry by it to show its icon, since a Wayland window cannot set an icon itself.
pub const APP_ID: &str = "blinkfeuer";

/// The desktop entry for `exec` (the path of the binary) with the icon at `icon` (an absolute path, so
/// it does not depend on the icon folders the reader searches).
pub fn desktop_entry(exec: &str, icon: &str) -> String {
    format!(
        "[Desktop Entry]\n\
         Type=Application\n\
         Name=Blinkfeuer\n\
         GenericName=Logic Analyzer\n\
         Comment=Logic analyzer for the Sipeed SLogic16U3 and FX2 analyzers\n\
         Exec=\"{}\"\n\
         Icon={icon}\n\
         Terminal=false\n\
         Categories=Development;Electronics;\n\
         StartupWMClass={APP_ID}\n",
        exec.replace('\\', "\\\\").replace('"', "\\\"")
    )
}

/// Install the icon (in the usual sizes) and a desktop entry for the running binary below `data_home`:
/// `~/.local/share` for the user, or a system folder such as `/usr/local/share` (WSLg only reads
/// desktop entries from the system folders). Returns the files written.
pub fn install_desktop(data_home: &std::path::Path, exec: &str) -> Result<Vec<String>, String> {
    let mut written = Vec::new();
    for size in [32u32, 48, 64, 128, 256] {
        let dir = data_home.join(format!("icons/hicolor/{size}x{size}/apps"));
        std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let path = dir.join(format!("{APP_ID}.png"));
        let p = path.to_string_lossy().into_owned();
        write_png(&p, size)?;
        written.push(p);
    }
    let dir = data_home.join("applications");
    std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let path = dir.join(format!("{APP_ID}.desktop"));
    let icon =
        std::path::absolute(data_home.join(format!("icons/hicolor/256x256/apps/{APP_ID}.png")))
            .map_err(|e| format!("{}: {e}", data_home.display()))?;
    std::fs::write(&path, desktop_entry(exec, &icon.to_string_lossy()))
        .map_err(|e| format!("{}: {e}", path.display()))?;
    written.push(path.to_string_lossy().into_owned());
    Ok(written)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pixel(img: &[u8], size: u32, x: u32, y: u32) -> [u8; 4] {
        let i = ((y * size + x) * 4) as usize;
        [img[i], img[i + 1], img[i + 2], img[i + 3]]
    }

    #[test]
    fn desktop_entry_points_at_the_binary_and_the_icon() {
        let e = desktop_entry(
            "/opt/blink feuer/blinkfeuer",
            "/usr/local/share/icons/b.png",
        );
        assert!(e.starts_with("[Desktop Entry]\n"));
        assert!(e.contains("Exec=\"/opt/blink feuer/blinkfeuer\"\n"));
        assert!(e.contains("Icon=/usr/local/share/icons/b.png\n"));
        assert!(e.contains("StartupWMClass=blinkfeuer\n"));
    }

    #[test]
    fn install_writes_icons_and_the_entry() {
        let dir = std::env::temp_dir().join(format!("bf-desktop-{}", std::process::id()));
        let files = install_desktop(&dir, "/usr/bin/blinkfeuer").unwrap();
        assert_eq!(files.len(), 6);
        assert!(dir
            .join("icons/hicolor/256x256/apps/blinkfeuer.png")
            .is_file());
        assert!(
            std::fs::read_to_string(dir.join("applications/blinkfeuer.desktop"))
                .unwrap()
                .contains("Exec=\"/usr/bin/blinkfeuer\"")
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn corners_are_transparent_and_the_parts_are_where_they_belong() {
        let s = 64;
        let img = render(s);
        assert_eq!(img.len(), (s * s * 4) as usize);
        assert_eq!(pixel(&img, s, 0, 0)[3], 0);
        assert_eq!(pixel(&img, s, s - 1, s - 1)[3], 0);
        // The beacon is yellow, the trace light grey, the tile dark; all opaque.
        let beacon = pixel(&img, s, 32, 22);
        assert!(
            beacon[0] > 240 && beacon[1] > 200 && beacon[2] < 90,
            "{beacon:?}"
        );
        let trace = pixel(&img, s, 14, 50);
        assert!(trace.iter().take(3).all(|&c| c > 200), "{trace:?}");
        let tile = pixel(&img, s, 8, 8);
        assert!(tile[3] == 255 && tile[0] < 60, "{tile:?}");
    }
}
