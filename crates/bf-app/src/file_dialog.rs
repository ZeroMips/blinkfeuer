//! blinkfeuer's file dialog, drawn with egui in the application window (no desktop portal needed, so it
//! behaves the same everywhere, WSL included): browse directories, pick a file, or type a name to save
//! under, with a confirmation before an existing file is replaced.

use crate::theme;
use egui::{Key, RichText};
use std::path::{Path, PathBuf};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry {
    pub name: String,
    pub is_dir: bool,
    pub size: u64,
}

/// What the dialog was closed with.
#[derive(Debug, PartialEq, Eq)]
pub enum Outcome {
    Chosen(PathBuf),
    Cancelled,
}

/// A file name matches a filter when it ends in one of its extensions (any case); `*` matches all.
pub fn matches_filter(name: &str, exts: &[String]) -> bool {
    exts.iter().any(|e| {
        e == "*"
            || Path::new(name)
                .extension()
                .is_some_and(|x| x.to_string_lossy().eq_ignore_ascii_case(e))
    })
}

/// The entries of `dir`: directories first, then the files that match the filter, each sorted by name
/// ignoring case. Hidden entries (leading dot) only when asked for.
pub fn list_dir(dir: &Path, exts: &[String], hidden: bool) -> std::io::Result<Vec<Entry>> {
    let mut out = Vec::new();
    for e in std::fs::read_dir(dir)?.flatten() {
        let name = e.file_name().to_string_lossy().into_owned();
        if !hidden && name.starts_with('.') {
            continue;
        }
        // Follow symbolic links, so a link to a directory can be entered.
        let Ok(meta) = std::fs::metadata(e.path()) else {
            continue;
        };
        if meta.is_dir() || matches_filter(&name, exts) {
            out.push(Entry {
                name,
                is_dir: meta.is_dir(),
                size: meta.len(),
            });
        }
    }
    out.sort_by(|a, b| {
        b.is_dir
            .cmp(&a.is_dir)
            .then_with(|| a.name.to_lowercase().cmp(&b.name.to_lowercase()))
    });
    Ok(out)
}

/// A typed path: absolute, `~/...` below the home directory, or relative to `dir`.
pub fn resolve(dir: &Path, typed: &str, home: Option<&Path>) -> PathBuf {
    let typed = typed.trim();
    if typed == "~" {
        if let Some(h) = home {
            return h.to_path_buf();
        }
    }
    if let (Some(rest), Some(h)) = (typed.strip_prefix("~/"), home) {
        return h.join(rest);
    }
    dir.join(typed)
}

fn human_size(n: u64) -> String {
    match n {
        n if n >= 1 << 30 => format!("{:.1} GiB", n as f64 / (1u64 << 30) as f64),
        n if n >= 1 << 20 => format!("{:.1} MiB", n as f64 / (1u64 << 20) as f64),
        n if n >= 1 << 10 => format!("{:.1} KiB", n as f64 / 1024.0),
        n => format!("{n} B"),
    }
}

pub struct FileDialog {
    title: String,
    save: bool,
    dir: PathBuf,
    /// The location field, edited by hand or following `dir`.
    location: String,
    /// File name: typed (save) or the selected file.
    name: String,
    filters: Vec<(String, Vec<String>)>,
    filter: usize,
    hidden: bool,
    entries: Vec<Entry>,
    error: Option<String>,
    /// An existing file waiting for the "replace" confirmation.
    replace: Option<PathBuf>,
}

impl FileDialog {
    pub fn new(
        title: &str,
        save: bool,
        dir: Option<PathBuf>,
        name: Option<String>,
        filters: Vec<(String, Vec<String>)>,
    ) -> Self {
        let dir = dir
            .filter(|d| d.is_dir())
            .or_else(|| std::env::current_dir().ok())
            .or_else(dirs::home_dir)
            .unwrap_or_else(|| PathBuf::from("/"));
        let mut d = Self {
            title: title.to_string(),
            save,
            dir: PathBuf::new(),
            location: String::new(),
            name: name.unwrap_or_default(),
            filters,
            filter: 0,
            hidden: false,
            entries: Vec::new(),
            error: None,
            replace: None,
        };
        d.enter(dir);
        d
    }

    fn exts(&self) -> &[String] {
        self.filters.get(self.filter).map_or(&[], |f| &f.1[..])
    }

    fn refresh(&mut self) {
        match list_dir(&self.dir, self.exts(), self.hidden) {
            Ok(e) => {
                self.entries = e;
                self.error = None;
            }
            Err(e) => {
                self.entries.clear();
                self.error = Some(format!("{}: {e}", self.dir.display()));
            }
        }
    }

    fn enter(&mut self, dir: PathBuf) {
        self.dir = std::fs::canonicalize(&dir).unwrap_or(dir);
        self.location = self.dir.display().to_string();
        self.replace = None;
        self.refresh();
    }

    /// Accept the typed or selected name: enter a directory, or finish with a file.
    fn accept(&mut self) -> Option<Outcome> {
        if self.name.trim().is_empty() {
            return None;
        }
        let path = resolve(&self.dir, &self.name, dirs::home_dir().as_deref());
        if path.is_dir() {
            self.name.clear();
            self.enter(path);
            return None;
        }
        if self.save {
            if !path.parent().is_some_and(Path::is_dir) {
                self.error = Some(format!("{}: no such directory", path.display()));
                return None;
            }
            if path.exists() && self.replace.as_deref() != Some(&path) {
                self.replace = Some(path);
                return None;
            }
        } else if !path.is_file() {
            self.error = Some(format!("{}: no such file", path.display()));
            return None;
        }
        Some(Outcome::Chosen(path))
    }

    /// Draw the dialog; `Some` once it is closed.
    pub fn show(&mut self, ctx: &egui::Context) -> Option<Outcome> {
        let mut outcome = None;
        let mut open = true;
        let dim = |t: String| RichText::new(t).color(theme::TEXT_DIM);
        egui::Window::new(&self.title)
            .id(egui::Id::new("blinkfeuer-file-dialog"))
            .open(&mut open)
            .collapsible(false)
            .default_size([620.0, 420.0])
            .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
            .show(ctx, |ui| {
                // Location row.
                ui.horizontal(|ui| {
                    if ui.button("Up").clicked() {
                        if let Some(p) = self.dir.parent().map(Path::to_path_buf) {
                            self.enter(p);
                        }
                    }
                    if ui.button("Home").clicked() {
                        if let Some(h) = dirs::home_dir() {
                            self.enter(h);
                        }
                    }
                    let r = ui.add(
                        egui::TextEdit::singleline(&mut self.location).desired_width(f32::INFINITY),
                    );
                    if r.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter)) {
                        let p = resolve(&self.dir, &self.location, dirs::home_dir().as_deref());
                        if p.is_dir() {
                            self.enter(p);
                        } else {
                            self.error = Some(format!("{}: not a directory", p.display()));
                        }
                    }
                });
                ui.separator();
                // Bottom rows first, so the list takes the remaining height.
                egui::TopBottomPanel::bottom("blinkfeuer-file-dialog-bottom")
                    .frame(egui::Frame::none())
                    .show_inside(ui, |ui| {
                        ui.add_space(4.0);
                        if let Some(e) = &self.error {
                            ui.label(RichText::new(e).color(theme::STOP));
                        }
                        if let Some(p) = &self.replace {
                            let name = p.file_name().unwrap_or_default().to_string_lossy();
                            ui.label(
                                RichText::new(format!(
                                    "{name} exists already. Click Save again to replace it."
                                ))
                                .color(theme::DEC_WARN),
                            );
                        }
                        ui.horizontal(|ui| {
                            ui.label(dim("Name".into()));
                            let r = ui.add(
                                egui::TextEdit::singleline(&mut self.name).desired_width(300.0),
                            );
                            if r.changed() {
                                self.replace = None;
                            }
                            if r.lost_focus() && ui.input(|i| i.key_pressed(Key::Enter)) {
                                outcome = self.accept();
                            }
                            let before = self.filter;
                            egui::ComboBox::from_id_salt("blinkfeuer-file-dialog-filter")
                                .selected_text(
                                    self.filters.get(self.filter).map_or("", |f| f.0.as_str()),
                                )
                                .show_ui(ui, |ui| {
                                    for (i, f) in self.filters.iter().enumerate() {
                                        ui.selectable_value(&mut self.filter, i, &f.0);
                                    }
                                });
                            if self.filter != before {
                                self.refresh();
                            }
                        });
                        ui.horizontal(|ui| {
                            if ui.checkbox(&mut self.hidden, "Hidden files").changed() {
                                self.refresh();
                            }
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    let label = if self.save { "Save" } else { "Open" };
                                    if ui.button(label).clicked() {
                                        outcome = self.accept();
                                    }
                                    if ui.button("Cancel").clicked() {
                                        outcome = Some(Outcome::Cancelled);
                                    }
                                },
                            );
                        });
                    });
                // Directory listing.
                let mut enter = None;
                let mut chosen = false;
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        for e in &self.entries {
                            let text = if e.is_dir {
                                format!("{}/", e.name)
                            } else {
                                e.name.clone()
                            };
                            ui.horizontal(|ui| {
                                let selected = !e.is_dir && self.name == e.name;
                                let r = ui.selectable_label(selected, text);
                                if !e.is_dir {
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| ui.label(dim(human_size(e.size))),
                                    );
                                }
                                if e.is_dir && (r.double_clicked() || r.clicked()) {
                                    enter = Some(self.dir.join(&e.name));
                                } else if r.clicked() || r.double_clicked() {
                                    self.name = e.name.clone();
                                    self.replace = None;
                                    chosen = r.double_clicked();
                                }
                            });
                        }
                        if self.entries.is_empty() && self.error.is_none() {
                            ui.label(dim("(no matching files)".into()));
                        }
                    });
                if let Some(d) = enter {
                    self.enter(d);
                }
                if chosen {
                    outcome = self.accept();
                }
            });
        if !open || ctx.input(|i| i.key_pressed(Key::Escape)) {
            outcome = Some(Outcome::Cancelled);
        }
        outcome
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn exts(e: &[&str]) -> Vec<String> {
        e.iter().map(|s| s.to_string()).collect()
    }

    struct TempDir(PathBuf);

    impl TempDir {
        fn new(tag: &str) -> Self {
            let d = std::env::temp_dir().join(format!("bf-dialog-{tag}-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&d);
            std::fs::create_dir_all(&d).unwrap();
            TempDir(d)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn filter_matches_extensions_in_any_case() {
        assert!(matches_filter("a.bfc", &exts(&["bfc", "sr"])));
        assert!(matches_filter("A.SR", &exts(&["bfc", "sr"])));
        assert!(!matches_filter("a.json", &exts(&["bfc"])));
        assert!(!matches_filter("bfc", &exts(&["bfc"])));
        assert!(matches_filter("anything", &exts(&["*"])));
    }

    #[test]
    fn listing_puts_directories_first_and_filters_files() {
        let t = TempDir::new("list");
        for f in ["b.bfc", "A.bfc", "notes.txt", ".hidden.bfc"] {
            std::fs::write(t.0.join(f), b"x").unwrap();
        }
        std::fs::create_dir(t.0.join("zdir")).unwrap();
        std::fs::create_dir(t.0.join(".git")).unwrap();
        let names = |v: Vec<Entry>| v.into_iter().map(|e| e.name).collect::<Vec<_>>();
        assert_eq!(
            names(list_dir(&t.0, &exts(&["bfc"]), false).unwrap()),
            ["zdir", "A.bfc", "b.bfc"]
        );
        assert_eq!(
            names(list_dir(&t.0, &exts(&["bfc"]), true).unwrap()),
            [".git", "zdir", ".hidden.bfc", "A.bfc", "b.bfc"]
        );
        assert_eq!(list_dir(&t.0, &exts(&["*"]), false).unwrap().len(), 4);
    }

    #[test]
    fn typed_paths_resolve_against_the_directory_and_home() {
        let dir = Path::new("/data/captures");
        let home = Some(Path::new("/home/u"));
        assert_eq!(resolve(dir, "x.bfc", home), dir.join("x.bfc"));
        assert_eq!(resolve(dir, "/tmp/x.bfc", home), Path::new("/tmp/x.bfc"));
        assert_eq!(resolve(dir, "~/x.bfc", home), Path::new("/home/u/x.bfc"));
        assert_eq!(resolve(dir, "~", home), Path::new("/home/u"));
        assert_eq!(resolve(dir, " sub/x.bfc ", home), dir.join("sub/x.bfc"));
    }

    #[test]
    fn saving_over_an_existing_file_needs_a_second_confirmation() {
        let t = TempDir::new("save");
        std::fs::write(t.0.join("old.bfc"), b"x").unwrap();
        let mut d = FileDialog::new(
            "Save",
            true,
            Some(t.0.clone()),
            Some("new.bfc".into()),
            vec![("Capture".into(), exts(&["bfc"]))],
        );
        let dir = d.dir.clone();
        assert_eq!(d.accept(), Some(Outcome::Chosen(dir.join("new.bfc"))));
        d.name = "old.bfc".into();
        assert_eq!(d.accept(), None);
        assert!(d.replace.is_some());
        assert_eq!(d.accept(), Some(Outcome::Chosen(dir.join("old.bfc"))));
        d.name = "missing/x.bfc".into();
        assert_eq!(d.accept(), None);
        assert!(d.error.is_some());
    }

    #[test]
    fn opening_needs_an_existing_file_and_a_directory_name_enters_it() {
        let t = TempDir::new("open");
        std::fs::create_dir(t.0.join("sub")).unwrap();
        std::fs::write(t.0.join("sub/a.bfc"), b"x").unwrap();
        let mut d = FileDialog::new(
            "Open",
            false,
            Some(t.0.clone()),
            None,
            vec![("Capture".into(), exts(&["bfc"]))],
        );
        d.name = "sub".into();
        assert_eq!(d.accept(), None);
        assert!(d.dir.ends_with("sub"));
        assert_eq!(d.entries.len(), 1);
        d.name = "nope.bfc".into();
        assert_eq!(d.accept(), None);
        d.name = "a.bfc".into();
        assert_eq!(d.accept(), Some(Outcome::Chosen(d.dir.join("a.bfc"))));
    }
}
