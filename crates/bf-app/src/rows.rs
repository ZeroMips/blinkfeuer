//! Row model of the waveform window: single signals and buses (groups of signals shown as one
//! value row), like the "Bus/Signal" setup of a bench logic analyzer. Pure data, no egui.

use serde::{Deserialize, Serialize};

fn yes() -> bool {
    true
}

/// Default trace colour (sRGB).
pub const DEFAULT_COLOR: [u8; 3] = [0xe2, 0xe6, 0xe9];

/// The predefined trace colours offered by the picker (chosen to read well on black).
/// The first entry is the default.
pub const PALETTE: [[u8; 3]; 16] = [
    DEFAULT_COLOR,
    [0x8b, 0x91, 0x98], // grey
    [0xff, 0x5f, 0x56], // red
    [0xff, 0x9a, 0x3c], // orange
    [0xff, 0xd2, 0x3f], // yellow
    [0xb5, 0xe8, 0x53], // lime
    [0x3c, 0xcf, 0x6e], // green
    [0x2e, 0xd3, 0xb7], // teal
    [0x3f, 0xd5, 0xff], // cyan
    [0x4d, 0x9f, 0xff], // blue
    [0x8a, 0x7d, 0xff], // indigo
    [0xc0, 0x7b, 0xff], // violet
    [0xf0, 0x6a, 0xd8], // magenta
    [0xff, 0x7a, 0xa8], // pink
    [0xff, 0x8f, 0x7a], // salmon
    [0xc4, 0x9a, 0x6c], // tan
];

#[derive(Clone, Copy, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum Radix {
    Hex,
    Dec,
    Bin,
}

impl Radix {
    pub const ALL: [Radix; 3] = [Radix::Hex, Radix::Dec, Radix::Bin];

    pub fn label(self) -> &'static str {
        match self {
            Radix::Hex => "Hex",
            Radix::Dec => "Decimal",
            Radix::Bin => "Binary",
        }
    }
}

/// A bus: `members` are channel numbers, most significant bit first.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Bus {
    /// Stable identity (the position in `Layout::items` changes when rows are moved).
    pub id: u32,
    pub name: String,
    pub members: Vec<u8>,
    pub radix: Radix,
    pub expanded: bool,
    pub color: [u8; 3],
    /// False: the bus and its members have no rows.
    #[serde(default = "yes")]
    pub shown: bool,
}

/// A collapsible folder of signals: no combined value, just organisation (like a tree node in the
/// Bus/Signal list of a bench logic analyzer).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Group {
    pub name: String,
    pub members: Vec<u8>,
    /// Protocol decoders shown inside the folder, below its signals (decoder ids).
    #[serde(default)]
    pub decoders: Vec<u32>,
    /// Buses shown inside the folder (by `Bus::id`); the bus itself stays in `Layout::items`.
    #[serde(default)]
    pub buses: Vec<u32>,
    pub expanded: bool,
    /// False: the folder and everything in it has no rows.
    #[serde(default = "yes")]
    pub shown: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Item {
    Signal(u8),
    Bus(Bus),
    Group(Group),
    /// A protocol decoder (by id) at the top level.
    Decoder(u32),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RowKind {
    Signal(u8),
    /// Index into `Layout::items`.
    Bus(usize),
    /// Index into `Layout::items`.
    Group(usize),
    /// A lane of a protocol decoder (rows are added by the application after the signal rows).
    Decode { id: u32, lane: u8 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Row {
    pub kind: RowKind,
    /// 0 at the top level, 1 inside a group or bus, 2 for the members of a bus inside a group.
    pub depth: u8,
    /// Index in `Layout::items` of the bus or group a nested row belongs to.
    pub owner: Option<usize>,
}

/// Something that can be picked up and moved to another place in the row list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mover {
    Signal(u8),
    Decoder(u32),
    /// Index into `Layout::items`.
    Bus(usize),
    /// Index into `Layout::items`.
    Group(usize),
}

/// Where a moved row ends up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dest {
    /// Before `items[i]` at the top level (`items.len()` appends).
    Top(usize),
    /// Into the group at `items[item]`; `pos` indexes its signals (for a signal) or its decoders
    /// (for a decoder) and is clamped to the end.
    Group { item: usize, pos: usize },
}

enum Loc {
    Top(usize),
    In(usize, usize),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Layout {
    pub names: Vec<String>,
    pub visible: Vec<bool>,
    /// Trace colour per channel (also used for the members of a bus).
    pub colors: Vec<[u8; 3]>,
    pub items: Vec<Item>,
    /// Decoders whose rows are not shown.
    #[serde(default)]
    pub hidden_decoders: Vec<u32>,
}

impl Layout {
    pub fn new(channels: usize) -> Self {
        Self {
            names: (0..channels).map(|i| format!("CH{i}")).collect(),
            visible: vec![true; channels],
            colors: vec![DEFAULT_COLOR; channels],
            items: (0..channels as u8).map(Item::Signal).collect(),
            hidden_decoders: Vec::new(),
        }
    }

    /// Whether a layout read from a file is consistent for `channels` channels: one name, colour and
    /// visibility per channel, every channel in exactly one place, bus ids unique.
    pub fn check(&self, channels: usize) -> Result<(), String> {
        if self.names.len() != channels || self.visible.len() != channels || self.colors.len() != channels {
            return Err(format!("the layout is for {} channels, not {channels}", self.names.len()));
        }
        let mut seen = vec![false; channels];
        let mut take = |ch: u8| -> Result<(), String> {
            match seen.get_mut(ch as usize) {
                None => Err(format!("channel {ch} does not exist")),
                Some(s) if *s => Err(format!("channel {ch} is placed twice")),
                Some(s) => {
                    *s = true;
                    Ok(())
                }
            }
        };
        let mut bus_ids = Vec::new();
        for it in &self.items {
            match it {
                Item::Signal(c) => take(*c)?,
                Item::Bus(b) => {
                    if b.members.is_empty() || bus_ids.contains(&b.id) {
                        return Err("damaged bus".into());
                    }
                    bus_ids.push(b.id);
                    b.members.iter().try_for_each(|c| take(*c))?;
                }
                Item::Group(g) => g.members.iter().try_for_each(|c| take(*c))?,
                Item::Decoder(_) => {}
            }
        }
        if seen.contains(&false) {
            return Err("a channel is missing from the layout".into());
        }
        let mut owned = Vec::new();
        for it in &self.items {
            if let Item::Group(g) = it {
                for id in &g.buses {
                    if !bus_ids.contains(id) || owned.contains(id) {
                        return Err("damaged group".into());
                    }
                    owned.push(*id);
                }
            }
        }
        Ok(())
    }

    /// Rename decoder ids (file ids to the ids of the decoders just created); places of unknown ids
    /// and duplicates are dropped.
    pub fn map_decoder_ids(&mut self, map: &std::collections::HashMap<u32, u32>) {
        let mut seen: Vec<u32> = Vec::new();
        let mut keep = |id: u32| -> Option<u32> {
            let n = *map.get(&id)?;
            if seen.contains(&n) {
                return None;
            }
            seen.push(n);
            Some(n)
        };
        let mut items = Vec::new();
        for it in std::mem::take(&mut self.items) {
            match it {
                Item::Decoder(id) => items.extend(keep(id).map(Item::Decoder)),
                Item::Group(mut g) => {
                    g.decoders = g.decoders.iter().filter_map(|d| keep(*d)).collect();
                    items.push(Item::Group(g));
                }
                other => items.push(other),
            }
        }
        self.items = items;
        self.hidden_decoders = self.hidden_decoders.iter().filter_map(|d| map.get(d).copied()).collect();
    }

    /// Give every decoder in `ids` that has no place yet a row at the end of the top level.
    pub fn place_missing_decoders(&mut self, ids: &[u32]) {
        for &id in ids {
            let placed = self.items.iter().any(|it| match it {
                Item::Decoder(d) => *d == id,
                Item::Group(g) => g.decoders.contains(&id),
                _ => false,
            });
            if !placed {
                self.add_decoder(id);
            }
        }
    }

    /// Names and one bus for the simulator's generator pattern.
    pub fn generator(channels: usize) -> Self {
        const NAMES: [&str; 16] = [
            "UART", "SPI CS#", "SPI SCK", "SPI MOSI", "I2C SCL", "I2C SDA", "PWM", "CLK 1 MHz", "GRAY0", "GRAY1",
            "GRAY2", "GRAY3", "GRAY4", "GRAY5", "GRAY6", "GRAY7",
        ];
        let mut l = Self::new(channels);
        for (i, n) in l.names.iter_mut().enumerate() {
            if let Some(s) = NAMES.get(i) {
                *n = (*s).to_string();
            }
        }
        if channels == 16 {
            let members: Vec<u8> = (8..16).rev().collect();
            l.add_bus("GRAY", members, Radix::Hex).expect("generator bus");
        }
        l
    }

    /// Names and one bus for the gateware's own generator pattern: CH0 UART, CH1-3 SPI, CH4 PWM, CH5 the
    /// 100 MHz clock, CH6-15 a 10-bit Gray counter.
    pub fn litex_generator(channels: usize) -> Self {
        const NAMES: [&str; 6] = ["UART", "SPI CS#", "SPI SCK", "SPI MOSI", "PWM", "CLK 100 MHz"];
        let mut l = Self::new(channels);
        for (i, n) in l.names.iter_mut().enumerate() {
            *n = match NAMES.get(i) {
                Some(s) => (*s).to_string(),
                None => format!("GRAY{}", i - NAMES.len()),
            };
        }
        if channels == 16 {
            let members: Vec<u8> = (6..16).rev().collect();
            l.add_bus("GRAY", members, Radix::Hex).expect("generator bus");
        }
        l
    }

    /// The visible rows, top to bottom, without protocol decoders.
    #[cfg(test)]
    pub fn rows(&self) -> Vec<Row> {
        self.rows_with(&|_| Vec::new())
    }

    pub fn rows_with(&self, lanes: &dyn Fn(u32) -> Vec<u8>) -> Vec<Row> {
        let mut out = Vec::new();
        let decode = |out: &mut Vec<Row>, id: u32, depth: u8, owner: Option<usize>| {
            if self.hidden_decoders.contains(&id) {
                return;
            }
            out.extend(lanes(id).into_iter().map(|lane| Row { kind: RowKind::Decode { id, lane }, depth, owner }));
        };
        let bus = |out: &mut Vec<Row>, i: usize, depth: u8, owner: Option<usize>| {
            let Some(b) = self.bus(i) else { return };
            if !b.shown {
                return;
            }
            out.push(Row { kind: RowKind::Bus(i), depth, owner });
            if b.expanded {
                out.extend(
                    b.members
                        .iter()
                        .filter(|&&ch| self.visible.get(ch as usize).copied().unwrap_or(false))
                        .map(|&ch| Row { kind: RowKind::Signal(ch), depth: depth + 1, owner: Some(i) }),
                );
            }
        };
        for (i, item) in self.items.iter().enumerate() {
            match item {
                Item::Signal(ch) => {
                    if self.visible.get(*ch as usize).copied().unwrap_or(false) {
                        out.push(Row { kind: RowKind::Signal(*ch), depth: 0, owner: None });
                    }
                }
                Item::Bus(b) => {
                    if self.group_index_of_bus(b.id).is_none() {
                        bus(&mut out, i, 0, None);
                    }
                }
                Item::Group(g) => {
                    if !g.shown {
                        continue;
                    }
                    out.push(Row { kind: RowKind::Group(i), depth: 0, owner: None });
                    if g.expanded {
                        out.extend(
                            g.members
                                .iter()
                                .filter(|&&ch| self.visible.get(ch as usize).copied().unwrap_or(false))
                                .map(|&ch| Row { kind: RowKind::Signal(ch), depth: 1, owner: Some(i) }),
                        );
                        for &id in &g.buses {
                            if let Some(bi) = self.bus_index(id) {
                                bus(&mut out, bi, 1, Some(i));
                            }
                        }
                        for &id in &g.decoders {
                            decode(&mut out, id, 1, Some(i));
                        }
                    }
                }
                Item::Decoder(id) => decode(&mut out, *id, 0, None),
            }
        }
        out
    }

    /// Index in `items` of the bus with this id.
    pub fn bus_index(&self, id: u32) -> Option<usize> {
        self.items.iter().position(|it| matches!(it, Item::Bus(b) if b.id == id))
    }

    /// Index in `items` of the group that holds the bus with this id.
    pub fn group_index_of_bus(&self, id: u32) -> Option<usize> {
        self.items.iter().position(|it| matches!(it, Item::Group(g) if g.buses.contains(&id)))
    }

    fn next_bus_id(&self) -> u32 {
        self.items.iter().filter_map(|it| if let Item::Bus(b) = it { Some(b.id + 1) } else { None }).max().unwrap_or(0)
    }

    pub fn bus(&self, item: usize) -> Option<&Bus> {
        match self.items.get(item) {
            Some(Item::Bus(b)) => Some(b),
            _ => None,
        }
    }

    pub fn group(&self, item: usize) -> Option<&Group> {
        match self.items.get(item) {
            Some(Item::Group(g)) => Some(g),
            _ => None,
        }
    }

    /// Whether the bus or group at `item` shows its members.
    pub fn is_expanded(&self, item: usize) -> bool {
        match self.items.get(item) {
            Some(Item::Bus(b)) => b.expanded,
            Some(Item::Group(g)) => g.expanded,
            _ => false,
        }
    }

    pub fn toggle_expanded(&mut self, item: usize) {
        match self.items.get_mut(item) {
            Some(Item::Bus(b)) => b.expanded = !b.expanded,
            Some(Item::Group(g)) => g.expanded = !g.expanded,
            _ => {}
        }
    }

    /// Index in `items` of the top-level row (signal, bus or group header), not of an indented member.
    pub fn item_of_row(&self, row: &Row) -> Option<usize> {
        if row.depth > 0 {
            return None;
        }
        match row.kind {
            RowKind::Signal(ch) => self.items.iter().position(|it| *it == Item::Signal(ch)),
            RowKind::Bus(i) | RowKind::Group(i) => Some(i),
            RowKind::Decode { id, .. } => self.items.iter().position(|it| *it == Item::Decoder(id)),
        }
    }

    /// A decoder was added: it gets a row at the end of the top level.
    pub fn add_decoder(&mut self, id: u32) {
        self.items.push(Item::Decoder(id));
    }

    /// A decoder was rebuilt under a new id: it keeps its place.
    pub fn replace_decoder(&mut self, old: u32, new: u32) {
        self.hidden_decoders.iter_mut().filter(|d| **d == old).for_each(|d| *d = new);
        for it in &mut self.items {
            match it {
                Item::Decoder(d) if *d == old => *d = new,
                Item::Group(g) => g.decoders.iter_mut().filter(|d| **d == old).for_each(|d| *d = new),
                _ => {}
            }
        }
    }

    /// A decoder was removed: drop its place wherever it was.
    pub fn remove_decoder(&mut self, id: u32) {
        self.hidden_decoders.retain(|d| *d != id);
        self.items.retain(|it| *it != Item::Decoder(id));
        for it in &mut self.items {
            if let Item::Group(g) = it {
                g.decoders.retain(|d| *d != id);
            }
        }
    }

    /// Hide or show the rows of a decoder.
    pub fn set_decoder_shown(&mut self, id: u32, shown: bool) {
        self.hidden_decoders.retain(|d| *d != id);
        if !shown {
            self.hidden_decoders.push(id);
        }
    }

    /// Put a signal or decoder into a new, empty group (named "Group N") where it is now.
    pub fn move_to_new_group(&mut self, m: Mover) -> bool {
        if !matches!(m, Mover::Signal(_) | Mover::Decoder(_)) {
            return false;
        }
        let at = match self.locate(m) {
            Some(Loc::Top(p)) => p,
            Some(Loc::In(g, _)) => g + 1,
            None => return false,
        };
        let n = self.items.iter().filter(|it| matches!(it, Item::Group(_))).count() + 1;
        let name = format!("Group {n}");
        self.items.insert(at, Item::Group(Group { name, members: Vec::new(), decoders: Vec::new(), buses: Vec::new(), expanded: true, shown: true }));
        self.move_to(m, Dest::Group { item: at, pos: usize::MAX })
    }

    /// The group a signal or decoder sits in, if any.
    pub fn group_of(&self, m: Mover) -> Option<usize> {
        match self.locate(m)? {
            Loc::In(g, _) => Some(g),
            Loc::Top(_) => None,
        }
    }

    /// What a gutter row picks up when dragged (members of a bus cannot be moved on their own).
    pub fn mover_of(&self, row: &Row) -> Option<Mover> {
        match row.kind {
            RowKind::Signal(ch) => {
                let in_bus = row.owner.is_some_and(|o| matches!(self.items.get(o), Some(Item::Bus(_))));
                (!in_bus).then_some(Mover::Signal(ch))
            }
            RowKind::Bus(i) => Some(Mover::Bus(i)),
            RowKind::Group(i) => Some(Mover::Group(i)),
            RowKind::Decode { id, .. } => Some(Mover::Decoder(id)),
        }
    }

    fn locate(&self, m: Mover) -> Option<Loc> {
        match m {
            Mover::Signal(ch) => self.items.iter().enumerate().find_map(|(i, it)| match it {
                Item::Signal(c) if *c == ch => Some(Loc::Top(i)),
                Item::Group(g) => g.members.iter().position(|c| *c == ch).map(|p| Loc::In(i, p)),
                _ => None,
            }),
            Mover::Decoder(id) => self.items.iter().enumerate().find_map(|(i, it)| match it {
                Item::Decoder(d) if *d == id => Some(Loc::Top(i)),
                Item::Group(g) => g.decoders.iter().position(|d| *d == id).map(|p| Loc::In(i, p)),
                _ => None,
            }),
            Mover::Bus(i) => {
                let Some(Item::Bus(b)) = self.items.get(i) else { return None };
                match self.group_index_of_bus(b.id) {
                    Some(g) => match self.items.get(g) {
                        Some(Item::Group(grp)) => grp.buses.iter().position(|x| *x == b.id).map(|p| Loc::In(g, p)),
                        _ => None,
                    },
                    None => Some(Loc::Top(i)),
                }
            }
            Mover::Group(i) => matches!(self.items.get(i), Some(Item::Group(_))).then_some(Loc::Top(i)),
        }
    }

    pub fn move_to(&mut self, m: Mover, dest: Dest) -> bool {
        match dest {
            Dest::Top(i) if i > self.items.len() => return false,
            Dest::Group { item, .. } => {
                if !matches!(self.items.get(item), Some(Item::Group(_))) || matches!(m, Mover::Group(_)) {
                    return false;
                }
            }
            _ => {}
        }
        let Some(from) = self.locate(m) else { return false };
        // A bus stays in `items`; whether it is inside a group is the group's list of bus ids.
        if let Mover::Bus(i) = m {
            let Some(Item::Bus(b)) = self.items.get(i) else { return false };
            let id = b.id;
            let mut dest = dest;
            if let Loc::In(g, p) = from {
                if let Some(Item::Group(grp)) = self.items.get_mut(g) {
                    grp.buses.remove(p);
                }
                if let Dest::Group { item, pos } = &mut dest {
                    if *item == g && p < *pos {
                        *pos -= 1;
                    }
                }
            }
            match dest {
                Dest::Group { item, pos } => {
                    if let Some(Item::Group(g)) = self.items.get_mut(item) {
                        g.buses.insert(pos.min(g.buses.len()), id);
                    }
                }
                Dest::Top(t) => {
                    let it = self.items.remove(i);
                    self.items.insert(if i < t { t - 1 } else { t }, it);
                }
            }
            return true;
        }
        let mut dest = dest;
        let moved = match from {
            Loc::Top(p) => {
                let it = self.items.remove(p);
                match &mut dest {
                    Dest::Top(i) if p < *i => *i -= 1,
                    Dest::Group { item, .. } if p < *item => *item -= 1,
                    _ => {}
                }
                it
            }
            Loc::In(g, p) => {
                let Some(Item::Group(grp)) = self.items.get_mut(g) else { return false };
                let it = match m {
                    Mover::Signal(_) => Item::Signal(grp.members.remove(p)),
                    _ => Item::Decoder(grp.decoders.remove(p)),
                };
                if let Dest::Group { item, pos } = &mut dest {
                    if *item == g && p < *pos {
                        *pos -= 1;
                    }
                }
                it
            }
        };
        match dest {
            Dest::Top(i) => self.items.insert(i.min(self.items.len()), moved),
            Dest::Group { item, pos } => {
                if let Some(Item::Group(g)) = self.items.get_mut(item) {
                    match moved {
                        Item::Signal(ch) => g.members.insert(pos.min(g.members.len()), ch),
                        Item::Decoder(id) => g.decoders.insert(pos.min(g.decoders.len()), id),
                        _ => {}
                    }
                }
            }
        }
        true
    }

    pub fn drop_gap(&self, rows: &[Row], m: Mover, gap: usize) -> Option<Dest> {
        let mut gap = gap.min(rows.len());
        // The lanes of one decoder belong together: a gap between them means "after the decoder".
        if let Some(Row { kind: RowKind::Decode { id, .. }, .. }) = gap.checked_sub(1).and_then(|g| rows.get(g)) {
            while matches!(rows.get(gap), Some(Row { kind: RowKind::Decode { id: j, .. }, .. }) if j == id) {
                gap += 1;
            }
        }
        // Likewise a bus and its expanded members.
        if let Some(Row { kind: RowKind::Bus(b), .. }) = gap.checked_sub(1).and_then(|g| rows.get(g)) {
            while matches!(rows.get(gap), Some(Row { owner: Some(o), kind: RowKind::Signal(_), .. }) if o == b) {
                gap += 1;
            }
        }
        match rows.get(gap) {
            Some(next) if next.depth > 0 => {
                let gi = next.owner?;
                let Some(Item::Group(g)) = self.items.get(gi) else { return None };
                let bus_pos = |i: usize| match self.items.get(i) {
                    Some(Item::Bus(b)) => g.buses.iter().position(|x| *x == b.id),
                    _ => None,
                };
                let pos = match (m, next.kind) {
                    (Mover::Signal(_), RowKind::Signal(ch)) => g.members.iter().position(|c| *c == ch)?,
                    (Mover::Signal(_), _) => g.members.len(),
                    (Mover::Bus(_), RowKind::Bus(i)) => bus_pos(i)?,
                    (Mover::Bus(_), RowKind::Decode { .. }) => g.buses.len(),
                    (Mover::Bus(_), _) => 0,
                    (Mover::Decoder(_), RowKind::Decode { id, .. }) => g.decoders.iter().position(|d| *d == id)?,
                    (Mover::Decoder(_), _) => 0,
                    _ => return None,
                };
                Some(Dest::Group { item: gi, pos })
            }
            Some(next) => self.item_of_row(next).map(Dest::Top),
            None => Some(Dest::Top(self.items.len())),
        }
    }

    /// Dropping on the middle of a group header puts a signal or decoder at the end of the group.
    pub fn drop_onto(&self, row: &Row, m: Mover) -> Option<Dest> {
        match (row.kind, m) {
            (RowKind::Group(i), Mover::Signal(_) | Mover::Decoder(_) | Mover::Bus(_)) if row.depth == 0 => Some(Dest::Group { item: i, pos: usize::MAX }),
            _ => None,
        }
    }

    /// Move a top-level item up (`delta < 0`) or down; returns its new index.
    pub fn move_item(&mut self, item: usize, delta: i32) -> usize {
        let to = (item as i64 + delta as i64).clamp(0, self.items.len().saturating_sub(1) as i64) as usize;
        if item < self.items.len() && to != item {
            let it = self.items.remove(item);
            self.items.insert(to, it);
        }
        to
    }

    pub fn row_label(&self, row: &Row) -> String {
        match row.kind {
            RowKind::Signal(ch) => self.names.get(ch as usize).cloned().unwrap_or_else(|| format!("CH{ch}")),
            RowKind::Bus(i) => match self.bus(i) {
                Some(b) => format!("{} {}", b.name, bus_range(&b.members)),
                None => String::new(),
            },
            RowKind::Group(i) => match self.group(i) {
                Some(g) => format!("{} ({})", g.name, g.members.len() + g.buses.len() + g.decoders.len()),
                None => String::new(),
            },
            RowKind::Decode { .. } => String::new(),
        }
    }

    /// Channels a row is made of (for edge snapping).
    pub fn row_channels(&self, row: &Row) -> Vec<u8> {
        match row.kind {
            RowKind::Signal(ch) => vec![ch],
            RowKind::Bus(i) => self.bus(i).map(|b| b.members.clone()).unwrap_or_default(),
            RowKind::Group(i) => self.group(i).map(|g| g.members.clone()).unwrap_or_default(),
            RowKind::Decode { .. } => Vec::new(),
        }
    }

    /// Collect the given top-level signals into a folder at the position of the first of them.
    pub fn add_group(&mut self, name: &str, members: Vec<u8>) -> Result<(), String> {
        let pos = self.take_members(&members)?;
        let name = if name.trim().is_empty() { "Group".to_string() } else { name.trim().to_string() };
        self.items.insert(pos.min(self.items.len()), Item::Group(Group { name, members, decoders: Vec::new(), buses: Vec::new(), expanded: true, shown: true }));
        Ok(())
    }

    /// Check that all `members` are free top-level signals, remove them from the top level and return
    /// the position of the first one.
    fn take_members(&mut self, members: &[u8]) -> Result<usize, String> {
        if members.is_empty() {
            return Err("no channels".into());
        }
        for (k, ch) in members.iter().enumerate() {
            if *ch as usize >= self.names.len() {
                return Err(format!("channel {ch} does not exist"));
            }
            if members[..k].contains(ch) {
                return Err(format!("channel {ch} listed twice"));
            }
            if !self.items.contains(&Item::Signal(*ch)) {
                return Err(format!("channel {ch} is already part of a bus or group"));
            }
        }
        let pos = self.items.iter().position(|it| matches!(it, Item::Signal(c) if members.contains(c))).unwrap_or(0);
        self.items.retain(|it| !matches!(it, Item::Signal(c) if members.contains(c)));
        Ok(pos)
    }

    /// Group the given top-level signals into a bus at the position of the first of them.
    pub fn add_bus(&mut self, name: &str, members: Vec<u8>, radix: Radix) -> Result<(), String> {
        if members.is_empty() {
            return Err("no channels".into());
        }
        for (k, ch) in members.iter().enumerate() {
            if *ch as usize >= self.names.len() {
                return Err(format!("channel {ch} does not exist"));
            }
            if members[..k].contains(ch) {
                return Err(format!("channel {ch} listed twice"));
            }
            if !self.items.contains(&Item::Signal(*ch)) {
                return Err(format!("channel {ch} is already part of a bus"));
            }
        }
        let pos = self.items.iter().position(|it| matches!(it, Item::Signal(c) if members.contains(c))).unwrap_or(0);
        self.items.retain(|it| !matches!(it, Item::Signal(c) if members.contains(c)));
        let id = self.next_bus_id();
        let name = if name.trim().is_empty() { "BUS".to_string() } else { name.trim().to_string() };
        self.items.insert(pos.min(self.items.len()), Item::Bus(Bus { id, name, members, radix, expanded: false, color: DEFAULT_COLOR, shown: true }));
        Ok(())
    }

    pub fn remove_bus(&mut self, item: usize) {
        let (mut members, decoders, buses) = match self.items.get(item) {
            Some(Item::Bus(b)) => {
                // A bus inside a group leaves it first.
                let id = b.id;
                let m = b.members.clone();
                for it in &mut self.items {
                    if let Item::Group(g) = it {
                        g.buses.retain(|x| *x != id);
                    }
                }
                (m, Vec::new(), Vec::new())
            }
            Some(Item::Group(g)) => (g.members.clone(), g.decoders.clone(), g.buses.clone()),
            _ => return,
        };
        members.sort_unstable();
        let out = members.into_iter().map(Item::Signal).chain(decoders.into_iter().map(Item::Decoder));
        self.items.splice(item..=item, out.collect::<Vec<_>>());
        // Buses of a dissolved group go to the top level, where the group was.
        if !buses.is_empty() {
            let at = item.min(self.items.len());
            let moved: Vec<Item> = buses.iter().filter_map(|id| self.bus_index(*id).map(|i| self.items.remove(i))).collect();
            let at = at.min(self.items.len());
            self.items.splice(at..at, moved);
        }
    }
}

/// "[15:8]" for consecutive members, "[5 bits]" otherwise.
pub fn bus_range(members: &[u8]) -> String {
    let desc = members.windows(2).all(|w| w[0] == w[1] + 1);
    let asc = members.windows(2).all(|w| w[1] == w[0] + 1);
    match (members.first(), members.last()) {
        (Some(a), Some(b)) if members.len() > 1 && (desc || asc) => format!("[{a}:{b}]"),
        (Some(a), _) if members.len() == 1 => format!("[{a}]"),
        _ => format!("[{} bits]", members.len()),
    }
}

/// Value of the bus in `state` (bit k of the channel word is channel k).
pub fn bus_value(state: u32, members: &[u8]) -> u64 {
    members.iter().fold(0u64, |acc, &ch| (acc << 1) | ((state >> ch) & 1) as u64)
}

pub fn format_bus(value: u64, bits: usize, radix: Radix) -> String {
    match radix {
        Radix::Hex => format!("{:0w$X}", value, w = bits.div_ceil(4).max(1)),
        Radix::Dec => format!("{value}"),
        Radix::Bin => format!("{:0w$b}", value, w = bits.max(1)),
    }
}

/// Parse "15-8", "0,2,4-7" or "8..15" into a channel list; ranges keep the written direction.
pub fn parse_channels(text: &str, channels: usize) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    for part in text.split([',', ' ']).map(str::trim).filter(|p| !p.is_empty()) {
        let num = |s: &str| -> Result<u8, String> {
            let v: u8 = s.trim().parse().map_err(|_| format!("'{s}' is not a channel number"))?;
            if (v as usize) < channels {
                Ok(v)
            } else {
                Err(format!("channel {v} does not exist"))
            }
        };
        let sep = if part.contains("..") { Some("..") } else if part.contains('-') { Some("-") } else { None };
        match sep.and_then(|s| part.split_once(s)) {
            Some((a, b)) => {
                let (a, b) = (num(a)?, num(b)?);
                if a <= b {
                    out.extend(a..=b);
                } else {
                    out.extend((b..=a).rev());
                }
            }
            None => out.push(num(part)?),
        }
    }
    if out.is_empty() {
        Err("no channels".into())
    } else {
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bus_values_are_msb_first() {
        // channels 3,2,1,0 = 1,0,1,1
        assert_eq!(bus_value(0b1011, &[3, 2, 1, 0]), 0b1011);
        assert_eq!(bus_value(0b1011, &[0, 1, 2, 3]), 0b1101);
        assert_eq!(bus_value(1 << 15, &[15, 14]), 0b10);
    }

    #[test]
    fn bus_formatting() {
        assert_eq!(format_bus(0x9a, 8, Radix::Hex), "9A");
        assert_eq!(format_bus(0x0a, 8, Radix::Hex), "0A");
        assert_eq!(format_bus(0x5, 10, Radix::Hex), "005");
        assert_eq!(format_bus(5, 4, Radix::Bin), "0101");
        assert_eq!(format_bus(77, 8, Radix::Dec), "77");
    }

    #[test]
    fn channel_lists() {
        assert_eq!(parse_channels("15-8", 16).unwrap(), vec![15, 14, 13, 12, 11, 10, 9, 8]);
        assert_eq!(parse_channels("8..11", 16).unwrap(), vec![8, 9, 10, 11]);
        assert_eq!(parse_channels("0, 2, 4-6", 16).unwrap(), vec![0, 2, 4, 5, 6]);
        assert!(parse_channels("16", 16).is_err());
        assert!(parse_channels("a", 16).is_err());
        assert!(parse_channels("", 16).is_err());
    }

    #[test]
    fn range_labels() {
        assert_eq!(bus_range(&[15, 14, 13]), "[15:13]");
        assert_eq!(bus_range(&[0, 1, 2]), "[0:2]");
        assert_eq!(bus_range(&[0, 2, 5]), "[3 bits]");
        assert_eq!(bus_range(&[7]), "[7]");
    }

    #[test]
    fn grouping_and_dissolving_buses() {
        let mut l = Layout::new(8);
        l.add_bus("B", vec![3, 2, 1], Radix::Hex).unwrap();
        // Members vanish from the top level, the bus sits where the first member was.
        let rows = l.rows();
        assert_eq!(rows.len(), 6);
        assert_eq!(rows[0].kind, RowKind::Signal(0));
        assert!(matches!(rows[1].kind, RowKind::Bus(_)));
        assert_eq!(l.row_label(&rows[1]), "B [3:1]");
        // Expanding shows the members, indented.
        if let Item::Bus(b) = &mut l.items[1] {
            b.expanded = true;
        }
        let rows = l.rows();
        assert_eq!(rows.len(), 9);
        assert!(rows[2].depth == 1 && rows[2].kind == RowKind::Signal(3));
        // A channel can only be in one bus.
        assert!(l.add_bus("C", vec![2, 4], Radix::Hex).is_err());
        assert!(l.add_bus("D", vec![9], Radix::Hex).is_err());
        assert!(l.add_bus("E", vec![4, 4], Radix::Hex).is_err());
        // Dissolving restores single signals in order.
        l.remove_bus(1);
        let chans: Vec<_> = l.rows().iter().map(|r| r.kind).collect();
        assert_eq!(chans, (0..8).map(RowKind::Signal).collect::<Vec<_>>());
    }

    #[test]
    fn colors_default_and_are_per_channel() {
        let mut l = Layout::new(4);
        assert!(l.colors.iter().all(|c| *c == DEFAULT_COLOR));
        l.colors[2] = [1, 2, 3];
        l.add_bus("B", vec![1, 0], Radix::Bin).unwrap();
        assert_eq!(l.colors[2], [1, 2, 3]);
        assert_eq!(l.bus(0).map(|b| b.color), Some(DEFAULT_COLOR));
    }

    #[test]
    fn palette_has_16_distinct_colors_starting_with_the_default() {
        assert_eq!(PALETTE[0], DEFAULT_COLOR);
        for (i, a) in PALETTE.iter().enumerate() {
            assert!(PALETTE[..i].iter().all(|b| b != a), "duplicate at {i}");
        }
    }

    #[test]
    fn groups_collapse_and_expand_without_a_value_row() {
        let mut l = Layout::new(6);
        l.add_group("SPI", vec![1, 2, 3]).unwrap();
        // CH0, the group header (expanded by default) with its three members, CH4, CH5.
        let rows = l.rows();
        assert_eq!(rows.len(), 1 + 1 + 3 + 2);
        assert!(matches!(rows[1].kind, RowKind::Group(_)));
        assert_eq!(l.row_label(&rows[1]), "SPI (3)");
        assert!(rows[2].depth == 1 && rows[2].kind == RowKind::Signal(1));
        l.toggle_expanded(1);
        assert_eq!(l.rows().len(), 1 + 1 + 2);
        assert!(!l.is_expanded(1));
        // Members are not in two places at once.
        assert!(l.add_group("X", vec![2]).is_err());
        assert!(l.add_bus("Y", vec![3], Radix::Hex).is_err());
        // Dissolving brings the signals back in channel order.
        l.remove_bus(1);
        assert_eq!(l.rows().len(), 6);
    }

    #[test]
    fn items_can_be_reordered() {
        let mut l = Layout::new(4);
        l.add_group("G", vec![1, 2]).unwrap(); // [CH0, G, CH3]
        let rows = l.rows();
        assert_eq!(l.item_of_row(&rows[0]), Some(0));
        assert_eq!(l.item_of_row(&rows[1]), Some(1));
        assert_eq!(l.item_of_row(&rows[2]), None, "indented members have no item of their own");
        assert_eq!(l.move_item(1, -1), 0);
        assert!(matches!(l.rows()[0].kind, RowKind::Group(0)));
        assert_eq!(l.move_item(0, -1), 0, "stays at the top");
        assert_eq!(l.move_item(2, 5), 2, "stays at the bottom");
        let last = l.items.len() - 1;
        assert_eq!(l.move_item(0, last as i32), last);
    }

    #[test]
    fn hidden_signals_have_no_row() {
        let mut l = Layout::new(4);
        l.visible[2] = false;
        assert_eq!(l.rows().len(), 3);
    }

    #[test]
    fn generator_layout_has_the_gray_bus() {
        let l = Layout::generator(16);
        let rows = l.rows();
        assert_eq!(rows.len(), 9);
        assert_eq!(l.row_label(&rows[0]), "UART");
        assert_eq!(l.row_label(&rows[8]), "GRAY [15:8]");
        assert_eq!(Layout::generator(8).rows().len(), 8);
    }

    fn names(l: &Layout, lanes: &dyn Fn(u32) -> Vec<u8>) -> Vec<String> {
        l.rows_with(lanes)
            .iter()
            .map(|r| {
                let t = match r.kind {
                    RowKind::Signal(c) => format!("{c}"),
                    RowKind::Bus(_) => "bus".into(),
                    RowKind::Group(_) => "grp".into(),
                    RowKind::Decode { id, lane } => format!("d{id}.{lane}"),
                };
                if r.depth > 0 { format!(" {t}") } else { t }
            })
            .collect()
    }

    #[test]
    fn decoders_live_at_the_top_level_or_in_a_group() {
        let two = |_: u32| vec![0u8, 1];
        let mut l = Layout::new(4);
        l.add_group("G", vec![1, 2]).unwrap();
        l.add_decoder(7);
        assert_eq!(names(&l, &two), ["0", "grp", " 1", " 2", "3", "d7.0", "d7.1"]);
        // Into the group: shown after its signals, indented, and hidden when it is collapsed.
        assert!(l.move_to(Mover::Decoder(7), Dest::Group { item: 1, pos: usize::MAX }));
        assert_eq!(names(&l, &two), ["0", "grp", " 1", " 2", " d7.0", " d7.1", "3"]);
        assert_eq!(l.group_of(Mover::Decoder(7)), Some(1));
        l.toggle_expanded(1);
        assert_eq!(names(&l, &two), ["0", "grp", "3"]);
        l.toggle_expanded(1);
        // Dissolving the group puts signals and decoder back on the top level.
        l.remove_bus(1);
        assert_eq!(names(&l, &two), ["0", "1", "2", "d7.0", "d7.1", "3"]);
        // Removing a decoder removes its rows wherever it was.
        l.add_group("H", vec![0]).unwrap();
        l.move_to(Mover::Decoder(7), Dest::Group { item: 0, pos: 0 });
        l.remove_decoder(7);
        assert_eq!(names(&l, &two), ["grp", " 0", "1", "2", "3"]);
    }

    #[test]
    fn signals_move_between_top_level_and_groups() {
        let mut l = Layout::new(5);
        l.add_group("G", vec![1, 2]).unwrap(); // 0, G[1,2], 3, 4
        let none = |_: u32| Vec::new();
        // Out of the group, to the end.
        assert!(l.move_to(Mover::Signal(1), Dest::Top(l.items.len())));
        assert_eq!(names(&l, &none), ["0", "grp", " 2", "3", "4", "1"]);
        // From the top level into the group, before channel 2.
        assert!(l.move_to(Mover::Signal(4), Dest::Group { item: 1, pos: 0 }));
        assert_eq!(names(&l, &none), ["0", "grp", " 4", " 2", "3", "1"]);
        // Reorder within the group: 4 after 2 (position counted before the removal).
        assert!(l.move_to(Mover::Signal(4), Dest::Group { item: 1, pos: 2 }));
        assert_eq!(names(&l, &none), ["0", "grp", " 2", " 4", "3", "1"]);
        // Top level: moving above the group shifts the group's index correctly.
        assert!(l.move_to(Mover::Signal(3), Dest::Top(0)));
        assert_eq!(names(&l, &none), ["3", "0", "grp", " 2", " 4", "1"]);
        // Buses and groups stay at the top level; nonsense is refused.
        let mut b = Layout::new(4);
        b.add_bus("B", vec![1, 0], Radix::Hex).unwrap();
        assert!(!b.move_to(Mover::Bus(0), Dest::Group { item: 0, pos: 0 }));
        assert!(!b.move_to(Mover::Signal(9), Dest::Top(0)));
        assert!(!b.move_to(Mover::Signal(2), Dest::Top(99)));
        assert!(b.move_to(Mover::Bus(0), Dest::Top(b.items.len())));
        assert_eq!(names(&b, &none), ["2", "3", "bus"]);
    }

    #[test]
    fn dropping_into_gaps_of_the_row_list() {
        let two = |_: u32| vec![0u8, 1];
        let mut l = Layout::new(4);
        l.add_group("G", vec![1, 2]).unwrap();
        l.add_decoder(7);
        // rows: 0 | grp |  1 |  2 | 3 | d7.0 | d7.1
        let rows = l.rows_with(&two);
        let sig = Mover::Signal(0);
        assert_eq!(l.drop_gap(&rows, sig, 0), Some(Dest::Top(0)));
        assert_eq!(l.drop_gap(&rows, sig, 2), Some(Dest::Group { item: 1, pos: 0 }), "between header and first member");
        assert_eq!(l.drop_gap(&rows, sig, 3), Some(Dest::Group { item: 1, pos: 1 }));
        assert_eq!(l.drop_gap(&rows, sig, 4), Some(Dest::Top(2)), "below the last member leaves the group");
        assert_eq!(l.drop_gap(&rows, sig, 7), Some(Dest::Top(4)));
        // Between the lanes of one decoder means after it.
        assert_eq!(l.drop_gap(&rows, sig, 6), Some(Dest::Top(4)));
        // The middle of a group header drops into the group.
        assert_eq!(l.drop_onto(&rows[1], sig), Some(Dest::Group { item: 1, pos: usize::MAX }));
        assert_eq!(l.drop_onto(&rows[0], sig), None);
        assert_eq!(l.drop_onto(&rows[1], Mover::Group(1)), None);
        // A group cannot go into a group.
        assert_eq!(l.drop_gap(&rows, Mover::Group(1), 3), None);
        // Decoder rows pick up their decoder; members of a bus cannot be picked up.
        assert_eq!(l.mover_of(&rows[5]), Some(Mover::Decoder(7)));
        let mut b = Layout::new(3);
        b.add_bus("B", vec![1, 0], Radix::Hex).unwrap();
        b.toggle_expanded(0);
        let rows = b.rows(); // bus, 1, 0, 2
        assert_eq!(b.mover_of(&rows[0]), Some(Mover::Bus(0)));
        assert_eq!(b.mover_of(&rows[1]), None);
        assert_eq!(b.drop_gap(&rows, Mover::Signal(2), 2), None, "inside a bus");
    }

    #[test]
    fn hidden_decoders_and_new_groups() {
        let one = |_: u32| vec![0u8];
        let mut l = Layout::new(3);
        l.add_decoder(5);
        l.set_decoder_shown(5, false);
        assert_eq!(names(&l, &one), ["0", "1", "2"]);
        l.set_decoder_shown(5, true);
        assert_eq!(names(&l, &one), ["0", "1", "2", "d5.0"]);
        // A signal and a decoder into new groups, at their old places.
        assert!(l.move_to_new_group(Mover::Signal(1)));
        assert_eq!(names(&l, &one), ["0", "grp", " 1", "2", "d5.0"]);
        assert!(l.move_to_new_group(Mover::Decoder(5)));
        assert_eq!(names(&l, &one), ["0", "grp", " 1", "2", "grp", " d5.0"]);
        assert!(l.group(1).is_some_and(|g| g.name == "Group 1"));
        assert!(l.group(3).is_some_and(|g| g.name == "Group 2"));
        l.set_decoder_shown(5, false);
        l.remove_decoder(5);
        assert!(l.hidden_decoders.is_empty());
    }

    #[test]
    fn buses_can_live_in_groups() {
        let none = |_: u32| Vec::new();
        let mut l = Layout::new(6);
        l.add_bus("B", vec![3, 2], Radix::Hex).unwrap(); // 0 1 B 4 5   (bus is item 2)
        l.add_group("G", vec![0]).unwrap(); // G[0] 1 B 4 5
        assert_eq!(names(&l, &none), ["grp", " 0", "1", "bus", "4", "5"]);
        let bus = l.items.iter().position(|it| matches!(it, Item::Bus(_))).unwrap();
        assert!(l.move_to(Mover::Bus(bus), Dest::Group { item: 0, pos: usize::MAX }));
        assert_eq!(names(&l, &none), ["grp", " 0", " bus", "1", "4", "5"]);
        assert_eq!(l.group_of(Mover::Bus(bus)), Some(0));
        // Expanded, its members sit one level deeper and cannot be picked up.
        l.toggle_expanded(bus);
        let rows = l.rows();
        assert_eq!(rows.iter().map(|r| r.depth).collect::<Vec<_>>(), [0, 1, 1, 2, 2, 0, 0, 0]);
        assert_eq!(l.mover_of(&rows[3]), None);
        assert_eq!(l.mover_of(&rows[2]), Some(Mover::Bus(bus)));
        // Hiding the group hides the bus with it; the bus reappears with the group.
        if let Item::Group(g) = &mut l.items[0] {
            g.shown = false;
        }
        assert_eq!(names(&l, &none), ["1", "4", "5"]);
        if let Item::Group(g) = &mut l.items[0] {
            g.shown = true;
        }
        // Out of the group to the top level (before channel 4), and the label counts what is inside.
        assert_eq!(l.row_label(&l.rows()[0]), "G (2)");
        let end = l.items.len();
        assert!(l.move_to(Mover::Bus(bus), Dest::Top(end)));
        assert_eq!(names(&l, &none), ["grp", " 0", "1", "4", "5", "bus", " 3", " 2"]);
    }

    #[test]
    fn hidden_buses_dissolving_groups_with_buses() {
        let none = |_: u32| Vec::new();
        let mut l = Layout::new(4);
        l.add_bus("B", vec![1, 0], Radix::Hex).unwrap();
        l.toggle_expanded(0);
        if let Item::Bus(b) = &mut l.items[0] {
            b.shown = false;
        }
        assert_eq!(names(&l, &none), ["2", "3"], "a hidden bus takes its members with it");
        if let Item::Bus(b) = &mut l.items[0] {
            b.shown = true;
        }
        // A hidden member leaves only its own row.
        l.visible[0] = false;
        assert_eq!(names(&l, &none), ["bus", " 1", "2", "3"]);
        l.visible[0] = true;
        // Dissolving a group with a bus in it keeps the bus.
        l.add_group("G", vec![2]).unwrap();
        let g = l.items.iter().position(|it| matches!(it, Item::Group(_))).unwrap();
        assert!(l.move_to(Mover::Bus(0), Dest::Group { item: g, pos: 0 }));
        let g = l.items.iter().position(|it| matches!(it, Item::Group(_))).unwrap();
        l.remove_bus(g);
        assert_eq!(names(&l, &none), ["2", "bus", " 1", " 0", "3"], "signals first, then buses, where the group was");
    }

    #[test]
    fn litex_generator_layout_has_its_ten_bit_gray_bus() {
        let l = Layout::litex_generator(16);
        assert_eq!(l.names[0], "UART");
        assert_eq!(l.names[5], "CLK 100 MHz");
        assert_eq!(l.names[6], "GRAY0");
        assert_eq!(l.names[15], "GRAY9");
        let rows = l.rows();
        assert_eq!(rows.len(), 7);
        assert_eq!(l.row_label(&rows[6]), "GRAY [15:6]");
        assert_eq!(Layout::litex_generator(8).rows().len(), 8);
    }
}
