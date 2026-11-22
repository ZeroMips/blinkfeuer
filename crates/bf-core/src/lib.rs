//! Core data model of blinkfeuer: sample formats, the capture store and (later) triggers.
//!
//! No UI and no USB code lives here, so everything is unit-testable and benchmarkable.

pub mod format;
pub mod store;
pub mod trigger;

pub use format::{pack, pack_alignment, ChannelCount, Unpacker};
pub use store::{CaptureStore, Column, Event, SharedStore, StoreConfig};
pub use trigger::{Cond, EventTrigger, Trigger};
