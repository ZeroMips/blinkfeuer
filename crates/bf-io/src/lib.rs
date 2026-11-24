//! File formats of blinkfeuer: the capture file (`.bfc`: the sparse event stream of a capture plus
//! the application's setup), exports such as VCD and the import of sigrok session files.

#![warn(missing_docs)]

pub mod capture;
pub mod sigrok;
pub mod vcd;

pub use capture::{read_capture, write_capture, Error, Loaded};
pub use sigrok::{read_sr, SigrokCapture};
pub use vcd::{write_vcd, VcdSignal};
