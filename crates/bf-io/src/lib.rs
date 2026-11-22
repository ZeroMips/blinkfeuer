//! File formats of blinkfeuer: the capture file (`.bfc`: the sparse event stream of a capture plus
//! the application's setup) and exports such as VCD.

pub mod capture;
pub mod vcd;

pub use capture::{read_capture, write_capture, Error, Loaded};
pub use vcd::{write_vcd, VcdSignal};
