//! Modbus RTU on top of the UART decoder.
//!
//! Frames are delimited by silence on the line: 3.5 character times, or a fixed 1.75 ms above
//! 19200 baud (Modbus over serial line, section 2.5.1.1). Devices often answer faster than that, so a
//! block of characters whose CRC fails is also tried as two or more back-to-back frames, each with a
//! valid CRC. Every frame is shown as a bar on the "Frame" lane (address and function) and split into
//! its fields on the "Fields" lane.
//!
//! Whether a frame is a request or a response follows from its function code and length where that is
//! unambiguous (e.g. a "read holding registers" request is always 8 bytes, its response 5 + 2n bytes).
//! Functions whose request and response look the same (write single register: the response echoes the
//! request) stay undecided.

use crate::layer::{ByteLayer, CharTiming};
use crate::{flag, format_value, Annotation, Format, Kind};
use std::ops::Range;

/// [`Annotation::bits`] of a [`Kind::ModbusFrame`]: the frame is a request (master to slave).
pub const REQUEST: u8 = 1;
/// ... a response (slave to master).
pub const RESPONSE: u8 = 2;

/// The longest RTU frame (address, PDU of up to 253 bytes, CRC).
const MAX_FRAME: usize = 256;
/// The shortest frame with a CRC: address, function, CRC.
const MIN_FRAME: usize = 4;

/// CRC-16/MODBUS (polynomial 0xA001 reflected, start 0xFFFF). Over a whole frame including its CRC
/// (low byte first, as transmitted) the result is 0.
pub fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &b in bytes {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xA001
            } else {
                crc >> 1
            };
        }
    }
    crc
}

/// Name of a function code (the exception bit is ignored).
pub fn function_name(fc: u8) -> Option<&'static str> {
    Some(match fc & 0x7F {
        1 => "Read Coils",
        2 => "Read Discrete Inputs",
        3 => "Read Holding Registers",
        4 => "Read Input Registers",
        5 => "Write Single Coil",
        6 => "Write Single Register",
        7 => "Read Exception Status",
        8 => "Diagnostics",
        11 => "Get Comm Event Counter",
        12 => "Get Comm Event Log",
        15 => "Write Multiple Coils",
        16 => "Write Multiple Registers",
        17 => "Report Server ID",
        20 => "Read File Record",
        21 => "Write File Record",
        22 => "Mask Write Register",
        23 => "Read/Write Multiple Registers",
        24 => "Read FIFO Queue",
        43 => "Encapsulated Interface Transport",
        _ => return None,
    })
}

/// Name of an exception code.
pub fn exception_name(code: u8) -> Option<&'static str> {
    Some(match code {
        1 => "Illegal Function",
        2 => "Illegal Data Address",
        3 => "Illegal Data Value",
        4 => "Server Device Failure",
        5 => "Acknowledge",
        6 => "Server Device Busy",
        8 => "Memory Parity Error",
        10 => "Gateway Path Unavailable",
        11 => "Gateway Target Device Failed to Respond",
        _ => return None,
    })
}

/// Numbers in the decoder's format; ASCII makes no sense for Modbus fields and shows hex.
fn num(value: u32, bits: u8, fmt: Format) -> String {
    let fmt = if fmt == Format::Ascii {
        Format::Hex
    } else {
        fmt
    };
    format_value(value, bits, fmt)
}

/// Text of a Modbus annotation (without the error suffixes, which [`Annotation::label`] adds).
pub fn label(a: &Annotation, fmt: Format) -> String {
    let v = a.value;
    match a.kind {
        Kind::ModbusFrame => {
            let (addr, fc) = ((v >> 8) as u8, v as u8);
            let what = match (fc & 0x80 != 0, function_name(fc)) {
                (true, Some(n)) => format!("Exception ({n})"),
                (true, None) => format!("Exception (function {})", fc & 0x7F),
                (false, Some(n)) => n.to_string(),
                (false, None) => format!("Function {fc}"),
            };
            let dir = match a.bits {
                REQUEST => " request",
                RESPONSE => " response",
                _ => "",
            };
            format!("Slave {}: {what}{dir}", num(addr as u32, 8, fmt))
        }
        Kind::ModbusAddress => format!("Slave {}", num(v, 8, fmt)),
        Kind::ModbusFunction => match function_name(v as u8) {
            Some(n) if v & 0x80 == 0 => format!("FC {v} {n}"),
            _ => format!("FC {v}"),
        },
        Kind::ModbusRegister => format!("Reg {}", num(v, 16, fmt)),
        Kind::ModbusQuantity => format!("Qty {v}"),
        Kind::ModbusByteCount => format!("{v} byte{}", if v == 1 { "" } else { "s" }),
        Kind::ModbusValue => num(v, 16, fmt),
        Kind::ModbusData => num(v, 8, fmt),
        Kind::ModbusException => match exception_name(v as u8) {
            Some(n) => format!("Exception {v}: {n}"),
            None => format!("Exception {v}"),
        },
        Kind::ModbusCrc => {
            if a.flags & flag::CHECKSUM != 0 {
                format!("CRC bad ({})", num(v, 16, Format::Hex))
            } else {
                "CRC ok".into()
            }
        }
        _ => String::new(),
    }
}

/// One field of a frame: `bytes` indexes into the frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Field {
    /// What the field is.
    pub kind: Kind,
    /// Its bytes in the frame.
    pub bytes: Range<usize>,
    /// Its value.
    pub value: u32,
}

/// The fields of a frame (CRC included) and whether it is a request or a response (0: undecided).
pub fn parse(frame: &[u8]) -> (u8, Vec<Field>) {
    let mut f = Vec::new();
    let mut add = |kind, bytes: Range<usize>, value: u32| f.push(Field { kind, bytes, value });
    if frame.len() < MIN_FRAME {
        for (i, &b) in frame.iter().enumerate() {
            add(Kind::ModbusData, i..i + 1, b as u32);
        }
        return (0, f);
    }
    let n = frame.len() - 2; // without the CRC
    let body = &frame[..n];
    let fc = body[1];
    let be16 = |i: usize| (body[i] as u32) << 8 | body[i + 1] as u32;
    add(Kind::ModbusAddress, 0..1, body[0] as u32);
    add(Kind::ModbusFunction, 1..2, fc as u32);
    // Everything from byte `from` on as data: registers (pairs) or bytes.
    let data = |add: &mut dyn FnMut(Kind, Range<usize>, u32), from: usize, registers: bool| {
        let mut i = from;
        while i < n {
            if registers && i + 2 <= n {
                add(Kind::ModbusValue, i..i + 2, be16(i));
                i += 2;
            } else {
                add(Kind::ModbusData, i..i + 1, body[i] as u32);
                i += 1;
            }
        }
    };
    let dir = match fc {
        _ if fc & 0x80 != 0 && n == 3 => {
            add(Kind::ModbusException, 2..3, body[2] as u32);
            RESPONSE
        }
        // Read coils, discrete inputs, holding registers, input registers.
        1..=4 if n >= 3 && body[2] as usize == n - 3 && (fc <= 2 || (n - 3).is_multiple_of(2)) => {
            add(Kind::ModbusByteCount, 2..3, body[2] as u32);
            data(&mut add, 3, fc >= 3);
            RESPONSE
        }
        1..=4 if n == 6 => {
            add(Kind::ModbusRegister, 2..4, be16(2));
            add(Kind::ModbusQuantity, 4..6, be16(4));
            REQUEST
        }
        // Write single coil / register: the response echoes the request.
        5 | 6 if n == 6 => {
            add(Kind::ModbusRegister, 2..4, be16(2));
            add(Kind::ModbusValue, 4..6, be16(4));
            0
        }
        // Write multiple coils / registers.
        15 | 16 if n == 6 => {
            add(Kind::ModbusRegister, 2..4, be16(2));
            add(Kind::ModbusQuantity, 4..6, be16(4));
            RESPONSE
        }
        15 | 16 if n >= 7 && body[6] as usize == n - 7 => {
            add(Kind::ModbusRegister, 2..4, be16(2));
            add(Kind::ModbusQuantity, 4..6, be16(4));
            add(Kind::ModbusByteCount, 6..7, body[6] as u32);
            data(&mut add, 7, fc == 16);
            REQUEST
        }
        _ => {
            data(&mut add, 2, false);
            0
        }
    };
    let crc = u16::from_le_bytes([frame[n], frame[n + 1]]);
    add(Kind::ModbusCrc, n..n + 2, crc as u32);
    (dir, f)
}

/// Split a block of characters (delimited by silence) into frames: the whole block if its CRC is
/// valid, otherwise the shortest prefix with a valid CRC followed by the split of the rest. What is
/// left without a valid CRC becomes one frame with a bad CRC.
pub fn split(bytes: &[u8]) -> Vec<(Range<usize>, bool)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        let rest = &bytes[i..];
        if rest.len() >= MIN_FRAME && crc16(rest) == 0 {
            out.push((i..bytes.len(), true));
            break;
        }
        match (i + MIN_FRAME..bytes.len()).find(|&j| crc16(&bytes[i..j]) == 0) {
            Some(j) => {
                out.push((i..j, true));
                i = j;
            }
            None => {
                out.push((i..bytes.len(), false));
                break;
            }
        }
    }
    out
}

/// The Modbus RTU byte layer: lanes "Frame" (1) and "Fields" (2).
pub struct ModbusRtu {
    /// Silence that ends a frame, in samples.
    t35: u64,
    /// Characters of the frame being received.
    chars: Vec<Annotation>,
}

impl Default for ModbusRtu {
    fn default() -> Self {
        Self::new()
    }
}

impl ModbusRtu {
    /// A layer waiting for the first frame.
    pub fn new() -> Self {
        Self {
            t35: 0,
            chars: Vec::new(),
        }
    }

    fn finish(&mut self, out: &mut Vec<Annotation>) {
        if self.chars.is_empty() {
            return;
        }
        let bytes: Vec<u8> = self.chars.iter().map(|c| c.value as u8).collect();
        for (range, crc_ok) in split(&bytes) {
            let chars = &self.chars[range.clone()];
            let frame = &bytes[range];
            let char_errors = chars.iter().fold(0, |f, c| f | c.flags) & flag::ERROR;
            let (dir, fields) = parse(frame);
            let crc_flag = if crc_ok { 0 } else { flag::CHECKSUM };
            out.push(Annotation {
                start: chars[0].start,
                end: chars[chars.len() - 1].end,
                value: match frame {
                    [addr, fc, ..] => (*addr as u32) << 8 | *fc as u32,
                    [addr] => (*addr as u32) << 8,
                    [] => 0,
                },
                lane: 1,
                kind: Kind::ModbusFrame,
                bits: if crc_ok { dir } else { 0 },
                flags: char_errors | crc_flag,
            });
            for f in fields {
                let cs = &chars[f.bytes.clone()];
                let errors = cs.iter().fold(0, |e, c| e | c.flags) & flag::ERROR;
                out.push(Annotation {
                    start: cs[0].start,
                    end: cs[cs.len() - 1].end,
                    value: f.value,
                    lane: 2,
                    kind: f.kind,
                    bits: 0,
                    flags: errors
                        | if f.kind == Kind::ModbusCrc {
                            crc_flag
                        } else {
                            0
                        },
                });
            }
        }
        self.chars.clear();
    }
}

impl ByteLayer for ModbusRtu {
    fn lanes(&self) -> &'static [&'static str] {
        &["RX", "Frame", "Fields"]
    }

    fn reset(&mut self, timing: CharTiming) {
        let t35 = if timing.baud() > 19_200.0 {
            1.75e-3 * timing.sample_rate
        } else {
            3.5 * timing.char_samples()
        };
        self.t35 = t35.round() as u64;
        self.chars.clear();
    }

    fn char(&mut self, c: &Annotation, out: &mut Vec<Annotation>) {
        if self
            .chars
            .last()
            .is_some_and(|l| c.start.saturating_sub(l.end) >= self.t35)
            || self.chars.len() >= MAX_FRAME
        {
            self.finish(out);
        }
        self.chars.push(*c);
    }

    fn idle(&mut self, until: u64, out: &mut Vec<Annotation>) {
        if self
            .chars
            .last()
            .is_some_and(|l| until.saturating_sub(l.end) >= self.t35)
        {
            self.finish(out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::decode_all;
    use crate::synth::Wave;
    use crate::uart::{Parity, Protocol, Uart, UartConfig};
    use crate::Decoder;
    use bf_core::{CaptureStore, ChannelCount, StoreConfig};

    const RATE: f64 = 4e6;

    /// Frames of a real 19200 8E1 bus (slave 0x65), with their CRCs.
    const WRITE_REQ: [u8; 8] = [0x65, 0x06, 0x01, 0x01, 0x00, 0x00, 0xD1, 0xD2];
    const READ_REQ: [u8; 8] = [0x65, 0x03, 0x04, 0x04, 0x00, 0x01, 0xCC, 0xDF];

    fn with_crc(body: &[u8]) -> Vec<u8> {
        let mut v = body.to_vec();
        v.extend_from_slice(&crc16(body).to_le_bytes());
        v
    }

    fn kinds(fields: &[Field]) -> Vec<(Kind, u32)> {
        fields.iter().map(|f| (f.kind, f.value)).collect()
    }

    #[test]
    fn crc_of_real_frames_is_zero_over_the_whole_frame() {
        assert_eq!(crc16(&WRITE_REQ), 0);
        assert_eq!(crc16(&READ_REQ), 0);
        assert_eq!(crc16(&WRITE_REQ[..6]).to_le_bytes(), [0xD1, 0xD2]);
        // The standard check value of CRC-16/MODBUS.
        assert_eq!(crc16(b"123456789"), 0x4B37);
    }

    #[test]
    fn read_holding_registers_request_and_response() {
        let (dir, f) = parse(&READ_REQ);
        assert_eq!(dir, REQUEST);
        assert_eq!(
            kinds(&f),
            vec![
                (Kind::ModbusAddress, 0x65),
                (Kind::ModbusFunction, 3),
                (Kind::ModbusRegister, 0x0404),
                (Kind::ModbusQuantity, 1),
                (Kind::ModbusCrc, 0xDFCC),
            ]
        );
        let resp = with_crc(&[0x65, 0x03, 0x04, 0x12, 0x34, 0xAB, 0xCD]);
        let (dir, f) = parse(&resp);
        assert_eq!(dir, RESPONSE);
        assert_eq!(
            kinds(&f)[2..5],
            [
                (Kind::ModbusByteCount, 4),
                (Kind::ModbusValue, 0x1234),
                (Kind::ModbusValue, 0xABCD)
            ]
        );
        assert_eq!(f[3].bytes, 3..5);
    }

    #[test]
    fn write_functions_and_exceptions() {
        let (dir, f) = parse(&WRITE_REQ);
        assert_eq!(dir, 0); // the response would look the same
        assert_eq!(
            kinds(&f)[2..4],
            [(Kind::ModbusRegister, 0x0101), (Kind::ModbusValue, 0)]
        );
        let req = with_crc(&[
            0x01, 0x10, 0x00, 0x10, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02,
        ]);
        let (dir, f) = parse(&req);
        assert_eq!(dir, REQUEST);
        assert_eq!(
            kinds(&f)[2..7],
            [
                (Kind::ModbusRegister, 0x10),
                (Kind::ModbusQuantity, 2),
                (Kind::ModbusByteCount, 4),
                (Kind::ModbusValue, 0x000A),
                (Kind::ModbusValue, 0x0102)
            ]
        );
        assert_eq!(
            parse(&with_crc(&[0x01, 0x10, 0x00, 0x10, 0x00, 0x02])).0,
            RESPONSE
        );
        let (dir, f) = parse(&with_crc(&[0x0A, 0x83, 0x02]));
        assert_eq!(dir, RESPONSE);
        assert_eq!(kinds(&f)[2], (Kind::ModbusException, 2));
    }

    #[test]
    fn unknown_functions_and_short_frames_still_show_their_bytes() {
        let (dir, f) = parse(&with_crc(&[0x01, 0x41, 0xAA, 0xBB]));
        assert_eq!(dir, 0);
        assert_eq!(
            kinds(&f)[2..4],
            [(Kind::ModbusData, 0xAA), (Kind::ModbusData, 0xBB)]
        );
        assert_eq!(parse(&[0x01, 0x03]).1.len(), 2);
    }

    #[test]
    fn back_to_back_frames_are_split_by_their_crcs() {
        let resp = with_crc(&[0x65, 0x03, 0x02, 0x00, 0x2A]);
        let mut block = READ_REQ.to_vec();
        block.extend_from_slice(&resp);
        assert_eq!(split(&block), vec![(0..8, true), (8..15, true)]);
        assert_eq!(split(&READ_REQ), vec![(0..8, true)]);
        // A corrupted frame stays one frame, marked bad.
        let mut bad = READ_REQ;
        bad[3] ^= 0x10;
        assert_eq!(split(&bad), vec![(0..8, false)]);
        // A good frame followed by garbage.
        let mut tail = READ_REQ.to_vec();
        tail.extend_from_slice(&[1, 2, 3]);
        assert_eq!(split(&tail), vec![(0..8, true), (8..11, false)]);
    }

    #[test]
    fn labels() {
        let a = |kind, value, bits, flags| Annotation {
            start: 0,
            end: 1,
            value,
            lane: 2,
            kind,
            bits,
            flags,
        };
        let hex = Format::Hex;
        assert_eq!(
            a(Kind::ModbusFrame, 0x6503, REQUEST, 0).label(hex),
            "Slave 65: Read Holding Registers request"
        );
        assert_eq!(
            a(Kind::ModbusFrame, 0x0A83, RESPONSE, 0).label(hex),
            "Slave 0A: Exception (Read Holding Registers) response"
        );
        assert_eq!(
            a(Kind::ModbusFrame, 0x6506, 0, flag::CHECKSUM).label(Format::Dec),
            "Slave 101: Write Single Register BAD CRC"
        );
        assert_eq!(
            a(Kind::ModbusFunction, 3, 0, 0).label(hex),
            "FC 3 Read Holding Registers"
        );
        assert_eq!(a(Kind::ModbusRegister, 0x0404, 0, 0).label(hex), "Reg 0404");
        assert_eq!(
            a(Kind::ModbusRegister, 0x0404, 0, 0).label(Format::Ascii),
            "Reg 0404"
        );
        assert_eq!(a(Kind::ModbusValue, 10, 0, 0).label(Format::Dec), "10");
        assert_eq!(
            a(Kind::ModbusException, 2, 0, 0).label(hex),
            "Exception 2: Illegal Data Address"
        );
        assert_eq!(a(Kind::ModbusCrc, 0xDFCC, 0, 0).label(hex), "CRC ok");
        assert_eq!(
            a(Kind::ModbusCrc, 0x1234, 0, flag::CHECKSUM).label(hex),
            "CRC bad (1234)"
        );
    }

    fn cfg(baud: f64) -> UartConfig {
        UartConfig {
            baud,
            parity: Parity::Even,
            protocol: Protocol::ModbusRtu,
            ..Default::default()
        }
    }

    fn decode(cfg: &UartConfig, w: &Wave) -> Vec<Annotation> {
        let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(8).unwrap(), RATE));
        s.push_samples(&w.samples);
        decode_all(Box::new(Uart::new(cfg.clone())), &s)
    }

    fn lane(a: &[Annotation], l: u8) -> Vec<Annotation> {
        a.iter().filter(|x| x.lane == l).copied().collect()
    }

    #[test]
    fn frames_on_the_wire_are_delimited_by_silence() {
        let c = cfg(19_200.0);
        let char_samples = RATE / 19_200.0 * 11.0;
        let mut w = Wave::new(RATE, 200_000, 1);
        let end = w.uart(0, 1_000, &c, &READ_REQ);
        // The answer 5 character times later: a new frame.
        let resp = with_crc(&[0x65, 0x03, 0x02, 0x00, 0x2A]);
        let end = w.uart(0, end + (5.0 * char_samples) as u64, &c, &resp);
        // Then a frame whose CRC is broken.
        let mut bad = WRITE_REQ;
        bad[5] = 0x01;
        w.uart(0, end + (10.0 * char_samples) as u64, &c, &bad);
        let a = decode(&c, &w);
        assert_eq!(lane(&a, 0).len(), 8 + 7 + 8);
        let frames = lane(&a, 1);
        assert_eq!(frames.len(), 3, "{frames:?}");
        assert_eq!(
            (frames[0].value, frames[0].bits, frames[0].flags),
            (0x6503, REQUEST, 0)
        );
        assert_eq!((frames[1].bits, frames[1].flags), (RESPONSE, 0));
        assert_eq!(frames[2].flags, flag::CHECKSUM);
        assert!(frames[2].is_error());
        let fields = lane(&a, 2);
        assert_eq!(fields.len(), 5 + 5 + 5);
        // Fields are sorted, do not overlap and span their characters.
        assert!(fields.windows(2).all(|p| p[0].end <= p[1].start));
        let chars = lane(&a, 0);
        assert_eq!(
            (fields[2].start, fields[2].end),
            (chars[2].start, chars[3].end)
        );
        assert_eq!(fields[14].flags, flag::CHECKSUM);
    }

    #[test]
    fn a_fast_answer_without_the_full_pause_is_split_by_crc() {
        let c = cfg(115_200.0);
        let mut w = Wave::new(RATE, 100_000, 1);
        let end = w.uart(0, 500, &c, &READ_REQ);
        // Only 2 character times of silence (the rule asks for 1.75 ms at this rate).
        let resp = with_crc(&[0x65, 0x03, 0x02, 0x00, 0x2A]);
        w.uart(0, end + (2.0 * RATE / 115_200.0 * 11.0) as u64, &c, &resp);
        let frames = lane(&decode(&c, &w), 1);
        assert_eq!(frames.len(), 2, "{frames:?}");
        assert!(frames.iter().all(|f| f.flags == 0));
        assert_eq!((frames[0].bits, frames[1].bits), (REQUEST, RESPONSE));
    }

    #[test]
    fn plain_uart_keeps_its_telegrams() {
        let c = UartConfig::default();
        assert_eq!(Uart::new(c.clone()).lanes(), &["RX", "Telegram"]);
        assert_eq!(Uart::new(cfg(9600.0)).lanes(), &["RX", "Frame", "Fields"]);
    }
}
