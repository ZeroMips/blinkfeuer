//! Detection of the UART frame format (data bits, parity, stop bits) at a known baud rate.
//!
//! Every common format is tried by decoding the channel with it; a format fits when (nearly) every
//! character decodes without a framing or parity error. Several formats can look the same on the wire,
//! so the choice among the fitting ones follows what can actually be told apart:
//!
//! * The shortest fitting frame wins. A longer frame only fits a shorter one through idle time after
//!   the characters (8N1 traffic with gaps also passes as 9N1 or 8N2), which is no evidence. For the
//!   same reason 8N2 traffic is reported as 8N1: an 8N1 receiver reads it correctly.
//! * Among frames of the same length a parity bit wins over one more data bit (7E1 over 8N1, 8E1 over
//!   9N1) when it checks out in every character and there are enough characters for that not to be
//!   chance (random data fails a parity check half of the time).
//! * Mark and space parity are a constant bit, the same as a stop bit or a data bit that never changes,
//!   and are not offered.
//!
//! A wrong baud rate can still decode a few characters cleanly by chance. Real UART edges inside a
//! character fall on bit boundaries, so a format only fits when (nearly) all of them do.

use crate::engine::decode_all;
use crate::uart::{Parity, Uart, UartConfig};
use crate::Kind;
use bf_core::CaptureStore;

/// Characters needed before parity that always checks out is taken as real (chance: 2^-16).
const PARITY_EVIDENCE: usize = 16;
/// Characters needed for any answer.
const MIN_CHARS: usize = 8;

/// The result of [`detect_format`].
#[derive(Clone, Debug, PartialEq)]
pub struct FormatGuess {
    /// Data bits per character.
    pub data_bits: u8,
    /// Parity bit.
    pub parity: Parity,
    /// Stop bits (always 1: 2 read the same way).
    pub stop_bits: f32,
    /// Characters decoded with this format, and how many of them had an error.
    pub chars: usize,
    /// See `chars`.
    pub errors: usize,
    /// What the result cannot rule out, e.g. 2 stop bits.
    pub note: Option<String>,
}

impl FormatGuess {
    /// The usual short form, e.g. "8E1".
    pub fn short(&self) -> String {
        let p = match self.parity {
            Parity::None => 'N',
            Parity::Even => 'E',
            Parity::Odd => 'O',
            Parity::Mark => 'M',
            Parity::Space => 'S',
        };
        format!("{}{p}{}", self.data_bits, self.stop_bits)
    }
}

/// Result of decoding with one candidate format.
#[derive(Clone, Copy, Debug)]
struct Trial {
    data_bits: u8,
    parity: Parity,
    chars: usize,
    framing: usize,
    parity_errors: usize,
    /// Edges inside the characters, and those not near a bit boundary.
    edges: usize,
    misaligned: usize,
}

impl Trial {
    fn frame_bits(&self) -> u32 {
        1 + self.data_bits as u32 + (self.parity != Parity::None) as u32 + 1
    }

    /// (Nearly) every character decodes cleanly: a stray error (a glitch, a character cut by the start
    /// or end of the capture) is allowed per 200 characters.
    fn fits(&self) -> bool {
        self.chars >= MIN_CHARS
            && (self.framing + self.parity_errors) * 200 <= self.chars
            && self.misaligned * 100 <= self.edges
    }
}

fn trial(store: &CaptureStore, base: &UartConfig, data_bits: u8, parity: Parity) -> Trial {
    let cfg = UartConfig {
        data_bits,
        parity,
        stop_bits: 1.0,
        telegrams: false,
        protocol: Default::default(),
        ..base.clone()
    };
    let anns = decode_all(Box::new(Uart::new(cfg)), store);
    let mut t = Trial {
        data_bits,
        parity,
        chars: 0,
        framing: 0,
        parity_errors: 0,
        edges: 0,
        misaligned: 0,
    };
    let bit = store.sample_rate() / base.baud.max(1.0);
    for a in anns.iter().filter(|a| a.lane == 0 && a.kind == Kind::Data) {
        t.chars += 1;
        t.framing += (a.flags & crate::flag::FRAMING != 0) as usize;
        t.parity_errors += (a.flags & crate::flag::PARITY != 0) as usize;
        store.toggles(base.rx, a.start + 1..a.end, |p, _| {
            let frac = ((p - a.start) as f64 / bit).fract();
            t.edges += 1;
            t.misaligned += (0.25..0.75).contains(&frac) as usize;
        });
    }
    t
}

/// Detect the frame format of the UART described by `base` (channel, baud rate, polarity and bit
/// order are taken from it; its frame format is ignored).
pub fn detect_format(store: &CaptureStore, base: &UartConfig) -> Result<FormatGuess, String> {
    let mut trials = Vec::new();
    for data_bits in [5u8, 6, 7, 8, 9] {
        for parity in [Parity::None, Parity::Even, Parity::Odd] {
            trials.push(trial(store, base, data_bits, parity));
        }
    }
    let most = trials.iter().map(|t| t.chars).max().unwrap_or(0);
    if most < MIN_CHARS {
        return Err(format!(
            "only {most} characters on this channel at {} baud: capture more traffic (or check the baud rate)",
            base.baud
        ));
    }
    let fitting: Vec<Trial> = trials.iter().copied().filter(Trial::fits).collect();
    let Some(shortest) = fitting.iter().map(|t| t.frame_bits()).min() else {
        return Err(
            "no frame format decodes without errors: check the baud rate and the polarity".into(),
        );
    };
    let same_length: Vec<Trial> = fitting
        .into_iter()
        .filter(|t| t.frame_bits() == shortest)
        .collect();
    // A parity bit that checks out in enough characters beats one more data bit.
    let with_parity = same_length
        .iter()
        .filter(|t| t.parity != Parity::None && t.chars >= PARITY_EVIDENCE)
        .min_by_key(|t| t.parity_errors)
        .copied();
    let plain = same_length
        .iter()
        .find(|t| t.parity == Parity::None)
        .copied();
    let (best, mut notes) = match (with_parity, plain) {
        (Some(p), _) => (p, Vec::new()),
        (None, Some(n)) => {
            let mut notes = Vec::new();
            if same_length
                .iter()
                .any(|t| t.parity != Parity::None && t.chars < PARITY_EVIDENCE)
            {
                notes.push(format!(
                    "too few characters to tell {} data bits from {} with parity",
                    n.data_bits,
                    n.data_bits - 1
                ));
            }
            (n, notes)
        }
        (None, None) => (same_length[0], Vec::new()),
    };
    // The format one stop bit longer would read the same characters: 2 stop bits cannot be ruled out
    // when every character is followed by idle line.
    notes.push("2 stop bits read the same way".into());
    let guess = FormatGuess {
        data_bits: best.data_bits,
        parity: best.parity,
        stop_bits: 1.0,
        chars: best.chars,
        errors: best.framing + best.parity_errors,
        note: Some(notes.join("; ")),
    };
    Ok(guess)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth::Wave;
    use bf_core::{ChannelCount, StoreConfig};

    const RATE: f64 = 2e6;

    fn store(w: &Wave) -> CaptureStore {
        let mut s = CaptureStore::new(StoreConfig::new(ChannelCount::new(8).unwrap(), RATE));
        s.push_samples(&w.samples);
        s
    }

    /// `n` characters in `fmt`, in bursts: back-to-back within a burst, idle between bursts.
    fn traffic(fmt: &UartConfig, data: &[u8]) -> CaptureStore {
        let mut w = Wave::new(RATE, 2_000_000, if fmt.invert { 0 } else { 1 });
        let mut t = 1_000;
        for burst in data.chunks(6) {
            t = w.uart(0, t, fmt, burst) + 3_000;
        }
        store(&w)
    }

    fn fmt(data_bits: u8, parity: Parity, stop_bits: f32) -> UartConfig {
        UartConfig {
            baud: 19_200.0,
            data_bits,
            parity,
            stop_bits,
            ..Default::default()
        }
    }

    /// Bytes masked to `bits`, varied enough that wrong guesses produce errors.
    fn data(bits: u8, n: usize) -> Vec<u8> {
        let mask = ((1u32 << bits.min(8)) - 1) as u8;
        (0..n as u32)
            .map(|i| ((i * 73 + 41) as u8 ^ (i >> 3) as u8) & mask)
            .collect()
    }

    fn detect(sent: &UartConfig, payload: &[u8]) -> FormatGuess {
        // The detector only knows the channel and the baud rate.
        let base = UartConfig {
            baud: sent.baud,
            invert: sent.invert,
            ..Default::default()
        };
        detect_format(&traffic(sent, payload), &base).unwrap()
    }

    #[test]
    fn common_formats_are_recognised() {
        for (bits, parity) in [
            (8, Parity::None),
            (8, Parity::Even),
            (8, Parity::Odd),
            (7, Parity::Even),
            (7, Parity::Odd),
        ] {
            let sent = fmt(bits, parity, 1.0);
            let g = detect(&sent, &data(bits, 60));
            assert_eq!((g.data_bits, g.parity), (bits, parity), "{sent:?}: {g:?}");
            assert_eq!(g.errors, 0);
            assert_eq!(g.chars, 60);
        }
    }

    #[test]
    fn two_stop_bits_read_as_one() {
        let g = detect(&fmt(8, Parity::None, 2.0), &data(8, 60));
        assert_eq!(g.short(), "8N1");
        assert!(g.note.unwrap().contains("2 stop bits"));
    }

    #[test]
    fn seven_bit_text_in_8n1_stays_8n1() {
        // ASCII text: the 8th bit is always 0, the parity of the 7 bits varies.
        let text = b"The quick brown fox jumps over the lazy dog 0123456789";
        let g = detect(&fmt(8, Parity::None, 1.0), text);
        assert_eq!(g.short(), "8N1");
    }

    #[test]
    fn inverted_lines_work_with_the_polarity_given() {
        let sent = UartConfig {
            invert: true,
            ..fmt(8, Parity::Even, 1.0)
        };
        assert_eq!(detect(&sent, &data(8, 40)).short(), "8E1");
    }

    #[test]
    fn too_little_traffic_or_a_wrong_baud_rate_is_reported() {
        let sent = fmt(8, Parity::None, 1.0);
        let s = traffic(&sent, &data(8, 3));
        assert!(detect_format(&s, &sent).unwrap_err().contains("only"));
        let s = traffic(&sent, &data(8, 60));
        let wrong = UartConfig {
            baud: 4_800.0,
            ..sent
        };
        let r = detect_format(&s, &wrong);
        assert!(r.is_err(), "{r:?}");
    }
}
