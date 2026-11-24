//! Wire sample format of the SLogic analyzers: sample-major packed samples, LSB = CH0.
//!
//! | channels | packing                                              |
//! |----------|------------------------------------------------------|
//! | 2        | 4 samples per byte, first sample in bits 1:0         |
//! | 4        | 2 samples per byte, first sample in the low nibble   |
//! | 8        | 1 byte per sample                                    |
//! | 16       | 2 bytes per sample, little endian                    |
//! | 32       | 4 bytes per sample, little endian                    |

/// Number of captured channels (a power of two between 2 and 32).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ChannelCount(u8);

impl ChannelCount {
    /// A channel count; `None` unless it is 2, 4, 8, 16 or 32.
    pub fn new(n: u8) -> Option<Self> {
        matches!(n, 2 | 4 | 8 | 16 | 32).then_some(Self(n))
    }

    /// The number of channels.
    pub fn get(self) -> u8 {
        self.0
    }

    /// Bit mask of the valid channels in a sample word.
    pub fn mask(self) -> u32 {
        if self.0 == 32 {
            u32::MAX
        } else {
            (1u32 << self.0) - 1
        }
    }

    /// Bits used per sample on the wire.
    pub fn bits_per_sample(self) -> u32 {
        self.0 as u32
    }

    /// Wire bytes needed for `samples` samples (rounded up).
    pub fn bytes_for(self, samples: u64) -> u64 {
        (samples * self.bits_per_sample() as u64).div_ceil(8)
    }

    /// Samples that `bytes` wire bytes carry (rounded down).
    pub fn samples_in(self, bytes: u64) -> u64 {
        bytes * 8 / self.bits_per_sample() as u64
    }
}

/// Incremental unpacker: turns wire bytes (arbitrary slicing, e.g. USB transfers that end in the
/// middle of a 16-bit sample) into sample words.
#[derive(Clone, Debug)]
pub struct Unpacker {
    channels: ChannelCount,
    carry: [u8; 4],
    carry_len: usize,
}

impl Unpacker {
    /// An unpacker for a stream of `channels` channels.
    pub fn new(channels: ChannelCount) -> Self {
        Self {
            channels,
            carry: [0; 4],
            carry_len: 0,
        }
    }

    /// The channel count it unpacks.
    pub fn channels(&self) -> ChannelCount {
        self.channels
    }

    /// Forget a partially received sample (start of a new capture).
    pub fn reset(&mut self) {
        self.carry_len = 0;
    }

    /// Append the samples contained in `data` to `out`. Returns the number of samples appended.
    pub fn unpack(&mut self, data: &[u8], out: &mut Vec<u32>) -> usize {
        let before = out.len();
        match self.channels.get() {
            2 => {
                out.reserve(data.len() * 4);
                for &b in data {
                    out.extend_from_slice(&[
                        (b & 3) as u32,
                        ((b >> 2) & 3) as u32,
                        ((b >> 4) & 3) as u32,
                        ((b >> 6) & 3) as u32,
                    ]);
                }
            }
            4 => {
                out.reserve(data.len() * 2);
                for &b in data {
                    out.push((b & 0xf) as u32);
                    out.push((b >> 4) as u32);
                }
            }
            8 => out.extend(data.iter().map(|&b| b as u32)),
            n => {
                let width = (n / 8) as usize; // 2 or 4 bytes
                let mut data = data;
                // Finish a sample split over two calls.
                if self.carry_len > 0 {
                    let need = width - self.carry_len;
                    let take = need.min(data.len());
                    self.carry[self.carry_len..self.carry_len + take]
                        .copy_from_slice(&data[..take]);
                    self.carry_len += take;
                    data = &data[take..];
                    if self.carry_len == width {
                        out.push(le_word(&self.carry[..width]));
                        self.carry_len = 0;
                    }
                }
                let whole = data.len() / width * width;
                out.reserve(whole / width);
                out.extend(data[..whole].chunks_exact(width).map(le_word));
                let rest = &data[whole..];
                self.carry[..rest.len()].copy_from_slice(rest);
                self.carry_len += rest.len();
            }
        }
        out.len() - before
    }
}

/// Samples per unit that `pack` needs to produce whole bytes (4 for 2 channels, 2 for 4, else 1).
pub fn pack_alignment(channels: ChannelCount) -> usize {
    match channels.get() {
        2 => 4,
        4 => 2,
        _ => 1,
    }
}

/// Inverse of [`Unpacker::unpack`]: append the wire bytes for `samples` to `out`.
/// `samples.len()` must be a multiple of [`pack_alignment`].
pub fn pack(channels: ChannelCount, samples: &[u32], out: &mut Vec<u8>) {
    assert_eq!(
        samples.len() % pack_alignment(channels),
        0,
        "unaligned sample count"
    );
    match channels.get() {
        2 => out.extend(samples.as_chunks::<4>().0.iter().map(|s| {
            (s[0] & 3) as u8
                | ((s[1] & 3) as u8) << 2
                | ((s[2] & 3) as u8) << 4
                | ((s[3] & 3) as u8) << 6
        })),
        4 => out.extend(
            samples
                .as_chunks::<2>()
                .0
                .iter()
                .map(|s| (s[0] & 0xf) as u8 | ((s[1] & 0xf) as u8) << 4),
        ),
        8 => out.extend(samples.iter().map(|&s| s as u8)),
        16 => {
            for &s in samples {
                out.extend_from_slice(&(s as u16).to_le_bytes());
            }
        }
        _ => {
            for &s in samples {
                out.extend_from_slice(&s.to_le_bytes());
            }
        }
    }
}

fn le_word(b: &[u8]) -> u32 {
    let mut w = 0u32;
    for (i, &x) in b.iter().enumerate() {
        w |= (x as u32) << (8 * i);
    }
    w
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cc(n: u8) -> ChannelCount {
        ChannelCount::new(n).unwrap()
    }

    #[test]
    fn four_channels_first_sample_in_low_nibble() {
        let mut u = Unpacker::new(cc(4));
        let mut out = Vec::new();
        u.unpack(&[0x21, 0x43], &mut out);
        assert_eq!(out, [1, 2, 3, 4]);
    }

    #[test]
    fn two_channels_four_samples_per_byte() {
        let mut u = Unpacker::new(cc(2));
        let mut out = Vec::new();
        u.unpack(&[0b11_10_01_00], &mut out);
        assert_eq!(out, [0, 1, 2, 3]);
    }

    #[test]
    fn eight_channels_one_byte_per_sample() {
        let mut u = Unpacker::new(cc(8));
        let mut out = Vec::new();
        u.unpack(&[0x00, 0xff, 0xa5], &mut out);
        assert_eq!(out, [0x00, 0xff, 0xa5]);
    }

    #[test]
    fn sixteen_channels_little_endian() {
        let mut u = Unpacker::new(cc(16));
        let mut out = Vec::new();
        u.unpack(&[0x34, 0x12, 0xcd, 0xab], &mut out);
        assert_eq!(out, [0x1234, 0xabcd]);
    }

    #[test]
    fn thirty_two_channels_little_endian() {
        let mut u = Unpacker::new(cc(32));
        let mut out = Vec::new();
        u.unpack(&[0x78, 0x56, 0x34, 0x12], &mut out);
        assert_eq!(out, [0x1234_5678]);
    }

    #[test]
    fn split_transfers_do_not_lose_or_shift_samples() {
        // 16 and 32 channels: transfers may end in the middle of a sample.
        for (n, width) in [(16u8, 2usize), (32, 4)] {
            let data: Vec<u8> = (0..(width * 11) as u8).collect();
            let mut whole = Vec::new();
            Unpacker::new(cc(n)).unpack(&data, &mut whole);
            for cut in 0..=data.len() {
                for cut2 in cut..=data.len() {
                    let mut u = Unpacker::new(cc(n));
                    let mut out = Vec::new();
                    u.unpack(&data[..cut], &mut out);
                    u.unpack(&data[cut..cut2], &mut out);
                    u.unpack(&data[cut2..], &mut out);
                    assert_eq!(out, whole, "n={n} cut={cut} cut2={cut2}");
                }
            }
        }
    }

    #[test]
    fn byte_accounting() {
        assert_eq!(cc(4).bytes_for(3), 2);
        assert_eq!(cc(16).bytes_for(3), 6);
        assert_eq!(cc(4).samples_in(5), 10);
        assert_eq!(cc(16).samples_in(5), 2);
        assert_eq!(cc(16).mask(), 0xffff);
        assert_eq!(cc(32).mask(), u32::MAX);
        assert!(ChannelCount::new(3).is_none());
    }

    #[test]
    fn pack_is_the_inverse_of_unpack() {
        for n in [2u8, 4, 8, 16, 32] {
            let ch = cc(n);
            let align = pack_alignment(ch);
            let samples: Vec<u32> = (0..align as u32 * 25)
                .map(|i| i.wrapping_mul(2654435761) & ch.mask())
                .collect();
            let mut bytes = Vec::new();
            pack(ch, &samples, &mut bytes);
            assert_eq!(bytes.len() as u64, ch.bytes_for(samples.len() as u64));
            let mut back = Vec::new();
            Unpacker::new(ch).unpack(&bytes, &mut back);
            assert_eq!(back, samples, "channels {n}");
        }
    }
}
