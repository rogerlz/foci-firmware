//! Absolute encoders fed into the TMC4671 ABN decoder.
//!
//! A board that reads an absolute encoder itself (serial, SPI, ...) writes
//! the position into `ABN_DECODER_COUNT` every reading. The TMC4671 then
//! derives both the electrical angle and the mechanical position from the
//! count, so commissioning and the motion loops treat the encoder as an
//! ABN encoder with `encoder_ppr` lines, and `PHI_E_EXT` stays free for the
//! commanded angle commissioning uses.
//!
//! [`CountFeed`] turns readings into counts. It follows the firmware's own
//! writes of `ABN_DECODER_PPR` (the count range) and `ABN_DECODER_COUNT`
//! (the encoder alignment's zero), so the board only has to pass every
//! `tmc_control` register write to [`CountFeed::observe_write`].

use foci_tmc4671::registers::{ABN_DECODER_COUNT, ABN_DECODER_PPR};

/// Converts absolute encoder readings into `ABN_DECODER_COUNT` values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CountFeed {
    /// Quadrature counts per revolution from the last `ABN_DECODER_PPR`
    /// write; zero until the decoder is configured.
    counts_per_rev: u32,
    /// Reading (in `counts_per_rev` units) that maps to count 0.
    zero: u32,
    /// Last reading, in `counts_per_rev` units.
    position: Option<u32>,
}

impl CountFeed {
    pub const fn new() -> Self {
        Self {
            counts_per_rev: 0,
            zero: 0,
            position: None,
        }
    }

    /// Scale a `position_bits`-wide single-turn reading to the configured
    /// count range and return the `ABN_DECODER_COUNT` to write, or `None`
    /// until the firmware has written `ABN_DECODER_PPR`.
    pub fn count(&mut self, position: u32, position_bits: u32) -> Option<u32> {
        if self.counts_per_rev == 0 {
            return None;
        }
        let scaled =
            ((u64::from(position) * u64::from(self.counts_per_rev)) >> position_bits) as u32;
        self.position = Some(scaled);
        Some(
            scaled
                .wrapping_add(self.counts_per_rev)
                .wrapping_sub(self.zero)
                % self.counts_per_rev,
        )
    }

    /// Follow a register write sent to the TMC4671: `ABN_DECODER_PPR` sets
    /// the count range, and `ABN_DECODER_COUNT` moves the zero so the
    /// current reading keeps mapping to the written count.
    pub fn observe_write(&mut self, addr: u8, value: u32) {
        if addr == ABN_DECODER_PPR.addr() {
            self.counts_per_rev = value;
            self.zero = 0;
        } else if addr == ABN_DECODER_COUNT.addr()
            && let Some(position) = self.position
            && self.counts_per_rev != 0
        {
            self.zero = position
                .wrapping_add(self.counts_per_rev)
                .wrapping_sub(value % self.counts_per_rev)
                % self.counts_per_rev;
        }
    }
}

impl Default for CountFeed {
    fn default() -> Self {
        Self::new()
    }
}

/// Mitsubishi HF-KP (MR-J3 series) serial encoder: 2.5 Mbaud RS-485,
/// half duplex, 18-bit single-turn position.
pub mod hfkp {
    pub const BAUD: u32 = 2_500_000;
    /// Request byte for the position frame.
    pub const REQUEST_POSITION: u8 = 0x32;
    /// Bytes received per request with the transceiver's receiver left on:
    /// the request echo followed by the 9-byte reply.
    pub const FRAME_LEN: usize = 10;
    pub const POSITION_BITS: u32 = 18;

    /// The 18-bit position from a received frame, or `None` when the echo
    /// is wrong or the reply's XOR checksum fails.
    pub fn decode_position(frame: &[u8; FRAME_LEN]) -> Option<u32> {
        if frame[0] != REQUEST_POSITION || frame[1..].iter().fold(0, |x, b| x ^ b) != 0 {
            return None;
        }
        Some(u32::from((frame[3] >> 6) & 3) | u32::from(frame[4]) << 2 | u32::from(frame[5]) << 10)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn configured(counts_per_rev: u32) -> CountFeed {
        let mut feed = CountFeed::new();
        feed.observe_write(ABN_DECODER_PPR.addr(), counts_per_rev);
        feed
    }

    #[test]
    fn feed_writes_nothing_until_the_decoder_is_configured() {
        assert_eq!(CountFeed::new().count(1_000, 18), None);
    }

    #[test]
    fn feed_scales_the_reading_to_the_count_range() {
        assert_eq!(configured(262_144).count(100_000, 18), Some(100_000));
        assert_eq!(configured(65_536).count(100_000, 18), Some(25_000));
    }

    #[test]
    fn alignment_write_moves_the_zero_to_the_current_reading() {
        let mut feed = configured(262_144);
        assert_eq!(feed.count(100_000, 18), Some(100_000));
        feed.observe_write(ABN_DECODER_COUNT.addr(), 0);
        assert_eq!(feed.count(100_000, 18), Some(0));
        assert_eq!(feed.count(99_990, 18), Some(262_144 - 10));
        feed.observe_write(ABN_DECODER_COUNT.addr(), 50);
        assert_eq!(feed.count(99_990, 18), Some(50));
    }

    #[test]
    fn a_new_count_range_resets_the_zero() {
        let mut feed = configured(262_144);
        feed.count(100_000, 18);
        feed.observe_write(ABN_DECODER_COUNT.addr(), 0);
        feed.observe_write(ABN_DECODER_PPR.addr(), 262_144);
        assert_eq!(feed.count(100_000, 18), Some(100_000));
    }

    fn hfkp_frame(position: u32) -> [u8; hfkp::FRAME_LEN] {
        let mut frame = [0u8; hfkp::FRAME_LEN];
        frame[0] = hfkp::REQUEST_POSITION;
        frame[1] = hfkp::REQUEST_POSITION;
        frame[3] = ((position & 3) << 6) as u8;
        frame[4] = (position >> 2) as u8;
        frame[5] = (position >> 10) as u8;
        frame[9] = frame[1..9].iter().fold(0, |x, b| x ^ b);
        frame
    }

    #[test]
    fn hfkp_decodes_the_18_bit_position() {
        assert_eq!(hfkp::decode_position(&hfkp_frame(0x2_ABCD)), Some(0x2_ABCD));
    }

    #[test]
    fn hfkp_rejects_a_bad_checksum_or_echo() {
        let mut frame = hfkp_frame(1_234);
        frame[4] ^= 1;
        assert_eq!(hfkp::decode_position(&frame), None);
        let mut frame = hfkp_frame(1_234);
        frame[0] = 0;
        assert_eq!(hfkp::decode_position(&frame), None);
    }
}
