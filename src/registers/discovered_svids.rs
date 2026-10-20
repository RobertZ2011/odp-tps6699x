//! Discovered SVIDs register (`0x21`).
//!
//! This register contains the SVID information returned from `Discover SVIDs REQ` messages.

use embedded_usb_pd::vdm::structured::Svid;

/// The address of the `Discovered SVIDs` register.
pub const ADDR: u8 = 0x21;

/// The length of the `Discovered SVIDs` register, in bytes.
pub const LEN: usize = 264 / 8;

/// High-level representation of the Discovered SVIDs register (`0x21`), containing the SVID
/// information returned from `Discover SVIDs REQ` messages.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DiscoveredSvids {
    /// Number of SVIDs discovered on SOP.
    pub number_sop_svids: u8,
    /// Number of SVIDs discovered on SOP'.
    pub number_sop_prime_svids: u8,
    /// SVIDs supported by the SOP port partner.
    pub svid_sop: [Svid; 8],
    /// SVIDs supported by the SOP' cable plug.
    pub svid_sop_prime: [Svid; 8],
}

impl DiscoveredSvids {
    pub const DEFAULT: Self = Self {
        number_sop_svids: 0,
        number_sop_prime_svids: 0,
        svid_sop: [Svid(0); 8],
        svid_sop_prime: [Svid(0); 8],
    };

    /// Returns an iterator over the valid SVIDs discovered on the SOP port partner.
    ///
    /// This will return at most 8 SVIDs, even if [`Self::number_sop_svids`] is
    /// a larger number. Neither the data sheet nor the USB PD specification explicitly
    /// state that there can be at most 8 SVIDs, but the register only has space
    /// for 8.
    pub fn svid_sop_iter(&self) -> impl ExactSizeIterator<Item = Svid> {
        self.svid_sop.into_iter().take(self.number_sop_svids as usize)
    }

    /// Returns an iterator over the valid SVIDs discovered on the SOP' cable plug.
    ///
    /// This will return at most 8 SVIDs, even if [`Self::number_sop_prime_svids`]
    /// is a larger number. Neither the data sheet nor the USB PD specification
    /// explicitly state that there can be at most 8 SVIDs, but the register only
    /// has space for 8.
    pub fn svid_sop_prime_iter(&self) -> impl ExactSizeIterator<Item = Svid> {
        self.svid_sop_prime
            .into_iter()
            .take(self.number_sop_prime_svids as usize)
    }
}

impl From<crate::registers::DiscoveredSvids> for DiscoveredSvids {
    fn from(raw: crate::registers::DiscoveredSvids) -> Self {
        Self {
            number_sop_svids: raw.number_sop_svids(),
            number_sop_prime_svids: raw.number_sop_prime_svids(),
            svid_sop: [
                Svid(raw.svid_sop_0()),
                Svid(raw.svid_sop_1()),
                Svid(raw.svid_sop_2()),
                Svid(raw.svid_sop_3()),
                Svid(raw.svid_sop_4()),
                Svid(raw.svid_sop_5()),
                Svid(raw.svid_sop_6()),
                Svid(raw.svid_sop_7()),
            ],
            svid_sop_prime: [
                Svid(raw.svid_sop_prime_0()),
                Svid(raw.svid_sop_prime_1()),
                Svid(raw.svid_sop_prime_2()),
                Svid(raw.svid_sop_prime_3()),
                Svid(raw.svid_sop_prime_4()),
                Svid(raw.svid_sop_prime_5()),
                Svid(raw.svid_sop_prime_6()),
                Svid(raw.svid_sop_prime_7()),
            ],
        }
    }
}

impl From<DiscoveredSvids> for crate::registers::DiscoveredSvids {
    fn from(svids: DiscoveredSvids) -> Self {
        let mut raw = Self::default();
        raw.set_number_sop_svids(svids.number_sop_svids);
        raw.set_number_sop_prime_svids(svids.number_sop_prime_svids);
        raw.set_svid_sop_0(svids.svid_sop[0].0);
        raw.set_svid_sop_1(svids.svid_sop[1].0);
        raw.set_svid_sop_2(svids.svid_sop[2].0);
        raw.set_svid_sop_3(svids.svid_sop[3].0);
        raw.set_svid_sop_4(svids.svid_sop[4].0);
        raw.set_svid_sop_5(svids.svid_sop[5].0);
        raw.set_svid_sop_6(svids.svid_sop[6].0);
        raw.set_svid_sop_7(svids.svid_sop[7].0);
        raw.set_svid_sop_prime_0(svids.svid_sop_prime[0].0);
        raw.set_svid_sop_prime_1(svids.svid_sop_prime[1].0);
        raw.set_svid_sop_prime_2(svids.svid_sop_prime[2].0);
        raw.set_svid_sop_prime_3(svids.svid_sop_prime[3].0);
        raw.set_svid_sop_prime_4(svids.svid_sop_prime[4].0);
        raw.set_svid_sop_prime_5(svids.svid_sop_prime[5].0);
        raw.set_svid_sop_prime_6(svids.svid_sop_prime[6].0);
        raw.set_svid_sop_prime_7(svids.svid_sop_prime[7].0);
        raw
    }
}

impl From<[u8; LEN]> for DiscoveredSvids {
    fn from(raw: [u8; LEN]) -> Self {
        crate::registers::DiscoveredSvids::from(raw).into()
    }
}

impl From<DiscoveredSvids> for [u8; LEN] {
    fn from(svids: DiscoveredSvids) -> Self {
        crate::registers::DiscoveredSvids::from(svids).into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use std::vec;
    use std::vec::Vec;

    #[test]
    fn impl_iterators() {
        let mut reg = DiscoveredSvids::default();

        reg.number_sop_svids = 3;
        reg.svid_sop[0] = Svid(0x1234);
        reg.svid_sop[1] = Svid(0x5678);
        reg.svid_sop[2] = Svid(0x9ABC);

        reg.number_sop_prime_svids = 2;
        reg.svid_sop_prime[0] = Svid(0xDEF0);
        reg.svid_sop_prime[1] = Svid(0xFFFF);

        assert_eq!(reg.number_sop_svids, 3);
        assert_eq!(
            reg.svid_sop_iter().collect::<Vec<_>>(),
            vec![Svid(0x1234), Svid(0x5678), Svid(0x9ABC)]
        );

        assert_eq!(reg.number_sop_prime_svids, 2);
        assert_eq!(
            reg.svid_sop_prime_iter().collect::<Vec<_>>(),
            vec![Svid(0xDEF0), Svid(0xFFFF)]
        );
    }

    #[test]
    fn default_has_no_svids() {
        let reg = DiscoveredSvids::default();
        assert_eq!(reg.number_sop_svids, 0);
        assert_eq!(reg.svid_sop_iter().len(), 0);

        assert_eq!(reg.number_sop_prime_svids, 0);
        assert_eq!(reg.svid_sop_prime_iter().len(), 0);
    }

    /// There's only space for 8 SVIDs in the register but the number of SVIDs could
    /// be larger, so the iterators should clamp to 8.
    #[test]
    fn iterators_clamp_to_8() {
        let mut reg = DiscoveredSvids::default();

        reg.number_sop_svids = 9;
        reg.number_sop_prime_svids = 10;
        assert_eq!(reg.number_sop_svids, 9);
        assert_eq!(reg.number_sop_prime_svids, 10);
        assert_eq!(reg.svid_sop_iter().len(), 8);
        assert_eq!(reg.svid_sop_prime_iter().len(), 8);
    }

    #[test]
    fn test_discovered_svids_nonzero_roundtrip() {
        let mut reg = DiscoveredSvids::default();

        // Set header counts and SVIDs
        reg.number_sop_svids = 4;
        reg.number_sop_prime_svids = 3;
        reg.svid_sop[0] = Svid(0xFF01);
        reg.svid_sop[1] = Svid(0xABCD);
        reg.svid_sop[2] = Svid(0x5678);
        reg.svid_sop[3] = Svid(0x9A00);
        reg.svid_sop_prime[0] = Svid(0x1234);
        reg.svid_sop_prime[1] = Svid(0xDEF0);
        reg.svid_sop_prime[2] = Svid(0x00FF);

        // sop_count=4, sop_prime_count=3, followed by sop/sop' values
        const EXPECTED: [u8; LEN] = [
            0x34, 0x01, 0xFF, 0xCD, 0xAB, 0x78, 0x56, 0x00, 0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34,
            0x12, 0xF0, 0xDE, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ];
        assert_eq!(<[u8; LEN]>::from(reg), EXPECTED);

        // Reconstruct from raw and verify fields and iterators
        let raw: crate::registers::DiscoveredSvids = reg.into();
        let reg2 = DiscoveredSvids::from(raw);
        assert_eq!(reg2, reg);

        let sop: Vec<Svid> = reg2.svid_sop_iter().collect();
        assert_eq!(sop, vec![Svid(0xFF01), Svid(0xABCD), Svid(0x5678), Svid(0x9A00)]);

        let sop_prime: Vec<Svid> = reg2.svid_sop_prime_iter().collect();
        assert_eq!(sop_prime, vec![Svid(0x1234), Svid(0xDEF0), Svid(0x00FF)]);
    }
}
