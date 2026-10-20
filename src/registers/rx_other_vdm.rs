//! Rx Other Vdm register (`0x61`).
//!
//! This register receives the user svid other vdm.
//!

/// The address of the `Rx Other Vdm` register.
pub const ADDR: u8 = 0x61;

/// The length of the `Rx Other Vdm` register, in bytes.
pub const LEN: usize = 29;

/// The maximum number of valid VDOs that can be received in the `Rx Other Vdm` register.
pub const MAX_VDO_COUNT: usize = 7;

/// The Received User SVID Other VDM register (`0x61`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RxOtherVdm {
    /// Number of valid VDOs.
    pub num_of_valid_vdos: u8,
    /// Frame Type of the message in the register.
    pub sop_type: u8,
    /// Sequence Number.
    pub sequence_number: u8,
    /// Rx VDM data objects (up to 7).
    pub vdm_data: [u32; MAX_VDO_COUNT],
}

impl RxOtherVdm {
    pub fn is_valid_other_vdm(&self) -> bool {
        self.num_of_valid_vdos > 0 && (self.num_of_valid_vdos as usize) <= MAX_VDO_COUNT
    }
}

impl From<crate::registers::RxOtherVdm> for RxOtherVdm {
    fn from(raw: crate::registers::RxOtherVdm) -> Self {
        Self {
            num_of_valid_vdos: raw.num_of_valid_vdos(),
            sop_type: raw.sop_type(),
            sequence_number: raw.sequence_number(),
            vdm_data: [
                raw.vdo_1(),
                raw.vdo_2(),
                raw.vdo_3(),
                raw.vdo_4(),
                raw.vdo_5(),
                raw.vdo_6(),
                raw.vdo_7(),
            ],
        }
    }
}

impl From<RxOtherVdm> for crate::registers::RxOtherVdm {
    fn from(vdm: RxOtherVdm) -> Self {
        let mut raw = Self::default();
        raw.set_num_of_valid_vdos(vdm.num_of_valid_vdos);
        raw.set_sop_type(vdm.sop_type);
        raw.set_sequence_number(vdm.sequence_number);
        raw.set_vdo_1(vdm.vdm_data[0]);
        raw.set_vdo_2(vdm.vdm_data[1]);
        raw.set_vdo_3(vdm.vdm_data[2]);
        raw.set_vdo_4(vdm.vdm_data[3]);
        raw.set_vdo_5(vdm.vdm_data[4]);
        raw.set_vdo_6(vdm.vdm_data[5]);
        raw.set_vdo_7(vdm.vdm_data[6]);
        raw
    }
}

impl From<[u8; LEN]> for RxOtherVdm {
    fn from(value: [u8; LEN]) -> Self {
        crate::registers::RxOtherVdm::from(value).into()
    }
}

impl From<RxOtherVdm> for [u8; LEN] {
    fn from(value: RxOtherVdm) -> Self {
        crate::registers::RxOtherVdm::from(value).into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rx_other_vdm_nonzero() {
        let vdm = RxOtherVdm {
            num_of_valid_vdos: 7,
            sop_type: 2,
            sequence_number: 6,
            vdm_data: [
                0xDEADBEEF, 0x11223344, 0x55667788, 0x9ABCDEF0, 0x13579BDF, 0x2468ACE0, 0xFEDCBA98,
            ],
        };

        // Convert to bytes and assert against expected
        // Byte 0: num_of_valid_vdos(2:0)=7, sop_type(4:3)=2, sequence_number(7:5)=6
        const EXPECTED: [u8; LEN] = [
            0xD7, 0xEF, 0xBE, 0xAD, 0xDE, 0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 0xF0, 0xDE, 0xBC, 0x9A, 0xDF,
            0x9B, 0x57, 0x13, 0xE0, 0xAC, 0x68, 0x24, 0x98, 0xBA, 0xDC, 0xFE,
        ];
        assert_eq!(<[u8; LEN]>::from(vdm), EXPECTED);

        // Reconstruct from expected bytes and verify fields
        let vdm2 = RxOtherVdm::from(EXPECTED);
        assert!(vdm2.is_valid_other_vdm());
        assert_eq!(vdm2, vdm);
    }
}
