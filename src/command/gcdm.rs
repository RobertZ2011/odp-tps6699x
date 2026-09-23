//! Get custom discovered modes command
use bytemuck::{Pod, Zeroable};
use embedded_usb_pd::vdm::structured::Svid;
use pack1::{U16LE, U32LE};

/// Input data length
pub const INPUT_LEN: usize = 3;

/// GCdm input
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Input {
    pub svid: Svid,
}

/// Raw wire format of [`Input`]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Pod, Zeroable)]
#[repr(C)]
pub struct InputRaw {
    _reserved: u8,
    pub svid: U16LE,
}

impl From<Input> for InputRaw {
    fn from(value: Input) -> Self {
        Self {
            _reserved: 0,
            svid: value.svid.0.into(),
        }
    }
}

impl From<InputRaw> for Input {
    fn from(value: InputRaw) -> Self {
        Self {
            svid: Svid(value.svid.get()),
        }
    }
}

impl From<Svid> for Input {
    fn from(svid: Svid) -> Self {
        Self { svid }
    }
}

/// Representation of a custom discovered mode
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DiscoveredMode {
    /// Raw VDO data
    pub vdo: u32,
    /// VDO object position
    pub position: u8,
}

/// Raw wire format of [`DiscoveredMode`]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Pod, Zeroable)]
#[repr(C)]
pub struct DiscoveredModeRaw {
    /// Raw VDO data
    pub vdo: U32LE,
    /// VDO object position
    pub position: u8,
}

impl From<DiscoveredMode> for DiscoveredModeRaw {
    fn from(value: DiscoveredMode) -> Self {
        Self {
            vdo: value.vdo.into(),
            position: value.position,
        }
    }
}

impl From<DiscoveredModeRaw> for DiscoveredMode {
    fn from(value: DiscoveredModeRaw) -> Self {
        Self {
            vdo: value.vdo.get(),
            position: value.position,
        }
    }
}

/// Output data length
pub const OUTPUT_LEN: usize = 35;

/// Length of the discovered modes array
pub const DISCOVERED_MODES_LEN: usize = 7;

/// GCdm output
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DiscoveredModes {
    pub alt_modes: [DiscoveredMode; DISCOVERED_MODES_LEN],
}

/// Raw wire format of [`DiscoveredModes`]
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Pod, Zeroable)]
#[repr(C)]
pub struct DiscoveredModesRaw {
    pub alt_modes: [DiscoveredModeRaw; DISCOVERED_MODES_LEN],
}

impl From<DiscoveredModes> for DiscoveredModesRaw {
    fn from(value: DiscoveredModes) -> Self {
        let mut alt_modes = [DiscoveredModeRaw::default(); DISCOVERED_MODES_LEN];
        for (raw, mode) in alt_modes.iter_mut().zip(value.alt_modes.iter()) {
            *raw = (*mode).into();
        }

        Self { alt_modes }
    }
}

impl From<DiscoveredModesRaw> for DiscoveredModes {
    fn from(value: DiscoveredModesRaw) -> Self {
        let mut alt_modes = [DiscoveredMode::default(); DISCOVERED_MODES_LEN];
        for (mode, raw) in alt_modes.iter_mut().zip(value.alt_modes.iter()) {
            *mode = (*raw).into();
        }

        Self { alt_modes }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn test_raw_sizes() {
        assert_eq!(core::mem::size_of::<InputRaw>(), INPUT_LEN);
        assert_eq!(core::mem::size_of::<DiscoveredModesRaw>(), OUTPUT_LEN);
    }

    #[test]
    fn test_gcdm_input_encode() {
        // Construct via From<Svid> and verify encode produces correct bytes
        let input = Input::from(Svid(0xAB12));

        // Encode layout: reserved byte (0x00) + SVID u16 LE (0x12, 0xAB)
        const EXPECTED: [u8; INPUT_LEN] = [0x00, 0x12, 0xAB];
        let raw = InputRaw::from(input);
        assert_eq!(bytemuck::bytes_of(&raw), &EXPECTED);

        // Verify accessor
        assert_eq!(input.svid, Svid(0xAB12));

        // Round trip back to the native type
        let decoded: InputRaw = bytemuck::pod_read_unaligned(&EXPECTED);
        assert_eq!(Input::from(decoded), input);
    }

    #[test]
    fn test_gcdm_discovered_modes_decode() {
        // Construct expected struct with non-zero values
        let expected_struct = DiscoveredModes {
            alt_modes: [
                DiscoveredMode {
                    vdo: 0x12345678,
                    position: 1,
                },
                DiscoveredMode {
                    vdo: 0x9ABCDEF0,
                    position: 2,
                },
                DiscoveredMode {
                    vdo: 0xDEADBEEF,
                    position: 3,
                },
                DiscoveredMode {
                    vdo: 0xA1B2C3D4,
                    position: 4,
                },
                DiscoveredMode {
                    vdo: 0xFDB97531,
                    position: 5,
                },
                DiscoveredMode {
                    vdo: 0x2468ACE0,
                    position: 6,
                },
                DiscoveredMode {
                    vdo: 0x13579BDF,
                    position: 7,
                },
            ],
        };

        const EXPECTED_BYTES: [u8; OUTPUT_LEN] = [
            0x78, 0x56, 0x34, 0x12, 0x01, 0xF0, 0xDE, 0xBC, 0x9A, 0x02, 0xEF, 0xBE, 0xAD, 0xDE, 0x03, 0xD4, 0xC3, 0xB2,
            0xA1, 0x04, 0x31, 0x75, 0xB9, 0xFD, 0x05, 0xE0, 0xAC, 0x68, 0x24, 0x06, 0xDF, 0x9B, 0x57, 0x13, 0x07,
        ];

        // Decode from bytes and verify against expected struct
        let raw: DiscoveredModesRaw = bytemuck::pod_read_unaligned(&EXPECTED_BYTES);
        let decoded = DiscoveredModes::from(raw);
        assert_eq!(decoded, expected_struct);

        // Verify individual field accessors
        assert_eq!(decoded.alt_modes[0].vdo, 0x12345678);
        assert_eq!(decoded.alt_modes[0].position, 1);
        assert_eq!(decoded.alt_modes[3].vdo, 0xA1B2C3D4);
        assert_eq!(decoded.alt_modes[3].position, 4);
        assert_eq!(decoded.alt_modes[6].vdo, 0x13579BDF);
        assert_eq!(decoded.alt_modes[6].position, 7);

        // Encoding the native type reproduces the original bytes
        assert_eq!(
            bytemuck::bytes_of(&DiscoveredModesRaw::from(expected_struct)),
            &EXPECTED_BYTES
        );
    }
}
