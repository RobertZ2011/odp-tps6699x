//! The `Trig` command.

use bytemuck::{Pod, Zeroable};
use embedded_usb_pd::PdError;

/// The length of the arguments for the `Trig` command.
#[allow(dead_code)]
pub(crate) const ARGS_LEN: usize = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Cmd {
    FaultInputPort1 = 0x21,
    FaultInputPort2 = 0x22,
    RetimerForcePwr = 0x2A,
    RetimerHighCurrentContract = 0x2F,
    I3cMasterIrq = 0x38,
    Mreset = 0x45,
}

impl From<Cmd> for u8 {
    fn from(value: Cmd) -> Self {
        value as u8
    }
}

impl TryFrom<u8> for Cmd {
    type Error = PdError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x21 => Ok(Cmd::FaultInputPort1),
            0x22 => Ok(Cmd::FaultInputPort2),
            0x2A => Ok(Cmd::RetimerForcePwr),
            0x2F => Ok(Cmd::RetimerHighCurrentContract),
            0x38 => Ok(Cmd::I3cMasterIrq),
            0x45 => Ok(Cmd::Mreset),
            _ => Err(PdError::InvalidParams),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Edge {
    /// Trig Vgpio falling edge
    Falling = 0,
    /// Trig Vgpio rising edge
    Rising = 1,
}

impl From<Edge> for u8 {
    fn from(value: Edge) -> Self {
        value as u8
    }
}

impl TryFrom<u8> for Edge {
    type Error = PdError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Edge::Falling),
            1 => Ok(Edge::Rising),
            _ => Err(PdError::InvalidParams),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Args {
    pub edge: Edge,
    pub cmd: Cmd,
}

/// Raw wire format of [`Args`]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Pod, Zeroable)]
#[repr(C)]
pub struct ArgsRaw {
    pub edge: u8,
    pub cmd: u8,
}

impl From<Args> for ArgsRaw {
    fn from(value: Args) -> Self {
        Self {
            edge: value.edge.into(),
            cmd: value.cmd.into(),
        }
    }
}

impl TryFrom<ArgsRaw> for Args {
    type Error = PdError;

    fn try_from(value: ArgsRaw) -> Result<Self, Self::Error> {
        Ok(Self {
            edge: value.edge.try_into()?,
            cmd: value.cmd.try_into()?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_args_raw_size() {
        assert_eq!(core::mem::size_of::<ArgsRaw>(), ARGS_LEN);
    }

    #[test]
    fn test_args_encode() {
        let args = Args {
            edge: Edge::Rising,
            cmd: Cmd::Mreset,
        };
        let raw = ArgsRaw::from(args);
        assert_eq!(bytemuck::bytes_of(&raw), &[0x01, 0x45]);
    }

    #[test]
    fn test_args_roundtrip() {
        let cases = [
            Args {
                edge: Edge::Falling,
                cmd: Cmd::FaultInputPort1,
            },
            Args {
                edge: Edge::Rising,
                cmd: Cmd::I3cMasterIrq,
            },
        ];

        for args in cases {
            let raw = ArgsRaw::from(args);
            let bytes = bytemuck::bytes_of(&raw);
            let decoded: ArgsRaw = bytemuck::pod_read_unaligned(bytes);
            assert_eq!(Args::try_from(decoded).unwrap(), args);
        }
    }

    #[test]
    fn test_invalid_args_raw() {
        assert_eq!(
            Args::try_from(ArgsRaw { edge: 0x02, cmd: 0x45 }),
            Err(PdError::InvalidParams)
        );
        assert_eq!(
            Args::try_from(ArgsRaw { edge: 0x01, cmd: 0xFF }),
            Err(PdError::InvalidParams)
        );
    }
}
