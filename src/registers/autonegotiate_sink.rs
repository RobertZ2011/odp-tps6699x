//! Autonegotiate Sink register (`0x37`).
//!
//! # TODO
//! - Use a units crate for safely expressing values like 20 mV per LSB, etc.

use embedded_usb_pd::pdo::{MA10_UNIT, MA50_UNIT, MV20_UNIT, MV50_UNIT, MW250_UNIT};

pub use crate::registers::{
    AutoComputeSinkMaxVoltage, AutoComputeSinkMinPower, AutoComputeSinkMinVoltage, AutoNegRdoPriority,
    NoCapabilityMismatch, PpsRequestInterval,
};

/// The address of the `Autonegotiate Sink` register.
pub const ADDR: u8 = 0x37;

/// The length of the `Autonegotiate Sink` register, in bytes.
pub const LEN: usize = 24;

/// High-level representation of the Autonegotiate Sink register (`0x37`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AutonegotiateSink {
    /// Configuration for tie-breaker in PDO selection.
    pub auto_neg_rdo_priority: AutoNegRdoPriority,
    /// Value used for the NoUSBSusp Flag in the RDO.
    pub no_usb_suspend: bool,
    /// How to compute the minimum power that the sink requires.
    pub auto_compute_sink_min_power: AutoComputeSinkMinPower,
    /// Configuration for capability mismatch in RDO.
    pub no_capability_mismatch: NoCapabilityMismatch,
    /// How to compute the minimum voltage that the sink requires.
    pub auto_compute_sink_min_voltage: AutoComputeSinkMinVoltage,
    /// How to compute the maximum voltage that the sink requires.
    pub auto_compute_sink_max_voltage: AutoComputeSinkMaxVoltage,
    /// Sink path and capability mismatch setting.
    pub auto_disable_sink_upon_capability_mismatch: bool,
    /// Disable the sinking path during contract negotiation, and enable it back after receiving PS-RDY from SRC.
    pub auto_disable_input_for_sink_standby: bool,
    /// Disable the sinking path during contract negotiation, and enable it back after receiving PS-RDY from SRC when in DBM.
    pub auto_disable_input_for_sink_standby_in_dbm: bool,
    /// Automatically re-enable the sink path for Sink Standby when PS-RDY is received.
    pub auto_enable_standby_srdy: bool,
    /// Keep the sink path disabled until final PD contract is set.
    pub auto_enable_input_after_snk_ready_in_dbm: bool,
    /// Maximum current to request in mA.
    pub auto_neg_max_current_ma: u16,
    /// Minimum operating power required by the sink in mW.
    pub auto_neg_sink_min_required_power_mw: u32,
    /// Maximum voltage to request in mV.
    pub auto_neg_max_voltage_mv: u16,
    /// Minimum voltage to request in mV.
    pub auto_neg_min_voltage_mv: u16,
    /// Capabilities mismatch power threshold in mW.
    pub auto_neg_capabilities_mismatch_power_mw: u32,
    /// Enable Sink PPS mode.
    pub pps_enable_sink_mode: bool,
    /// Sink PPS request interval.
    pub pps_request_interval: PpsRequestInterval,
    /// Selection for CV or CC mode.
    pub pps_source_operating_mode: bool,
    /// Select only a source with full voltage range.
    pub pps_required_full_voltage_range: bool,
    /// Sink path handling during supply type transition.
    pub pps_disable_sink_upon_non_apdo_contract: bool,
    /// Enable Sink Path above PPS Sink Voltage threshold.
    pub enable_sink_path_above_pps_sink_voltage_threshold: bool,
    /// Disable Sink Path if not in PPS contract.
    pub disable_sink_if_not_in_pps_contract: bool,
    /// Voltage threshold in Sink PPS mode.
    pub pps_sink_voltage_threshold: u16,
    /// Operation current in Sink PPS mode in mA.
    pub pps_operating_current_ma: u16,
    /// Desired output voltage in mV.
    pub pps_output_voltage_mv: u16,
    /// Enable Sink EPR AVS mode.
    pub epr_avs_enable_sink_mode: bool,
    /// AVS operating current.
    pub epr_avs_operating_current: u8,
    /// AVS operating voltage.
    pub epr_avs_output_voltage: u16,
}

impl AutonegotiateSink {
    /// The default bytes for the autonegotiate sink register.
    const DEFAULT: [u8; LEN] = [
        0xBE, 0x40, 0x1F, 0x0F, 0x90, 0x91, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    ];
}

impl From<crate::registers::AutonegotiateSink> for AutonegotiateSink {
    fn from(raw: crate::registers::AutonegotiateSink) -> Self {
        Self {
            auto_neg_rdo_priority: raw.auto_neg_rdo_priority(),
            no_usb_suspend: raw.no_usb_suspend(),
            auto_compute_sink_min_power: raw.auto_compute_sink_min_power(),
            no_capability_mismatch: raw.no_capability_mismatch(),
            auto_compute_sink_min_voltage: raw.auto_compute_sink_min_voltage(),
            auto_compute_sink_max_voltage: raw.auto_compute_sink_max_voltage(),
            auto_disable_sink_upon_capability_mismatch: raw.auto_disable_sink_upon_capability_mismatch(),
            auto_disable_input_for_sink_standby: raw.auto_disable_input_for_sink_standby(),
            auto_disable_input_for_sink_standby_in_dbm: raw.auto_disable_input_for_sink_standby_in_dbm(),
            auto_enable_standby_srdy: raw.auto_enable_standby_srdy(),
            auto_enable_input_after_snk_ready_in_dbm: raw.auto_enable_input_after_snk_ready_in_dbm(),
            auto_neg_max_current_ma: raw.auto_neg_max_current() * MA10_UNIT,
            auto_neg_sink_min_required_power_mw: raw.auto_neg_sink_min_required_power() as u32 * MW250_UNIT,
            auto_neg_max_voltage_mv: raw.auto_neg_max_voltage() * MV50_UNIT,
            auto_neg_min_voltage_mv: raw.auto_neg_min_voltage() * MV50_UNIT,
            auto_neg_capabilities_mismatch_power_mw: raw.auto_neg_capabilities_mismatch_power() as u32 * MW250_UNIT,
            pps_enable_sink_mode: raw.pps_enable_sink_mode(),
            pps_request_interval: raw.pps_request_interval(),
            pps_source_operating_mode: raw.pps_source_operating_mode(),
            pps_required_full_voltage_range: raw.pps_required_full_voltage_range(),
            pps_disable_sink_upon_non_apdo_contract: raw.pps_disable_sink_upon_non_apdo_contract(),
            enable_sink_path_above_pps_sink_voltage_threshold: raw.enable_sink_path_above_pps_sink_voltage_threshold(),
            disable_sink_if_not_in_pps_contract: raw.disable_sink_if_not_in_pps_contract(),
            pps_sink_voltage_threshold: raw.pps_sink_voltage_threshold(),
            pps_operating_current_ma: raw.pps_operating_current() as u16 * MA50_UNIT,
            pps_output_voltage_mv: raw.pps_output_voltage() * MV20_UNIT,
            epr_avs_enable_sink_mode: raw.epr_avs_enable_sink_mode(),
            epr_avs_operating_current: raw.epr_avs_operating_current(),
            epr_avs_output_voltage: raw.epr_avs_output_voltage(),
        }
    }
}

impl From<AutonegotiateSink> for crate::registers::AutonegotiateSink {
    fn from(sink: AutonegotiateSink) -> Self {
        let mut raw = Self::default();
        raw.set_auto_neg_rdo_priority(sink.auto_neg_rdo_priority);
        raw.set_no_usb_suspend(sink.no_usb_suspend);
        raw.set_auto_compute_sink_min_power(sink.auto_compute_sink_min_power);
        raw.set_no_capability_mismatch(sink.no_capability_mismatch);
        raw.set_auto_compute_sink_min_voltage(sink.auto_compute_sink_min_voltage);
        raw.set_auto_compute_sink_max_voltage(sink.auto_compute_sink_max_voltage);
        raw.set_auto_disable_sink_upon_capability_mismatch(sink.auto_disable_sink_upon_capability_mismatch);
        raw.set_auto_disable_input_for_sink_standby(sink.auto_disable_input_for_sink_standby);
        raw.set_auto_disable_input_for_sink_standby_in_dbm(sink.auto_disable_input_for_sink_standby_in_dbm);
        raw.set_auto_enable_standby_srdy(sink.auto_enable_standby_srdy);
        raw.set_auto_enable_input_after_snk_ready_in_dbm(sink.auto_enable_input_after_snk_ready_in_dbm);
        raw.set_auto_neg_max_current(sink.auto_neg_max_current_ma / MA10_UNIT);
        raw.set_auto_neg_sink_min_required_power((sink.auto_neg_sink_min_required_power_mw / MW250_UNIT) as u16);
        raw.set_auto_neg_max_voltage(sink.auto_neg_max_voltage_mv / MV50_UNIT);
        raw.set_auto_neg_min_voltage(sink.auto_neg_min_voltage_mv / MV50_UNIT);
        raw.set_auto_neg_capabilities_mismatch_power(
            (sink.auto_neg_capabilities_mismatch_power_mw / MW250_UNIT) as u16,
        );
        raw.set_pps_enable_sink_mode(sink.pps_enable_sink_mode);
        raw.set_pps_request_interval(sink.pps_request_interval);
        raw.set_pps_source_operating_mode(sink.pps_source_operating_mode);
        raw.set_pps_required_full_voltage_range(sink.pps_required_full_voltage_range);
        raw.set_pps_disable_sink_upon_non_apdo_contract(sink.pps_disable_sink_upon_non_apdo_contract);
        raw.set_enable_sink_path_above_pps_sink_voltage_threshold(
            sink.enable_sink_path_above_pps_sink_voltage_threshold,
        );
        raw.set_disable_sink_if_not_in_pps_contract(sink.disable_sink_if_not_in_pps_contract);
        raw.set_pps_sink_voltage_threshold(sink.pps_sink_voltage_threshold);
        raw.set_pps_operating_current((sink.pps_operating_current_ma / MA50_UNIT) as u8);
        raw.set_pps_output_voltage(sink.pps_output_voltage_mv / MV20_UNIT);
        raw.set_epr_avs_enable_sink_mode(sink.epr_avs_enable_sink_mode);
        raw.set_epr_avs_operating_current(sink.epr_avs_operating_current);
        raw.set_epr_avs_output_voltage(sink.epr_avs_output_voltage);
        raw
    }
}

impl From<[u8; LEN]> for AutonegotiateSink {
    fn from(value: [u8; LEN]) -> Self {
        crate::registers::AutonegotiateSink::from(value).into()
    }
}

impl From<AutonegotiateSink> for [u8; LEN] {
    fn from(value: AutonegotiateSink) -> Self {
        crate::registers::AutonegotiateSink::from(value).into()
    }
}

impl Default for AutonegotiateSink {
    fn default() -> Self {
        Self::DEFAULT.into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_matches_datasheet() {
        let actual = AutonegotiateSink::default();

        assert_eq!(actual.auto_neg_rdo_priority, AutoNegRdoPriority::HigherVoltage);
        assert!(actual.no_usb_suspend);
        assert_eq!(
            actual.auto_compute_sink_min_power,
            AutoComputeSinkMinPower::ComputedByPdController
        );
        assert_eq!(actual.no_capability_mismatch, NoCapabilityMismatch::Disabled);
        assert_eq!(
            actual.auto_compute_sink_min_voltage,
            AutoComputeSinkMinVoltage::ComputedByPdController
        );
        assert_eq!(
            actual.auto_compute_sink_max_voltage,
            AutoComputeSinkMaxVoltage::ComputedByPdController
        );
        assert!(!actual.auto_disable_sink_upon_capability_mismatch);
        assert!(actual.auto_disable_input_for_sink_standby);
        assert!(!actual.auto_disable_input_for_sink_standby_in_dbm);
        assert!(!actual.auto_enable_standby_srdy);
        assert!(!actual.auto_enable_input_after_snk_ready_in_dbm);
        assert_eq!(actual.auto_neg_max_current_ma, 0x1F4 * MA10_UNIT);
        assert_eq!(actual.auto_neg_sink_min_required_power_mw, 0x3C * MW250_UNIT);
        assert_eq!(actual.auto_neg_max_voltage_mv, 0x190 * MV50_UNIT);
        assert_eq!(actual.auto_neg_min_voltage_mv, 0x64 * MV50_UNIT);
        assert_eq!(actual.auto_neg_capabilities_mismatch_power_mw, 0);
        assert!(!actual.pps_enable_sink_mode);
        assert_eq!(actual.pps_request_interval, PpsRequestInterval::EightSeconds);
        assert!(!actual.pps_source_operating_mode);
        assert!(!actual.pps_required_full_voltage_range);
        assert!(!actual.pps_disable_sink_upon_non_apdo_contract);
        assert!(!actual.enable_sink_path_above_pps_sink_voltage_threshold);
        assert!(!actual.disable_sink_if_not_in_pps_contract);
        assert_eq!(actual.pps_sink_voltage_threshold, 0);
        assert_eq!(actual.pps_operating_current_ma, 0);
        assert_eq!(actual.pps_output_voltage_mv, 0);
        assert!(!actual.epr_avs_enable_sink_mode);
        assert_eq!(actual.epr_avs_operating_current, 0);
        assert_eq!(actual.epr_avs_output_voltage, 0);

        let bytes: [u8; LEN] = actual.into();
        assert_eq!(bytes, AutonegotiateSink::DEFAULT);
    }

    #[test]
    fn test_autonegotiate_sink_nonzero_roundtrip() {
        const EXPECTED: [u8; LEN] = [
            0xFF, 0xB7, 0x1A, 0x0F, 0x90, 0x91, 0xE1, 0x01, 0x3D, 0x00, 0xB8, 0xAA, 0x3C, 0xF4, 0x01, 0x00, 0x01, 0x00,
            0x00, 0x00, 0x7F, 0x78, 0x15, 0x00,
        ];

        let sink = AutonegotiateSink {
            auto_neg_rdo_priority: AutoNegRdoPriority::LowerVoltage,
            no_usb_suspend: true,
            auto_compute_sink_min_power: AutoComputeSinkMinPower::ComputedByPdController,
            no_capability_mismatch: NoCapabilityMismatch::Disabled,
            auto_compute_sink_min_voltage: AutoComputeSinkMinVoltage::ComputedByPdController,
            auto_compute_sink_max_voltage: AutoComputeSinkMaxVoltage::ComputedByPdController,
            auto_disable_sink_upon_capability_mismatch: true,
            auto_disable_input_for_sink_standby: true,
            auto_disable_input_for_sink_standby_in_dbm: true,
            auto_enable_standby_srdy: true,
            auto_enable_input_after_snk_ready_in_dbm: true,
            auto_neg_max_current_ma: 0x1AB * MA10_UNIT,
            auto_neg_sink_min_required_power_mw: 15_000,
            auto_neg_max_voltage_mv: 20_000,
            auto_neg_min_voltage_mv: 5_000,
            auto_neg_capabilities_mismatch_power_mw: 7_500,
            pps_enable_sink_mode: true,
            pps_request_interval: PpsRequestInterval::TwoSeconds,
            pps_source_operating_mode: true,
            pps_required_full_voltage_range: true,
            pps_disable_sink_upon_non_apdo_contract: true,
            enable_sink_path_above_pps_sink_voltage_threshold: true,
            disable_sink_if_not_in_pps_contract: true,
            pps_sink_voltage_threshold: 0x555,
            pps_operating_current_ma: 3_000,
            pps_output_voltage_mv: 5_000,
            epr_avs_enable_sink_mode: true,
            epr_avs_operating_current: 0x7F,
            epr_avs_output_voltage: 0x0ABC,
        };

        let bytes: [u8; LEN] = sink.into();
        assert_eq!(bytes, EXPECTED);

        let raw: crate::registers::AutonegotiateSink = sink.into();
        let sink2 = AutonegotiateSink::from(raw);
        assert_eq!(sink2, sink);
    }
}
