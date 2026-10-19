//! Port Configuration register (`0x28`).
//!
//! This register controls various port configuration options.
//!
//! # TODO
//! - Use a units crate for safely expressing values like 50 mV per LSB, etc.

use embedded_usb_pd::pdo::MV50_UNIT;

pub use crate::registers::{
    CrossbarType, ExternalDcdcType, LevelShifterDirectionCfg, OvpForPp5V as OvpForPp5v, SbuMuxDebugSetting,
    SbuMuxDefaultSetting, SbuMuxUsage, TypeCStateMachine, TypeCSupportOptions, Usb3Rate, VbusOvpUsage,
    VbusSinkUvpTripHv,
};

/// The address of the `Port Configuration` register.
pub const ADDR: u8 = 0x28;

/// The length of the `Port Configuration` register, in bytes.
pub const LEN: usize = 18;

/// High-level representation of the Port Configuration register (`0x28`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PortConfig {
    /// TypeC State machine.
    pub typec_state_machine: TypeCStateMachine,
    /// Crossbar Type.
    pub crossbar_type: CrossbarType,
    /// PP_EXT ActiveLow.
    pub pp_ext_active_low: bool,
    /// TypeC Support Options.
    pub typec_support_options: TypeCSupportOptions,
    /// Disable PD.
    pub disable_pd: bool,
    /// USB Communication Capable.
    pub usb_communication_capable: bool,
    /// DebugAccessory Support.
    pub debug_accessory_support: bool,
    /// USB3 Rate.
    pub usb3_rate: Usb3Rate,
    /// Crossbar I2C Controller enable.
    pub crossbar_i2c_controller_enable: bool,
    /// VBUS OVP Usage.
    pub vbus_ovp_usage: VbusOvpUsage,
    /// OVP for PP5V.
    pub ovp_for_pp5v: OvpForPp5v,
    /// Crossbar Config Type 1 Extended.
    pub crossbar_config_type1_extended: bool,
    /// Remove Safe-State Between USB3-to-DP Transition.
    pub remove_safe_state_between_usb3_to_dp_transition: bool,
    /// VBUS Sink UVP Trip HV.
    pub vbus_sink_uvp_trip_hv: VbusSinkUvpTripHv,
    /// APDO VBUS UVP Threshold.
    pub apdo_vbus_uvp_threshold: u8,
    /// APDO ILIM Over Shoot.
    pub apdo_ilim_over_shoot: u8,
    /// APDO VBUS Uvp TripPoint Offset.
    pub apdo_vbus_uvp_trip_point_offset: u16,
    /// VBUS For Valid PPS Status.
    pub vbus_for_valid_pps_status: u16,
    /// External DCDC Type field.
    pub external_dcdc_type: ExternalDcdcType,
    /// Enables support for GPIO10 to I2C IRQ.
    pub sink_mode_i2c_irq_config: bool,
    /// Threshold voltage to trigger the GREATER_THAN_THRESHOLD_VOLTAGE GPIO Event in millivolts.
    pub gt_threshold_voltage_mv: u16,
    /// Enable internal auxbiasing.
    pub enable_internal_auxbiasing: bool,
    /// Enable internal level shifter.
    pub enable_internal_level_shifter: bool,
    /// Level shifter direction configuration.
    pub level_shifter_direction_cfg: LevelShifterDirectionCfg,
    /// SBU Mux Debug Setting field.
    pub sbu_mux_debug_setting: SbuMuxDebugSetting,
    /// SBU Mux Default Setting field.
    pub sbu_mux_default_setting: SbuMuxDefaultSetting,
    /// SBU Mux Usage field.
    pub sbu_mux_usage: SbuMuxUsage,
    /// EPR Supported as Source.
    pub epr_supported_as_source: bool,
    /// EPR Supported as Sink.
    pub epr_supported_as_sink: bool,
    /// Disable sourcing in dead battery mode.
    pub disable_sourcing_in_dbm: bool,
    /// Configuration for flipping Px_DBG.
    pub flip_crossbar_dbg_setting: bool,
    /// Configuration for flipping Px_AUX.
    pub flip_crossbar_aux_setting: bool,
    /// Configuration for flipping PX_SBTX/RX.
    pub flip_crossbar_sbtx_setting: bool,
}

impl Default for PortConfig {
    fn default() -> Self {
        crate::registers::PortConfig::default().into()
    }
}

impl From<crate::registers::PortConfig> for PortConfig {
    fn from(raw: crate::registers::PortConfig) -> Self {
        Self {
            typec_state_machine: raw.typec_state_machine(),
            crossbar_type: raw.crossbar_type(),
            pp_ext_active_low: raw.pp_ext_active_low(),
            typec_support_options: raw.typec_support_options(),
            disable_pd: raw.disable_pd(),
            usb_communication_capable: raw.usb_communication_capable(),
            debug_accessory_support: raw.debug_accessory_support(),
            usb3_rate: raw.usb_3_rate(),
            crossbar_i2c_controller_enable: raw.crossbar_i_2_c_controller_enable(),
            vbus_ovp_usage: raw.vbus_ovp_usage(),
            ovp_for_pp5v: raw.ovp_for_pp_5_v(),
            crossbar_config_type1_extended: raw.crossbar_config_type_1_extended(),
            remove_safe_state_between_usb3_to_dp_transition: raw.remove_safe_state_between_usb_3_to_dp_transition(),
            vbus_sink_uvp_trip_hv: raw.vbus_sink_uvp_trip_hv(),
            apdo_vbus_uvp_threshold: raw.apdo_vbus_uvp_threshold(),
            apdo_ilim_over_shoot: raw.apdo_ilim_over_shoot(),
            apdo_vbus_uvp_trip_point_offset: raw.apdo_vbus_uvp_trip_point_offset(),
            vbus_for_valid_pps_status: raw.vbus_for_valid_pps_status(),
            external_dcdc_type: raw.external_dcdc_type(),
            sink_mode_i2c_irq_config: raw.sink_mode_i_2_c_irq_config(),
            gt_threshold_voltage_mv: raw.gt_threshold_voltage() * MV50_UNIT,
            enable_internal_auxbiasing: raw.enable_internal_auxbiasing(),
            enable_internal_level_shifter: raw.enable_internal_level_shifter(),
            level_shifter_direction_cfg: raw.level_shifter_direction_cfg(),
            sbu_mux_debug_setting: raw.sbu_mux_debug_setting(),
            sbu_mux_default_setting: raw.sbu_mux_default_setting(),
            sbu_mux_usage: raw.sbu_mux_usage(),
            epr_supported_as_source: raw.epr_supported_as_source(),
            epr_supported_as_sink: raw.epr_supported_as_sink(),
            disable_sourcing_in_dbm: raw.disable_sourcing_in_dbm(),
            flip_crossbar_dbg_setting: raw.flip_crossbar_dbg_setting(),
            flip_crossbar_aux_setting: raw.flip_crossbar_aux_setting(),
            flip_crossbar_sbtx_setting: raw.flip_crossbar_sbtx_setting(),
        }
    }
}

impl From<PortConfig> for crate::registers::PortConfig {
    fn from(config: PortConfig) -> Self {
        let mut raw = Self::default();
        raw.set_typec_state_machine(config.typec_state_machine);
        raw.set_crossbar_type(config.crossbar_type);
        raw.set_pp_ext_active_low(config.pp_ext_active_low);
        raw.set_typec_support_options(config.typec_support_options);
        raw.set_disable_pd(config.disable_pd);
        raw.set_usb_communication_capable(config.usb_communication_capable);
        raw.set_debug_accessory_support(config.debug_accessory_support);
        raw.set_usb_3_rate(config.usb3_rate);
        raw.set_crossbar_i_2_c_controller_enable(config.crossbar_i2c_controller_enable);
        raw.set_vbus_ovp_usage(config.vbus_ovp_usage);
        raw.set_ovp_for_pp_5_v(config.ovp_for_pp5v);
        raw.set_crossbar_config_type_1_extended(config.crossbar_config_type1_extended);
        raw.set_remove_safe_state_between_usb_3_to_dp_transition(
            config.remove_safe_state_between_usb3_to_dp_transition,
        );
        raw.set_vbus_sink_uvp_trip_hv(config.vbus_sink_uvp_trip_hv);
        raw.set_apdo_vbus_uvp_threshold(config.apdo_vbus_uvp_threshold);
        raw.set_apdo_ilim_over_shoot(config.apdo_ilim_over_shoot);
        raw.set_apdo_vbus_uvp_trip_point_offset(config.apdo_vbus_uvp_trip_point_offset);
        raw.set_vbus_for_valid_pps_status(config.vbus_for_valid_pps_status);
        raw.set_external_dcdc_type(config.external_dcdc_type);
        raw.set_sink_mode_i_2_c_irq_config(config.sink_mode_i2c_irq_config);
        raw.set_gt_threshold_voltage(config.gt_threshold_voltage_mv / MV50_UNIT);
        raw.set_enable_internal_auxbiasing(config.enable_internal_auxbiasing);
        raw.set_enable_internal_level_shifter(config.enable_internal_level_shifter);
        raw.set_level_shifter_direction_cfg(config.level_shifter_direction_cfg);
        raw.set_sbu_mux_debug_setting(config.sbu_mux_debug_setting);
        raw.set_sbu_mux_default_setting(config.sbu_mux_default_setting);
        raw.set_sbu_mux_usage(config.sbu_mux_usage);
        raw.set_epr_supported_as_source(config.epr_supported_as_source);
        raw.set_epr_supported_as_sink(config.epr_supported_as_sink);
        raw.set_disable_sourcing_in_dbm(config.disable_sourcing_in_dbm);
        raw.set_flip_crossbar_dbg_setting(config.flip_crossbar_dbg_setting);
        raw.set_flip_crossbar_aux_setting(config.flip_crossbar_aux_setting);
        raw.set_flip_crossbar_sbtx_setting(config.flip_crossbar_sbtx_setting);
        raw
    }
}

impl From<[u8; LEN]> for PortConfig {
    fn from(value: [u8; LEN]) -> Self {
        crate::registers::PortConfig::from(value).into()
    }
}

impl From<PortConfig> for [u8; LEN] {
    fn from(value: PortConfig) -> Self {
        crate::registers::PortConfig::from(value).into()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_port_config_default() {
        let config = PortConfig::default();
        assert_eq!(<[u8; LEN]>::from(config), [0u8; LEN]);
    }

    #[test]
    fn test_port_config_nonzero_roundtrip() {
        const EXPECTED: [u8; LEN] = [
            0x86, 0xDD, 0xE2, 0x5C, 0xCD, 0xAB, 0x34, 0x12, 0x01, 0x01, 0x23, 0x01, 0x00, 0x00, 0x00, 0xB0, 0x53, 0x3F,
        ];

        let config = PortConfig {
            typec_state_machine: TypeCStateMachine::Drp,
            crossbar_type: CrossbarType::Type2,
            pp_ext_active_low: true,
            typec_support_options: TypeCSupportOptions::TrySrcDrp,
            disable_pd: true,
            usb_communication_capable: true,
            debug_accessory_support: true,
            usb3_rate: Usb3Rate::Gen2,
            crossbar_i2c_controller_enable: true,
            vbus_ovp_usage: VbusOvpUsage::Pct111,
            ovp_for_pp5v: OvpForPp5v::V5P8,
            crossbar_config_type1_extended: true,
            remove_safe_state_between_usb3_to_dp_transition: true,
            vbus_sink_uvp_trip_hv: VbusSinkUvpTripHv::Pct25,
            apdo_vbus_uvp_threshold: 0x3,
            apdo_ilim_over_shoot: 0x2,
            apdo_vbus_uvp_trip_point_offset: 0xABCD,
            vbus_for_valid_pps_status: 0x1234,
            external_dcdc_type: ExternalDcdcType::Bq25758,
            sink_mode_i2c_irq_config: true,
            gt_threshold_voltage_mv: 0x0123 * MV50_UNIT,
            enable_internal_auxbiasing: true,
            enable_internal_level_shifter: true,
            level_shifter_direction_cfg: LevelShifterDirectionCfg::I2CControlled,
            sbu_mux_debug_setting: SbuMuxDebugSetting::AnyDebug,
            sbu_mux_default_setting: SbuMuxDefaultSetting::EnableDbg,
            sbu_mux_usage: SbuMuxUsage::AllAux,
            epr_supported_as_source: true,
            epr_supported_as_sink: true,
            disable_sourcing_in_dbm: true,
            flip_crossbar_dbg_setting: true,
            flip_crossbar_aux_setting: true,
            flip_crossbar_sbtx_setting: true,
        };

        let bytes: [u8; LEN] = config.into();
        assert_eq!(bytes, EXPECTED);

        let raw: crate::registers::PortConfig = config.into();
        let config2 = PortConfig::from(raw);
        assert_eq!(config2, config);
    }
}
