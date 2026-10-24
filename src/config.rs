//! Board-specific TMC4671 hardware configuration.

/// ADC_I_SELECT routing UX=I0, WY=I1, V=I2 with ADC_I0/ADC_I1 taken from the sigma-delta inputs.
pub const ADC_I_SELECT_UX_I0_V_I2_WY_I1: u32 = foci_tmc4671::registers::AdcISelect::new()
    .with_i0_select(0)
    .with_i1_select(1)
    .with_ux_select(0)
    .with_v_select(2)
    .with_wy_select(1)
    .into_bits();

/// DSADC MDEC (datasheet 4.11.2): `round((PWM_MAXCNT + 1) / 12) - 2`, floored at 0.
pub const fn dsadc_mdec_from_pwm_maxcnt(pwm_maxcnt: u16) -> u16 {
    let pwm_ticks = pwm_maxcnt as u32 + 1;
    let rounded_div_12 = (pwm_ticks + 6) / 12;
    if rounded_div_12 <= 2 {
        0
    } else {
        (rounded_div_12 - 2) as u16
    }
}

const CURRENT_FILTER_PWM_RATIO: f32 = 0.4;

/// Board-specific TMC4671 hardware configuration, constructed as a `const` by each board crate.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct TmcConfig {
    pub pwm_maxcnt: u16,
    /// TMC4671 clock frequency in Hz.
    pub tmc_clock_hz: u32,
    /// High-side break-before-make dead-time in 10 ns cycles.
    pub bbm_h: u8,
    /// Low-side break-before-make dead-time in 10 ns cycles.
    pub bbm_l: u8,
    pub dsadc_mdec_a: u16,
    pub dsadc_mdec_b: u16,
    pub adc_i_select: u32,
    /// R/L measurement excitation voltage in PIDOUT units.
    pub excitation_voltage: u16,
    /// Effective electrical delay for current-loop synthesis.
    pub electrical_model_theta_e_us: u32,
    /// Max encoder movement during flux-axis current validation.
    pub current_validation_flux_max_encoder_delta_counts: u16,
    /// Max encoder movement during torque-axis current validation.
    pub current_validation_torque_max_encoder_delta_counts: u16,
    /// Maximum expected resistance-profile current in TMC current counts.
    pub resistance_max_expected_current_count: u16,
    /// Whether the scale metadata is hardware-validated for physical display.
    pub resistance_scale_metadata_validated: bool,
    /// Milliamps per LSB of `PID_TORQUE_FLUX_LIMITS`.
    pub current_scale_ma_per_lsb: f32,
    /// RMS milliamp ceiling for run and homing current.
    pub max_run_current_ma: u32,
    /// VM sense divider high-side resistance in ohms.
    pub vm_divider_high_ohms: u32,
    /// VM sense divider low-side resistance in ohms.
    pub vm_divider_low_ohms: u32,
    /// TMC4671 `ADC_VM` reference span in millivolts.
    pub vm_adc_reference_millivolts: u32,
    /// Centered positive `ADC_VM` count span of the 16-bit raw ADC.
    pub vm_adc_center_counts: u32,
    /// `MOTOR_TYPE_N_POLE_PAIRS` written at commissioning and kept as the
    /// arm sentinel. Steppers use
    /// [`STEPPER_MOTOR_TYPE_N_POLE_PAIRS`](crate::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS).
    pub motor_type_n_pole_pairs: u32,
}

/// Which physical encoder drives a TMC4671 selection register.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FeedbackSource {
    /// ABN (quadrature) encoder.
    Abn,
    /// Analog or Hall-effect encoder, read through the AENC input.
    Analog,
}

/// A channel's steady-state feedback wiring (`PHI_E_SELECTION`, `POSITION_SELECTION`). Declares
/// wiring, not live register state; commissioning may transiently write other values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct FeedbackConfig {
    pub phi_e_source: FeedbackSource,
    pub position_source: FeedbackSource,
}

impl TmcConfig {
    /// PWM switching frequency in Hz (datasheet section 4.9.3):
    /// `fPWM = 4 * fCLK / (PWM_MAXCNT + 1)`
    pub const fn pwm_freq_hz(&self) -> u32 {
        4 * self.tmc_clock_hz / (self.pwm_maxcnt as u32 + 1)
    }

    /// Default torque/flux filter cutoff in Hz: `0.4 * fPWM`, capped at the ceiling.
    pub fn default_current_filter_hz(&self) -> u16 {
        ((self.pwm_freq_hz() as f32 * CURRENT_FILTER_PWM_RATIO) as u16)
            .min(crate::biquad::CURRENT_CUTOFF_MAX_HZ)
    }

    /// BBM register value: `(bbm_h << 8) | bbm_l`.
    pub const fn bbm_register(&self) -> u32 {
        foci_tmc4671::registers::PwmBbmHBbmL::new()
            .with_pwm_bbm_h(self.bbm_h)
            .with_pwm_bbm_l(self.bbm_l)
            .into_bits()
    }

    /// DSADC_MDEC_B_MDEC_A register value: group A in bits 15:0, group B in bits 31:16.
    pub const fn dsadc_mdec_register(&self) -> u32 {
        ((self.dsadc_mdec_b as u32) << 16) | (self.dsadc_mdec_a as u32)
    }

    /// Build the board's standard resistance-identification profile.
    pub const fn resistance_profile(
        &self,
        scale_metadata_validated: bool,
    ) -> crate::resistance_identification::ResistanceProfile {
        let mut profile = crate::resistance_identification::ResistanceProfile::standard();
        profile.pwm_maxcnt = self.pwm_maxcnt;
        profile.bbm_h = self.bbm_h;
        profile.bbm_l = self.bbm_l;
        profile.dsadc_mdec_a = self.dsadc_mdec_a;
        profile.dsadc_mdec_b = self.dsadc_mdec_b;
        profile.max_expected_current_count = self.resistance_max_expected_current_count;
        profile.scale_metadata_validated = scale_metadata_validated;
        profile
    }

    /// Milli-counts of `ADC_VM` per bus volt for this board's divider and ADC.
    pub const fn vm_counts_per_volt_milli(&self) -> u32 {
        crate::resistance_identification::counts_per_volt_milli(
            self.vm_adc_center_counts,
            self.vm_divider_low_ohms,
            self.vm_divider_high_ohms,
            self.vm_adc_reference_millivolts,
        )
    }

    /// Bus-voltage sense for the resistance sweep, or `None` when no `ADC_VM` offset
    /// has been captured or the board constants give a zero scale.
    pub fn bus_voltage_sense(
        &self,
        offset: Option<crate::state::tmc::AdcVmOffsetSnapshot>,
    ) -> Option<crate::resistance_identification::BusVoltageSense> {
        let offset = offset?;
        let counts_per_volt_milli = self.vm_counts_per_volt_milli();
        if counts_per_volt_milli == 0 {
            return None;
        }
        Some(crate::resistance_identification::BusVoltageSense {
            offset_raw: offset.offset_raw,
            counts_per_volt_milli,
        })
    }
}

#[cfg(test)]
impl TmcConfig {
    /// Minimal valid configuration for host-side unit tests.
    pub fn default_for_test() -> Self {
        Self {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            adc_i_select: ADC_I_SELECT_UX_I0_V_I2_WY_I1,
            excitation_voltage: 512,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
            max_run_current_ma: 10_000,
            vm_divider_high_ohms: 71_500,
            vm_divider_low_ohms: 1_500,
            vm_adc_reference_millivolts: 2_500,
            vm_adc_center_counts: 32_767,
            motor_type_n_pole_pairs: crate::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS,
        }
    }
}

#[cfg(test)]
impl FeedbackConfig {
    /// ABN on both selectors — every board's configuration today.
    pub const fn abn_abn() -> Self {
        Self {
            phi_e_source: FeedbackSource::Abn,
            position_source: FeedbackSource::Abn,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> TmcConfig {
        TmcConfig {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            adc_i_select: ADC_I_SELECT_UX_I0_V_I2_WY_I1,
            excitation_voltage: 512,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
            max_run_current_ma: 10_000,
            vm_divider_high_ohms: 71_500,
            vm_divider_low_ohms: 1_500,
            vm_adc_reference_millivolts: 2_500,
            vm_adc_center_counts: 32_767,
            motor_type_n_pole_pairs: crate::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS,
        }
    }

    #[test]
    fn production_vm_constants_give_the_spec_milli_counts_per_volt() {
        let config = test_config();
        assert_eq!(config.vm_counts_per_volt_milli(), 269_318);
    }

    #[test]
    fn bus_voltage_sense_needs_a_captured_offset() {
        let config = test_config();
        assert_eq!(config.bus_voltage_sense(None), None);
        let sense = config
            .bus_voltage_sense(Some(crate::state::tmc::AdcVmOffsetSnapshot {
                offset_raw: 33_671,
                sample_count: 8,
            }))
            .expect("offset present");
        assert_eq!(sense.offset_raw, 33_671);
        assert_eq!(sense.counts_per_volt_milli, 269_318);
    }

    #[test]
    fn pwm_freq_hz_datasheet_formula() {
        let cfg = test_config();
        assert_eq!(cfg.pwm_freq_hz(), 25_000);
    }

    #[test]
    fn current_filter_default_is_0_4_times_pwm_freq() {
        assert_eq!(test_config().default_current_filter_hz(), 10_000);
    }

    #[test]
    fn current_filter_default_scales_with_pwm_freq() {
        let mut cfg = test_config();
        cfg.pwm_maxcnt = 9999;
        assert_eq!(cfg.default_current_filter_hz(), 4_000);
    }

    #[test]
    fn current_filter_default_is_capped_at_the_biquad_ceiling() {
        let mut cfg = test_config();
        cfg.pwm_maxcnt = 1999;
        assert_eq!(
            cfg.default_current_filter_hz(),
            crate::biquad::CURRENT_CUTOFF_MAX_HZ
        );
    }

    #[test]
    fn current_filter_default_has_no_floor_below_nyquist_at_low_pwm_freq() {
        let mut cfg = test_config();
        cfg.pwm_maxcnt = 49_999;
        assert_eq!(cfg.default_current_filter_hz(), 800);
    }

    #[test]
    fn bbm_register_packs_correctly() {
        let cfg = test_config();
        assert_eq!(cfg.bbm_register(), (10 << 8) | 10);
    }

    #[test]
    fn dsadc_mdec_register_packs_correctly() {
        let cfg = test_config();
        assert_eq!(cfg.dsadc_mdec_register(), (331 << 16) | 331);
    }

    #[test]
    fn dsadc_mdec_register_packs_group_b_high_group_a_low() {
        let cfg = TmcConfig {
            pwm_maxcnt: 3999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 40,
            bbm_l: 40,
            dsadc_mdec_a: 660,
            dsadc_mdec_b: 331,
            adc_i_select: ADC_I_SELECT_UX_I0_V_I2_WY_I1,
            excitation_voltage: 512,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
            max_run_current_ma: 10_000,
            vm_divider_high_ohms: 71_500,
            vm_divider_low_ohms: 1_500,
            vm_adc_reference_millivolts: 2_500,
            vm_adc_center_counts: 32_767,
            motor_type_n_pole_pairs: crate::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS,
        };
        assert_eq!(cfg.dsadc_mdec_register(), (331 << 16) | 660);
    }

    #[test]
    fn pwm_freq_hz_at_maxcnt_999() {
        let cfg = TmcConfig {
            pwm_maxcnt: 999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            adc_i_select: ADC_I_SELECT_UX_I0_V_I2_WY_I1,
            excitation_voltage: 512,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
            max_run_current_ma: 10_000,
            vm_divider_high_ohms: 71_500,
            vm_divider_low_ohms: 1_500,
            vm_adc_reference_millivolts: 2_500,
            vm_adc_center_counts: 32_767,
            motor_type_n_pole_pairs: crate::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS,
        };
        assert_eq!(cfg.pwm_freq_hz(), 100_000);
    }

    #[test]
    fn pwm_freq_hz_at_maxcnt_1999() {
        let cfg = TmcConfig {
            pwm_maxcnt: 1999,
            tmc_clock_hz: 25_000_000,
            bbm_h: 10,
            bbm_l: 10,
            dsadc_mdec_a: 331,
            dsadc_mdec_b: 331,
            adc_i_select: ADC_I_SELECT_UX_I0_V_I2_WY_I1,
            excitation_voltage: 512,
            electrical_model_theta_e_us: 160,
            current_validation_flux_max_encoder_delta_counts: 4,
            current_validation_torque_max_encoder_delta_counts: 16,
            resistance_max_expected_current_count: 1600,
            resistance_scale_metadata_validated: false,
            current_scale_ma_per_lsb: 1.155,
            max_run_current_ma: 10_000,
            vm_divider_high_ohms: 71_500,
            vm_divider_low_ohms: 1_500,
            vm_adc_reference_millivolts: 2_500,
            vm_adc_center_counts: 32_767,
            motor_type_n_pole_pairs: crate::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS,
        };
        assert_eq!(cfg.pwm_freq_hz(), 50_000);
    }

    #[test]
    fn dsadc_mdec_from_pwm_maxcnt_uses_datasheet_formula() {
        assert_eq!(dsadc_mdec_from_pwm_maxcnt(3999), 331);
        assert_eq!(dsadc_mdec_from_pwm_maxcnt(2047), 169);
        assert_eq!(dsadc_mdec_from_pwm_maxcnt(1023), 83);
    }

    #[test]
    fn openffboard_resistance_profile_matches_power_stage() {
        let cfg = TmcConfig::default_for_test();
        let profile = cfg.resistance_profile(true);

        assert_eq!(profile.pwm_maxcnt, cfg.pwm_maxcnt);
        assert_eq!(profile.bbm_h, cfg.bbm_h);
        assert_eq!(profile.bbm_l, cfg.bbm_l);
        assert_eq!(profile.dsadc_mdec_a, cfg.dsadc_mdec_a);
        assert_eq!(profile.dsadc_mdec_b, cfg.dsadc_mdec_b);
    }

    #[test]
    fn feedback_config_abn_abn_is_abn_on_both_selectors() {
        let config = FeedbackConfig::abn_abn();
        assert_eq!(config.phi_e_source, FeedbackSource::Abn);
        assert_eq!(config.position_source, FeedbackSource::Abn);
    }
}
