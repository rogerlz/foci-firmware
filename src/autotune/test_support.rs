use super::ElectricalModel;

pub(super) fn autotune_test_config() -> crate::config::TmcConfig {
    crate::config::TmcConfig {
        pwm_maxcnt: 3999,
        tmc_clock_hz: 25_000_000,
        bbm_h: 10,
        bbm_l: 10,
        dsadc_mdec_a: 331,
        dsadc_mdec_b: 331,
        adc_i_select: crate::config::ADC_I_SELECT_UX_I0_V_I2_WY_I1,
        excitation_voltage: 512,
        electrical_model_theta_e_us: 160,
        current_validation_flux_max_encoder_delta_counts: 4,
        current_validation_torque_max_encoder_delta_counts: 16,
        resistance_max_expected_current_count: 1600,
        resistance_voltage_scale: 1,
        resistance_settle_ms: 500,
        resistance_samples_per_window: 8,
        resistance_sample_interval_ms: 5,
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

pub(super) fn test_electrical_model() -> ElectricalModel {
    ElectricalModel {
        r_count_milli: 1125,
        l_count_micro: 2950,
        tau_e_us: 2622,
        theta_e_us: 700,
        theta_source: 0,
    }
}
