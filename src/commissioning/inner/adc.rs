//! Phase 1: ADC offset calibration.
//! programs them back into `ADC_I0/I1_SCALE_OFFSET`.

use super::{
    AdcCalState, AdcResidualStage, CoilCheckState, CoilTestState, EncoderCheckState, InnerPhase,
};
use crate::autotune::{CommissionError, CommissionPhase, InnerSequence};
use crate::commissioning::types::{
    ADC_RESIDUAL_STAGE_CHOP_OFF, ADC_RESIDUAL_STAGE_CHOP_ON, AdcCalResult, AdcResidualEvidence,
    CommissioningBackend, StepResult,
};
use crate::tmc_register_values::{
    MOTION_STOPPED, PWM_CHOP_CENTERED_FOC, PWM_CHOP_OFF, adc_i0_scale_offset, adc_i1_scale_offset,
};

const ADC_CALIBRATION_SAMPLES: u16 = 128;

const ADC_CALIBRATION_SETTLE_US: u64 = 250_000;

const ADC_CALIBRATION_SAMPLE_PERIOD_US: u64 = 1_000;

const ADC_OFFSET_MIN: u16 = 30_000;
const ADC_OFFSET_MAX: u16 = 36_000;

fn residual_stage_code(stage: AdcResidualStage) -> u8 {
    match stage {
        AdcResidualStage::ChopOff => ADC_RESIDUAL_STAGE_CHOP_OFF,
        AdcResidualStage::ChopOn => ADC_RESIDUAL_STAGE_CHOP_ON,
    }
}

fn residual_stage_chop_value(stage: AdcResidualStage) -> u32 {
    match stage {
        AdcResidualStage::ChopOff => 0,
        AdcResidualStage::ChopOn => 7,
    }
}

fn signed_lower_half(raw: u32) -> i16 {
    (raw & 0xFFFF) as u16 as i16
}

fn signed_upper_half(raw: u32) -> i16 {
    ((raw >> 16) & 0xFFFF) as u16 as i16
}

fn mean_i16(sum: i32, count: u16) -> i16 {
    let count = i32::from(count.max(1));
    (sum / count).clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

struct ResidualRegisters {
    pwm_sv_chop: u8,
    pwm_bbm: u32,
    adc_i0_scale_offset: u32,
    adc_i1_scale_offset: u32,
}

struct ResidualSums {
    adc_iux: i32,
    adc_iwy: i32,
    pid_flux: i32,
    pid_torque: i32,
}

fn residual_evidence(
    stage: AdcResidualStage,
    count: u16,
    registers: ResidualRegisters,
    sums: ResidualSums,
) -> AdcResidualEvidence {
    AdcResidualEvidence {
        stage: residual_stage_code(stage),
        sample_count: count.min(u16::from(u8::MAX)) as u8,
        pwm_sv_chop: registers.pwm_sv_chop,
        pwm_bbm: registers.pwm_bbm,
        adc_i0_scale_offset: registers.adc_i0_scale_offset,
        adc_i1_scale_offset: registers.adc_i1_scale_offset,
        adc_iux_mean_count: mean_i16(sums.adc_iux, count),
        adc_iwy_mean_count: mean_i16(sums.adc_iwy, count),
        pid_flux_mean_count: mean_i16(sums.pid_flux, count),
        pid_torque_mean_count: mean_i16(sums.pid_torque, count),
    }
}

pub(crate) async fn step_adc_calibrate<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    selftest_results: &mut crate::selftest::SelftestResults,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let cal_state = match phase {
        InnerPhase::AdcCalibrate(cal_state) => cal_state,
        _ => return StepResult::ReadyNow,
    };

    match cal_state {
        AdcCalState::Setup => {
            backend.set_enable_pin(false);
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                .await
                .is_err()
                || backend
                    .write_register(PIDOUT_UQ_UD_LIMITS, 1024)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            match backend.read_register(PIDOUT_UQ_UD_LIMITS).await {
                Ok(1024) => {}
                _ => return super::fail_inner(phase, CommissionError::Spi),
            }

            if backend
                .write_register_verified(PID_TORQUE_FLUX_TARGET, 0)
                .await
                .is_err()
                || backend.write_register_verified(UQ_UD_EXT, 0).await.is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            if backend
                .write_register_verified(PWM_POLARITIES, 0)
                .await
                .is_err()
                || backend
                    .write_register_verified(PWM_MAXCNT, backend.pwm_maxcnt() as u32)
                    .await
                    .is_err()
                || backend
                    .write_register_verified(PWM_BBM_H_BBM_L, backend.bbm_register())
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            if backend
                .write_register(PWM_SV_CHOP, PWM_CHOP_OFF)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            if backend
                .read_register_verified(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                .await
                .is_err()
                || backend
                    .read_register_verified(PWM_SV_CHOP, PWM_CHOP_OFF)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            if backend
                .write_register_verified(DSADC_MDEC_B_MDEC_A, backend.dsadc_mdec_register())
                .await
                .is_err()
                || backend
                    .write_register_verified(ADC_I_SELECT, backend.tmc_config().adc_i_select)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let motor_type_n_pole_pairs = backend.tmc_config().motor_type_n_pole_pairs;
            if backend
                .write_register_verified(MOTOR_TYPE_N_POLE_PAIRS, motor_type_n_pole_pairs)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            backend.store_motor_sentinel(motor_type_n_pole_pairs);

            if backend
                .write_register_verified(ADC_I0_SCALE_OFFSET, adc_i0_scale_offset(0))
                .await
                .is_err()
                || backend
                    .write_register_verified(ADC_I1_SCALE_OFFSET, adc_i1_scale_offset(0))
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            backend.set_enable_pin(true);

            let deadline = match backend.elapsed_us().checked_add(ADC_CALIBRATION_SETTLE_US) {
                Some(deadline) => deadline,
                None => return super::fail_inner(phase, CommissionError::InvalidSchedule),
            };
            *cal_state = AdcCalState::Settle {
                deadline_us: deadline,
            };
            StepResult::At(deadline)
        }

        AdcCalState::Settle { deadline_us } => {
            let now_us = backend.elapsed_us();
            if now_us < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            let next_sample_us = match now_us.checked_add(ADC_CALIBRATION_SAMPLE_PERIOD_US) {
                Some(deadline) => deadline,
                None => return super::fail_inner(phase, CommissionError::InvalidSchedule),
            };
            *cal_state = AdcCalState::Sample {
                sum_i0: 0,
                sum_i1: 0,
                count: 0,
                next_sample_us,
            };
            StepResult::At(next_sample_us)
        }

        AdcCalState::Sample {
            sum_i0,
            sum_i1,
            count,
            next_sample_us,
        } => {
            let now_us = backend.elapsed_us();
            if now_us < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }
            match backend.read_register(ADC_IWY_IUX).await {
                Ok(raw) => {
                    let i_ux = (raw & 0xFFFF) as i16 as i32;
                    let i_wy = ((raw >> 16) & 0xFFFF) as i16 as i32;
                    *sum_i0 += i_ux;
                    *sum_i1 += i_wy;
                    *count += 1;
                }
                Err(_) => {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }
            if *count >= ADC_CALIBRATION_SAMPLES {
                *cal_state = AdcCalState::Finalize {
                    sum_i0: *sum_i0,
                    sum_i1: *sum_i1,
                };
                StepResult::ReadyNow
            } else {
                let next_deadline = match crate::commissioning::schedule::next_grid_deadline(
                    *next_sample_us,
                    ADC_CALIBRATION_SAMPLE_PERIOD_US,
                    now_us,
                ) {
                    Ok(deadline) => deadline,
                    Err(error) => return super::fail_inner(phase, error),
                };
                *next_sample_us = next_deadline;
                StepResult::At(next_deadline)
            }
        }

        AdcCalState::Finalize { sum_i0, sum_i1 } => {
            let n = ADC_CALIBRATION_SAMPLES as i32;
            let offset_i0 = (*sum_i0 / n) as i16 as u16;
            let offset_i1 = (*sum_i1 / n) as i16 as u16;

            if !(ADC_OFFSET_MIN..=ADC_OFFSET_MAX).contains(&offset_i0)
                || !(ADC_OFFSET_MIN..=ADC_OFFSET_MAX).contains(&offset_i1)
            {
                if matches!(sequence, InnerSequence::Selftest) {
                    selftest_results.set(
                        crate::selftest::SelftestStage::AdcCalibration,
                        crate::selftest::SelftestStageValue::fail(0),
                    );
                }
                return super::fail_inner(phase, CommissionError::AdcFault);
            }

            if matches!(sequence, InnerSequence::Selftest) {
                selftest_results.set(
                    crate::selftest::SelftestStage::AdcCalibration,
                    crate::selftest::SelftestStageValue::adc_offsets(offset_i0, offset_i1),
                );
            }

            let adc_i0_word = adc_i0_scale_offset(offset_i0);
            let adc_i1_word = adc_i1_scale_offset(offset_i1);
            if backend
                .write_register_verified(ADC_I0_SCALE_OFFSET, adc_i0_word)
                .await
                .is_err()
                || backend
                    .write_register_verified(ADC_I1_SCALE_OFFSET, adc_i1_word)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            if backend
                .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                .await
                .is_err()
                || backend
                    .write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .is_err()
                || backend.write_register(UQ_UD_EXT, 0).await.is_err()
                || backend
                    .write_register_verified(PWM_SV_CHOP, PWM_CHOP_OFF)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let deadline = match backend.elapsed_us().checked_add(ADC_CALIBRATION_SETTLE_US) {
                Some(deadline) => deadline,
                None => return super::fail_inner(phase, CommissionError::InvalidSchedule),
            };
            *cal_state = AdcCalState::ResidualSettle {
                stage: AdcResidualStage::ChopOff,
                offset_i0,
                offset_i1,
                chop0: None,
                deadline_us: deadline,
            };
            StepResult::At(deadline)
        }

        AdcCalState::ResidualSettle {
            stage,
            offset_i0,
            offset_i1,
            chop0,
            deadline_us,
        } => {
            let now_us = backend.elapsed_us();
            if now_us < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            let pwm_sv_chop = match backend.read_register(PWM_SV_CHOP).await {
                Ok(value) if value == residual_stage_chop_value(*stage) => value as u8,
                _ => return super::fail_inner(phase, CommissionError::Spi),
            };
            let pwm_bbm = match backend.read_register(PWM_BBM_H_BBM_L).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let adc_i0_scale_offset = match backend.read_register(ADC_I0_SCALE_OFFSET).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let adc_i1_scale_offset = match backend.read_register(ADC_I1_SCALE_OFFSET).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let next_sample_us = match now_us.checked_add(ADC_CALIBRATION_SAMPLE_PERIOD_US) {
                Some(deadline) => deadline,
                None => return super::fail_inner(phase, CommissionError::InvalidSchedule),
            };
            *cal_state = AdcCalState::ResidualSample {
                stage: *stage,
                offset_i0: *offset_i0,
                offset_i1: *offset_i1,
                adc_i0_scale_offset,
                adc_i1_scale_offset,
                pwm_sv_chop,
                pwm_bbm,
                adc_iux_sum: 0,
                adc_iwy_sum: 0,
                pid_flux_sum: 0,
                pid_torque_sum: 0,
                count: 0,
                chop0: *chop0,
                next_sample_us,
            };
            StepResult::At(next_sample_us)
        }

        AdcCalState::ResidualSample {
            stage,
            offset_i0,
            offset_i1,
            adc_i0_scale_offset,
            adc_i1_scale_offset,
            pwm_sv_chop,
            pwm_bbm,
            adc_iux_sum,
            adc_iwy_sum,
            pid_flux_sum,
            pid_torque_sum,
            count,
            chop0,
            next_sample_us,
        } => {
            let now_us = backend.elapsed_us();
            if now_us < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }
            let adc_raw = match backend.read_register(ADC_IWY_IUX).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let pid_raw = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                Ok(value) => value,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            *adc_iux_sum += i32::from(signed_lower_half(adc_raw));
            *adc_iwy_sum += i32::from(signed_upper_half(adc_raw));
            *pid_flux_sum += i32::from(signed_lower_half(pid_raw));
            *pid_torque_sum += i32::from(signed_upper_half(pid_raw));
            *count += 1;

            if *count < ADC_CALIBRATION_SAMPLES {
                let next_deadline = match crate::commissioning::schedule::next_grid_deadline(
                    *next_sample_us,
                    ADC_CALIBRATION_SAMPLE_PERIOD_US,
                    now_us,
                ) {
                    Ok(deadline) => deadline,
                    Err(error) => return super::fail_inner(phase, error),
                };
                *next_sample_us = next_deadline;
                return StepResult::At(next_deadline);
            }

            let evidence = residual_evidence(
                *stage,
                *count,
                ResidualRegisters {
                    pwm_sv_chop: *pwm_sv_chop,
                    pwm_bbm: *pwm_bbm,
                    adc_i0_scale_offset: *adc_i0_scale_offset,
                    adc_i1_scale_offset: *adc_i1_scale_offset,
                },
                ResidualSums {
                    adc_iux: *adc_iux_sum,
                    adc_iwy: *adc_iwy_sum,
                    pid_flux: *pid_flux_sum,
                    pid_torque: *pid_torque_sum,
                },
            );

            match *stage {
                AdcResidualStage::ChopOff => {
                    if backend
                        .write_register_verified(PWM_SV_CHOP, PWM_CHOP_CENTERED_FOC)
                        .await
                        .is_err()
                    {
                        return super::fail_inner(phase, CommissionError::Spi);
                    }
                    let deadline = match backend.elapsed_us().checked_add(ADC_CALIBRATION_SETTLE_US)
                    {
                        Some(deadline) => deadline,
                        None => {
                            return super::fail_inner(phase, CommissionError::InvalidSchedule);
                        }
                    };
                    *cal_state = AdcCalState::ResidualSettle {
                        stage: AdcResidualStage::ChopOn,
                        offset_i0: *offset_i0,
                        offset_i1: *offset_i1,
                        chop0: Some(evidence),
                        deadline_us: deadline,
                    };
                    StepResult::At(deadline)
                }
                AdcResidualStage::ChopOn => {
                    let encoder_count = backend.read_mechanical_encoder_position_counts().await;
                    let result = AdcCalResult {
                        adc_i0: *offset_i0,
                        adc_i1: *offset_i1,
                        encoder_count,
                        encoder_alignment_electrical_residual_counts: 0,
                        encoder_alignment_stability_counts: 0,
                        encoder_alignment_movement_counts: 0,
                        encoder_alignment_min_movement_counts: 0,
                        encoder_alignment_counts_per_electrical_rev: 0,
                        adc_residual_chop0: chop0.unwrap_or(AdcResidualEvidence::EMPTY),
                        adc_residual_chop7: evidence,
                    };
                    *adc_cal = Some(result.clone());

                    match sequence {
                        InnerSequence::Calibrate => {
                            *phase = InnerPhase::EncoderCheck {
                                state: EncoderCheckState::Setup,
                            };
                            StepResult::PhaseChanged(CommissionPhase::EncoderCheck)
                        }
                        InnerSequence::Selftest | InnerSequence::Commission => {
                            *phase = InnerPhase::CoilCheck {
                                state: CoilCheckState::CoilA(CoilTestState::Setup { phase: 0 }),
                                coil_a: None,
                            };
                            StepResult::PhaseChanged(CommissionPhase::CoilCheck)
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use crate::commissioning::types::StepResult;

    const ADC_CALIBRATION_MAX_REGISTER_OPS_PER_ARM: usize = 26;

    fn offset_sum(offset: u16) -> i32 {
        i32::from(offset as i16) * i32::from(ADC_CALIBRATION_SAMPLES)
    }

    fn pack_signed_halves(low: i16, high: i16) -> u32 {
        ((high as u16 as u32) << 16) | low as u16 as u32
    }

    fn register_op_count(backend: &MockBackend) -> usize {
        backend
            .spi_log
            .iter()
            .filter(|op| !matches!(op, SpiOp::EnablePin(_)))
            .count()
    }

    #[test]
    fn setup_fails_closed_when_a_pre_enable_write_fails() {
        use foci_tmc4671::registers::*;

        let pre_enable_registers = [
            PID_TORQUE_FLUX_TARGET.addr(),
            UQ_UD_EXT.addr(),
            PWM_POLARITIES.addr(),
            PWM_MAXCNT.addr(),
            PWM_BBM_H_BBM_L.addr(),
            DSADC_MDEC_B_MDEC_A.addr(),
            ADC_I_SELECT.addr(),
            ADC_I0_SCALE_OFFSET.addr(),
            ADC_I1_SCALE_OFFSET.addr(),
        ];

        for addr in pre_enable_registers {
            pollster::block_on(async {
                let mut backend = MockBackend::new();
                backend.fail_on_write = Some(addr);
                let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Setup);
                let mut adc_cal = None;
                let mut selftest_results = crate::selftest::SelftestResults::default();

                let result = step_adc_calibrate(
                    &mut phase,
                    InnerSequence::Calibrate,
                    &mut adc_cal,
                    &mut selftest_results,
                    &mut backend,
                )
                .await;

                assert!(
                    matches!(result, StepResult::InnerFailed(CommissionError::Spi)),
                    "register 0x{addr:02x} write failure must fail the setup arm, got {result:?}"
                );
                assert!(
                    !backend.enable_pin,
                    "register 0x{addr:02x} write failure must not leave the motor enabled"
                );
            });
        }
    }

    #[test]
    fn setup_writes_board_configured_adc_i_select() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.tmc_config.adc_i_select = 0x1234_5678;
            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Setup);
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(ADC_CALIBRATION_SETTLE_US)));
            assert!(register_op_count(&backend) <= ADC_CALIBRATION_MAX_REGISTER_OPS_PER_ARM);
            assert!(
                backend.spi_log.iter().any(|op| matches!(
                    op,
                    SpiOp::Write(addr, 0x1234_5678)
                        if *addr == foci_tmc4671::registers::ADC_I_SELECT.addr()
                )),
                "ADC setup must use the board-provided ADC_I_SELECT value"
            );
        });
    }

    #[test]
    fn adc_settle_reports_its_deadline_and_arms_the_sample_grid_when_due() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 100;
            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Settle {
                deadline_us: 250_000,
            });
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let early = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;
            assert!(matches!(early, StepResult::At(250_000)));
            assert_eq!(register_op_count(&backend), 0);

            backend.elapsed_us_value = 250_000;
            let due = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;
            assert!(matches!(due, StepResult::At(251_000)));
            assert!(matches!(
                phase,
                InnerPhase::AdcCalibrate(AdcCalState::Sample {
                    next_sample_us: 251_000,
                    ..
                })
            ));
        });
    }

    #[test]
    fn late_adc_offset_sample_reads_once_and_skips_to_the_future_grid() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 3_500;
            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Sample {
                sum_i0: 0,
                sum_i1: 0,
                count: 0,
                next_sample_us: 1_000,
            });
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(4_000)));
            assert_eq!(register_op_count(&backend), 1);
            assert!(matches!(
                phase,
                InnerPhase::AdcCalibrate(AdcCalState::Sample {
                    count: 1,
                    next_sample_us: 4_000,
                    ..
                })
            ));
        });
    }

    #[test]
    fn late_adc_residual_sample_reads_one_bundle_and_skips_elapsed_slots() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 3_500;
            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::ResidualSample {
                stage: AdcResidualStage::ChopOff,
                offset_i0: 33_000,
                offset_i1: 33_100,
                adc_i0_scale_offset: (256 << 16) | 33_000,
                adc_i1_scale_offset: (256 << 16) | 33_100,
                pwm_sv_chop: 0,
                pwm_bbm: 0,
                adc_iux_sum: 0,
                adc_iwy_sum: 0,
                pid_flux_sum: 0,
                pid_torque_sum: 0,
                count: 0,
                chop0: None,
                next_sample_us: 1_000,
            });
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(4_000)));
            assert_eq!(register_op_count(&backend), 2);
            assert!(matches!(
                phase,
                InnerPhase::AdcCalibrate(AdcCalState::ResidualSample {
                    count: 1,
                    next_sample_us: 4_000,
                    ..
                })
            ));
        });
    }

    #[test]
    fn adc_cal_stores_sentinel_after_motor_type_write() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Setup);
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(ADC_CALIBRATION_SETTLE_US)));
            assert_eq!(
                backend.last_motor_sentinel_stored.get(),
                Some((2 << 16) | 50),
            );
        });
    }

    #[test]
    fn finalize_fails_when_programmed_adc_offset_does_not_verify() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.read_override = Some((
                foci_tmc4671::registers::ADC_I0_SCALE_OFFSET.addr(),
                (256 << 16) | 32_999,
            ));
            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Finalize {
                sum_i0: offset_sum(33_000),
                sum_i1: offset_sum(33_100),
            });
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let result = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(adc_cal.is_none());
        });
    }

    #[test]
    fn adc_cal_records_open_loop_residuals_before_and_after_chop_restore() {
        pollster::block_on(async {
            use crate::commissioning::types::{
                ADC_RESIDUAL_STAGE_CHOP_OFF, ADC_RESIDUAL_STAGE_CHOP_ON,
            };
            use foci_tmc4671::registers::{ADC_IWY_IUX, PID_TORQUE_FLUX_ACTUAL, PWM_BBM_H_BBM_L};

            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(ADC_IWY_IUX.addr(), pack_signed_halves(12, -34))
                .unwrap();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), pack_signed_halves(5, -7))
                .unwrap();
            backend
                .registers
                .insert(PWM_BBM_H_BBM_L.addr(), 0x0009_0009)
                .unwrap();

            let mut phase = InnerPhase::AdcCalibrate(AdcCalState::Finalize {
                sum_i0: offset_sum(33_000),
                sum_i1: offset_sum(33_100),
            });
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();

            let mut scheduled = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;
            assert!(matches!(scheduled, StepResult::At(250_000)));
            assert!(register_op_count(&backend) <= ADC_CALIBRATION_MAX_REGISTER_OPS_PER_ARM);
            assert!(adc_cal.is_none());

            backend.elapsed_us_value = 250_000;
            backend.spi_log.clear();
            scheduled = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;
            assert!(matches!(scheduled, StepResult::At(251_000)));
            assert!(register_op_count(&backend) <= ADC_CALIBRATION_MAX_REGISTER_OPS_PER_ARM);

            for _ in 0..ADC_CALIBRATION_SAMPLES {
                let StepResult::At(deadline_us) = scheduled else {
                    panic!("residual sampling should retain an absolute schedule");
                };
                backend.elapsed_us_value = deadline_us;
                backend.spi_log.clear();
                scheduled = step_adc_calibrate(
                    &mut phase,
                    InnerSequence::Calibrate,
                    &mut adc_cal,
                    &mut selftest_results,
                    &mut backend,
                )
                .await;
                assert!(register_op_count(&backend) <= ADC_CALIBRATION_MAX_REGISTER_OPS_PER_ARM);
            }

            let StepResult::At(deadline_us) = scheduled else {
                panic!("chop restore should enter a scheduled settle");
            };
            backend.elapsed_us_value = deadline_us;
            backend.spi_log.clear();
            scheduled = step_adc_calibrate(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut selftest_results,
                &mut backend,
            )
            .await;
            assert!(matches!(scheduled, StepResult::At(_)));
            assert!(register_op_count(&backend) <= ADC_CALIBRATION_MAX_REGISTER_OPS_PER_ARM);

            for _ in 0..ADC_CALIBRATION_SAMPLES {
                let StepResult::At(deadline_us) = scheduled else {
                    panic!("residual sampling should retain an absolute schedule");
                };
                backend.elapsed_us_value = deadline_us;
                backend.spi_log.clear();
                scheduled = step_adc_calibrate(
                    &mut phase,
                    InnerSequence::Calibrate,
                    &mut adc_cal,
                    &mut selftest_results,
                    &mut backend,
                )
                .await;
                assert!(register_op_count(&backend) <= ADC_CALIBRATION_MAX_REGISTER_OPS_PER_ARM);
            }

            assert!(matches!(
                scheduled,
                StepResult::PhaseChanged(CommissionPhase::EncoderCheck)
            ));
            let cal = adc_cal.expect("ADC calibration result should be recorded");
            assert_eq!(cal.adc_residual_chop0.stage, ADC_RESIDUAL_STAGE_CHOP_OFF);
            assert_eq!(cal.adc_residual_chop0.sample_count, 128);
            assert_eq!(cal.adc_residual_chop0.pwm_sv_chop, 0);
            assert_eq!(cal.adc_residual_chop0.pwm_bbm, 0x0009_0009);
            assert_eq!(
                cal.adc_residual_chop0.adc_i0_scale_offset,
                (256 << 16) | 33_000
            );
            assert_eq!(
                cal.adc_residual_chop0.adc_i1_scale_offset,
                (256 << 16) | 33_100
            );
            assert_eq!(cal.adc_residual_chop0.adc_iux_mean_count, 12);
            assert_eq!(cal.adc_residual_chop0.adc_iwy_mean_count, -34);
            assert_eq!(cal.adc_residual_chop0.pid_flux_mean_count, 5);
            assert_eq!(cal.adc_residual_chop0.pid_torque_mean_count, -7);

            assert_eq!(cal.adc_residual_chop7.stage, ADC_RESIDUAL_STAGE_CHOP_ON);
            assert_eq!(cal.adc_residual_chop7.sample_count, 128);
            assert_eq!(cal.adc_residual_chop7.pwm_sv_chop, 7);
            assert_eq!(cal.adc_residual_chop7.adc_iux_mean_count, 12);
            assert_eq!(cal.adc_residual_chop7.pid_flux_mean_count, 5);
        });
    }
}
