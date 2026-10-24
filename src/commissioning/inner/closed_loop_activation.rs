//! Phase 8: Closed-loop entry.

use super::InnerPhase;
use crate::autotune::{
    CLOSED_LOOP_ACTIVATION_STATUS_FAIL_DRIFT, CLOSED_LOOP_ACTIVATION_STATUS_FAIL_RUNAWAY,
    CLOSED_LOOP_ACTIVATION_STATUS_PASS, CLOSED_LOOP_ACTIVATION_STATUS_WARN_DRIFT,
    ClosedLoopActivationState, CommissionError, CommissionStatus, HOLD_GATE_DWELL_US,
    HOLD_GATE_MAX_SAMPLES, HOLD_GATE_SAMPLE_PERIOD_US, HOLD_GATE_STATUS_PASS, HoldGateAccumulator,
    HoldGateMetrics, HoldGateSample, InnerSequence, hold_status_failure_reason,
};
use crate::commissioning::types::{
    AdcCalResult, AdcResidualEvidence, CommissionResult, CommissioningBackend, InnerResult,
    StepResult,
};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics};
use crate::tmc::{decode_signed_lower_half, decode_signed_upper_half};
use crate::tmc_register_values::{
    MOTION_POSITION_ADVANCED_PI, MOTION_STOPPED, PWM_CHOP_CENTERED_FOC,
};
use foci_tmc4671::pid::{PidIGain, PidPGain};

const CLOSED_LOOP_VELOCITY_SELECTION: u32 = 0x0000_0009;
pub(crate) const STANDARD_VELOCITY_METER_COUNTS_PER_MINUTE: u32 = 262_144;
pub(crate) const CLOSED_LOOP_POSITION_SELECTION: u32 = 9;
const CLOSED_LOOP_PHI_E_SELECTION: u32 = 3;
const CLOSED_LOOP_ACTIVATION_STABILITY_THRESHOLD: u32 = 2;
const CLOSED_LOOP_ACTIVATION_BOUNDED_DRIFT_WARN_THRESHOLD: u32 = 64;
const CLOSED_LOOP_ACTIVATION_RUNAWAY_THRESHOLD: u32 = 32760;

const CLOSED_LOOP_ACTIVATION_MAX_ATTEMPTS: u8 = 3;

const CLOSED_LOOP_ACTIVATION_VELOCITY_QUIET_RPM: u32 = 4;
const CLOSED_LOOP_ACTIVATION_VELOCITY_QUIET_SAMPLES: u8 = 3;
const CLOSED_LOOP_ACTIVATION_VELOCITY_SAMPLE_PERIOD_US: u64 = 2_000;
const CLOSED_LOOP_ACTIVATION_VELOCITY_SETTLE_CAP_US: u64 = 100_000;

const DIAG_CLOSED_LOOP_ACTIVATION_RESULT: u8 = 1;

fn push_closed_loop_activation_result(
    diagnostics: &mut CommissionDiagnostics,
    entry_status: u8,
    position_1: i32,
    position_2: i32,
    drift: u32,
) {
    diagnostics.push(CommissionDiagnostic {
        phase: crate::autotune::CommissionPhase::ClosedLoopActivation.to_u8(),
        code: DIAG_CLOSED_LOOP_ACTIVATION_RESULT,
        status: entry_status,
        value0: position_1 as u32,
        value1: position_2 as u32,
        value2: drift,
    });
}

fn gain_registers(
    inner: crate::pid::InnerLoopGains,
    outer: crate::pid::OuterLoopGains,
) -> (u32, u32, u32, u32) {
    let (flux_reg, torque_reg) = inner.to_registers();
    let (velocity_reg, position_reg) = outer.to_registers();
    (flux_reg, torque_reg, velocity_reg, position_reg)
}

async fn verify_closed_loop_armed_state<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), ()> {
    use foci_tmc4671::registers::*;

    let physical_step_width = match backend.configured_physical_step_width() {
        Some(value) if value != 0 => value,
        _ => {
            defmt::error!("closed-loop: configured physical STEP_WIDTH is missing or zero");
            return Err(());
        }
    };
    let encoder_ppr = backend.mechanical_encoder_quadrature_counts_per_rev();
    if encoder_ppr == 0 {
        defmt::error!("closed-loop: ABN_DECODER_PPR expected value is zero");
        return Err(());
    }

    let checks = [
        (PIDOUT_UQ_UD_LIMITS, backend.operational_voltage_limit()),
        (PWM_SV_CHOP, PWM_CHOP_CENTERED_FOC),
        (MODE_RAMP_MODE_MOTION, MOTION_POSITION_ADVANCED_PI),
        (
            MOTOR_TYPE_N_POLE_PAIRS,
            backend.tmc_config().motor_type_n_pole_pairs,
        ),
        (PHI_E_SELECTION, CLOSED_LOOP_PHI_E_SELECTION),
        (POSITION_SELECTION, CLOSED_LOOP_POSITION_SELECTION),
        (VELOCITY_SELECTION, CLOSED_LOOP_VELOCITY_SELECTION),
        (ABN_DECODER_PPR, encoder_ppr),
        (ADC_I_SELECT, backend.tmc_config().adc_i_select),
        (
            PID_TORQUE_FLUX_LIMITS,
            backend.current_limit().register_value(),
        ),
        (STEP_WIDTH, physical_step_width),
    ];

    for (register, expected) in checks {
        match backend.read_register(register).await {
            Ok(value) if value == expected => {}
            Ok(value) => {
                defmt::error!(
                    "closed-loop: register {} readback {}, expected {}",
                    register.addr(),
                    value,
                    expected
                );
                return Err(());
            }
            Err(_) => {
                defmt::error!("closed-loop: register {} read failed", register.addr());
                return Err(());
            }
        }
    }

    Ok(())
}

fn saturating_u8(value: u16) -> u8 {
    u8::try_from(value).unwrap_or(u8::MAX)
}

fn saturating_u16(value: u32) -> u16 {
    u16::try_from(value).unwrap_or(u16::MAX)
}

fn copy_hold_metrics(inner: &mut InnerResult, metrics: HoldGateMetrics) {
    inner.current_hold_status = metrics.status;
    inner.current_hold_warning_flags = metrics.warning_flags;
    inner.current_hold_elapsed_us = metrics.elapsed_us;
    inner.current_hold_requested_sample_period_us = metrics.requested_sample_period_us;
    inner.current_hold_sample_count = saturating_u8(metrics.sample_count);
    inner.current_hold_position_span_count = saturating_u16(metrics.position_span_count);
    inner.current_hold_position_drift_count = saturating_u16(metrics.position_drift_count);
    inner.current_hold_torque_mean_count = metrics.torque_mean_count;
    inner.current_hold_torque_rms_count = metrics.torque_rms_count;
    inner.current_hold_torque_peak_to_peak_count = metrics.torque_peak_to_peak_count;
    inner.current_hold_torque_crossing_count = saturating_u8(metrics.torque_crossing_count);
    inner.current_hold_flux_mean_count = metrics.flux_mean_count;
    inner.current_hold_flux_rms_count = metrics.flux_rms_count;
    inner.current_hold_flux_peak_to_peak_count = metrics.flux_peak_to_peak_count;
    inner.current_hold_flux_crossing_count = saturating_u8(metrics.flux_crossing_count);
    inner.current_hold_status_flags_or = metrics.status_flags_or;
    inner.current_hold_actionable_status_count = saturating_u8(metrics.actionable_status_count);
}

#[derive(Clone, Copy)]
struct FallbackTerminalState {
    velocity_p: PidPGain,
    velocity_i: PidIGain,
    position_p: PidPGain,
    position_i: PidIGain,
    velocity_limit: u32,
    velocity_filter_hz: u16,
    torque_filter_hz: u16,
    position_filter_hz: u16,
    flux_filter_hz: u16,
}

fn finish_commission(inner: InnerResult, terminal: FallbackTerminalState) -> StepResult {
    StepResult::CommissionDone(CommissionResult {
        inner,
        fallback_velocity_p: terminal.velocity_p.raw(),
        fallback_velocity_i: terminal.velocity_i.raw(),
        fallback_position_p: terminal.position_p.raw(),
        fallback_position_i: terminal.position_i.raw(),
        fallback_velocity_limit: terminal.velocity_limit,
        velocity_filter_hz: terminal.velocity_filter_hz,
        torque_filter_hz: terminal.torque_filter_hz,
        position_filter_hz: terminal.position_filter_hz,
        flux_filter_hz: terminal.flux_filter_hz,
    })
}

async fn fail_sustained_hold<B: CommissioningBackend>(
    mut inner: InnerResult,
    backend: &mut B,
    failure_reason: u8,
    terminal: FallbackTerminalState,
) -> StepResult {
    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
    inner.status = CommissionStatus::Failed(CommissionError::CurrentHoldFailed);
    inner.current_loop_failure_reason = failure_reason;
    finish_commission(inner, terminal)
}

fn copy_closed_loop_activation_metrics(
    inner: &mut InnerResult,
    entry_status: u8,
    position_1: i32,
    position_2: i32,
    drift: u32,
    runaway: bool,
) {
    inner.closed_loop_activation_status = entry_status;
    inner.closed_loop_activation_position_1 = position_1;
    inner.closed_loop_activation_position_2 = position_2;
    inner.closed_loop_activation_drift_count = saturating_u16(drift);
    inner.closed_loop_activation_threshold_count =
        CLOSED_LOOP_ACTIVATION_STABILITY_THRESHOLD as u16;
    inner.closed_loop_activation_runaway = u8::from(runaway);
}

struct ActivationFailureMetrics {
    entry_status: u8,
    position_1: i32,
    position_2: i32,
    drift: u32,
    runaway: bool,
}

async fn fail_closed_loop_activation<B: CommissioningBackend>(
    inner_result: &mut Option<InnerResult>,
    backend: &mut B,
    metrics: ActivationFailureMetrics,
    terminal: FallbackTerminalState,
) -> Option<StepResult> {
    let mut inner = inner_result.take()?;

    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
    inner.status = CommissionStatus::Failed(CommissionError::ClosedLoopActivationUnstable);
    inner.current_loop_failure_reason =
        crate::autotune::CURRENT_LOOP_FAILURE_CLOSED_LOOP_ACTIVATION;
    copy_closed_loop_activation_metrics(
        &mut inner,
        metrics.entry_status,
        metrics.position_1,
        metrics.position_2,
        metrics.drift,
        metrics.runaway,
    );
    Some(finish_commission(inner, terminal))
}

pub(crate) async fn step_closed_loop_activation<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (
        cle_state,
        flux_p,
        flux_i,
        torque_p,
        torque_i,
        velocity_p,
        velocity_i,
        position_p,
        position_i,
        velocity_limit,
        velocity_filter_hz,
        torque_filter_hz,
        position_filter_hz,
        flux_filter_hz,
        inner_result,
        attempt,
    ) = match phase {
        InnerPhase::ClosedLoopActivation {
            state,
            flux_p,
            flux_i,
            torque_p,
            torque_i,
            velocity_p,
            velocity_i,
            position_p,
            position_i,
            velocity_limit,
            velocity_filter_hz,
            torque_filter_hz,
            position_filter_hz,
            flux_filter_hz,
            inner_result,
            attempt,
        } => (
            state,
            *flux_p,
            *flux_i,
            *torque_p,
            *torque_i,
            *velocity_p,
            *velocity_i,
            *position_p,
            *position_i,
            *velocity_limit,
            *velocity_filter_hz,
            *torque_filter_hz,
            *position_filter_hz,
            *flux_filter_hz,
            inner_result,
            *attempt,
        ),
        _ => return StepResult::ReadyNow,
    };

    match cle_state {
        ClosedLoopActivationState::WriteGains => {
            let (flux_reg, torque_reg, velocity_reg, position_reg) = gain_registers(
                crate::pid::InnerLoopGains {
                    flux_p,
                    flux_i,
                    torque_p,
                    torque_i,
                },
                crate::pid::OuterLoopGains {
                    position_p,
                    position_i,
                    velocity_p,
                    velocity_i,
                },
            );
            if backend
                .write_register(PID_FLUX_P_FLUX_I, flux_reg)
                .await
                .is_err()
                || backend
                    .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            if backend
                .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            if backend
                .write_register(PID_POSITION_P_POSITION_I, position_reg)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            *cle_state = ClosedLoopActivationState::WriteLimits;
            StepResult::ReadyNow
        }

        ClosedLoopActivationState::WriteLimits => {
            if backend
                .write_register(PID_VELOCITY_LIMIT, velocity_limit)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let voltage_limit = backend.operational_voltage_limit();
            if backend
                .write_register(PIDOUT_UQ_UD_LIMITS, voltage_limit)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            let voltage_limit_readback = backend.read_register(PIDOUT_UQ_UD_LIMITS).await;
            match voltage_limit_readback {
                Ok(value) if value == voltage_limit => {}
                _ => {
                    defmt::error!("closed-loop: PIDOUT_UQ_UD_LIMITS readback mismatch");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }

            *cle_state = ClosedLoopActivationState::VerifyPhiE;
            StepResult::ReadyNow
        }

        ClosedLoopActivationState::VerifyPhiE => {
            if backend
                .write_register(PHI_E_SELECTION, CLOSED_LOOP_PHI_E_SELECTION)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            match backend.read_register(PHI_E_SELECTION).await {
                Ok(CLOSED_LOOP_PHI_E_SELECTION) => {}
                Ok(val) => {
                    defmt::error!("closed-loop: PHI_E_SELECTION={}, expected 3", val);
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                Err(_) => {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }

            *cle_state = ClosedLoopActivationState::ZeroAndClear;
            StepResult::ReadyNow
        }

        ClosedLoopActivationState::ZeroAndClear => {
            if backend
                .write_register(PID_POSITION_ACTUAL, 0)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let integrators_ok = async {
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_TORQUE_ERROR_SUM,
                    0,
                )
                .await
                .map_err(|_| CommissionError::Spi)?;
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_FLUX_ERROR_SUM,
                    0,
                )
                .await
                .map_err(|_| CommissionError::Spi)?;
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_VELOCITY_ERROR_SUM,
                    0,
                )
                .await
                .map_err(|_| CommissionError::Spi)?;
                crate::commissioning::helpers::write_sub_register_checked(
                    backend,
                    PID_ERROR_PID_POSITION_ERROR_SUM,
                    0,
                )
                .await
                .map_err(|_| CommissionError::Spi)?;
                crate::tmc_control::interlock::clear_status_flags_via_backend(backend).await
            };
            if let Err(error) = integrators_ok.await {
                return super::fail_inner(phase, error);
            }

            *cle_state = ClosedLoopActivationState::WritePositionConfig;
            StepResult::ReadyNow
        }

        ClosedLoopActivationState::WritePositionConfig => {
            if backend
                .write_register(VELOCITY_SELECTION, CLOSED_LOOP_VELOCITY_SELECTION)
                .await
                .is_err()
                || backend
                    .write_register(POSITION_SELECTION, CLOSED_LOOP_POSITION_SELECTION)
                    .await
                    .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            match backend.read_register(POSITION_SELECTION).await {
                Ok(CLOSED_LOOP_POSITION_SELECTION) => {}
                _ => {
                    defmt::error!("closed-loop: POSITION_SELECTION readback mismatch");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }

            let physical_step_width = match backend.configured_physical_step_width() {
                Some(value) if value != 0 => value,
                _ => {
                    defmt::error!("closed-loop: configured physical STEP_WIDTH is missing or zero");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };
            if backend
                .write_register(STEP_WIDTH, physical_step_width)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            match backend.read_register(STEP_WIDTH).await {
                Ok(val) if val == physical_step_width => {}
                _ => {
                    defmt::error!("closed-loop: STEP_WIDTH readback mismatch");
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            }
            defmt::info!("STEP_WIDTH(0x78) = {} written", physical_step_width);

            *cle_state = ClosedLoopActivationState::WriteFilters;
            StepResult::ReadyNow
        }

        ClosedLoopActivationState::WriteFilters => {
            let pwm_maxcnt = backend.pwm_maxcnt();

            if velocity_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_velocity(
                    backend,
                    velocity_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_V_ENABLE,
                    0,
                )
                .await;
            }

            if torque_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_torque(
                    backend,
                    torque_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_T_ENABLE,
                    0,
                )
                .await;
            }

            if position_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_position(
                    backend,
                    position_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_X_ENABLE,
                    0,
                )
                .await;
            }

            if flux_filter_hz > 0 {
                crate::commissioning::helpers::write_biquad_flux(
                    backend,
                    flux_filter_hz,
                    pwm_maxcnt,
                )
                .await;
            } else {
                crate::commissioning::helpers::write_sub_register(
                    backend,
                    CONFIG_BIQUAD_F_ENABLE,
                    0,
                )
                .await;
            }

            *cle_state = ClosedLoopActivationState::EnterPositionMode;
            StepResult::ReadyNow
        }

        ClosedLoopActivationState::EnterPositionMode => {
            if backend
                .write_register(MODE_RAMP_MODE_MOTION, MOTION_POSITION_ADVANCED_PI)
                .await
                .is_err()
            {
                return super::fail_inner(phase, CommissionError::Spi);
            }
            if verify_closed_loop_armed_state(backend).await.is_err() {
                return super::fail_inner(phase, CommissionError::Spi);
            }

            let now = backend.elapsed_us();
            let Some(deadline_us) = now.checked_add(CLOSED_LOOP_ACTIVATION_VELOCITY_SETTLE_CAP_US)
            else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *cle_state = ClosedLoopActivationState::VelocitySettle {
                next_sample_us: now,
                deadline_us,
                quiet_samples: 0,
            };
            StepResult::ReadyNow
        }

        ClosedLoopActivationState::VelocitySettle {
            next_sample_us,
            deadline_us,
            quiet_samples,
        } => {
            let now = backend.elapsed_us();
            if now < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }

            let velocity = match backend.read_register(PID_VELOCITY_ACTUAL).await {
                Ok(value) => value as i32,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            *quiet_samples = if velocity.unsigned_abs() <= CLOSED_LOOP_ACTIVATION_VELOCITY_QUIET_RPM
            {
                *quiet_samples + 1
            } else {
                0
            };

            if *quiet_samples >= CLOSED_LOOP_ACTIVATION_VELOCITY_QUIET_SAMPLES {
                *cle_state = ClosedLoopActivationState::AssertEnable;
                return StepResult::ReadyNow;
            }
            if now >= *deadline_us {
                defmt::warn!(
                    "closed-loop: velocity meter still reads {} at the settle cap, enabling anyway",
                    velocity
                );
                *cle_state = ClosedLoopActivationState::AssertEnable;
                return StepResult::ReadyNow;
            }

            let Some(next) = now.checked_add(CLOSED_LOOP_ACTIVATION_VELOCITY_SAMPLE_PERIOD_US)
            else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *next_sample_us = next;
            StepResult::At(next)
        }

        ClosedLoopActivationState::AssertEnable => {
            match crate::tmc_control::lifecycle::settle_closed_loop_hold_before_enable(backend)
                .await
            {
                Ok(()) => {}
                Err(CommissionError::FaultInterlock) => {
                    return super::fail_inner(phase, CommissionError::FaultInterlock);
                }
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            }
            if backend.interlock_tripped() {
                return super::fail_inner(phase, CommissionError::FaultInterlock);
            }
            backend.set_enable_pin(true);

            let now = backend.elapsed_us();
            let Some(deadline_us) = now.checked_add(10_000) else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *cle_state = ClosedLoopActivationState::StabilityCheck1 {
                position_1: 0,
                deadline_us,
            };
            StepResult::At(deadline_us)
        }

        ClosedLoopActivationState::StabilityCheck1 {
            position_1: _,
            deadline_us,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }

            let pos1 = match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(val) => val as i32,
                Err(_) => return super::fail_inner(phase, CommissionError::Spi),
            };
            let now = backend.elapsed_us();
            let Some(next_deadline_us) = now.checked_add(20_000) else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *cle_state = ClosedLoopActivationState::StabilityCheck2 {
                position_1: pos1,
                deadline_us: next_deadline_us,
            };
            StepResult::At(next_deadline_us)
        }

        ClosedLoopActivationState::StabilityCheck2 {
            position_1,
            deadline_us,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }

            let pos2 = match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(val) => val as i32,
                Err(_) => {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };

            if pos2.unsigned_abs() > CLOSED_LOOP_ACTIVATION_RUNAWAY_THRESHOLD
                && pos2.unsigned_abs() > position_1.unsigned_abs()
            {
                defmt::error!(
                    "closed-loop: runaway detected: pos1={}, pos2={}",
                    position_1,
                    pos2
                );
                let drift = (pos2.wrapping_sub(*position_1)).unsigned_abs();
                push_closed_loop_activation_result(
                    diagnostics,
                    CLOSED_LOOP_ACTIVATION_STATUS_FAIL_RUNAWAY,
                    *position_1,
                    pos2,
                    drift,
                );
                if matches!(sequence, InnerSequence::Commission)
                    && let Some(result) = fail_closed_loop_activation(
                        inner_result,
                        backend,
                        ActivationFailureMetrics {
                            entry_status: CLOSED_LOOP_ACTIVATION_STATUS_FAIL_RUNAWAY,
                            position_1: *position_1,
                            position_2: pos2,
                            drift,
                            runaway: true,
                        },
                        FallbackTerminalState {
                            velocity_p,
                            velocity_i,
                            position_p,
                            position_i,
                            velocity_limit,
                            velocity_filter_hz,
                            torque_filter_hz,
                            position_filter_hz,
                            flux_filter_hz,
                        },
                    )
                    .await
                {
                    return result;
                }
                let _ = backend
                    .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                    .await;
                return super::fail_inner(phase, CommissionError::ClosedLoopActivationUnstable);
            }

            let drift = (pos2.wrapping_sub(*position_1)).unsigned_abs();
            let bounded_hunt = drift > CLOSED_LOOP_ACTIVATION_STABILITY_THRESHOLD
                && drift <= CLOSED_LOOP_ACTIVATION_BOUNDED_DRIFT_WARN_THRESHOLD;

            if bounded_hunt && matches!(sequence, InnerSequence::Commission) {
                defmt::warn!(
                    "closed-loop: bounded entry drift: pos1={}, pos2={}, drift={}",
                    position_1,
                    pos2,
                    drift
                );
                if let Some(inner) = inner_result.as_mut() {
                    copy_closed_loop_activation_metrics(
                        inner,
                        CLOSED_LOOP_ACTIVATION_STATUS_WARN_DRIFT,
                        *position_1,
                        pos2,
                        drift,
                        false,
                    );
                }
                *cle_state = ClosedLoopActivationState::SustainedHoldStart;
                return StepResult::ReadyNow;
            }

            if drift > CLOSED_LOOP_ACTIVATION_STABILITY_THRESHOLD && !bounded_hunt {
                defmt::error!(
                    "closed-loop: unstable (attempt {}): pos1={}, pos2={}, drift={}",
                    attempt,
                    position_1,
                    pos2,
                    drift
                );
                push_closed_loop_activation_result(
                    diagnostics,
                    CLOSED_LOOP_ACTIVATION_STATUS_FAIL_DRIFT,
                    *position_1,
                    pos2,
                    drift,
                );
                let _ = backend
                    .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                    .await;

                let next_attempt = attempt + 1;
                if next_attempt < CLOSED_LOOP_ACTIVATION_MAX_ATTEMPTS {
                    backend.set_enable_pin(false);
                    let InnerPhase::ClosedLoopActivation { state, attempt, .. } = phase else {
                        unreachable!("phase was just matched as ClosedLoopActivation");
                    };
                    *state = ClosedLoopActivationState::WriteGains;
                    *attempt = next_attempt;
                    return StepResult::ReadyNow;
                }

                if matches!(sequence, InnerSequence::Commission)
                    && let Some(result) = fail_closed_loop_activation(
                        inner_result,
                        backend,
                        ActivationFailureMetrics {
                            entry_status: CLOSED_LOOP_ACTIVATION_STATUS_FAIL_DRIFT,
                            position_1: *position_1,
                            position_2: pos2,
                            drift,
                            runaway: false,
                        },
                        FallbackTerminalState {
                            velocity_p,
                            velocity_i,
                            position_p,
                            position_i,
                            velocity_limit,
                            velocity_filter_hz,
                            torque_filter_hz,
                            position_filter_hz,
                            flux_filter_hz,
                        },
                    )
                    .await
                {
                    return result;
                }
                return super::fail_inner(phase, CommissionError::ClosedLoopActivationUnstable);
            }

            defmt::info!("closed-loop position mode active (engine)");

            match sequence {
                InnerSequence::Calibrate => {
                    let adc_result = adc_cal.clone().unwrap_or(AdcCalResult {
                        adc_i0: 0,
                        adc_i1: 0,
                        encoder_count: 0,
                        encoder_alignment_electrical_residual_counts: 0,
                        encoder_alignment_stability_counts: 0,
                        encoder_alignment_movement_counts: 0,
                        encoder_alignment_min_movement_counts: 0,
                        encoder_alignment_counts_per_electrical_rev: 0,
                        adc_residual_chop0: AdcResidualEvidence::EMPTY,
                        adc_residual_chop7: AdcResidualEvidence::EMPTY,
                    });
                    StepResult::CalibrateDone(adc_result)
                }
                InnerSequence::Commission => {
                    if let Some(inner) = inner_result.as_mut() {
                        copy_closed_loop_activation_metrics(
                            inner,
                            CLOSED_LOOP_ACTIVATION_STATUS_PASS,
                            *position_1,
                            pos2,
                            drift,
                            false,
                        );
                    }
                    *cle_state = ClosedLoopActivationState::SustainedHoldStart;
                    StepResult::ReadyNow
                }
                InnerSequence::Selftest => {
                    unreachable!("Selftest never reaches ClosedLoopActivation")
                }
            }
        }

        ClosedLoopActivationState::SustainedHoldStart => {
            let now = backend.elapsed_us();
            let Some(deadline_us) = now.checked_add(HOLD_GATE_DWELL_US as u64) else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *cle_state = ClosedLoopActivationState::SustainedHoldSample {
                accumulator: HoldGateAccumulator::commissioning(
                    HOLD_GATE_SAMPLE_PERIOD_US as u16,
                    backend.feedback_config(),
                ),
                started_us: now,
                next_sample_us: now,
                deadline_us,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            };
            StepResult::ReadyNow
        }

        ClosedLoopActivationState::SustainedHoldSample {
            accumulator,
            started_us,
            next_sample_us,
            deadline_us,
            requested_sample_period_us,
            sample_overrun,
        } => {
            let now = backend.elapsed_us();
            if now < *next_sample_us {
                return StepResult::At(*next_sample_us);
            }

            let pid_torque_flux_actual = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
                Ok(value) => value,
                Err(_) => {
                    let _ = backend
                        .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                        .await;
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };
            let position_count = match backend.read_register(PID_POSITION_ACTUAL).await {
                Ok(value) => value as i32,
                Err(_) => {
                    let _ = backend
                        .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                        .await;
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };
            let status_flags = match backend.read_register(STATUS_FLAGS).await {
                Ok(value) => value,
                Err(_) => {
                    let _ = backend
                        .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                        .await;
                    return super::fail_inner(phase, CommissionError::Spi);
                }
            };

            accumulator.push(HoldGateSample {
                elapsed_us: now.saturating_sub(*started_us).min(u64::from(u32::MAX)) as u32,
                position_count,
                torque_count: decode_signed_upper_half(pid_torque_flux_actual),
                flux_count: decode_signed_lower_half(pid_torque_flux_actual),
                status_flags,
            });

            if now > *next_sample_us {
                *sample_overrun = true;
            }

            if now < *deadline_us && accumulator.sample_count() < HOLD_GATE_MAX_SAMPLES {
                let next_deadline = match crate::commissioning::schedule::next_grid_deadline(
                    *next_sample_us,
                    u64::from(*requested_sample_period_us),
                    now,
                ) {
                    Ok(deadline) => deadline,
                    Err(error) => return super::fail_inner(phase, error),
                };
                *next_sample_us = next_deadline;
                return StepResult::At(next_deadline);
            }

            let elapsed_us = now.saturating_sub(*started_us).min(u64::from(u32::MAX)) as u32;
            let metrics = accumulator.finish(elapsed_us, false);
            let Some(mut inner) = inner_result.take() else {
                let _ = backend
                    .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                    .await;
                return super::fail_inner(phase, CommissionError::Spi);
            };
            copy_hold_metrics(&mut inner, metrics);

            if metrics.status != HOLD_GATE_STATUS_PASS {
                let failure_reason = hold_status_failure_reason(metrics.status)
                    .unwrap_or(crate::autotune::CURRENT_LOOP_FAILURE_HOLD_SAMPLE);
                return fail_sustained_hold(
                    inner,
                    backend,
                    failure_reason,
                    FallbackTerminalState {
                        velocity_p,
                        velocity_i,
                        position_p,
                        position_i,
                        velocity_limit,
                        velocity_filter_hz,
                        torque_filter_hz,
                        position_filter_hz,
                        flux_filter_hz,
                    },
                )
                .await;
            }

            finish_commission(
                inner,
                FallbackTerminalState {
                    velocity_p,
                    velocity_i,
                    position_p,
                    position_i,
                    velocity_limit,
                    velocity_filter_hz,
                    torque_filter_hz,
                    position_filter_hz,
                    flux_filter_hz,
                },
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::{
        CURRENT_VALIDATION_AXIS_TORQUE, HOLD_GATE_SAMPLE_PERIOD_US, HOLD_GATE_STATUS_PASS,
        HOLD_GATE_WARNING_SAMPLE_OVERRUN, HoldGateAccumulator, HoldGateSample,
        current_loop_actionable_status_mask,
    };
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::types::{CurrentValidationAxisEvidence, InductanceEvidence};
    use foci_tmc4671::registers::{
        ABN_DECODER_PPR, ADC_I_SELECT, MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS,
        PHI_E_SELECTION, PID_POSITION_ACTUAL, PID_TORQUE_FLUX_ACTUAL, PID_TORQUE_FLUX_LIMITS,
        PID_VELOCITY_ACTUAL, PID_VELOCITY_LIMIT, PIDOUT_UQ_UD_LIMITS, POSITION_SELECTION,
        PWM_SV_CHOP, STATUS_FLAGS, STEP_WIDTH, VELOCITY_SELECTION,
    };

    const CLOSED_LOOP_ACTIVATION_MAX_REGISTER_OPS_PER_ARM: usize = 48;

    #[test]
    fn closed_loop_velocity_selection_uses_the_standard_meter_on_the_mechanical_angle() {
        use crate::commissioning::outer::velocity::reproduction::{
            BUILD_FEATURE_STANDARD_METER_QUANTUM, BUILD_FEATURES,
        };
        use foci_tmc4671::registers::VelocitySelection;

        let selection = VelocitySelection::from(CLOSED_LOOP_VELOCITY_SELECTION);

        assert_eq!(
            selection.velocity_meter_selection(),
            0,
            "commissioning must run the standard velocity meter"
        );
        assert_eq!(
            STANDARD_VELOCITY_METER_COUNTS_PER_MINUTE,
            60 * 65_536 / 15,
            "the gate's velocity quantum must be the standard meter's sample rate"
        );
        assert_ne!(
            BUILD_FEATURES & BUILD_FEATURE_STANDARD_METER_QUANTUM,
            0,
            "the build must declare the standard-meter quantum it gates with"
        );
        assert_eq!(
            selection.velocity_selection(),
            9,
            "phi_m_abn must remain the velocity feedback source"
        );
    }

    fn closed_loop_phase_at(state: ClosedLoopActivationState) -> InnerPhase {
        InnerPhase::ClosedLoopActivation {
            state,
            flux_p: PidPGain::ZERO,
            flux_i: PidIGain::ZERO,
            torque_p: PidPGain::ZERO,
            torque_i: PidIGain::ZERO,
            velocity_p: PidPGain::ZERO,
            velocity_i: PidIGain::ZERO,
            position_p: PidPGain::ZERO,
            position_i: PidIGain::ZERO,
            velocity_limit: 500_000,
            velocity_filter_hz: 0,
            torque_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
            inner_result: Some(sample_inner_result(CommissionStatus::Accepted)),
            attempt: 0,
        }
    }

    fn closed_loop_phase_at_without_inner(state: ClosedLoopActivationState) -> InnerPhase {
        let mut phase = closed_loop_phase_at(state);
        let InnerPhase::ClosedLoopActivation { inner_result, .. } = &mut phase else {
            unreachable!();
        };
        *inner_result = None;
        phase
    }

    fn closed_loop_phase_at_with_attempt(
        state: ClosedLoopActivationState,
        attempt: u8,
    ) -> InnerPhase {
        let mut phase = closed_loop_phase_at(state);
        let InnerPhase::ClosedLoopActivation {
            attempt: phase_attempt,
            ..
        } = &mut phase
        else {
            unreachable!();
        };
        *phase_attempt = attempt;
        phase
    }

    fn sample_inner_result(status: CommissionStatus) -> InnerResult {
        InnerResult {
            status,
            r_count_milli: 1706,
            l_count_micro: 1245,
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
            candidate_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            candidate_axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            candidate_gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            candidate_measured_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            candidate_axis_split_clamped: 0,
            candidate_flux_p: 100,
            candidate_flux_i: 10,
            candidate_torque_p: 200,
            candidate_torque_i: 20,
            candidate_attempt: 0,
            lambda_us: 100,
            theta_e_us: 50,
            theta_source: 0,
            bandwidth_hz: 1000,
            inductance: InductanceEvidence::EMPTY,
            inner_warning_flags: 0,
            timing: [crate::commissioning::types::CommissionTimingSummary::NOT_RUN; 3],
            timing_summary: 0,
            resistance_selected_count_slope_milli: 1706,
            resistance_gain_path_count_slope_milli: 1706,
            resistance_axis0_count_slope_milli: 1700,
            resistance_axis1_count_slope_milli: 1712,
            resistance_axis0_intercept_count: 0,
            resistance_axis1_intercept_count: 0,
            resistance_axis0_rmse_permille: 0,
            resistance_axis1_rmse_permille: 0,
            resistance_selected_mask_axis0: 0b1111_1000,
            resistance_selected_mask_axis1: 0b1111_1000,
            resistance_axis0_signed_count_slope_milli: 1702,
            resistance_axis1_signed_count_slope_milli: 1710,
            resistance_axis0_signed_asymmetry_permille: 0,
            resistance_axis1_signed_asymmetry_permille: 0,
            resistance_axis0_drift_permille: 0,
            resistance_axis1_drift_permille: 0,
            resistance_status_flags_or: 0,
            resistance_warning_flags: 0,
            resistance_peak_abs_current_count: 1_440,
            resistance_max_abs_steady_mean_current_count: 1_280,
            resistance_current_ceiling_count: 1_600,
            resistance_power_stage_tripped: false,
            encoder_alignment_count: 0,
            encoder_alignment_electrical_residual_counts: 0,
            encoder_alignment_stability_counts: 0,
            encoder_alignment_movement_counts: 0,
            encoder_alignment_min_movement_counts: 0,
            encoder_alignment_counts_per_electrical_rev: 0,
            adc_residual_chop0: AdcResidualEvidence::EMPTY,
            adc_residual_chop7: AdcResidualEvidence::EMPTY,
            current_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            current_validation_axes: CURRENT_VALIDATION_AXIS_TORQUE,
            retry_budget_exhausted: 0,
            current_loop_failure_reason: crate::autotune::CURRENT_LOOP_FAILURE_NONE,
            closed_loop_activation_status: crate::autotune::CLOSED_LOOP_ACTIVATION_STATUS_NOT_RUN,
            closed_loop_activation_position_1: 0,
            closed_loop_activation_position_2: 0,
            closed_loop_activation_drift_count: 0,
            closed_loop_activation_threshold_count: 0,
            closed_loop_activation_runaway: 0,
            current_hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
            current_hold_warning_flags: 0,
            current_hold_elapsed_us: 0,
            current_hold_requested_sample_period_us: crate::autotune::HOLD_GATE_SAMPLE_PERIOD_US
                as u16,
            current_hold_sample_count: 0,
            current_hold_position_span_count: 0,
            current_hold_position_drift_count: 0,
            current_hold_torque_mean_count: 0,
            current_hold_torque_rms_count: 0,
            current_hold_torque_peak_to_peak_count: 0,
            current_hold_torque_crossing_count: 0,
            current_hold_flux_mean_count: 0,
            current_hold_flux_rms_count: 0,
            current_hold_flux_peak_to_peak_count: 0,
            current_hold_flux_crossing_count: 0,
            current_hold_status_flags_or: 0,
            current_hold_actionable_status_count: 0,
            current_validation_step_amplitude: 128,
            current_validation_flux_step_amplitude: 128,
            current_validation_torque_step_amplitude: 128,
            current_validation_pidout_limit: 2400,
            current_validation_current_limited: 422,
            current_validation_voltage_limited: 128,
            current_validation_max_p: 3474,
            current_settled_validation_count: 0,
            current_settled_validation:
                [crate::commissioning::types::CurrentValidationSettledSampleEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES],
            current_flux_validation_count: 0,
            current_flux_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            current_torque_validation_count: 0,
            current_torque_validation: [CurrentValidationAxisEvidence::EMPTY;
                crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
        }
    }

    fn pack_torque_flux_actual(torque: i16, flux: i16) -> u32 {
        ((torque as u16 as u32) << 16) | flux as u16 as u32
    }

    fn seed_armed_state_registers(backend: &mut MockBackend) {
        backend.current_limit_value = current_limit(2816);
        backend.operational_voltage_limit_value = 29_000;
        backend.configured_physical_width = Some(16);

        let registers = [
            (PWM_SV_CHOP.addr(), 7),
            (MOTOR_TYPE_N_POLE_PAIRS.addr(), (2 << 16) | 50),
            (PHI_E_SELECTION.addr(), 3),
            (POSITION_SELECTION.addr(), CLOSED_LOOP_POSITION_SELECTION),
            (VELOCITY_SELECTION.addr(), CLOSED_LOOP_VELOCITY_SELECTION),
            (
                ABN_DECODER_PPR.addr(),
                backend.mechanical_encoder_quadrature_counts_per_rev_value,
            ),
            (ADC_I_SELECT.addr(), backend.tmc_config.adc_i_select),
            (
                PID_TORQUE_FLUX_LIMITS.addr(),
                backend.current_limit_value.register_value(),
            ),
            (
                STEP_WIDTH.addr(),
                backend.configured_physical_width.unwrap(),
            ),
            (
                PIDOUT_UQ_UD_LIMITS.addr(),
                backend.operational_voltage_limit_value,
            ),
            (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
        ];
        for (addr, value) in registers {
            backend.registers.insert(addr, value).ok();
        }
    }

    fn write_log(backend: &MockBackend) -> heapless::Vec<(u8, u32), 128> {
        let write_count = backend
            .spi_log
            .iter()
            .filter(|op| matches!(op, SpiOp::Write(_, _)))
            .count();
        let writes = backend
            .spi_log
            .iter()
            .filter_map(|op| {
                if let SpiOp::Write(addr, val) = op {
                    Some((*addr, *val))
                } else {
                    None
                }
            })
            .collect::<heapless::Vec<_, 128>>();
        assert_eq!(
            writes.len(),
            write_count,
            "write_log helper capacity truncated the SPI log"
        );
        writes
    }

    fn register_op_count(backend: &MockBackend) -> usize {
        backend
            .spi_log
            .iter()
            .filter(|op| !matches!(op, SpiOp::EnablePin(_)))
            .count()
    }

    #[test]
    fn write_position_config_programs_selection_registers() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(16);

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::WriteFilters,
                    ..
                }
            ));
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (VELOCITY_SELECTION.addr(), CLOSED_LOOP_VELOCITY_SELECTION),
                    (POSITION_SELECTION.addr(), 9),
                    (STEP_WIDTH.addr(), 16),
                ]
            );
        });
    }

    #[test]
    fn write_position_config_verifies_mechanical_selection_before_step_width() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(16);
            backend.read_override = Some((POSITION_SELECTION.addr(), 8));

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .iter()
                    .any(|(addr, _)| *addr == STEP_WIDTH.addr()),
                "STEP_WIDTH must not be armed before mechanical selection verifies"
            );
        });
    }

    #[test]
    fn write_position_config_propagates_mechanical_selection_read_error() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(16);
            backend.fail_on_read = Some(POSITION_SELECTION.addr());

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .iter()
                    .any(|(addr, _)| *addr == STEP_WIDTH.addr()),
                "POSITION_SELECTION read failure must block STEP_WIDTH"
            );
        });
    }

    #[test]
    fn write_position_config_uses_published_physical_width_not_legacy_mirror() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(8);

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (VELOCITY_SELECTION.addr(), CLOSED_LOOP_VELOCITY_SELECTION),
                    (POSITION_SELECTION.addr(), CLOSED_LOOP_POSITION_SELECTION),
                    (STEP_WIDTH.addr(), 8),
                ]
            );
        });
    }

    #[test]
    fn write_position_config_rejects_unpublished_legacy_width() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .iter()
                    .any(|(addr, _)| *addr == STEP_WIDTH.addr()),
                "an unpublished legacy width must not arm STEP_WIDTH"
            );
        });
    }

    #[test]
    fn write_position_config_rejects_zero_published_physical_width() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::WritePositionConfig);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.configured_physical_width = Some(0);

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .iter()
                    .any(|(addr, _)| *addr == STEP_WIDTH.addr()),
                "zero physical width must not arm STEP_WIDTH"
            );
        });
    }

    #[test]
    fn write_limits_uses_configured_operational_voltage_limit() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::WriteLimits);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.operational_voltage_limit_value = 20_000;

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert_eq!(
                write_log(&backend).as_slice(),
                &[
                    (PID_VELOCITY_LIMIT.addr(), 500_000),
                    (PIDOUT_UQ_UD_LIMITS.addr(), 20_000),
                ]
            );
            assert_eq!(
                backend
                    .spi_log
                    .iter()
                    .filter(|op| matches!(op, SpiOp::Read))
                    .count(),
                1
            );
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::VerifyPhiE,
                    ..
                }
            ));
        });
    }

    #[test]
    fn enter_position_mode_accepts_expected_armed_state_readbacks() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::EnterPositionMode);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::VelocitySettle {
                        quiet_samples: 0,
                        ..
                    },
                    ..
                }
            ));
            assert!(
                !backend.enable_pin,
                "the bridge must stay disabled until the velocity meter settles"
            );
        });
    }

    fn velocity_settle_phase(next_sample_us: u64, quiet_samples: u8) -> InnerPhase {
        closed_loop_phase_at(ClosedLoopActivationState::VelocitySettle {
            next_sample_us,
            deadline_us: CLOSED_LOOP_ACTIVATION_VELOCITY_SETTLE_CAP_US,
            quiet_samples,
        })
    }

    async fn step_calibrate(phase: &mut InnerPhase, backend: &mut MockBackend) -> StepResult {
        step_closed_loop_activation(
            phase,
            InnerSequence::Calibrate,
            &mut None,
            &mut CommissionDiagnostics::default(),
            backend,
        )
        .await
    }

    #[test]
    fn velocity_settle_asserts_enable_only_after_consecutive_quiet_samples() {
        pollster::block_on(async {
            let mut phase = velocity_settle_phase(0, 0);
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend.read_override_remaining = Some((PID_VELOCITY_ACTUAL.addr(), 1966, 2));
            backend.registers.insert(PID_VELOCITY_ACTUAL.addr(), 0).ok();

            let period = CLOSED_LOOP_ACTIVATION_VELOCITY_SAMPLE_PERIOD_US;
            let mut now = 0;
            let mut ticks = 0;
            loop {
                backend.elapsed_us_value = now;
                let result = step_calibrate(&mut phase, &mut backend).await;
                ticks += 1;
                match result {
                    StepResult::At(next) => {
                        assert_eq!(next, now + period, "samples must sit on the period grid");
                        assert!(
                            !backend.enable_pin,
                            "no enable while the meter is loud or unproven"
                        );
                        now = next;
                    }
                    StepResult::ReadyNow => break,
                    other => panic!("unexpected step result {other:?}"),
                }
                assert!(ticks < 16, "settle never completed");
            }

            assert_eq!(ticks, 5, "two loud samples, then three quiet ones in a row");
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::AssertEnable,
                    ..
                }
            ));
            assert!(!backend.enable_pin);
        });
    }

    #[test]
    fn velocity_settle_resets_the_quiet_streak_on_motion() {
        pollster::block_on(async {
            let mut phase = velocity_settle_phase(0, 2);
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_VELOCITY_ACTUAL.addr(), (-500i32) as u32)
                .ok();

            let result = step_calibrate(&mut phase, &mut backend).await;

            assert!(matches!(result, StepResult::At(_)));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::VelocitySettle {
                        quiet_samples: 0,
                        ..
                    },
                    ..
                }
            ));
        });
    }

    #[test]
    fn velocity_settle_proceeds_when_the_cap_expires() {
        pollster::block_on(async {
            let mut phase = velocity_settle_phase(0, 0);
            let mut backend = MockBackend::new();
            backend.read_override = Some((PID_VELOCITY_ACTUAL.addr(), 500));
            backend.elapsed_us_value = CLOSED_LOOP_ACTIVATION_VELOCITY_SETTLE_CAP_US;

            let result = step_calibrate(&mut phase, &mut backend).await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::AssertEnable,
                    ..
                }
            ));
        });
    }

    #[test]
    fn velocity_settle_waits_for_the_next_sample_without_reading() {
        pollster::block_on(async {
            let mut phase = velocity_settle_phase(4_000, 1);
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 3_999;

            let result = step_calibrate(&mut phase, &mut backend).await;

            assert!(matches!(result, StepResult::At(4_000)));
            assert_eq!(register_op_count(&backend), 0);
        });
    }

    #[test]
    fn stability_checks_return_exact_one_shot_deadlines() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::StabilityCheck1 {
                position_1: 0,
                deadline_us: 10_000,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 9_999;

            let early = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(early, StepResult::At(10_000)));
            assert_eq!(register_op_count(&backend), 0);

            backend.elapsed_us_value = 10_000;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 7).ok();
            let due = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(due, StepResult::At(30_000)));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::StabilityCheck2 {
                        position_1: 7,
                        deadline_us: 30_000,
                    },
                    ..
                }
            ));

            backend.spi_log.clear();
            backend.elapsed_us_value = 29_999;
            let second_early = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(second_early, StepResult::At(30_000)));
            assert_eq!(register_op_count(&backend), 0);
        });
    }

    #[test]
    fn filter_writes_respect_entry_operation_bound() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::WriteFilters);
            let InnerPhase::ClosedLoopActivation {
                velocity_filter_hz,
                torque_filter_hz,
                position_filter_hz,
                flux_filter_hz,
                ..
            } = &mut phase
            else {
                unreachable!();
            };
            *velocity_filter_hz = 100;
            *torque_filter_hz = 100;
            *position_filter_hz = 100;
            *flux_filter_hz = 100;
            let mut adc_cal = None;
            let mut backend = MockBackend::new();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert_eq!(
                register_op_count(&backend),
                CLOSED_LOOP_ACTIVATION_MAX_REGISTER_OPS_PER_ARM
            );
        });
    }

    #[test]
    fn assert_enable_sets_hold_target_before_enable() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::AssertEnable);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 11)
                .ok();
            backend
                .registers
                .insert(STATUS_FLAGS.addr(), 0xffff_ffff)
                .ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(10_000)));
            let enable_index = backend
                .spi_log
                .iter()
                .position(|op| matches!(op, SpiOp::EnablePin(true)))
                .expect("closed-loop entry should assert enable");
            let target_rewrite_index = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        SpiOp::Write(addr, 11) if *addr == PID_POSITION_ACTUAL.addr()
                    )
                })
                .expect("closed-loop entry should rewrite PID_POSITION_ACTUAL to observed actual");
            let status_clear_index = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(
                        op,
                        SpiOp::Write(addr, 0) if *addr == STATUS_FLAGS.addr()
                    )
                })
                .expect("closed-loop entry should clear STATUS_FLAGS before enable");
            assert!(target_rewrite_index < enable_index);
            assert!(status_clear_index < enable_index);
        });
    }

    #[test]
    fn enter_position_mode_rejects_unpublished_legacy_width() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::EnterPositionMode);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend.configured_physical_width = None;

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(!backend.enable_pin, "missing scale must fail closed");
        });
    }

    #[test]
    fn assert_enable_refuses_to_enable_when_the_interlock_is_tripped() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::AssertEnable);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend.interlock_tripped = true;

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::FaultInterlock)
            ));
            assert!(!backend.enable_pin, "tripped interlock must fail closed");
            assert!(
                !backend
                    .spi_log
                    .iter()
                    .any(|op| matches!(op, SpiOp::Write(addr, _) if *addr == STATUS_FLAGS.addr())),
                "STATUS_FLAGS must not be cleared behind a tripped interlock"
            );
        });
    }

    #[test]
    fn zero_and_clear_refuses_to_clear_status_flags_when_the_interlock_is_tripped() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::ZeroAndClear);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend.interlock_tripped = true;

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::FaultInterlock)
            ));
            assert!(
                !backend
                    .spi_log
                    .iter()
                    .any(|op| matches!(op, SpiOp::Write(addr, _) if *addr == STATUS_FLAGS.addr())),
                "ZeroAndClear must not write STATUS_FLAGS behind a tripped interlock"
            );
        });
    }

    #[test]
    fn enter_position_mode_rejects_divergent_legacy_width_without_enable() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::EnterPositionMode);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend.configured_physical_width = Some(8);

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(!backend.enable_pin, "divergent STEP_WIDTH must fail closed");
        });
    }

    #[test]
    fn enter_position_mode_propagates_armed_state_read_error_without_enable() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::EnterPositionMode);
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            seed_armed_state_registers(&mut backend);
            backend.fail_on_read = Some(STEP_WIDTH.addr());

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                !backend.enable_pin,
                "armed-state read failure must fail closed"
            );
        });
    }

    #[test]
    fn enter_position_mode_rejects_stale_armed_state_readbacks() {
        pollster::block_on(async {
            let stale_registers = [
                (PWM_SV_CHOP.addr(), 0),
                (MOTOR_TYPE_N_POLE_PAIRS.addr(), 3 << 16),
                (PHI_E_SELECTION.addr(), 1),
                (POSITION_SELECTION.addr(), 8),
                (VELOCITY_SELECTION.addr(), 0x0000_0109),
                (ABN_DECODER_PPR.addr(), 3999),
                (ADC_I_SELECT.addr(), 0),
                (PID_TORQUE_FLUX_LIMITS.addr(), 1024),
                (PIDOUT_UQ_UD_LIMITS.addr(), 16_000),
                (STEP_WIDTH.addr(), 8),
            ];

            for (addr, stale_value) in stale_registers {
                let mut phase = closed_loop_phase_at(ClosedLoopActivationState::EnterPositionMode);
                let mut adc_cal = None;
                let mut backend = MockBackend::new();
                seed_armed_state_registers(&mut backend);
                backend.registers.insert(addr, stale_value).ok();

                let result = step_closed_loop_activation(
                    &mut phase,
                    InnerSequence::Calibrate,
                    &mut adc_cal,
                    &mut CommissionDiagnostics::default(),
                    &mut backend,
                )
                .await;

                assert!(
                    matches!(result, StepResult::InnerFailed(CommissionError::Spi)),
                    "register 0x{addr:02x} stale value 0x{stale_value:08x} was accepted"
                );
            }
        });
    }

    #[test]
    fn commission_runs_sustained_hold_after_fast_stability_check() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 0).ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::SustainedHoldStart,
                    ..
                }
            ));
            assert!(matches!(result, StepResult::ReadyNow));
        });
    }

    #[test]
    fn commission_bounded_entry_drift_runs_sustained_hold_with_warning_evidence() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 17)
                .ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            let InnerPhase::ClosedLoopActivation {
                state: ClosedLoopActivationState::SustainedHoldStart,
                inner_result: Some(inner),
                ..
            } = phase
            else {
                panic!("expected bounded entry drift to continue into sustained hold");
            };
            assert_eq!(inner.status, CommissionStatus::Accepted);
            assert_eq!(
                inner.current_loop_failure_reason,
                crate::autotune::CURRENT_LOOP_FAILURE_NONE
            );
            assert_eq!(
                inner.closed_loop_activation_status,
                CLOSED_LOOP_ACTIVATION_STATUS_WARN_DRIFT
            );
            assert_eq!(inner.closed_loop_activation_position_1, 0);
            assert_eq!(inner.closed_loop_activation_position_2, 17);
            assert_eq!(inner.closed_loop_activation_drift_count, 17);
            assert_eq!(inner.closed_loop_activation_threshold_count, 2);
            assert_eq!(inner.closed_loop_activation_runaway, 0);
            assert!(
                !write_log(&backend)
                    .as_slice()
                    .contains(&(MODE_RAMP_MODE_MOTION.addr(), 0)),
                "bounded entry drift must leave position mode active for sustained hold"
            );
        });
    }

    #[test]
    fn calibrate_skips_sustained_hold_after_fast_stability_check() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 0).ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::CalibrateDone(_)));
        });
    }

    #[test]
    fn calibrate_accepts_bounded_entry_drift_without_retry_or_failure() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 16)
                .ok();
            let mut diagnostics = CommissionDiagnostics::default();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(result, StepResult::CalibrateDone(_)),
                "bounded drift within the warn band should be accepted, got {result:?}"
            );
            assert!(
                diagnostics.iter().next().is_none(),
                "accepted bounded drift should not push a failure diagnostic"
            );
        });
    }

    #[test]
    fn stability_drift_retries_before_exhausting_attempts() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 100)
                .ok();
            let mut diagnostics = CommissionDiagnostics::default();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(result, StepResult::ReadyNow),
                "bounded drift on the first attempt should retry, got {result:?}"
            );
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::WriteGains,
                    attempt: 1,
                    ..
                }
            ));
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(
                events[0].phase,
                crate::autotune::CommissionPhase::ClosedLoopActivation.to_u8()
            );
            assert_eq!(events[0].code, DIAG_CLOSED_LOOP_ACTIVATION_RESULT);
            assert_eq!(events[0].status, CLOSED_LOOP_ACTIVATION_STATUS_FAIL_DRIFT);
            assert_eq!(events[0].value1, 100);
            assert_eq!(events[0].value2, 100);
        });
    }

    #[test]
    fn stability_drift_retry_disables_the_bridge_before_re_entering() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 100)
                .ok();

            let result = step_calibrate(&mut phase, &mut backend).await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert!(
                !backend.enable_pin,
                "a retry must re-run the velocity settle with the bridge disabled"
            );
            let stop_index = backend
                .spi_log
                .iter()
                .position(|op| {
                    matches!(op, SpiOp::Write(addr, 0) if *addr == MODE_RAMP_MODE_MOTION.addr())
                })
                .expect("retry should write stopped mode");
            let disable_index = backend
                .spi_log
                .iter()
                .position(|op| matches!(op, SpiOp::EnablePin(false)))
                .expect("retry should lower the enable pin");
            assert!(stop_index < disable_index);
        });
    }

    #[test]
    fn stability_drift_reports_closed_loop_unstable_after_exhausting_attempts() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at_with_attempt(
                ClosedLoopActivationState::StabilityCheck2 {
                    position_1: 0,
                    deadline_us: 0,
                },
                CLOSED_LOOP_ACTIVATION_MAX_ATTEMPTS - 1,
            );
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 100)
                .ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::ClosedLoopActivationUnstable)
            ));
        });
    }

    #[test]
    fn stability_runaway_never_retries_even_on_first_attempt() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::StabilityCheck2 {
                position_1: 0,
                deadline_us: 0,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(
                    PID_POSITION_ACTUAL.addr(),
                    CLOSED_LOOP_ACTIVATION_RUNAWAY_THRESHOLD + 1,
                )
                .ok();
            let mut diagnostics = CommissionDiagnostics::default();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(
                    result,
                    StepResult::InnerFailed(CommissionError::ClosedLoopActivationUnstable)
                ),
                "runaway must fail immediately regardless of attempt budget, got {result:?}"
            );
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].status, CLOSED_LOOP_ACTIVATION_STATUS_FAIL_RUNAWAY);
        });
    }

    #[test]
    fn commission_large_entry_drift_returns_failed_result_with_entry_evidence() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at_with_attempt(
                ClosedLoopActivationState::StabilityCheck2 {
                    position_1: 0,
                    deadline_us: 0,
                },
                CLOSED_LOOP_ACTIVATION_MAX_ATTEMPTS - 1,
            );
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 65)
                .ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected failed CommissionDone with closed-loop entry evidence");
            };
            assert_eq!(
                result.inner.status,
                CommissionStatus::Failed(CommissionError::ClosedLoopActivationUnstable)
            );
            assert_eq!(result.inner.current_loop_failure_reason, 17);
            assert_eq!(
                result.inner.closed_loop_activation_status,
                CLOSED_LOOP_ACTIVATION_STATUS_FAIL_DRIFT
            );
            assert_eq!(result.inner.closed_loop_activation_position_1, 0);
            assert_eq!(result.inner.closed_loop_activation_position_2, 65);
            assert_eq!(result.inner.closed_loop_activation_drift_count, 65);
            assert_eq!(result.inner.closed_loop_activation_threshold_count, 2);
            assert_eq!(result.inner.closed_loop_activation_runaway, 0);
            assert!(
                write_log(&backend)
                    .as_slice()
                    .contains(&(MODE_RAMP_MODE_MOTION.addr(), 0)),
                "unstable entry must stop position mode before returning evidence"
            );
        });
    }

    #[test]
    fn sustained_hold_position_span_accepts_with_warning_evidence() {
        pollster::block_on(async {
            let mut accumulator = HoldGateAccumulator::new(
                0,
                current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
            );
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 0,
                torque_count: 0,
                flux_count: 0,
                status_flags: 0,
            });
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::SustainedHoldSample {
                accumulator,
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64;
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 17)
                .ok();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 0)
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected accepted CommissionDone with hold evidence");
            };
            assert_eq!(result.inner.status, CommissionStatus::Accepted);
            assert_eq!(
                result.inner.current_loop_failure_reason,
                crate::autotune::CURRENT_LOOP_FAILURE_NONE
            );
            assert_eq!(result.inner.current_hold_status, HOLD_GATE_STATUS_PASS);
            assert_eq!(
                result.inner.current_hold_warning_flags,
                crate::autotune::HOLD_GATE_WARNING_POSITION_SPAN
            );
            assert!(backend.enable_pin);
        });
    }

    #[test]
    fn sustained_hold_pass_returns_commission_done_with_metrics_and_upstream_evidence() {
        pollster::block_on(async {
            let mut accumulator = HoldGateAccumulator::new(
                0,
                current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
            );
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 4,
                torque_count: 20,
                flux_count: -4,
                status_flags: 0,
            });
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::SustainedHoldSample {
                accumulator,
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 6).ok();
            backend
                .registers
                .insert(
                    PID_TORQUE_FLUX_ACTUAL.addr(),
                    pack_torque_flux_actual(24, -2),
                )
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected CommissionDone");
            };
            assert_eq!(result.inner.status, CommissionStatus::Accepted);
            assert_eq!(result.inner.r_count_milli, 1706);
            assert_eq!(
                result.inner.current_validation_axes,
                CURRENT_VALIDATION_AXIS_TORQUE
            );
            assert_eq!(result.inner.current_hold_status, HOLD_GATE_STATUS_PASS);
            assert_eq!(result.inner.current_hold_sample_count, 2);
            assert_eq!(
                result.inner.current_hold_elapsed_us,
                HOLD_GATE_SAMPLE_PERIOD_US
            );
            assert_eq!(result.inner.current_hold_position_span_count, 2);
            assert_eq!(result.inner.current_hold_torque_mean_count, 22);
            assert_eq!(result.inner.current_hold_flux_mean_count, -3);
        });
    }

    #[test]
    fn sustained_hold_position_span_returns_accepted_result_with_evidence() {
        pollster::block_on(async {
            let mut accumulator = HoldGateAccumulator::new(
                0,
                current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
            );
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 0,
                torque_count: 0,
                flux_count: 0,
                status_flags: 0,
            });
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::SustainedHoldSample {
                accumulator,
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64;
            backend
                .registers
                .insert(PID_POSITION_ACTUAL.addr(), 17)
                .ok();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 0)
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected accepted CommissionDone with hold evidence");
            };
            assert_eq!(result.inner.status, CommissionStatus::Accepted);
            assert_eq!(
                result.inner.current_loop_failure_reason,
                crate::autotune::CURRENT_LOOP_FAILURE_NONE
            );
            assert_eq!(result.inner.current_hold_status, HOLD_GATE_STATUS_PASS);
            assert_eq!(
                result.inner.current_hold_warning_flags,
                crate::autotune::HOLD_GATE_WARNING_POSITION_SPAN
            );
            assert_eq!(result.inner.current_hold_sample_count, 2);
            assert_eq!(result.inner.current_hold_position_span_count, 17);
            assert!(backend.enable_pin);
        });
    }

    #[test]
    fn sustained_hold_missing_inner_result_fails_without_default_evidence() {
        pollster::block_on(async {
            let mut accumulator = HoldGateAccumulator::new(
                0,
                current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
            );
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 4,
                torque_count: 20,
                flux_count: -4,
                status_flags: 0,
            });
            let mut phase = closed_loop_phase_at_without_inner(
                ClosedLoopActivationState::SustainedHoldSample {
                    accumulator,
                    started_us: 0,
                    next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                    deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                    requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                    sample_overrun: false,
                },
            );
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 4).ok();
            backend
                .registers
                .insert(
                    PID_TORQUE_FLUX_ACTUAL.addr(),
                    pack_torque_flux_actual(20, -4),
                )
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                write_log(&backend)
                    .as_slice()
                    .contains(&(MODE_RAMP_MODE_MOTION.addr(), 0)),
                "missing inner result must stop position mode before failing"
            );
        });
    }

    #[test]
    fn sustained_hold_spi_read_failure_stops_motion_mode() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::SustainedHoldSample {
                accumulator: HoldGateAccumulator::new(
                    HOLD_GATE_SAMPLE_PERIOD_US as u16,
                    current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
                ),
                started_us: 0,
                next_sample_us: 0,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.fail_on_read = Some(PID_TORQUE_FLUX_ACTUAL.addr());

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(
                write_log(&backend)
                    .as_slice()
                    .contains(&(MODE_RAMP_MODE_MOTION.addr(), 0)),
                "SPI read failure must stop position mode before returning the error"
            );
        });
    }

    #[test]
    fn sustained_hold_late_sample_marks_overrun_and_preserves_grid() {
        pollster::block_on(async {
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::SustainedHoldSample {
                accumulator: HoldGateAccumulator::new(
                    HOLD_GATE_SAMPLE_PERIOD_US as u16,
                    current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
                ),
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: (HOLD_GATE_SAMPLE_PERIOD_US * 3) as u64,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64 + 1;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 0).ok();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 0)
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::At(next) if next == (HOLD_GATE_SAMPLE_PERIOD_US * 2) as u64
            ));
            assert!(matches!(
                phase,
                InnerPhase::ClosedLoopActivation {
                    state: ClosedLoopActivationState::SustainedHoldSample {
                        next_sample_us,
                        sample_overrun: true,
                        ..
                    },
                    ..
                } if next_sample_us == (HOLD_GATE_SAMPLE_PERIOD_US * 2) as u64
            ));
        });
    }

    #[test]
    fn sustained_hold_overrun_finishes_with_warning_not_failure() {
        pollster::block_on(async {
            let mut accumulator = HoldGateAccumulator::new(
                HOLD_GATE_SAMPLE_PERIOD_US as u16,
                current_loop_actionable_status_mask(crate::config::FeedbackConfig::abn_abn()),
            );
            accumulator.push(HoldGateSample {
                elapsed_us: 0,
                position_count: 0,
                torque_count: 0,
                flux_count: 0,
                status_flags: 0,
            });
            let mut phase = closed_loop_phase_at(ClosedLoopActivationState::SustainedHoldSample {
                accumulator,
                started_us: 0,
                next_sample_us: HOLD_GATE_SAMPLE_PERIOD_US as u64,
                deadline_us: HOLD_GATE_SAMPLE_PERIOD_US as u64 + 1,
                requested_sample_period_us: HOLD_GATE_SAMPLE_PERIOD_US as u16,
                sample_overrun: false,
            });
            let mut adc_cal = None;
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = HOLD_GATE_SAMPLE_PERIOD_US as u64 + 1;
            backend.registers.insert(PID_POSITION_ACTUAL.addr(), 0).ok();
            backend
                .registers
                .insert(PID_TORQUE_FLUX_ACTUAL.addr(), 0)
                .ok();
            backend.registers.insert(STATUS_FLAGS.addr(), 0).ok();

            let result = step_closed_loop_activation(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                &mut CommissionDiagnostics::default(),
                &mut backend,
            )
            .await;

            let StepResult::CommissionDone(result) = result else {
                panic!("expected accepted CommissionDone with hold evidence");
            };
            assert_eq!(result.inner.status, CommissionStatus::Accepted);
            assert_eq!(result.inner.current_hold_status, HOLD_GATE_STATUS_PASS);
            assert_eq!(
                result.inner.current_hold_warning_flags,
                HOLD_GATE_WARNING_SAMPLE_OVERRUN
            );
            assert!(
                backend.enable_pin,
                "warning-only overrun must leave the accepted hold enabled"
            );
        });
    }
}
