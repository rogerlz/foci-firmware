//! Phase 4: Encoder presence/direction check and D-axis alignment.

use super::{
    ElectricalIdentificationData, ElectricalIdentificationState, EncoderAlignState,
    EncoderCheckState, InnerPhase,
};
use crate::autotune::{ClosedLoopActivationState, CommissionError, CommissionPhase, InnerSequence};
use crate::commissioning::CalibrateEntryGains;
use crate::commissioning::encoder_domain::{EncoderCount, EncoderDomain};
use crate::commissioning::types::{AdcCalResult, CommissioningBackend, StepResult};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics};
use crate::tmc_register_values::{MOTION_STOPPED, MOTION_TORQUE_ADVANCED_PI};

const ENCODER_DIRECTION_STEPS: u16 = 50;

const ENCODER_DIRECTION_TARGET_PHI_E: u32 = 32768;

const ENCODER_DIRECTION_STEP_US: u64 = 10_000;

const ENCODER_ALIGNMENT_SETTLE_US: u64 = 500_000;

const ENCODER_ALIGNMENT_STABILITY_US: u64 = 10_000;

/// Consecutive stable pairs, one `ENCODER_ALIGNMENT_STABILITY_US` apart,
/// before committing. A rotor without detent torque swings around the
/// aligned position under current control (the current loop cancels the
/// back-EMF that would damp it), and a single pair can land on a turning
/// point of that swing.
const ENCODER_ALIGNMENT_STABLE_PAIRS: u8 = 10;

const ENCODER_ALIGNMENT_STILL_TIMEOUT_US: u64 = 2_000_000;

const ENCODER_ALIGNMENT_VOLTAGE_LIMIT: u32 = 2000;

const ENCODER_MOVEMENT_CURRENT_MIN: u32 = 100;

const ENCODER_MOVEMENT_CURRENT_MAX: u32 = 1000;

const ENCODER_DIRECTION_MAX_ATTEMPTS: u8 = 3;

const ENCODER_MOVEMENT_CURRENT_SCALE: [(u32, u32, u32); ENCODER_DIRECTION_MAX_ATTEMPTS as usize] = [
    (1, 2, ENCODER_MOVEMENT_CURRENT_MAX),
    (3, 4, 2_000),
    (1, 1, u32::MAX),
];

const ENCODER_READ_SAMPLES: usize = 3;
/// Read jitter and alignment stability limits, as a fraction of a
/// revolution: 2 counts of a 1000-line (4000-count) encoder, never below
/// 2 counts.
const ENCODER_STABILITY_REV_DIVISOR: u32 = 2000;
const ENCODER_STABILITY_MIN_COUNTS: u32 = 2;
const ENCODER_READ_ATTEMPTS: usize = 3;

const DIAG_ENCODER_READ_UNSTABLE: u8 = 1;
const DIAG_ENCODER_DIRECTION_RESULT: u8 = 2;
const DIAG_ENCODER_ALIGN_RESULT: u8 = 1;
const DIAG_ENCODER_DIRECTION_EXPECTED: u8 = 40;
const DIAG_ENCODER_DIRECTION_BOUNDS: u8 = 41;

const PHI_E_UNITS_PER_ELECTRICAL_REV: u32 = 65_536;

const ENCODER_DIRECTION_TOLERANCE_NUMERATOR: u32 = 1;
const ENCODER_DIRECTION_TOLERANCE_DENOMINATOR: u32 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct EncoderDirectionWindow {
    expected_signed_counts: i32,
    min_signed_counts: i32,
    max_signed_counts: i32,
}

impl EncoderDirectionWindow {
    fn contains(self, signed_counts: i32) -> bool {
        (self.min_signed_counts..=self.max_signed_counts).contains(&signed_counts)
    }
}

fn encoder_direction_window(
    counts_per_rev: u32,
    pole_pairs: u16,
) -> Option<EncoderDirectionWindow> {
    if pole_pairs == 0 {
        return None;
    }
    let counts_per_electrical_rev = u64::from(counts_per_rev) / u64::from(pole_pairs);
    let expected = counts_per_electrical_rev * u64::from(ENCODER_DIRECTION_TARGET_PHI_E)
        / u64::from(PHI_E_UNITS_PER_ELECTRICAL_REV);
    let tolerance = expected * u64::from(ENCODER_DIRECTION_TOLERANCE_NUMERATOR)
        / u64::from(ENCODER_DIRECTION_TOLERANCE_DENOMINATOR);
    Some(EncoderDirectionWindow {
        expected_signed_counts: i32::try_from(expected).unwrap_or(i32::MAX),
        min_signed_counts: i32::try_from(expected.saturating_sub(tolerance)).unwrap_or(i32::MAX),
        max_signed_counts: i32::try_from(expected.saturating_add(tolerance)).unwrap_or(i32::MAX),
    })
}

fn encoder_alignment_min_movement(encoder_counts_per_rev: u32, pole_pairs: u16) -> u32 {
    let denominator = 20_u64 * u64::from(pole_pairs.max(1));
    let numerator = u64::from(encoder_counts_per_rev);
    ((numerator + denominator / 2) / denominator).max(2) as u32
}

fn encoder_movement_current_target(current_limit: u32, attempt: u8) -> i16 {
    let index = usize::from(attempt.min(ENCODER_DIRECTION_MAX_ATTEMPTS - 1));
    let (numerator, denominator, ceiling) = ENCODER_MOVEMENT_CURRENT_SCALE[index];
    ((current_limit * numerator / denominator).clamp(ENCODER_MOVEMENT_CURRENT_MIN, ceiling))
        .min(current_limit) as i16
}

pub(crate) fn encoder_stability_counts(counts_per_rev: u32) -> u32 {
    (counts_per_rev / ENCODER_STABILITY_REV_DIVISOR).max(ENCODER_STABILITY_MIN_COUNTS)
}

fn saturating_u16(value: u32) -> u16 {
    value.min(u32::from(u16::MAX)) as u16
}

fn counts_per_electrical_rev(counts_per_rev: u32, pole_pairs: u16) -> u16 {
    saturating_u16((counts_per_rev / u32::from(pole_pairs.max(1))).max(1))
}

fn folded_electrical_residual_counts(count: u32, counts_per_electrical_rev: u16) -> u16 {
    let period = u32::from(counts_per_electrical_rev).max(1);
    let remainder = count % period;
    let folded = remainder.min(period.saturating_sub(remainder));
    saturating_u16(folded)
}

fn push_encoder_read_unstable(
    diagnostics: &mut CommissionDiagnostics,
    phase_id: CommissionPhase,
    samples: [u32; ENCODER_READ_SAMPLES],
) {
    diagnostics.push(CommissionDiagnostic {
        phase: phase_id.to_u8(),
        code: DIAG_ENCODER_READ_UNSTABLE,
        status: 1,
        value0: samples[0],
        value1: samples[1],
        value2: samples[2],
    });
}

fn push_encoder_direction_result(
    diagnostics: &mut CommissionDiagnostics,
    count_start: EncoderCount,
    count_end: EncoderCount,
    delta_abs: u32,
    failed: bool,
) {
    diagnostics.push(CommissionDiagnostic {
        phase: CommissionPhase::EncoderCheck.to_u8(),
        code: DIAG_ENCODER_DIRECTION_RESULT,
        status: u8::from(failed),
        value0: count_start.raw_counts(),
        value1: count_end.raw_counts(),
        value2: delta_abs,
    });
}

fn push_encoder_direction_expected(
    diagnostics: &mut CommissionDiagnostics,
    commanded_electrical_displacement: u32,
    expected_signed_counts: i32,
    observed_signed_counts: i32,
) {
    diagnostics.push(CommissionDiagnostic {
        phase: CommissionPhase::EncoderCheck.to_u8(),
        code: DIAG_ENCODER_DIRECTION_EXPECTED,
        status: 1,
        value0: commanded_electrical_displacement,
        value1: expected_signed_counts as u32,
        value2: observed_signed_counts as u32,
    });
}

fn push_encoder_direction_bounds(
    diagnostics: &mut CommissionDiagnostics,
    min_signed_counts: i32,
    max_signed_counts: i32,
    pole_pairs: u16,
) {
    diagnostics.push(CommissionDiagnostic {
        phase: CommissionPhase::EncoderCheck.to_u8(),
        code: DIAG_ENCODER_DIRECTION_BOUNDS,
        status: 1,
        value0: min_signed_counts as u32,
        value1: max_signed_counts as u32,
        value2: u32::from(pole_pairs),
    });
}

fn push_encoder_align_result(
    diagnostics: &mut CommissionDiagnostics,
    movement: u32,
    min_movement: u32,
    stability: u32,
    failed: bool,
) {
    diagnostics.push(CommissionDiagnostic {
        phase: CommissionPhase::EncoderAlign.to_u8(),
        code: DIAG_ENCODER_ALIGN_RESULT,
        status: u8::from(failed),
        value0: movement,
        value1: min_movement,
        value2: stability,
    });
}

async fn read_mechanical_encoder_position_counts_stable<B: CommissioningBackend>(
    backend: &mut B,
    diagnostics: Option<&mut CommissionDiagnostics>,
    phase_id: CommissionPhase,
) -> Result<EncoderCount, CommissionError> {
    use foci_tmc4671::registers::ABN_DECODER_COUNT;

    let domain = EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev())
        .map_err(|_| CommissionError::EncoderFault)?;
    let mut last_samples = [0u32; ENCODER_READ_SAMPLES];
    for _ in 0..ENCODER_READ_ATTEMPTS {
        let mut samples = [0u32; ENCODER_READ_SAMPLES];
        for sample in samples.iter_mut() {
            *sample = backend
                .read_register(ABN_DECODER_COUNT)
                .await
                .map_err(|_| CommissionError::Spi)?;
        }
        last_samples = samples;
        let counts = samples.map(|sample| domain.count(sample));
        let mut spread = 0;
        let mut ambiguous = false;
        for previous in counts {
            for current in counts {
                match domain.delta(previous, current) {
                    Ok(delta) => spread = spread.max(delta.absolute_counts()),
                    Err(_) => ambiguous = true,
                }
            }
        }
        if !ambiguous && spread <= encoder_stability_counts(domain.counts_per_rev()) {
            return Ok(counts[ENCODER_READ_SAMPLES - 1]);
        }
    }
    if let Some(diagnostics) = diagnostics {
        push_encoder_read_unstable(diagnostics, phase_id, last_samples);
    }
    Err(CommissionError::EncoderFault)
}

pub(crate) async fn start_encoder_current_drive<B: CommissioningBackend>(
    backend: &mut B,
    phi_e: u32,
    attempt: u8,
) -> Result<(), CommissionError> {
    use foci_tmc4671::registers::*;

    let current_limit = u32::from(backend.current_limit());
    if current_limit == 0 {
        return Err(CommissionError::NoCurrent);
    }

    let target = encoder_movement_current_target(current_limit, attempt);
    let defaults = crate::pid::DEFAULT_INNER_GAINS;
    let (flux_reg, torque_reg) = defaults.to_registers();
    let target_reg: u32 = PidTorqueFluxTarget::new()
        .with_pid_flux_target(target)
        .with_pid_torque_target(0)
        .into();

    backend.set_enable_pin(false);
    backend
        .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_TORQUE_FLUX_TARGET, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(UQ_UD_EXT, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_FLUX_P_FLUX_I, flux_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(
            PID_TORQUE_FLUX_LIMITS,
            backend.current_limit().register_value(),
        )
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PIDOUT_UQ_UD_LIMITS, ENCODER_ALIGNMENT_VOLTAGE_LIMIT)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PHI_E_SELECTION, 1)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PHI_E_EXT, phi_e)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_TORQUE_FLUX_TARGET, target_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(MODE_RAMP_MODE_MOTION, MOTION_TORQUE_ADVANCED_PI)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend.set_enable_pin(true);
    Ok(())
}

pub(crate) async fn stop_encoder_current_drive<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), CommissionError> {
    use foci_tmc4671::registers::*;

    backend.set_enable_pin(false);

    let flux_target = backend.write_register(PID_TORQUE_FLUX_TARGET, 0).await;
    let uq_ud_ext = backend.write_register(UQ_UD_EXT, 0).await;
    let ramp_mode = backend
        .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
        .await;

    flux_target
        .and(uq_ud_ext)
        .and(ramp_mode)
        .map_err(|_| CommissionError::Spi)
}

pub(crate) async fn step_encoder_check<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    diagnostics: &mut CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let state = match phase {
        InnerPhase::EncoderCheck { state } => state,
        _ => return StepResult::ReadyNow,
    };

    match state {
        EncoderCheckState::Setup => {
            if EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev()).is_err() {
                return super::fail_inner(phase, CommissionError::EncoderFault);
            }

            if let Err(error) = start_encoder_current_drive(backend, 0, 0).await {
                return super::fail_inner(phase, error);
            }

            let deadline = match backend
                .elapsed_us()
                .checked_add(ENCODER_ALIGNMENT_SETTLE_US)
            {
                Some(deadline) => deadline,
                None => return super::fail_inner(phase, CommissionError::InvalidSchedule),
            };
            *state = EncoderCheckState::Settle {
                deadline_us: deadline,
                attempt: 0,
            };
            StepResult::At(deadline)
        }

        EncoderCheckState::Settle {
            deadline_us,
            attempt,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }

            let count_start = match read_mechanical_encoder_position_counts_stable(
                backend,
                Some(diagnostics),
                CommissionPhase::EncoderCheck,
            )
            .await
            {
                Ok(value) => value,
                Err(error) => return super::fail_inner(phase, error),
            };

            let att = *attempt;
            let sweep_deadline = match backend.elapsed_us().checked_add(ENCODER_DIRECTION_STEP_US) {
                Some(deadline) => deadline,
                None => return super::fail_inner(phase, CommissionError::InvalidSchedule),
            };
            *state = EncoderCheckState::Sweep {
                step: 0,
                deadline_us: sweep_deadline,
                count_start,
                attempt: att,
            };
            StepResult::At(sweep_deadline)
        }

        EncoderCheckState::Sweep {
            step,
            deadline_us,
            count_start,
            attempt,
        } => {
            let now_us = backend.elapsed_us();
            if now_us < *deadline_us {
                return StepResult::At(*deadline_us);
            }

            *step += 1;
            if *step > ENCODER_DIRECTION_STEPS {
                let cs = *count_start;
                let att = *attempt;
                *state = EncoderCheckState::Finalize {
                    count_start: cs,
                    attempt: att,
                };
                return StepResult::ReadyNow;
            }

            let phi = (ENCODER_DIRECTION_TARGET_PHI_E as u64 * *step as u64
                / ENCODER_DIRECTION_STEPS as u64) as u32;
            let _ = backend.write_register(PHI_E_EXT, phi).await;

            let next_deadline = match crate::commissioning::schedule::next_grid_deadline(
                *deadline_us,
                ENCODER_DIRECTION_STEP_US,
                now_us,
            ) {
                Ok(deadline) => deadline,
                Err(error) => return super::fail_inner(phase, error),
            };
            *deadline_us = next_deadline;
            StepResult::At(next_deadline)
        }

        EncoderCheckState::Finalize {
            count_start,
            attempt,
        } => {
            let count_end = match read_mechanical_encoder_position_counts_stable(
                backend,
                Some(diagnostics),
                CommissionPhase::EncoderCheck,
            )
            .await
            {
                Ok(value) => value,
                Err(error) => {
                    let _ = stop_encoder_current_drive(backend).await;
                    return super::fail_inner(phase, error);
                }
            };

            if let Err(error) = stop_encoder_current_drive(backend).await {
                return super::fail_inner(phase, error);
            }

            let domain =
                match EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev()) {
                    Ok(domain) => domain,
                    Err(_) => return super::fail_inner(phase, CommissionError::EncoderFault),
                };
            let delta = match domain.delta(*count_start, count_end) {
                Ok(delta) => delta,
                Err(_) => {
                    push_encoder_direction_result(diagnostics, *count_start, count_end, 0, true);
                    return super::fail_inner(phase, CommissionError::EncoderFault);
                }
            };
            defmt::info!(
                "encoder-dir: start={} end={} delta={}",
                count_start.raw_counts(),
                count_end.raw_counts(),
                delta.signed_counts()
            );
            let pole_pairs = backend.motor_pole_pairs();
            let window = encoder_direction_window(
                backend.mechanical_encoder_quadrature_counts_per_rev(),
                pole_pairs,
            );
            let above_window =
                window.is_some_and(|window| delta.signed_counts() > window.max_signed_counts);
            let accepted = window.is_some_and(|window| window.contains(delta.signed_counts()));
            if !accepted {
                push_encoder_direction_result(
                    diagnostics,
                    *count_start,
                    count_end,
                    delta.absolute_counts(),
                    true,
                );
                if let Some(window) = window {
                    push_encoder_direction_expected(
                        diagnostics,
                        ENCODER_DIRECTION_TARGET_PHI_E,
                        window.expected_signed_counts,
                        delta.signed_counts(),
                    );
                    push_encoder_direction_bounds(
                        diagnostics,
                        window.min_signed_counts,
                        window.max_signed_counts,
                        pole_pairs,
                    );
                }
                defmt::warn!(
                    "encoder-dir FAIL (attempt {}): start={} end={} delta={} pole_pairs={}",
                    attempt,
                    count_start.raw_counts(),
                    count_end.raw_counts(),
                    delta.signed_counts(),
                    pole_pairs
                );

                if above_window {
                    if matches!(sequence, InnerSequence::Selftest) {
                        selftest_results.set(
                            crate::selftest::SelftestStage::Encoder,
                            crate::selftest::SelftestStageValue::fail(0),
                        );
                    }
                    return super::fail_inner(phase, CommissionError::EncoderFault);
                }

                let next_attempt = *attempt + 1;
                if next_attempt < ENCODER_DIRECTION_MAX_ATTEMPTS {
                    if let Err(error) = start_encoder_current_drive(backend, 0, next_attempt).await
                    {
                        return super::fail_inner(phase, error);
                    }
                    let deadline = match backend
                        .elapsed_us()
                        .checked_add(ENCODER_ALIGNMENT_SETTLE_US)
                    {
                        Some(deadline) => deadline,
                        None => {
                            return super::fail_inner(phase, CommissionError::InvalidSchedule);
                        }
                    };
                    *state = EncoderCheckState::Settle {
                        deadline_us: deadline,
                        attempt: next_attempt,
                    };
                    return StepResult::At(deadline);
                }

                if matches!(sequence, InnerSequence::Selftest) {
                    selftest_results.set(
                        crate::selftest::SelftestStage::Encoder,
                        crate::selftest::SelftestStageValue::fail(0),
                    );
                }
                return super::fail_inner(phase, CommissionError::EncoderFault);
            }

            if matches!(sequence, InnerSequence::Selftest) {
                selftest_results.set(
                    crate::selftest::SelftestStage::Encoder,
                    crate::selftest::SelftestStageValue {
                        status: 0,
                        value: delta.absolute_counts(),
                    },
                );
                let reversed = delta.signed_counts() < 0;
                selftest_results.set(
                    crate::selftest::SelftestStage::EncoderDirection,
                    crate::selftest::SelftestStageValue {
                        status: 0,
                        value: if reversed { 1 } else { 0 },
                    },
                );
            }

            *phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::Setup {
                    count_before: count_end,
                },
            };
            StepResult::PhaseChanged(CommissionPhase::EncoderAlign)
        }
    }
}

pub(crate) async fn step_encoder_align<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    adc_cal: &mut Option<AdcCalResult>,
    calibrate_entry_gains: Option<CalibrateEntryGains>,
    _selftest_results: &mut crate::selftest::SelftestResults,
    diagnostics: &mut CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let state = match phase {
        InnerPhase::EncoderAlign { state } => state,
        _ => return StepResult::ReadyNow,
    };

    match state {
        EncoderAlignState::Setup { count_before } => {
            if EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev()).is_err() {
                return super::fail_inner(phase, CommissionError::EncoderFault);
            }
            if backend.current_limit().raw() == 0 {
                return super::fail_inner(phase, CommissionError::NoCurrent);
            }

            if let Err(error) = start_encoder_current_drive(backend, 0, 0).await {
                return super::fail_inner(phase, error);
            }

            if backend
                .read_register_verified(PHI_E_SELECTION, 1)
                .await
                .is_err()
                || backend
                    .read_register_verified(MODE_RAMP_MODE_MOTION, MOTION_TORQUE_ADVANCED_PI)
                    .await
                    .is_err()
            {
                let _ = stop_encoder_current_drive(backend).await;
                return super::fail_inner(phase, CommissionError::Spi);
            }

            backend.set_enable_pin(true);

            let min_movement = encoder_alignment_min_movement(
                backend.mechanical_encoder_quadrature_counts_per_rev(),
                backend.motor_pole_pairs(),
            );
            let deadline = match backend
                .elapsed_us()
                .checked_add(ENCODER_ALIGNMENT_SETTLE_US)
            {
                Some(deadline) => deadline,
                None => return super::fail_inner(phase, CommissionError::InvalidSchedule),
            };
            *state = EncoderAlignState::Settle {
                count_before: *count_before,
                deadline_us: deadline,
                min_movement,
            };
            StepResult::At(deadline)
        }

        EncoderAlignState::Settle {
            count_before,
            deadline_us,
            min_movement,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            *state = EncoderAlignState::StabilityCheck1 {
                count_before: *count_before,
                min_movement: *min_movement,
            };
            StepResult::ReadyNow
        }

        EncoderAlignState::StabilityCheck1 {
            count_before,
            min_movement,
        } => {
            let count1 = match read_mechanical_encoder_position_counts_stable(
                backend,
                Some(diagnostics),
                CommissionPhase::EncoderAlign,
            )
            .await
            {
                Ok(value) => value,
                Err(error) => {
                    let _ = stop_encoder_current_drive(backend).await;
                    return super::fail_inner(phase, error);
                }
            };
            let now = backend.elapsed_us();
            let (Some(deadline), Some(give_up)) = (
                now.checked_add(ENCODER_ALIGNMENT_STABILITY_US),
                now.checked_add(ENCODER_ALIGNMENT_STILL_TIMEOUT_US),
            ) else {
                return super::fail_inner(phase, CommissionError::InvalidSchedule);
            };
            *state = EncoderAlignState::StabilityCheck2 {
                count_before: *count_before,
                count1,
                deadline_us: deadline,
                min_movement: *min_movement,
                stable_pairs: 0,
                give_up_us: give_up,
            };
            StepResult::At(deadline)
        }

        EncoderAlignState::StabilityCheck2 {
            count_before,
            count1,
            deadline_us,
            min_movement,
            stable_pairs,
            give_up_us,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }

            let count2 = match read_mechanical_encoder_position_counts_stable(
                backend,
                Some(diagnostics),
                CommissionPhase::EncoderAlign,
            )
            .await
            {
                Ok(value) => value,
                Err(error) => {
                    let _ = stop_encoder_current_drive(backend).await;
                    return super::fail_inner(phase, error);
                }
            };

            let domain =
                match EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev()) {
                    Ok(domain) => domain,
                    Err(_) => {
                        let _ = stop_encoder_current_drive(backend).await;
                        return super::fail_inner(phase, CommissionError::EncoderFault);
                    }
                };
            let movement1 = domain.delta(*count_before, *count1);
            let movement2 = domain.delta(*count_before, count2);
            let stability = domain.delta(*count1, count2);
            let (movement1, movement2, stability) = match (movement1, movement2, stability) {
                (Ok(movement1), Ok(movement2), Ok(stability)) => (
                    movement1.absolute_counts(),
                    movement2.absolute_counts(),
                    stability.absolute_counts(),
                ),
                _ => {
                    let _ = stop_encoder_current_drive(backend).await;
                    return super::fail_inner(phase, CommissionError::EncoderFault);
                }
            };
            let movement = movement1.max(movement2);
            let stable = stability <= encoder_stability_counts(domain.counts_per_rev());
            let now = backend.elapsed_us();
            let keep_watching = if stable {
                *stable_pairs = stable_pairs.saturating_add(1);
                *stable_pairs < ENCODER_ALIGNMENT_STABLE_PAIRS
            } else {
                *stable_pairs = 0;
                now < *give_up_us
            };
            if movement >= *min_movement && keep_watching {
                *count1 = count2;
                *deadline_us = now.saturating_add(ENCODER_ALIGNMENT_STABILITY_US);
                return StepResult::At(*deadline_us);
            }
            if movement < *min_movement || !stable {
                push_encoder_align_result(diagnostics, movement, *min_movement, stability, true);
                defmt::warn!(
                    "encoder-align FAIL: count_before={} count1={} count2={} \
                     movement={} min={} stable={}",
                    count_before.raw_counts(),
                    count1.raw_counts(),
                    count2.raw_counts(),
                    movement,
                    *min_movement,
                    stable
                );
                let _ = stop_encoder_current_drive(backend).await;
                return super::fail_inner(phase, CommissionError::EncoderFault);
            }

            let counts_per_elec = counts_per_electrical_rev(
                backend.mechanical_encoder_quadrature_counts_per_rev(),
                backend.motor_pole_pairs(),
            );
            *state = EncoderAlignState::Commit {
                old_count: *count1,
                electrical_residual_counts: folded_electrical_residual_counts(
                    count1.raw_counts(),
                    counts_per_elec,
                ),
                stability_counts: saturating_u16(stability),
                movement_counts: saturating_u16(movement),
                min_movement_counts: saturating_u16(*min_movement),
                counts_per_electrical_rev: counts_per_elec,
            };
            StepResult::ReadyNow
        }

        EncoderAlignState::Commit {
            old_count,
            electrical_residual_counts,
            stability_counts,
            movement_counts,
            min_movement_counts,
            counts_per_electrical_rev,
        } => {
            if backend.write_register(ABN_DECODER_COUNT, 0).await.is_err()
                || backend
                    .write_register(ABN_DECODER_PHI_E_PHI_M_OFFSET, 0)
                    .await
                    .is_err()
                || backend.write_register(PHI_E_SELECTION, 3).await.is_err()
            {
                let _ = stop_encoder_current_drive(backend).await;
                return super::fail_inner(phase, CommissionError::Spi);
            }

            if backend
                .read_register_verified(PHI_E_SELECTION, 3)
                .await
                .is_err()
            {
                let _ = stop_encoder_current_drive(backend).await;
                return super::fail_inner(phase, CommissionError::Spi);
            }

            *state = EncoderAlignState::StopDrive {
                old_count: *old_count,
                electrical_residual_counts: *electrical_residual_counts,
                stability_counts: *stability_counts,
                movement_counts: *movement_counts,
                min_movement_counts: *min_movement_counts,
                counts_per_electrical_rev: *counts_per_electrical_rev,
            };
            StepResult::ReadyNow
        }

        EncoderAlignState::StopDrive {
            old_count,
            electrical_residual_counts,
            stability_counts,
            movement_counts,
            min_movement_counts,
            counts_per_electrical_rev,
        } => {
            if let Err(error) = stop_encoder_current_drive(backend).await {
                return super::fail_inner(phase, error);
            }

            if let Some(result) = adc_cal.as_mut() {
                result.encoder_count = old_count.raw_counts();
                result.encoder_alignment_electrical_residual_counts = *electrical_residual_counts;
                result.encoder_alignment_stability_counts = *stability_counts;
                result.encoder_alignment_movement_counts = *movement_counts;
                result.encoder_alignment_min_movement_counts = *min_movement_counts;
                result.encoder_alignment_counts_per_electrical_rev = *counts_per_electrical_rev;
            }

            match sequence {
                InnerSequence::Calibrate => {
                    let Some(gains) = calibrate_entry_gains else {
                        return super::fail_inner(phase, CommissionError::CurrentValidationFailed);
                    };
                    let Some(outer) = gains.outer else {
                        return super::fail_inner(phase, CommissionError::CurrentValidationFailed);
                    };
                    *phase = InnerPhase::ClosedLoopActivation {
                        state: ClosedLoopActivationState::WriteGains,
                        flux_p: gains.inner.flux_p,
                        flux_i: gains.inner.flux_i,
                        torque_p: gains.inner.torque_p,
                        torque_i: gains.inner.torque_i,
                        velocity_p: outer.velocity_p,
                        velocity_i: outer.velocity_i,
                        position_p: outer.position_p,
                        position_i: outer.position_i,
                        velocity_limit: gains.velocity_limit,
                        velocity_filter_hz: gains.velocity_filter_hz,
                        torque_filter_hz: gains.torque_filter_hz,
                        position_filter_hz: gains.position_filter_hz,
                        flux_filter_hz: gains.flux_filter_hz,
                        inner_result: None,
                        attempt: 0,
                    };
                    StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
                }
                InnerSequence::Selftest | InnerSequence::Commission => {
                    let voltage = backend.excitation_voltage() as u32;
                    *phase = InnerPhase::ElectricalIdentification {
                        state: ElectricalIdentificationState::ResistanceStart,
                        data: ElectricalIdentificationData::new(voltage, backend.current_limit())
                            .with_motor(
                                backend.tmc_config().inductance_voltage,
                                backend.motor_pole_pairs(),
                            ),
                    };
                    StepResult::PhaseChanged(CommissionPhase::ElectricalIdentification)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{
        MockBackend, SHARED_STATE_MUTEX, SpiOp, current_limit,
    };

    const ENCODER_MAX_REGISTER_OPS_PER_ARM: usize = 14;

    fn register_op_count(backend: &MockBackend) -> usize {
        backend
            .spi_log
            .iter()
            .filter(|op| !matches!(op, SpiOp::EnablePin(_)))
            .count()
    }

    #[test]
    fn encoder_direction_setup_and_late_sweep_use_absolute_grid_schedules() {
        pollster::block_on(async {
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Setup,
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.current_limit_value = current_limit(2_448);

            let setup = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(setup, StepResult::At(ENCODER_ALIGNMENT_SETTLE_US)));
            assert!(register_op_count(&backend) <= ENCODER_MAX_REGISTER_OPS_PER_ARM);
            assert!(matches!(
                phase,
                InnerPhase::EncoderCheck {
                    state: EncoderCheckState::Settle {
                        deadline_us: ENCODER_ALIGNMENT_SETTLE_US,
                        attempt: 0,
                    }
                }
            ));

            backend.elapsed_us_value = ENCODER_ALIGNMENT_SETTLE_US;
            let settled = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            match settled {
                StepResult::At(deadline) => {
                    assert_eq!(
                        deadline,
                        ENCODER_ALIGNMENT_SETTLE_US + ENCODER_DIRECTION_STEP_US
                    )
                }
                other => panic!("expected settle to schedule the sweep, got {other:?}"),
            }

            let InnerPhase::EncoderCheck {
                state: EncoderCheckState::Sweep { deadline_us, .. },
            } = &mut phase
            else {
                panic!("settle should enter direction sweep");
            };
            *deadline_us = 1_000;
            backend.elapsed_us_value = 35_000;
            backend.spi_log.clear();
            let late = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(
                matches!(late, StepResult::At(41_000)),
                "late sweep returned {late:?} in {phase:?}"
            );
            assert_eq!(register_op_count(&backend), 1);
            assert!(matches!(
                phase,
                InnerPhase::EncoderCheck {
                    state: EncoderCheckState::Sweep {
                        step: 1,
                        deadline_us: 41_000,
                        ..
                    }
                }
            ));
        });
    }

    #[test]
    fn encoder_alignment_waits_report_exact_deadlines_without_early_reads() {
        pollster::block_on(async {
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::Settle {
                    count_before: encoder_count(100),
                    deadline_us: 5_000,
                    min_movement: 2,
                },
            };
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 4_000;

            let early = step_encoder_align(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(early, StepResult::At(5_000)));
            assert_eq!(register_op_count(&backend), 0);

            backend.elapsed_us_value = 5_000;
            let due = step_encoder_align(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(due, StepResult::ReadyNow));

            backend.spi_log.clear();
            let first_read = step_encoder_align(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(first_read, StepResult::At(15_000)));
            assert_eq!(register_op_count(&backend), ENCODER_READ_SAMPLES);
        });
    }
    fn encoder_count(raw_counts: u32) -> EncoderCount {
        EncoderDomain::new(4_000).unwrap().count(raw_counts)
    }

    #[test]
    fn encoder_alignment_min_movement_uses_encoder_count_scale() {
        assert_eq!(encoder_alignment_min_movement(4_000, 50), 4);
    }

    #[test]
    fn encoder_stability_counts_keep_two_counts_on_a_1000_line_encoder() {
        assert_eq!(encoder_stability_counts(4_000), 2);
        assert_eq!(encoder_stability_counts(400), 2);
    }

    #[test]
    fn encoder_stability_counts_scale_with_high_resolution_encoders() {
        assert_eq!(encoder_stability_counts(262_144), 131);
    }

    #[test]
    fn encoder_direction_window_derives_expected_and_tolerance_from_pole_pairs() {
        let window = encoder_direction_window(4_000, 50).expect("50 pole pairs is valid");
        assert_eq!(window.expected_signed_counts, 40);
        assert_eq!(window.min_signed_counts, 30);
        assert_eq!(window.max_signed_counts, 50);
    }

    #[test]
    fn encoder_direction_window_scales_with_configured_pole_pairs() {
        let window = encoder_direction_window(4_000, 25).expect("25 pole pairs is valid");
        assert_eq!(window.expected_signed_counts, 80);
        assert_eq!(window.min_signed_counts, 60);
        assert_eq!(window.max_signed_counts, 100);
    }

    #[test]
    fn encoder_direction_window_rejects_zero_pole_pairs() {
        assert_eq!(encoder_direction_window(4_000, 0), None);
    }

    #[test]
    fn encoder_movement_current_target_escalates_within_current_limit() {
        assert_eq!(encoder_movement_current_target(2_448, 0), 1_000);
        assert_eq!(encoder_movement_current_target(2_448, 1), 1_836);
        assert_eq!(encoder_movement_current_target(2_448, 2), 2_448);
        assert_eq!(encoder_movement_current_target(2_448, 9), 2_448);
        assert_eq!(encoder_movement_current_target(50, 0), 50);
        assert_eq!(encoder_movement_current_target(50, 2), 50);
    }

    #[test]
    fn stable_helper_accepts_when_all_samples_agree() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            let _ = backend
                .write_register(foci_tmc4671::registers::ABN_DECODER_COUNT, 1234)
                .await;
            let result = read_mechanical_encoder_position_counts_stable(
                &mut backend,
                None,
                CommissionPhase::EncoderCheck,
            )
            .await;
            assert_eq!(result.unwrap().raw_counts(), 1234);
        });
    }

    #[test]
    fn stable_read_accepts_samples_across_encoder_origin() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[3999, 0, 1]);
            let result = read_mechanical_encoder_position_counts_stable(
                &mut backend,
                None,
                CommissionPhase::EncoderCheck,
            )
            .await;
            assert!(
                result.is_ok(),
                "wrap-near samples should be accepted, got {:?}",
                result
            );
            assert_eq!(result.unwrap().raw_counts(), 1);
        });
    }

    #[test]
    fn stable_read_half_turn_ambiguity_exhausts_all_attempts() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[0, 2000, 0, 0, 2000, 0, 0, 2000, 0]);
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = read_mechanical_encoder_position_counts_stable(
                &mut backend,
                Some(&mut diagnostics),
                CommissionPhase::EncoderCheck,
            )
            .await;

            assert_eq!(result, Err(CommissionError::EncoderFault));
            assert_eq!(
                backend.abn_decoder_count_pos,
                ENCODER_READ_SAMPLES * ENCODER_READ_ATTEMPTS
            );
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].code, DIAG_ENCODER_READ_UNSTABLE);
        });
    }

    #[test]
    fn stable_helper_rejects_after_exhausted_attempts() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[500, 600, 500, 600, 500, 600, 500, 600, 500]);
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let result = read_mechanical_encoder_position_counts_stable(
                &mut backend,
                Some(&mut diagnostics),
                CommissionPhase::EncoderCheck,
            )
            .await;
            match result {
                Err(crate::autotune::CommissionError::EncoderFault) => {}
                other => panic!("expected EncoderFault, got {:?}", other),
            }
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].phase, CommissionPhase::EncoderCheck.to_u8());
            assert_eq!(events[0].code, DIAG_ENCODER_READ_UNSTABLE);
            assert_eq!(events[0].status, 1);
            assert_eq!(events[0].value0, 500);
            assert_eq!(events[0].value1, 600);
            assert_eq!(events[0].value2, 500);
        });
    }

    #[test]
    fn encoder_check_zero_delta_retries_before_exhausting_attempts() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(1234),
                    attempt: 0,
                },
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.current_limit_value = current_limit(2_448);
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[1234, 1234, 1234]);

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(result, StepResult::At(_)),
                "zero delta on the first attempt should retry, got {result:?}"
            );
            assert!(matches!(
                phase,
                InnerPhase::EncoderCheck {
                    state: EncoderCheckState::Settle { attempt: 1, .. }
                }
            ));
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].code, DIAG_ENCODER_DIRECTION_RESULT);
            assert_eq!(events[0].status, 1);
        });
    }

    #[test]
    fn encoder_check_zero_delta_records_diagnostic() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(1234),
                    attempt: ENCODER_DIRECTION_MAX_ATTEMPTS - 1,
                },
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[1234, 1234, 1234]);

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].phase, CommissionPhase::EncoderCheck.to_u8());
            assert_eq!(events[0].code, DIAG_ENCODER_DIRECTION_RESULT);
            assert_eq!(events[0].status, 1);
            assert_eq!(events[0].value0, 1234);
            assert_eq!(events[0].value1, 1234);
            assert_eq!(events[0].value2, 0);
        });
    }

    #[test]
    fn encoder_direction_origin_crossing_is_positive() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(3998),
                    attempt: 0,
                },
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend
                .last_motor_sentinel_stored
                .set(Some(pole_pairs_sentinel(500)));
            backend.set_abn_decoder_count_sequence(&[2, 2, 2]);

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Selftest,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::EncoderAlign)
            ));
            assert_eq!(
                selftest_results
                    .stage(crate::selftest::SelftestStage::Encoder)
                    .unwrap()
                    .value,
                4
            );
            assert_eq!(
                selftest_results
                    .stage(crate::selftest::SelftestStage::EncoderDirection)
                    .unwrap()
                    .value,
                0
            );
        });
    }

    fn pole_pairs_sentinel(n_pole_pairs: u16) -> u32 {
        foci_tmc4671::registers::MotorTypeNPolePairs::new()
            .with_n_pole_pairs(n_pole_pairs)
            .into()
    }

    fn finalize_backend(count_end: u32, attempt: u8) -> (InnerPhase, MockBackend) {
        let phase = InnerPhase::EncoderCheck {
            state: EncoderCheckState::Finalize {
                count_start: encoder_count(0),
                attempt,
            },
        };
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(2_448);
        backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
        backend
            .last_motor_sentinel_stored
            .set(Some(pole_pairs_sentinel(50)));
        backend.set_abn_decoder_count_sequence(&[count_end, count_end, count_end]);
        (phase, backend)
    }

    #[test]
    fn encoder_direction_check_accepts_correct_sign_and_ppr_consistent_displacement() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(40, 0);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::EncoderAlign)
            ));
        });
    }

    #[test]
    fn encoder_direction_check_accepts_wrap_boundary_displacement() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(3980),
                    attempt: 0,
                },
            };
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend
                .last_motor_sentinel_stored
                .set(Some(pole_pairs_sentinel(50)));
            backend.set_abn_decoder_count_sequence(&[20, 20, 20]);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::EncoderAlign)
            ));
        });
    }

    #[test]
    fn encoder_direction_check_rejects_reversed_sign_with_plausible_magnitude() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) =
                finalize_backend(3_960, ENCODER_DIRECTION_MAX_ATTEMPTS - 1);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
        });
    }

    #[test]
    fn encoder_direction_check_rejects_underscale_displacement() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(10, ENCODER_DIRECTION_MAX_ATTEMPTS - 1);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);

            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 3);
            assert_eq!(events[0].code, DIAG_ENCODER_DIRECTION_RESULT);
            assert_eq!(events[0].value0, 0);
            assert_eq!(events[0].value1, 10);
            assert_eq!(events[0].value2, 10);
            assert_eq!(events[1].code, DIAG_ENCODER_DIRECTION_EXPECTED);
            assert_eq!(events[1].value0, ENCODER_DIRECTION_TARGET_PHI_E);
            assert_eq!(events[1].value1, 40);
            assert_eq!(events[1].value2, 10);
            assert_eq!(events[2].code, DIAG_ENCODER_DIRECTION_BOUNDS);
            assert_eq!(events[2].value0, 30);
            assert_eq!(events[2].value1, 50);
            assert_eq!(events[2].value2, 50);
        });
    }

    #[test]
    fn encoder_direction_check_rejects_overscale_displacement() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) =
                finalize_backend(1_000, ENCODER_DIRECTION_MAX_ATTEMPTS - 1);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
        });
    }

    #[test]
    fn encoder_direction_check_overscale_fails_immediately_without_retry() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(1_000, 0);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
        });
    }

    #[test]
    fn encoder_direction_check_underscale_still_retries_with_escalated_current() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(10, 0);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(result, StepResult::At(_)),
                "underscale on the first attempt should retry, got {result:?}"
            );
            assert!(matches!(
                phase,
                InnerPhase::EncoderCheck {
                    state: EncoderCheckState::Settle { attempt: 1, .. }
                }
            ));
        });
    }

    #[test]
    fn encoder_direction_check_reversed_sign_still_retries_with_escalated_current() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(3_960, 0);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(result, StepResult::At(_)),
                "reversed sign on the first attempt should retry, got {result:?}"
            );
            assert!(matches!(
                phase,
                InnerPhase::EncoderCheck {
                    state: EncoderCheckState::Settle { attempt: 1, .. }
                }
            ));
        });
    }

    #[test]
    fn encoder_direction_check_accepts_lower_tolerance_boundary() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(30, 0);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::EncoderAlign)
            ));
        });
    }

    #[test]
    fn encoder_direction_check_rejects_just_below_lower_tolerance_boundary() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(29, ENCODER_DIRECTION_MAX_ATTEMPTS - 1);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
        });
    }

    #[test]
    fn encoder_direction_check_accepts_upper_tolerance_boundary() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(50, 0);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::EncoderAlign)
            ));
        });
    }

    #[test]
    fn encoder_direction_check_rejects_just_above_upper_tolerance_boundary() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(51, ENCODER_DIRECTION_MAX_ATTEMPTS - 1);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
        });
    }

    #[test]
    fn encoder_direction_check_uses_configured_pole_pairs_not_a_hardcoded_constant() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(0),
                    attempt: 0,
                },
            };
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend
                .last_motor_sentinel_stored
                .set(Some(pole_pairs_sentinel(25)));
            backend.set_abn_decoder_count_sequence(&[80, 80, 80]);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::EncoderAlign)
            ));
        });
    }

    #[test]
    fn encoder_direction_check_fails_closed_without_a_calibrated_pole_pair_count() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let (mut phase, mut backend) = finalize_backend(40, ENCODER_DIRECTION_MAX_ATTEMPTS - 1);
            backend.last_motor_sentinel_stored.set(None);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].code, DIAG_ENCODER_DIRECTION_RESULT);
        });
    }

    #[test]
    fn encoder_direction_half_turn_stops_current_without_committing_direction() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(0),
                    attempt: 0,
                },
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[2000, 2000, 2000]);

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Selftest,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::EncoderFault)
            ));
            assert!(
                selftest_results
                    .stage(crate::selftest::SelftestStage::EncoderDirection)
                    .is_none()
            );
        });
    }

    #[test]
    fn stop_encoder_current_drive_lowers_gpio_before_a_failed_write() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.fail_on_write = Some(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr());

            let result = stop_encoder_current_drive(&mut backend).await;

            assert!(matches!(result, Err(CommissionError::Spi)));
            assert!(!backend.enable_pin);
        });
    }

    #[test]
    fn encoder_check_finalize_fails_when_stop_drive_write_fails() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Finalize {
                    count_start: encoder_count(3998),
                    attempt: 0,
                },
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[2, 2, 2]);
            backend.fail_on_write = Some(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET.addr());

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Selftest,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, InnerPhase::Failed(CommissionError::Spi)));
        });
    }

    #[test]
    fn invalid_encoder_domain_fails_before_direction_drive() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderCheck {
                state: EncoderCheckState::Setup,
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.current_limit_value = current_limit(1000);
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 0;

            let result = step_encoder_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(!backend.spi_log.iter().any(|op| matches!(
                op,
                crate::commission_dispatch::test_support::SpiOp::Write(_, _)
                    | crate::commission_dispatch::test_support::SpiOp::EnablePin(true)
            )));
        });
    }

    #[test]
    fn encoder_align_stability_accepts_wrap_boundary() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StabilityCheck2 {
                    count_before: encoder_count(3998),
                    count1: encoder_count(3999),
                    deadline_us: 0,
                    min_movement: 2,
                    // The final pair: stable commits, unstable fails.
                    stable_pairs: ENCODER_ALIGNMENT_STABLE_PAIRS - 1,
                    give_up_us: 0,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 0,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend
                .last_motor_sentinel_stored
                .set(Some(pole_pairs_sentinel(50)));
            backend.set_abn_decoder_count_sequence(&[0, 0, 0]);

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            let InnerPhase::EncoderAlign {
                state:
                    EncoderAlignState::Commit {
                        old_count,
                        electrical_residual_counts: 1,
                        stability_counts: 1,
                        movement_counts: 2,
                        min_movement_counts: 2,
                        counts_per_electrical_rev: 80,
                    },
            } = phase
            else {
                panic!("valid wrap crossing should advance to alignment commit");
            };
            assert_eq!(old_count.raw_counts(), 3999);
        });
    }

    #[test]
    fn invalid_encoder_domain_fails_before_alignment_drive() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::Setup {
                    count_before: encoder_count(10),
                },
            };
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 0;

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(backend.spi_log.is_empty());
        });
    }

    #[test]
    fn active_alignment_spi_failures_stop_drive() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            use foci_tmc4671::registers::{ABN_DECODER_COUNT, PHI_E_SELECTION};

            for case in 0..3 {
                let mut phase = if case == 0 {
                    InnerPhase::EncoderAlign {
                        state: EncoderAlignState::Setup {
                            count_before: encoder_count(10),
                        },
                    }
                } else {
                    InnerPhase::EncoderAlign {
                        state: EncoderAlignState::Commit {
                            old_count: encoder_count(10),
                            electrical_residual_counts: 1,
                            stability_counts: 1,
                            movement_counts: 4,
                            min_movement_counts: 2,
                            counts_per_electrical_rev: 80,
                        },
                    }
                };
                let mut backend = MockBackend::new();
                backend.current_limit_value = current_limit(1000);
                backend.enable_pin = case != 0;
                if case == 1 {
                    backend.fail_on_write = Some(ABN_DECODER_COUNT.addr());
                } else {
                    backend.read_override = Some((PHI_E_SELECTION.addr(), 0));
                }

                let result = step_encoder_align(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut None,
                    None,
                    &mut crate::selftest::SelftestResults::default(),
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
                    "case {case} left alignment drive enabled"
                );
            }
        });
    }

    #[test]
    fn encoder_align_rejection_records_movement_diagnostic() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StabilityCheck2 {
                    count_before: encoder_count(1000),
                    count1: encoder_count(1001),
                    deadline_us: 0,
                    min_movement: 4,
                    // The final pair: stable commits, unstable fails.
                    stable_pairs: ENCODER_ALIGNMENT_STABLE_PAIRS - 1,
                    give_up_us: 0,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 0,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[1001, 1001, 1001]);

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            let events: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(events.len(), 1);
            assert_eq!(events[0].phase, CommissionPhase::EncoderAlign.to_u8());
            assert_eq!(events[0].code, DIAG_ENCODER_ALIGN_RESULT);
            assert_eq!(events[0].status, 1);
            assert_eq!(events[0].value0, 1);
            assert_eq!(events[0].value1, 4);
            assert_eq!(events[0].value2, 0);
        });
    }

    #[test]
    fn encoder_alignment_half_turn_stops_drive_without_committing_alignment() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StabilityCheck2 {
                    count_before: encoder_count(0),
                    count1: encoder_count(2000),
                    deadline_us: 0,
                    min_movement: 2,
                    // The final pair: stable commits, unstable fails.
                    stable_pairs: ENCODER_ALIGNMENT_STABLE_PAIRS - 1,
                    give_up_us: 0,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 77,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[2000, 2000, 2000]);

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::EncoderFault)
            ));
            assert!(!backend.enable_pin);
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::EncoderFault)
            ));
            assert_eq!(adc_cal.unwrap().encoder_count, 77);
        });
    }

    #[test]
    fn calibrate_stop_drive_enters_closed_loop_with_entry_gains() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StopDrive {
                    old_count: encoder_count(42),
                    electrical_residual_counts: 2,
                    stability_counts: 1,
                    movement_counts: 25,
                    min_movement_counts: 2,
                    counts_per_electrical_rev: 80,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 0,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            let gains = CalibrateEntryGains {
                inner: crate::pid::InnerLoopGains {
                    flux_p: crate::pid::p_gain(711),
                    flux_i: crate::pid::i_gain(2_544),
                    torque_p: crate::pid::p_gain(712),
                    torque_i: crate::pid::i_gain(2_560),
                },
                outer: Some(crate::pid::test_outer_gains(627, 1, 1434, 2)),
                velocity_limit: 500_000,
                velocity_filter_hz: 11,
                torque_filter_hz: 13,
                position_filter_hz: 17,
                flux_filter_hz: 19,
            };

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                Some(gains),
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::ClosedLoopActivation)
            ));
            assert_eq!(adc_cal.as_ref().unwrap().encoder_count, 42);
            let InnerPhase::ClosedLoopActivation {
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
                ..
            } = phase
            else {
                panic!("calibrate StopDrive should enter ClosedLoopActivation");
            };

            assert_eq!(flux_p, gains.inner.flux_p);
            assert_eq!(flux_i, gains.inner.flux_i);
            assert_eq!(torque_p, gains.inner.torque_p);
            assert_eq!(torque_i, gains.inner.torque_i);
            let outer = gains.outer.unwrap();
            assert_eq!(velocity_p, outer.velocity_p);
            assert_eq!(velocity_i, outer.velocity_i);
            assert_eq!(position_p, outer.position_p);
            assert_eq!(position_i, outer.position_i);
            assert_eq!(velocity_limit, gains.velocity_limit);
            assert_eq!(velocity_filter_hz, gains.velocity_filter_hz);
            assert_eq!(torque_filter_hz, gains.torque_filter_hz);
            assert_eq!(position_filter_hz, gains.position_filter_hz);
            assert_eq!(flux_filter_hz, gains.flux_filter_hz);
        });
    }

    #[test]
    fn absent_calibrate_entry_gains_fails_commissioning_not_arms() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StopDrive {
                    old_count: encoder_count(42),
                    electrical_residual_counts: 2,
                    stability_counts: 1,
                    movement_counts: 25,
                    min_movement_counts: 2,
                    counts_per_electrical_rev: 80,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 0,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Calibrate,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::CurrentValidationFailed)
            ));
        });
    }

    #[test]
    fn electrical_identification_entry_freezes_the_current_limit() {
        pollster::block_on(async {
            let _guard = SHARED_STATE_MUTEX.lock().await;
            let mut phase = InnerPhase::EncoderAlign {
                state: EncoderAlignState::StopDrive {
                    old_count: encoder_count(42),
                    electrical_residual_counts: 2,
                    stability_counts: 1,
                    movement_counts: 25,
                    min_movement_counts: 2,
                    counts_per_electrical_rev: 80,
                },
            };
            let mut adc_cal = Some(AdcCalResult {
                adc_i0: 1,
                adc_i1: 2,
                encoder_count: 0,
                encoder_alignment_electrical_residual_counts: 0,
                encoder_alignment_stability_counts: 0,
                encoder_alignment_movement_counts: 0,
                encoder_alignment_min_movement_counts: 0,
                encoder_alignment_counts_per_electrical_rev: 0,
                adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
                adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            });
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.current_limit_value = current_limit(2816);

            let result = step_encoder_align(
                &mut phase,
                InnerSequence::Commission,
                &mut adc_cal,
                None,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::ElectricalIdentification)
            ));
            let InnerPhase::ElectricalIdentification { data, .. } = phase else {
                panic!("commission StopDrive should enter ElectricalIdentification");
            };
            assert_eq!(u32::from(data.current_limit), 2816);
        });
    }
}
