//! Phase 5: Electrical identification (resistance and inductance).

use super::{
    ElectricalIdentificationData, ElectricalIdentificationState, InductanceCaptureState,
    InductanceRestoreOperation, InnerPhase,
};
use crate::autotune::{
    CommissionError, CommissionPhase, ELECTRICAL_MODEL_RESISTANCE_SCALE, ElectricalModel,
    InnerSequence, Permille,
};
use crate::commissioning::backend::timed_read_register_pair;
use crate::commissioning::capture_timing::{
    CaptureTiming, CaptureTimingEvidence, SampleStart, TimingEnvelope,
};
use crate::commissioning::encoder_domain::{EncoderDelta, EncoderDomain, EncoderMathError};
use crate::commissioning::types::{
    COMMISSION_TIMING_METHOD_INDUCTANCE, COMMISSION_TIMING_METHOD_RESISTANCE, CommissioningBackend,
    INDUCTANCE_SOURCE_ROTATING_FIELD, INDUCTANCE_WARN_DRIFT, INDUCTANCE_WARN_HIGH_RMS,
    INDUCTANCE_WARN_MAG_CROSSCHECK, INDUCTANCE_WARN_STATUS_FLAGS, InductanceEstimateEvidence,
    InductanceEvidence, InductanceFrameEvidence, InductanceRunEvidence, StepResult,
};
use crate::electrical_capture::{
    PhaseDelayEstimate, PhaseDelayInput, PhaseDelayRejection, estimate_phase_delay,
};
use crate::tmc_register_values::{
    MOTION_STOPPED, MOTION_UQ_UD_EXT, PWM_CHOP_CENTERED_FOC, PWM_CHOP_OFF,
};
use foci_tmc4671::registers::Register;

const ELECTRICAL_COOLDOWN_US: u64 = 50_000;

pub(super) const INDUCTANCE_AC_UD_EXT: i16 = 768;
/// 1 kHz electrical on a 50-pole-pair stepper; see [`inductance_openloop_rpm`].
pub(super) const INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM: u32 = 1_200;
const INDUCTANCE_OPENLOOP_ACCELERATION: u32 = 0x000f_ffff;
const INDUCTANCE_OPENLOOP_RAMP_POLL_US: u64 = 200;
/// Floor of the open-loop ramp timeout; see [`inductance_openloop_ramp_timeout_us`].
const INDUCTANCE_OPENLOOP_RAMP_TIMEOUT_US: u64 = 2_000;
const INDUCTANCE_AC_SETTLE_US: u64 = 8_500;
const INDUCTANCE_AC_SAMPLE_PERIOD_US: u16 = 200;
const INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS: u32 = 20 * 65_536;
const INDUCTANCE_MIN_ABS_IQ_MILLI_COUNT: i32 = 30_000;
/// Rotor motion allowed during the capture: 8 counts of a 1000-line
/// (4000-count) encoder, as a fraction of a revolution.
const INDUCTANCE_MAX_ENCODER_DELTA_COUNTS: u32 = 8;
const INDUCTANCE_MAX_ENCODER_DELTA_REV_DIVISOR: u32 = 500;
const INDUCTANCE_MAX_DRIFT_WARN_PERMILLE: u16 = 70;
const INDUCTANCE_MAX_IQ_RMS_WARN_PERMILLE: u16 = 250;
const INDUCTANCE_MAX_MAG_CROSSCHECK_WARN_PERMILLE: u16 = 100;
const INDUCTANCE_REALIZED_FREQUENCY_TARGET_MILLIHZ: u32 = 1_000_000;
const INDUCTANCE_REALIZED_FREQUENCY_TOLERANCE_MILLIHZ: u32 = 50_000;
const INDUCTANCE_MIN_AC_SAMPLE_COUNT: u16 = 90;
const INDUCTANCE_MAX_AC_SAMPLE_COUNT: u16 = 256;
const PHI_E_SELECTION_OPENLOOP: u32 = 2;
const L_INT_OMEGA_SCALE_MILLI: u64 = 6_283;

/// Open-loop `OPENLOOP_VELOCITY_TARGET` (mechanical rpm) that rotates the
/// field at the 1 kHz electrical frequency the capture is validated against.
pub(super) fn inductance_openloop_rpm(pole_pairs: u16) -> u32 {
    (INDUCTANCE_REALIZED_FREQUENCY_TARGET_MILLIHZ * 60 / 1_000) / u32::from(pole_pairs.max(1))
}

/// Time allowed for the open-loop ramp to reach `target_rpm`: 1.5x the
/// nominal ramp at `INDUCTANCE_OPENLOOP_ACCELERATION`, never below 2 ms.
fn inductance_openloop_ramp_timeout_us(target_rpm: u32) -> u64 {
    (u64::from(target_rpm) * 1_500_000 / u64::from(INDUCTANCE_OPENLOOP_ACCELERATION))
        .max(INDUCTANCE_OPENLOOP_RAMP_TIMEOUT_US)
}

fn inductance_max_encoder_delta_counts(
    encoder_domain: crate::commissioning::encoder_domain::EncoderDomain,
) -> u32 {
    (encoder_domain.counts_per_rev() / INDUCTANCE_MAX_ENCODER_DELTA_REV_DIVISOR)
        .max(INDUCTANCE_MAX_ENCODER_DELTA_COUNTS)
}

const INDUCTANCE_DIAGNOSTIC_CURRENT_FRACTION: Permille = Permille::new(200);

const _: () = assert!(
    INDUCTANCE_DIAGNOSTIC_CURRENT_FRACTION.raw() >= 1
        && INDUCTANCE_DIAGNOSTIC_CURRENT_FRACTION.raw() <= 1000
);

#[cfg(test)]
const fn scheduled_samples_for_fixed_window(frequency_millihz: u32) -> u32 {
    if frequency_millihz == 0 {
        return 0;
    }
    let window_us = INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS as u64 * 1_000_000_000
        / (65_536 * frequency_millihz as u64);
    (window_us / INDUCTANCE_AC_SAMPLE_PERIOD_US as u64) as u32
}

const DIAG_ELECTRICAL_ID_EXCITATION: u8 = 1;
const DIAG_ELECTRICAL_ID_RESISTANCE_A: u8 = 2;
const DIAG_ELECTRICAL_ID_RESISTANCE_B: u8 = 3;
const DIAG_ELECTRICAL_ID_REJECT_NO_SAMPLES: u8 = 20;
const DIAG_ELECTRICAL_ID_REJECT_MODEL_SCALE_ZERO: u8 = 22;
const DIAG_ELECTRICAL_ID_REJECT_THETA_TOO_LARGE: u8 = 29;
const DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_FREQUENCY: u8 = 32;
const DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_CAPTURE: u8 = 33;
const DIAG_ELECTRICAL_ID_INDUCTANCE_STATUS_REJECT: u8 = 41;

pub(crate) const INNER_WARN_BIT_RESISTANCE_MISMATCH: u8 = 1 << 0;
pub(crate) const INNER_WARN_BIT_THETA_TAU: u8 = 1 << 3;

const RESISTANCE_MISMATCH_WARN_PERMILLE: u32 = 50;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct ModelAssembly {
    pub(crate) model: ElectricalModel,
    pub(crate) warning_flags: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum ModelAssemblyError {
    SampleCount,
    ModelScaleZero { l_count_micro: u32 },
    ThetaTooLarge { theta_us: u32, tau_us: u32 },
}

fn log_model_assembly_failure(err: ModelAssemblyError) {
    match err {
        ModelAssemblyError::SampleCount => {
            defmt::warn!("electrical id: no usable per-coil samples");
        }
        ModelAssemblyError::ModelScaleZero { l_count_micro } => {
            defmt::warn!(
                "electrical id: model scale zero (l_count_micro={})",
                l_count_micro
            );
        }
        ModelAssemblyError::ThetaTooLarge { theta_us, tau_us } => {
            defmt::warn!(
                "electrical id: theta={}us >= tau={}us / 2",
                theta_us,
                tau_us
            );
        }
    }
}

fn diagnostic_for_model_assembly_failure(
    err: ModelAssemblyError,
) -> crate::selftest::CommissionDiagnostic {
    let (code, value0, value1, value2) = match err {
        ModelAssemblyError::SampleCount => (DIAG_ELECTRICAL_ID_REJECT_NO_SAMPLES, 0, 0, 0),
        ModelAssemblyError::ModelScaleZero { l_count_micro } => (
            DIAG_ELECTRICAL_ID_REJECT_MODEL_SCALE_ZERO,
            l_count_micro,
            l_count_micro,
            0,
        ),
        ModelAssemblyError::ThetaTooLarge { theta_us, tau_us } => (
            DIAG_ELECTRICAL_ID_REJECT_THETA_TOO_LARGE,
            theta_us,
            tau_us,
            0,
        ),
    };
    crate::selftest::CommissionDiagnostic {
        phase: CommissionPhase::ElectricalIdentification.to_u8(),
        code,
        status: 1,
        value0,
        value1,
        value2,
    }
}

impl From<ModelAssemblyError> for CommissionError {
    fn from(err: ModelAssemblyError) -> Self {
        match err {
            ModelAssemblyError::SampleCount => Self::ElectricalModelNoSamples,
            ModelAssemblyError::ModelScaleZero { .. } => Self::ElectricalModelScaleZero,
            ModelAssemblyError::ThetaTooLarge { .. } => Self::ElectricalThetaTooLarge,
        }
    }
}

fn permille_diff(a: u32, b: u32) -> u32 {
    let average = (a as u64 + b as u64) / 2;
    if average == 0 {
        return 0;
    }
    let diff = a.abs_diff(b) as u64;
    (diff * 1000 / average) as u32
}

fn electrical_id_measurement_diagnostic(
    code: u8,
    value0: u32,
    value1: u32,
    value2: u32,
) -> crate::selftest::CommissionDiagnostic {
    crate::selftest::CommissionDiagnostic {
        phase: CommissionPhase::ElectricalIdentification.to_u8(),
        code,
        status: 0,
        value0,
        value1,
        value2,
    }
}

fn per_coil_resistance_slope(
    data: &ElectricalIdentificationData,
    coil_idx: usize,
) -> Option<(u32, u32, u32)> {
    let result = data.resistance_result?;
    let axis = if coil_idx == 0 {
        result.axis0
    } else {
        result.axis1
    };
    let r_count_milli = axis.r_count_slope_milli.max(0) as u32;
    if r_count_milli == 0 {
        return None;
    }
    Some((r_count_milli, r_count_milli, u32::from(axis.selected_count)))
}

fn l_count_micro_from_reactance(
    x_average_count_ratio_milli: u32,
    realized_frequency_millihz: u32,
) -> Option<u32> {
    if x_average_count_ratio_milli == 0 || realized_frequency_millihz == 0 {
        return None;
    }
    let numerator = u128::from(x_average_count_ratio_milli).checked_mul(1_000_000_000)?;
    let denominator =
        u128::from(L_INT_OMEGA_SCALE_MILLI).checked_mul(u128::from(realized_frequency_millihz))?;
    u32::try_from(numerator.checked_div(denominator)?).ok()
}

fn record_electrical_id_measurement_diagnostics(
    data: &ElectricalIdentificationData,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
) {
    diagnostics.push(electrical_id_measurement_diagnostic(
        DIAG_ELECTRICAL_ID_EXCITATION,
        data.voltage,
        u32::from(data.inductance_ud.unsigned_abs()),
        data.inductance_openloop_rpm,
    ));
    for coil_idx in 0..2 {
        if let Some((avg_current, r_count_milli, count)) = per_coil_resistance_slope(data, coil_idx)
        {
            diagnostics.push(electrical_id_measurement_diagnostic(
                if coil_idx == 0 {
                    DIAG_ELECTRICAL_ID_RESISTANCE_A
                } else {
                    DIAG_ELECTRICAL_ID_RESISTANCE_B
                },
                avg_current,
                r_count_milli,
                count,
            ));
        }
    }
}

fn compute_tau_e_us(data: &ElectricalIdentificationData) -> Result<u32, ModelAssemblyError> {
    let result = data
        .resistance_result
        .ok_or(ModelAssemblyError::SampleCount)?;
    let r_count_milli = result.selected_r_count_slope_milli.max(0) as u32;
    if r_count_milli == 0 {
        return Err(ModelAssemblyError::SampleCount);
    }
    let inductance = data.inductance.ok_or(ModelAssemblyError::SampleCount)?;
    let l_count_micro = l_count_micro_from_reactance(
        inductance.estimate.x_average_count_ratio_milli,
        inductance.run.realized_frequency_millihz,
    )
    .ok_or(ModelAssemblyError::ModelScaleZero { l_count_micro: 0 })?;
    if l_count_micro == 0 {
        return Err(ModelAssemblyError::ModelScaleZero { l_count_micro });
    }
    Ok(
        (u64::from(l_count_micro) * u64::from(ELECTRICAL_MODEL_RESISTANCE_SCALE)
            / u64::from(r_count_milli)) as u32,
    )
}

fn phase_delay_from_evidence(
    data: &ElectricalIdentificationData,
) -> Result<PhaseDelayEstimate, PhaseDelayRejection> {
    let inductance = data.inductance.ok_or(PhaseDelayRejection::NoSamples)?;
    let r_count_milli = data
        .resistance_result
        .map(|result| result.selected_r_count_slope_milli.max(0) as u32)
        .unwrap_or(0);
    estimate_phase_delay(&PhaseDelayInput {
        id_mean_milli_count: inductance.frame.id_mean_milli_count,
        iq_mean_milli_count: inductance.frame.iq_mean_milli_count,
        id_rms_milli_count: inductance.frame.id_rms_milli_count,
        iq_rms_milli_count: inductance.frame.iq_rms_milli_count,
        sample_count: inductance.run.sample_count,
        r_count_milli,
        ud_count: data.inductance_ud.unsigned_abs(),
        realized_frequency_millihz: inductance.run.realized_frequency_millihz,
    })
}

fn assemble_electrical_model(
    data: &ElectricalIdentificationData,
    configured_theta_e_us: u32,
) -> Result<ModelAssembly, ModelAssemblyError> {
    let result = data
        .resistance_result
        .ok_or(ModelAssemblyError::SampleCount)?;
    let mut r_count_milli_per_coil = [0u32; 2];
    let axis_results = [result.axis0, result.axis1];
    for (coil_idx, axis) in axis_results.iter().enumerate() {
        let r_count_milli_coil = axis.r_count_slope_milli.max(0) as u32;
        if r_count_milli_coil == 0 {
            return Err(ModelAssemblyError::SampleCount);
        }
        r_count_milli_per_coil[coil_idx] = r_count_milli_coil;
    }
    let r_count_milli = result.selected_r_count_slope_milli.max(0) as u32;
    if r_count_milli == 0 {
        return Err(ModelAssemblyError::SampleCount);
    }

    let r_diff_permille = permille_diff(r_count_milli_per_coil[0], r_count_milli_per_coil[1]);
    let inductance = data.inductance.ok_or(ModelAssemblyError::SampleCount)?;
    let l_count_micro = l_count_micro_from_reactance(
        inductance.estimate.x_average_count_ratio_milli,
        inductance.run.realized_frequency_millihz,
    )
    .ok_or(ModelAssemblyError::ModelScaleZero { l_count_micro: 0 })?;

    let tau_e_us = compute_tau_e_us(data)?;
    if l_count_micro == 0 {
        return Err(ModelAssemblyError::ModelScaleZero { l_count_micro });
    }

    let phase = phase_delay_from_evidence(data);
    let (theta_e_us_input, theta_source) = resolve_theta_e_us_input(
        phase.as_ref().ok().map(|e| e.theta_e_us),
        tau_e_us,
        configured_theta_e_us,
    );
    let id_mean = inductance.frame.id_mean_milli_count;
    let iq_mean = inductance.frame.iq_mean_milli_count;
    defmt::info!(
        "electrical id: theta_e={}us sigma={:?}us measured={:?}us configured={}us source={:?} \
         id={} iq={} excess_centideg={:?} reject={:?}",
        theta_e_us_input,
        phase.as_ref().ok().map(|e| e.sigma_us),
        phase.as_ref().ok().map(|e| e.theta_e_us),
        configured_theta_e_us,
        theta_source,
        id_mean,
        iq_mean,
        phase.as_ref().ok().map(|e| e.excess_centideg),
        phase.as_ref().err(),
    );
    let theta_source_wire: u8 = match theta_source {
        ThetaSource::Measured => 0,
        ThetaSource::ConfigFallback => 1,
    };

    if theta_e_us_input >= tau_e_us / 2 {
        return Err(ModelAssemblyError::ThetaTooLarge {
            theta_us: theta_e_us_input,
            tau_us: tau_e_us,
        });
    }

    let mut warning_flags: u8 = 0;
    if r_diff_permille >= RESISTANCE_MISMATCH_WARN_PERMILLE {
        warning_flags |= INNER_WARN_BIT_RESISTANCE_MISMATCH;
    }
    if theta_e_us_input >= tau_e_us / 4 {
        warning_flags |= INNER_WARN_BIT_THETA_TAU;
    }

    Ok(ModelAssembly {
        model: ElectricalModel {
            r_count_milli,
            l_count_micro,
            tau_e_us,
            theta_e_us: theta_e_us_input,
            theta_source: theta_source_wire,
        },
        warning_flags,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum ThetaSource {
    Measured,
    ConfigFallback,
}

pub(crate) fn resolve_theta_e_us_input(
    measured: Option<u32>,
    tau_e_us: u32,
    configured_theta_e_us: u32,
) -> (u32, ThetaSource) {
    match measured {
        Some(theta) if theta > 0 && theta < tau_e_us / 2 => (theta, ThetaSource::Measured),
        _ => (configured_theta_e_us, ThetaSource::ConfigFallback),
    }
}

struct CommissionResistanceBackend<'a, B: CommissioningBackend> {
    backend: &'a mut B,
    current_limit: foci_tmc4671::current_limit::PidTorqueFluxLimit,
}

impl<B: CommissioningBackend> crate::resistance_identification::ResistanceBackend
    for CommissionResistanceBackend<'_, B>
{
    async fn read_register(
        &mut self,
        addr: u8,
    ) -> Result<u32, crate::resistance_identification::ResistanceBackendError> {
        self.backend
            .read_register(Register::new(addr))
            .await
            .map_err(|_| crate::resistance_identification::ResistanceBackendError::Spi)
    }

    async fn write_register(
        &mut self,
        addr: u8,
        value: u32,
    ) -> Result<(), crate::resistance_identification::ResistanceBackendError> {
        self.backend
            .write_register(Register::new(addr), value)
            .await
            .map_err(|_| crate::resistance_identification::ResistanceBackendError::Spi)
    }

    async fn delay_ms(&mut self, ms: u64) {
        let mut remaining_ms = ms;
        while remaining_ms > 0 {
            let chunk_ms = remaining_ms.min(1_000);
            self.backend.delay_us((chunk_ms * 1_000) as u32).await;
            remaining_ms -= chunk_ms;
        }
    }

    fn elapsed_us(&self) -> u64 {
        self.backend.elapsed_us()
    }

    fn set_enable_pin(&mut self, enabled: bool) {
        self.backend.set_enable_pin(enabled);
    }

    fn interlock_tripped(&self) -> bool {
        self.backend.interlock_tripped()
    }

    fn trip_power_stage(&mut self) {
        self.backend.set_enable_pin(false);
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        true
    }

    fn profile(&self) -> crate::resistance_identification::ResistanceProfile {
        let mut profile = self.backend.tmc_config().resistance_profile(
            self.backend
                .tmc_config()
                .resistance_scale_metadata_validated,
        );
        let envelope = crate::tmc_control::current_limit::diagnostic_current_envelope(
            self.current_limit,
            crate::resistance_identification::RESISTANCE_DIAGNOSTIC_CURRENT_FRACTION,
        );
        profile.max_expected_current_count = profile.max_expected_current_count.min(envelope);
        profile
    }

    fn encoder_counts_per_rev(&self) -> u32 {
        self.backend.mechanical_encoder_quadrature_counts_per_rev()
    }

    fn feedback_config(&self) -> crate::config::FeedbackConfig {
        self.backend.feedback_config()
    }

    fn adc_offsets_ready(&self) -> bool {
        self.backend.adc_offsets_ready()
    }

    fn bus_voltage_sense(&self) -> Option<crate::resistance_identification::BusVoltageSense> {
        self.backend
            .tmc_config()
            .bus_voltage_sense(self.backend.adc_vm_offset())
    }
}

fn begin_inductance_restore<B: CommissioningBackend>(
    state: &mut ElectricalIdentificationState,
    backend: &mut B,
    primary_error: Option<CommissionError>,
    evidence: Option<InductanceEvidence>,
    timing: Option<CaptureTimingEvidence>,
) -> StepResult {
    backend.set_enable_pin(false);
    let timing_accepted = timing.is_some()
        && !matches!(
            state,
            ElectricalIdentificationState::InductanceSample { .. }
        );
    *state = ElectricalIdentificationState::InductanceRestore {
        operation: InductanceRestoreOperation::RetainedUqUdExt,
        primary_error,
        evidence,
        timing,
        timing_accepted,
    };
    StepResult::ReadyNow
}

fn inductance_restore_write(
    operation: InductanceRestoreOperation,
) -> (Register, u32, Option<InductanceRestoreOperation>) {
    use InductanceRestoreOperation::*;
    use foci_tmc4671::registers::*;

    match operation {
        RetainedUqUdExt => (UQ_UD_EXT, 0, Some(RetainedOpenloopVelocity)),
        RetainedOpenloopVelocity => (OPENLOOP_VELOCITY_TARGET, 0, Some(RetainedMotionStop)),
        RetainedMotionStop => (
            MODE_RAMP_MODE_MOTION,
            MOTION_STOPPED,
            Some(RetainedPhiSelection),
        ),
        RetainedPhiSelection => (PHI_E_SELECTION, 0, Some(CanonicalMotionStop)),
        CanonicalMotionStop => (
            MODE_RAMP_MODE_MOTION,
            MOTION_STOPPED,
            Some(CanonicalTorqueFluxTarget),
        ),
        CanonicalTorqueFluxTarget => (PID_TORQUE_FLUX_TARGET, 0, Some(CanonicalTorqueFluxOffset)),
        CanonicalTorqueFluxOffset => (PID_TORQUE_FLUX_OFFSET, 0, Some(CanonicalUqUdExt)),
        CanonicalUqUdExt => (UQ_UD_EXT, 0, Some(CanonicalPhiSelection)),
        CanonicalPhiSelection => (PHI_E_SELECTION, 0, Some(CanonicalPwmChop)),
        CanonicalPwmChop => (PWM_SV_CHOP, PWM_CHOP_OFF, Some(CanonicalVelocityOffset)),
        CanonicalVelocityOffset => (PID_VELOCITY_OFFSET, 0, Some(CanonicalVoltageLimit)),
        CanonicalVoltageLimit => (PIDOUT_UQ_UD_LIMITS, 1024, Some(TorqueErrorSelect)),
        TorqueErrorSelect => (
            PID_ERROR_PID_TORQUE_ERROR_SUM.addr,
            PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr as u32,
            Some(TorqueErrorClear),
        ),
        TorqueErrorClear => (
            PID_ERROR_PID_TORQUE_ERROR_SUM.data,
            0,
            Some(FluxErrorSelect),
        ),
        FluxErrorSelect => (
            PID_ERROR_PID_FLUX_ERROR_SUM.addr,
            PID_ERROR_PID_FLUX_ERROR_SUM.sub_addr as u32,
            Some(FluxErrorClear),
        ),
        FluxErrorClear => (
            PID_ERROR_PID_FLUX_ERROR_SUM.data,
            0,
            Some(VelocityErrorSelect),
        ),
        VelocityErrorSelect => (
            PID_ERROR_PID_VELOCITY_ERROR_SUM.addr,
            PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr as u32,
            Some(VelocityErrorClear),
        ),
        VelocityErrorClear => (
            PID_ERROR_PID_VELOCITY_ERROR_SUM.data,
            0,
            Some(PositionErrorSelect),
        ),
        PositionErrorSelect => (
            PID_ERROR_PID_POSITION_ERROR_SUM.addr,
            PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
            Some(PositionErrorClear),
        ),
        PositionErrorClear => (PID_ERROR_PID_POSITION_ERROR_SUM.data, 0, None),
    }
}

fn pack_ud_drive(ud_ext: i16) -> u32 {
    foci_tmc4671::registers::UqUdExt::new()
        .with_ud_ext(ud_ext)
        .with_uq_ext(0)
        .into_bits()
}

fn decode_rotating_frame_current(actual: u32) -> (i16, i16) {
    let id_count = (actual & 0xffff) as u16 as i16;
    let iq_count = (actual >> 16) as u16 as i16;
    (id_count, iq_count)
}

fn inductance_encoder_delta(
    domain: EncoderDomain,
    start: u32,
    end: u32,
) -> Result<EncoderDelta, InductanceReject> {
    domain
        .delta(domain.count(start), domain.count(end))
        .map_err(|error| match error {
            EncoderMathError::InvalidCountsPerRev | EncoderMathError::AmbiguousHalfTurn => {
                InductanceReject::EncoderMotion
            }
        })
}

fn selected_resistance_count_slope(data: &ElectricalIdentificationData) -> Option<i32> {
    Some(data.resistance_result?.selected_r_count_slope_milli)
}

fn realized_frequency_in_range(realized_millihz: u32) -> bool {
    realized_millihz.abs_diff(INDUCTANCE_REALIZED_FREQUENCY_TARGET_MILLIHZ)
        <= INDUCTANCE_REALIZED_FREQUENCY_TOLERANCE_MILLIHZ
}

fn inductance_frequency_rejection_diagnostic(
    realized_frequency_millihz: u32,
    elapsed_us: u32,
    sample_count: u16,
) -> crate::selftest::CommissionDiagnostic {
    crate::selftest::CommissionDiagnostic {
        phase: CommissionPhase::ElectricalIdentification.to_u8(),
        code: DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_FREQUENCY,
        status: 1,
        value0: realized_frequency_millihz,
        value1: elapsed_us,
        value2: u32::from(sample_count),
    }
}

#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
enum InductanceCaptureRejectReason {
    PhiInterval = 1,
    ZeroPhiDelta = 2,
    FrameAccumulator = 3,
    SaliencyAccumulator = 4,
    FirstHalfAccumulator = 5,
    SecondHalfAccumulator = 6,
    ElapsedInterpolation = 7,
}

impl InductanceCaptureRejectReason {
    fn code(self) -> u32 {
        self as u32
    }
}

fn pack_u16_pair(high: u16, low: u16) -> u32 {
    (u32::from(high) << 16) | u32::from(low)
}

fn pack_i16_pair(high: i16, low: i16) -> u32 {
    pack_u16_pair(high as u16, low as u16)
}

fn low_u16(value: u32) -> u16 {
    value.min(u32::from(u16::MAX)) as u16
}

fn inductance_capture_rejection_diagnostic(
    reason: InductanceCaptureRejectReason,
    sample_count: u16,
    value2: u32,
) -> crate::selftest::CommissionDiagnostic {
    crate::selftest::CommissionDiagnostic {
        phase: CommissionPhase::ElectricalIdentification.to_u8(),
        code: DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_CAPTURE,
        status: 1,
        value0: reason.code(),
        value1: u32::from(sample_count),
        value2,
    }
}

fn push_inductance_capture_rejection(
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    reason: InductanceCaptureRejectReason,
    sample_count: u16,
    value2: u32,
) {
    diagnostics.push(inductance_capture_rejection_diagnostic(
        reason,
        sample_count,
        value2,
    ));
}

enum InductanceSamplePoll {
    At,
    Complete {
        elapsed_us: u32,
        timing: CaptureTimingEvidence,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CaptureWait {
    DeadlineReached,
    Shutdown,
}

async fn wait_for_capture_deadline<B: CommissioningBackend>(
    backend: &mut B,
    deadline_us: u64,
) -> CaptureWait {
    loop {
        if backend.is_shutdown_requested() {
            return CaptureWait::Shutdown;
        }
        let now_us = backend.elapsed_us();
        if now_us >= deadline_us {
            return CaptureWait::DeadlineReached;
        }
        let remaining_us = deadline_us - now_us;
        let delay_us = u32::try_from(remaining_us).unwrap_or(u32::MAX);
        backend.delay_us(delay_us).await;
    }
}

async fn poll_inductance_sample<B: CommissioningBackend>(
    backend: &mut B,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    capture: &mut InductanceCaptureState,
    now_us: u64,
) -> Result<InductanceSamplePoll, CommissionError> {
    use foci_tmc4671::registers::{OPENLOOP_PHI, PID_TORQUE_FLUX_ACTUAL};

    if now_us < capture.next_sample_us {
        return Ok(InductanceSamplePoll::At);
    }
    if backend.is_shutdown_requested() {
        return Err(CommissionError::Shutdown);
    }
    if capture.sample_count >= INDUCTANCE_MAX_AC_SAMPLE_COUNT {
        return Err(CommissionError::InductanceCaptureWindow);
    }
    let sample_start = capture
        .timing
        .classify_sample_start(capture.next_sample_us, now_us)
        .map_err(CommissionError::from);
    match sample_start? {
        SampleStart::Sample => {}
        SampleStart::Skip { next_deadline_us } => {
            capture.next_sample_us = next_deadline_us;
            return Ok(InductanceSamplePoll::At);
        }
    }

    let sample_io = timed_read_register_pair(backend, OPENLOOP_PHI, PID_TORQUE_FLUX_ACTUAL).await;
    let (current_phi, actual) = sample_io.result.map_err(|_| CommissionError::Spi)?;
    let current_phi = current_phi as u16;
    let observation_us = backend.elapsed_us();
    let current_elapsed_us = observation_us
        .checked_sub(capture.start_us)
        .and_then(|elapsed| u32::try_from(elapsed).ok())
        .ok_or(CommissionError::ElapsedTimeConversionInvalid)?;
    let expected_phi_delta_counts = match current_elapsed_us
        .checked_sub(capture.previous_elapsed_us)
    {
        Some(elapsed_since_previous_us) => crate::electrical_capture::expected_phi_delta_counts(
            INDUCTANCE_REALIZED_FREQUENCY_TARGET_MILLIHZ,
            elapsed_since_previous_us,
        )
        .ok_or(CommissionError::TimingArithmeticOverflow)?,
        None => {
            crate::electrical_capture::phi_delta_counts_forward(capture.previous_phi, current_phi)
        }
    };
    if sample_io.overflowed {
        capture.timing.mark_source_overflow(sample_io.wall_us);
        return Err(CommissionError::TimingTelemetryOverflow);
    }
    let spi_wall_us = sample_io.wall_us;
    let interval = crate::electrical_capture::phi_locked_ac_interval(
        capture.accumulated_phi_delta_counts,
        INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
        capture.previous_phi,
        current_phi,
        expected_phi_delta_counts,
    )
    .ok_or_else(|| {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::PhiInterval,
            capture.sample_count,
            pack_u16_pair(capture.previous_phi, current_phi),
        );
        CommissionError::InductanceCaptureRejected
    })?;
    if interval.sample_weight_counts == 0 {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::ZeroPhiDelta,
            capture.sample_count,
            pack_u16_pair(capture.previous_phi, current_phi),
        );
        return Err(CommissionError::InductanceCaptureRejected);
    }

    let (id_count, iq_count) = decode_rotating_frame_current(actual);
    let magnitude_sq =
        i64::from(id_count) * i64::from(id_count) + i64::from(iq_count) * i64::from(iq_count);
    let envelope = i64::from(capture.current_envelope);
    if magnitude_sq > envelope * envelope {
        return Err(CommissionError::InductanceCurrentEnvelope);
    }
    let current_sample = pack_i16_pair(id_count, iq_count);
    if capture
        .accumulator
        .push_weighted(id_count, iq_count, interval.sample_weight_counts)
        .is_none()
    {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::FrameAccumulator,
            capture.sample_count,
            current_sample,
        );
        return Err(CommissionError::InductanceCaptureRejected);
    }
    if capture
        .saliency
        .push_weighted_sample(
            current_phi,
            interval.sample_weight_counts,
            id_count,
            iq_count,
        )
        .is_none()
    {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::SaliencyAccumulator,
            capture.sample_count,
            current_sample,
        );
        return Err(CommissionError::InductanceCaptureRejected);
    }

    let half_target = INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS / 2;
    let before = capture.accumulated_phi_delta_counts;
    if before < half_target {
        let first_weight = interval.sample_weight_counts.min(half_target - before);
        if capture
            .first_half
            .push_weighted(id_count, iq_count, first_weight)
            .is_none()
        {
            push_inductance_capture_rejection(
                diagnostics,
                InductanceCaptureRejectReason::FirstHalfAccumulator,
                capture.sample_count,
                current_sample,
            );
            return Err(CommissionError::InductanceCaptureRejected);
        }
        let second_weight = interval.sample_weight_counts - first_weight;
        if second_weight != 0
            && capture
                .second_half
                .push_weighted(id_count, iq_count, second_weight)
                .is_none()
        {
            push_inductance_capture_rejection(
                diagnostics,
                InductanceCaptureRejectReason::SecondHalfAccumulator,
                capture.sample_count,
                current_sample,
            );
            return Err(CommissionError::InductanceCaptureRejected);
        }
    } else if capture
        .second_half
        .push_weighted(id_count, iq_count, interval.sample_weight_counts)
        .is_none()
    {
        push_inductance_capture_rejection(
            diagnostics,
            InductanceCaptureRejectReason::SecondHalfAccumulator,
            capture.sample_count,
            current_sample,
        );
        return Err(CommissionError::InductanceCaptureRejected);
    }

    capture.sample_count = capture.sample_count.saturating_add(1);
    let completed_elapsed_us = if interval.complete {
        Some(
            crate::electrical_capture::interpolate_phi_window_elapsed_us(
                capture.previous_elapsed_us,
                current_elapsed_us,
                interval,
            )
            .ok_or_else(|| {
                push_inductance_capture_rejection(
                    diagnostics,
                    InductanceCaptureRejectReason::ElapsedInterpolation,
                    capture.sample_count,
                    pack_u16_pair(
                        low_u16(capture.previous_elapsed_us),
                        low_u16(current_elapsed_us),
                    ),
                );
                CommissionError::InductanceCaptureRejected
            })?,
        )
    } else {
        None
    };
    let poll_completion_us = backend.elapsed_us();
    let slot = capture
        .timing
        .record_sample(
            capture.next_sample_us,
            now_us,
            observation_us,
            poll_completion_us,
            spi_wall_us,
        )
        .map_err(CommissionError::from)?;
    capture.timing.finish().map_err(CommissionError::from)?;
    capture.next_sample_us = slot.next_deadline_us;
    if let Some(completed_elapsed_us) = completed_elapsed_us {
        capture.accumulated_phi_delta_counts = interval.accumulated_phi_delta_counts;
        let timing = capture.timing.finish().map_err(CommissionError::from)?;
        return Ok(InductanceSamplePoll::Complete {
            elapsed_us: completed_elapsed_us,
            timing,
        });
    }
    capture.previous_elapsed_us = current_elapsed_us;
    capture.previous_phi = current_phi;
    capture.accumulated_phi_delta_counts = interval.accumulated_phi_delta_counts;
    Ok(InductanceSamplePoll::At)
}

pub(crate) async fn drive_deadline_capture<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    backend: &mut B,
) -> Option<StepResult> {
    let InnerPhase::ElectricalIdentification { state, data } = phase else {
        return None;
    };

    match state {
        ElectricalIdentificationState::InductanceSample { capture } => loop {
            if matches!(
                wait_for_capture_deadline(backend, capture.next_sample_us).await,
                CaptureWait::Shutdown
            ) {
                return None;
            }
            let now_us = backend.elapsed_us();
            match poll_inductance_sample(backend, diagnostics, capture, now_us).await {
                Ok(InductanceSamplePoll::At) => {}
                Ok(InductanceSamplePoll::Complete { elapsed_us, timing }) => {
                    *state = ElectricalIdentificationState::InductanceReadStatus {
                        capture: *capture,
                        completed_elapsed_us: elapsed_us,
                        timing,
                    };
                    return Some(StepResult::ReadyNow);
                }
                Err(CommissionError::Shutdown) => return None,
                Err(error) => {
                    let (evidence, overflowed) = capture.timing.snapshot();
                    data.inductance_timing = Some(evidence);
                    data.timing_overflow_mask |=
                        u8::from(overflowed) << COMMISSION_TIMING_METHOD_INDUCTANCE;
                    return Some(begin_inductance_restore(
                        state,
                        backend,
                        Some(error),
                        None,
                        Some(evidence),
                    ));
                }
            }
        },
        _ => None,
    }
}

struct InductanceCaptureFinish<'a> {
    data: &'a ElectricalIdentificationData,
    encoder_domain: EncoderDomain,
    encoder_start: u32,
    encoder_after: u32,
    elapsed_us: u32,
    phi_delta_counts: u32,
    sample_count: u16,
    status_flags_or: u32,
    accumulator: crate::electrical_capture::AcFrameAccumulator,
    first_half: crate::electrical_capture::AcFrameAccumulator,
    second_half: crate::electrical_capture::AcFrameAccumulator,
    saliency_accumulator: crate::electrical_capture::AcSaliencyAccumulator,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InductanceReject {
    CaptureWindow,
    Frequency,
    LowQuadratureSignal,
    MissingResistance,
    Reactance,
    Drift,
    EncoderMotion,
}

impl InductanceReject {
    fn commission_error(self) -> CommissionError {
        match self {
            Self::EncoderMotion => CommissionError::InductanceEncoderMotion,
            Self::CaptureWindow => CommissionError::InductanceCaptureWindow,
            Self::Frequency => CommissionError::InductanceFrequencyOutOfRange,
            Self::LowQuadratureSignal => CommissionError::InductanceLowQuadratureSignal,
            Self::MissingResistance => CommissionError::InductanceMissingResistanceEvidence,
            Self::Reactance => CommissionError::InductanceReactanceInvalid,
            Self::Drift => CommissionError::InductanceDriftInvalid,
        }
    }
}

fn finish_inductance_evidence(
    input: InductanceCaptureFinish<'_>,
) -> Result<InductanceEvidence, InductanceReject> {
    let InductanceCaptureFinish {
        data,
        encoder_domain,
        encoder_start,
        encoder_after,
        elapsed_us,
        phi_delta_counts,
        sample_count,
        status_flags_or,
        accumulator,
        first_half,
        second_half,
        saliency_accumulator,
    } = input;

    if phi_delta_counts != INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS
        || sample_count < INDUCTANCE_MIN_AC_SAMPLE_COUNT
    {
        return Err(InductanceReject::CaptureWindow);
    }

    let realized_frequency =
        crate::electrical_capture::realized_frequency_millihz(phi_delta_counts, elapsed_us)
            .ok_or(InductanceReject::Frequency)?;
    if !realized_frequency_in_range(realized_frequency) {
        return Err(InductanceReject::Frequency);
    }

    let frame = accumulator
        .finish(elapsed_us, realized_frequency, phi_delta_counts)
        .ok_or(InductanceReject::CaptureWindow)?;
    if frame.sample_count < INDUCTANCE_MIN_AC_SAMPLE_COUNT
        || frame.iq_mean_milli_count.abs() < INDUCTANCE_MIN_ABS_IQ_MILLI_COUNT
    {
        return Err(InductanceReject::LowQuadratureSignal);
    }

    let r_count_slope_milli =
        selected_resistance_count_slope(data).ok_or(InductanceReject::MissingResistance)?;
    let reactance = crate::electrical_capture::rotating_frame_reactance(
        frame,
        data.inductance_ud,
        r_count_slope_milli,
    )
    .ok_or(InductanceReject::Reactance)?;

    let half_phi_delta = phi_delta_counts / 2;
    let first_frame = first_half
        .finish(elapsed_us / 2, realized_frequency, half_phi_delta)
        .ok_or(InductanceReject::Drift)?;
    let second_frame = second_half
        .finish(elapsed_us / 2, realized_frequency, half_phi_delta)
        .ok_or(InductanceReject::Drift)?;
    let first_reactance = crate::electrical_capture::rotating_frame_reactance(
        first_frame,
        data.inductance_ud,
        r_count_slope_milli,
    )
    .ok_or(InductanceReject::Drift)?;
    let second_reactance = crate::electrical_capture::rotating_frame_reactance(
        second_frame,
        data.inductance_ud,
        r_count_slope_milli,
    )
    .ok_or(InductanceReject::Drift)?;
    let drift_permille = if first_reactance.x_quad_count_ratio_milli == 0
        && second_reactance.x_quad_count_ratio_milli == 0
    {
        0
    } else {
        Permille::checked_ratio(
            u64::from(
                first_reactance
                    .x_quad_count_ratio_milli
                    .abs_diff(second_reactance.x_quad_count_ratio_milli),
            ),
            u64::from(
                first_reactance
                    .x_quad_count_ratio_milli
                    .max(second_reactance.x_quad_count_ratio_milli),
            ),
        )
        .ok_or(InductanceReject::Drift)?
        .raw()
    };

    let saliency = match saliency_accumulator
        .finish(crate::electrical_capture::SaliencyFinishInput {
            phi_delta_counts,
            frame,
            x_average_count_ratio_milli: reactance.x_quad_count_ratio_milli,
            ud_count: data.inductance_ud,
        })
        .and_then(crate::electrical_capture::extract_saliency_from_second_harmonic)
    {
        Some(evidence) => evidence,
        None => crate::electrical_capture::AcSaliencyEvidence::not_determinable(
            reactance.x_quad_count_ratio_milli,
        ),
    };

    let mut warning_flags = 0u16;
    let iq_rms_permille = Permille::checked_ratio(
        u64::from(frame.iq_rms_milli_count),
        u64::from(frame.iq_mean_milli_count.unsigned_abs()),
    )
    .ok_or(InductanceReject::LowQuadratureSignal)?
    .raw();
    if iq_rms_permille > INDUCTANCE_MAX_IQ_RMS_WARN_PERMILLE {
        warning_flags |= INDUCTANCE_WARN_HIGH_RMS;
    }
    if drift_permille > INDUCTANCE_MAX_DRIFT_WARN_PERMILLE {
        warning_flags |= INDUCTANCE_WARN_DRIFT;
    }
    if reactance.x_mag_vs_quad_permille > INDUCTANCE_MAX_MAG_CROSSCHECK_WARN_PERMILLE {
        warning_flags |= INDUCTANCE_WARN_MAG_CROSSCHECK;
    }
    if status_flags_or != 0 {
        warning_flags |= INDUCTANCE_WARN_STATUS_FLAGS;
    }

    let encoder_delta = inductance_encoder_delta(encoder_domain, encoder_start, encoder_after)?;
    if encoder_delta.absolute_counts() > inductance_max_encoder_delta_counts(encoder_domain) {
        return Err(InductanceReject::EncoderMotion);
    }
    let encoder_delta_counts = encoder_delta.signed_counts();

    Ok(InductanceEvidence {
        run: InductanceRunEvidence {
            source: INDUCTANCE_SOURCE_ROTATING_FIELD,
            status: 0,
            warning_flags,
            ud_count: data.inductance_ud,
            realized_frequency_millihz: realized_frequency,
            elapsed_us,
            openloop_phi_delta_counts: phi_delta_counts,
            sample_count: frame.sample_count,
            encoder_delta_counts,
            status_flags_or,
        },
        frame: InductanceFrameEvidence {
            id_mean_milli_count: frame.id_mean_milli_count,
            iq_mean_milli_count: frame.iq_mean_milli_count,
            id_rms_milli_count: frame.id_rms_milli_count,
            iq_rms_milli_count: frame.iq_rms_milli_count,
            drift_permille,
            zero_id_mean_milli_count: 0,
            zero_iq_mean_milli_count: 0,
        },
        estimate: InductanceEstimateEvidence {
            x_average_count_ratio_milli: reactance.x_quad_count_ratio_milli,
            x_d_count_ratio_milli: saliency.x_d_count_ratio_milli,
            x_q_count_ratio_milli: saliency.x_q_count_ratio_milli,
            saliency_status: saliency.status,
            saliency_permille: saliency.saliency_permille.raw(),
            x_mag_nominal_count_ratio_milli: reactance.x_mag_nominal_count_ratio_milli,
            x_mag_shift_minus_permille: reactance.x_mag_shift_minus_permille,
            x_mag_shift_plus_permille: reactance.x_mag_shift_plus_permille,
            x_mag_vs_quad_permille: reactance.x_mag_vs_quad_permille,
        },
    })
}

fn electrical_timing_summaries(
    data: &ElectricalIdentificationData,
) -> [crate::commissioning::types::CommissionTimingSummary; 3] {
    use crate::commissioning::types::CommissionTimingSummary;

    let evidence = [
        (COMMISSION_TIMING_METHOD_RESISTANCE, data.resistance_timing),
        (COMMISSION_TIMING_METHOD_INDUCTANCE, data.inductance_timing),
    ];
    let mut summaries = [CommissionTimingSummary::NOT_RUN; 3];
    for (method, evidence) in evidence {
        summaries[usize::from(method)] = CommissionTimingSummary {
            status: (data.timing_states >> (method * 2)) & 0x03,
            overflowed: data.timing_overflow_mask & (1 << method) != 0,
            evidence: evidence
                .unwrap_or(crate::commissioning::capture_timing::CaptureTimingEvidence::EMPTY),
        };
    }
    summaries
}

fn set_timing_state(data: &mut ElectricalIdentificationData, method: u8, status: u8) {
    let shift = method * 2;
    data.timing_states = (data.timing_states & !(0x03 << shift)) | ((status & 0x03) << shift);
}

fn push_timing_diagnostics(
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    summaries: [crate::commissioning::types::CommissionTimingSummary; 3],
) {
    for (method, summary) in summaries.iter().copied().enumerate() {
        if summary.status == crate::commissioning::types::COMMISSION_TIMING_NOT_RUN {
            continue;
        }
        let evidence = summary.evidence;
        let code = 34 + (method as u8 * 2);
        diagnostics.push(crate::selftest::CommissionDiagnostic {
            phase: CommissionPhase::ElectricalIdentification.to_u8(),
            code,
            status: u8::from(
                summary.status == crate::commissioning::types::COMMISSION_TIMING_REJECTED,
            ),
            value0: u32::from(evidence.requested_period_us)
                | (u32::from(evidence.valid_samples) << 16),
            value1: u32::from(evidence.missed_samples)
                | (u32::from(evidence.max_consecutive_misses) << 16)
                | (u32::from(summary.status) << 24)
                | (u32::from(summary.overflowed) << 31),
            value2: evidence.max_lateness_us,
        });
        diagnostics.push(crate::selftest::CommissionDiagnostic {
            phase: CommissionPhase::ElectricalIdentification.to_u8(),
            code: code + 1,
            status: 0,
            value0: evidence.max_interval_us,
            value1: evidence.max_poll_wall_us,
            value2: evidence.max_spi_wall_us,
        });
    }
}

fn push_resistance_current_diagnostic(
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    current: crate::resistance_identification::ResistanceCurrentEvidence,
) {
    diagnostics.push(crate::selftest::CommissionDiagnostic {
        phase: CommissionPhase::ElectricalIdentification.to_u8(),
        code: 40,
        status: u8::from(current.power_stage_tripped()),
        value0: u32::from(current.peak_abs_current_count()),
        value1: u32::from(current.max_abs_steady_mean_current_count()),
        value2: u32::from(current.current_ceiling_count()),
    });
}

pub(crate) fn push_active_electrical_diagnostics(
    phase: &InnerPhase,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
) {
    let InnerPhase::ElectricalIdentification { state, data } = phase else {
        return;
    };
    let mut summaries = electrical_timing_summaries(data);
    let mut active_resistance_current = None;
    match state {
        ElectricalIdentificationState::ResistanceIdentify { runner } => {
            let timing = runner.timing_evidence_with_active();
            summaries[usize::from(COMMISSION_TIMING_METHOD_RESISTANCE)] =
                crate::commissioning::types::CommissionTimingSummary::rejected(
                    timing.capture_evidence(),
                    timing.overflowed,
                );
            active_resistance_current = Some(runner.current_evidence());
        }
        ElectricalIdentificationState::InductanceSample { capture } => {
            let (evidence, overflowed) = capture.timing.snapshot();
            summaries[usize::from(COMMISSION_TIMING_METHOD_INDUCTANCE)] =
                crate::commissioning::types::CommissionTimingSummary::rejected(
                    evidence, overflowed,
                );
        }
        ElectricalIdentificationState::InductanceReadStatus { timing, .. }
        | ElectricalIdentificationState::InductanceReadEncoder { timing, .. }
        | ElectricalIdentificationState::InductanceEvaluate { timing, .. } => {
            summaries[usize::from(COMMISSION_TIMING_METHOD_INDUCTANCE)] =
                crate::commissioning::types::CommissionTimingSummary::rejected(
                    *timing,
                    data.timing_overflow_mask & (1 << COMMISSION_TIMING_METHOD_INDUCTANCE) != 0,
                );
        }
        ElectricalIdentificationState::InductanceRestore {
            timing: Some(timing),
            ..
        } => {
            summaries[usize::from(COMMISSION_TIMING_METHOD_INDUCTANCE)] =
                crate::commissioning::types::CommissionTimingSummary::rejected(
                    *timing,
                    data.timing_overflow_mask & (1 << COMMISSION_TIMING_METHOD_INDUCTANCE) != 0,
                );
        }
        _ => {}
    }
    push_timing_diagnostics(diagnostics, summaries);
    if let Some(current) = active_resistance_current {
        push_resistance_current_diagnostic(diagnostics, current);
    }
}

fn fail_electrical_id(
    phase: &mut InnerPhase,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    error: CommissionError,
) -> StepResult {
    if let InnerPhase::ElectricalIdentification { data, .. } = &*phase {
        push_timing_diagnostics(diagnostics, electrical_timing_summaries(data));
    }
    super::fail_inner(phase, error)
}

pub(crate) async fn step_electrical_identification<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    diagnostics: &mut crate::selftest::CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (electrical_identification_state, data) = match phase {
        InnerPhase::ElectricalIdentification { state, data } => (state, data),
        _ => return StepResult::ReadyNow,
    };

    match electrical_identification_state {
        ElectricalIdentificationState::ResistanceStart => {
            data.encoder_domain =
                match EncoderDomain::new(backend.mechanical_encoder_quadrature_counts_per_rev()) {
                    Ok(domain) => Some(domain),
                    Err(EncoderMathError::InvalidCountsPerRev) => {
                        return fail_electrical_id(
                            phase,
                            diagnostics,
                            CommissionError::EncoderFault,
                        );
                    }
                    Err(EncoderMathError::AmbiguousHalfTurn) => unreachable!(),
                };
            let config = backend.tmc_config();
            let profile = config.resistance_profile(config.resistance_scale_metadata_validated);
            let runner = match crate::resistance_identification::ResistanceRunner::new(
                crate::resistance_identification::ResistanceRunMode::CommissioningManagedEnable,
                profile,
                backend.elapsed_us(),
            ) {
                Ok(runner) => runner,
                Err(error) => {
                    defmt::warn!("resistance id failed: {}", error);
                    return fail_electrical_id(phase, diagnostics, CommissionError::from(error));
                }
            };
            set_timing_state(
                data,
                COMMISSION_TIMING_METHOD_RESISTANCE,
                crate::commissioning::types::COMMISSION_TIMING_REJECTED,
            );
            *electrical_identification_state =
                ElectricalIdentificationState::ResistanceIdentify { runner };
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::ResistanceIdentify { runner } => {
            let now_us = backend.elapsed_us();
            let poll = runner
                .poll(
                    &mut CommissionResistanceBackend {
                        backend,
                        current_limit: data.current_limit,
                    },
                    now_us,
                )
                .await;
            let resistance_timing = runner.timing_evidence();
            data.resistance_timing = Some(resistance_timing.capture_evidence());
            data.timing_overflow_mask = (data.timing_overflow_mask
                & !(1 << COMMISSION_TIMING_METHOD_RESISTANCE))
                | (u8::from(resistance_timing.overflowed) << COMMISSION_TIMING_METHOD_RESISTANCE);

            match poll {
                crate::resistance_identification::ResistancePoll::ReadyNow => StepResult::ReadyNow,
                crate::resistance_identification::ResistancePoll::At(deadline_us) => {
                    StepResult::At(deadline_us)
                }
                crate::resistance_identification::ResistancePoll::Complete(Ok(resistance)) => {
                    data.resistance_result = Some(resistance);
                    set_timing_state(
                        data,
                        COMMISSION_TIMING_METHOD_RESISTANCE,
                        crate::commissioning::types::COMMISSION_TIMING_ACCEPTED,
                    );
                    *electrical_identification_state =
                        ElectricalIdentificationState::InductanceSetup;
                    StepResult::ReadyNow
                }
                crate::resistance_identification::ResistancePoll::Complete(Err(error)) => {
                    defmt::warn!("resistance id failed: {}", error);
                    push_resistance_current_diagnostic(diagnostics, runner.current_evidence());
                    fail_electrical_id(phase, diagnostics, CommissionError::from(error))
                }
            }
        }

        ElectricalIdentificationState::InductanceSetup => {
            if data.encoder_domain.is_none() {
                return fail_electrical_id(phase, diagnostics, CommissionError::EncoderFault);
            }
            *electrical_identification_state =
                ElectricalIdentificationState::InductanceSetupOperation { operation: 0 };
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::InductanceSetupOperation { operation } => {
            let pidout_limit = data
                .voltage
                .min(u32::from(data.inductance_ud.unsigned_abs()));
            let result: Result<(), CommissionError> = match *operation {
                0 => backend
                    .write_register(PIDOUT_UQ_UD_LIMITS, pidout_limit)
                    .await
                    .map_err(|_| CommissionError::Spi),
                1 => backend
                    .write_register(UQ_UD_EXT, 0)
                    .await
                    .map_err(|_| CommissionError::Spi),
                2 => backend
                    .write_register(OPENLOOP_VELOCITY_TARGET, 0)
                    .await
                    .map_err(|_| CommissionError::Spi),
                3 => backend
                    .write_register(OPENLOOP_MODE, 0)
                    .await
                    .map_err(|_| CommissionError::Spi),
                4 => backend
                    .write_register(OPENLOOP_ACCELERATION, INDUCTANCE_OPENLOOP_ACCELERATION)
                    .await
                    .map_err(|_| CommissionError::Spi),
                5 => backend
                    .write_register(OPENLOOP_VELOCITY_TARGET, data.inductance_openloop_rpm)
                    .await
                    .map_err(|_| CommissionError::Spi),
                6 => backend
                    .write_register(PHI_E_SELECTION, PHI_E_SELECTION_OPENLOOP)
                    .await
                    .map_err(|_| CommissionError::Spi),
                7 => backend
                    .write_register(MODE_RAMP_MODE_MOTION, MOTION_UQ_UD_EXT)
                    .await
                    .map_err(|_| CommissionError::Spi),
                8 => backend
                    .write_register_verified(PWM_SV_CHOP, PWM_CHOP_CENTERED_FOC)
                    .await
                    .map(|_| ())
                    .map_err(|_| CommissionError::Spi),
                _ => crate::tmc_control::interlock::clear_status_flags_via_backend(backend).await,
            };
            if let Err(error) = result {
                return begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    Some(error),
                    None,
                    None,
                );
            }
            if *operation < 9 {
                *operation += 1;
                return StepResult::ReadyNow;
            }
            let Some(deadline_us) =
                backend
                    .elapsed_us()
                    .checked_add(inductance_openloop_ramp_timeout_us(
                        data.inductance_openloop_rpm,
                    ))
            else {
                return begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    Some(CommissionError::InvalidSchedule),
                    None,
                    None,
                );
            };
            if backend.interlock_tripped() {
                return begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    Some(CommissionError::FaultInterlock),
                    None,
                    None,
                );
            }
            backend.set_enable_pin(true);
            *electrical_identification_state =
                ElectricalIdentificationState::InductanceWaitVelocity { deadline_us };
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::InductanceWaitVelocity { deadline_us } => {
            let velocity = match backend.read_register(OPENLOOP_VELOCITY_ACTUAL).await {
                Ok(velocity) => velocity,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_identification_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        None,
                    );
                }
            };
            if velocity != data.inductance_openloop_rpm {
                let now_us = backend.elapsed_us();
                if now_us < *deadline_us {
                    return StepResult::At(
                        now_us
                            .saturating_add(INDUCTANCE_OPENLOOP_RAMP_POLL_US)
                            .min(*deadline_us),
                    );
                }
                return begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    Some(CommissionError::InductanceVelocityTimeout),
                    None,
                    None,
                );
            }
            *electrical_identification_state =
                ElectricalIdentificationState::InductanceReadEncoderStart;
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::InductanceReadEncoderStart => {
            let encoder_start = match backend.read_register(ABN_DECODER_COUNT).await {
                Ok(count) => count,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_identification_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        None,
                    );
                }
            };
            *electrical_identification_state =
                ElectricalIdentificationState::InductanceApplyDrive { encoder_start };
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::InductanceApplyDrive { encoder_start } => {
            if backend
                .write_register(UQ_UD_EXT, pack_ud_drive(data.inductance_ud))
                .await
                .is_err()
            {
                return begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    Some(CommissionError::Spi),
                    None,
                    None,
                );
            }
            let Some(deadline_us) = backend.elapsed_us().checked_add(INDUCTANCE_AC_SETTLE_US)
            else {
                return begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    Some(CommissionError::InvalidSchedule),
                    None,
                    None,
                );
            };
            let encoder_domain = data
                .encoder_domain
                .expect("validated before inductance setup");
            *electrical_identification_state =
                ElectricalIdentificationState::InductanceDriveSettle {
                    deadline_us,
                    encoder_start: *encoder_start,
                    encoder_domain,
                };
            StepResult::At(deadline_us)
        }

        ElectricalIdentificationState::InductanceDriveSettle {
            deadline_us,
            encoder_start,
            encoder_domain,
        } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            *electrical_identification_state =
                ElectricalIdentificationState::InductanceReadPreStatus {
                    encoder_start: *encoder_start,
                    encoder_domain: *encoder_domain,
                };
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::InductanceReadPreStatus {
            encoder_start,
            encoder_domain,
        } => {
            let status_flags_or = match backend.read_register(STATUS_FLAGS).await {
                Ok(status) => status,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_identification_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        None,
                    );
                }
            };
            let entry_mask = crate::autotune::current_loop_entry_actionable_status_mask(
                backend.feedback_config(),
            );
            if crate::autotune::is_actionable_status(status_flags_or, entry_mask) {
                defmt::warn!(
                    "Inductance pre-status rejected: status_flags_or=0x{:08x} entry_mask=0x{:08x}",
                    status_flags_or,
                    entry_mask,
                );
                diagnostics.push(crate::selftest::CommissionDiagnostic {
                    phase: CommissionPhase::ElectricalIdentification.to_u8(),
                    code: DIAG_ELECTRICAL_ID_INDUCTANCE_STATUS_REJECT,
                    status: 1,
                    value0: status_flags_or,
                    value1: entry_mask,
                    value2: 0,
                });
                return begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    Some(CommissionError::InductanceCaptureRejected),
                    None,
                    None,
                );
            }
            *electrical_identification_state =
                ElectricalIdentificationState::InductanceReadStartPhi {
                    encoder_start: *encoder_start,
                    encoder_domain: *encoder_domain,
                    status_flags_or,
                };
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::InductanceReadStartPhi {
            encoder_start,
            encoder_domain,
            status_flags_or,
        } => {
            let previous_phi = match backend.read_register(OPENLOOP_PHI).await {
                Ok(phi) => phi as u16,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_identification_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        None,
                    );
                }
            };
            let start_us = backend.elapsed_us();
            let Some(next_sample_us) =
                start_us.checked_add(u64::from(INDUCTANCE_AC_SAMPLE_PERIOD_US))
            else {
                return begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    Some(CommissionError::InvalidSchedule),
                    None,
                    None,
                );
            };
            set_timing_state(
                data,
                COMMISSION_TIMING_METHOD_INDUCTANCE,
                crate::commissioning::types::COMMISSION_TIMING_REJECTED,
            );
            *electrical_identification_state = ElectricalIdentificationState::InductanceSample {
                capture: InductanceCaptureState {
                    encoder_domain: *encoder_domain,
                    encoder_start: *encoder_start,
                    start_us,
                    previous_elapsed_us: 0,
                    previous_phi,
                    accumulated_phi_delta_counts: 0,
                    accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                    first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                    saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                    sample_count: 0,
                    status_flags_or: *status_flags_or,
                    next_sample_us,
                    timing: CaptureTiming::new(
                        INDUCTANCE_AC_SAMPLE_PERIOD_US,
                        TimingEnvelope::INDUCTANCE,
                    ),
                    current_envelope:
                        crate::tmc_control::current_limit::diagnostic_current_envelope(
                            data.current_limit,
                            INDUCTANCE_DIAGNOSTIC_CURRENT_FRACTION,
                        ),
                },
            };
            StepResult::At(next_sample_us)
        }

        ElectricalIdentificationState::InductanceSample { capture } => {
            let now_us = backend.elapsed_us();
            match poll_inductance_sample(backend, diagnostics, capture, now_us).await {
                Ok(InductanceSamplePoll::At) => StepResult::At(capture.next_sample_us),
                Ok(InductanceSamplePoll::Complete { elapsed_us, timing }) => {
                    *electrical_identification_state =
                        ElectricalIdentificationState::InductanceReadStatus {
                            capture: *capture,
                            completed_elapsed_us: elapsed_us,
                            timing,
                        };
                    StepResult::ReadyNow
                }
                Err(error) => {
                    let (evidence, overflowed) = capture.timing.snapshot();
                    data.inductance_timing = Some(evidence);
                    data.timing_overflow_mask |=
                        u8::from(overflowed) << COMMISSION_TIMING_METHOD_INDUCTANCE;
                    begin_inductance_restore(
                        electrical_identification_state,
                        backend,
                        Some(error),
                        None,
                        Some(evidence),
                    )
                }
            }
        }

        ElectricalIdentificationState::InductanceReadStatus {
            capture,
            completed_elapsed_us,
            timing,
        } => {
            let mut next_capture = *capture;
            let completed_elapsed_us = *completed_elapsed_us;
            let timing = *timing;
            let status = match backend.read_register(STATUS_FLAGS).await {
                Ok(status) => status,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_identification_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        Some(timing),
                    );
                }
            };
            next_capture.status_flags_or |= status;
            let entry_mask = crate::autotune::current_loop_entry_actionable_status_mask(
                backend.feedback_config(),
            );
            if crate::autotune::is_actionable_status(status, entry_mask) {
                defmt::warn!(
                    "Inductance capture status rejected: status=0x{:08x} entry_mask=0x{:08x}",
                    status,
                    entry_mask,
                );
                diagnostics.push(crate::selftest::CommissionDiagnostic {
                    phase: CommissionPhase::ElectricalIdentification.to_u8(),
                    code: DIAG_ELECTRICAL_ID_INDUCTANCE_STATUS_REJECT,
                    status: 1,
                    value0: status,
                    value1: entry_mask,
                    value2: u32::from(capture.sample_count),
                });
                return begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    Some(CommissionError::InductanceCaptureRejected),
                    None,
                    Some(timing),
                );
            }
            *electrical_identification_state =
                ElectricalIdentificationState::InductanceReadEncoder {
                    capture: next_capture,
                    completed_elapsed_us,
                    timing,
                };
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::InductanceReadEncoder {
            capture,
            completed_elapsed_us,
            timing,
        } => {
            let capture = *capture;
            let completed_elapsed_us = *completed_elapsed_us;
            let timing = *timing;
            let encoder_after = match backend.read_register(ABN_DECODER_COUNT).await {
                Ok(count) => count,
                Err(_) => {
                    return begin_inductance_restore(
                        electrical_identification_state,
                        backend,
                        Some(CommissionError::Spi),
                        None,
                        Some(timing),
                    );
                }
            };
            *electrical_identification_state = ElectricalIdentificationState::InductanceEvaluate {
                capture,
                encoder_after,
                completed_elapsed_us,
                timing,
            };
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::InductanceEvaluate {
            capture,
            encoder_after,
            completed_elapsed_us,
            timing,
        } => {
            let capture = *capture;
            let encoder_after = *encoder_after;
            let completed_elapsed_us = *completed_elapsed_us;
            let timing = *timing;
            let evidence = finish_inductance_evidence(InductanceCaptureFinish {
                data,
                encoder_domain: capture.encoder_domain,
                encoder_start: capture.encoder_start,
                encoder_after,
                elapsed_us: completed_elapsed_us,
                phi_delta_counts: capture.accumulated_phi_delta_counts,
                sample_count: capture.sample_count,
                status_flags_or: capture.status_flags_or,
                accumulator: capture.accumulator,
                first_half: capture.first_half,
                second_half: capture.second_half,
                saliency_accumulator: capture.saliency,
            });
            match evidence {
                Ok(evidence) => begin_inductance_restore(
                    electrical_identification_state,
                    backend,
                    None,
                    Some(evidence),
                    Some(timing),
                ),
                Err(error) => {
                    if error == InductanceReject::Frequency {
                        let frequency = crate::electrical_capture::realized_frequency_millihz(
                            capture.accumulated_phi_delta_counts,
                            completed_elapsed_us,
                        )
                        .unwrap_or(0);
                        diagnostics.push(inductance_frequency_rejection_diagnostic(
                            frequency,
                            completed_elapsed_us,
                            capture.sample_count,
                        ));
                    }
                    begin_inductance_restore(
                        electrical_identification_state,
                        backend,
                        Some(error.commission_error()),
                        None,
                        Some(timing),
                    )
                }
            }
        }

        ElectricalIdentificationState::InductanceRestore {
            operation,
            primary_error,
            evidence,
            timing,
            timing_accepted,
        } => {
            let (register, value, next) = inductance_restore_write(*operation);
            if backend
                .write_register_verified(register, value)
                .await
                .is_err()
            {
                *primary_error = Some(CommissionError::Spi);
                *timing_accepted = false;
            }
            if let Some(next) = next {
                *operation = next;
                return StepResult::ReadyNow;
            }
            data.inductance_timing = *timing;
            if *timing_accepted {
                set_timing_state(
                    data,
                    COMMISSION_TIMING_METHOD_INDUCTANCE,
                    crate::commissioning::types::COMMISSION_TIMING_ACCEPTED,
                );
            }
            if let Some(error) = *primary_error {
                defmt::warn!(
                    "Inductance identification failed: primary_error={} timing_accepted={}",
                    error,
                    *timing_accepted
                );
                return fail_electrical_id(phase, diagnostics, error);
            }
            let Some(evidence) = *evidence else {
                defmt::warn!(
                    "Inductance identification rejected: no evidence assembled, timing_accepted={}",
                    *timing_accepted
                );
                data.inductance_timing = *timing;
                return fail_electrical_id(
                    phase,
                    diagnostics,
                    CommissionError::InductanceCaptureRejected,
                );
            };
            data.inductance = Some(evidence);
            let Some(deadline_us) = backend.elapsed_us().checked_add(ELECTRICAL_COOLDOWN_US) else {
                return fail_electrical_id(phase, diagnostics, CommissionError::InvalidSchedule);
            };
            *electrical_identification_state =
                ElectricalIdentificationState::InductanceDisable { deadline_us };
            StepResult::At(deadline_us)
        }

        ElectricalIdentificationState::InductanceDisable { deadline_us } => {
            if backend.elapsed_us() < *deadline_us {
                return StepResult::At(*deadline_us);
            }
            *electrical_identification_state = ElectricalIdentificationState::Validate;
            StepResult::ReadyNow
        }

        ElectricalIdentificationState::Validate => {
            let assembly = match assemble_electrical_model(
                data,
                backend.tmc_config().electrical_model_theta_e_us,
            ) {
                Ok(assembly) => assembly,
                Err(err) => {
                    log_model_assembly_failure(err);
                    record_electrical_id_measurement_diagnostics(data, diagnostics);
                    diagnostics.push(diagnostic_for_model_assembly_failure(err));
                    return fail_electrical_id(phase, diagnostics, CommissionError::from(err));
                }
            };
            let timing = electrical_timing_summaries(data);
            push_timing_diagnostics(diagnostics, timing);

            match sequence {
                InnerSequence::Selftest => {
                    selftest_results.set(
                        crate::selftest::SelftestStage::RModelEvidence,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: assembly.model.r_count_milli,
                        },
                    );
                    selftest_results.set(
                        crate::selftest::SelftestStage::LModelEvidence,
                        crate::selftest::SelftestStageValue {
                            status: 0,
                            value: assembly.model.l_count_micro,
                        },
                    );
                    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
                    StepResult::SelftestDone
                }
                InnerSequence::Commission => {
                    let resistance_result = data
                        .resistance_result
                        .expect("resistance result populated before Validate");
                    *phase = InnerPhase::CurrentTune {
                        model: assembly.model,
                        inner_warning_flags: assembly.warning_flags,
                        resistance_result,
                        inductance: data
                            .inductance
                            .unwrap_or(crate::commissioning::types::InductanceEvidence::EMPTY),
                        timing,
                    };
                    StepResult::PhaseChanged(CommissionPhase::CurrentTune)
                }
                InnerSequence::Calibrate => {
                    unreachable!()
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn inductance_openloop_targets_one_kilohertz_electrical() {
        assert_eq!(
            inductance_openloop_rpm(50),
            INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM
        );
        assert_eq!(inductance_openloop_rpm(3), 20_000);
    }

    #[test]
    fn inductance_openloop_ramp_timeout_scales_with_the_target() {
        assert_eq!(
            inductance_openloop_ramp_timeout_us(INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM),
            INDUCTANCE_OPENLOOP_RAMP_TIMEOUT_US
        );
        assert_eq!(inductance_openloop_ramp_timeout_us(20_000), 28_610);
    }

    use super::*;
    use crate::autotune::Permille;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::backend::CommissionBoardConfig;
    use crate::commissioning::inner::ElectricalIdentificationData;
    use crate::commissioning::types::{INDUCTANCE_WARN_STATUS_FLAGS, StepResult};
    use crate::resistance_identification::{
        ElectricalAxis, ResistanceAxisResult, ResistanceRunResult, ResistanceWarningFlags,
        SignedAnchorEvidence,
    };
    use foci_tmc4671::registers::{
        MODE_RAMP_MODE_MOTION, OPENLOOP_PHI, OPENLOOP_VELOCITY_ACTUAL, OPENLOOP_VELOCITY_TARGET,
        PHI_E_SELECTION, PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM,
        PID_ERROR_PID_TORQUE_ERROR_SUM, PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_TORQUE_FLUX_ACTUAL,
        PID_TORQUE_FLUX_OFFSET, PID_TORQUE_FLUX_TARGET, PID_VELOCITY_OFFSET, PIDOUT_UQ_UD_LIMITS,
        PWM_SV_CHOP, STATUS_FLAGS, UQ_UD_EXT,
    };

    const ELECTRICAL_ID_MAX_REGISTER_OPS_PER_ARM: usize = 2;

    const TEST_THETA_E_US: u32 = 160;
    const NOMINAL_INDUCTANCE_CAPTURE_US: u32 =
        INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS / 65_536 * 1_000;
    const IPARK_CIRCULAR_LIMIT_BIT: u32 = 1 << 17;

    #[test]
    fn electrical_capture_periods_preserve_required_windows() {
        assert_eq!(INDUCTANCE_AC_SAMPLE_PERIOD_US, 200);
        assert_eq!(INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS, 20 * 65_536);
        assert_eq!(scheduled_samples_for_fixed_window(1_050_000), 95);
        assert_eq!(
            scheduled_samples_for_fixed_window(1_050_000)
                - u32::from(INDUCTANCE_MIN_AC_SAMPLE_COUNT),
            5
        );
        assert!(
            scheduled_samples_for_fixed_window(1_050_000)
                >= u32::from(INDUCTANCE_MIN_AC_SAMPLE_COUNT)
        );
    }

    fn axis_result(axis: ElectricalAxis, slope_milli: i32) -> ResistanceAxisResult {
        ResistanceAxisResult {
            axis,
            r_count_slope_milli: slope_milli,
            slope_num: i64::from(slope_milli),
            slope_den: 1000,
            intercept_count: 0,
            rmse_permille: Permille::ZERO,
            selected_mask: 0b1111_1000,
            excluded_point_mask: 0b0000_0111,
            selected_count: 5,
            signed_anchor: Some(SignedAnchorEvidence {
                positive_current_count: 900,
                negative_current_count: -900,
                signed_count_slope_milli: slope_milli,
                asymmetry_permille: Permille::ZERO,
                repeated_positive_current_count: 900,
                drift_permille: Permille::ZERO,
            }),
            warning_flags: ResistanceWarningFlags::NONE,
        }
    }

    fn resistance_result(slope0_milli: i32, slope1_milli: i32) -> ResistanceRunResult {
        ResistanceRunResult {
            selected_r_count_slope_milli: (slope0_milli + slope1_milli) / 2,
            baseline_flags: 0,
            status_flags_or: 0,
            new_flags: 0,
            axis0: axis_result(ElectricalAxis::PHASE0, slope0_milli),
            axis1: axis_result(ElectricalAxis::PHASE90, slope1_milli),
            warning_flags: ResistanceWarningFlags::NONE,
            current: crate::resistance_identification::ResistanceCurrentEvidence::new(1_600),
        }
    }

    fn inductance_evidence_fixture(
        x_average_count_ratio_milli: u32,
        realized_frequency_millihz: u32,
    ) -> InductanceEvidence {
        InductanceEvidence {
            run: InductanceRunEvidence {
                source: INDUCTANCE_SOURCE_ROTATING_FIELD,
                status: 0,
                warning_flags: 0,
                ud_count: INDUCTANCE_AC_UD_EXT,
                realized_frequency_millihz,
                elapsed_us: 8_000,
                openloop_phi_delta_counts: INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
                sample_count: 104,
                encoder_delta_counts: 0,
                status_flags_or: 0,
            },
            frame: InductanceFrameEvidence {
                id_mean_milli_count: 20_000,
                iq_mean_milli_count: -84_000,
                id_rms_milli_count: 5_000,
                iq_rms_milli_count: 21_000,
                drift_permille: 40,
                zero_id_mean_milli_count: 0,
                zero_iq_mean_milli_count: 0,
            },
            estimate: InductanceEstimateEvidence {
                x_average_count_ratio_milli,
                x_d_count_ratio_milli: x_average_count_ratio_milli + 600,
                x_q_count_ratio_milli: x_average_count_ratio_milli.saturating_sub(600),
                saliency_status: crate::electrical_capture::AC_SALIENCY_DETERMINABLE,
                saliency_permille: 140,
                x_mag_nominal_count_ratio_milli: x_average_count_ratio_milli + 180,
                x_mag_shift_minus_permille: 4,
                x_mag_shift_plus_permille: 4,
                x_mag_vs_quad_permille: 20,
            },
        }
    }

    fn representative_data() -> ElectricalIdentificationData {
        let mut data = ElectricalIdentificationData::new(
            512,
            crate::commission_dispatch::test_support::current_limit(2816),
        );
        data.resistance_result = Some(resistance_result(1706, 1706));
        data.inductance = Some(inductance_evidence_fixture(8_600, 1_000_000));
        data.encoder_domain = Some(EncoderDomain::new(4000).unwrap());
        data
    }

    fn resistance_result_fixture() -> ResistanceRunResult {
        resistance_result(1706, 1706)
    }

    fn inductance_capture_state(
        previous_elapsed_us: u32,
        previous_phi: u16,
        accumulated_phi_delta_counts: u32,
        sample_count: u16,
    ) -> InductanceCaptureState {
        InductanceCaptureState {
            encoder_domain: EncoderDomain::new(4000).unwrap(),
            encoder_start: 0,
            start_us: 0,
            previous_elapsed_us,
            previous_phi,
            accumulated_phi_delta_counts,
            accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
            first_half: crate::electrical_capture::AcFrameAccumulator::new(),
            second_half: crate::electrical_capture::AcFrameAccumulator::new(),
            saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
            sample_count,
            status_flags_or: 0,
            next_sample_us: 0,
            timing: CaptureTiming::new(INDUCTANCE_AC_SAMPLE_PERIOD_US, TimingEnvelope::INDUCTANCE),
            current_envelope: 563,
        }
    }

    fn fixed_window_phi(index: u32, sample_count: u32) -> u16 {
        ((index * INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS / sample_count) & 0xffff) as u16
    }

    fn fixed_window_phi_weight(index: u32, sample_count: u32) -> u32 {
        let start = index * INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS / sample_count;
        let end = (index + 1) * INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS / sample_count;
        end - start
    }

    #[test]
    fn electrical_id_enters_rotating_inductance_after_resistance() {
        let mut data = ElectricalIdentificationData::new(
            768,
            crate::commission_dispatch::test_support::current_limit(2816),
        );
        data.resistance_result = Some(resistance_result_fixture());

        let state = ElectricalIdentificationState::InductanceSetup;

        assert!(data.resistance_result.is_some());
        assert!(matches!(
            state,
            ElectricalIdentificationState::InductanceSetup
        ));
    }

    #[test]
    fn model_assembly_uses_full_width_resistance_math() {
        let data = representative_data();

        let assembly = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap();

        assert_eq!(assembly.model.r_count_milli, 1706);
        assert!(
            (795..=805).contains(&assembly.model.tau_e_us),
            "tau should use R=1.706, not truncated: {}",
            assembly.model.tau_e_us
        );
        assert!(
            (1360..=1375).contains(&assembly.model.l_count_micro),
            "l_count_micro should preserve the count-space L scale: {}",
            assembly.model.l_count_micro
        );
    }

    #[test]
    fn model_assembly_uses_realized_frequency_rotating_inductance() {
        let mut data = representative_data();
        data.inductance = Some(inductance_evidence_fixture(8_600, 900_000));

        let assembly =
            assemble_electrical_model(&data, TEST_THETA_E_US).expect("model should assemble");

        let expected_l_count_micro = l_count_micro_from_reactance(8_600, 900_000).unwrap();
        assert_eq!(assembly.model.l_count_micro, expected_l_count_micro);
        assert_eq!(
            assembly.model.tau_e_us,
            (u64::from(expected_l_count_micro) * u64::from(ELECTRICAL_MODEL_RESISTANCE_SCALE)
                / 1706) as u32
        );
    }

    #[test]
    fn model_assembly_uses_configured_theta() {
        let mut data = ElectricalIdentificationData::new(
            512,
            crate::commission_dispatch::test_support::current_limit(2816),
        );
        data.resistance_result = Some(resistance_result(1706, 1706));
        data.inductance = Some(inductance_evidence_fixture(9_500, 1_000_000));

        let assembly = assemble_electrical_model(&data, 320).unwrap();

        assert_eq!(assembly.model.theta_e_us, 320);
    }

    #[test]
    fn validate_transitions_to_current_tune_with_model() {
        pollster::block_on(async {
            let mut data = representative_data();
            data.timing_states = crate::commissioning::types::COMMISSION_TIMING_ACCEPTED
                | (crate::commissioning::types::COMMISSION_TIMING_ACCEPTED << 2);
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::Validate,
                data,
            };
            let mut backend = MockBackend::new();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::PhaseChanged(CommissionPhase::CurrentTune)
            ));
            let InnerPhase::CurrentTune {
                model,
                inner_warning_flags,
                resistance_result,
                inductance: _,
                timing,
            } = phase
            else {
                panic!("expected CurrentTune, got {phase:?}");
            };
            assert_eq!(
                resistance_result.selected_r_count_slope_milli, 1706,
                "resistance result from Phase 5 should carry forward unchanged"
            );
            assert!(
                (795..=805).contains(&model.tau_e_us),
                "tau should be reported from the rotating-field L/R model: {}",
                model.tau_e_us
            );
            assert_eq!(inner_warning_flags & (1 << 2), 0);
            assert_eq!(timing.len(), 3);
            assert_eq!(
                diagnostics
                    .iter()
                    .map(|event| event.code)
                    .collect::<std::vec::Vec<_>>(),
                std::vec![34, 35, 36, 37],
            );
        });
    }

    #[test]
    fn inductance_disable_enters_validate_when_the_deadline_passes() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceDisable { deadline_us: 1_000 },
                data: ElectricalIdentificationData::new(512, current_limit(2816)),
            };
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert!(
                backend.spi_log.is_empty(),
                "no SPI traffic on the transition"
            );
            let InnerPhase::ElectricalIdentification { state, .. } = &phase else {
                unreachable!()
            };
            assert!(
                matches!(state, ElectricalIdentificationState::Validate),
                "{state:?}"
            );
        });
    }

    #[test]
    fn timing_summaries_keep_the_delay_slot_reserved() {
        use crate::commissioning::types::{
            COMMISSION_TIMING_ACCEPTED, COMMISSION_TIMING_REJECTED, CommissionTimingSummary,
            pack_commission_timing_summary,
        };
        let mut data = representative_data();
        let resistance_evidence = CaptureTimingEvidence {
            requested_period_us: 200,
            valid_samples: 40,
            missed_samples: 0,
            max_consecutive_misses: 0,
            max_lateness_us: 5,
            max_interval_us: 205,
            max_poll_wall_us: 12,
            max_spi_wall_us: 8,
        };
        let inductance_evidence = CaptureTimingEvidence {
            requested_period_us: 100,
            valid_samples: 64,
            missed_samples: 3,
            max_consecutive_misses: 1,
            max_lateness_us: 40,
            max_interval_us: 140,
            max_poll_wall_us: 20,
            max_spi_wall_us: 15,
        };
        data.resistance_timing = Some(resistance_evidence);
        data.inductance_timing = Some(inductance_evidence);
        set_timing_state(&mut data, 0, COMMISSION_TIMING_ACCEPTED);
        set_timing_state(&mut data, 1, COMMISSION_TIMING_REJECTED);
        data.timing_states |= COMMISSION_TIMING_REJECTED << 4;

        let summaries = electrical_timing_summaries(&data);

        assert_eq!(summaries[0].status, COMMISSION_TIMING_ACCEPTED);
        assert_eq!(summaries[0].evidence, resistance_evidence);
        assert_eq!(summaries[1].status, COMMISSION_TIMING_REJECTED);
        assert_eq!(summaries[1].evidence, inductance_evidence);
        assert_eq!(summaries[2], CommissionTimingSummary::NOT_RUN);
        assert_eq!(
            pack_commission_timing_summary(summaries),
            pack_commission_timing_summary([
                summaries[0],
                summaries[1],
                CommissionTimingSummary::NOT_RUN
            ])
        );
    }

    fn two_phase_motor_type_n_pole_pairs() -> u32 {
        foci_tmc4671::registers::MotorTypeNPolePairs::new()
            .with_motor_type(foci_tmc4671::registers::MotorType::TwoPhase)
            .with_n_pole_pairs(50)
            .into()
    }

    fn seed_resistance_power_stage_registers(backend: &mut MockBackend) {
        use foci_tmc4671::registers::{
            DSADC_MDEC_B_MDEC_A, MOTOR_TYPE_N_POLE_PAIRS, PWM_BBM_H_BBM_L, PWM_MAXCNT,
        };
        let config = *backend.tmc_config();
        backend
            .registers
            .insert(
                MOTOR_TYPE_N_POLE_PAIRS.addr(),
                two_phase_motor_type_n_pole_pairs(),
            )
            .ok();
        backend
            .registers
            .insert(PWM_MAXCNT.addr(), u32::from(config.pwm_maxcnt))
            .ok();
        backend
            .registers
            .insert(
                PWM_BBM_H_BBM_L.addr(),
                (u32::from(config.bbm_h) << 8) | u32::from(config.bbm_l),
            )
            .ok();
        backend
            .registers
            .insert(
                DSADC_MDEC_B_MDEC_A.addr(),
                (u32::from(config.dsadc_mdec_b) << 16) | u32::from(config.dsadc_mdec_a),
            )
            .ok();
    }

    fn register_operation_count(log: &[SpiOp]) -> usize {
        log.iter()
            .filter(|operation| matches!(operation, SpiOp::Read | SpiOp::Write(_, _)))
            .count()
    }

    fn write_position(log: &[SpiOp], register: Register, value: u32) -> usize {
        log.iter()
            .position(|op| {
                matches!(op, SpiOp::Write(addr, val) if *addr == register.addr() && *val == value)
            })
            .unwrap_or_else(|| {
                panic!(
                    "expected write of register {:#x} to {:#x}",
                    register.addr(),
                    value
                )
            })
    }

    fn enable_position(log: &[SpiOp], enabled: bool) -> usize {
        log.iter()
            .position(|op| matches!(op, SpiOp::EnablePin(state) if *state == enabled))
            .unwrap_or_else(|| panic!("expected set_enable_pin({enabled})"))
    }

    fn assert_ac_teardown_after_failure(backend: &MockBackend) {
        assert!(!backend.enable_pin, "AC failure must lower the enable pin");
        let enable_low = enable_position(&backend.spi_log, false);
        for (register, value) in [
            (UQ_UD_EXT, 0),
            (OPENLOOP_VELOCITY_TARGET, 0),
            (MODE_RAMP_MODE_MOTION, 0),
            (PHI_E_SELECTION, 0),
        ] {
            let position = write_position(&backend.spi_log, register, value);
            assert!(
                enable_low < position,
                "enable-low must precede register {:#x} teardown write",
                register.addr()
            );
        }
    }

    async fn finish_inductance_restore(
        phase: &mut InnerPhase,
        selftest_results: &mut crate::selftest::SelftestResults,
        diagnostics: &mut crate::selftest::CommissionDiagnostics,
        backend: &mut MockBackend,
    ) -> StepResult {
        for _ in 0..21 {
            let result = step_electrical_identification(
                phase,
                InnerSequence::Commission,
                selftest_results,
                diagnostics,
                backend,
            )
            .await;
            if !matches!(result, StepResult::ReadyNow) {
                return result;
            }
        }
        panic!("inductance restoration did not terminate");
    }

    fn expected_inductance_restore_writes() -> [(u8, u32); 20] {
        [
            (UQ_UD_EXT.addr(), 0),
            (OPENLOOP_VELOCITY_TARGET.addr(), 0),
            (MODE_RAMP_MODE_MOTION.addr(), 0),
            (PHI_E_SELECTION.addr(), 0),
            (MODE_RAMP_MODE_MOTION.addr(), 0),
            (PID_TORQUE_FLUX_TARGET.addr(), 0),
            (PID_TORQUE_FLUX_OFFSET.addr(), 0),
            (UQ_UD_EXT.addr(), 0),
            (PHI_E_SELECTION.addr(), 0),
            (PWM_SV_CHOP.addr(), 0),
            (PID_VELOCITY_OFFSET.addr(), 0),
            (PIDOUT_UQ_UD_LIMITS.addr(), 1024),
            (
                PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr(),
                PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr as u32,
            ),
            (PID_ERROR_PID_TORQUE_ERROR_SUM.data.addr(), 0),
            (
                PID_ERROR_PID_FLUX_ERROR_SUM.addr.addr(),
                PID_ERROR_PID_FLUX_ERROR_SUM.sub_addr as u32,
            ),
            (PID_ERROR_PID_FLUX_ERROR_SUM.data.addr(), 0),
            (
                PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(),
                PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr as u32,
            ),
            (PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
            (
                PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
            ),
            (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
        ]
    }

    async fn run_bounded_inductance_restore(
        phase: &mut InnerPhase,
        backend: &mut MockBackend,
    ) -> StepResult {
        let mut selftest_results = crate::selftest::SelftestResults::default();
        let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
        for operation in 0..20 {
            let writes_before = backend
                .spi_log
                .iter()
                .filter(|entry| matches!(entry, SpiOp::Write(_, _)))
                .count();
            let result = step_electrical_identification(
                phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                backend,
            )
            .await;
            let writes_after = backend
                .spi_log
                .iter()
                .filter(|entry| matches!(entry, SpiOp::Write(_, _)))
                .count();
            let expected_writes = if backend.read_override.is_some_and(|(address, _)| {
                address == expected_inductance_restore_writes()[operation].0
            }) {
                3
            } else {
                1
            };
            assert_eq!(
                writes_after - writes_before,
                expected_writes,
                "restore poll {operation}"
            );
            if operation < 19 {
                assert!(matches!(result, StepResult::ReadyNow));
            } else {
                return result;
            }
        }
        unreachable!()
    }

    fn begin_test_inductance_restore(
        backend: &mut MockBackend,
        primary_error: Option<CommissionError>,
        evidence: Option<InductanceEvidence>,
    ) -> InnerPhase {
        let mut state = ElectricalIdentificationState::InductanceSetup;
        let result = begin_inductance_restore(&mut state, backend, primary_error, evidence, None);
        assert!(matches!(result, StepResult::ReadyNow));
        let mut data = representative_data();
        data.inductance = None;
        data.inductance_timing = None;
        InnerPhase::ElectricalIdentification { state, data }
    }

    #[test]
    fn inductance_restore_runs_complete_verified_sequence_before_cooldown() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            let mut phase = begin_test_inductance_restore(
                &mut backend,
                None,
                Some(inductance_evidence_fixture(8_600, 1_000_000)),
            );

            let result = run_bounded_inductance_restore(&mut phase, &mut backend).await;

            assert!(matches!(result, StepResult::At(_)));
            let InnerPhase::ElectricalIdentification { state, data } = &phase else {
                panic!("expected electrical-id cooldown, got {phase:?}");
            };
            assert!(matches!(
                state,
                ElectricalIdentificationState::InductanceDisable { .. }
            ));
            assert!(data.inductance.is_some());
            assert!(!backend.enable_pin);
            let enable_low = enable_position(&backend.spi_log, false);
            let writes: heapless::Vec<(u8, u32), 32> = backend
                .spi_log
                .iter()
                .filter_map(|entry| match entry {
                    SpiOp::Write(address, value) => Some((*address, *value)),
                    _ => None,
                })
                .collect();
            assert_eq!(writes.as_slice(), &expected_inductance_restore_writes());
            assert!(enable_low < write_position(&backend.spi_log, UQ_UD_EXT, 0));
        });
    }

    #[test]
    fn inductance_restore_mismatch_replaces_success_with_spi_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.read_override = Some((UQ_UD_EXT.addr(), 1));
            let mut phase = begin_test_inductance_restore(
                &mut backend,
                None,
                Some(inductance_evidence_fixture(8_600, 1_000_000)),
            );

            let result = run_bounded_inductance_restore(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, InnerPhase::Failed(CommissionError::Spi)));
        });
    }

    #[test]
    fn inductance_restore_mismatch_replaces_primary_error_with_spi_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.read_override = Some((UQ_UD_EXT.addr(), 1));
            let mut phase = begin_test_inductance_restore(
                &mut backend,
                Some(CommissionError::InductanceCaptureRejected),
                None,
            );

            let result = run_bounded_inductance_restore(&mut phase, &mut backend).await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            assert!(matches!(phase, InnerPhase::Failed(CommissionError::Spi)));
        });
    }

    async fn drive_inductance_schedule(
        phase: &mut InnerPhase,
        selftest_results: &mut crate::selftest::SelftestResults,
        diagnostics: &mut crate::selftest::CommissionDiagnostics,
        backend: &mut MockBackend,
    ) -> StepResult {
        for _ in 0..512 {
            let result = step_electrical_identification(
                phase,
                InnerSequence::Commission,
                selftest_results,
                diagnostics,
                backend,
            )
            .await;
            if matches!(
                phase,
                InnerPhase::ElectricalIdentification {
                    state: ElectricalIdentificationState::InductanceDisable { .. },
                    ..
                }
            ) {
                return StepResult::ReadyNow;
            }
            match result {
                StepResult::ReadyNow => {}
                StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                other => return other,
            }
        }
        panic!("inductance schedule did not terminate");
    }

    fn find_inductance_capture_rejection(
        diagnostics: &crate::selftest::CommissionDiagnostics,
    ) -> crate::selftest::CommissionDiagnostic {
        diagnostics
            .iter()
            .find(|event| {
                event.phase == CommissionPhase::ElectricalIdentification.to_u8() && event.code == 33
            })
            .expect("inductance capture rejection should retain branch detail")
    }

    #[test]
    fn inductance_setup_bounds_voltage_before_enable_and_starts_hardware_ramp() {
        pollster::block_on(async {
            let mut data = ElectricalIdentificationData::new(
                1024,
                crate::commission_dispatch::test_support::current_limit(2816),
            );
            data.encoder_domain = Some(EncoderDomain::new(4000).unwrap());
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSetup,
                data,
            };
            let mut backend = MockBackend::new();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(result, StepResult::ReadyNow),
                "scheduler gap must not make the AC window fail: {result:?}"
            );
            for _ in 0..10 {
                let result = step_electrical_identification(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                assert!(matches!(result, StepResult::ReadyNow));
            }
            assert!(backend.enable_pin);
            assert!(matches!(
                phase,
                InnerPhase::ElectricalIdentification {
                    state: ElectricalIdentificationState::InductanceWaitVelocity { .. },
                    ..
                }
            ));

            let enable_high = enable_position(&backend.spi_log, true);
            for (register, value) in [
                (PIDOUT_UQ_UD_LIMITS, INDUCTANCE_AC_UD_EXT as u32),
                (UQ_UD_EXT, 0),
                (OPENLOOP_VELOCITY_TARGET, 0),
                (
                    OPENLOOP_VELOCITY_TARGET,
                    INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM,
                ),
                (PHI_E_SELECTION, PHI_E_SELECTION_OPENLOOP),
                (PWM_SV_CHOP, 7),
                (STATUS_FLAGS, 0),
            ] {
                let position = write_position(&backend.spi_log, register, value);
                assert!(
                    position < enable_high,
                    "register {:#x} setup write must precede enable-high",
                    register.addr()
                );
            }
        });
    }

    #[test]
    fn inductance_setup_refuses_to_enable_when_the_interlock_trips_during_the_status_clear() {
        pollster::block_on(async {
            let mut data = ElectricalIdentificationData::new(
                1024,
                crate::commission_dispatch::test_support::current_limit(2816),
            );
            data.encoder_domain = Some(EncoderDomain::new(4000).unwrap());
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSetup,
                data,
            };
            let mut backend = MockBackend::new();
            backend.trip_interlock_on_write = Some(STATUS_FLAGS.addr());
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            for _ in 0..11 {
                let result = step_electrical_identification(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                assert!(matches!(result, StepResult::ReadyNow));
            }

            assert!(
                !backend.enable_pin,
                "inductance setup must not enable behind a tripped interlock"
            );
            let InnerPhase::ElectricalIdentification { state, .. } = &phase else {
                panic!("expected electrical identification, got {phase:?}");
            };
            assert!(
                matches!(
                    state,
                    ElectricalIdentificationState::InductanceRestore {
                        primary_error: Some(CommissionError::FaultInterlock),
                        ..
                    }
                ),
                "the interlock refusal must reach the host as FaultInterlock, got {state:?}"
            );
        });
    }

    #[test]
    fn inductance_velocity_timeout_tears_down_openloop_drive() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceWaitVelocity { deadline_us: 10 },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = 20;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let start_restore = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(start_restore, StepResult::ReadyNow));
            let result = finish_inductance_restore(
                &mut phase,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceVelocityTimeout)
            ));
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_velocity_wait_repolls_before_final_deadline() {
        pollster::block_on(async {
            let final_deadline_us = 2_000;
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceWaitVelocity {
                    deadline_us: final_deadline_us,
                },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend
                .registers
                .insert(OPENLOOP_VELOCITY_ACTUAL.addr(), 600)
                .ok();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let waiting = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            let StepResult::At(repoll_us) = waiting else {
                panic!("expected a scheduled velocity repoll, got {waiting:?}");
            };
            assert!(
                repoll_us > 0 && repoll_us < final_deadline_us,
                "velocity ramp must be repolled later and before the final deadline"
            );

            backend.elapsed_us_value = repoll_us;
            backend
                .registers
                .insert(
                    OPENLOOP_VELOCITY_ACTUAL.addr(),
                    INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM,
                )
                .ok();
            let ready = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(ready, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                InnerPhase::ElectricalIdentification {
                    state: ElectricalIdentificationState::InductanceReadEncoderStart,
                    ..
                }
            ));
        });
    }

    #[test]
    fn inductance_sample_spi_failure_tears_down_openloop_drive() {
        pollster::block_on(async {
            let mut capture = inductance_capture_state(0, 1_000, 0, 0);
            capture
                .timing
                .record_sample(1_000, 1_000, 1_010, 1_020, 10)
                .unwrap();
            let mut data = representative_data();
            data.timing_states = crate::commissioning::types::COMMISSION_TIMING_REJECTED << 2;
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                data,
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.fail_on_read = Some(PID_TORQUE_FLUX_ACTUAL.addr());
            backend.registers.insert(OPENLOOP_PHI.addr(), 1).ok();
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let start_restore = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(start_restore, StepResult::ReadyNow));
            let result = finish_inductance_restore(
                &mut phase,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::Spi)
            ));
            let timing = diagnostics
                .iter()
                .find(|event| event.code == 36)
                .expect("inductance timing header");
            assert_eq!(timing.value0 >> 16, 1);
            assert_eq!((timing.value1 >> 24) & 0x03, 2);
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_capture_rejection_records_zero_phi_delta_detail() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample {
                    capture: inductance_capture_state(0, 1_000, 0, 0),
                },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.set_openloop_phi_sequence(&[1_000], 0);
            backend.set_pid_torque_flux_actual_sequence(&[((-84i16 as u16 as u32) << 16) | 20]);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let start_restore = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(start_restore, StepResult::ReadyNow));
            let result = finish_inductance_restore(
                &mut phase,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceCaptureRejected)
            ));
            let detail = find_inductance_capture_rejection(&diagnostics);
            assert_eq!(detail.status, 1);
            assert_eq!(detail.value0, 2);
            assert_eq!(detail.value1, 0);
            assert_eq!(detail.value2, (1_000u32 << 16) | 1_000u32);
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_capture_rejection_records_elapsed_interpolation_detail() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample {
                    capture: inductance_capture_state(
                        9_000,
                        0,
                        INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS - 1_000,
                        INDUCTANCE_MIN_AC_SAMPLE_COUNT,
                    ),
                },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = u64::from(INDUCTANCE_AC_SAMPLE_PERIOD_US);
            backend.set_openloop_phi_sequence(&[1_000], 0);
            backend.set_pid_torque_flux_actual_sequence(&[((-84i16 as u16 as u32) << 16) | 20]);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let start_restore = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(start_restore, StepResult::ReadyNow));
            let result = finish_inductance_restore(
                &mut phase,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceCaptureRejected)
            ));
            let detail = find_inductance_capture_rejection(&diagnostics);
            assert_eq!(detail.status, 1);
            assert_eq!(detail.value0, 7);
            assert_eq!(detail.value1, u32::from(INDUCTANCE_MIN_AC_SAMPLE_COUNT + 1));
            assert_eq!(
                detail.value2,
                (9_000u32 << 16) | u32::from(INDUCTANCE_AC_SAMPLE_PERIOD_US)
            );
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_sample_reads_one_phi_current_pair_and_returns_deadline() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample {
                    capture: inductance_capture_state(0, 0, 0, 0),
                },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend
                .registers
                .insert(foci_tmc4671::registers::STATUS_FLAGS.addr(), 0)
                .ok();

            let mut phi_samples = heapless::Vec::<u32, 128>::new();
            let mut current_samples = heapless::Vec::<u32, 128>::new();
            let actual = ((-84i16 as u16 as u32) << 16) | 20u32;
            for index in 1..=100u32 {
                phi_samples.push(index.wrapping_mul(5_243) & 0xffff).ok();
                current_samples.push(actual).ok();
            }
            backend.set_openloop_phi_sequence(phi_samples.as_slice(), 0);
            backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());

            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(_)));
            assert_eq!(register_operation_count(&backend.spi_log), 2);
            assert!(matches!(
                phase,
                InnerPhase::ElectricalIdentification {
                    state: ElectricalIdentificationState::InductanceSample { .. },
                    ..
                }
            ));
        });
    }

    #[test]
    fn scheduled_inductance_uses_observation_timestamp_for_estimator_elapsed_time() {
        pollster::block_on(async {
            let mut capture = inductance_capture_state(0, 0, 0, 0);
            capture.start_us = 900;
            capture.next_sample_us = 1_000;
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.set_elapsed_us_sequence(&[1_000, 1_000, 1_080, 1_080, 1_090]);
            backend.set_openloop_phi_sequence(&[1_000], 0);
            backend.set_pid_torque_flux_actual_sequence(&[((-84i16 as u16 as u32) << 16) | 20]);
            let mut selftest = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(1_200)));
            let InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                ..
            } = phase
            else {
                panic!("capture should remain active");
            };
            assert_eq!(capture.previous_elapsed_us, 180);
            assert_eq!(backend.register_pair_reads, 1);
        });
    }

    #[test]
    fn inductance_poll_wall_includes_work_after_observation() {
        pollster::block_on(async {
            let mut capture = inductance_capture_state(0, 0, 0, 0);
            capture.start_us = 10_000;
            capture.next_sample_us = 10_000;
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 10_020;
            backend.set_elapsed_us_sequence(&[10_000, 10_000, 10_020, 10_020, 10_100]);
            backend.set_openloop_phi_sequence(&[1_000], 0);
            backend.set_pid_torque_flux_actual_sequence(&[((-84i16 as u16 as u32) << 16) | 20]);
            let mut selftest = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(
                matches!(result, StepResult::At(10_200)),
                "expected next sample deadline, got {result:?} in {phase:?}"
            );
            let InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                ..
            } = phase
            else {
                panic!("capture should remain active");
            };
            let timing = capture.timing.finish().unwrap();
            assert_eq!(timing.max_spi_wall_us, 20);
            assert_eq!(timing.max_poll_wall_us, 100);
        });
    }

    #[test]
    fn inductance_accepts_two_nonconsecutive_misses_without_catch_up() {
        pollster::block_on(async {
            let mut capture = inductance_capture_state(0, 0, 0, 0);
            capture.next_sample_us = 200;
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.set_openloop_phi_sequence(&[1_000, 2_000, 3_000, 4_000], 0);
            let actual = ((-84i16 as u16 as u32) << 16) | 20;
            backend.set_pid_torque_flux_actual_sequence(&[actual; 4]);
            let mut selftest = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            for now in [400, 600, 1_000, 1_200] {
                backend.elapsed_us_value = now;
                let before = register_operation_count(&backend.spi_log);
                let result = step_electrical_identification(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                assert!(
                    matches!(result, StepResult::At(_)),
                    "now={now} result={result:?} phase={phase:?}"
                );
                assert_eq!(register_operation_count(&backend.spi_log) - before, 2);
            }
            let InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                ..
            } = phase
            else {
                panic!("capture should remain active");
            };
            let timing = capture.timing.finish().unwrap();
            assert_eq!(timing.missed_samples, 2);
            assert_eq!(timing.max_consecutive_misses, 1);
        });
    }

    #[test]
    fn inductance_skips_excessively_late_slot_before_spi() {
        pollster::block_on(async {
            let mut capture = inductance_capture_state(0, 0, 0, 0);
            capture.next_sample_us = 200;
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = 292;
            let mut selftest = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let before = register_operation_count(&backend.spi_log);
            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(400)));
            assert_eq!(register_operation_count(&backend.spi_log) - before, 0);
            let InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                ..
            } = phase
            else {
                panic!("capture should remain active");
            };
            let evidence = capture.timing.finish().unwrap();
            assert_eq!(evidence.valid_samples, 0);
            assert_eq!(evidence.missed_samples, 1);
            assert_eq!(evidence.max_consecutive_misses, 1);
            assert_eq!(evidence.max_lateness_us, 92);
        });
    }

    #[test]
    fn inductance_retains_two_consecutive_late_slots_before_spi() {
        pollster::block_on(async {
            let mut capture = inductance_capture_state(0, 0, 0, 0);
            capture.next_sample_us = 200;
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = 468;
            let mut selftest = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let before = register_operation_count(&backend.spi_log);
            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(600)));
            assert_eq!(register_operation_count(&backend.spi_log) - before, 0);
            assert!(backend.enable_pin);
            let InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                ..
            } = phase
            else {
                panic!("capture should remain active");
            };
            assert_eq!(capture.next_sample_us, 600);
            let evidence = capture.timing.finish().unwrap();
            assert_eq!(evidence.missed_samples, 2);
            assert_eq!(evidence.max_consecutive_misses, 2);
        });
    }

    #[test]
    fn inductance_six_total_misses_defer_to_final_coverage_gate() {
        pollster::block_on(async {
            let mut capture = inductance_capture_state(0, 0, 0, 0);
            capture.next_sample_us = 200;
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.set_openloop_phi_sequence(&[26_214, 39_321, 13_107, 26_214, 0, 13_107], 0);
            let actual = ((-84i16 as u16 as u32) << 16) | 20;
            backend.set_pid_torque_flux_actual_sequence(&[actual; 6]);
            let mut selftest = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            for now in [400, 600, 1_200, 1_400, 2_000] {
                backend.elapsed_us_value = now;
                let _ = step_electrical_identification(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
            }
            backend.elapsed_us_value = 2_400;
            let before = register_operation_count(&backend.spi_log);
            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::At(2_600)));
            assert_eq!(register_operation_count(&backend.spi_log) - before, 2);
            assert!(backend.enable_pin);
            match phase {
                InnerPhase::ElectricalIdentification {
                    state: ElectricalIdentificationState::InductanceSample { capture },
                    ..
                } => {
                    let evidence = capture.timing.finish().unwrap();
                    assert_eq!(capture.next_sample_us, 2_600);
                    assert_eq!(evidence.missed_samples, 6);
                    assert_eq!(evidence.max_consecutive_misses, 2);
                }
                InnerPhase::ElectricalIdentification { state, .. } => {
                    panic!("expected active inductance capture, got {state:?}")
                }
                _ => panic!("expected electrical identification phase"),
            }
        });
    }

    #[test]
    fn inductance_completion_uses_separate_status_and_encoder_polls() {
        pollster::block_on(async {
            let mut capture = inductance_capture_state(
                0,
                0,
                INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS - 1_000,
                INDUCTANCE_MIN_AC_SAMPLE_COUNT,
            );
            capture.next_sample_us = 100;
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample { capture },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = 100;
            backend.set_openloop_phi_sequence(&[1_000], 0);
            backend.set_pid_torque_flux_actual_sequence(&[((-84i16 as u16 as u32) << 16) | 20]);
            let mut selftest = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let sample = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(sample, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                InnerPhase::ElectricalIdentification {
                    state: ElectricalIdentificationState::InductanceReadStatus { .. },
                    ..
                }
            ));
            let before_status = register_operation_count(&backend.spi_log);
            let _ = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert_eq!(
                register_operation_count(&backend.spi_log) - before_status,
                1
            );
            assert!(matches!(
                phase,
                InnerPhase::ElectricalIdentification {
                    state: ElectricalIdentificationState::InductanceReadEncoder { .. },
                    ..
                }
            ));
        });
    }

    #[test]
    fn inductance_sample_excludes_scheduler_gap_before_first_phi_sample() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceReadStartPhi {
                    encoder_start: 0,
                    encoder_domain: EncoderDomain::new(4000).unwrap(),
                    status_flags_or: 0,
                },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend.elapsed_us_value = 1_046;
            backend
                .registers
                .insert(foci_tmc4671::registers::STATUS_FLAGS.addr(), 0)
                .ok();

            let mut phi_samples = heapless::Vec::<u32, 128>::new();
            let mut current_samples = heapless::Vec::<u32, 128>::new();
            let actual = ((-84i16 as u16 as u32) << 16) | 20u32;
            for index in 0..=100u32 {
                phi_samples
                    .push(u32::from(fixed_window_phi(index, 100)))
                    .ok();
                current_samples.push(actual).ok();
            }
            backend.set_openloop_phi_sequence(phi_samples.as_slice(), 0);
            backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());

            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = drive_inductance_schedule(
                &mut phase,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            let InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceDisable { .. },
                data,
            } = phase
            else {
                panic!("expected completed inductance capture, got {phase:?}");
            };
            let evidence = data
                .inductance
                .expect("inductance evidence should be retained");
            assert!(
                (NOMINAL_INDUCTANCE_CAPTURE_US - 100..=NOMINAL_INDUCTANCE_CAPTURE_US + 100)
                    .contains(&evidence.run.elapsed_us),
                "scheduler gap must not be charged to AC elapsed_us: {}",
                evidence.run.elapsed_us
            );
            assert!(realized_frequency_in_range(
                evidence.run.realized_frequency_millihz
            ));
        });
    }

    #[test]
    fn inductance_frequency_rejection_records_measured_window_detail() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample {
                    capture: inductance_capture_state(0, 0, 0, 0),
                },
                data: representative_data(),
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            backend
                .registers
                .insert(foci_tmc4671::registers::STATUS_FLAGS.addr(), 0)
                .ok();

            let mut phi_samples = heapless::Vec::<u32, 128>::new();
            let mut current_samples = heapless::Vec::<u32, 128>::new();
            let actual = ((-84i16 as u16 as u32) << 16) | 20u32;
            for index in 1..=100u32 {
                phi_samples
                    .push(u32::from(fixed_window_phi(index, 90)))
                    .ok();
                current_samples.push(actual).ok();
            }
            backend.set_openloop_phi_sequence(phi_samples.as_slice(), 0);
            backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());

            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = drive_inductance_schedule(
                &mut phase,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceFrequencyOutOfRange)
            ));
            let detail = diagnostics
                .iter()
                .find(|event| event.code == DIAG_ELECTRICAL_ID_REJECT_INDUCTANCE_FREQUENCY)
                .expect("frequency rejection should retain realized-frequency evidence");
            assert_eq!(
                detail.phase,
                CommissionPhase::ElectricalIdentification.to_u8()
            );
            assert_eq!(detail.status, 1);
            assert!(detail.value0 > INDUCTANCE_REALIZED_FREQUENCY_TARGET_MILLIHZ);
            assert!(detail.value1 < NOMINAL_INDUCTANCE_CAPTURE_US);
            assert!(detail.value2 >= u32::from(INDUCTANCE_MIN_AC_SAMPLE_COUNT));
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_finish_accepts_benign_status_flags_as_warning() {
        let mut accumulator = crate::electrical_capture::AcFrameAccumulator::new();
        let mut first_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut second_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut saliency = crate::electrical_capture::AcSaliencyAccumulator::new();

        for index in 0..100u32 {
            let weight = fixed_window_phi_weight(index, 100);
            let phi = fixed_window_phi(index, 100);
            accumulator.push_weighted(20, -84, weight).unwrap();
            saliency.push_weighted_sample(phi, weight, 20, -84).unwrap();
            if index < 50 {
                first_half.push_weighted(20, -84, weight).unwrap();
            } else {
                second_half.push_weighted(20, -84, weight).unwrap();
            }
        }

        let evidence = finish_inductance_evidence(InductanceCaptureFinish {
            data: &representative_data(),
            encoder_domain: EncoderDomain::new(4000).unwrap(),
            encoder_start: 10,
            encoder_after: 10,
            elapsed_us: NOMINAL_INDUCTANCE_CAPTURE_US,
            phi_delta_counts: INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
            sample_count: 100,
            status_flags_or: 0x7000_4400,
            accumulator,
            first_half,
            second_half,
            saliency_accumulator: saliency,
        })
        .expect("non-actionable hardware status flags should not reject inductance evidence");

        assert_eq!(evidence.run.status_flags_or, 0x7000_4400);
        assert_ne!(evidence.run.warning_flags & INDUCTANCE_WARN_STATUS_FLAGS, 0);
    }

    #[test]
    fn inductance_finish_marks_isotropic_capture_not_determinable() {
        let mut accumulator = crate::electrical_capture::AcFrameAccumulator::new();
        let mut first_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut second_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut saliency = crate::electrical_capture::AcSaliencyAccumulator::new();

        for index in 0..100u32 {
            let weight = fixed_window_phi_weight(index, 100);
            let phi = fixed_window_phi(index, 100);
            accumulator.push_weighted(20, -84, weight).unwrap();
            saliency.push_weighted_sample(phi, weight, 20, -84).unwrap();
            if index < 50 {
                first_half.push_weighted(20, -84, weight).unwrap();
            } else {
                second_half.push_weighted(20, -84, weight).unwrap();
            }
        }

        let expected_x_average = expected_frame_x_quad(accumulator);

        let evidence = finish_inductance_evidence(InductanceCaptureFinish {
            data: &representative_data(),
            encoder_domain: EncoderDomain::new(4000).unwrap(),
            encoder_start: 10,
            encoder_after: 10,
            elapsed_us: NOMINAL_INDUCTANCE_CAPTURE_US,
            phi_delta_counts: INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
            sample_count: 100,
            status_flags_or: 0,
            accumulator,
            first_half,
            second_half,
            saliency_accumulator: saliency,
        })
        .expect("isotropic capture is accepted");

        assert_eq!(
            evidence.estimate.saliency_status,
            crate::electrical_capture::AC_SALIENCY_NOT_DETERMINABLE
        );
        assert_eq!(evidence.estimate.x_d_count_ratio_milli, 0);
        assert_eq!(evidence.estimate.x_q_count_ratio_milli, 0);
        assert_eq!(evidence.estimate.saliency_permille, 0);
        assert_eq!(
            evidence.estimate.x_average_count_ratio_milli,
            expected_x_average
        );
    }

    #[test]
    fn inductance_finish_publishes_strong_saliency_bracketing_the_average() {
        let mut accumulator = crate::electrical_capture::AcFrameAccumulator::new();
        let mut first_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut second_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut saliency = crate::electrical_capture::AcSaliencyAccumulator::new();

        let x_avg = 8.5_f64;
        let r = 0.25 * x_avg;
        for index in 0..100u32 {
            let weight = fixed_window_phi_weight(index, 100);
            let phi = fixed_window_phi(index, 100);
            let theta = f64::from(phi) * core::f64::consts::TAU / 65_536.0;
            let x = x_avg * (1.0 + 0.30 * (2.0 * theta + 0.9).cos());
            let z_sq = r * r + x * x;
            let id = (f64::from(INDUCTANCE_AC_UD_EXT) * r / z_sq).round() as i16;
            let iq = (-f64::from(INDUCTANCE_AC_UD_EXT) * x / z_sq).round() as i16;
            accumulator.push_weighted(id, iq, weight).unwrap();
            saliency.push_weighted_sample(phi, weight, id, iq).unwrap();
            if index < 50 {
                first_half.push_weighted(id, iq, weight).unwrap();
            } else {
                second_half.push_weighted(id, iq, weight).unwrap();
            }
        }

        let expected_x_average = expected_frame_x_quad(accumulator);

        let evidence = finish_inductance_evidence(InductanceCaptureFinish {
            data: &representative_data(),
            encoder_domain: EncoderDomain::new(4000).unwrap(),
            encoder_start: 10,
            encoder_after: 10,
            elapsed_us: NOMINAL_INDUCTANCE_CAPTURE_US,
            phi_delta_counts: INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
            sample_count: 100,
            status_flags_or: 0,
            accumulator,
            first_half,
            second_half,
            saliency_accumulator: saliency,
        })
        .expect("salient capture is accepted");

        let estimate = evidence.estimate;
        assert_eq!(estimate.x_average_count_ratio_milli, expected_x_average);
        assert_eq!(
            estimate.saliency_status,
            crate::electrical_capture::AC_SALIENCY_DETERMINABLE
        );
        assert!(estimate.x_d_count_ratio_milli > estimate.x_average_count_ratio_milli);
        assert!(estimate.x_average_count_ratio_milli > estimate.x_q_count_ratio_milli);
        assert_eq!(
            estimate.x_d_count_ratio_milli - estimate.x_average_count_ratio_milli,
            estimate.x_average_count_ratio_milli - estimate.x_q_count_ratio_milli
        );
        assert!(
            (540..=660).contains(&estimate.saliency_permille),
            "{}",
            estimate.saliency_permille
        );
    }

    fn expected_frame_x_quad(accumulator: crate::electrical_capture::AcFrameAccumulator) -> u32 {
        let frame = accumulator
            .finish(
                NOMINAL_INDUCTANCE_CAPTURE_US,
                1_000_000,
                INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
            )
            .expect("frame evidence");
        crate::electrical_capture::rotating_frame_reactance(frame, INDUCTANCE_AC_UD_EXT, 1_706)
            .expect("frame reactance")
            .x_quad_count_ratio_milli
    }

    #[test]
    fn inductance_finish_reports_encoder_motion_rejection() {
        let mut accumulator = crate::electrical_capture::AcFrameAccumulator::new();
        let mut first_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut second_half = crate::electrical_capture::AcFrameAccumulator::new();
        let mut saliency = crate::electrical_capture::AcSaliencyAccumulator::new();

        for index in 0..100u32 {
            let weight = fixed_window_phi_weight(index, 100);
            let phi = fixed_window_phi(index, 100);
            accumulator.push_weighted(20, -84, weight).unwrap();
            saliency.push_weighted_sample(phi, weight, 20, -84).unwrap();
            if index < 50 {
                first_half.push_weighted(20, -84, weight).unwrap();
            } else {
                second_half.push_weighted(20, -84, weight).unwrap();
            }
        }

        let rejection = match finish_inductance_evidence(InductanceCaptureFinish {
            data: &representative_data(),
            encoder_domain: EncoderDomain::new(4000).unwrap(),
            encoder_start: 10,
            encoder_after: 10 + INDUCTANCE_MAX_ENCODER_DELTA_COUNTS + 1,
            elapsed_us: NOMINAL_INDUCTANCE_CAPTURE_US,
            phi_delta_counts: INDUCTANCE_AC_TARGET_PHI_DELTA_COUNTS,
            sample_count: 100,
            status_flags_or: 0,
            accumulator,
            first_half,
            second_half,
            saliency_accumulator: saliency,
        }) {
            Ok(_) => panic!("encoder motion must reject the inductance capture"),
            Err(rejection) => rejection,
        };

        assert_eq!(rejection, InductanceReject::EncoderMotion);
    }

    #[test]
    fn inductance_encoder_delta_crosses_origin_forward() {
        let domain = EncoderDomain::new(4000).unwrap();

        assert_eq!(
            inductance_encoder_delta(domain, 3998, 2)
                .unwrap()
                .signed_counts(),
            4
        );
    }

    #[test]
    fn inductance_encoder_delta_maps_half_turn_to_motion_rejection() {
        let domain = EncoderDomain::new(4000).unwrap();

        assert_eq!(
            inductance_encoder_delta(domain, 0, 2000),
            Err(InductanceReject::EncoderMotion)
        );
        assert_eq!(
            InductanceReject::EncoderMotion.commission_error(),
            CommissionError::InductanceEncoderMotion
        );
    }

    #[test]
    fn inductance_pre_status_tolerates_entry_demoted_limiter_bit() {
        pollster::block_on(async {
            let domain = EncoderDomain::new(4000).unwrap();
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceReadPreStatus {
                    encoder_start: 0,
                    encoder_domain: domain,
                },
                data: ElectricalIdentificationData::new(
                    1024,
                    crate::commission_dispatch::test_support::current_limit(2816),
                ),
            };
            let mut backend = MockBackend::new();
            let _ = backend.registers.insert(
                foci_tmc4671::registers::STATUS_FLAGS.addr(),
                IPARK_CIRCULAR_LIMIT_BIT,
            );
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert!(
                matches!(
                    phase,
                    InnerPhase::ElectricalIdentification {
                        state: ElectricalIdentificationState::InductanceReadStartPhi { .. },
                        ..
                    }
                ),
                "entry-demoted limiter bit must not reject the inductance pre-status check: {phase:?}"
            );
        });
    }

    #[test]
    fn inductance_capture_status_tolerates_entry_demoted_limiter_bit() {
        pollster::block_on(async {
            let domain = EncoderDomain::new(4000).unwrap();
            let capture = InductanceCaptureState {
                encoder_domain: domain,
                encoder_start: 0,
                start_us: 0,
                previous_elapsed_us: 0,
                previous_phi: 0,
                accumulated_phi_delta_counts: 0,
                accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
                first_half: crate::electrical_capture::AcFrameAccumulator::new(),
                second_half: crate::electrical_capture::AcFrameAccumulator::new(),
                saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
                sample_count: 100,
                status_flags_or: 0,
                next_sample_us: 0,
                timing: CaptureTiming::new(
                    INDUCTANCE_AC_SAMPLE_PERIOD_US,
                    TimingEnvelope::INDUCTANCE,
                ),
                current_envelope: crate::tmc_control::current_limit::diagnostic_current_envelope(
                    crate::commission_dispatch::test_support::current_limit(2816),
                    INDUCTANCE_DIAGNOSTIC_CURRENT_FRACTION,
                ),
            };
            let (timing_evidence, _overflowed) = capture.timing.snapshot();
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceReadStatus {
                    capture,
                    completed_elapsed_us: 0,
                    timing: timing_evidence,
                },
                data: ElectricalIdentificationData::new(
                    1024,
                    crate::commission_dispatch::test_support::current_limit(2816),
                ),
            };
            let mut backend = MockBackend::new();
            let _ = backend.registers.insert(
                foci_tmc4671::registers::STATUS_FLAGS.addr(),
                IPARK_CIRCULAR_LIMIT_BIT,
            );
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(result, StepResult::ReadyNow));
            assert!(
                matches!(
                    phase,
                    InnerPhase::ElectricalIdentification {
                        state: ElectricalIdentificationState::InductanceReadEncoder { .. },
                        ..
                    }
                ),
                "entry-demoted limiter bit must not reject the inductance mid-capture check: {phase:?}"
            );
        });
    }

    #[test]
    fn inductance_setup_rejects_missing_validated_domain_before_excitation() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSetup,
                data: ElectricalIdentificationData::new(
                    1024,
                    crate::commission_dispatch::test_support::current_limit(2816),
                ),
            };
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 0;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
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
            assert!(backend.spi_log.is_empty());
        });
    }

    #[test]
    fn resistance_rejects_invalid_domain_before_identification() {
        pollster::block_on(async {
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::ResistanceStart,
                data: ElectricalIdentificationData::new(
                    1024,
                    crate::commission_dispatch::test_support::current_limit(2816),
                ),
            };
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 0;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = step_electrical_identification(
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
            assert!(
                backend.spi_log.is_empty(),
                "invalid encoder configuration must not start resistance excitation"
            );
        });
    }

    #[test]
    fn inductance_half_turn_path_tears_down_without_committing_evidence() {
        pollster::block_on(async {
            let mut data = ElectricalIdentificationData::new(
                512,
                crate::commission_dispatch::test_support::current_limit(2816),
            );
            data.resistance_result = Some(resistance_result_fixture());
            data.encoder_domain = Some(EncoderDomain::new(4000).unwrap());
            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::InductanceSample {
                    capture: inductance_capture_state(0, 0, 0, 0),
                },
                data,
            };
            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            let _ = backend.registers.insert(STATUS_FLAGS.addr(), 0);
            let _ = backend
                .registers
                .insert(foci_tmc4671::registers::ABN_DECODER_COUNT.addr(), 2000);
            let mut phi_samples = heapless::Vec::<u32, 128>::new();
            let mut current_samples = heapless::Vec::<u32, 128>::new();
            let actual = ((-84i16 as u16 as u32) << 16) | 20u32;
            for index in 1..=100u32 {
                phi_samples
                    .push(u32::from(fixed_window_phi(index, 100)))
                    .ok();
                current_samples.push(actual).ok();
            }
            backend.set_openloop_phi_sequence(phi_samples.as_slice(), 0);
            backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = drive_inductance_schedule(
                &mut phase,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceEncoderMotion)
            ));
            assert!(matches!(
                phase,
                InnerPhase::Failed(CommissionError::InductanceEncoderMotion)
            ));
            assert_ac_teardown_after_failure(&backend);
        });
    }

    fn inductance_envelope_fixture(sample: u32) -> (InnerPhase, MockBackend) {
        let mut data = ElectricalIdentificationData::new(512, current_limit(2816));
        data.resistance_result = Some(resistance_result_fixture());
        data.encoder_domain = Some(EncoderDomain::new(4000).unwrap());
        let phase = InnerPhase::ElectricalIdentification {
            state: ElectricalIdentificationState::InductanceSample {
                capture: inductance_capture_state(0, 0, 0, 0),
            },
            data,
        };
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        let _ = backend.registers.insert(STATUS_FLAGS.addr(), 0);
        let _ = backend
            .registers
            .insert(foci_tmc4671::registers::ABN_DECODER_COUNT.addr(), 2000);
        let mut phi_samples = heapless::Vec::<u32, 128>::new();
        let mut current_samples = heapless::Vec::<u32, 128>::new();
        for index in 1..=100u32 {
            phi_samples
                .push(u32::from(fixed_window_phi(index, 100)))
                .ok();
            current_samples.push(sample).ok();
        }
        backend.set_openloop_phi_sequence(phi_samples.as_slice(), 0);
        backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());
        (phase, backend)
    }

    #[test]
    fn inductance_sample_at_the_envelope_passes_the_check() {
        pollster::block_on(async {
            let (mut phase, mut backend) = inductance_envelope_fixture(563u32 << 16);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = drive_inductance_schedule(
                &mut phase,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(!matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceCurrentEnvelope)
            ));
        });
    }

    #[test]
    fn inductance_sample_above_the_envelope_rejects_and_tears_down() {
        pollster::block_on(async {
            let (mut phase, mut backend) = inductance_envelope_fixture(564u32 << 16);
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let result = drive_inductance_schedule(
                &mut phase,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::InductanceCurrentEnvelope)
            ));
            assert_ac_teardown_after_failure(&backend);
        });
    }

    #[test]
    fn inductance_current_envelope_has_wire_code_76() {
        assert_eq!(CommissionError::InductanceCurrentEnvelope.status_code(), 76);
    }

    #[test]
    fn inductance_low_quadrature_rejection_maps_to_specific_error() {
        assert_eq!(
            InductanceReject::LowQuadratureSignal.commission_error(),
            CommissionError::InductanceLowQuadratureSignal
        );
    }

    #[test]
    fn resistance_runner_is_polled_with_bounded_spi_work() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            backend.script_resistance_current_proportional_to_ud = true;

            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::ResistanceStart,
                data: ElectricalIdentificationData::new(
                    u32::from(backend.tmc_config().excitation_voltage),
                    crate::commission_dispatch::test_support::current_limit(2816),
                ),
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let mut reached_inductance = false;
            for _ in 0..5_000 {
                let before = register_operation_count(&backend.spi_log);
                let step = step_electrical_identification(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                let operations = register_operation_count(&backend.spi_log) - before;
                assert!(
                    operations <= ELECTRICAL_ID_MAX_REGISTER_OPS_PER_ARM,
                    "resistance poll performed {operations} SPI ops"
                );

                if matches!(
                    phase,
                    InnerPhase::ElectricalIdentification {
                        state: ElectricalIdentificationState::InductanceSetup,
                        ..
                    }
                ) {
                    assert!(matches!(step, StepResult::ReadyNow));
                    reached_inductance = true;
                    break;
                }
                match step {
                    StepResult::ReadyNow => {}
                    StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                    other => panic!("unexpected resistance poll result: {other:?}"),
                }
            }
            assert!(reached_inductance, "resistance runner did not complete");
            let InnerPhase::ElectricalIdentification { data, .. } = &phase else {
                panic!("expected ElectricalIdentification, got {phase:?}");
            };
            assert!(
                data.resistance_result.is_some(),
                "resistance result should be stored on success"
            );
        });
    }

    #[test]
    fn resistance_critical_status_restores_before_terminal_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            backend.script_resistance_current_proportional_to_ud = true;

            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::ResistanceStart,
                data: ElectricalIdentificationData::new(
                    u32::from(backend.tmc_config().excitation_voltage),
                    crate::commission_dispatch::test_support::current_limit(2816),
                ),
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let critical_status = 1 << 1;
            let mut injected = false;
            let mut containment_seen = false;
            let mut restore_polls = 0;

            for _ in 0..200 {
                let before = register_operation_count(&backend.spi_log);
                let step = step_electrical_identification(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                let operations = register_operation_count(&backend.spi_log) - before;
                assert!(
                    operations <= ELECTRICAL_ID_MAX_REGISTER_OPS_PER_ARM,
                    "resistance poll performed {operations} SPI ops"
                );

                if injected && !backend.enable_pin && !containment_seen {
                    assert!(matches!(step, StepResult::ReadyNow));
                    let InnerPhase::ElectricalIdentification {
                        state: ElectricalIdentificationState::ResistanceIdentify { runner },
                        ..
                    } = &phase
                    else {
                        panic!("critical status failed before runner restoration: {phase:?}");
                    };
                    assert_eq!(runner.status_flags_or() & critical_status, critical_status);
                    assert_eq!(
                        runner.primary_error(),
                        Some(crate::resistance_identification::ResistanceError::CriticalStatus)
                    );
                    containment_seen = true;
                    continue;
                }

                if containment_seen && matches!(step, StepResult::ReadyNow) {
                    restore_polls += 1;
                }
                if let StepResult::InnerFailed(error) = step {
                    assert_eq!(error, CommissionError::ResistanceIdFailed);
                    assert!(containment_seen);
                    assert!(restore_polls > 0, "failure bypassed resumable restoration");
                    assert!(!backend.enable_pin);
                    return;
                }
                if let StepResult::At(deadline_us) = step {
                    backend.elapsed_us_value = deadline_us;
                }
                if backend.enable_pin && !injected {
                    backend.read_override = Some((STATUS_FLAGS.addr(), critical_status));
                    injected = true;
                }
            }
            panic!("critical resistance status did not reach terminal failure");
        });
    }

    #[test]
    fn resistance_cleanup_failure_retains_trip_diagnostic() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            backend.script_resistance_current_proportional_to_ud = true;
            backend.tmc_config.resistance_max_expected_current_count = 100;

            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::ResistanceStart,
                data: ElectricalIdentificationData::new(
                    u32::from(backend.tmc_config().excitation_voltage),
                    crate::commission_dispatch::test_support::current_limit(2816),
                ),
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let mut saw_enabled = false;
            let mut injected_cleanup_failure = false;

            for _ in 0..200 {
                let step = step_electrical_identification(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;

                saw_enabled |= backend.enable_pin;
                if saw_enabled && !backend.enable_pin && !injected_cleanup_failure {
                    backend.fail_on_write = Some(UQ_UD_EXT.addr());
                    injected_cleanup_failure = true;
                }

                if let StepResult::InnerFailed(error) = step {
                    assert_eq!(error, CommissionError::Spi);
                    let current = diagnostics
                        .iter()
                        .find(|event| event.code == 40)
                        .expect("resistance current diagnostic");
                    assert_eq!(current.value0, 512);
                    assert_eq!(current.value1, 0);
                    assert_eq!(current.value2, 100);
                    assert_eq!(current.status, 1);
                    return;
                }
                if let StepResult::At(deadline_us) = step {
                    backend.elapsed_us_value = deadline_us;
                }
            }
            panic!("resistance cleanup failure did not terminate");
        });
    }

    #[test]
    fn resistance_warning_status_is_retained_without_terminal_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);
            backend.script_resistance_current_proportional_to_ud = true;

            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::ResistanceStart,
                data: ElectricalIdentificationData::new(
                    u32::from(backend.tmc_config().excitation_voltage),
                    crate::commission_dispatch::test_support::current_limit(2816),
                ),
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();
            let warning_status = 1 << 28;
            let mut injected = false;

            for _ in 0..5_000 {
                let before = register_operation_count(&backend.spi_log);
                let step = step_electrical_identification(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                let operations = register_operation_count(&backend.spi_log) - before;
                assert!(
                    operations <= ELECTRICAL_ID_MAX_REGISTER_OPS_PER_ARM,
                    "resistance poll performed {operations} SPI ops"
                );

                if let InnerPhase::ElectricalIdentification {
                    state: ElectricalIdentificationState::InductanceSetup,
                    data,
                } = &phase
                {
                    assert!(matches!(step, StepResult::ReadyNow));
                    let result = data.resistance_result.expect("resistance result");
                    assert_eq!(result.status_flags_or & warning_status, warning_status);
                    assert!(
                        result
                            .warning_flags
                            .contains(ResistanceWarningFlags::STATUS_FLAGS)
                    );
                    return;
                }
                match step {
                    StepResult::ReadyNow => {}
                    StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                    other => panic!("warning-only status became terminal: {other:?}"),
                }
                if backend.enable_pin && !injected {
                    backend.read_override = Some((STATUS_FLAGS.addr(), warning_status));
                    injected = true;
                }
            }
            panic!("warning-only resistance run did not complete");
        });
    }

    #[test]
    fn resistance_reports_specific_quality_gate_failure_after_restoration() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            seed_resistance_power_stage_registers(&mut backend);

            let mut phase = InnerPhase::ElectricalIdentification {
                state: ElectricalIdentificationState::ResistanceStart,
                data: ElectricalIdentificationData::new(
                    u32::from(backend.tmc_config().excitation_voltage),
                    crate::commission_dispatch::test_support::current_limit(2816),
                ),
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            let expected_status =
                crate::resistance_identification::ResistanceError::MeasurementEnvelopeUnsupported
                    .status_code();
            for _ in 0..2_000 {
                let step = step_electrical_identification(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                match step {
                    StepResult::ReadyNow => {}
                    StepResult::At(deadline_us) => backend.elapsed_us_value = deadline_us,
                    StepResult::InnerFailed(error) => {
                        assert_eq!(error.status_code(), expected_status);
                        assert_ne!(
                            error.status_code(),
                            CommissionError::ResistanceIdFailed.status_code()
                        );
                        return;
                    }
                    other => panic!("unexpected resistance poll result: {other:?}"),
                }
            }
            panic!("resistance quality gate did not fail");
        });
    }

    #[test]
    fn shutdown_snapshot_reports_active_inductance_without_polling() {
        let mut inductance_capture = inductance_capture_state(0, 1_000, 0, 0);
        inductance_capture
            .timing
            .record_sample(1_000, 1_000, 1_010, 1_020, 10)
            .unwrap();
        let mut inductance_data = representative_data();
        inductance_data.timing_states =
            crate::commissioning::types::COMMISSION_TIMING_REJECTED << 2;
        let inductance_phase = InnerPhase::ElectricalIdentification {
            state: ElectricalIdentificationState::InductanceSample {
                capture: inductance_capture,
            },
            data: inductance_data,
        };
        let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

        push_active_electrical_diagnostics(&inductance_phase, &mut diagnostics);

        let header = diagnostics
            .iter()
            .find(|event| event.code == 36)
            .expect("active inductance header");
        assert_eq!(header.value0 >> 16, 1);
        assert_eq!((header.value1 >> 24) & 0x03, 2);
    }

    #[test]
    fn shutdown_snapshot_retains_completed_inductance_across_post_capture_states() {
        let capture = inductance_capture_state(0, 1_000, 0, 0);
        let timing = CaptureTimingEvidence {
            requested_period_us: INDUCTANCE_AC_SAMPLE_PERIOD_US,
            valid_samples: 7,
            missed_samples: 1,
            max_consecutive_misses: 1,
            max_lateness_us: 20,
            max_interval_us: 120,
            max_poll_wall_us: 15,
            max_spi_wall_us: 10,
        };
        let states = [
            ElectricalIdentificationState::InductanceReadStatus {
                capture,
                completed_elapsed_us: 700,
                timing,
            },
            ElectricalIdentificationState::InductanceReadEncoder {
                capture,
                completed_elapsed_us: 700,
                timing,
            },
            ElectricalIdentificationState::InductanceEvaluate {
                capture,
                encoder_after: 0,
                completed_elapsed_us: 700,
                timing,
            },
            ElectricalIdentificationState::InductanceRestore {
                operation: InductanceRestoreOperation::RetainedUqUdExt,
                primary_error: Some(CommissionError::InductanceCaptureRejected),
                evidence: None,
                timing: Some(timing),
                timing_accepted: false,
            },
        ];

        for state in states {
            let mut data = representative_data();
            data.timing_states = crate::commissioning::types::COMMISSION_TIMING_REJECTED << 2;
            data.timing_overflow_mask = 1 << 1;
            let phase = InnerPhase::ElectricalIdentification { state, data };
            let mut diagnostics = crate::selftest::CommissionDiagnostics::default();

            push_active_electrical_diagnostics(&phase, &mut diagnostics);

            let header = diagnostics
                .iter()
                .find(|event| event.code == 36)
                .expect("completed inductance timing header");
            assert_eq!(header.value0 >> 16, 7);
            assert_eq!((header.value1 >> 24) & 0x03, 2);
            assert_ne!(header.value1 & (1 << 31), 0);
            assert_eq!(header.value2, 20);
        }
    }

    fn data_with_per_coil_currents(c0: i32, c1: i32) -> ElectricalIdentificationData {
        let slope0 = (512_000 / c0.max(1)).max(1);
        let slope1 = (512_000 / c1.max(1)).max(1);
        let mut data = ElectricalIdentificationData::new(
            512,
            crate::commission_dispatch::test_support::current_limit(2816),
        );
        data.resistance_result = Some(resistance_result(slope0, slope1));
        data.inductance = Some(inductance_evidence_fixture(8_600, 1_000_000));
        data
    }

    #[test]
    fn model_failure_diagnostic_codes_are_stable() {
        let no_samples = diagnostic_for_model_assembly_failure(ModelAssemblyError::SampleCount);
        assert_eq!(
            no_samples.phase,
            CommissionPhase::ElectricalIdentification.to_u8()
        );
        assert_eq!(no_samples.code, 20);
        assert_eq!(no_samples.status, 1);

        let scale_zero =
            diagnostic_for_model_assembly_failure(ModelAssemblyError::ModelScaleZero {
                l_count_micro: 0,
            });
        assert_eq!(scale_zero.code, 22);

        let theta = diagnostic_for_model_assembly_failure(ModelAssemblyError::ThetaTooLarge {
            theta_us: 320,
            tau_us: 600,
        });
        assert_eq!(theta.code, 29);
        assert_eq!(theta.value0, 320);
        assert_eq!(theta.value1, 600);
    }

    #[test]
    fn permille_diff_uses_average_denominator_for_coil_disagreement() {
        assert_eq!(permille_diff(100, 130), 260);
    }

    #[test]
    fn assemble_accepts_representative_data() {
        let data = representative_data();
        let assembly = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap();
        assert_eq!(
            assembly.warning_flags & INNER_WARN_BIT_RESISTANCE_MISMATCH,
            0
        );
        assert_eq!(assembly.warning_flags & INNER_WARN_BIT_THETA_TAU, 0);
    }

    #[test]
    fn assemble_rejects_missing_inductance_evidence() {
        let mut data = representative_data();
        data.inductance = None;
        let err = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap_err();
        assert!(matches!(err, ModelAssemblyError::SampleCount));
    }

    #[test]
    fn model_assembly_errors_map_to_specific_commission_errors() {
        assert_eq!(
            CommissionError::from(ModelAssemblyError::SampleCount),
            CommissionError::ElectricalModelNoSamples
        );
        assert_eq!(
            CommissionError::from(ModelAssemblyError::ModelScaleZero { l_count_micro: 0 }),
            CommissionError::ElectricalModelScaleZero
        );
        assert_eq!(
            CommissionError::from(ModelAssemblyError::ThetaTooLarge {
                theta_us: 120,
                tau_us: 200,
            }),
            CommissionError::ElectricalThetaTooLarge
        );
    }

    #[test]
    fn assemble_rejects_zero_l_count_micro_with_model_scale_zero() {
        let mut data = ElectricalIdentificationData::new(
            512,
            crate::commission_dispatch::test_support::current_limit(2816),
        );
        data.resistance_result = Some(resistance_result(100, 100));
        data.inductance = Some(inductance_evidence_fixture(1, 1_000_000));
        let err = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap_err();
        assert!(matches!(
            err,
            ModelAssemblyError::ModelScaleZero { l_count_micro: 0 }
        ));
    }

    #[test]
    fn assemble_warns_on_borderline_resistance_mismatch() {
        let data = data_with_per_coil_currents(300, 320);
        let assembly = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap();
        assert_ne!(
            assembly.warning_flags & INNER_WARN_BIT_RESISTANCE_MISMATCH,
            0
        );
    }

    fn with_recorded_phase_evidence(
        mut data: ElectricalIdentificationData,
    ) -> ElectricalIdentificationData {
        let inductance = data
            .inductance
            .as_mut()
            .expect("representative data carries inductance evidence");
        inductance.frame.id_mean_milli_count = -10_657;
        inductance.frame.iq_mean_milli_count = -52_615;
        inductance.frame.id_rms_milli_count = 11_909;
        inductance.frame.iq_rms_milli_count = 11_265;
        inductance.run.sample_count = 100;
        inductance.run.realized_frequency_millihz = 1_000_000;
        if let Some(result) = data.resistance_result.as_mut() {
            result.selected_r_count_slope_milli = 938;
        }
        data
    }

    #[test]
    fn assemble_uses_the_phase_delay_when_the_run_lags() {
        let data = with_recorded_phase_evidence(representative_data());
        let assembly = assemble_electrical_model(&data, 160).unwrap();
        assert_eq!(assembly.model.theta_e_us, 42);
        assert_eq!(assembly.model.theta_source, ThetaSource::Measured as u8);
    }

    #[test]
    fn assemble_falls_back_when_the_current_leads() {
        let mut data = with_recorded_phase_evidence(representative_data());
        data.inductance.as_mut().unwrap().frame.iq_mean_milli_count = 52_615;
        let assembly = assemble_electrical_model(&data, 160).unwrap();
        assert_eq!(assembly.model.theta_e_us, 160);
        assert_eq!(
            assembly.model.theta_source,
            ThetaSource::ConfigFallback as u8
        );
    }

    #[test]
    fn phase_delay_from_evidence_rejects_without_inductance() {
        let mut data = with_recorded_phase_evidence(representative_data());
        data.inductance = None;
        assert_eq!(
            phase_delay_from_evidence(&data),
            Err(PhaseDelayRejection::NoSamples)
        );
    }

    #[test]
    fn validate_uses_measured_theta_when_available_and_accepted() {
        let data = with_recorded_phase_evidence(representative_data());
        let assembly = assemble_electrical_model(&data, TEST_THETA_E_US).unwrap();
        assert_eq!(assembly.model.theta_source, 0);
        assert_eq!(assembly.model.theta_e_us, 42);
    }

    #[test]
    fn resistance_profile_ceiling_binds_to_the_frozen_limit() {
        use crate::resistance_identification::ResistanceBackend;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;

        let mut backend = MockBackend::new();
        let adapter = CommissionResistanceBackend {
            backend: &mut backend,
            current_limit: PidTorqueFluxLimit::new(2816).unwrap(),
        };
        assert_eq!(adapter.profile().max_expected_current_count, 1600);
    }

    #[test]
    fn resistance_profile_ceiling_binds_to_a_low_run_current() {
        use crate::resistance_identification::ResistanceBackend;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;

        let mut backend = MockBackend::new();
        let adapter = CommissionResistanceBackend {
            backend: &mut backend,
            current_limit: PidTorqueFluxLimit::new(367).unwrap(),
        };
        assert_eq!(adapter.profile().max_expected_current_count, 220);
    }

    #[test]
    fn resistance_adapter_derives_the_bus_sense_from_the_captured_offset() {
        use crate::resistance_identification::ResistanceBackend;

        let mut without_offset = MockBackend::new();
        let adapter = CommissionResistanceBackend {
            backend: &mut without_offset,
            current_limit: current_limit(2000),
        };
        assert_eq!(adapter.bus_voltage_sense(), None);

        let mut with_offset = MockBackend::new();
        with_offset.adc_vm_offset_value = Some(crate::state::tmc::AdcVmOffsetSnapshot {
            offset_raw: 33_671,
            sample_count: 8,
        });
        let adapter = CommissionResistanceBackend {
            backend: &mut with_offset,
            current_limit: current_limit(2000),
        };
        let sense = adapter.bus_voltage_sense().expect("offset captured");
        assert_eq!(sense.offset_raw, 33_671);
        assert_eq!(sense.counts_per_volt_milli, 269_318);
    }

    #[test]
    fn resolve_theta_prefers_measured_when_under_tau_half() {
        let (theta, source) = resolve_theta_e_us_input(Some(400), 1_000, 900);
        assert_eq!(theta, 400);
        assert_eq!(source, ThetaSource::Measured);
    }

    #[test]
    fn resolve_theta_falls_back_when_measurement_absent() {
        let (theta, source) = resolve_theta_e_us_input(None, 1_000, 300);
        assert_eq!(theta, 300);
        assert_eq!(source, ThetaSource::ConfigFallback);
    }

    #[test]
    fn resolve_theta_falls_back_when_measured_theta_is_zero() {
        let (theta, source) = resolve_theta_e_us_input(Some(0), 1_000, 160);
        assert_eq!(theta, 160);
        assert_eq!(source, ThetaSource::ConfigFallback);
    }

    #[test]
    fn resolve_theta_falls_back_when_measured_theta_too_large() {
        let (theta, source) = resolve_theta_e_us_input(Some(500), 1_000, 300);
        assert_eq!(theta, 300);
        assert_eq!(source, ThetaSource::ConfigFallback);
    }

    #[test]
    fn resolve_theta_fallback_can_still_be_rejected_by_assemble_electrical_model() {
        let (theta, source) = resolve_theta_e_us_input(None, 1_000, 600);
        assert_eq!(theta, 600);
        assert_eq!(source, ThetaSource::ConfigFallback);
        assert!(theta >= 1_000 / 2);
    }
}
