//! Current-loop commissioning: phase enum, dispatcher, and per-phase step submodules.

use crate::autotune::{
    ClosedLoopActivationState, CommissionError, CommissionProfile, ElectricalModel, InnerSequence,
    TransientBuffer,
};
#[cfg(test)]
use crate::commissioning::OuterGainRollbackSnapshot;
use crate::commissioning::encoder_domain::EncoderCount;
use crate::commissioning::inner::current_loop::{CurrentValidateData, CurrentValidateState};
use crate::commissioning::types::{
    AdcCalResult, CommissioningBackend, InductanceEvidence, InnerResult, StepResult,
};
#[cfg(test)]
use crate::tmc_register_values::{MOTION_POSITION_ADVANCED_PI, MOTION_STOPPED};
#[cfg(test)]
use foci_tmc4671::pid::PidGainPair;
use foci_tmc4671::pid::{PidIGain, PidPGain};

pub mod adc;
pub mod closed_loop_activation;
pub mod coil;
pub mod current_loop;
pub mod electrical_identification;
pub mod encoder;
mod gain_floor;
pub mod phase;

#[derive(Debug, defmt::Format)]
pub(crate) enum AdcCalState {
    Setup,
    Settle {
        deadline_us: u64,
    },
    Sample {
        sum_i0: i32,
        sum_i1: i32,
        count: u16,
        next_sample_us: u64,
    },
    Finalize {
        sum_i0: i32,
        sum_i1: i32,
    },
    ResidualSettle {
        stage: AdcResidualStage,
        offset_i0: u16,
        offset_i1: u16,
        chop0: Option<crate::commissioning::types::AdcResidualEvidence>,
        deadline_us: u64,
    },
    ResidualSample {
        stage: AdcResidualStage,
        offset_i0: u16,
        offset_i1: u16,
        adc_i0_scale_offset: u32,
        adc_i1_scale_offset: u32,
        pwm_sv_chop: u8,
        pwm_bbm: u32,
        adc_iux_sum: i32,
        adc_iwy_sum: i32,
        pid_flux_sum: i32,
        pid_torque_sum: i32,
        count: u16,
        chop0: Option<crate::commissioning::types::AdcResidualEvidence>,
        next_sample_us: u64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum AdcResidualStage {
    ChopOff,
    ChopOn,
}

#[derive(Debug, defmt::Format)]
pub(crate) enum CoilTestState {
    Setup {
        phase: u8,
    },
    Settle {
        phase: u8,
        deadline_us: u64,
        /// Encoder count at the previous stillness poll.
        last_count: Option<u32>,
        /// Read anyway once the rotor has not settled by this time.
        give_up_us: u64,
    },
    Read {
        phase: u8,
    },
    Cooldown {
        phase: u8,
        deadline_us: u64,
    },
}

#[derive(Debug, defmt::Format)]
pub(crate) enum CoilCheckState {
    CoilA(CoilTestState),
    CoilB(CoilTestState),
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct CoilObservation {
    pub(crate) expected: i16,
    pub(crate) other: i16,
}

#[derive(Debug, defmt::Format)]
pub(crate) enum EncoderCheckState {
    Setup,
    Settle {
        deadline_us: u64,
        attempt: u8,
    },
    Sweep {
        step: u16,
        deadline_us: u64,
        count_start: EncoderCount,
        attempt: u8,
    },
    Finalize {
        count_start: EncoderCount,
        attempt: u8,
    },
}

#[derive(Debug, defmt::Format)]
pub(crate) enum EncoderAlignState {
    Setup {
        count_before: EncoderCount,
    },
    /// Two-sided alignment: hold the field off zero so the rotor reaches zero
    /// from that side. `first_rest` is the rest position of the first side.
    Approach {
        deadline_us: u64,
        first_rest: Option<EncoderCount>,
    },
    Settle {
        count_before: EncoderCount,
        deadline_us: u64,
        min_movement: u32,
        first_rest: Option<EncoderCount>,
    },
    StabilityCheck1 {
        count_before: EncoderCount,
        min_movement: u32,
        first_rest: Option<EncoderCount>,
    },
    StabilityCheck2 {
        count_before: EncoderCount,
        count1: EncoderCount,
        deadline_us: u64,
        min_movement: u32,
        /// Consecutive stable pairs seen so far.
        stable_pairs: u8,
        /// Fail if the rotor has not held still by this time.
        give_up_us: u64,
        first_rest: Option<EncoderCount>,
    },
    Commit {
        old_count: EncoderCount,
        /// `ABN_DECODER_COUNT` written at the rest position: 0, or the offset
        /// that puts the midpoint of a two-sided alignment at 0.
        commit_count: u32,
        electrical_residual_counts: u16,
        stability_counts: u16,
        movement_counts: u16,
        min_movement_counts: u16,
        counts_per_electrical_rev: u16,
    },
    StopDrive {
        old_count: EncoderCount,
        electrical_residual_counts: u16,
        stability_counts: u16,
        movement_counts: u16,
        min_movement_counts: u16,
        counts_per_electrical_rev: u16,
    },
}

#[derive(Debug, Clone, defmt::Format)]
pub(crate) struct ElectricalIdentificationData {
    pub(crate) voltage: u32,
    pub(crate) current_limit: foci_tmc4671::current_limit::PidTorqueFluxLimit,
    pub(crate) resistance_result: Option<crate::resistance_identification::ResistanceRunResult>,
    pub(crate) resistance_timing:
        Option<crate::commissioning::capture_timing::CaptureTimingEvidence>,
    pub(crate) inductance: Option<InductanceEvidence>,
    pub(crate) inductance_timing:
        Option<crate::commissioning::capture_timing::CaptureTimingEvidence>,
    pub(crate) timing_states: u8,
    pub(crate) timing_overflow_mask: u8,
    pub(crate) encoder_domain: Option<crate::commissioning::encoder_domain::EncoderDomain>,
    /// `UQ_UD_EXT` drive of the inductance capture.
    pub(crate) inductance_ud: i16,
    /// Open-loop target giving a 1 kHz electrical rotation for the motor's
    /// pole pairs.
    pub(crate) inductance_openloop_rpm: u32,
}

impl ElectricalIdentificationData {
    pub(crate) fn new(
        voltage: u32,
        current_limit: foci_tmc4671::current_limit::PidTorqueFluxLimit,
    ) -> Self {
        Self {
            voltage,
            current_limit,
            resistance_result: None,
            resistance_timing: None,
            inductance: None,
            inductance_timing: None,
            timing_states: 0,
            timing_overflow_mask: 0,
            encoder_domain: None,
            inductance_ud: electrical_identification::INDUCTANCE_AC_UD_EXT,
            inductance_openloop_rpm:
                electrical_identification::INDUCTANCE_OPENLOOP_VELOCITY_TARGET_RPM,
        }
    }

    /// Size the inductance capture for the motor: drive `inductance_voltage`
    /// and rotate at 1 kHz electrical for `pole_pairs`.
    pub(crate) fn with_motor(mut self, inductance_voltage: u16, pole_pairs: u16) -> Self {
        self.inductance_ud = i16::try_from(inductance_voltage).unwrap_or(i16::MAX);
        self.inductance_openloop_rpm =
            electrical_identification::inductance_openloop_rpm(pole_pairs);
        self
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub(crate) struct InductanceCaptureState {
    pub(crate) encoder_domain: crate::commissioning::encoder_domain::EncoderDomain,
    pub(crate) encoder_start: u32,
    pub(crate) start_us: u64,
    pub(crate) previous_elapsed_us: u32,
    pub(crate) previous_phi: u16,
    pub(crate) accumulated_phi_delta_counts: u32,
    pub(crate) accumulator: crate::electrical_capture::AcFrameAccumulator,
    pub(crate) first_half: crate::electrical_capture::AcFrameAccumulator,
    pub(crate) second_half: crate::electrical_capture::AcFrameAccumulator,
    pub(crate) saliency: crate::electrical_capture::AcSaliencyAccumulator,
    pub(crate) sample_count: u16,
    pub(crate) status_flags_or: u32,
    pub(crate) next_sample_us: u64,
    pub(crate) timing: crate::commissioning::capture_timing::CaptureTiming,
    pub(crate) current_envelope: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum InductanceRestoreOperation {
    RetainedUqUdExt,
    RetainedOpenloopVelocity,
    RetainedMotionStop,
    RetainedPhiSelection,
    CanonicalMotionStop,
    CanonicalTorqueFluxTarget,
    CanonicalTorqueFluxOffset,
    CanonicalUqUdExt,
    CanonicalPhiSelection,
    CanonicalPwmChop,
    CanonicalVelocityOffset,
    CanonicalVoltageLimit,
    TorqueErrorSelect,
    TorqueErrorClear,
    FluxErrorSelect,
    FluxErrorClear,
    VelocityErrorSelect,
    VelocityErrorClear,
    PositionErrorSelect,
    PositionErrorClear,
}

#[derive(Debug, defmt::Format)]
#[allow(
    clippy::large_enum_variant,
    reason = "no heap; capture state uses fixed-size accumulators and timestamp storage"
)]
pub(crate) enum ElectricalIdentificationState {
    ResistanceStart,
    ResistanceIdentify {
        runner: crate::resistance_identification::ResistanceRunner,
    },

    InductanceSetup,
    InductanceSetupOperation {
        operation: u8,
    },
    InductanceWaitVelocity {
        deadline_us: u64,
    },
    InductanceReadEncoderStart,
    InductanceApplyDrive {
        encoder_start: u32,
    },
    InductanceDriveSettle {
        deadline_us: u64,
        encoder_start: u32,
        encoder_domain: crate::commissioning::encoder_domain::EncoderDomain,
    },
    InductanceReadPreStatus {
        encoder_start: u32,
        encoder_domain: crate::commissioning::encoder_domain::EncoderDomain,
    },
    InductanceReadStartPhi {
        encoder_start: u32,
        encoder_domain: crate::commissioning::encoder_domain::EncoderDomain,
        status_flags_or: u32,
    },
    InductanceSample {
        capture: InductanceCaptureState,
    },
    InductanceReadStatus {
        capture: InductanceCaptureState,
        completed_elapsed_us: u32,
        timing: crate::commissioning::capture_timing::CaptureTimingEvidence,
    },
    InductanceReadEncoder {
        capture: InductanceCaptureState,
        completed_elapsed_us: u32,
        timing: crate::commissioning::capture_timing::CaptureTimingEvidence,
    },
    InductanceEvaluate {
        capture: InductanceCaptureState,
        encoder_after: u32,
        completed_elapsed_us: u32,
        timing: crate::commissioning::capture_timing::CaptureTimingEvidence,
    },
    InductanceRestore {
        operation: InductanceRestoreOperation,
        primary_error: Option<crate::autotune::CommissionError>,
        evidence: Option<InductanceEvidence>,
        timing: Option<crate::commissioning::capture_timing::CaptureTimingEvidence>,
        timing_accepted: bool,
    },
    InductanceDisable {
        deadline_us: u64,
    },

    Validate,
}

#[allow(clippy::large_enum_variant)]
#[derive(Debug, defmt::Format)]
pub(crate) enum InnerPhase {
    AdcCalibrate(AdcCalState),
    CoilCheck {
        state: CoilCheckState,
        coil_a: Option<CoilObservation>,
    },
    PhaseWiringCheck {
        coil_a: CoilObservation,
        coil_b: CoilObservation,
    },
    EncoderCheck {
        state: EncoderCheckState,
    },
    EncoderAlign {
        state: EncoderAlignState,
    },
    ElectricalIdentification {
        state: ElectricalIdentificationState,
        data: ElectricalIdentificationData,
    },
    CurrentTune {
        model: ElectricalModel,
        inner_warning_flags: u8,
        resistance_result: crate::resistance_identification::ResistanceRunResult,
        inductance: InductanceEvidence,
        timing: [crate::commissioning::types::CommissionTimingSummary; 3],
    },
    CurrentValidate {
        state: CurrentValidateState,
        data: CurrentValidateData,
    },
    GainFloorMeasurement {
        state: gain_floor::GainFloorState,
        i_targets: [i16; 3],
        resistance_current_ceiling_count: u16,
        inner_result: Option<InnerResult>,
        capture: gain_floor::capture::CaptureBuffer,
    },
    ClosedLoopActivation {
        state: ClosedLoopActivationState,
        flux_p: PidPGain,
        flux_i: PidIGain,
        torque_p: PidPGain,
        torque_i: PidIGain,
        velocity_p: PidPGain,
        velocity_i: PidIGain,
        position_p: PidPGain,
        position_i: PidIGain,
        velocity_limit: u32,
        velocity_filter_hz: u16,
        torque_filter_hz: u16,
        position_filter_hz: u16,
        flux_filter_hz: u16,
        inner_result: Option<InnerResult>,
        attempt: u8,
    },
    Failed(CommissionError),
}

impl InnerPhase {
    pub(crate) fn current_phase_id(&self) -> Option<crate::autotune::CommissionPhase> {
        use crate::autotune::CommissionPhase;

        match self {
            Self::AdcCalibrate(_) => Some(CommissionPhase::AdcCalibrate),
            Self::CoilCheck { .. } => Some(CommissionPhase::CoilCheck),
            Self::PhaseWiringCheck { .. } => Some(CommissionPhase::PhaseWiringCheck),
            Self::EncoderCheck { .. } => Some(CommissionPhase::EncoderCheck),
            Self::EncoderAlign { .. } => Some(CommissionPhase::EncoderAlign),
            Self::ElectricalIdentification { .. } => {
                Some(CommissionPhase::ElectricalIdentification)
            }
            Self::CurrentTune { .. } => Some(CommissionPhase::CurrentTune),
            Self::CurrentValidate { .. } => Some(CommissionPhase::CurrentValidate),
            Self::GainFloorMeasurement { .. } => Some(CommissionPhase::GainFloorMeasurement),
            Self::ClosedLoopActivation { .. } => Some(CommissionPhase::ClosedLoopActivation),
            Self::Failed(_) => None,
        }
    }
}

pub(crate) fn fail_inner(phase: &mut InnerPhase, err: CommissionError) -> StepResult {
    *phase = InnerPhase::Failed(err);
    StepResult::InnerFailed(err)
}

/// Mutable state and inputs the inner phases share for one step.
pub(crate) struct InnerStepContext<'a> {
    pub(crate) buf: &'a mut TransientBuffer,
    pub(crate) adc_cal: &'a mut Option<AdcCalResult>,
    pub(crate) calibrate_entry_gains: Option<crate::commissioning::CalibrateEntryGains>,
    pub(crate) selftest_results: &'a mut crate::selftest::SelftestResults,
    pub(crate) diagnostics: &'a mut crate::selftest::CommissionDiagnostics,
}

pub(crate) async fn step_inner<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    profile: CommissionProfile,
    sequence: InnerSequence,
    context: InnerStepContext<'_>,
    backend: &mut B,
) -> StepResult {
    let result = step_inner_dispatch(phase, profile, sequence, context, backend).await;
    if matches!(result, StepResult::InnerFailed(_)) {
        crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
    }
    result
}

async fn step_inner_dispatch<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    profile: CommissionProfile,
    sequence: InnerSequence,
    context: InnerStepContext<'_>,
    backend: &mut B,
) -> StepResult {
    let InnerStepContext {
        buf,
        adc_cal,
        calibrate_entry_gains,
        selftest_results,
        diagnostics,
    } = context;
    match phase {
        InnerPhase::AdcCalibrate(_) => {
            adc::step_adc_calibrate(phase, sequence, adc_cal, selftest_results, backend).await
        }
        InnerPhase::CoilCheck { .. } => {
            coil::step_coil_check(phase, sequence, selftest_results, diagnostics, backend).await
        }
        InnerPhase::PhaseWiringCheck { coil_a, coil_b } => {
            let a = *coil_a;
            let b = *coil_b;
            phase::step_phase_wiring(phase, sequence, selftest_results, a, b)
        }
        InnerPhase::EncoderCheck { .. } => {
            encoder::step_encoder_check(phase, sequence, selftest_results, diagnostics, backend)
                .await
        }
        InnerPhase::EncoderAlign { .. } => {
            encoder::step_encoder_align(
                phase,
                sequence,
                adc_cal,
                calibrate_entry_gains,
                selftest_results,
                diagnostics,
                backend,
            )
            .await
        }
        InnerPhase::ElectricalIdentification { .. } => {
            electrical_identification::step_electrical_identification(
                phase,
                sequence,
                selftest_results,
                diagnostics,
                backend,
            )
            .await
        }
        InnerPhase::CurrentTune { .. } => {
            current_loop::step_current_tune(phase, profile, backend).await
        }
        InnerPhase::CurrentValidate { .. } => {
            current_loop::step_current_validate(phase, buf, profile, sequence, adc_cal, backend)
                .await
        }
        InnerPhase::GainFloorMeasurement { .. } => {
            gain_floor::step_gain_floor_measurement(phase, backend).await
        }
        InnerPhase::ClosedLoopActivation { .. } => {
            closed_loop_activation::step_closed_loop_activation(
                phase,
                sequence,
                adc_cal,
                diagnostics,
                backend,
            )
            .await
        }
        InnerPhase::Failed(err) => StepResult::InnerFailed(*err),
    }
}

#[cfg(test)]
pub(crate) async fn restore_entry_gains<B: CommissioningBackend>(
    snapshot: &OuterGainRollbackSnapshot,
    backend: &mut B,
    tmc_state: &crate::state::tmc::TmcChannelState,
) -> Result<(), CommissionError> {
    use foci_tmc4671::registers::*;

    backend
        .write_register(PID_TORQUE_FLUX_TARGET, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_VELOCITY_TARGET, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_VELOCITY_OFFSET, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;

    backend
        .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
        .await
        .map_err(|_| CommissionError::Spi)?;

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

    crate::tmc_control::interlock::clear_status_flags_via_backend(backend).await?;

    crate::tmc_control::pi_representation::verify_pi_representation(backend, tmc_state)
        .await
        .map_err(|_| CommissionError::ChipResetDetected)?;
    let velocity_reg = PidGainPair::new(snapshot.velocity_p, snapshot.velocity_i).register_value();
    backend
        .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    let position_reg = PidGainPair::new(snapshot.position_p, snapshot.position_i).register_value();
    backend
        .write_register(PID_POSITION_P_POSITION_I, position_reg)
        .await
        .map_err(|_| CommissionError::Spi)?;
    backend
        .write_register(PID_VELOCITY_LIMIT, snapshot.velocity_limit)
        .await
        .map_err(|_| CommissionError::Spi)?;

    let pwm_maxcnt = backend.pwm_maxcnt();
    if snapshot.velocity_filter_hz > 0 {
        crate::commissioning::helpers::write_biquad_velocity(
            backend,
            snapshot.velocity_filter_hz,
            pwm_maxcnt,
        )
        .await;
    } else {
        crate::commissioning::helpers::write_sub_register(backend, CONFIG_BIQUAD_V_ENABLE, 0).await;
    }
    if snapshot.torque_filter_hz > 0 {
        crate::commissioning::helpers::write_biquad_torque(
            backend,
            snapshot.torque_filter_hz,
            pwm_maxcnt,
        )
        .await;
    } else {
        crate::commissioning::helpers::write_sub_register(backend, CONFIG_BIQUAD_T_ENABLE, 0).await;
    }
    if snapshot.position_filter_hz > 0 {
        crate::commissioning::helpers::write_biquad_position(
            backend,
            snapshot.position_filter_hz,
            pwm_maxcnt,
        )
        .await;
    } else {
        crate::commissioning::helpers::write_sub_register(backend, CONFIG_BIQUAD_X_ENABLE, 0).await;
    }
    if snapshot.flux_filter_hz > 0 {
        crate::commissioning::helpers::write_biquad_flux(
            backend,
            snapshot.flux_filter_hz,
            pwm_maxcnt,
        )
        .await;
    } else {
        crate::commissioning::helpers::write_sub_register(backend, CONFIG_BIQUAD_F_ENABLE, 0).await;
    }

    backend
        .write_register(PIDOUT_UQ_UD_LIMITS, snapshot.voltage_limit)
        .await
        .map_err(|_| CommissionError::Spi)?;

    backend
        .write_register(PID_POSITION_ACTUAL, 0)
        .await
        .map_err(|_| CommissionError::Spi)?;

    backend
        .write_register(MODE_RAMP_MODE_MOTION, MOTION_POSITION_ADVANCED_PI)
        .await
        .map_err(|_| CommissionError::Spi)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{AdcCalState, InnerPhase};
    use crate::autotune::{CommissionError, CommissionPhase};

    #[test]
    fn inner_phase_id_maps_active_and_failed_phases() {
        assert_eq!(
            InnerPhase::AdcCalibrate(AdcCalState::Setup).current_phase_id(),
            Some(CommissionPhase::AdcCalibrate)
        );
        assert_eq!(
            InnerPhase::Failed(CommissionError::Spi).current_phase_id(),
            None
        );
    }
}
