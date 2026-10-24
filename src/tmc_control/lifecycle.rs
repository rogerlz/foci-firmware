//! Shared motor lifecycle helpers for terminal TMC control paths.

use crate::commission_dispatch::DispatchState;
use crate::commissioning::types::CommissioningBackend;
use crate::tmc_register_values::MOTION_POSITION_ADVANCED_PI;

/// A motor lifecycle flag transitioned by [`arm_motor_electrical`],
/// [`arm_motor_for_steps`], or [`disarm_motor`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum LifecycleFlag {
    /// `DispatchState::motor_enabled` (electrical arm/disarm).
    MotorEnabled,
    /// `TmcChannelState::motor_armed` (step-timer processing arm/disarm).
    MotorArmed,
    /// `DispatchState::calibrated`.
    Calibrated,
}

pub(crate) async fn settle_closed_loop_hold_before_enable<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), crate::autotune::CommissionError> {
    let mut step = ClosedLoopHoldSettleStep::start();
    loop {
        if let Some(result) = step.advance(backend).await {
            return result;
        }
    }
}

/// Mark the motor electrically energized after verified calibration.
///
/// Drains pending configuration, verifies the calibration sentinel, zeroes
/// `PID_TORQUE_FLUX_OFFSET`, rewrites `PID_TORQUE_FLUX_LIMITS` with the operational limit,
/// then raises the enable pin and
/// sets `state.motor_enabled`. Does not arm stepping; see [`arm_motor_for_steps`].
///
/// # Panics
///
/// Panics if `state.calibrated` is false.
///
/// # Errors
///
/// Returns [`crate::autotune::CommissionError::SafeStateIncomplete`] if the
/// most recent safe-state restore pass had unresolved write failures.
/// Returns [`crate::autotune::CommissionError::ChipResetDetected`] if the
/// sentinel is inactive or does not match the TMC4671 readback.
/// Returns [`crate::autotune::CommissionError::InvalidCurrentLimit`] if the
/// stored run-current limit is zero (unconfigured).
/// Returns [`crate::autotune::CommissionError::Spi`] if the pending-config
/// drain fails (the failed item is named in the log), if the torque-flux
/// offset clear or limit rewrite fails, or if configuration was recorded after the drain.
/// Returns [`crate::autotune::CommissionError::FaultInterlock`] if the fault
/// interlock is tripped when the enable pin would be raised.
pub async fn arm_motor_electrical<B: CommissioningBackend>(
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> Result<(), crate::autotune::CommissionError> {
    let mut step = ArmMotorElectricalStep::start();
    loop {
        if let Some(result) = step.advance(backend, state).await {
            return result;
        }
    }
}

/// Arm the stepper for step-timer processing.
///
/// Asserts `state.motor_enabled` (set only by `arm_motor_electrical`);
/// stores the channel motor-armed flag. Caller is responsible for any
/// prerequisite state (e.g. zeroing the stepper channel position) before
/// invoking. Always-on assert.
pub fn arm_motor_for_steps(state: &mut DispatchState<'_>) {
    assert!(
        *state.motor_enabled,
        "FATAL: arm_motor_for_steps called with motor_enabled=false"
    );
    state
        .tmc
        .motor_armed
        .store(true, core::sync::atomic::Ordering::Relaxed);
}

/// De-energize the motor and clear all armed/enabled state.
///
/// Runs the safe-state restore, then clears the motor-armed, calibrated, and enabled state and
/// the velocity feedforward clamp. Idempotent. Stopping the stepper queue stays board-side.
pub async fn disarm_motor<B: CommissioningBackend>(backend: &mut B, state: &mut DispatchState<'_>) {
    state
        .tmc
        .motor_armed
        .store(false, core::sync::atomic::Ordering::Relaxed);
    backend.record_lifecycle(LifecycleFlag::MotorArmed, false);
    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
    state.tmc.clear_velocity_clamp_latch();
    *state.calibrated = false;
    *state.motor_enabled = false;
    backend.record_lifecycle(LifecycleFlag::MotorEnabled, false);
}

/// Apply the shared shutdown safe-state policy for one TMC channel.
///
/// Clears runtime-owned homing state, de-energizes via [`disarm_motor`], then stops any active
/// commissioning engine; a retained breakaway campaign completes after the disarm.
pub async fn apply_shutdown_safe_state<B: CommissioningBackend>(
    runtime: &mut crate::tmc_control::TmcControlRuntime,
    backend: &mut B,
    tmc: &'static crate::state::tmc::TmcChannelState,
    stepper: &'static crate::state::stepper::StepperChannelState,
) {
    crate::commissioning::outer::position_tune::abort_stimulus(backend);
    if runtime.is_restore_pending() {
        runtime.request_restore_cancel();
    }
    while runtime.restore_cancel_pending() {
        let _ = runtime
            .drive_restore_cancel(
                backend,
                tmc,
                stepper,
                crate::autotune::CommissionError::Shutdown,
            )
            .await;
    }
    runtime.clear_homing_runtime();
    tmc.homing_clamp_active
        .store(false, core::sync::atomic::Ordering::Relaxed);
    tmc.stall_latched
        .store(false, core::sync::atomic::Ordering::Relaxed);
    let mut state = runtime.dispatch_state(tmc, stepper);
    disarm_motor(backend, &mut state).await;
    runtime.take_commission_engine_after_handled_stop();
}

#[derive(Clone, Copy)]
pub(crate) enum ClosedLoopHoldSettleStep {
    CheckMode,
    ReadPositionActual,
    WritePositionActual {
        actual: u32,
    },
    ClearIntegrators {
        ops: [(foci_tmc4671::registers::Register, u32); 8],
        next: u8,
    },
    ClearStatusFlags,
}

impl ClosedLoopHoldSettleStep {
    pub(crate) const fn start() -> Self {
        Self::CheckMode
    }

    pub(crate) async fn advance<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
    ) -> Option<Result<(), crate::autotune::CommissionError>> {
        use crate::autotune::CommissionError;

        match self {
            Self::CheckMode => {
                let mode = match backend
                    .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                    .await
                {
                    Ok(mode) => mode,
                    Err(_) => return Some(Err(CommissionError::Spi)),
                };
                if mode != MOTION_POSITION_ADVANCED_PI {
                    return Some(Ok(()));
                }
                *self = Self::ReadPositionActual;
                None
            }
            Self::ReadPositionActual => {
                match backend
                    .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                    .await
                {
                    Ok(actual) => {
                        *self = Self::WritePositionActual { actual };
                        None
                    }
                    Err(_) => Some(Err(CommissionError::Spi)),
                }
            }
            Self::WritePositionActual { actual } => {
                let actual = *actual;
                if backend
                    .write_register(foci_tmc4671::registers::PID_POSITION_ACTUAL, actual)
                    .await
                    .is_err()
                {
                    return Some(Err(CommissionError::Spi));
                }
                use foci_tmc4671::registers::{
                    PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM,
                    PID_ERROR_PID_TORQUE_ERROR_SUM, PID_ERROR_PID_VELOCITY_ERROR_SUM, Register,
                };
                let mut ops = [(Register::new(0), 0u32); 8];
                for (i, sub) in [
                    PID_ERROR_PID_TORQUE_ERROR_SUM,
                    PID_ERROR_PID_FLUX_ERROR_SUM,
                    PID_ERROR_PID_VELOCITY_ERROR_SUM,
                    PID_ERROR_PID_POSITION_ERROR_SUM,
                ]
                .into_iter()
                .enumerate()
                {
                    ops[i * 2] = (sub.addr, u32::from(sub.sub_addr));
                    ops[i * 2 + 1] = (sub.data, 0);
                }
                *self = Self::ClearIntegrators { ops, next: 0 };
                None
            }
            Self::ClearIntegrators { ops, next } => {
                let (reg, value) = ops[*next as usize];
                if backend.write_register(reg, value).await.is_err() {
                    return Some(Err(CommissionError::Spi));
                }
                *next += 1;
                if *next >= 8 {
                    *self = Self::ClearStatusFlags;
                }
                None
            }
            Self::ClearStatusFlags => {
                Some(crate::tmc_control::interlock::clear_status_flags_via_backend(backend).await)
            }
        }
    }
}

#[derive(Clone, Copy)]
pub(crate) enum ArmMotorElectricalStep {
    Drain,
    VerifySentinelReadback,
    VerifyPiRepresentation(crate::tmc_control::pi_representation::PiRepresentationVerifyStep),
    ClearTorqueOffset,
    WriteCurrentLimit,
    Settle(ClosedLoopHoldSettleStep),
    FinalizeEnable,
}

impl ArmMotorElectricalStep {
    pub(crate) const fn start() -> Self {
        Self::Drain
    }

    pub(crate) async fn advance<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        state: &mut DispatchState<'_>,
    ) -> Option<Result<(), crate::autotune::CommissionError>> {
        use crate::autotune::CommissionError;
        use core::sync::atomic::Ordering::Relaxed;

        match self {
            Self::Drain => {
                assert!(
                    *state.calibrated,
                    "FATAL: arm_motor_electrical called with calibrated=false; \
                     SPI restore must precede arm"
                );
                state.tmc.stall_latched.store(false, Relaxed);
                state.tmc.homing_clamp_active.store(false, Relaxed);
                if state.tmc.safe_state_last_pass_failed.load(Relaxed) {
                    defmt::error!(
                        "arm_motor_electrical: safe-state cleanup incomplete, refusing arm"
                    );
                    return Some(Err(CommissionError::SafeStateIncomplete));
                }
                if let Err(item) = backend
                    .drain_pending_config(crate::tmc_control::pending_config::PendingConfigSink {
                        stored_inner_gains: state.stored_gains,
                    })
                    .await
                {
                    defmt::error!(
                        "arm_motor_electrical: pending config apply failed ({}), refusing arm",
                        item
                    );
                    return Some(Err(CommissionError::Spi));
                }
                *self = Self::VerifySentinelReadback;
                None
            }
            Self::VerifySentinelReadback => {
                use foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS;
                let expected = state.tmc.motor_sentinel.load(Relaxed);
                if expected == 0 {
                    defmt::error!("arm_motor_electrical: missing MOTOR_TYPE sentinel");
                    return Some(Err(CommissionError::ChipResetDetected));
                }
                if backend
                    .read_register_verified(MOTOR_TYPE_N_POLE_PAIRS, expected)
                    .await
                    .is_err()
                {
                    return Some(Err(CommissionError::ChipResetDetected));
                }
                *self = Self::VerifyPiRepresentation(
                    crate::tmc_control::pi_representation::PiRepresentationVerifyStep::start(),
                );
                None
            }
            Self::VerifyPiRepresentation(step) => match step.advance(backend, state.tmc).await {
                None => None,
                Some(Ok(())) => {
                    state.tmc.safe_state_incomplete.store(false, Relaxed);
                    *self = Self::ClearTorqueOffset;
                    None
                }
                Some(Err(_)) => Some(Err(CommissionError::ChipResetDetected)),
            },
            Self::ClearTorqueOffset => {
                if backend
                    .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET, 0)
                    .await
                    .is_err()
                {
                    return Some(Err(CommissionError::Spi));
                }
                *self = Self::WriteCurrentLimit;
                None
            }
            Self::WriteCurrentLimit => {
                let run_limit = state.tmc.load_current_limit(Relaxed);
                if run_limit.raw() == 0 {
                    defmt::error!("arm_motor_electrical: no run current configured");
                    return Some(Err(CommissionError::InvalidCurrentLimit));
                }
                if backend
                    .write_register(
                        foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                        crate::tmc_control::current_limit::operational_limit_register(state.tmc),
                    )
                    .await
                    .is_err()
                {
                    return Some(Err(CommissionError::Spi));
                }
                *self = Self::Settle(ClosedLoopHoldSettleStep::start());
                None
            }
            Self::Settle(settle) => match settle.advance(backend).await {
                None => None,
                Some(Ok(())) => {
                    *self = Self::FinalizeEnable;
                    None
                }
                Some(Err(error)) => Some(Err(error)),
            },
            Self::FinalizeEnable => {
                if backend.interlock_tripped() {
                    return Some(Err(CommissionError::FaultInterlock));
                }
                let armed = critical_section::with(|_| {
                    if state.tmc.config_apply_pending.load(Relaxed) != 0 {
                        return false;
                    }
                    backend.set_enable_pin(true);
                    true
                });
                if !armed {
                    defmt::error!(
                        "arm_motor_electrical: configuration recorded after the drain, refusing arm"
                    );
                    return Some(Err(CommissionError::Spi));
                }
                *state.motor_enabled = true;
                backend.record_lifecycle(LifecycleFlag::MotorEnabled, true);
                Some(Ok(()))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::breakaway_campaign_engine;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, fresh_state_locals};
    use crate::tmc_control::pending_config::PendingConfigItem;
    use foci_firmware_macros::fixture_test;

    fn dispatch_state<'a>(
        calibrated: &'a mut bool,
        motor_enabled: &'a mut bool,
        stored_gains: &'a mut Option<crate::pid::InnerLoopGains>,
    ) -> DispatchState<'a> {
        crate::state::tmc::tmc_channel0().mark_pi_representation_established();
        DispatchState::new(
            0,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            calibrated,
            motor_enabled,
            stored_gains,
        )
    }

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    #[test]
    #[should_panic(expected = "calibrated=false")]
    fn arm_motor_electrical_panics_when_uncalibrated() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let (mut cal, mut en, mut gains) = fresh_state_locals();
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
            let _ = arm_motor_electrical(&mut backend, &mut state).await;
        });
    }

    #[test]
    #[should_panic(expected = "motor_enabled=false")]
    fn arm_motor_for_steps_panics_when_motor_disabled() {
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        arm_motor_for_steps(&mut state);
    }

    #[fixture_test]
    async fn arm_then_disarm_returns_to_baseline() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("sentinel should match");
        arm_motor_for_steps(&mut state);
        assert!(backend.enable_pin);
        assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
        disarm_motor(&mut backend, &mut state).await;
        assert!(!backend.enable_pin);
        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_drains_pending_config_before_energizing() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        tmc_state()
            .config_apply_pending
            .store(crate::state::tmc::PENDING_CURRENT, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;

        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("a clean drain and matching sentinel should arm");

        let drained = backend
            .spi_log
            .iter()
            .position(|op| *op == SpiOp::DrainPendingConfig)
            .expect("arming must drain recorded configuration");
        let energized = backend
            .spi_log
            .iter()
            .position(|op| *op == SpiOp::EnablePin(true))
            .expect("a successful arm must assert the enable pin");
        assert!(
            drained < energized,
            "recorded configuration must reach the chip before it is energized"
        );
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_config_is_recorded_while_it_drains() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.pending_config_set_during_drain = crate::state::tmc::PENDING_FILTERS;

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "must not energize against configuration recorded after the drain"
        );
        assert!(!*state.motor_enabled);
        assert_eq!(
            tmc_state().config_apply_pending.load(Ordering::Relaxed),
            crate::state::tmc::PENDING_FILTERS,
            "the late bit stays pending for the next control tick to drain"
        );

        tmc_state().config_apply_pending.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_config_lands_just_before_the_enable_pin() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.pending_config_set_at_interlock_check = crate::state::tmc::PENDING_CURRENT;

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "the pending check must sit after every awaiting step, not just after the drain"
        );
        assert!(!*state.motor_enabled);

        tmc_state().config_apply_pending.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_the_pending_config_drain_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        tmc_state()
            .config_apply_pending
            .store(crate::state::tmc::PENDING_CURRENT, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.pending_config_result = Err(PendingConfigItem::Current);
        backend.spi_log.clear();

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "must not energize while recorded configuration is unwritten"
        );
        assert!(!*state.motor_enabled);
        assert_eq!(
            backend.spi_log.as_slice(),
            &[SpiOp::DrainPendingConfig],
            "a failed drain must return before the sentinel verify touches the chip"
        );
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_last_safe_state_pass_failed() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        tmc_state()
            .safe_state_last_pass_failed
            .store(true, Ordering::Relaxed);
        let spi_log_len_before = backend.spi_log.len();

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::SafeStateIncomplete)
        );
        assert_eq!(
            backend.spi_log.len(),
            spi_log_len_before,
            "must not touch the chip at all while the last restore pass failed"
        );
        assert!(
            !backend.enable_pin,
            "must not enable while the last restore pass failed"
        );
        assert!(!*state.motor_enabled);

        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state()
            .safe_state_last_pass_failed
            .store(false, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_clears_incomplete_latch_only_after_sentinel_succeeds() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        tmc_state()
            .safe_state_last_pass_failed
            .store(false, Ordering::Relaxed);
        tmc_state()
            .safe_state_incomplete
            .store(true, Ordering::Relaxed);

        backend.read_override = Some((
            foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS.addr(),
            0x0002_0032 ^ 1,
        ));
        let mismatched = arm_motor_electrical(&mut backend, &mut state).await;
        assert_eq!(
            mismatched,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(
            tmc_state().safe_state_incomplete.load(Ordering::Relaxed),
            "a failed sentinel check must not clear the compound latch"
        );
        backend.read_override = None;

        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::new(0x1234).unwrap(),
            Ordering::Relaxed,
        );
        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("sentinel should match once the override is removed");
        assert!(
            !tmc_state().safe_state_incomplete.load(Ordering::Relaxed),
            "a successful sentinel pass must clear the compound latch"
        );

        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state()
            .safe_state_incomplete
            .store(false, Ordering::Relaxed);
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
        *state.motor_enabled = false;
    }

    #[fixture_test]
    async fn disarm_motor_clears_latched_velocity_clamp() {
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state().record_velocity_clamp_engagement();
        assert!(tmc_state().velocity_clamp_latched());

        disarm_motor(&mut backend, &mut state).await;

        assert!(!tmc_state().velocity_clamp_latched());
        assert_eq!(tmc_state().velocity_clamp_count(), 0);
    }

    #[fixture_test]
    async fn arm_motor_success_path() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("sentinel should match");
        assert!(backend.enable_pin);
        assert!(*state.motor_enabled);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        arm_motor_for_steps(&mut state);
        assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_sentinel_mismatches() {
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        backend.read_override = Some((foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS.addr(), 0));
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let result = arm_motor_electrical(&mut backend, &mut state).await;
        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(
            !backend.enable_pin,
            "enable pin must remain low on mismatch"
        );
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_pi_representation_mismatches() {
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        tmc_state().mark_pi_representation_established();
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.read_override = Some((
            foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                .data
                .addr(),
            0,
        ));
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(!backend.enable_pin);
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
        tmc_state().clear_pi_representation_established();
    }

    #[fixture_test]
    async fn arm_motor_electrical_succeeds_when_sentinel_matches() {
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        let run_limit = PidTorqueFluxLimit::new(0x1234).unwrap();
        tmc_state().store_current_limit(run_limit, core::sync::atomic::Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.spi_log.clear();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let result = arm_motor_electrical(&mut backend, &mut state).await;
        assert_eq!(result, Ok(()));
        assert!(backend.enable_pin);
        let torque_limit_write_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::Write(addr, value)
                        if *addr == foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS.addr()
                            && *value == run_limit.register_value()
                )
            })
            .expect("arm should rewrite PID_TORQUE_FLUX_LIMITS from the stored run-current limit");
        let enable_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::EnablePin(true)
                )
            })
            .expect("arm should assert enable");
        assert!(torque_limit_write_index < enable_index);
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    async fn arm_limit_writes(reserved: bool) -> std::vec::Vec<u32> {
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        tmc_state().store_current_limit(
            crate::commission_dispatch::test_support::current_limit(2557),
            core::sync::atomic::Ordering::Relaxed,
        );
        tmc_state().set_offset_headroom_reserved(reserved, core::sync::atomic::Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.spi_log.clear();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        assert_eq!(arm_motor_electrical(&mut backend, &mut state).await, Ok(()));

        backend
            .spi_log
            .iter()
            .filter_map(|op| match op {
                SpiOp::Write(addr, value)
                    if *addr == foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS.addr() =>
                {
                    Some(*value)
                }
                _ => None,
            })
            .collect()
    }

    #[fixture_test]
    async fn arm_writes_the_reserved_limit_while_the_reservation_is_set() {
        assert_eq!(arm_limit_writes(true).await, std::vec![2557 - 1278]);
    }

    #[fixture_test]
    async fn arm_writes_the_committed_limit_while_the_reservation_is_clear() {
        assert_eq!(arm_limit_writes(false).await, std::vec![2557]);
    }

    #[fixture_test]
    async fn arm_motor_electrical_clears_stale_stall_latch() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        tmc_state().motor_sentinel.store(0x0002_0032, Relaxed);
        tmc_state().store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        tmc_state().stall_latched.store(true, Relaxed);
        tmc_state().homing_clamp_active.store(true, Relaxed);
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("sentinel and run current should be valid");

        assert!(!tmc_state().stall_latched.load(Relaxed));
        assert!(!tmc_state().homing_clamp_active.load(Relaxed));
        tmc_state().motor_sentinel.store(0, Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_without_run_current() {
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        tmc_state().store_current_limit(
            PidTorqueFluxLimit::ZERO,
            core::sync::atomic::Ordering::Relaxed,
        );
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::InvalidCurrentLimit)
        );
        assert!(
            !backend.enable_pin,
            "enable pin must remain low without a configured run current"
        );
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_sets_hold_target_before_enable_in_position_mode() {
        use foci_tmc4671::registers::{
            MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS, PID_POSITION_ACTUAL,
            PID_POSITION_TARGET, STATUS_FLAGS,
        };

        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        tmc_state().store_current_limit(
            PidTorqueFluxLimit::new(0x1234).unwrap(),
            core::sync::atomic::Ordering::Relaxed,
        );
        let _ = backend
            .write_register(MOTOR_TYPE_N_POLE_PAIRS, 0x0002_0032)
            .await;
        let _ = backend
            .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
            .await;
        let _ = backend.write_register(PID_POSITION_TARGET, 0).await;
        let _ = backend.write_register(PID_POSITION_ACTUAL, 11).await;
        backend.spi_log.clear();

        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(result, Ok(()));
        let enable_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::EnablePin(true)
                )
            })
            .expect("arm should assert enable");
        let target_rewrite_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::Write(addr, 11)
                        if *addr == PID_POSITION_ACTUAL.addr()
                )
            })
            .expect("arm should rewrite PID_POSITION_ACTUAL to observed actual");
        let status_clear_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::Write(addr, 0)
                        if *addr == STATUS_FLAGS.addr()
                )
            })
            .expect("arm should clear STATUS_FLAGS");
        assert!(target_rewrite_index < enable_index);
        assert!(status_clear_index < enable_index);
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    #[fixture_test]
    async fn shutdown_safe_state_disarms() {
        use core::sync::atomic::Ordering;

        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        {
            let state = runtime.dispatch_state(
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            );
            *state.calibrated = true;
            *state.motor_enabled = true;
        }
        tmc_state().motor_armed.store(true, Ordering::Relaxed);

        apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        )
        .await;

        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        assert!(!runtime.is_calibrated());
        assert!(!runtime.is_motor_enabled());
    }

    #[fixture_test]
    fn shutdown_breakaway_cancellation_completes_after_safe_state() {
        use core::sync::atomic::Ordering;

        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        runtime.set_commission_engine(breakaway_campaign_engine());
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        {
            let state = runtime.dispatch_state(
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            );
            *state.calibrated = true;
            *state.motor_enabled = true;
        }
        tmc_state().motor_armed.store(true, Ordering::Relaxed);
        crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
        crate::diagnostic::begin_breakaway_campaign();
        crate::diagnostic::panic_after_breakaway_completion_for_test();

        let reset = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            pollster::block_on(apply_shutdown_safe_state(
                &mut runtime,
                &mut backend,
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            ));
        }));

        assert!(reset.is_err(), "completion must reach the reset injection");
        assert!(!backend.enable_pin, "safe disarm must precede completion");
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        assert!(
            runtime.is_commission_active(),
            "engine removal must follow completion"
        );
        assert_eq!(
            crate::diagnostic::current_breakaway_breadcrumb_for_test(),
            Some(crate::diagnostic::BreakawayBreadcrumb {
                checkpoint: crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
                interrupted: false,
            })
        );

        pollster::block_on(apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        ));
        assert!(!runtime.is_commission_active());
    }

    #[test]
    fn apply_shutdown_safe_state_lowers_gpio_before_safe_state_writes() {
        pollster::block_on(async {
            use crate::commission_dispatch::test_support::SpiOp;

            let mut backend = MockBackend::new();
            backend.enable_pin = true;
            let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);

            apply_shutdown_safe_state(
                &mut runtime,
                &mut backend,
                tmc_state(),
                crate::state::stepper::stepper_channel0(),
            )
            .await;

            let enable_pin_false_index = backend
                .spi_log
                .iter()
                .position(|op| matches!(op, SpiOp::EnablePin(false)))
                .expect("EnablePin(false) must be logged");
            let first_write_index = backend
                .spi_log
                .iter()
                .position(|op| matches!(op, SpiOp::Write(..)))
                .expect("disarm must write the safe-state registers");
            assert!(
                enable_pin_false_index < first_write_index,
                "GPIO must be lowered before any safe-state SPI write: \
                 enable_pin_false_index={enable_pin_false_index}, \
                 first_write_index={first_write_index}"
            );
        });
    }

    #[test]
    fn apply_shutdown_safe_state_clears_homing_runtime() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
            let tmc = crate::state::tmc::tmc_channel0();
            let stepper = crate::state::stepper::stepper_channel0();
            use core::sync::atomic::Ordering::Relaxed;
            runtime.arm_stall_detector_for_test(100, 50, 2);
            tmc.homing_clamp_active.store(true, Relaxed);
            tmc.stall_latched.store(true, Relaxed);

            let _ = apply_shutdown_safe_state(&mut runtime, &mut backend, tmc, stepper).await;

            assert!(!runtime.stall_detector_armed_for_test());
            assert!(!tmc.homing_clamp_active.load(Relaxed));
            assert!(!tmc.stall_latched.load(Relaxed));

            tmc.homing_clamp_active.store(false, Relaxed);
            tmc.stall_latched.store(false, Relaxed);
        });
    }

    #[fixture_test]
    async fn arm_refuses_to_raise_the_enable_pin_when_the_interlock_is_tripped() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;

        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        state.tmc.motor_sentinel.store(0x0002_0032, Relaxed);
        state
            .tmc
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.interlock_tripped = true;

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::FaultInterlock)
        );
        assert!(!backend.enable_pin);
        assert!(!*state.motor_enabled);

        state.tmc.motor_sentinel.store(0, Relaxed);
    }

    #[fixture_test]
    async fn pre_arm_reset_does_not_clear_status_flags_when_the_interlock_is_tripped() {
        use foci_tmc4671::registers::{MODE_RAMP_MODE_MOTION, STATUS_FLAGS};

        let mut backend = MockBackend::new();
        backend
            .registers
            .insert(MODE_RAMP_MODE_MOTION.addr(), MOTION_POSITION_ADVANCED_PI)
            .unwrap();
        backend.interlock_tripped = true;

        let result = settle_closed_loop_hold_before_enable(&mut backend).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::FaultInterlock)
        );
        assert!(
            !backend.spi_log.iter().any(|op| matches!(
                op,
                crate::commission_dispatch::test_support::SpiOp::Write(addr, _)
                    if *addr == STATUS_FLAGS.addr()
            )),
            "STATUS_FLAGS must not be written while the interlock is tripped"
        );
    }

    #[test]
    fn the_pending_config_check_and_the_enable_pin_share_one_critical_section() {
        let source = include_str!("lifecycle.rs");
        let source = source
            .split("#[cfg(test)]")
            .next()
            .expect("the module has a non-test half");
        let body = source
            .split("Self::FinalizeEnable => {")
            .nth(1)
            .expect("the arm step machine has a FinalizeEnable arm")
            .split("Some(Ok(()))")
            .next()
            .expect("the FinalizeEnable arm ends in success");

        let mask = body
            .find("critical_section::with")
            .expect("the arm decision must run with interrupts masked");
        let check = body
            .find("config_apply_pending.load(")
            .expect("the arm must check for configuration recorded after the drain");
        let energize = body
            .find("set_enable_pin(true)")
            .expect("the arm must raise the enable pin");

        assert!(
            mask < check && check < energize,
            "the critical section must open before the pending check, which \
             must precede the enable pin"
        );
        assert!(
            !body[mask..energize].contains(".await"),
            "an awaiting step between the check and the enable pin hands the \
             window back to the higher-priority config handlers"
        );
    }

    async fn arm_op_log(position_mode: bool) -> std::vec::Vec<SpiOp> {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;

        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        if position_mode {
            let _ = backend
                .write_register(
                    foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                    MOTION_POSITION_ADVANCED_PI,
                )
                .await;
        }
        backend.spi_log.clear();

        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("arm must succeed against a healthy mock backend");
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        backend.spi_log
    }

    fn arm_prefix() -> [SpiOp; 7] {
        use foci_tmc4671::registers::{
            CONFIG_ADVANCED_PI_REPRESENT, PID_TORQUE_FLUX_LIMITS, PID_TORQUE_FLUX_OFFSET,
        };
        [
            SpiOp::DrainPendingConfig,
            SpiOp::Read,
            SpiOp::Write(
                CONFIG_ADVANCED_PI_REPRESENT.addr.addr(),
                u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
            ),
            SpiOp::Read,
            SpiOp::Write(PID_TORQUE_FLUX_OFFSET.addr(), 0),
            SpiOp::Write(PID_TORQUE_FLUX_LIMITS.addr(), 0x1234),
            SpiOp::Read,
        ]
    }

    fn arm_suffix() -> [SpiOp; 2] {
        [
            SpiOp::EnablePin(true),
            SpiOp::Lifecycle {
                flag: LifecycleFlag::MotorEnabled,
                value: true,
            },
        ]
    }

    #[fixture_test]
    async fn arm_motor_electrical_op_log_not_in_position_mode() {
        let expected: std::vec::Vec<SpiOp> = arm_prefix().into_iter().chain(arm_suffix()).collect();

        assert_eq!(arm_op_log(false).await, expected);
    }

    #[fixture_test]
    async fn arm_motor_electrical_op_log_in_position_mode() {
        use foci_tmc4671::registers::{
            PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM,
            PID_ERROR_PID_TORQUE_ERROR_SUM, PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_POSITION_ACTUAL,
            STATUS_FLAGS,
        };
        let mut expected: std::vec::Vec<SpiOp> = arm_prefix().into();
        expected.push(SpiOp::Read);
        expected.push(SpiOp::Write(PID_POSITION_ACTUAL.addr(), 0));
        for sum in [
            PID_ERROR_PID_TORQUE_ERROR_SUM,
            PID_ERROR_PID_FLUX_ERROR_SUM,
            PID_ERROR_PID_VELOCITY_ERROR_SUM,
            PID_ERROR_PID_POSITION_ERROR_SUM,
        ] {
            expected.push(SpiOp::Write(sum.addr.addr(), u32::from(sum.sub_addr)));
            expected.push(SpiOp::Write(sum.data.addr(), 0));
        }
        expected.push(SpiOp::Write(STATUS_FLAGS.addr(), 0));
        expected.extend(arm_suffix());

        assert_eq!(arm_op_log(true).await, expected);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_advances_one_op_per_call_in_position_mode() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;

        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                MOTION_POSITION_ADVANCED_PI,
            )
            .await;
        backend.spi_log.clear();

        let mut step = ArmMotorElectricalStep::start();
        let mut ticks = 0usize;
        loop {
            let before = backend.spi_log.len();
            let result = step.advance(&mut backend, &mut state).await;
            let raw_after = backend.spi_log[before..]
                .iter()
                .filter(|op| matches!(op, SpiOp::Write(_, _) | SpiOp::Read))
                .count();
            assert!(
                raw_after <= 1,
                "tick {}: advance() issued {} raw register op(s), expected at most 1",
                ticks,
                raw_after
            );
            ticks += 1;
            if result.is_some() {
                break;
            }
        }
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_last_safe_state_pass_failed() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .safe_state_last_pass_failed
            .store(true, Ordering::Relaxed);
        let spi_log_len_before = backend.spi_log.len();

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::SafeStateIncomplete)
        );
        assert_eq!(
            backend.spi_log.len(),
            spi_log_len_before,
            "must not touch the chip at all while the last restore pass failed"
        );
        assert!(!backend.enable_pin);
        tmc_state()
            .safe_state_last_pass_failed
            .store(false, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_the_pending_config_drain_fails() {
        use crate::tmc_control::pending_config::PendingConfigItem;
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        backend.pending_config_result = Err(PendingConfigItem::Current);

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        assert_eq!(
            backend.spi_log.as_slice(),
            &[SpiOp::DrainPendingConfig],
            "a failed drain must return before the sentinel verify touches the chip"
        );
        let _ = Ordering::Relaxed;
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_sentinel_mismatches() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032 ^ 1,
            )
            .await;

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_without_run_current() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::InvalidCurrentLimit)
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_pi_representation_not_established() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        tmc_state().clear_pi_representation_established();

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state().mark_pi_representation_established();
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_settle_mode_read_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.fail_on_read = Some(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr());

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_settle_position_read_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                MOTION_POSITION_ADVANCED_PI,
            )
            .await;
        backend.fail_on_read = Some(foci_tmc4671::registers::PID_POSITION_ACTUAL.addr());

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_settle_position_write_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                MOTION_POSITION_ADVANCED_PI,
            )
            .await;
        backend.fail_on_write = Some(foci_tmc4671::registers::PID_POSITION_ACTUAL.addr());

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_an_integrator_write_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                MOTION_POSITION_ADVANCED_PI,
            )
            .await;
        backend.fail_on_write = Some(
            foci_tmc4671::registers::PID_ERROR_PID_TORQUE_ERROR_SUM
                .data
                .addr(),
        );

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_status_flags_write_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                MOTION_POSITION_ADVANCED_PI,
            )
            .await;
        backend.fail_on_write = Some(foci_tmc4671::registers::STATUS_FLAGS.addr());

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_interlock_tripped_during_status_clear() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                MOTION_POSITION_ADVANCED_PI,
            )
            .await;
        backend.interlock_tripped = true;

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::FaultInterlock)
        );
        assert!(
            !backend
                .spi_log
                .iter()
                .any(|op| *op == SpiOp::Write(foci_tmc4671::registers::STATUS_FLAGS.addr(), 0)),
            "an interlock-tripped status-clear must not write STATUS_FLAGS"
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_interlock_trips_only_after_status_clear() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                MOTION_POSITION_ADVANCED_PI,
            )
            .await;

        let mut step = ArmMotorElectricalStep::start();
        loop {
            match step.advance(&mut backend, &mut state).await {
                None if matches!(step, ArmMotorElectricalStep::FinalizeEnable) => break,
                None => continue,
                Some(_) => unreachable!("must not complete before FinalizeEnable"),
            }
        }
        backend.interlock_tripped = true;

        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::FaultInterlock)
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_config_lands_just_before_the_enable_pin() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.pending_config_set_at_interlock_check = crate::state::tmc::PENDING_CURRENT;

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "the pending check must sit after every awaiting step, not just after the drain"
        );
        assert!(!*state.motor_enabled);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state().config_apply_pending.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_config_recorded_between_polls() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;

        let mut step = ArmMotorElectricalStep::start();
        assert!(
            step.advance(&mut backend, &mut state).await.is_none(),
            "the Drain phase must not complete the arm in a single tick"
        );
        tmc_state()
            .config_apply_pending
            .store(crate::state::tmc::PENDING_FILTERS, Ordering::Relaxed);

        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "configuration recorded between two poll_step calls must still refuse the arm"
        );
        assert!(!*state.motor_enabled);
        assert_eq!(
            tmc_state().config_apply_pending.load(Ordering::Relaxed),
            crate::state::tmc::PENDING_FILTERS,
            "the bit recorded between polls stays pending for the next control tick to drain"
        );

        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state().config_apply_pending.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn drain_pending_config_worst_case_op_count_is_bounded() {
        use crate::tmc_control::pending_config::{PendingConfigSink, drain_pending_config};
        use crate::tmc_control::test_spi::RuntimeMockSpi;
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12;
        use foci_tmc4671::spi::Tmc4671Spi;

        const WORST_CASE_OP_COUNT: usize = 59;

        let tmc_state = crate::state::tmc::tmc_channel0();
        tmc_state.config_apply_pending.store(
            crate::state::tmc::PENDING_VOLTAGE_LIMIT
                | crate::state::tmc::PENDING_CURRENT
                | crate::state::tmc::PENDING_ENCODER_DIR
                | crate::state::tmc::PENDING_INNER_GAINS
                | crate::state::tmc::PENDING_OUTER_GAINS
                | crate::state::tmc::PENDING_FILTERS,
            Relaxed,
        );
        tmc_state.voltage_limit_cfg.store(
            crate::state::tmc::MIN_OPERATIONAL_PIDOUT_UQ_UD_LIMIT,
            Relaxed,
        );
        tmc_state.run_current_ma_cfg.store(1500, Relaxed);
        tmc_state.encoder_invert_cfg.store(Some(false), Relaxed);
        tmc_state.store_pending_inner_gains(
            crate::pid::InnerLoopGains {
                flux_p: foci_tmc4671::pid::PidPGain::new(100).unwrap(),
                flux_i: foci_tmc4671::pid::PidIGain::new(100).unwrap(),
                torque_p: foci_tmc4671::pid::PidPGain::new(100).unwrap(),
                torque_i: foci_tmc4671::pid::PidIGain::new(100).unwrap(),
            },
            Relaxed,
        );
        tmc_state.store_pending_outer_gains(
            crate::pid::OuterLoopGains {
                velocity_p: foci_tmc4671::pid::PidPGain::new(100).unwrap(),
                velocity_i: foci_tmc4671::pid::PidIGain::new(100).unwrap(),
                position_p: foci_tmc4671::pid::PidPGain::new(100).unwrap(),
                position_i: foci_tmc4671::pid::PidIGain::new(100).unwrap(),
            },
            Relaxed,
        );
        tmc_state.velocity_filter_hz.store(Some(200), Relaxed);
        tmc_state.torque_filter_hz.store(Some(300), Relaxed);
        tmc_state.position_filter_hz.store(Some(400), Relaxed);
        tmc_state.flux_filter_hz.store(Some(500), Relaxed);
        tmc_state.mark_pi_representation_established();

        let mut stored_inner_gains = None;
        let raw_spi = RuntimeMockSpi::default()
            .with_read(ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12)
            .with_read(ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        drain_pending_config(
            &mut tmc,
            tmc_state,
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
                resistance_max_expected_current_count: 2000,
                resistance_voltage_scale: 1,
                resistance_settle_ms: 500,
                resistance_samples_per_window: 8,
                resistance_sample_interval_ms: 5,
                inductance_voltage: 768,
                resistance_scale_metadata_validated: true,
                current_scale_ma_per_lsb: 1.155,
                max_run_current_ma: 10_000,
                vm_divider_high_ohms: 71_500,
                vm_divider_low_ohms: 1_500,
                vm_adc_reference_millivolts: 2_500,
                vm_adc_center_counts: 32_767,
                motor_type_n_pole_pairs:
                    crate::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS,
            },
            PendingConfigSink {
                stored_inner_gains: &mut stored_inner_gains,
            },
        )
        .await
        .expect("a fully-populated, valid pending config must drain cleanly");

        let raw_spi = tmc.into_inner();
        let op_count = raw_spi.writes.len() + raw_spi.read_requests.len();
        assert_eq!(
            op_count, WORST_CASE_OP_COUNT,
            "drain_pending_config's worst-case op count changed -- if this is \
             an intentional new pending-config item or a wider filter set, \
             update WORST_CASE_OP_COUNT deliberately; if not, this is new, \
             uncapped growth in arm_motor_electrical's single Drain tick"
        );

        tmc_state.config_apply_pending.store(0, Relaxed);
    }

    #[fixture_test]
    async fn arm_after_a_failed_offset_clear_and_queue_flush_zeroes_the_stale_offset_first() {
        use crate::commission_dispatch::test_support::current_limit;
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::{
            MOTOR_TYPE_N_POLE_PAIRS, PID_TORQUE_FLUX_LIMITS, PID_TORQUE_FLUX_OFFSET,
        };
        let limit = current_limit(1_000);
        let held_limit = crate::tmc_control::current_limit::reserved_limit(limit).register_value();
        let stale_offset = 400u32 << 16;
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        tmc.store_current_limit(limit, Relaxed);
        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        runtime.hold_offset_headroom_for_test(tmc, stale_offset);
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(PID_TORQUE_FLUX_OFFSET.addr());

        apply_shutdown_safe_state(&mut runtime, &mut backend, tmc, stepper).await;
        assert!(
            backend
                .spi_log
                .contains(&SpiOp::Write(PID_TORQUE_FLUX_OFFSET.addr(), 0)),
            "safe state attempted the offset clear"
        );
        assert_ne!(
            backend.registers.get(&PID_TORQUE_FLUX_OFFSET.addr()),
            Some(&0),
            "the offset clear never reached the chip"
        );
        backend.fail_on_write = None;
        let flushed_at = backend.spi_log.len();
        runtime.reset_runtime_state();
        crate::tmc_control::TmcControlRuntime::reset_channel_state(tmc, stepper);
        tmc.store_current_limit(limit, Relaxed);
        tmc.motor_sentinel.store(0x0002_0032, Relaxed);
        let _ = backend
            .write_register(MOTOR_TYPE_N_POLE_PAIRS, 0x0002_0032)
            .await;
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("arm succeeds once the offset write succeeds");

        let mut writes = std::vec![
            (PID_TORQUE_FLUX_LIMITS.addr(), held_limit),
            (PID_TORQUE_FLUX_OFFSET.addr(), stale_offset),
        ];
        writes.extend(
            backend
                .spi_log
                .iter()
                .enumerate()
                .filter_map(|(index, op)| match op {
                    SpiOp::Write(addr, _)
                        if index < flushed_at && *addr == PID_TORQUE_FLUX_OFFSET.addr() =>
                    {
                        None
                    }
                    SpiOp::Write(addr, value) => Some((*addr, *value)),
                    _ => None,
                }),
        );
        let limit_write = writes
            .iter()
            .rposition(|(addr, _)| *addr == PID_TORQUE_FLUX_LIMITS.addr())
            .expect("arm writes the operational limit");
        assert_eq!(
            writes[..limit_write]
                .iter()
                .rev()
                .find(|(addr, _)| *addr == PID_TORQUE_FLUX_OFFSET.addr()),
            Some(&(PID_TORQUE_FLUX_OFFSET.addr(), 0)),
            "{writes:?}"
        );
        crate::tmc_control::test_spi::assert_limit_plus_offset_within(
            &writes,
            u32::from(limit.raw()),
        );
        tmc.motor_sentinel.store(0, Relaxed);
    }
}
