use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_AGPI_A_RAW_ADC_VM_RAW, ADC_IWY_IUX, ADC_RAW_ADDR, ADC_RAW_DATA,
    DSADC_MDEC_B_MDEC_A, MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS, MotorType,
    MotorTypeNPolePairs, PHI_E_EXT, PHI_E_SELECTION, PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L,
    PWM_MAXCNT, PWM_SV_CHOP, STATUS_FLAGS, UQ_UD_EXT,
};

use super::{
    PHASE_DEADLINE_US, ResistancePoll, ResistanceRunner, ResistanceState, WindowCapture,
    WindowPlan, WindowPurpose, acquisition_opportunities_through, exposure_is_supported,
    max_opportunities_per_window, nominal_duration_us, pack_ud, phase_current_for_axis,
    wrapped_encoder_delta, write,
};
use crate::autotune::current_loop_actionable_status_mask;
use crate::commissioning::capture_timing::{CaptureTiming, TimingEnvelope, sample_slot};
use crate::resistance_identification::bus_voltage::{
    BusVoltageSense, bus_millivolts, bus_millivolts_in_range, scale_profile_to_bus,
};
use crate::resistance_identification::runner::{ResistanceBackend, ResistanceRunMode};
use crate::resistance_identification::types::{ResistanceError, ResistanceWarningFlags};
use crate::tmc_register_values::{MOTION_STOPPED, MOTION_UQ_UD_EXT};

impl ResistanceRunner {
    pub(super) async fn read_saved_limit<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
    ) -> ResistancePoll {
        if !backend.adc_offsets_ready() {
            return self.complete_without_restore(ResistanceError::NotAdcReady);
        }
        if self.mode == ResistanceRunMode::ExternallyArmed
            && !backend.is_armed_for_open_loop_voltage()
        {
            return self.complete_without_restore(ResistanceError::NotArmed);
        }
        match backend.read_register(PIDOUT_UQ_UD_LIMITS.addr()).await {
            Ok(value) => {
                self.saved_voltage_limit = value;
                if self.mode == ResistanceRunMode::ExternallyArmed
                    && value < u32::from(self.profile.max_abs_ud().unsigned_abs())
                {
                    return self.complete_without_restore(ResistanceError::OutOfBoundsParameter);
                }
                self.state = ResistanceState::ReadMotorType;
                ResistancePoll::ReadyNow
            }
            Err(_) => self.complete_without_restore(ResistanceError::BackendIo),
        }
    }

    pub(super) async fn read_motor_type<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
    ) -> ResistancePoll {
        match backend.read_register(MOTOR_TYPE_N_POLE_PAIRS.addr()).await {
            Ok(raw)
                if matches!(
                    MotorTypeNPolePairs::from(raw).motor_type(),
                    Ok(MotorType::TwoPhase | MotorType::ThreePhase)
                ) =>
            {
                self.state = ResistanceState::ReadBusVoltage(0);
                ResistancePoll::ReadyNow
            }
            Ok(_) => self.complete_without_restore(ResistanceError::InvalidMotorType),
            Err(_) => self.complete_without_restore(ResistanceError::BackendIo),
        }
    }

    pub(super) async fn read_bus_voltage<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        operation: u8,
    ) -> ResistancePoll {
        let Some(sense) = backend.bus_voltage_sense() else {
            self.warning_flags
                .insert(ResistanceWarningFlags::BUS_VOLTAGE_UNSCALED);
            defmt::warn!("resistance sweep: no bus-voltage sense, ladder runs unscaled");
            self.state = ResistanceState::Setup(0);
            return ResistancePoll::ReadyNow;
        };
        let vm_selector = u32::from(ADC_AGPI_A_RAW_ADC_VM_RAW.sub_addr);
        match operation {
            0 => match backend.read_register(ADC_RAW_ADDR.addr()).await {
                Ok(saved) => self.saved_adc_raw_addr = saved,
                Err(_) => return self.complete_without_restore(ResistanceError::BackendIo),
            },
            1 => {
                if write(backend, ADC_RAW_ADDR.addr(), vm_selector)
                    .await
                    .is_err()
                {
                    self.state = ResistanceState::ReadBusVoltage(3);
                    return ResistancePoll::ReadyNow;
                }
            }
            2 => {
                self.adc_vm_raw = backend
                    .read_register(ADC_RAW_DATA.addr())
                    .await
                    .ok()
                    .map(|raw| (raw & 0xffff) as u16);
            }
            _ => {
                let restored = write(backend, ADC_RAW_ADDR.addr(), self.saved_adc_raw_addr).await;
                return match (self.adc_vm_raw, restored) {
                    (Some(raw), Ok(())) => self.apply_bus_voltage(sense, raw),
                    _ => self.complete_without_restore(ResistanceError::BackendIo),
                };
            }
        }
        self.state = ResistanceState::ReadBusVoltage(operation + 1);
        ResistancePoll::ReadyNow
    }

    fn apply_bus_voltage(&mut self, sense: BusVoltageSense, raw: u16) -> ResistancePoll {
        let Some(v_bus_mv) = bus_millivolts(raw, sense) else {
            defmt::error!(
                "resistance sweep: ADC_VM raw {} at or below offset {}",
                raw,
                sense.offset_raw
            );
            return self.complete_without_restore(ResistanceError::BusVoltageOutOfRange);
        };
        if !bus_millivolts_in_range(v_bus_mv) {
            defmt::error!(
                "resistance sweep: bus {} mV outside the supported range",
                v_bus_mv
            );
            return self.complete_without_restore(ResistanceError::BusVoltageOutOfRange);
        }
        let scaled = scale_profile_to_bus(&self.profile, v_bus_mv);
        if !exposure_is_supported(&scaled) {
            return self.complete_without_restore(ResistanceError::UnsupportedProfile);
        }
        if nominal_duration_us(&scaled).is_none_or(|duration| duration > PHASE_DEADLINE_US) {
            return self.complete_without_restore(ResistanceError::PhaseDeadlineExceeded);
        }
        let max_abs_ud = u32::from(scaled.max_abs_ud().unsigned_abs());
        if self.mode == ResistanceRunMode::ExternallyArmed && self.saved_voltage_limit < max_abs_ud
        {
            return self.complete_without_restore(ResistanceError::OutOfBoundsParameter);
        }
        defmt::info!(
            "resistance sweep: bus {} mV, ladder top {} counts",
            v_bus_mv,
            max_abs_ud
        );
        self.profile = scaled;
        self.state = ResistanceState::Setup(0);
        ResistancePoll::ReadyNow
    }

    pub(super) async fn poll_setup<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        operation: u8,
    ) -> ResistancePoll {
        let result = match operation {
            0 => write(backend, MODE_RAMP_MODE_MOTION.addr(), MOTION_STOPPED).await,
            1 => write(backend, UQ_UD_EXT.addr(), 0).await,
            2 => {
                write(
                    backend,
                    PIDOUT_UQ_UD_LIMITS.addr(),
                    u32::from(self.profile.max_abs_ud().unsigned_abs()),
                )
                .await
            }
            3 => write(backend, PWM_SV_CHOP.addr(), self.profile.pwm_sv_chop).await,
            4 => {
                self.verify_read(backend, PWM_SV_CHOP.addr(), self.profile.pwm_sv_chop)
                    .await
            }
            5 => {
                self.verify_read(
                    backend,
                    PWM_MAXCNT.addr(),
                    u32::from(self.profile.pwm_maxcnt),
                )
                .await
            }
            6 => {
                self.verify_read(
                    backend,
                    PWM_BBM_H_BBM_L.addr(),
                    (u32::from(self.profile.bbm_h) << 8) | u32::from(self.profile.bbm_l),
                )
                .await
            }
            7 => {
                self.verify_read(
                    backend,
                    DSADC_MDEC_B_MDEC_A.addr(),
                    (u32::from(self.profile.dsadc_mdec_b) << 16)
                        | u32::from(self.profile.dsadc_mdec_a),
                )
                .await
            }
            8 => match backend.read_register(STATUS_FLAGS.addr()).await {
                Ok(flags) => {
                    self.baseline_status_flags = flags;
                    Ok(())
                }
                Err(_) => Err(ResistanceError::BackendIo),
            },
            9 => {
                crate::tmc_control::interlock::clear_status_flags_via_resistance_backend(backend)
                    .await
            }
            10 => match backend.read_register(STATUS_FLAGS.addr()).await {
                Ok(flags) => {
                    self.record_status_evidence(flags);
                    if flags & current_loop_actionable_status_mask(backend.feedback_config()) != 0 {
                        return self.fail(backend, ResistanceError::CriticalStatus);
                    }
                    Ok(())
                }
                Err(_) => Err(ResistanceError::BackendIo),
            },
            _ => Ok(()),
        };
        if let Err(error) = result {
            return self.fail(backend, error);
        }
        if operation == 0 {
            self.energized = true;
        }
        if operation < 10 {
            self.state = ResistanceState::Setup(operation + 1);
        } else {
            if backend.interlock_tripped() {
                return self.fail(backend, ResistanceError::CriticalStatus);
            }
            if self.mode == ResistanceRunMode::CommissioningManagedEnable {
                backend.set_enable_pin(true);
            }
            self.energized = true;
            self.state = ResistanceState::SelectAxis(0);
        }
        ResistancePoll::ReadyNow
    }

    pub(super) async fn verify_read<B: ResistanceBackend>(
        &self,
        backend: &mut B,
        addr: u8,
        expected: u32,
    ) -> Result<(), ResistanceError> {
        match backend.read_register(addr).await {
            Ok(value) if value == expected => Ok(()),
            Ok(_) => Err(ResistanceError::WrongPowerStage),
            Err(_) => Err(ResistanceError::BackendIo),
        }
    }

    pub(super) async fn select_axis<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        operation: u8,
    ) -> ResistancePoll {
        let axis = self.profile.axes[self.axis_index as usize];
        let result = if operation == 0 {
            write(backend, PHI_E_SELECTION.addr(), 1).await
        } else {
            write(backend, PHI_E_EXT.addr(), u32::from(axis.phi_e_ext)).await
        };
        if let Err(error) = result {
            return self.fail(backend, error);
        }
        if operation == 0 {
            self.state = ResistanceState::SelectAxis(1);
        } else {
            self.begin_window(WindowPurpose::InitialZero, 0, self.profile.zero_settle_ms);
        }
        ResistancePoll::ReadyNow
    }

    pub(super) fn begin_window(&mut self, purpose: WindowPurpose, ud: i16, settle_ms: u16) {
        self.state = ResistanceState::Window(WindowCapture {
            plan: WindowPlan {
                purpose,
                ud,
                settle_ms,
            },
            operation: 0,
            encoder_before: 0,
            current_sum: 0,
            status_or: 0,
            acquisition_samples: 0,
            acquisition_opportunities: 0,
            settle_deadline_us: 0,
            next_sample_us: 0,
            timing: CaptureTiming::new(
                self.profile.sample_interval_ms.saturating_mul(1_000),
                TimingEnvelope::RESISTANCE,
            ),
        });
    }

    pub(super) async fn poll_window<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        mut capture: WindowCapture,
        now_us: u64,
    ) -> ResistancePoll {
        match capture.operation {
            0 => {
                let value = if capture.plan.ud == 0 {
                    0
                } else {
                    pack_ud(capture.plan.ud)
                };
                if let Err(error) = write(backend, UQ_UD_EXT.addr(), value).await {
                    return self.fail(backend, error);
                }
                capture.operation = 1;
            }
            1 => {
                let mode = if capture.plan.ud == 0 {
                    MOTION_STOPPED
                } else {
                    MOTION_UQ_UD_EXT
                };
                if let Err(error) = write(backend, MODE_RAMP_MODE_MOTION.addr(), mode).await {
                    return self.fail(backend, error);
                }
                if capture.plan.ud != 0 {
                    let excitation_start_us = backend.elapsed_us();
                    let settle_us = u64::from(capture.plan.settle_ms) * 1_000;
                    let period_us = u64::from(self.profile.sample_interval_ms) * 1_000;
                    let Some(settle_deadline_us) = excitation_start_us.checked_add(settle_us)
                    else {
                        return self.fail(backend, ResistanceError::PhaseDeadlineExceeded);
                    };
                    let Some(next_sample_us) = excitation_start_us.checked_add(period_us) else {
                        return self.fail(backend, ResistanceError::PhaseDeadlineExceeded);
                    };
                    if settle_deadline_us > self.deadline_us || next_sample_us > self.deadline_us {
                        return self.fail(backend, ResistanceError::PhaseDeadlineExceeded);
                    }
                    capture.settle_deadline_us = settle_deadline_us;
                    capture.next_sample_us = next_sample_us;
                }
                capture.operation = 2;
            }
            2 => match backend.read_register(ABN_DECODER_COUNT.addr()).await {
                Ok(value) => {
                    capture.encoder_before = value;
                    let settle_us = u64::from(capture.plan.settle_ms) * 1_000;
                    let Some(deadline) = now_us.checked_add(settle_us) else {
                        return self.fail(backend, ResistanceError::PhaseDeadlineExceeded);
                    };
                    if deadline > self.deadline_us {
                        return self.fail(backend, ResistanceError::PhaseDeadlineExceeded);
                    }
                    if capture.plan.ud == 0 {
                        capture.settle_deadline_us = deadline;
                        capture.next_sample_us = deadline;
                    }
                    capture.operation = 3;
                    self.state = ResistanceState::Window(capture);
                    return ResistancePoll::At(capture.next_sample_us);
                }
                Err(_) => return self.fail(backend, ResistanceError::BackendIo),
            },
            _ => return self.poll_sample(backend, capture, now_us).await,
        }
        self.state = ResistanceState::Window(capture);
        ResistancePoll::ReadyNow
    }

    pub(super) async fn poll_sample<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        mut capture: WindowCapture,
        now_us: u64,
    ) -> ResistancePoll {
        if now_us < capture.next_sample_us {
            return ResistancePoll::At(capture.next_sample_us);
        }
        let io_start = backend.elapsed_us();
        let packed = match backend.read_register(ADC_IWY_IUX.addr()).await {
            Ok(value) => value,
            Err(_) => {
                return self.fail_window(backend, &capture, ResistanceError::BackendIo, false);
            }
        };
        let axis = self.profile.axes[self.axis_index as usize];
        let current = phase_current_for_axis(axis, packed);
        if capture.plan.ud != 0 {
            self.current.observe_energized_sample(current);
            if current.unsigned_abs() > self.current.current_ceiling_count() {
                self.current.mark_power_stage_tripped();
                backend.trip_power_stage();
                return self.fail_window(
                    backend,
                    &capture,
                    ResistanceError::PeakCurrentExceeded,
                    false,
                );
            }
        }
        let first_done = backend.elapsed_us();
        let flags = match backend.read_register(STATUS_FLAGS.addr()).await {
            Ok(value) => value,
            Err(_) => {
                return self.fail_window(backend, &capture, ResistanceError::BackendIo, false);
            }
        };
        self.record_status_evidence(flags);
        if flags & current_loop_actionable_status_mask(backend.feedback_config()) != 0 {
            return self.fail_window(backend, &capture, ResistanceError::CriticalStatus, false);
        }
        let io_done = backend.elapsed_us();
        let Some(first_read_wall) = first_done.checked_sub(io_start) else {
            return self.fail_window(backend, &capture, ResistanceError::TimingRejected, true);
        };
        let Some(second_read_wall) = io_done.checked_sub(first_done) else {
            return self.fail_window(backend, &capture, ResistanceError::TimingRejected, true);
        };
        let Some(spi_wall) = first_read_wall.checked_add(second_read_wall) else {
            return self.fail_window(backend, &capture, ResistanceError::TimingRejected, true);
        };
        let Ok(spi_wall_u32) = u32::try_from(spi_wall) else {
            return self.fail_window(backend, &capture, ResistanceError::TimingRejected, true);
        };
        let due_us = capture.next_sample_us;
        let slot = match sample_slot(
            due_us,
            self.profile.sample_interval_ms.saturating_mul(1_000),
            now_us,
        ) {
            Ok(slot) => slot,
            Err(_) => {
                return self.fail_window(backend, &capture, ResistanceError::TimingRejected, true);
            }
        };
        capture.status_or |= flags;
        let acquisition_opportunities = match acquisition_opportunities_through(
            due_us,
            slot.sample_deadline_us,
            capture.settle_deadline_us,
            self.profile.sample_interval_ms.saturating_mul(1_000),
        ) {
            Ok(value) => value,
            Err(error) => return self.fail_window(backend, &capture, error, true),
        };
        capture.acquisition_opportunities = capture
            .acquisition_opportunities
            .saturating_add(u8::try_from(acquisition_opportunities).unwrap_or(u8::MAX));
        if slot.sample_deadline_us >= capture.settle_deadline_us {
            capture.current_sum += i32::from(current);
            capture.acquisition_samples += 1;
        }
        capture.next_sample_us = slot.next_deadline_us;
        let completion_us = backend.elapsed_us();
        if completion_us > self.deadline_us {
            return self.fail_window(
                backend,
                &capture,
                ResistanceError::PhaseDeadlineExceeded,
                false,
            );
        }
        if capture
            .timing
            .record_sample(due_us, now_us, io_done, completion_us, spi_wall_u32)
            .is_err()
        {
            return self.fail_window(
                backend,
                &capture,
                ResistanceError::TimingRejected,
                capture.timing.snapshot().1,
            );
        }
        if capture.acquisition_opportunities > max_opportunities_per_window(&self.profile)
            || capture.timing.finish().is_err()
        {
            return self.fail_window(
                backend,
                &capture,
                ResistanceError::TimingRejected,
                capture.timing.snapshot().1,
            );
        }
        if capture.acquisition_samples == self.profile.samples_per_window {
            let evidence = match capture.timing.finish() {
                Ok(evidence) => evidence,
                Err(_) => {
                    return self.fail_window(
                        backend,
                        &capture,
                        ResistanceError::TimingRejected,
                        capture.timing.snapshot().1,
                    );
                }
            };
            self.timing.completed_windows = match self.timing.completed_windows.checked_add(1) {
                Some(value) => value,
                None => {
                    self.timing.overflowed = true;
                    u8::MAX
                }
            };
            self.timing.merge(evidence, false);
            self.state = ResistanceState::ReadEncoderAfter(capture);
            ResistancePoll::ReadyNow
        } else {
            self.state = ResistanceState::Window(capture);
            ResistancePoll::At(capture.next_sample_us)
        }
    }

    fn fail_window<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        capture: &WindowCapture,
        error: ResistanceError,
        source_overflow: bool,
    ) -> ResistancePoll {
        let (evidence, overflowed) = capture.timing.snapshot();
        self.timing.merge(evidence, overflowed || source_overflow);
        self.fail(backend, error)
    }

    fn record_status_evidence(&mut self, flags: u32) {
        self.status_flags_or |= flags;
        if flags & self.profile.status_flags_warn_mask != 0 {
            self.warning_flags
                .insert(ResistanceWarningFlags::STATUS_FLAGS);
        }
    }

    pub(super) async fn read_encoder_after<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        capture: WindowCapture,
    ) -> ResistancePoll {
        let after = match backend.read_register(ABN_DECODER_COUNT.addr()).await {
            Ok(value) => value,
            Err(_) => return self.fail(backend, ResistanceError::BackendIo),
        };
        let delta = wrapped_encoder_delta(
            capture.encoder_before,
            after,
            backend.encoder_counts_per_rev(),
        );
        if capture.plan.ud != 0
            && delta.unsigned_abs() > u32::from(self.profile.encoder_move_warn_counts)
        {
            self.warning_flags
                .insert(ResistanceWarningFlags::ENCODER_MOVED);
        }
        let mean = capture.current_sum / i32::from(self.profile.samples_per_window);
        self.finish_window(backend, capture.plan, mean)
    }
}
