use super::*;
use crate::tmc::{
    CurrentStepAxis, TorqueSampleKind, tmc_current_step_test, tmc_current_torque_sample_test,
    tmc_current_vector_step_test, tmc_position_torque_offset_sample_test, tmc_voltage_step_test,
};
use crate::tmc_control::TmcControlRuntime;
use foci_firmware_macros::fixture_test;
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ABN_DECODER_PPR, ADC_AGPI_A_RAW_ADC_VM_RAW, INTERIM_FOC_IQ_ID,
    INTERIM_FOC_UQ_UD_LIMITED, MODE_RAMP_MODE_MOTION, PID_ERROR_PID_FLUX_ERROR_SUM,
    PID_ERROR_PID_TORQUE_ERROR_SUM, PID_POSITION_ACTUAL, PID_POSITION_TARGET,
    PID_TORQUE_FLUX_ACTUAL, PID_TORQUE_FLUX_LIMITS, PID_TORQUE_FLUX_OFFSET, PID_TORQUE_FLUX_TARGET,
    PIDOUT_UQ_UD_LIMITS, STATUS_FLAGS, UQ_UD_EXT,
};
use foci_tmc4671::spi::Tmc4671Spi;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MockSpiError;

impl embedded_hal_async::spi::Error for MockSpiError {
    fn kind(&self) -> embedded_hal_async::spi::ErrorKind {
        embedded_hal_async::spi::ErrorKind::Other
    }
}

#[derive(Default)]
struct DiagnosticMockSpi {
    writes: std::vec::Vec<(u8, u32)>,
    write_attempts: std::vec::Vec<(u8, u32)>,
    read_requests: std::vec::Vec<u8>,
    reads: std::collections::VecDeque<u32>,
    register_values: std::collections::BTreeMap<u8, u32>,
    last_read_addr: Option<u8>,
    transaction_attempts: usize,
    fail_transaction: Option<usize>,
    fail_writes_to: Option<u8>,
    fail_reads_from: Option<u8>,
    fail_write_when: Option<std::boxed::Box<WriteFailure>>,
}

type WriteFailure = dyn FnMut(usize, u8, u32) -> bool;

impl DiagnosticMockSpi {
    fn with_read(mut self, value: u32) -> Self {
        self.reads.push_back(value);
        self
    }

    fn fail_writes_to(mut self, addr: u8) -> Self {
        self.fail_writes_to = Some(addr);
        self
    }

    fn fail_reads_from(mut self, addr: u8) -> Self {
        self.fail_reads_from = Some(addr);
        self
    }

    /// Fail each write for which `fails(attempt_index, addr, value)` returns true.
    fn fail_write_when(mut self, fails: impl FnMut(usize, u8, u32) -> bool + 'static) -> Self {
        self.fail_write_when = Some(std::boxed::Box::new(fails));
        self
    }

    fn with_register(mut self, register: foci_tmc4671::registers::Register, value: u32) -> Self {
        self.register_values.insert(register.addr(), value);
        self
    }
}

impl embedded_hal_async::spi::ErrorType for DiagnosticMockSpi {
    type Error = MockSpiError;
}

impl embedded_hal_async::spi::SpiDevice<u8> for DiagnosticMockSpi {
    async fn transaction(
        &mut self,
        operations: &mut [embedded_hal_async::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.transaction_attempts += 1;
        let fail_this_transaction = self.fail_transaction == Some(self.transaction_attempts);
        for op in operations {
            match op {
                embedded_hal_async::spi::Operation::Write(buf) if buf.len() == 5 => {
                    let addr = buf[0] & 0x7f;
                    let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    let attempt = self.write_attempts.len();
                    self.write_attempts.push((addr, value));
                    let predicate_fails = self
                        .fail_write_when
                        .as_mut()
                        .is_some_and(|fails| fails(attempt, addr, value));
                    if fail_this_transaction || self.fail_writes_to == Some(addr) || predicate_fails
                    {
                        return Err(MockSpiError);
                    }
                    self.writes.push((addr, value));
                    self.register_values.insert(addr, value);
                }
                embedded_hal_async::spi::Operation::Write(buf) if buf.len() == 1 => {
                    let addr = buf[0];
                    self.read_requests.push(addr);
                    self.last_read_addr = Some(addr);
                    if fail_this_transaction || self.fail_reads_from == Some(addr) {
                        return Err(MockSpiError);
                    }
                }
                embedded_hal_async::spi::Operation::Read(buf) => {
                    let value = self.reads.pop_front().unwrap_or_else(|| {
                        self.last_read_addr
                            .and_then(|addr| self.register_values.get(&addr).copied())
                            .unwrap_or(0)
                    });
                    buf.copy_from_slice(&value.to_be_bytes());
                }
                embedded_hal_async::spi::Operation::Write(_)
                | embedded_hal_async::spi::Operation::DelayNs(_) => {}
                embedded_hal_async::spi::Operation::Transfer(read, write) => {
                    read.copy_from_slice(write);
                }
                embedded_hal_async::spi::Operation::TransferInPlace(_) => {}
            }
        }
        Ok(())
    }
}

#[derive(Default)]
struct MockDelay {
    delays_ms: std::vec::Vec<u64>,
    delays_us: std::vec::Vec<u32>,
    now_us: core::cell::Cell<u64>,
    advance_on_delay: bool,
    advance_on_now_us: u64,
}

#[derive(Default)]
struct MockDiagnosticPowerStage {
    trip_calls: u8,
    motor_armed_at_trip: bool,
}

impl DiagnosticPowerStage for MockDiagnosticPowerStage {
    fn trip_power_stage(&mut self) {
        self.trip_calls = self.trip_calls.saturating_add(1);
        self.motor_armed_at_trip = crate::state::tmc::tmc_channel0()
            .motor_armed
            .load(core::sync::atomic::Ordering::Relaxed);
    }
}

impl DiagnosticDelay for MockDelay {
    async fn delay_ms(&mut self, ms: u64) {
        self.delays_ms.push(ms);
        if self.advance_on_delay {
            self.now_us
                .set(self.now_us.get().wrapping_add(ms.saturating_mul(1_000)));
        }
    }

    async fn delay_us(&mut self, us: u32) {
        self.delays_us.push(us);
        if self.advance_on_delay {
            self.now_us
                .set(self.now_us.get().wrapping_add(u64::from(us)));
        }
    }

    fn now_us(&self) -> u64 {
        let now_us = self.now_us.get();
        self.now_us.set(now_us.wrapping_add(self.advance_on_now_us));
        now_us
    }
}

/// Board config fixture for diagnostics that need `DiagnosticContext::tmc_config`.
/// Mirrors a board's own `const TMC_CONFIG`; values match
/// `crate::config::TmcConfig::default_for_test`.
static TEST_TMC_CONFIG: crate::config::TmcConfig = crate::config::TmcConfig {
    pwm_maxcnt: 3999,
    tmc_clock_hz: 25_000_000,
    bbm_h: 9,
    bbm_l: 9,
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
    inductance_voltage: 768,
    resistance_scale_metadata_validated: true,
    current_scale_ma_per_lsb: 1.155,
    max_run_current_ma: 10_000,
    vm_divider_high_ohms: 71_500,
    vm_divider_low_ohms: 1_500,
    vm_adc_reference_millivolts: 2_500,
    vm_adc_center_counts: 32_767,
    motor_type_n_pole_pairs: crate::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS,
};

fn ready_runtime() -> TmcControlRuntime {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_calibrated_for_test(true);
    runtime
}

#[test]
fn pack_voltage_ext_preserves_signed_halves() {
    assert_eq!(pack_voltage_ext(0x1234, -2), 0x1234_fffe);
    assert_eq!(pack_voltage_ext(-1, 0x4567), 0xffff_4567);
}

#[test]
fn empty_reports_preserve_headers_and_zero_samples() {
    let torque = empty_current_torque_sample_report(
        7,
        TorqueSampleKind::CurrentTorqueSample,
        CURRENT_STEP_STATUS_NOT_READY,
        -11,
        22,
        3,
        44,
    );
    assert_eq!(torque.oid, 7);
    assert_eq!(torque.kind, TorqueSampleKind::CurrentTorqueSample);
    assert_eq!(torque.status, CURRENT_STEP_STATUS_NOT_READY);
    assert_eq!(torque.target, -11);
    assert_eq!(torque.flux_target, 22);
    assert_eq!(torque.sample_delay_ms, 3);
    assert_eq!(torque.voltage_limit, 44);
    assert_eq!(torque.torque_sample, 0);
    assert_eq!(torque.status_flags, 0);

    let voltage = empty_voltage_step_report(8, CURRENT_STEP_STATUS_SPI, -33, 44, 5);
    assert_eq!(voltage.oid, 8);
    assert_eq!(voltage.status, CURRENT_STEP_STATUS_SPI);
    assert_eq!(voltage.uq_ext, -33);
    assert_eq!(voltage.ud_ext, 44);
    assert_eq!(voltage.sample_delay_ms, 5);
    assert_eq!(voltage.encoder_delta_after, 0);

    let resistance = empty_resistance_run_report(9, CURRENT_STEP_STATUS_SPI);
    assert_eq!(resistance.oid, 9);
    assert_eq!(resistance.status, CURRENT_STEP_STATUS_SPI);
    assert_eq!(resistance.peak_abs_current_count, 0);
    assert_eq!(resistance.max_abs_steady_mean_current_count, 0);
    assert_eq!(resistance.current_ceiling_count, 0);
    assert!(!resistance.power_stage_tripped);
}

#[test]
fn failed_resistance_report_retains_current_evidence() {
    let mut current = crate::resistance_identification::ResistanceCurrentEvidence::new(1_600);
    current.observe_energized_sample(-1_800);
    current.record_completed_energized_mean(-900);
    current.mark_power_stage_tripped();

    let report = super::resistance::failed_run_report(
        7,
        crate::resistance_identification::ResistanceError::BackendIo.status_code(),
        current,
    );

    assert_eq!(report.oid, 7);
    assert_eq!(
        report.status,
        crate::resistance_identification::ResistanceError::BackendIo.status_code()
    );
    assert_eq!(report.peak_abs_current_count, 1_800);
    assert_eq!(report.max_abs_steady_mean_current_count, 900);
    assert_eq!(report.current_ceiling_count, 1_600);
    assert!(report.power_stage_tripped);
}

#[test]
fn commissioning_busy_diagnostic_returns_not_ready_without_hardware() {
    let request = crate::tmc::TmcRequest::new(
        0,
        crate::tmc::TmcRequestKind::CurrentStepTest {
            oid: 9,
            axis: crate::tmc::CurrentStepAxis::Torque,
            target: 64,
            duration_ms: 5,
            voltage_limit: 512,
        },
    );

    let outcome =
        super::busy_diagnostic_report(request, &crate::config::TmcConfig::default_for_test());

    let super::DiagnosticOutcome::Handled(super::DiagnosticReport::CurrentStep { reply, report }) =
        outcome
    else {
        panic!("expected scalar current-step busy report");
    };
    assert_eq!(
        reply,
        super::CurrentStepReply::Scalar(crate::tmc::CurrentStepAxis::Torque)
    );
    assert_eq!(report.oid, 9);
    assert_eq!(report.status, super::CURRENT_STEP_STATUS_NOT_READY);
    assert_eq!(report.torque_target, 64);
}

#[test]
fn current_step_not_ready_returns_status_without_writes() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 123, 4, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 999,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, report }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(reply, CurrentStepReply::Scalar(CurrentStepAxis::Torque));
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.oid, 7);
        assert_eq!(report.torque_target, 123);
        assert!(tmc.into_inner().writes.is_empty());
        assert!(delay.delays_ms.is_empty());
    });
}

#[test]
fn diagnostic_resistance_adapter_derives_the_bus_sense_from_the_readiness_offset() {
    use crate::resistance_identification::ResistanceBackend;

    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let mut delay = MockDelay::default();
    let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
    let mut power_stage = MockDiagnosticPowerStage::default();
    let mut ctx = DiagnosticContext {
        host_liveness_expired: None,
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 16_000,
            adc_vm_offset: Some(crate::state::tmc::AdcVmOffsetSnapshot {
                offset_raw: 33_671,
                sample_count: 8,
            }),
        },
    };

    let adapter = super::resistance::DiagnosticResistanceBackend::new(&mut ctx);
    let sense = adapter.bus_voltage_sense().expect("offset present");

    assert_eq!(sense.offset_raw, 33_671);
    assert_eq!(sense.counts_per_volt_milli, 269_318);
}

#[test]
fn diagnostic_resistance_adapter_has_no_bus_sense_without_an_offset() {
    use crate::resistance_identification::ResistanceBackend;

    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let mut delay = MockDelay::default();
    let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
    let mut power_stage = MockDiagnosticPowerStage::default();
    let mut ctx = DiagnosticContext {
        host_liveness_expired: None,
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 16_000,
            adc_vm_offset: None,
        },
    };

    let adapter = super::resistance::DiagnosticResistanceBackend::new(&mut ctx);

    assert_eq!(adapter.bus_voltage_sense(), None);
}

#[test]
fn current_vector_uses_vector_report_flag() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = tmc_current_vector_step_test(0, 9, 11, -12, 4, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: true,
                    stepper_active: false,
                    saved_voltage_limit: 999,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, report }) = outcome
        else {
            panic!("expected current vector report");
        };
        assert_eq!(reply, CurrentStepReply::Vector);
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.torque_target, 11);
        assert_eq!(report.flux_target, -12);
    });
}

#[test]
fn current_step_success_records_settle_delays_and_restores_limit() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0)
            .with_read(0xaaaa_bbbb)
            .with_read(100)
            .with_read(1000)
            .with_read(0x0005_0000)
            .with_read(0x0009_000a)
            .with_read(0x000b_000c)
            .with_read(0x000d_000e)
            .with_read(0x0000_1234)
            .with_read(120)
            .with_read(0x0006_0000);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_step_request(
            tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 20, 3, 555),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, 0);
        assert_eq!(report.encoder_before, 100);
        assert_eq!(report.encoder_after, 120);
        assert_eq!(report.encoder_delta, 20);
        assert_eq!(report.adc_vm_raw, 0x1234);
        assert_eq!(delay.delays_ms, std::vec![20, 3, 20]);

        let raw_spi = tmc.into_inner();
        let expected_writes = std::vec![
            (PID_TORQUE_FLUX_TARGET.addr(), 0),
            (PIDOUT_UQ_UD_LIMITS.addr(), 555),
            (
                PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr(),
                u32::from(PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr),
            ),
            (PID_ERROR_PID_TORQUE_ERROR_SUM.data.addr(), 0),
            (
                PID_ERROR_PID_FLUX_ERROR_SUM.addr.addr(),
                u32::from(PID_ERROR_PID_FLUX_ERROR_SUM.sub_addr),
            ),
            (PID_ERROR_PID_FLUX_ERROR_SUM.data.addr(), 0),
            (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0001),
            (
                PID_TORQUE_FLUX_TARGET.addr(),
                crate::tmc::pack_torque_flux_target(20, 0),
            ),
            (
                INTERIM_FOC_IQ_ID.addr.addr(),
                u32::from(INTERIM_FOC_IQ_ID.sub_addr),
            ),
            (
                INTERIM_FOC_UQ_UD_LIMITED.addr.addr(),
                u32::from(INTERIM_FOC_UQ_UD_LIMITED.sub_addr),
            ),
            (
                ADC_AGPI_A_RAW_ADC_VM_RAW.addr.addr(),
                u32::from(ADC_AGPI_A_RAW_ADC_VM_RAW.sub_addr),
            ),
            (PID_TORQUE_FLUX_TARGET.addr(), 0),
            (PID_TORQUE_FLUX_TARGET.addr(), 0),
            (PIDOUT_UQ_UD_LIMITS.addr(), 777),
            (PID_POSITION_TARGET.addr(), 0xaaaa_bbbb),
            (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
        ];
        assert_eq!(raw_spi.write_attempts, expected_writes);
        assert_eq!(raw_spi.writes, raw_spi.write_attempts);
        assert_eq!(
            raw_spi.read_requests,
            std::vec![
                MODE_RAMP_MODE_MOTION.addr(),
                PID_POSITION_ACTUAL.addr(),
                ABN_DECODER_COUNT.addr(),
                ABN_DECODER_PPR.addr(),
                PID_TORQUE_FLUX_ACTUAL.addr(),
                PID_TORQUE_FLUX_ACTUAL.addr(),
                INTERIM_FOC_IQ_ID.data.addr(),
                INTERIM_FOC_UQ_UD_LIMITED.data.addr(),
                ADC_AGPI_A_RAW_ADC_VM_RAW.data.addr(),
                ABN_DECODER_COUNT.addr(),
                PID_TORQUE_FLUX_ACTUAL.addr(),
            ]
        );
    });
}

#[test]
fn flux_current_step_drives_and_reports_flux_axis() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0)
            .with_read(0xaaaa_bbbb)
            .with_read(100)
            .with_read(1000)
            .with_read(0x0005_fffe)
            .with_read(0x0009_0014)
            .with_read(0x000b_000c)
            .with_read(0x000d_000e)
            .with_read(0x0000_1234)
            .with_read(120)
            .with_read(0x0006_fff9);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_step_request(
            tmc_current_step_test(0, 7, CurrentStepAxis::Flux, 20, 3, 555),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, report }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(reply, CurrentStepReply::Scalar(CurrentStepAxis::Flux));
        assert_eq!(report.status, 0);
        assert_eq!(report.torque_target, 20);
        assert_eq!(report.flux_target, 20);
        assert_eq!(report.torque_before, -2);
        assert_eq!(report.torque_during, 20);
        assert_eq!(report.torque_after, -7);
        assert_eq!(report.flux_during, 20);

        let raw_spi = tmc.into_inner();
        assert!(raw_spi.write_attempts.contains(&(
            PID_TORQUE_FLUX_TARGET.addr(),
            crate::tmc::pack_torque_flux_target(0, 20),
        )));
    });
}

#[test]
fn busy_flux_current_step_reports_flux_axis() {
    let outcome = busy_diagnostic_report(
        tmc_current_step_test(0, 7, CurrentStepAxis::Flux, 20, 30, 555),
        &TEST_TMC_CONFIG,
    );

    let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, report }) = outcome
    else {
        panic!("expected current step report");
    };
    assert_eq!(reply, CurrentStepReply::Scalar(CurrentStepAxis::Flux));
    assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
}

#[test]
fn busy_current_vector_step_uses_vector_reply() {
    let outcome = busy_diagnostic_report(
        tmc_current_vector_step_test(0, 7, 20, -10, 30, 555),
        &TEST_TMC_CONFIG,
    );

    let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, .. }) = outcome else {
        panic!("expected current step report");
    };
    assert_eq!(reply, CurrentStepReply::Vector);
}

#[test]
fn busy_torque_samples_report_their_kind() {
    for (req, kind) in [
        (
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            TorqueSampleKind::CurrentTorqueSample,
        ),
        (
            tmc_position_torque_offset_sample_test(0, 4, 22, 3, 444),
            TorqueSampleKind::PositionTorqueOffset,
        ),
    ] {
        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) =
            busy_diagnostic_report(req, &TEST_TMC_CONFIG)
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.kind, kind);
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
    }
}

#[test]
fn current_step_invalid_ppr_aborts_before_excitation() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0)
            .with_read(0xaaaa_bbbb)
            .with_read(100)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_step_request(
            tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 20, 3, 555),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        assert!(!raw_spi.write_attempts.contains(&(
            PID_TORQUE_FLUX_TARGET.addr(),
            crate::tmc::pack_torque_flux_target(20, 0),
        )));
    });
}

#[test]
fn current_step_exact_half_turn_aborts_via_post_measurement_error() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0)
            .with_read(0xaaaa_bbbb)
            .with_read(100)
            .with_read(1000)
            .with_read(0x0005_0000)
            .with_read(0x0009_000a)
            .with_read(0x000b_000c)
            .with_read(0x000d_000e)
            .with_read(0x0000_1234)
            .with_read(600);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_step_request(
            tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 20, 3, 555),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
    });
}

#[test]
fn current_step_host_liveness_expiry_reports_dedicated_status_and_restores() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(555)
            .with_read(100)
            .with_read(1000)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut calls = 0u32;
        let mut expire_during_main_delay_final_slice = || {
            calls += 1;
            calls >= 5
        };
        let req = tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 123, 150, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_during_main_delay_final_slice),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20, 100, 50]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_TARGET.addr(), 0),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (PID_POSITION_TARGET.addr(), 555),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn current_step_pre_settle_host_liveness_expiry_reports_dedicated_status() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(555);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut expire_immediately = || true;
        let req = tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 123, 150, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_immediately),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert!(delay.delays_ms.is_empty());
    });
}

#[test]
fn current_step_post_settle_host_liveness_expiry_reports_dedicated_status() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(555)
            .with_read(100)
            .with_read(1000)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut calls = 0u32;
        let mut expire_at_post_settle = || {
            calls += 1;
            calls >= 5
        };
        let req = tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 123, 20, 500);

        let outcome = run_current_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_at_post_settle),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) = outcome
        else {
            panic!("expected current step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20, 20]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_TARGET.addr(), 0),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (PID_POSITION_TARGET.addr(), 555),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn torque_sample_not_ready_does_not_emit_detail() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());

        let outcome = run_current_torque_sample_request(
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.kind, TorqueSampleKind::CurrentTorqueSample);
    });
}

#[test]
fn current_torque_sample_invalid_ppr_aborts_before_excitation() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0)
            .with_read(0xaaaa_bbbb)
            .with_read(100)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_torque_sample_request(
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        assert!(!raw_spi.write_attempts.contains(&(
            PID_TORQUE_FLUX_TARGET.addr(),
            crate::tmc::pack_torque_flux_target(22, 0),
        )));
    });
}

#[test]
fn current_torque_sample_exact_half_turn_aborts_via_post_measurement_error() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0)
            .with_read(0xaaaa_bbbb)
            .with_read(100)
            .with_read(1000)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(600);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_torque_sample_request(
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
    });
}

#[test]
fn current_torque_sample_success_reports_current_torque_sample_kind() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0)
            .with_read(0xaaaa_bbbb)
            .with_read(100)
            .with_read(1000)
            .with_read(0x0005_0000)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(110)
            .with_read(0x0009_000a)
            .with_read(0x000b_000c)
            .with_read(0x000d_000e)
            .with_read(0)
            .with_read(0)
            .with_read(0x000f_0010)
            .with_read(0x0000_1234)
            .with_read(120)
            .with_read(0x0006_0000);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_current_torque_sample_request(
            tmc_current_torque_sample_test(0, 4, 22, 0, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, 0);
        assert_eq!(report.kind, TorqueSampleKind::CurrentTorqueSample);
    });
}

#[fixture_test]
fn position_offset_rejects_non_position_mode() {
    pollster::block_on(async {
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let raw_spi = DiagnosticMockSpi::default().with_read(0x8000_0001);
        let mut tmc = Tmc4671Spi::new(raw_spi);

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.kind, TorqueSampleKind::PositionTorqueOffset);
        assert!(
            tmc.into_inner()
                .read_requests
                .contains(&MODE_RAMP_MODE_MOTION.addr())
        );
    });
}

#[fixture_test]
fn position_offset_spi_failure_attempts_offset_restore() {
    pollster::block_on(async {
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(0x0000_5678)
            .fail_writes_to(PIDOUT_UQ_UD_LIMITS.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![
                (PIDOUT_UQ_UD_LIMITS.addr(), 444),
                (PID_TORQUE_FLUX_OFFSET.addr(), 0x0000_5678),
                (PID_TORQUE_FLUX_LIMITS.addr(), OFFSET_SAMPLE_LIMIT),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
        assert_eq!(
            raw_spi.writes,
            std::vec![
                (PID_TORQUE_FLUX_OFFSET.addr(), 0x0000_5678),
                (PID_TORQUE_FLUX_LIMITS.addr(), OFFSET_SAMPLE_LIMIT),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
        assert_eq!(
            raw_spi.read_requests,
            std::vec![MODE_RAMP_MODE_MOTION.addr(), PID_TORQUE_FLUX_OFFSET.addr()]
        );
    });
}

#[fixture_test]
fn position_torque_offset_sample_invalid_ppr_aborts_before_excitation() {
    pollster::block_on(async {
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(0x0000_5678)
            .with_read(100)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        let offset_reg = u32::from(33i16 as u16) << 16;
        assert!(
            !raw_spi
                .write_attempts
                .contains(&(PID_TORQUE_FLUX_OFFSET.addr(), offset_reg))
        );
    });
}

#[fixture_test]
fn position_torque_offset_sample_exact_half_turn_aborts_via_post_measurement_error() {
    pollster::block_on(async {
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(0x0000_5678)
            .with_read(100)
            .with_read(1000)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(600);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
    });
}

#[fixture_test]
fn position_torque_offset_sample_success_reports_position_torque_offset_kind() {
    pollster::block_on(async {
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(0x0000_5678)
            .with_read(100)
            .with_read(1000)
            .with_read(0x0005_0000)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(0)
            .with_read(110)
            .with_read(0x0009_000a)
            .with_read(0x000b_000c)
            .with_read(0x000d_000e)
            .with_read(0)
            .with_read(0)
            .with_read(0x000f_0010)
            .with_read(0x0000_1234)
            .with_read(120)
            .with_read(0x0006_0000);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, 0);
        assert_eq!(report.kind, TorqueSampleKind::PositionTorqueOffset);
    });
}

#[test]
fn current_torque_sample_host_liveness_expiry_reports_dedicated_status_and_restores() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(555)
            .with_read(100)
            .with_read(1000)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut calls = 0u32;
        let mut expire_during_main_delay_final_slice = || {
            calls += 1;
            calls >= 5
        };
        let req = tmc_current_torque_sample_test(0, 7, 100, 50, 150, 500);

        let outcome = run_current_torque_sample_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_during_main_delay_final_slice),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20, 100, 50]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_TARGET.addr(), 0),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (PID_POSITION_TARGET.addr(), 555),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[fixture_test]
fn position_offset_host_liveness_expiry_reports_dedicated_status_and_restores() {
    pollster::block_on(async {
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(0x0000_5678)
            .with_read(100)
            .with_read(1000)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut calls = 0u32;
        let mut expire_during_main_delay_final_slice = || {
            calls += 1;
            calls >= 3
        };

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 150, 444),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_during_main_delay_final_slice),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![100, 50]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_OFFSET.addr(), 0x0000_5678),
                (PID_TORQUE_FLUX_LIMITS.addr(), OFFSET_SAMPLE_LIMIT),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn current_torque_sample_pre_settle_host_liveness_expiry_reports_dedicated_status() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(555);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut expire_immediately = || true;
        let req = tmc_current_torque_sample_test(0, 7, 100, 50, 150, 500);

        let outcome = run_current_torque_sample_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_immediately),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert!(delay.delays_ms.is_empty());
    });
}

#[test]
fn current_torque_sample_post_settle_host_liveness_expiry_reports_dedicated_status() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(555)
            .with_read(100)
            .with_read(1000)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut calls = 0u32;
        let mut expire_at_post_settle = || {
            calls += 1;
            calls >= 5
        };
        let req = tmc_current_torque_sample_test(0, 7, 100, 50, 20, 500);

        let outcome = run_current_torque_sample_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_at_post_settle),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected torque sample report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20, 20]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_TARGET.addr(), 0),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (PID_POSITION_TARGET.addr(), 555),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[fixture_test]
fn position_offset_post_settle_host_liveness_expiry_reports_dedicated_status() {
    pollster::block_on(async {
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(0x0000_5678)
            .with_read(100)
            .with_read(1000)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut calls = 0u32;
        let mut expire_at_post_settle = || {
            calls += 1;
            calls >= 3
        };

        let outcome = run_position_torque_offset_sample_request(
            tmc_position_torque_offset_sample_test(0, 4, 33, 20, 444),
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_at_post_settle),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome
        else {
            panic!("expected position offset report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
        assert_eq!(
            restore_tail,
            [
                (PID_TORQUE_FLUX_OFFSET.addr(), 0x0000_5678),
                (PID_TORQUE_FLUX_LIMITS.addr(), OFFSET_SAMPLE_LIMIT),
                (PIDOUT_UQ_UD_LIMITS.addr(), 777),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

const OFFSET_SAMPLE_LIMIT: u32 = 1_000;

fn store_channel0_limit(raw: u32) {
    crate::state::tmc::tmc_channel0().store_current_limit(
        foci_tmc4671::current_limit::PidTorqueFluxLimit::new(raw).expect("valid limit"),
        core::sync::atomic::Ordering::Relaxed,
    );
}

fn offset_sample_spi() -> DiagnosticMockSpi {
    DiagnosticMockSpi::default()
        .with_register(
            MODE_RAMP_MODE_MOTION,
            crate::tmc_register_values::MOTION_POSITION_ADVANCED_PI,
        )
        .with_register(ABN_DECODER_PPR, 1_000)
}

fn offset_register(target: i16) -> u32 {
    u32::from(target as u16) << 16
}

fn limit_and_offset_writes(log: &[(u8, u32)]) -> std::vec::Vec<(u8, u32)> {
    log.iter()
        .copied()
        .filter(|(addr, _)| {
            *addr == PID_TORQUE_FLUX_LIMITS.addr() || *addr == PID_TORQUE_FLUX_OFFSET.addr()
        })
        .collect()
}

async fn run_offset_sample(
    tmc: &mut Tmc4671Spi<DiagnosticMockSpi>,
    runtime: &mut TmcControlRuntime,
    target: i16,
) -> CurrentTorqueSampleReport {
    let mut delay = MockDelay::default();
    let outcome = run_position_torque_offset_sample_request(
        tmc_position_torque_offset_sample_test(0, 4, target, 3, 444),
        DiagnosticContext {
            host_liveness_expired: None,
            tmc,
            runtime,
            delay: &mut delay,
            power_stage: &mut MockDiagnosticPowerStage::default(),
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
            readiness: DiagnosticReadiness {
                homing_active: false,
                stepper_active: false,
                saved_voltage_limit: 777,
                adc_vm_offset: None,
            },
        },
    )
    .await;
    let DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) = outcome else {
        panic!("expected position offset report");
    };
    report
}

async fn maintenance_tick(
    tmc: &mut Tmc4671Spi<DiagnosticMockSpi>,
    runtime: &mut TmcControlRuntime,
) -> Result<(), ()> {
    let tmc_config = crate::config::TmcConfig::default_for_test();
    let config = crate::tmc_control::TmcControlConfig::new(
        84_000_000,
        tmc_config.pwm_freq_hz(),
        0.03125,
        tmc_config,
    );
    let refs = crate::tmc_control::TmcControlStateRefs::new(
        crate::state::tmc::tmc_channel0(),
        crate::state::stepper::stepper_channel0(),
    );
    runtime
        .apply_feedforward_tick(tmc, refs, &config, false, false, true)
        .await
}

fn assert_offset_then_limit_restored(log: &[(u8, u32)], saved: u32) {
    let pairs = limit_and_offset_writes(log);
    let last_offset = pairs
        .iter()
        .rposition(|(addr, _)| *addr == PID_TORQUE_FLUX_OFFSET.addr())
        .unwrap_or_else(|| panic!("no offset restore in {pairs:?}"));
    let last_limit = pairs
        .iter()
        .rposition(|(addr, _)| *addr == PID_TORQUE_FLUX_LIMITS.addr())
        .unwrap_or_else(|| panic!("no limit restore in {pairs:?}"));
    assert_eq!(pairs[last_offset].1, saved, "{pairs:?}");
    assert_eq!(pairs[last_limit].1, OFFSET_SAMPLE_LIMIT, "{pairs:?}");
    assert!(last_offset < last_limit, "{pairs:?}");
}

#[fixture_test]
async fn offset_diagnostic_lowers_the_limit_before_writing_the_offset() {
    store_channel0_limit(OFFSET_SAMPLE_LIMIT);
    let mut runtime = ready_runtime();
    let mut tmc = Tmc4671Spi::new(offset_sample_spi());

    let report = run_offset_sample(&mut tmc, &mut runtime, 300).await;

    assert_eq!(report.status, 0);
    let spi = tmc.into_inner();
    assert_eq!(
        limit_and_offset_writes(&spi.write_attempts),
        std::vec![
            (PID_TORQUE_FLUX_LIMITS.addr(), 700),
            (PID_TORQUE_FLUX_OFFSET.addr(), offset_register(300)),
            (PID_TORQUE_FLUX_OFFSET.addr(), 0),
            (PID_TORQUE_FLUX_OFFSET.addr(), 0),
            (PID_TORQUE_FLUX_LIMITS.addr(), OFFSET_SAMPLE_LIMIT),
        ]
    );
    crate::tmc_control::test_spi::assert_limit_plus_offset_within(&spi.writes, OFFSET_SAMPLE_LIMIT);
    assert!(!runtime.offset_headroom_engaged());
}

#[fixture_test]
async fn offset_diagnostic_rejects_targets_above_the_limit() {
    for target in [1_001, -1_001, i16::MIN] {
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let mut runtime = ready_runtime();
        let mut tmc = Tmc4671Spi::new(offset_sample_spi());

        let report = run_offset_sample(&mut tmc, &mut runtime, target).await;

        assert_eq!(
            report.status, CURRENT_STEP_STATUS_NOT_READY,
            "target {target}"
        );
        let spi = tmc.into_inner();
        assert!(
            spi.write_attempts.is_empty(),
            "target {target}: {:?}",
            spi.write_attempts
        );
    }
    for target in [1_000, -1_000] {
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let mut runtime = ready_runtime();
        let mut tmc = Tmc4671Spi::new(offset_sample_spi());

        let report = run_offset_sample(&mut tmc, &mut runtime, target).await;

        assert_eq!(report.status, 0, "target {target}");
        let spi = tmc.into_inner();
        let pairs = limit_and_offset_writes(&spi.writes);
        assert_eq!(
            pairs[..2],
            [
                (PID_TORQUE_FLUX_LIMITS.addr(), 0),
                (PID_TORQUE_FLUX_OFFSET.addr(), offset_register(target)),
            ],
            "target {target}"
        );
        crate::tmc_control::test_spi::assert_limit_plus_offset_within(
            &spi.writes,
            OFFSET_SAMPLE_LIMIT,
        );
        assert_offset_then_limit_restored(&spi.writes, 0);
    }
}

#[fixture_test]
async fn offset_diagnostic_restores_on_every_failure_path() {
    let mut baseline = Tmc4671Spi::new(offset_sample_spi());
    store_channel0_limit(OFFSET_SAMPLE_LIMIT);
    let _ = run_offset_sample(&mut baseline, &mut ready_runtime(), 300).await;
    let baseline_writes = baseline.into_inner().write_attempts;

    let single_failures = (0..baseline_writes.len()).map(|failed| {
        (
            std::format!("write {failed} {:?} fails once", baseline_writes[failed]),
            offset_sample_spi().fail_write_when(move |attempt, _, _| attempt == failed),
        )
    });
    let persistent_failures = baseline_writes.iter().map(|&(addr, _)| {
        (
            std::format!("every write to {addr:#04x} fails"),
            offset_sample_spi().fail_writes_to(addr),
        )
    });
    let read_failure = core::iter::once((
        std::string::String::from("mid-run STATUS_FLAGS read fails"),
        offset_sample_spi().fail_reads_from(STATUS_FLAGS.addr()),
    ));

    for (case, spi) in single_failures
        .chain(persistent_failures)
        .chain(read_failure)
    {
        crate::state::tmc::reset_all_tmc_channels();
        store_channel0_limit(OFFSET_SAMPLE_LIMIT);
        let mut runtime = ready_runtime();
        let mut tmc = Tmc4671Spi::new(spi);

        let _ = run_offset_sample(&mut tmc, &mut runtime, 300).await;

        let spi = tmc.inner();
        crate::tmc_control::test_spi::assert_limit_plus_offset_within(
            &spi.writes,
            OFFSET_SAMPLE_LIMIT,
        );
        let limit_or_offset_blocked = [PID_TORQUE_FLUX_LIMITS, PID_TORQUE_FLUX_OFFSET]
            .iter()
            .any(|register| spi.fail_writes_to == Some(register.addr()));
        if limit_or_offset_blocked {
            assert!(runtime.offset_headroom_engaged(), "{case}");
            continue;
        }
        if runtime.offset_headroom_engaged() {
            maintenance_tick(&mut tmc, &mut runtime)
                .await
                .unwrap_or_else(|()| panic!("{case}: recovery tick failed"));
        }
        let spi = tmc.into_inner();
        assert_offset_then_limit_restored(&spi.writes, 0);
        crate::tmc_control::test_spi::assert_limit_plus_offset_within(
            &spi.writes,
            OFFSET_SAMPLE_LIMIT,
        );
        assert!(!runtime.offset_headroom_engaged(), "{case}");
    }
}

#[fixture_test]
async fn offset_diagnostic_never_restores_the_limit_after_a_failed_offset_clear() {
    let target_offset = offset_register(300);
    let lowered = (PID_TORQUE_FLUX_LIMITS.addr(), 700);
    let target_write = (PID_TORQUE_FLUX_OFFSET.addr(), target_offset);
    let clear = (PID_TORQUE_FLUX_OFFSET.addr(), 0);
    let full_limit = (PID_TORQUE_FLUX_LIMITS.addr(), OFFSET_SAMPLE_LIMIT);

    store_channel0_limit(OFFSET_SAMPLE_LIMIT);
    let mut runtime = ready_runtime();
    let mut clear_failures = 0;
    let transient = offset_sample_spi()
        .fail_reads_from(STATUS_FLAGS.addr())
        .fail_write_when(move |_, addr, value| {
            let fails = addr == PID_TORQUE_FLUX_OFFSET.addr() && value == 0 && clear_failures == 0;
            clear_failures += usize::from(fails);
            fails
        });
    let mut tmc = Tmc4671Spi::new(transient);

    let report = run_offset_sample(&mut tmc, &mut runtime, 300).await;

    assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
    let spi = tmc.into_inner();
    assert_eq!(
        limit_and_offset_writes(&spi.write_attempts),
        std::vec![lowered, target_write, clear, clear, full_limit]
    );
    assert_eq!(
        limit_and_offset_writes(&spi.writes),
        std::vec![lowered, target_write, clear, full_limit]
    );
    assert!(!runtime.offset_headroom_engaged());

    crate::state::tmc::reset_all_tmc_channels();
    store_channel0_limit(OFFSET_SAMPLE_LIMIT);
    let mut runtime = ready_runtime();
    let clear_blocked = std::rc::Rc::new(core::cell::Cell::new(true));
    let blocked = clear_blocked.clone();
    let persistent = offset_sample_spi()
        .fail_reads_from(STATUS_FLAGS.addr())
        .fail_write_when(move |_, addr, value| {
            blocked.get() && addr == PID_TORQUE_FLUX_OFFSET.addr() && value == 0
        });
    let mut tmc = Tmc4671Spi::new(persistent);

    let report = run_offset_sample(&mut tmc, &mut runtime, 300).await;

    assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
    let attempts = limit_and_offset_writes(&tmc.inner().write_attempts);
    assert_eq!(attempts[..2], [lowered, target_write]);
    assert!(
        attempts[2..].iter().all(|write| *write == clear),
        "{attempts:?}"
    );
    assert!(attempts.len() > 3, "the clear is retried: {attempts:?}");
    assert_eq!(
        limit_and_offset_writes(&tmc.inner().writes),
        std::vec![lowered, target_write]
    );
    assert!(runtime.offset_headroom_engaged());
    assert!(
        crate::state::tmc::tmc_channel0()
            .offset_headroom_reserved(core::sync::atomic::Ordering::Relaxed)
    );

    for _ in 0..3 {
        assert!(maintenance_tick(&mut tmc, &mut runtime).await.is_err());
    }
    assert!(!tmc.inner().write_attempts.contains(&full_limit));
    assert!(runtime.offset_headroom_engaged());

    clear_blocked.set(false);
    maintenance_tick(&mut tmc, &mut runtime).await.unwrap();

    let spi = tmc.into_inner();
    assert_eq!(
        limit_and_offset_writes(&spi.writes),
        std::vec![lowered, target_write, clear, full_limit]
    );
    assert!(!runtime.offset_headroom_engaged());
    assert!(
        !crate::state::tmc::tmc_channel0()
            .offset_headroom_reserved(core::sync::atomic::Ordering::Relaxed)
    );
    crate::tmc_control::test_spi::assert_limit_plus_offset_within(&spi.writes, OFFSET_SAMPLE_LIMIT);
}

#[fixture_test]
async fn offset_diagnostic_saves_the_released_offset_not_a_live_one() {
    let live_offset = offset_register(400);
    store_channel0_limit(OFFSET_SAMPLE_LIMIT);
    let tmc_state = crate::state::tmc::tmc_channel0();
    let mut runtime = ready_runtime();
    runtime.hold_offset_headroom_for_test(tmc_state, live_offset);
    let mut tmc = Tmc4671Spi::new(
        offset_sample_spi()
            .with_register(PID_TORQUE_FLUX_OFFSET, live_offset)
            .with_register(PID_TORQUE_FLUX_LIMITS, 500),
    );

    runtime
        .release_offset_headroom(&mut tmc, tmc_state)
        .await
        .expect("request entry releases the reservation");
    let report = run_offset_sample(&mut tmc, &mut runtime, 300).await;

    assert_eq!(report.status, 0);
    let spi = tmc.into_inner();
    let pairs = limit_and_offset_writes(&spi.writes);
    let target_at = pairs
        .iter()
        .position(|write| *write == (PID_TORQUE_FLUX_OFFSET.addr(), offset_register(300)))
        .expect("target offset written");
    assert!(
        pairs[target_at + 1..]
            .iter()
            .filter(|(addr, _)| *addr == PID_TORQUE_FLUX_OFFSET.addr())
            .all(|(_, value)| *value == 0),
        "{pairs:?}"
    );
    assert_offset_then_limit_restored(&spi.writes, 0);
    crate::tmc_control::test_spi::assert_limit_plus_offset_within(&spi.writes, OFFSET_SAMPLE_LIMIT);
}

#[test]
fn voltage_step_rejects_non_position_mode() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default().with_read(0x8000_0001);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_voltage_step_request(
            tmc_voltage_step_test(0, 3, 44, -55, 6),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_NOT_READY);
        assert_eq!(report.uq_ext, 44);
        assert_eq!(report.ud_ext, -55);
    });
}

#[test]
fn voltage_step_failure_clears_external_voltage_and_restores_mode() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(100)
            .with_read(1000)
            .with_read(0x0001_0000)
            .fail_writes_to(UQ_UD_EXT.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_voltage_step_request(
            tmc_voltage_step_test(0, 3, 44, -55, 6),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![
                (STATUS_FLAGS.addr(), 0),
                (UQ_UD_EXT.addr(), pack_voltage_ext(44, -55)),
                (MODE_RAMP_MODE_MOTION.addr(), 0),
                (UQ_UD_EXT.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
        assert_eq!(
            raw_spi.writes,
            std::vec![
                (STATUS_FLAGS.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
        assert_eq!(
            raw_spi.read_requests,
            std::vec![
                MODE_RAMP_MODE_MOTION.addr(),
                ABN_DECODER_COUNT.addr(),
                ABN_DECODER_PPR.addr(),
                PID_TORQUE_FLUX_ACTUAL.addr(),
            ]
        );
    });
}

#[test]
fn voltage_step_invalid_ppr_aborts_before_excitation() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(100)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_voltage_step_request(
            tmc_voltage_step_test(0, 3, 44, -55, 6),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
        let raw_spi = tmc.into_inner();
        assert!(
            !raw_spi
                .write_attempts
                .contains(&(UQ_UD_EXT.addr(), pack_voltage_ext(44, -55)))
        );
    });
}

#[test]
fn voltage_step_exact_half_turn_aborts_via_post_measurement_error() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(100)
            .with_read(1000)
            .with_read(0x0001_0000)
            .with_read(600);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();

        let outcome = run_voltage_step_request(
            tmc_voltage_step_test(0, 3, 44, -55, 6),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_SPI);
    });
}

#[test]
fn voltage_step_host_liveness_expiry_reports_dedicated_status_and_restores() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(100)
            .with_read(1000)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut calls = 0u32;
        let mut expire_during_main_delay_final_slice = || {
            calls += 1;
            calls >= 3
        };
        let req = crate::tmc::tmc_voltage_step_test(0, 4, 100, -50, 150);

        let outcome = run_voltage_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_during_main_delay_final_slice),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![100, 50]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 3..];
        assert_eq!(
            restore_tail,
            [
                (MODE_RAMP_MODE_MOTION.addr(), 0),
                (UQ_UD_EXT.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn voltage_step_post_settle_host_liveness_expiry_reports_dedicated_status() {
    pollster::block_on(async {
        let raw_spi = DiagnosticMockSpi::default()
            .with_read(0x8000_0003)
            .with_read(100)
            .with_read(1000)
            .with_read(0);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut calls = 0u32;
        let mut expire_at_post_settle = || {
            calls += 1;
            calls >= 3
        };
        let req = crate::tmc::tmc_voltage_step_test(0, 4, 100, -50, 20);

        let outcome = run_voltage_step_request(
            req,
            DiagnosticContext {
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
                host_liveness_expired: Some(&mut expire_at_post_settle),
            },
        )
        .await;

        let DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report)) = outcome else {
            panic!("expected voltage step report");
        };
        assert_eq!(report.status, CURRENT_STEP_STATUS_HOST_LIVENESS);
        assert_eq!(delay.delays_ms, std::vec![20]);

        let raw_spi = tmc.into_inner();
        let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 3..];
        assert_eq!(
            restore_tail,
            [
                (MODE_RAMP_MODE_MOTION.addr(), 0),
                (UQ_UD_EXT.addr(), 0),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );
    });
}

#[test]
fn dispatcher_returns_not_diagnostic_for_normal_request() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let req = crate::tmc::tmc_set_current(0, 1000);

        let outcome = handle_diagnostic_request(
            req.clone(),
            DiagnosticContext {
                host_liveness_expired: None,
                tmc: &mut tmc,
                runtime: &mut runtime,
                delay: &mut delay,
                power_stage: &mut MockDiagnosticPowerStage::default(),
                tmc_config: &TEST_TMC_CONFIG,
                feedback_config: crate::config::FeedbackConfig::abn_abn(),
                readiness: DiagnosticReadiness {
                    homing_active: false,
                    stepper_active: false,
                    saved_voltage_limit: 777,
                    adc_vm_offset: None,
                },
            },
        )
        .await;

        let DiagnosticOutcome::NotDiagnostic(returned) = outcome else {
            panic!("expected NotDiagnostic");
        };
        assert_eq!(returned, req);
    });
}

#[test]
fn dispatcher_refuses_every_diagnostic_while_the_limit_reservation_is_engaged() {
    pollster::block_on(async {
        let reservation_state = crate::state::tmc::TmcChannelState::new();
        let requests = [
            tmc_position_torque_offset_sample_test(0, 4, 33, 3, 444),
            tmc_current_step_test(0, 7, CurrentStepAxis::Torque, 123, 4, 500),
        ];
        for req in requests {
            let mut runtime = ready_runtime();
            runtime.hold_offset_headroom_for_test(&reservation_state, 0x0400_0000);
            let mut delay = MockDelay::default();
            let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default().with_read(0x8000_0003));

            let outcome = handle_diagnostic_request(
                req.clone(),
                DiagnosticContext {
                    host_liveness_expired: None,
                    tmc: &mut tmc,
                    runtime: &mut runtime,
                    delay: &mut delay,
                    power_stage: &mut MockDiagnosticPowerStage::default(),
                    tmc_config: &TEST_TMC_CONFIG,
                    feedback_config: crate::config::FeedbackConfig::abn_abn(),
                    readiness: DiagnosticReadiness {
                        homing_active: false,
                        stepper_active: false,
                        saved_voltage_limit: 777,
                        adc_vm_offset: None,
                    },
                },
            )
            .await;

            let status = match outcome {
                DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report)) => {
                    assert_eq!(report.kind, TorqueSampleKind::PositionTorqueOffset);
                    assert_eq!(report.oid, 4);
                    report.status
                }
                DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { report, .. }) => {
                    assert_eq!(report.oid, 7);
                    report.status
                }
                _ => panic!("expected a not-ready report for {:?}", req.kind),
            };
            assert_eq!(status, CURRENT_STEP_STATUS_NOT_READY);
            let spi = tmc.into_inner();
            assert!(spi.write_attempts.is_empty(), "{:?}", spi.write_attempts);
            assert!(spi.read_requests.is_empty(), "{:?}", spi.read_requests);
            assert!(delay.delays_ms.is_empty());
            assert!(runtime.offset_headroom_engaged());
        }
    });
}

#[derive(Debug, Clone, PartialEq, Eq)]
enum RecordedResistanceEvent {
    Profile { oid: u8 },
    Axis { oid: u8, electrical_axis: u8 },
    Done { oid: u8, status: u8 },
}

#[derive(Default)]
struct RecordingDiagnosticSink {
    events: std::vec::Vec<RecordedResistanceEvent>,
}

impl DiagnosticReplySink for RecordingDiagnosticSink {
    fn foci_current_step_result(
        &mut self,
        _report_seq: u8,
        _axis: CurrentStepAxis,
        _report: CurrentStepReport,
    ) {
    }
    fn foci_current_vector_step_result(&mut self, _report_seq: u8, _report: CurrentStepReport) {}
    fn foci_current_torque_sample_pid_result(
        &mut self,
        _report_seq: u8,
        _report: &CurrentTorqueSampleReport,
    ) {
    }
    fn foci_current_torque_sample_result(
        &mut self,
        _report_seq: u8,
        _report: CurrentTorqueSampleReport,
    ) {
    }
    fn foci_current_torque_sample_detail_result(
        &mut self,
        _report_seq: u8,
        _report: &CurrentTorqueSampleReport,
    ) {
    }
    fn foci_voltage_step_result(&mut self, _report_seq: u8, _report: VoltageStepReport) {}
    fn foci_voltage_step_detail_result(&mut self, _report_seq: u8, _report: &VoltageStepReport) {}

    fn foci_resistance_profile(&mut self, _report_seq: u8, report: ResistanceProfileReport) {
        self.events
            .push(RecordedResistanceEvent::Profile { oid: report.oid });
    }

    fn foci_resistance_axis(&mut self, _report_seq: u8, report: ResistanceAxisReport) {
        self.events.push(RecordedResistanceEvent::Axis {
            oid: report.oid,
            electrical_axis: report.electrical_axis,
        });
    }

    fn foci_resistance_run(&mut self, _report_seq: u8, report: ResistanceRunReport) {
        self.events.push(RecordedResistanceEvent::Done {
            oid: report.oid,
            status: report.status,
        });
    }
}

#[fixture_test]
async fn resistance_test_request_dispatches_to_shared_runner() {
    let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
    channel_state
        .motor_sentinel
        .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let mut delay = MockDelay::default();
    let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
    let req = crate::tmc::tmc_resistance_test(0, 7, 0);

    let outcome = handle_diagnostic_request(
        req,
        DiagnosticContext {
            host_liveness_expired: None,
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut MockDiagnosticPowerStage::default(),
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
            readiness: DiagnosticReadiness {
                homing_active: false,
                stepper_active: false,
                saved_voltage_limit: 777,
                adc_vm_offset: None,
            },
        },
    )
    .await;

    channel_state
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);

    let DiagnosticOutcome::Handled(report) = outcome else {
        panic!("expected resistance test report");
    };

    let mut sink = RecordingDiagnosticSink::default();
    emit_diagnostic_report(&mut sink, 0, report);

    assert!(
        sink.events
            .iter()
            .any(|event| matches!(event, RecordedResistanceEvent::Done { oid: 7, .. }))
    );
}

#[fixture_test]
fn resistance_trip_lowers_board_stage_before_clearing_shared_state() {
    let channel_state = crate::state::tmc::tmc_channel0();
    let stepper_state = crate::state::stepper::stepper_channel0();
    channel_state.reset();
    stepper_state.reset();
    channel_state
        .motor_armed
        .store(true, core::sync::atomic::Ordering::Relaxed);
    stepper_state
        .timer_active_mirror
        .store(true, core::sync::atomic::Ordering::Relaxed);

    let mut runtime = ready_runtime();
    let mut delay = MockDelay::default();
    let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
    let mut power_stage = MockDiagnosticPowerStage::default();
    let mut ctx = DiagnosticContext {
        host_liveness_expired: None,
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 777,
            adc_vm_offset: None,
        },
    };

    {
        let mut backend = super::resistance::DiagnosticResistanceBackend::new(&mut ctx);
        crate::resistance_identification::ResistanceBackend::set_enable_pin(&mut backend, true);
        crate::resistance_identification::ResistanceBackend::set_enable_pin(&mut backend, false);
    }
    assert_eq!(power_stage.trip_calls, 0);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());

    let mut ctx = DiagnosticContext {
        host_liveness_expired: None,
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 777,
            adc_vm_offset: None,
        },
    };
    {
        let mut backend = super::resistance::DiagnosticResistanceBackend::new(&mut ctx);
        crate::resistance_identification::ResistanceBackend::trip_power_stage(&mut backend);
    }

    assert_eq!(power_stage.trip_calls, 1);
    assert!(power_stage.motor_armed_at_trip);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(
        !channel_state
            .motor_armed
            .load(core::sync::atomic::Ordering::Relaxed)
    );
    assert!(
        !stepper_state
            .timer_active_mirror
            .load(core::sync::atomic::Ordering::Relaxed)
    );
    channel_state.reset();
    stepper_state.reset();
}

#[fixture_test]
async fn resistance_test_not_armed_reports_no_axes() {
    let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
    channel_state
        .motor_sentinel
        .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let mut delay = MockDelay::default();
    let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
    let req = crate::tmc::tmc_resistance_test(0, 9, 0);

    let outcome = handle_diagnostic_request(
        req,
        DiagnosticContext {
            host_liveness_expired: None,
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut MockDiagnosticPowerStage::default(),
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
            readiness: DiagnosticReadiness {
                homing_active: false,
                stepper_active: false,
                saved_voltage_limit: 777,
                adc_vm_offset: None,
            },
        },
    )
    .await;

    channel_state
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);

    let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { profile, run, axes }) =
        outcome
    else {
        panic!("expected resistance test report");
    };
    assert_eq!(profile.oid, 9);
    assert_eq!(
        run.status,
        crate::resistance_identification::ResistanceError::NotArmed.status_code()
    );
    assert!(axes.is_none());
}

#[fixture_test]
async fn resistance_test_uses_board_monotonic_for_sample_timing() {
    let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
    channel_state
        .motor_sentinel
        .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
    channel_state
        .encoder_ppr
        .store(1_000, core::sync::atomic::Ordering::Relaxed);

    let mut runtime = ready_runtime();
    let mut delay = MockDelay {
        advance_on_delay: true,
        advance_on_now_us: 3_000,
        ..MockDelay::default()
    };
    let raw_spi = DiagnosticMockSpi::default()
        .with_read(2_000)
        .with_read(0x0002_0032)
        .with_read(7)
        .with_read(u32::from(TEST_TMC_CONFIG.pwm_maxcnt))
        .with_read((u32::from(TEST_TMC_CONFIG.bbm_h) << 8) | u32::from(TEST_TMC_CONFIG.bbm_l))
        .with_read(
            (u32::from(TEST_TMC_CONFIG.dsadc_mdec_b) << 16)
                | u32::from(TEST_TMC_CONFIG.dsadc_mdec_a),
        )
        .with_read(0)
        .with_read(0)
        .with_read(0)
        .with_read(0);
    let mut tmc = Tmc4671Spi::new(raw_spi);
    let req = crate::tmc::tmc_resistance_test(0, 10, 0);

    let outcome = handle_diagnostic_request(
        req,
        DiagnosticContext {
            host_liveness_expired: None,
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut MockDiagnosticPowerStage::default(),
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
            readiness: DiagnosticReadiness {
                homing_active: false,
                stepper_active: false,
                saved_voltage_limit: 2_000,
                adc_vm_offset: None,
            },
        },
    )
    .await;

    channel_state
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);
    channel_state
        .encoder_ppr
        .store(0, core::sync::atomic::Ordering::Relaxed);

    let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { run, axes, .. }) = outcome
    else {
        panic!("expected resistance test report");
    };
    assert_eq!(
        run.status,
        crate::resistance_identification::ResistanceError::TimingRejected.status_code()
    );
    assert!(axes.is_none());
    assert!(delay.now_us.get() > 0);
}

#[fixture_test]
async fn resistance_test_rejects_during_homing_without_invoking_runner() {
    let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
    channel_state
        .motor_sentinel
        .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

    let mut runtime = ready_runtime();
    let mut delay = MockDelay::default();
    let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
    let req = crate::tmc::tmc_resistance_test(0, 11, 0);

    let outcome = handle_diagnostic_request(
        req,
        DiagnosticContext {
            host_liveness_expired: None,
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut MockDiagnosticPowerStage::default(),
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
            readiness: DiagnosticReadiness {
                homing_active: true,
                stepper_active: false,
                saved_voltage_limit: 777,
                adc_vm_offset: None,
            },
        },
    )
    .await;

    channel_state
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);

    let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { profile, run, axes }) =
        outcome
    else {
        panic!("expected resistance test report");
    };
    assert_eq!(profile.oid, 11);
    assert_eq!(run.status, CURRENT_STEP_STATUS_NOT_READY);
    assert!(axes.is_none());
    assert!(
        tmc.into_inner().writes.is_empty(),
        "expected no SPI writes (no energizing) while homing is active"
    );
}

#[test]
fn resistance_backend_delay_ms_liveness_expiry_injects_one_shot_error_then_recovers() {
    pollster::block_on(async {
        let mut runtime = ready_runtime();
        let mut delay = MockDelay::default();
        let mut tmc = Tmc4671Spi::new(DiagnosticMockSpi::default());
        let mut power_stage = MockDiagnosticPowerStage::default();
        let mut calls = 0u32;
        let mut expire_after_final_slice = || {
            calls += 1;
            calls >= 3
        };
        let mut ctx = DiagnosticContext {
            tmc: &mut tmc,
            runtime: &mut runtime,
            delay: &mut delay,
            power_stage: &mut power_stage,
            tmc_config: &TEST_TMC_CONFIG,
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
            readiness: DiagnosticReadiness {
                homing_active: false,
                stepper_active: false,
                saved_voltage_limit: 777,
                adc_vm_offset: None,
            },
            host_liveness_expired: Some(&mut expire_after_final_slice),
        };
        let mut backend = super::resistance::DiagnosticResistanceBackend::new(&mut ctx);

        crate::resistance_identification::ResistanceBackend::delay_ms(&mut backend, 150).await;

        let first = crate::resistance_identification::ResistanceBackend::write_register(
            &mut backend,
            0x01,
            0,
        )
        .await;

        let second = crate::resistance_identification::ResistanceBackend::write_register(
            &mut backend,
            0x01,
            5,
        )
        .await;

        let third = crate::resistance_identification::ResistanceBackend::write_register(
            &mut backend,
            0x01,
            9,
        )
        .await;

        let aborted = backend.host_liveness_aborted();

        assert!(aborted);
        assert!(first.is_err());
        assert!(second.is_ok());
        assert!(third.is_ok());
        assert_eq!(delay.delays_ms, std::vec![100, 50]);
    });
}

#[test]
fn resistance_terminal_status_prefers_host_liveness_over_backend_error_status() {
    use crate::resistance_identification::ResistanceError;

    assert_eq!(
        super::resistance::terminal_status(ResistanceError::BackendIo, true),
        crate::autotune::CommissionError::HostLivenessExpired.status_code()
    );
}

#[test]
fn resistance_terminal_status_uses_error_status_when_not_liveness_aborted() {
    use crate::resistance_identification::ResistanceError;

    assert_eq!(
        super::resistance::terminal_status(ResistanceError::BackendIo, false),
        ResistanceError::BackendIo.status_code()
    );
}

#[fixture_test]
async fn resistance_test_request_host_liveness_expiry_reports_dedicated_status() {
    let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
    channel_state
        .motor_sentinel
        .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

    let raw_spi = DiagnosticMockSpi::default()
        .with_read(1600)
        .with_read(0x0002_0032)
        .with_read(7)
        .with_read(3999)
        .with_read(2313)
        .with_read(21_692_747)
        .with_read(0)
        .with_read(0);
    let mut tmc = Tmc4671Spi::new(raw_spi);
    let mut runtime = ready_runtime();
    let mut delay = MockDelay::default();
    let mut power_stage = MockDiagnosticPowerStage::default();
    let mut calls = 0u32;
    let mut expire_during_settle_final_slice = || {
        calls += 1;
        calls >= 4
    };
    let ctx = DiagnosticContext {
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 777,
            adc_vm_offset: None,
        },
        host_liveness_expired: Some(&mut expire_during_settle_final_slice),
    };

    let outcome = super::resistance::run_resistance_test_request(
        crate::tmc::tmc_resistance_test(0, 7, 0),
        ctx,
    )
    .await;

    channel_state
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);

    let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { run, .. }) = outcome else {
        panic!("expected resistance test report");
    };

    assert_eq!(
        run.status,
        crate::autotune::CommissionError::HostLivenessExpired.status_code()
    );
    assert_eq!(delay.delays_ms, std::vec![100, 100, 50]);
    let raw_spi = tmc.into_inner();
    let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
    assert_eq!(
        restore_tail,
        [
            (
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr(),
                foci_tmc4671::registers::MotionMode::Stopped as u32
            ),
            (foci_tmc4671::registers::UQ_UD_EXT.addr(), 0),
            (foci_tmc4671::registers::PWM_SV_CHOP.addr(), 0),
            (foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS.addr(), 1600),
        ]
    );
}

#[fixture_test]
async fn resistance_test_request_genuine_spi_failure_reports_generic_status() {
    let channel_state = crate::state::tmc::tmc_channel(0).unwrap();
    channel_state
        .motor_sentinel
        .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);

    let raw_spi = DiagnosticMockSpi::default()
        .with_read(1600)
        .with_read(0x0002_0032)
        .with_read(7)
        .with_read(3999)
        .with_read(2313)
        .with_read(21_692_747)
        .with_read(0)
        .with_read(0)
        .fail_writes_to(foci_tmc4671::registers::PHI_E_EXT.addr());
    let mut tmc = Tmc4671Spi::new(raw_spi);
    let mut runtime = ready_runtime();
    let mut delay = MockDelay::default();
    let mut power_stage = MockDiagnosticPowerStage::default();
    let ctx = DiagnosticContext {
        tmc: &mut tmc,
        runtime: &mut runtime,
        delay: &mut delay,
        power_stage: &mut power_stage,
        tmc_config: &TEST_TMC_CONFIG,
        feedback_config: crate::config::FeedbackConfig::abn_abn(),
        readiness: DiagnosticReadiness {
            homing_active: false,
            stepper_active: false,
            saved_voltage_limit: 777,
            adc_vm_offset: None,
        },
        host_liveness_expired: None,
    };

    let outcome = super::resistance::run_resistance_test_request(
        crate::tmc::tmc_resistance_test(0, 7, 0),
        ctx,
    )
    .await;

    channel_state
        .motor_sentinel
        .store(0, core::sync::atomic::Ordering::Relaxed);

    let DiagnosticOutcome::Handled(DiagnosticReport::ResistanceTest { run, .. }) = outcome else {
        panic!("expected resistance test report");
    };

    assert_eq!(
        run.status,
        crate::resistance_identification::ResistanceError::BackendIo.status_code()
    );
    assert_ne!(
        run.status,
        crate::autotune::CommissionError::HostLivenessExpired.status_code()
    );
    let raw_spi = tmc.into_inner();
    let restore_tail = &raw_spi.write_attempts[raw_spi.write_attempts.len() - 4..];
    assert_eq!(
        restore_tail,
        [
            (
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr(),
                foci_tmc4671::registers::MotionMode::Stopped as u32
            ),
            (foci_tmc4671::registers::UQ_UD_EXT.addr(), 0),
            (foci_tmc4671::registers::PWM_SV_CHOP.addr(), 0),
            (foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS.addr(), 1600),
        ]
    );
}
