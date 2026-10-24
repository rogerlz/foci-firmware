//! Phase 2: Coil connectivity check.
//! carries the observations forward into the phase-wiring check.

use super::{CoilCheckState, CoilObservation, CoilTestState, InnerPhase};
use crate::autotune::{CommissionError, CommissionPhase, InnerSequence, Permille};
use crate::commissioning::types::{CommissioningBackend, StepResult};
use crate::selftest::{CommissionDiagnostic, CommissionDiagnostics};
use crate::tmc_register_values::{MOTION_STOPPED, MOTION_UQ_UD_EXT};

const DIAG_COIL_CHECK_A_SAMPLE: u8 = 1;
const DIAG_COIL_CHECK_B_SAMPLE: u8 = 2;

const MOTOR_CURRENT_THRESHOLD: u16 = 200;

const COIL_DIAGNOSTIC_CURRENT_FRACTION: Permille = Permille::new(1000);

const COIL_SETTLE_US: u64 = 100_000;

/// A free rotor (no detent torque) swings while it aligns to the excitation
/// and its back-EMF lowers the current, so after `COIL_SETTLE_US` keep
/// polling the encoder until two readings this far apart agree.
const COIL_STILL_POLL_US: u64 = 10_000;

const COIL_SETTLE_MAX_US: u64 = 1_000_000;

const COIL_COOLDOWN_US: u64 = 50_000;

/// Whether the encoder held still since the previous poll. Without a
/// configured encoder domain there is nothing to wait on.
fn rotor_still<B: CommissioningBackend>(backend: &B, last_count: Option<u32>, count: u32) -> bool {
    let Ok(domain) = crate::commissioning::encoder_domain::EncoderDomain::new(
        backend.mechanical_encoder_quadrature_counts_per_rev(),
    ) else {
        return true;
    };
    let Some(last_count) = last_count else {
        return false;
    };
    domain
        .delta(domain.count(last_count), domain.count(count))
        .is_ok_and(|delta| {
            delta.absolute_counts()
                <= super::encoder::encoder_stability_counts(domain.counts_per_rev())
        })
}

pub(crate) async fn step_coil_check<B: CommissioningBackend>(
    phase: &mut InnerPhase,
    sequence: InnerSequence,
    selftest_results: &mut crate::selftest::SelftestResults,
    diagnostics: &mut CommissionDiagnostics,
    backend: &mut B,
) -> StepResult {
    use foci_tmc4671::registers::*;

    let (state, coil_a_obs) = match phase {
        InnerPhase::CoilCheck { state, coil_a } => (state, coil_a),
        _ => return StepResult::ReadyNow,
    };

    let is_coil_b = matches!(state, CoilCheckState::CoilB(_));

    match state {
        CoilCheckState::CoilA(coil_test) | CoilCheckState::CoilB(coil_test) => match coil_test {
            CoilTestState::Setup { phase: coil_phase } => {
                let p = *coil_phase;
                if backend.write_register(PHI_E_SELECTION, 1).await.is_err() {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                let phi_e = if p == 0 { 0u32 } else { 16384 };
                if backend.write_register(PHI_E_EXT, phi_e).await.is_err() {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                let voltage = backend.excitation_voltage() as u32;
                if backend
                    .write_register(UQ_UD_EXT, voltage & 0xFFFF)
                    .await
                    .is_err()
                {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, MOTION_UQ_UD_EXT)
                    .await
                    .is_err()
                {
                    return super::fail_inner(phase, CommissionError::Spi);
                }
                backend.set_enable_pin(true);

                let now = backend.elapsed_us();
                let (Some(deadline), Some(give_up)) = (
                    now.checked_add(COIL_SETTLE_US),
                    now.checked_add(COIL_SETTLE_MAX_US),
                ) else {
                    return super::fail_inner(phase, CommissionError::InvalidSchedule);
                };
                *coil_test = CoilTestState::Settle {
                    phase: p,
                    deadline_us: deadline,
                    last_count: None,
                    give_up_us: give_up,
                };
                StepResult::At(deadline)
            }

            CoilTestState::Settle {
                phase: coil_phase,
                deadline_us,
                last_count,
                give_up_us,
            } => {
                let now = backend.elapsed_us();
                if now < *deadline_us {
                    return StepResult::At(*deadline_us);
                }
                let Ok(count) = backend.read_register(ABN_DECODER_COUNT).await else {
                    return super::fail_inner(phase, CommissionError::Spi);
                };
                if now < *give_up_us && !rotor_still(backend, *last_count, count) {
                    *last_count = Some(count);
                    *deadline_us = now.saturating_add(COIL_STILL_POLL_US);
                    return StepResult::At(*deadline_us);
                }
                *coil_test = CoilTestState::Read { phase: *coil_phase };
                StepResult::ReadyNow
            }

            CoilTestState::Read { phase: coil_phase } => {
                let p = *coil_phase;
                let raw = match backend.read_register(ADC_IWY_IUX).await {
                    Ok(v) => v,
                    Err(_) => {
                        return super::fail_inner(phase, CommissionError::Spi);
                    }
                };

                let _ = backend
                    .write_register(MODE_RAMP_MODE_MOTION, MOTION_STOPPED)
                    .await;
                let _ = backend.write_register(UQ_UD_EXT, 0).await;
                backend.set_enable_pin(false);

                let i_ux = (raw & 0xFFFF) as i16;
                let i_wy = ((raw >> 16) & 0xFFFF) as i16;

                let obs = if p == 0 {
                    CoilObservation {
                        expected: i_ux,
                        other: i_wy,
                    }
                } else {
                    CoilObservation {
                        expected: i_wy,
                        other: i_ux,
                    }
                };

                let sample_failed = obs.expected.unsigned_abs() <= MOTOR_CURRENT_THRESHOLD;
                diagnostics.push(CommissionDiagnostic {
                    phase: CommissionPhase::CoilCheck.to_u8(),
                    code: if p == 0 {
                        DIAG_COIL_CHECK_A_SAMPLE
                    } else {
                        DIAG_COIL_CHECK_B_SAMPLE
                    },
                    status: if sample_failed { 1 } else { 0 },
                    value0: obs.expected as u16 as u32,
                    value1: obs.other as u16 as u32,
                    value2: raw,
                });

                let envelope = crate::tmc_control::current_limit::diagnostic_current_envelope(
                    backend.current_limit(),
                    COIL_DIAGNOSTIC_CURRENT_FRACTION,
                );
                let over_envelope =
                    obs.expected.unsigned_abs() > envelope || obs.other.unsigned_abs() > envelope;

                if over_envelope || sample_failed {
                    if matches!(sequence, InnerSequence::Selftest) {
                        let stage = if is_coil_b {
                            crate::selftest::SelftestStage::CoilB
                        } else {
                            crate::selftest::SelftestStage::CoilA
                        };
                        selftest_results.set(stage, crate::selftest::SelftestStageValue::fail(0));
                    }
                    let error = if over_envelope {
                        CommissionError::CoilCurrentEnvelope
                    } else {
                        CommissionError::CoilFault
                    };
                    return super::fail_inner(phase, error);
                }

                let deadline = match backend.elapsed_us().checked_add(COIL_COOLDOWN_US) {
                    Some(deadline) => deadline,
                    None => return super::fail_inner(phase, CommissionError::InvalidSchedule),
                };
                *coil_test = CoilTestState::Cooldown {
                    phase: p,
                    deadline_us: deadline,
                };

                if !is_coil_b {
                    if matches!(sequence, InnerSequence::Selftest) {
                        selftest_results.set(
                            crate::selftest::SelftestStage::CoilA,
                            crate::selftest::SelftestStageValue::coil_current(obs.expected),
                        );
                    }
                    *coil_a_obs = Some(obs);
                } else {
                    if matches!(sequence, InnerSequence::Selftest) {
                        selftest_results.set(
                            crate::selftest::SelftestStage::CoilB,
                            crate::selftest::SelftestStageValue::coil_current(obs.expected),
                        );
                    }
                    let a = coil_a_obs.unwrap_or(CoilObservation {
                        expected: 0,
                        other: 0,
                    });
                    *phase = InnerPhase::PhaseWiringCheck {
                        coil_a: a,
                        coil_b: obs,
                    };
                    return StepResult::PhaseChanged(CommissionPhase::PhaseWiringCheck);
                }
                StepResult::At(deadline)
            }

            CoilTestState::Cooldown {
                phase: _coil_phase,
                deadline_us,
            } => {
                if backend.elapsed_us() < *deadline_us {
                    return StepResult::At(*deadline_us);
                }
                *state = CoilCheckState::CoilB(CoilTestState::Setup { phase: 1 });
                StepResult::ReadyNow
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::{CommissionProfile, InnerSequence};
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, current_limit};
    use crate::commissioning::types::StepResult;
    use crate::selftest::CommissionDiagnostics;
    use foci_tmc4671::registers::ADC_IWY_IUX;

    const COIL_CHECK_MAX_REGISTER_OPS_PER_ARM: usize = 4;

    fn register_op_count(backend: &MockBackend) -> usize {
        backend
            .spi_log
            .iter()
            .filter(|op| !matches!(op, SpiOp::EnablePin(_)))
            .count()
    }

    #[test]
    fn coil_setup_and_wait_states_report_exact_schedules() {
        pollster::block_on(async {
            let mut phase = InnerPhase::CoilCheck {
                state: CoilCheckState::CoilA(CoilTestState::Setup { phase: 0 }),
                coil_a: None,
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 1_000;

            let setup = step_coil_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(setup, StepResult::At(101_000)));
            assert!(register_op_count(&backend) <= COIL_CHECK_MAX_REGISTER_OPS_PER_ARM);

            backend.elapsed_us_value = 100_000;
            let early = step_coil_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(early, StepResult::At(101_000)));

            backend.elapsed_us_value = 101_000;
            let first_poll = step_coil_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(first_poll, StepResult::At(111_000)));

            backend.elapsed_us_value = 111_000;
            let due = step_coil_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(due, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                InnerPhase::CoilCheck {
                    state: CoilCheckState::CoilA(CoilTestState::Read { phase: 0 }),
                    ..
                }
            ));
        });
    }

    #[test]
    fn coil_settle_waits_for_a_swinging_rotor_to_stop() {
        pollster::block_on(async {
            let mut phase = InnerPhase::CoilCheck {
                state: CoilCheckState::CoilA(CoilTestState::Settle {
                    phase: 0,
                    deadline_us: 101_000,
                    last_count: None,
                    give_up_us: 1_001_000,
                }),
                coil_a: None,
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[100, 140, 141]);

            let mut now = 101_000;
            for expected in [111_000, 121_000] {
                backend.elapsed_us_value = now;
                let result = step_coil_check(
                    &mut phase,
                    InnerSequence::Commission,
                    &mut selftest_results,
                    &mut diagnostics,
                    &mut backend,
                )
                .await;
                assert!(matches!(result, StepResult::At(at) if at == expected));
                now += 10_000;
            }

            backend.elapsed_us_value = now;
            let still = step_coil_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(still, StepResult::ReadyNow));
        });
    }

    #[test]
    fn coil_settle_reads_anyway_after_the_give_up_time() {
        pollster::block_on(async {
            let mut phase = InnerPhase::CoilCheck {
                state: CoilCheckState::CoilA(CoilTestState::Settle {
                    phase: 0,
                    deadline_us: 1_001_000,
                    last_count: Some(0),
                    give_up_us: 1_001_000,
                }),
                coil_a: None,
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 4000;
            backend.set_abn_decoder_count_sequence(&[500]);
            backend.elapsed_us_value = 1_001_000;

            let result = step_coil_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(result, StepResult::ReadyNow));
        });
    }

    #[test]
    fn coil_read_and_cooldown_bound_work_and_report_exact_schedules() {
        pollster::block_on(async {
            let mut phase = InnerPhase::CoilCheck {
                state: CoilCheckState::CoilA(CoilTestState::Read { phase: 0 }),
                coil_a: None,
            };
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.current_limit_value = current_limit(2816);
            backend.elapsed_us_value = 1_000;
            backend.registers.insert(ADC_IWY_IUX.addr(), 300).unwrap();

            let read = step_coil_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(read, StepResult::At(51_000)));
            assert!(register_op_count(&backend) <= COIL_CHECK_MAX_REGISTER_OPS_PER_ARM);

            backend.spi_log.clear();
            backend.elapsed_us_value = 50_000;
            let early = step_coil_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(early, StepResult::At(51_000)));
            assert_eq!(register_op_count(&backend), 0);

            backend.elapsed_us_value = 51_000;
            let due = step_coil_check(
                &mut phase,
                InnerSequence::Commission,
                &mut selftest_results,
                &mut diagnostics,
                &mut backend,
            )
            .await;
            assert!(matches!(due, StepResult::ReadyNow));
            assert!(matches!(
                phase,
                InnerPhase::CoilCheck {
                    state: CoilCheckState::CoilB(CoilTestState::Setup { phase: 1 }),
                    ..
                }
            ));
        });
    }

    #[test]
    fn coil_fault_records_raw_current_sample() {
        pollster::block_on(async {
            let mut phase = InnerPhase::CoilCheck {
                state: CoilCheckState::CoilA(CoilTestState::Read { phase: 0 }),
                coil_a: None,
            };
            let mut buf = crate::autotune::TransientBuffer::new();
            let mut adc_cal = None;
            let mut selftest_results = crate::selftest::SelftestResults::default();
            let mut diagnostics = CommissionDiagnostics::default();
            let mut backend = MockBackend::new();
            backend.current_limit_value = current_limit(2816);
            let raw = ((-20i16 as u16 as u32) << 16) | 120u16 as u32;
            backend.registers.insert(ADC_IWY_IUX.addr(), raw).unwrap();

            let result = super::super::step_inner(
                &mut phase,
                CommissionProfile::Balanced,
                InnerSequence::Commission,
                super::super::InnerStepContext {
                    buf: &mut buf,
                    adc_cal: &mut adc_cal,
                    calibrate_entry_gains: None,
                    selftest_results: &mut selftest_results,
                    diagnostics: &mut diagnostics,
                },
                &mut backend,
            )
            .await;

            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::CoilFault)
            ));
            let items: heapless::Vec<_, 4> = diagnostics.iter().collect();
            assert_eq!(items.len(), 1);
            assert_eq!(items[0].phase, CommissionPhase::CoilCheck.to_u8());
            assert_eq!(items[0].status, 1);
            assert_eq!(items[0].value0, 120);
            assert_eq!(items[0].value1, -20i16 as u16 as u32);
            assert_eq!(items[0].value2, raw);
        });
    }

    fn read_state_with_sample(limit: u32, raw: u32) -> (InnerPhase, MockBackend) {
        let phase = InnerPhase::CoilCheck {
            state: CoilCheckState::CoilA(CoilTestState::Read { phase: 0 }),
            coil_a: None,
        };
        let mut backend = MockBackend::new();
        backend.current_limit_value = current_limit(limit);
        backend.registers.insert(ADC_IWY_IUX.addr(), raw).unwrap();
        (phase, backend)
    }

    async fn run_read(phase: &mut InnerPhase, backend: &mut MockBackend) -> StepResult {
        let mut selftest_results = crate::selftest::SelftestResults::default();
        let mut diagnostics = CommissionDiagnostics::default();
        step_coil_check(
            phase,
            InnerSequence::Commission,
            &mut selftest_results,
            &mut diagnostics,
            backend,
        )
        .await
    }

    #[test]
    fn coil_sample_above_the_current_limit_rejects_with_the_envelope_error() {
        pollster::block_on(async {
            let (mut phase, mut backend) = read_state_with_sample(2816, 2817);
            let result = run_read(&mut phase, &mut backend).await;
            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::CoilCurrentEnvelope)
            ));
        });
    }

    #[test]
    fn coil_over_current_on_the_unexcited_phase_also_rejects() {
        pollster::block_on(async {
            let (mut phase, mut backend) = read_state_with_sample(2816, (2817u32 << 16) | 300);
            let result = run_read(&mut phase, &mut backend).await;
            assert!(matches!(
                result,
                StepResult::InnerFailed(CommissionError::CoilCurrentEnvelope)
            ));
        });
    }

    #[test]
    fn coil_sample_at_the_current_limit_passes() {
        pollster::block_on(async {
            let (mut phase, mut backend) = read_state_with_sample(2816, 2816);
            let result = run_read(&mut phase, &mut backend).await;
            assert!(matches!(result, StepResult::At(_)));
        });
    }
}
