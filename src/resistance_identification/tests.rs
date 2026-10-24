use core::cell::Cell;
use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, ADC_RAW_ADDR, ADC_RAW_DATA, DSADC_MDEC_B_MDEC_A,
    MODE_RAMP_MODE_MOTION, MotionMode, MotorType, MotorTypeNPolePairs, PHI_E_SELECTION,
    PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP, STATUS_FLAGS, UQ_UD_EXT,
};
use pollster::block_on;

use super::bus_voltage::BusVoltageSense;
use super::profile::ResistanceProfile;
use super::runner::{
    ResistanceBackend, ResistanceBackendError, ResistancePoll, ResistanceRunMode, ResistanceRunner,
    run_resistance_identification,
};
use super::state::acquisition_opportunities_through;
use super::types::{ResistanceCurrentEvidence, ResistanceError, ResistanceWarningFlags};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RecordedWrite {
    addr: u8,
    value: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BackendEvent {
    Read(u8),
    Write(u8, u32),
    Enable(bool),
    TripPowerStage,
}

struct FakeResistanceBackend {
    writes: heapless::Vec<RecordedWrite, 256>,
    profile: ResistanceProfile,
    armed_for_open_loop_voltage: bool,
    adc_offsets_ready: bool,
    motor_type: MotorType,
    saved_voltage_limit: u32,
    readback_pwm_maxcnt: u32,
    readback_pwm_bbm: u32,
    readback_dsadc: u32,
    readback_pwm_sv_chop: u32,
    encoder_counts_per_rev: u32,
    scripted_status_flags: u32,
    scripted_status_reads: heapless::Vec<u32, 8>,
    scripted_status_read_index: usize,
    scripted_energized_current_count: i32,
    scripted_energized_samples: heapless::Vec<i16, 128>,
    scripted_energized_sample_index: usize,
    scripted_current_per_ud_milli: i32,
    scripted_encoder_delta_count: u32,
    last_ud: i16,
    last_motion_mode: u32,
    energized_mode_write_us: Option<u64>,
    energized_window_current_reads: u16,
    encoder_count: u32,
    enable_pin: bool,
    elapsed_us: u64,
    fail_after_writes: Option<usize>,
    fail_writes_after_trip: bool,
    power_stage_tripped: bool,
    write_count: usize,
    scripted_zero_currents: heapless::Vec<i32, 32>,
    zero_windows_sampled: usize,
    zero_window_counted: bool,
    scripted_positive_anchor_current: Option<i32>,
    scripted_negative_anchor_current: Option<i32>,
    operations: usize,
    enable_transitions: heapless::Vec<bool, 8>,
    io_elapsed_us: u64,
    scripted_elapsed_us: heapless::Vec<u64, 8>,
    scripted_elapsed_index: Cell<usize>,
    record_events: bool,
    events: heapless::Vec<BackendEvent, 64>,
    interlock_tripped: bool,
    bus_voltage_sense: Option<BusVoltageSense>,
    scripted_adc_vm_raw: u16,
    adc_raw_addr: u32,
    fail_reads_at: Option<u8>,
    fail_write_once_at: Option<u8>,
}

impl FakeResistanceBackend {
    fn standard() -> Self {
        let profile = ResistanceProfile::standard();
        Self {
            writes: heapless::Vec::new(),
            profile,
            armed_for_open_loop_voltage: true,
            adc_offsets_ready: true,
            motor_type: MotorType::TwoPhase,
            saved_voltage_limit: 16_000,
            readback_pwm_maxcnt: u32::from(profile.pwm_maxcnt),
            readback_pwm_bbm: (u32::from(profile.bbm_h) << 8) | u32::from(profile.bbm_l),
            readback_dsadc: (u32::from(profile.dsadc_mdec_b) << 16)
                | u32::from(profile.dsadc_mdec_a),
            readback_pwm_sv_chop: profile.pwm_sv_chop,
            encoder_counts_per_rev: 4000,
            scripted_status_flags: 0,
            scripted_status_reads: heapless::Vec::new(),
            scripted_status_read_index: 0,
            scripted_energized_current_count: 0,
            scripted_energized_samples: heapless::Vec::new(),
            scripted_energized_sample_index: 0,
            scripted_current_per_ud_milli: 1000,
            scripted_encoder_delta_count: 0,
            last_ud: 0,
            last_motion_mode: 0,
            energized_mode_write_us: None,
            energized_window_current_reads: 0,
            encoder_count: 0,
            enable_pin: false,
            elapsed_us: 0,
            fail_after_writes: None,
            fail_writes_after_trip: false,
            power_stage_tripped: false,
            write_count: 0,
            scripted_zero_currents: heapless::Vec::new(),
            zero_windows_sampled: 0,
            zero_window_counted: false,
            scripted_positive_anchor_current: None,
            scripted_negative_anchor_current: None,
            operations: 0,
            enable_transitions: heapless::Vec::new(),
            io_elapsed_us: 1,
            scripted_elapsed_us: heapless::Vec::new(),
            scripted_elapsed_index: Cell::new(0),
            record_events: false,
            events: heapless::Vec::new(),
            interlock_tripped: false,
            bus_voltage_sense: Some(BusVoltageSense {
                offset_raw: 33_000,
                counts_per_volt_milli: 269_318,
            }),
            scripted_adc_vm_raw: 33_000 + 12_927,
            adc_raw_addr: 0,
            fail_reads_at: None,
            fail_write_once_at: None,
        }
    }

    fn begin_poll(&mut self) {
        self.operations = 0;
    }

    fn operations_since_poll(&self) -> usize {
        self.operations
    }

    fn script_elapsed(&mut self, timestamps: &[u64]) {
        self.scripted_elapsed_us = heapless::Vec::from_slice(timestamps).unwrap();
        self.scripted_elapsed_index.set(0);
    }

    fn script_status_reads(&mut self, flags: &[u32]) {
        self.scripted_status_reads = heapless::Vec::from_slice(flags).unwrap();
        self.scripted_status_read_index = 0;
    }

    fn script_bus_millivolts(&mut self, millivolts: u32) {
        let sense = self.bus_voltage_sense.expect("sense configured");
        let delta = u64::from(millivolts) * u64::from(sense.counts_per_volt_milli) / 1_000_000;
        self.scripted_adc_vm_raw = sense.offset_raw + delta as u16;
    }

    fn fail_next_write(&mut self) {
        self.fail_after_writes = Some(self.write_count);
    }

    fn start_event_recording(&mut self) {
        self.events.clear();
        self.record_events = true;
    }

    fn record_event(&mut self, event: BackendEvent) {
        if self.record_events {
            self.events.push(event).expect("backend event log overflow");
        }
    }

    fn wrote_safe_state(&self) -> bool {
        let mode = MODE_RAMP_MODE_MOTION.addr();
        let uq_ud = UQ_UD_EXT.addr();
        let chop = PWM_SV_CHOP.addr();
        let limits = PIDOUT_UQ_UD_LIMITS.addr();

        let has = |addr: u8, value: u32| {
            self.writes
                .iter()
                .rev()
                .find(|write| write.addr == addr)
                .is_some_and(|write| write.value == value)
        };

        has(mode, 0) && has(uq_ud, 0) && has(chop, 0) && has(limits, self.saved_voltage_limit)
    }

    fn wrote_nothing(&self) -> bool {
        self.writes.is_empty()
    }

    fn energized_current_for_ud(&self) -> i32 {
        if self.scripted_energized_current_count != 0 {
            return self.scripted_energized_current_count;
        }
        let anchor_ud = self.profile.signed_anchor_ud;
        if self.last_ud == anchor_ud
            && let Some(current) = self.scripted_positive_anchor_current
        {
            return current;
        }
        if self.last_ud == -anchor_ud
            && let Some(current) = self.scripted_negative_anchor_current
        {
            return current;
        }
        i32::from(self.last_ud) * self.scripted_current_per_ud_milli / 1000
    }

    fn zero_current_for_window(&self) -> i32 {
        if self.scripted_zero_currents.is_empty() {
            return 0;
        }
        let sampled = self.zero_windows_sampled.max(1);
        let index = (sampled - 1) % self.scripted_zero_currents.len();
        self.scripted_zero_currents[index]
    }

    fn packed_phase_current(&mut self) -> u32 {
        let energized = self.last_motion_mode == 8;
        let value = if energized
            && let Some(&current) = self
                .scripted_energized_samples
                .get(self.scripted_energized_sample_index)
        {
            self.scripted_energized_sample_index += 1;
            i32::from(current)
        } else if energized {
            self.energized_current_for_ud()
        } else {
            self.zero_current_for_window()
        };
        let half = (value as i16) as u16 as u32;
        (half << 16) | half
    }
}

fn poll_runner(
    runner: &mut ResistanceRunner,
    backend: &mut FakeResistanceBackend,
    now_us: u64,
) -> ResistancePoll {
    backend.elapsed_us = backend.elapsed_us.max(now_us);
    block_on(runner.poll(backend, now_us))
}

impl ResistanceBackend for FakeResistanceBackend {
    async fn read_register(&mut self, addr: u8) -> Result<u32, ResistanceBackendError> {
        self.record_event(BackendEvent::Read(addr));
        self.operations += 1;
        self.elapsed_us += self.io_elapsed_us;
        if self.fail_reads_at == Some(addr) {
            return Err(ResistanceBackendError::Spi);
        }
        if addr == foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS.addr() {
            return Ok(MotorTypeNPolePairs::new()
                .with_motor_type(self.motor_type)
                .with_n_pole_pairs(50)
                .into());
        }
        if addr == PIDOUT_UQ_UD_LIMITS.addr() {
            return Ok(self.saved_voltage_limit);
        }
        if addr == PWM_MAXCNT.addr() {
            return Ok(self.readback_pwm_maxcnt);
        }
        if addr == PWM_BBM_H_BBM_L.addr() {
            return Ok(self.readback_pwm_bbm);
        }
        if addr == DSADC_MDEC_B_MDEC_A.addr() {
            return Ok(self.readback_dsadc);
        }
        if addr == PWM_SV_CHOP.addr() {
            return Ok(self.readback_pwm_sv_chop);
        }
        if addr == STATUS_FLAGS.addr() {
            if let Some(&flags) = self
                .scripted_status_reads
                .get(self.scripted_status_read_index)
            {
                self.scripted_status_read_index += 1;
                self.scripted_status_flags = flags;
            }
            return Ok(self.scripted_status_flags);
        }
        if addr == ADC_IWY_IUX.addr() {
            if self.last_motion_mode == MotionMode::UqUdExt as u32 {
                self.energized_window_current_reads += 1;
            }
            if self.last_motion_mode == 0 && !self.zero_window_counted {
                self.zero_windows_sampled += 1;
                self.zero_window_counted = true;
            }
            return Ok(self.packed_phase_current());
        }
        if addr == ABN_DECODER_COUNT.addr() {
            return Ok(self.encoder_count);
        }
        if addr == ADC_RAW_ADDR.addr() {
            return Ok(self.adc_raw_addr);
        }
        if addr == ADC_RAW_DATA.addr() {
            let vm_selector =
                u32::from(foci_tmc4671::registers::ADC_AGPI_A_RAW_ADC_VM_RAW.sub_addr);
            if self.adc_raw_addr == vm_selector {
                return Ok(u32::from(self.scripted_adc_vm_raw));
            }
            return Ok(0);
        }
        Ok(0)
    }

    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ResistanceBackendError> {
        self.record_event(BackendEvent::Write(addr, value));
        self.operations += 1;
        self.elapsed_us += self.io_elapsed_us;
        self.write_count += 1;
        self.writes
            .push(RecordedWrite { addr, value })
            .expect("write log overflow");
        if addr == UQ_UD_EXT.addr() {
            self.last_ud = (value & 0xffff) as u16 as i16;
        }
        if addr == MODE_RAMP_MODE_MOTION.addr() {
            self.last_motion_mode = value;
            if value == MotionMode::UqUdExt as u32 {
                self.energized_mode_write_us = Some(self.elapsed_us);
                self.energized_window_current_reads = 0;
            }
            self.zero_window_counted = false;
        }
        if addr == ADC_RAW_ADDR.addr() {
            self.adc_raw_addr = value;
        }
        if self.fail_write_once_at == Some(addr) {
            self.fail_write_once_at = None;
            return Err(ResistanceBackendError::Spi);
        }
        if addr == STATUS_FLAGS.addr() && value == 0 {
            self.scripted_status_flags = 0;
        }
        if let Some(threshold) = self.fail_after_writes
            && self.write_count > threshold
        {
            return Err(ResistanceBackendError::Spi);
        }
        if self.fail_writes_after_trip && self.power_stage_tripped {
            return Err(ResistanceBackendError::Spi);
        }
        Ok(())
    }

    async fn delay_ms(&mut self, ms: u64) {
        self.elapsed_us = self.elapsed_us.saturating_add(ms.saturating_mul(1000));
        if self.last_motion_mode == 8 {
            self.encoder_count = self
                .encoder_count
                .wrapping_add(self.scripted_encoder_delta_count);
        }
    }

    fn elapsed_us(&self) -> u64 {
        let index = self.scripted_elapsed_index.get();
        if let Some(&timestamp) = self.scripted_elapsed_us.get(index) {
            self.scripted_elapsed_index.set(index + 1);
            return timestamp;
        }
        self.elapsed_us
    }

    fn set_enable_pin(&mut self, enabled: bool) {
        self.record_event(BackendEvent::Enable(enabled));
        self.enable_pin = enabled;
        self.enable_transitions
            .push(enabled)
            .expect("enable transition log overflow");
    }

    fn interlock_tripped(&self) -> bool {
        self.interlock_tripped
    }

    fn trip_power_stage(&mut self) {
        self.record_event(BackendEvent::TripPowerStage);
        self.enable_pin = false;
        self.power_stage_tripped = true;
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        self.armed_for_open_loop_voltage
    }

    fn profile(&self) -> ResistanceProfile {
        self.profile
    }

    fn encoder_counts_per_rev(&self) -> u32 {
        self.encoder_counts_per_rev
    }

    fn feedback_config(&self) -> crate::config::FeedbackConfig {
        crate::config::FeedbackConfig::abn_abn()
    }

    fn adc_offsets_ready(&self) -> bool {
        self.adc_offsets_ready
    }

    fn bus_voltage_sense(&self) -> Option<BusVoltageSense> {
        self.bus_voltage_sense
    }
}

fn assert_write_order(writes: &[RecordedWrite], first_addr: u8, second_addr: u8) {
    let first = writes
        .iter()
        .position(|write| write.addr == first_addr)
        .expect("first address never written");
    let second = writes
        .iter()
        .position(|write| write.addr == second_addr)
        .expect("second address never written");
    assert!(
        first < second,
        "expected addr {first_addr:#x} before {second_addr:#x}, got {first} >= {second}"
    );
}

#[test]
fn runner_writes_voltage_limit_before_chopper() {
    let mut backend = FakeResistanceBackend::standard();

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert!(result.result().is_ok(), "expected ok, got {result:?}");
    assert_write_order(
        &backend.writes,
        PIDOUT_UQ_UD_LIMITS.addr(),
        PWM_SV_CHOP.addr(),
    );
}

#[test]
fn three_phase_motor_is_excited_in_the_rotor_frame() {
    let phi_e_selections = |motor_type| {
        let mut backend = FakeResistanceBackend::standard();
        backend.motor_type = motor_type;
        let _ = block_on(run_resistance_identification(
            &mut backend,
            ResistanceRunMode::CommissioningManagedEnable,
        ));
        backend
            .writes
            .iter()
            .filter(|write| write.addr == PHI_E_SELECTION.addr())
            .map(|write| write.value)
            .next()
    };

    assert_eq!(phi_e_selections(MotorType::TwoPhase), Some(1));
    assert_eq!(phi_e_selections(MotorType::ThreePhase), Some(3));
}

#[test]
fn current_evidence_separates_individual_peak_from_completed_mean() {
    let mut evidence = ResistanceCurrentEvidence::new(1_600);

    evidence.observe_energized_sample(200);
    evidence.observe_energized_sample(-320);

    assert_eq!(evidence.peak_abs_current_count(), 320);
    assert_eq!(evidence.max_abs_steady_mean_current_count(), 0);
    assert_eq!(evidence.current_ceiling_count(), 1_600);
    assert!(!evidence.power_stage_tripped());

    evidence.record_completed_energized_mean(-250);

    assert_eq!(evidence.peak_abs_current_count(), 320);
    assert_eq!(evidence.max_abs_steady_mean_current_count(), 250);
}

#[test]
fn current_evidence_represents_i16_min_without_narrowing() {
    let mut evidence = ResistanceCurrentEvidence::new(u16::MAX);

    evidence.observe_energized_sample(i16::MIN);

    assert_eq!(evidence.peak_abs_current_count(), 32_768);
}

#[test]
fn zero_window_samples_do_not_update_current_evidence() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_zero_currents = heapless::Vec::from_slice(&[1_500]).unwrap();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut now = drive_to_first_sample(&mut runner, &mut backend, 1_000);

    for _ in 0..SAMPLES_PER_WINDOW_FOR_TEST {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::ReadyNow => break,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }

    assert_eq!(runner.current_evidence().peak_abs_current_count(), 0);
    assert_eq!(
        runner
            .current_evidence()
            .max_abs_steady_mean_current_count(),
        0
    );
}

#[test]
fn successful_run_carries_observed_current_evidence() {
    let mut backend = FakeResistanceBackend::standard();

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .result()
    .expect("standard resistance run should succeed");

    assert_eq!(result.current.peak_abs_current_count(), 1_440);
    assert_eq!(result.current.max_abs_steady_mean_current_count(), 1_440);
    assert_eq!(result.current.current_ceiling_count(), 1_600);
    assert!(!result.current.power_stage_tripped());
}

#[test]
fn energized_monitoring_starts_one_period_after_excitation_entry() {
    let mut backend = FakeResistanceBackend::standard();
    backend.profile.settle_ms = 10;
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut now = 1_000;

    let first_monitoring_deadline = loop {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline)
                if backend.last_motion_mode == MotionMode::UqUdExt as u32 =>
            {
                break deadline;
            }
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    };
    let excitation_start = backend
        .energized_mode_write_us
        .expect("energized mode write must anchor the schedule");
    let period_us = u64::from(backend.profile.sample_interval_ms) * 1_000;

    assert_eq!(first_monitoring_deadline, excitation_start + period_us);
}

#[test]
fn exact_settle_boundary_begins_eight_sample_acquisition() {
    let mut backend = FakeResistanceBackend::standard();
    backend.profile.settle_ms = 10;
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut now = 1_000;

    loop {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline)
                if backend.last_motion_mode == MotionMode::UqUdExt as u32 =>
            {
                now = deadline;
                break;
            }
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }

    let first_sample = poll_runner(&mut runner, &mut backend, now);
    assert!(matches!(first_sample, ResistancePoll::At(_)));
    assert_eq!(
        runner.current_evidence().peak_abs_current_count(),
        backend.last_ud.unsigned_abs()
    );
    assert_eq!(
        runner
            .current_evidence()
            .max_abs_steady_mean_current_count(),
        0
    );
    now = match first_sample {
        ResistancePoll::At(deadline) => deadline,
        _ => unreachable!(),
    };

    loop {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::ReadyNow => break,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }

    assert_eq!(backend.energized_window_current_reads, 9);
}

#[test]
fn wake_crossing_settle_counts_only_acquisition_deadlines() {
    assert_eq!(
        acquisition_opportunities_through(5_000, 10_000, 10_000, 5_000),
        Ok(1)
    );
}

#[test]
fn delayed_wake_crossing_settle_takes_one_acquisition_sample_without_catch_up() {
    let mut backend = FakeResistanceBackend::standard();
    backend.profile.settle_ms = 10;
    let period_us = u64::from(backend.profile.sample_interval_ms) * 1_000;
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut now = 1_000;

    loop {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline)
                if backend.last_motion_mode == MotionMode::UqUdExt as u32 =>
            {
                now = deadline + period_us;
                break;
            }
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }
    let completed_before = runner.timing_evidence().completed_windows;

    loop {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::ReadyNow => break,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }

    assert_eq!(backend.energized_window_current_reads, 8);
    let timing = runner.timing_evidence();
    assert_eq!(timing.completed_windows, completed_before + 1);
    assert_eq!(timing.missed_samples, 1);
    assert_eq!(timing.max_consecutive_misses, 1);
}

#[test]
fn zero_window_still_waits_for_settle_before_sampling() {
    let mut backend = FakeResistanceBackend::standard();
    backend.profile.zero_settle_ms = 10;
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);

    assert_eq!(backend.zero_windows_sampled, 0);
    assert_eq!(
        poll_runner(&mut runner, &mut backend, due - 1),
        ResistancePoll::At(due)
    );
    assert_eq!(backend.zero_windows_sampled, 0);

    assert!(matches!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::At(_)
    ));
    assert_eq!(backend.zero_windows_sampled, 1);
}

#[test]
fn runner_rejects_wrong_power_stage() {
    let mut backend = FakeResistanceBackend::standard();
    backend.readback_pwm_maxcnt = 2047;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .result()
    .unwrap_err();

    assert_eq!(err, ResistanceError::WrongPowerStage);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_restores_safe_state_on_backend_error() {
    let mut backend = FakeResistanceBackend::standard();
    backend.fail_after_writes = Some(4);

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .result()
    .unwrap_err();

    assert_eq!(err, ResistanceError::BackendIo);
    assert!(backend.wrote_safe_state());
}

#[test]
fn externally_armed_mode_rejects_when_motor_is_not_armed() {
    let mut backend = FakeResistanceBackend::standard();
    backend.armed_for_open_loop_voltage = false;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .result()
    .unwrap_err();

    assert_eq!(err, ResistanceError::NotArmed);
}

#[test]
fn runner_aborts_when_profile_peak_current_is_exceeded() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_energized_current_count = 1800;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .result()
    .unwrap_err();

    assert_eq!(err, ResistanceError::PeakCurrentExceeded);
    assert!(backend.wrote_safe_state());
}

#[test]
fn failed_terminal_retains_triggering_current_evidence() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_energized_current_count = 1_800;

    let terminal = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert_eq!(terminal.result(), Err(ResistanceError::PeakCurrentExceeded));
    assert_eq!(terminal.current().peak_abs_current_count(), 1_800);
    assert_eq!(terminal.current().max_abs_steady_mean_current_count(), 0);
    assert_eq!(terminal.current().current_ceiling_count(), 1_600);
    assert!(terminal.current().power_stage_tripped());
}

#[test]
fn cleanup_error_terminal_preserves_prior_trip_evidence() {
    let mut backend = FakeResistanceBackend::standard();
    for _ in 0..107 {
        backend.scripted_energized_samples.push(1_000).unwrap();
    }
    backend.scripted_energized_samples.push(1_800).unwrap();
    backend.fail_writes_after_trip = true;

    let terminal = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert_eq!(terminal.result(), Err(ResistanceError::BackendIo));
    assert_eq!(terminal.current().peak_abs_current_count(), 1_800);
    assert_eq!(
        terminal.current().max_abs_steady_mean_current_count(),
        1_000
    );
    assert_eq!(terminal.current().current_ceiling_count(), 1_600);
    assert!(terminal.current().power_stage_tripped());
}

#[test]
fn first_over_limit_settle_sample_trips_before_status_or_restore_io() {
    let mut backend = FakeResistanceBackend::standard();
    backend.profile.settle_ms = 10;
    backend.scripted_energized_current_count = 1_800;
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut now = 1_000;

    loop {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline)
                if backend.last_motion_mode == MotionMode::UqUdExt as u32 =>
            {
                now = deadline;
                break;
            }
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }
    backend.start_event_recording();

    assert_eq!(
        poll_runner(&mut runner, &mut backend, now),
        ResistancePoll::ReadyNow
    );
    assert_eq!(
        backend.events.as_slice(),
        &[
            BackendEvent::Read(ADC_IWY_IUX.addr()),
            BackendEvent::TripPowerStage,
            BackendEvent::Enable(false),
        ]
    );
    assert!(!backend.enable_pin);
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::PeakCurrentExceeded)
    );
    assert_eq!(runner.current_evidence().peak_abs_current_count(), 1_800);
    assert!(runner.current_evidence().power_stage_tripped());
}

#[test]
fn early_spike_terminates_before_later_normal_samples() {
    let mut backend = FakeResistanceBackend::standard();
    backend.profile.settle_ms = 10;
    backend.scripted_energized_samples = heapless::Vec::from_slice(&[1_800, 100, 100]).unwrap();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut now = 1_000;

    loop {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline)
                if backend.last_motion_mode == MotionMode::UqUdExt as u32 =>
            {
                now = deadline;
                break;
            }
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }

    assert_eq!(
        poll_runner(&mut runner, &mut backend, now),
        ResistancePoll::ReadyNow
    );
    assert_eq!(backend.scripted_energized_sample_index, 1);
    assert_eq!(backend.energized_window_current_reads, 1);
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::PeakCurrentExceeded)
    );
}

#[test]
fn sample_ceiling_uses_strict_unsigned_absolute_comparison() {
    for (current, trips) in [
        (1_600, false),
        (1_601, true),
        (-1_601, true),
        (i16::MIN, true),
    ] {
        let mut backend = FakeResistanceBackend::standard();
        backend.profile.settle_ms = 10;
        backend.scripted_energized_samples = heapless::Vec::from_slice(&[current]).unwrap();
        let mut runner = ResistanceRunner::new(
            ResistanceRunMode::CommissioningManagedEnable,
            backend.profile(),
            1_000,
        )
        .unwrap();
        let mut now = 1_000;

        loop {
            match poll_runner(&mut runner, &mut backend, now) {
                ResistancePoll::ReadyNow => {}
                ResistancePoll::At(deadline)
                    if backend.last_motion_mode == MotionMode::UqUdExt as u32 =>
                {
                    now = deadline;
                    break;
                }
                ResistancePoll::At(deadline) => now = deadline,
                ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
            }
        }

        let poll = poll_runner(&mut runner, &mut backend, now);
        assert_eq!(
            matches!(poll, ResistancePoll::ReadyNow),
            trips,
            "current={current} poll={poll:?}"
        );
        assert_eq!(runner.current_evidence().power_stage_tripped(), trips);
        assert_eq!(
            runner.current_evidence().peak_abs_current_count(),
            current.unsigned_abs()
        );
        assert_eq!(
            runner.primary_error(),
            trips.then_some(ResistanceError::PeakCurrentExceeded)
        );
    }
}

#[test]
fn runner_stops_after_good_fit_before_unsafe_high_points() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_current_per_ud_milli = 1200;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert!(
        result.result().is_ok(),
        "expected adaptive sweep to fit, got {result:?}"
    );
    assert!(!result.result().unwrap().current.power_stage_tripped());
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == u32::from(1440u16)),
        "adaptive sweep must not apply the first predicted-unsafe high point"
    );
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == u32::from(1600u16)),
        "adaptive sweep must not continue to the fixed maximum once the fit is good"
    );
}

#[test]
fn runner_stops_after_good_fit_before_near_limit_high_point() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_current_per_ud_milli = 1100;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert!(
        result.result().is_ok(),
        "expected adaptive sweep to fit with near-limit headroom, got {result:?}"
    );
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == u32::from(1440u16)),
        "adaptive sweep must not energize a point predicted inside the peak-current headroom band"
    );
}

#[test]
fn runner_skips_predicted_unsafe_point_when_fit_is_not_yet_supported() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_current_per_ud_milli = 1400;
    backend.profile.min_selected_points = 6;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .result()
    .unwrap_err();

    assert_eq!(
        err,
        ResistanceError::MeasurementEnvelopeUnsupported,
        "predicted-unsafe next point should be rejected as an unsupported measurement envelope"
    );
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == u32::from(1280u16)),
        "next predicted-unsafe point must be skipped before energizing"
    );
}

#[test]
fn runner_sets_encoder_moved_warning_when_motion_exceeds_profile_threshold() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_encoder_delta_count = 129;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .result()
    .unwrap();

    assert!(
        result
            .warning_flags
            .contains(ResistanceWarningFlags::ENCODER_MOVED)
    );
}

#[test]
fn runner_sets_status_flags_warning_when_sampled_status_matches_mask() {
    let mut backend = FakeResistanceBackend::standard();
    let expected_status_flags = 1 << 28;
    assert_ne!(
        expected_status_flags & backend.profile().status_flags_warn_mask,
        0
    );
    backend.script_status_reads(&[0, expected_status_flags]);

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .result()
    .unwrap();

    assert!(
        result
            .warning_flags
            .contains(ResistanceWarningFlags::STATUS_FLAGS)
    );
    assert_eq!(result.status_flags_or, expected_status_flags);
    assert_eq!(result.baseline_flags, 0);
}

fn assert_written_before_restore(writes: &[RecordedWrite], addr: u8, value: u32) {
    let restore_start = writes
        .len()
        .saturating_sub(usize::from(super::state::RESTORE_WRITE_COUNT));
    assert!(
        writes[..restore_start]
            .iter()
            .any(|write| write.addr == addr && write.value == value),
        "expected addr {addr:#x}={value} written before the safe-state restore, got {writes:?}"
    );
}

#[test]
fn commissioning_mode_uses_full_sweep_authority_despite_low_saved_baseline() {
    let mut backend = FakeResistanceBackend::standard();
    backend.saved_voltage_limit = 1024;
    let max_abs_ud = u32::from(backend.profile().max_abs_ud().unsigned_abs());
    assert!(
        max_abs_ud > backend.saved_voltage_limit,
        "fixture must exercise the corrupted-clamp scenario"
    );

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert!(result.result().is_ok(), "expected ok, got {result:?}");
    assert_written_before_restore(&backend.writes, PIDOUT_UQ_UD_LIMITS.addr(), max_abs_ud);
    assert!(backend.wrote_safe_state());
}

#[test]
fn externally_armed_mode_rejects_when_saved_limit_is_below_sweep_authority() {
    let mut backend = FakeResistanceBackend::standard();
    let max_abs_ud = u32::from(backend.profile().max_abs_ud().unsigned_abs());
    backend.saved_voltage_limit = max_abs_ud - 1;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .result()
    .unwrap_err();

    assert_eq!(err, ResistanceError::OutOfBoundsParameter);
    assert!(
        backend.wrote_nothing(),
        "an authority-reject must not touch the externally-armed motor: {:?}",
        backend.writes
    );
}

#[test]
fn externally_armed_mode_rejects_wrong_motor_type_without_restore() {
    let mut backend = FakeResistanceBackend::standard();
    backend.motor_type = foci_tmc4671::registers::MotorType::Dc;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .result()
    .unwrap_err();

    assert_eq!(err, ResistanceError::InvalidMotorType);
    assert!(
        backend.wrote_nothing(),
        "a motor-type reject must not touch the externally-armed motor: {:?}",
        backend.writes
    );
}

#[test]
fn externally_armed_mode_proceeds_when_saved_limit_meets_sweep_authority() {
    let mut backend = FakeResistanceBackend::standard();
    let max_abs_ud = u32::from(backend.profile().max_abs_ud().unsigned_abs());
    backend.saved_voltage_limit = max_abs_ud;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ));

    assert!(result.result().is_ok(), "expected ok, got {result:?}");
    assert_written_before_restore(&backend.writes, PIDOUT_UQ_UD_LIMITS.addr(), max_abs_ud);
}

#[test]
fn runner_rejects_profile_exceeding_exposure_budget() {
    let mut backend = FakeResistanceBackend::standard();
    backend.profile.max_total_exposure = backend.profile.exposure_per_axis() - 1;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .result()
    .unwrap_err();

    assert_eq!(err, ResistanceError::UnsupportedProfile);
    assert!(
        backend.wrote_nothing(),
        "an exposure-budget reject must not touch the externally-armed motor: {:?}",
        backend.writes
    );
}

#[test]
fn sweep_point_is_bracketed_against_adjacent_zero_windows_not_initial_zero() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_zero_currents = heapless::Vec::from_slice(&[
        0, 0, 120, 0, 0, 0, 0, 0, 0, 0, 0, 0, 120, 0, 0, 0, 0, 0, 0, 0,
    ])
    .unwrap();

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .result()
    .expect("adjacent-bracket sweep should fit");

    assert_eq!(
        result.axis0.selected_mask & (1 << 2),
        0,
        "ud=800 point must be excluded under adjacent-zero bracketing (delta 740), got mask {:#010b}",
        result.axis0.selected_mask
    );
    assert_eq!(
        result.axis0.selected_mask, 0b0111_1000,
        "only the measured high-current points clear the threshold under adjacent bracketing"
    );
}

#[test]
fn signed_anchor_positive_current_is_zero_subtracted_like_the_negative_anchor() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_zero_currents = heapless::Vec::from_slice(&[100; 11]).unwrap();

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .result()
    .expect("nonzero-offset sweep should fit");

    let anchor = result
        .axis0
        .signed_anchor
        .expect("axis must report signed-anchor evidence");

    assert_eq!(
        anchor.positive_current_count, 860,
        "first positive anchor must be zero-subtracted (960 - 100), not raw 960"
    );
    assert_eq!(anchor.repeated_positive_current_count, 860);
    assert_eq!(anchor.negative_current_count, -1060);
}

#[test]
fn resumable_setup_performs_one_register_operation_per_poll() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();

    for _ in 0..24 {
        backend.begin_poll();
        let poll = poll_runner(&mut runner, &mut backend, 1_000);
        assert_eq!(
            backend.operations_since_poll(),
            1,
            "setup poll was not bounded"
        );
        if matches!(poll, ResistancePoll::At(_)) {
            return;
        }
        assert_eq!(poll, ResistancePoll::ReadyNow);
    }
    panic!("runner did not reach the first settle");
}

#[test]
fn resumable_sample_performs_exactly_current_and_status_reads() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut now = 1_000;

    loop {
        backend.begin_poll();
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline) => {
                now = deadline;
                backend.begin_poll();
                let sample = poll_runner(&mut runner, &mut backend, now);
                assert_eq!(backend.operations_since_poll(), 2);
                assert!(matches!(
                    sample,
                    ResistancePoll::At(_) | ResistancePoll::ReadyNow
                ));
                break;
            }
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }
}

#[test]
fn profile_that_cannot_fit_phase_deadline_rejects_before_enable() {
    let backend = FakeResistanceBackend::standard();
    let mut profile = backend.profile();
    profile.settle_ms = u16::MAX;
    profile.max_window_exposure = u64::MAX;
    profile.max_total_exposure = u64::MAX;

    assert!(matches!(
        ResistanceRunner::new(
            ResistanceRunMode::CommissioningManagedEnable,
            profile,
            1_000,
        ),
        Err(ResistanceError::PhaseDeadlineExceeded)
    ));
    assert!(backend.enable_transitions.is_empty());
}

fn drive_to_first_sample(
    runner: &mut ResistanceRunner,
    backend: &mut FakeResistanceBackend,
    now: u64,
) -> u64 {
    for _ in 0..32 {
        match poll_runner(runner, backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline) => return deadline,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }
    panic!("runner did not reach sample wait");
}

fn finish_restoration(
    runner: &mut ResistanceRunner,
    backend: &mut FakeResistanceBackend,
    now: u64,
) -> Result<super::types::ResistanceRunResult, ResistanceError> {
    for _ in 0..8 {
        backend.begin_poll();
        let poll = poll_runner(runner, backend, now);
        assert_eq!(
            backend.operations_since_poll(),
            1,
            "restore poll was not bounded"
        );
        if let ResistancePoll::Complete(result) = poll {
            return result;
        }
    }
    panic!("restoration did not complete");
}

#[test]
fn one_missed_slot_uses_ninth_opportunity_without_catch_up() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    let period = u64::from(backend.profile().sample_interval_ms) * 1_000;

    backend.begin_poll();
    let mut next = match poll_runner(&mut runner, &mut backend, due + period) {
        ResistancePoll::At(deadline) => deadline,
        poll => panic!("unexpected delayed-sample result: {poll:?}"),
    };
    assert_eq!(backend.operations_since_poll(), 2, "no catch-up reads");
    for _ in 1..SAMPLES_PER_WINDOW_FOR_TEST {
        next = match poll_runner(&mut runner, &mut backend, next) {
            ResistancePoll::At(deadline) => deadline,
            ResistancePoll::ReadyNow => break,
            poll => panic!("unexpected sample result: {poll:?}"),
        };
    }
    assert_eq!(runner.timing_evidence().completed_windows, 1);
    assert_eq!(runner.timing_evidence().missed_samples, 1);
}

const SAMPLES_PER_WINDOW_FOR_TEST: u8 = 8;

#[test]
fn active_window_snapshot_is_available_for_shutdown_diagnostics() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);

    assert!(matches!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::At(_)
    ));
    let timing = runner.timing_evidence_with_active();

    assert_eq!(timing.completed_windows, 0);
    assert_eq!(timing.valid_samples, 1);
    assert_ne!(timing.requested_period_us, 0);
}

#[test]
fn resistance_sample_completion_at_next_deadline_is_accepted() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    let period = u64::from(backend.profile().sample_interval_ms) * 1_000;
    backend.script_elapsed(&[due, due + 20, due + 40, due + period]);

    assert_eq!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::At(due + period)
    );
    let timing = runner.timing_evidence_with_active();
    assert_eq!(timing.valid_samples, 1);
    assert_eq!(timing.max_poll_wall_us, period as u32);
    assert_eq!(timing.max_spi_wall_us, 40);
}

#[test]
fn resistance_bookkeeping_overrun_rejects_without_committing_sample() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    let period = u64::from(backend.profile().sample_interval_ms) * 1_000;
    backend.script_elapsed(&[due, due + 20, due + 40, due + period + 1]);

    assert_eq!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::ReadyNow
    );
    assert!(!backend.enable_pin);
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::TimingRejected)
    );
    let timing = runner.timing_evidence();
    assert_eq!(timing.valid_samples, 0);
    assert_eq!(timing.max_poll_wall_us, period as u32 + 1);
    assert_eq!(timing.max_spi_wall_us, 40);
}

#[test]
fn second_missed_slot_rejects_the_window() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    let period = u64::from(backend.profile().sample_interval_ms) * 1_000;
    let next = match poll_runner(&mut runner, &mut backend, due + period) {
        ResistancePoll::At(deadline) => deadline,
        poll => panic!("unexpected first miss result: {poll:?}"),
    };

    assert_eq!(
        poll_runner(&mut runner, &mut backend, next + period),
        ResistancePoll::ReadyNow
    );
    assert!(!backend.enable_pin, "timing rejection must lower GPIO");
    let timing = runner.timing_evidence();
    assert_eq!(timing.completed_windows, 0);
    assert_eq!(timing.requested_period_us, period as u16);
    assert_eq!(timing.valid_samples, 2);
    assert_eq!(timing.missed_samples, 2);
    assert_eq!(timing.max_consecutive_misses, 1);
    assert_eq!(
        finish_restoration(&mut runner, &mut backend, next + period),
        Err(ResistanceError::TimingRejected)
    );
}

#[test]
fn absolute_phase_deadline_transitions_to_gpio_first_restore() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let _ = drive_to_first_sample(&mut runner, &mut backend, 1_000);

    assert_eq!(
        poll_runner(&mut runner, &mut backend, 20_001_000),
        ResistancePoll::ReadyNow
    );
    assert!(!backend.enable_pin);
    assert_eq!(
        finish_restoration(&mut runner, &mut backend, 20_001_000),
        Err(ResistanceError::PhaseDeadlineExceeded)
    );
}

#[test]
fn critical_status_stops_measurement_and_lowers_gpio_before_restore_io() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    backend.scripted_status_flags = 1 << 26;
    let writes_before = backend.writes.len();

    assert_eq!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::ReadyNow
    );
    assert!(!backend.enable_pin);
    assert_eq!(
        backend.writes.len(),
        writes_before,
        "restore await ran too early"
    );
    assert_eq!(runner.status_flags_or(), 1 << 26);
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::CriticalStatus)
    );
    assert_eq!(
        finish_restoration(&mut runner, &mut backend, due),
        Err(ResistanceError::CriticalStatus)
    );
}

#[test]
fn setup_refuses_to_clear_status_flags_when_the_interlock_is_tripped() {
    let mut backend = FakeResistanceBackend::standard();
    backend.interlock_tripped = true;
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();

    let mut now = 1_000;
    for _ in 0..32 {
        if runner.primary_error().is_some() {
            break;
        }
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }

    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::CriticalStatus)
    );
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == STATUS_FLAGS.addr()),
        "setup must not write STATUS_FLAGS behind a tripped interlock"
    );
    assert_eq!(
        finish_restoration(&mut runner, &mut backend, now),
        Err(ResistanceError::CriticalStatus)
    );
}

#[test]
fn setup_refuses_to_enable_when_the_interlock_trips_after_the_status_clear() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();

    let mut now = 1_000;
    let cleared = |backend: &FakeResistanceBackend| {
        backend
            .writes
            .iter()
            .any(|write| write.addr == STATUS_FLAGS.addr())
    };
    for _ in 0..32 {
        if cleared(&backend) {
            break;
        }
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
    }
    assert!(
        cleared(&backend),
        "setup never reached the STATUS_FLAGS clear"
    );
    assert!(!backend.enable_pin);

    backend.interlock_tripped = true;

    assert_eq!(
        poll_runner(&mut runner, &mut backend, now),
        ResistancePoll::ReadyNow
    );

    assert!(
        !backend.enable_pin,
        "setup must not raise the enable pin behind a tripped interlock"
    );
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::CriticalStatus)
    );
    assert_eq!(
        finish_restoration(&mut runner, &mut backend, now),
        Err(ResistanceError::CriticalStatus)
    );
}

#[test]
fn warning_only_status_is_retained_without_terminating_sample() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    backend.scripted_status_flags = 1 << 28;

    assert!(matches!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::At(_)
    ));
    assert!(backend.enable_pin);
    assert_eq!(runner.baseline_status_flags(), 0);
    assert_eq!(runner.status_flags_or(), 1 << 28);
    assert!(
        runner
            .warning_flags()
            .contains(ResistanceWarningFlags::STATUS_FLAGS)
    );
}

#[test]
fn resumable_success_matches_compatibility_loop_and_remains_terminal() {
    let mut compatibility_backend = FakeResistanceBackend::standard();
    let expected = block_on(run_resistance_identification(
        &mut compatibility_backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .result()
    .unwrap();
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut now = 1_000;

    let terminal = loop {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::Complete(result) => break result,
        }
    };

    assert_eq!(terminal, Ok(expected));
    assert_eq!(
        poll_runner(&mut runner, &mut backend, now),
        ResistancePoll::Complete(Ok(expected))
    );
}

#[test]
fn backward_sample_timestamp_rejects_timing_and_lowers_gpio() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    backend.script_elapsed(&[100, 90, 110]);

    assert_eq!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::ReadyNow
    );
    assert!(!backend.enable_pin);
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::TimingRejected)
    );
    let timing = runner.timing_evidence();
    assert_eq!(timing.completed_windows, 0);
    assert_ne!(timing.requested_period_us, 0);
    assert!(timing.overflowed);
}

#[test]
fn unrepresentable_sample_spi_sum_rejects_timing() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    backend.script_elapsed(&[0, u64::from(u32::MAX), u64::from(u32::MAX) + 1]);

    assert_eq!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::ReadyNow
    );
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::TimingRejected)
    );
    let timing = runner.timing_evidence();
    assert_eq!(timing.completed_windows, 0);
    assert_ne!(timing.requested_period_us, 0);
    assert!(timing.overflowed);
}

#[test]
fn inherited_status_is_retained_cleared_and_absent_from_first_sample() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_status_flags = 0x7000_4480;
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();

    let first_settle_deadline = drive_to_first_sample(&mut runner, &mut backend, 1_000);

    assert!(first_settle_deadline > 1_000);
    assert!(
        backend
            .writes
            .iter()
            .any(|write| { write.addr == STATUS_FLAGS.addr() && write.value == 0 })
    );
    assert_eq!(runner.baseline_status_flags(), 0x7000_4480);
    assert_eq!(runner.status_flags_or(), 0);
    assert!(
        !runner
            .warning_flags()
            .contains(ResistanceWarningFlags::STATUS_FLAGS)
    );
    assert_eq!(runner.primary_error(), None);
    assert!(backend.enable_pin);
    assert!(matches!(
        poll_runner(&mut runner, &mut backend, first_settle_deadline),
        ResistancePoll::At(_) | ResistancePoll::ReadyNow
    ));
    assert_eq!(runner.baseline_status_flags(), 0x7000_4480);
    assert_eq!(runner.status_flags_or(), 0);
    assert_eq!(runner.primary_error(), None);
    assert!(backend.enable_pin);
}

#[test]
fn fresh_critical_status_after_clear_refuses_enable_and_starts_restore() {
    let mut backend = FakeResistanceBackend::standard();
    backend.script_status_reads(&[0x7000_4480, 1 << 26]);
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut writes_before_fresh_read = None;

    for _ in 0..24 {
        if backend.scripted_status_read_index == 1 {
            writes_before_fresh_read = Some(backend.writes.len());
        }
        backend.begin_poll();
        let poll = poll_runner(&mut runner, &mut backend, 1_000);
        assert_eq!(backend.operations_since_poll(), 1);
        match poll {
            ResistancePoll::ReadyNow if runner.primary_error().is_none() => {}
            ResistancePoll::ReadyNow => break,
            poll => panic!("unexpected pre-enable poll: {poll:?}"),
        }
    }

    assert_eq!(runner.baseline_status_flags(), 0x7000_4480);
    assert_eq!(runner.status_flags_or(), 1 << 26);
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::CriticalStatus)
    );
    assert!(!backend.enable_transitions.iter().any(|&enabled| enabled));
    assert!(!backend.enable_pin);
    assert_eq!(
        backend.writes.len(),
        writes_before_fresh_read.expect("fresh status read was not reached"),
        "restore I/O ran before the GPIO-first transition returned"
    );
}

#[test]
fn critical_sample_status_precedes_coincident_timing_rejection() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_status_flags = 0x7000_4480;
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    backend.scripted_status_flags = 1 << 26;
    backend.script_elapsed(&[0, u64::from(u32::MAX), u64::from(u32::MAX) + 1]);
    let writes_before = backend.writes.len();

    assert_eq!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::ReadyNow
    );
    assert_eq!(runner.baseline_status_flags(), 0x7000_4480);
    assert_eq!(runner.status_flags_or(), 1 << 26);
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::CriticalStatus)
    );
    assert!(!backend.enable_pin);
    assert_eq!(
        backend.writes.len(),
        writes_before,
        "restore await ran before the GPIO-first transition returned"
    );
}

#[test]
fn restore_failure_replaces_success_and_remains_terminal() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let mut now = 1_000;
    let mut was_enabled = false;

    for _ in 0..5_000 {
        match poll_runner(&mut runner, &mut backend, now) {
            ResistancePoll::ReadyNow => {}
            ResistancePoll::At(deadline) => now = deadline,
            ResistancePoll::Complete(result) => panic!("unexpected completion: {result:?}"),
        }
        was_enabled |= backend.enable_pin;
        if was_enabled && !backend.enable_pin && runner.primary_error().is_none() {
            break;
        }
    }
    assert!(
        was_enabled && !backend.enable_pin,
        "runner did not enter restore"
    );
    backend.fail_next_write();

    assert_eq!(
        finish_restoration(&mut runner, &mut backend, now),
        Err(ResistanceError::BackendIo)
    );
    assert_eq!(
        poll_runner(&mut runner, &mut backend, now),
        ResistancePoll::Complete(Err(ResistanceError::BackendIo))
    );
}

#[test]
fn restore_failure_replaces_existing_primary_error_and_remains_terminal() {
    let mut backend = FakeResistanceBackend::standard();
    let mut runner = ResistanceRunner::new(
        ResistanceRunMode::CommissioningManagedEnable,
        backend.profile(),
        1_000,
    )
    .unwrap();
    let due = drive_to_first_sample(&mut runner, &mut backend, 1_000);
    backend.scripted_status_flags = 1 << 26;
    assert_eq!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::ReadyNow
    );
    assert_eq!(
        runner.primary_error(),
        Some(ResistanceError::CriticalStatus)
    );
    backend.fail_next_write();

    assert_eq!(
        finish_restoration(&mut runner, &mut backend, due),
        Err(ResistanceError::BackendIo)
    );
    assert_eq!(
        poll_runner(&mut runner, &mut backend, due),
        ResistancePoll::Complete(Err(ResistanceError::BackendIo))
    );
}

fn first_mode_write_index(writes: &[RecordedWrite]) -> usize {
    writes
        .iter()
        .position(|write| write.addr == MODE_RAMP_MODE_MOTION.addr())
        .expect("setup never wrote MODE_RAMP_MODE_MOTION")
}

fn energized_ud_writes(writes: &[RecordedWrite]) -> heapless::Vec<i16, 32> {
    writes
        .iter()
        .filter(|write| write.addr == UQ_UD_EXT.addr())
        .map(|write| (write.value & 0xffff) as u16 as i16)
        .filter(|ud| *ud != 0)
        .collect()
}

fn adc_raw_addr_writes(writes: &[RecordedWrite]) -> heapless::Vec<u32, 8> {
    writes
        .iter()
        .filter(|write| write.addr == ADC_RAW_ADDR.addr())
        .map(|write| write.value)
        .collect()
}

#[test]
fn bus_voltage_read_restores_adc_raw_addr_before_setup() {
    let mut backend = FakeResistanceBackend::standard();
    backend.adc_raw_addr = 7;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert!(result.result().is_ok(), "{result:?}");
    let select = backend
        .writes
        .iter()
        .position(|w| w.addr == ADC_RAW_ADDR.addr() && w.value == 1)
        .expect("ADC_RAW_ADDR never selected the VM sub-register");
    let restore = backend
        .writes
        .iter()
        .position(|w| w.addr == ADC_RAW_ADDR.addr() && w.value == 7)
        .expect("ADC_RAW_ADDR never restored");
    let setup = first_mode_write_index(&backend.writes);
    assert!(
        select < restore && restore < setup,
        "{select} {restore} {setup}"
    );
    assert_eq!(energized_ud_writes(&backend.writes)[0], 512);
}

#[test]
fn failed_bus_voltage_read_still_restores_adc_raw_addr() {
    let mut backend = FakeResistanceBackend::standard();
    backend.adc_raw_addr = 7;
    backend.fail_reads_at = Some(ADC_RAW_DATA.addr());

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert_eq!(result.result(), Err(ResistanceError::BackendIo));
    assert_eq!(adc_raw_addr_writes(&backend.writes).as_slice(), &[1, 7]);
    assert!(
        !backend
            .writes
            .iter()
            .any(|w| w.addr == MODE_RAMP_MODE_MOTION.addr())
    );
    assert!(backend.enable_transitions.is_empty());
}

#[test]
fn failed_selector_write_still_attempts_the_adc_raw_addr_restore() {
    let mut backend = FakeResistanceBackend::standard();
    backend.adc_raw_addr = 7;
    backend.fail_write_once_at = Some(ADC_RAW_ADDR.addr());
    backend.start_event_recording();

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert_eq!(result.result(), Err(ResistanceError::BackendIo));
    assert_eq!(adc_raw_addr_writes(&backend.writes).as_slice(), &[1, 7]);
    assert!(
        !backend
            .writes
            .iter()
            .any(|w| w.addr == MODE_RAMP_MODE_MOTION.addr() || w.addr == ADC_RAW_DATA.addr())
    );
    assert!(
        !backend
            .events
            .iter()
            .any(|e| matches!(e, BackendEvent::Read(addr) if *addr == ADC_RAW_DATA.addr())),
        "the data read must be skipped after a failed selector write"
    );
    assert!(backend.enable_transitions.is_empty());
}

#[test]
fn ladder_scales_to_the_measured_bus_voltage() {
    let mut backend = FakeResistanceBackend::standard();
    backend.script_bus_millivolts(24_000);
    backend.scripted_current_per_ud_milli = 500;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert!(result.result().is_ok(), "{result:?}");
    let uds = energized_ud_writes(&backend.writes);
    assert_eq!(uds[0], 1024, "first energized point must be the scaled 512");
    assert!(
        uds.contains(&1920),
        "repeated anchor must be the scaled 960: {uds:?}"
    );
    assert!(
        uds.contains(&-1920),
        "negative anchor must be the scaled -960: {uds:?}"
    );
    assert!(
        !result
            .result()
            .unwrap()
            .warning_flags
            .contains(ResistanceWarningFlags::BUS_VOLTAGE_UNSCALED)
    );
    let limit_write = backend
        .writes
        .iter()
        .find(|w| w.addr == PIDOUT_UQ_UD_LIMITS.addr())
        .expect("setup writes the voltage limit");
    assert_eq!(
        limit_write.value, 3200,
        "setup limit must be the scaled max_abs_ud"
    );
}

#[test]
fn out_of_range_bus_reading_fails_before_any_setup_write() {
    for raw in [33_000u16, 33_000 + 18_852] {
        let mut backend = FakeResistanceBackend::standard();
        backend.scripted_adc_vm_raw = raw;

        let result = block_on(run_resistance_identification(
            &mut backend,
            ResistanceRunMode::CommissioningManagedEnable,
        ));

        assert_eq!(
            result.result(),
            Err(ResistanceError::BusVoltageOutOfRange),
            "raw {raw}"
        );
        assert!(
            !backend
                .writes
                .iter()
                .any(|w| w.addr == MODE_RAMP_MODE_MOTION.addr()),
            "raw {raw}: setup must not start"
        );
        assert_eq!(
            adc_raw_addr_writes(&backend.writes).as_slice(),
            &[1, 0],
            "raw {raw}"
        );
        assert!(backend.enable_transitions.is_empty(), "raw {raw}");
    }
}

#[test]
fn missing_bus_sense_runs_unscaled_without_touching_adc_raw_addr() {
    let mut backend = FakeResistanceBackend::standard();
    backend.bus_voltage_sense = None;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    let run = result.result().expect("unscaled sweep completes");
    assert!(
        run.warning_flags
            .contains(ResistanceWarningFlags::BUS_VOLTAGE_UNSCALED)
    );
    assert!(adc_raw_addr_writes(&backend.writes).is_empty());
    assert_eq!(energized_ud_writes(&backend.writes)[0], 512);
}

#[test]
fn externally_armed_rechecks_voltage_authority_against_the_scaled_ladder() {
    let mut refused = FakeResistanceBackend::standard();
    refused.saved_voltage_limit = 1_600;
    refused.script_bus_millivolts(24_000);

    let result = block_on(run_resistance_identification(
        &mut refused,
        ResistanceRunMode::ExternallyArmed,
    ));

    assert_eq!(result.result(), Err(ResistanceError::OutOfBoundsParameter));
    assert!(
        !refused
            .writes
            .iter()
            .any(|w| w.addr == MODE_RAMP_MODE_MOTION.addr() || w.addr == PIDOUT_UQ_UD_LIMITS.addr())
    );
    assert!(refused.enable_transitions.is_empty());

    let mut allowed = FakeResistanceBackend::standard();
    allowed.saved_voltage_limit = 1_600;

    let result = block_on(run_resistance_identification(
        &mut allowed,
        ResistanceRunMode::ExternallyArmed,
    ));

    assert!(
        result.result().is_ok(),
        "48 V with a 1,600 authority must still run: {result:?}"
    );
}

#[test]
fn scaled_ladder_that_breaks_the_exposure_budget_fails_before_energizing() {
    let mut backend = FakeResistanceBackend::standard();
    backend.profile.max_window_exposure = 1_600u64 * 1_600 * 500;
    backend.script_bus_millivolts(24_000);

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert_eq!(result.result(), Err(ResistanceError::UnsupportedProfile));
    assert!(
        !backend
            .writes
            .iter()
            .any(|w| w.addr == MODE_RAMP_MODE_MOTION.addr())
    );
    assert!(backend.enable_transitions.is_empty());
}
