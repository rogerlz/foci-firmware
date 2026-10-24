//! Shared mock SPI and commissioning-backend test support for `tmc_control`.

use core::convert::Infallible;
use core::sync::atomic::AtomicU64;

use embedded_hal_async::spi::{ErrorKind, ErrorType, Operation, SpiDevice};

use crate::commissioning::backend::{CommissionBackendHooks, SharedCommissioningBackend};
use crate::config::{FeedbackConfig, TmcConfig};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct MockSpiError;

impl embedded_hal_async::spi::Error for MockSpiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

#[derive(Default)]
pub(crate) struct RuntimeMockSpi {
    pub(crate) writes: std::vec::Vec<(u8, u32)>,
    pub(crate) write_attempts: std::vec::Vec<(u8, u32)>,
    pub(crate) read_requests: std::vec::Vec<u8>,
    pub(crate) reads: std::collections::VecDeque<u32>,
    pub(crate) failed_write_addresses: std::vec::Vec<u8>,
    pub(crate) echo_writes: bool,
    pub(crate) fail_reads: bool,
    pub(crate) last_read_request: Option<u8>,
    pub(crate) written: std::collections::BTreeMap<u8, u32>,
    pub(crate) pending_config_race: Option<(u8, std::boxed::Box<dyn Fn()>)>,
    pub(crate) write_observer: Option<std::boxed::Box<dyn Fn(u8, u32)>>,
}

impl RuntimeMockSpi {
    pub(crate) fn with_read(mut self, value: u32) -> Self {
        self.reads.push_back(value);
        self
    }

    pub(crate) fn fail_writes_to(mut self, addr: u8) -> Self {
        self.failed_write_addresses.push(addr);
        self
    }

    pub(crate) fn attempted_write_to(&self, register: foci_tmc4671::registers::Register) -> bool {
        self.write_attempts
            .iter()
            .any(|(addr, _)| *addr == register.addr())
    }

    pub(crate) fn read_requested(&self, register: foci_tmc4671::registers::Register) -> bool {
        self.read_requests
            .iter()
            .any(|addr| *addr == register.addr())
    }

    pub(crate) fn echo_writes(mut self) -> Self {
        self.echo_writes = true;
        self
    }

    pub(crate) fn fail_reads(mut self) -> Self {
        self.fail_reads = true;
        self
    }

    pub(crate) fn race_pending_config_on_write(
        mut self,
        addr: u8,
        handler: impl Fn() + 'static,
    ) -> Self {
        self.pending_config_race = Some((addr, std::boxed::Box::new(handler)));
        self
    }

    pub(crate) fn observe_writes(mut self, observer: impl Fn(u8, u32) + 'static) -> Self {
        self.write_observer = Some(std::boxed::Box::new(observer));
        self
    }
}

impl ErrorType for RuntimeMockSpi {
    type Error = MockSpiError;
}

impl SpiDevice<u8> for RuntimeMockSpi {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                Operation::Write(buf) if buf.len() == 5 => {
                    let addr = buf[0] & 0x7f;
                    let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    if let Some((race_addr, handler)) = &self.pending_config_race
                        && *race_addr == addr
                    {
                        handler();
                    }
                    self.write_attempts.push((addr, value));
                    if self.failed_write_addresses.contains(&addr) {
                        return Err(MockSpiError);
                    }
                    self.writes.push((addr, value));
                    self.written.insert(addr, value);
                    if let Some(observer) = &self.write_observer {
                        observer(addr, value);
                    }
                }
                Operation::Write(buf) if buf.len() == 1 => {
                    let addr = buf[0] & 0x7f;
                    self.read_requests.push(buf[0]);
                    self.last_read_request = Some(addr);
                }
                Operation::Read(buf) => {
                    if self.fail_reads {
                        return Err(MockSpiError);
                    }
                    let value = self.reads.pop_front().unwrap_or_else(|| {
                        if self.echo_writes {
                            self.last_read_request
                                .and_then(|addr| self.written.get(&addr).copied())
                                .unwrap_or(0)
                        } else {
                            0
                        }
                    });
                    buf.copy_from_slice(&value.to_be_bytes());
                }
                Operation::Write(_) | Operation::DelayNs(_) => {}
                Operation::Transfer(read, write) => {
                    read.copy_from_slice(write);
                }
                Operation::TransferInPlace(_) => {}
            }
        }
        Ok(())
    }
}

impl embedded_hal::spi::SpiDevice<u8> for RuntimeMockSpi {
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                embedded_hal::spi::Operation::Write(buf) if buf.len() == 5 => {
                    let addr = buf[0] & 0x7f;
                    let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                    self.write_attempts.push((addr, value));
                    if self.failed_write_addresses.contains(&addr) {
                        return Err(MockSpiError);
                    }
                    self.writes.push((addr, value));
                    self.written.insert(addr, value);
                }
                embedded_hal::spi::Operation::Write(buf) if buf.len() == 1 => {
                    let addr = buf[0] & 0x7f;
                    self.read_requests.push(buf[0]);
                    self.last_read_request = Some(addr);
                }
                embedded_hal::spi::Operation::Read(buf) => {
                    if self.fail_reads {
                        return Err(MockSpiError);
                    }
                    let value = self.reads.pop_front().unwrap_or_else(|| {
                        if self.echo_writes {
                            self.last_read_request
                                .and_then(|addr| self.written.get(&addr).copied())
                                .unwrap_or(0)
                        } else {
                            0
                        }
                    });
                    buf.copy_from_slice(&value.to_be_bytes());
                }
                embedded_hal::spi::Operation::Write(_)
                | embedded_hal::spi::Operation::DelayNs(_) => {}
                embedded_hal::spi::Operation::Transfer(read, write) => {
                    read.copy_from_slice(write);
                }
                embedded_hal::spi::Operation::TransferInPlace(_) => {}
            }
        }
        Ok(())
    }
}

/// Replay an ordered write log from limit `committed_limit` and offset 0, asserting after every
/// write to `PID_TORQUE_FLUX_LIMITS` or `PID_TORQUE_FLUX_OFFSET` that the applied limit plus the
/// magnitude of the applied torque offset stays within `committed_limit`.
pub(crate) fn assert_limit_plus_offset_within(writes: &[(u8, u32)], committed_limit: u32) {
    assert_limit_plus_offset_within_schedule(writes, committed_limit, &[]);
}

/// Like [`assert_limit_plus_offset_within`], with the committed limit replaced by `new_limit`
/// just before the write at `write_index` for each `(write_index, new_limit)` in `schedule`.
pub(crate) fn assert_limit_plus_offset_within_schedule(
    writes: &[(u8, u32)],
    committed_limit: u32,
    schedule: &[(usize, u32)],
) {
    use foci_tmc4671::registers::{PID_TORQUE_FLUX_LIMITS, PID_TORQUE_FLUX_OFFSET};

    let mut committed = committed_limit;
    let mut limit = committed_limit;
    let mut offset = 0u32;
    for (index, &(addr, value)) in writes.iter().enumerate() {
        if let Some(&(_, next)) = schedule.iter().find(|(at, _)| *at == index) {
            committed = next;
        }
        if addr == PID_TORQUE_FLUX_LIMITS.addr() {
            limit = value & 0xffff;
        } else if addr == PID_TORQUE_FLUX_OFFSET.addr() {
            offset = u32::from(crate::tmc::decode_signed_upper_half(value).unsigned_abs());
        } else {
            continue;
        }
        assert!(
            limit + offset <= committed,
            "write {index} ({addr:#04x} = {value:#010x}) leaves limit {limit} + |offset| \
             {offset} above committed limit {committed}: {writes:?}"
        );
    }
}

#[derive(Debug, Default)]
pub(crate) struct TestPin {
    pub(crate) high: bool,
}

impl embedded_hal::digital::ErrorType for TestPin {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for TestPin {
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.high = false;
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Infallible> {
        self.high = true;
        Ok(())
    }
}

pub(crate) static ELAPSED_US: AtomicU64 = AtomicU64::new(0);

pub(crate) static TEST_CONFIG: TmcConfig = TmcConfig {
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
    resistance_max_expected_current_count: 1600,
    resistance_scale_metadata_validated: false,
    current_scale_ma_per_lsb: 1.155,
    max_run_current_ma: 10_000,
    vm_divider_high_ohms: 71_500,
    vm_divider_low_ohms: 1_500,
    vm_adc_reference_millivolts: 2_500,
    vm_adc_center_counts: 32_767,
    motor_type_n_pole_pairs: crate::tmc_register_values::STEPPER_MOTOR_TYPE_N_POLE_PAIRS,
};
pub(crate) static TEST_FEEDBACK_CONFIGS: [FeedbackConfig; 8] = {
    let mut configs = [FeedbackConfig::abn_abn(); 8];
    configs[3] = FeedbackConfig {
        phi_e_source: crate::config::FeedbackSource::Analog,
        position_source: crate::config::FeedbackSource::Abn,
    };
    configs
};

pub(crate) struct TestHooks;

impl CommissionBackendHooks for TestHooks {
    fn commission_tmc_config() -> &'static TmcConfig {
        &TEST_CONFIG
    }

    fn commission_feedback_configs() -> &'static [FeedbackConfig] {
        &TEST_FEEDBACK_CONFIGS
    }

    fn commission_elapsed_us() -> u64 {
        ELAPSED_US.load(core::sync::atomic::Ordering::Relaxed)
    }

    fn commission_is_shutdown_requested() -> bool {
        false
    }

    async fn commission_delay_us(_us: u32) {}
}

pub(crate) type TestBackend<'a> =
    SharedCommissioningBackend<'a, RuntimeMockSpi, TestPin, TestHooks>;

#[derive(Default)]
pub(crate) struct MockRawSpi {
    pub(crate) writes: std::vec::Vec<std::vec::Vec<u8>>,
    pub(crate) fail_write_after: Option<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct MockRawSpiError;

impl embedded_hal_async::spi::Error for MockRawSpiError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for MockRawSpi {
    type Error = MockRawSpiError;
}

impl SpiDevice<u8> for MockRawSpi {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                Operation::Read(buf) => {
                    buf.copy_from_slice(&0u32.to_be_bytes());
                }
                Operation::Write(buf) => {
                    if self
                        .fail_write_after
                        .is_some_and(|limit| self.writes.len() >= limit)
                    {
                        return Err(MockRawSpiError);
                    }
                    self.writes.push(buf.to_vec())
                }
                Operation::Transfer(read, write) => {
                    read.copy_from_slice(write);
                }
                Operation::TransferInPlace(_) => {}
                Operation::DelayNs(_) => {}
            }
        }
        Ok(())
    }
}
