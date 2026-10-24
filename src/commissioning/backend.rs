//! Hardware backend capability traits for commissioning.

use core::convert::Infallible;
use core::marker::PhantomData;
use core::sync::atomic::Ordering::Relaxed;

use crate::config::TmcConfig;
use crate::tmc_control::pending_config::{PendingConfigItem, PendingConfigSink};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::{ErrorType, SpiDevice};
use foci_klipper::stepper::StepMove;
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::registers::{ABN_DECODER_COUNT, MotorTypeNPolePairs, Register};
use foci_tmc4671::spi::{SpiError, Tmc4671Spi};

/// Register I/O capability for TMC4671 commissioning operations.
#[allow(async_fn_in_trait)]
pub trait CommissionRegisterIo {
    type Error;

    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error>;

    /// Read one register for a fixed-rate capture observation.
    async fn read_capture_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        self.read_register(reg).await
    }

    /// Read two TMC4671 registers for one capture observation.
    async fn read_register_pair(
        &mut self,
        first: Register,
        second: Register,
    ) -> Result<(u32, u32), Self::Error> {
        let first = self.read_register(first).await?;
        let second = self.read_register(second).await?;
        Ok((first, second))
    }

    async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), Self::Error>;

    /// Write a register, read it back, and compare against `val`.
    ///
    /// Production board implementations and production-semantic test backends
    /// must preserve the `Tmc4671Spi` retry policy.
    async fn write_register_verified(&mut self, reg: Register, val: u32)
    -> Result<(), Self::Error>;

    /// Read a register and compare against `expected`.
    ///
    /// Production board implementations and production-semantic test backends
    /// must preserve the `Tmc4671Spi` retry policy.
    async fn read_register_verified(
        &mut self,
        reg: Register,
        expected: u32,
    ) -> Result<u32, Self::Error>;
}

/// Result of one existing register await with measured monotonic wall time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimedRegisterIo<T> {
    pub result: T,
    /// Time spent inside the register-operation await, in microseconds.
    pub wall_us: u32,
    /// Whether elapsed-time subtraction or narrowing could not be represented.
    pub overflowed: bool,
}

#[cfg(test)]
pub(crate) async fn timed_read_register<B>(
    backend: &mut B,
    reg: Register,
) -> TimedRegisterIo<Result<u32, B::Error>>
where
    B: CommissioningBackend,
{
    let start_us = backend.elapsed_us();
    let result = backend.read_register(reg).await;
    let end_us = backend.elapsed_us();
    let (wall_us, overflowed) = measured_wall_us(start_us, end_us);
    TimedRegisterIo {
        result,
        wall_us,
        overflowed,
    }
}

/// Read two registers as one backend operation and measure its wall time.
pub async fn timed_read_register_pair<B>(
    backend: &mut B,
    first: Register,
    second: Register,
) -> TimedRegisterIo<Result<(u32, u32), B::Error>>
where
    B: CommissioningBackend,
{
    let start_us = backend.elapsed_us();
    let result = backend.read_register_pair(first, second).await;
    let end_us = backend.elapsed_us();
    let (wall_us, overflowed) = measured_wall_us(start_us, end_us);
    TimedRegisterIo {
        result,
        wall_us,
        overflowed,
    }
}

#[cfg(test)]
pub(crate) async fn timed_write_register<B>(
    backend: &mut B,
    reg: Register,
    value: u32,
) -> TimedRegisterIo<Result<(), B::Error>>
where
    B: CommissioningBackend,
{
    let start_us = backend.elapsed_us();
    let result = backend.write_register(reg, value).await;
    let end_us = backend.elapsed_us();
    let (wall_us, overflowed) = measured_wall_us(start_us, end_us);
    TimedRegisterIo {
        result,
        wall_us,
        overflowed,
    }
}

fn measured_wall_us(start_us: u64, end_us: u64) -> (u32, bool) {
    let Some(wall_us) = end_us.checked_sub(start_us) else {
        return (0, true);
    };
    match u32::try_from(wall_us) {
        Ok(wall_us) => (wall_us, false),
        Err(_) => (u32::MAX, true),
    }
}

/// Explicit motor lifecycle side effects used by commissioning.
pub trait CommissionMotorControl {
    /// Store the chip-verified `MOTOR_TYPE_N_POLE_PAIRS` sentinel checked by `arm_motor_electrical`.
    fn store_motor_sentinel(&self, _value: u32) {}

    fn set_enable_pin(&mut self, state: bool);

    /// True when a fault is latched, a disable is pending, or the STATUS pin
    /// is asserted. Checked before every non-bootstrap `STATUS_FLAGS` clear
    /// and before every enable-pin assertion outside the calibration phases.
    fn interlock_tripped(&self) -> bool;

    /// Record a motor lifecycle flag transition at the exact mutation site.
    fn record_lifecycle(
        &mut self,
        _flag: crate::tmc_control::lifecycle::LifecycleFlag,
        _value: bool,
    ) {
    }

    /// Persist the outcome of a safe-state restore pass.
    ///
    /// `timestamp_ms` is a board-monotonic millisecond timestamp.
    fn record_safe_state_report(
        &mut self,
        _report: crate::tmc_safe_state::SafeStateRestoreReport,
        _timestamp_ms: u32,
    ) {
    }
}

/// Encoder sampling capability used by inner and outer commissioning.
#[allow(async_fn_in_trait)]
pub trait CommissionEncoderRead: CommissionRegisterIo {
    /// Read the current ABN encoder position, in quadrature counts.
    async fn read_mechanical_encoder_position_counts(&mut self) -> u32;

    /// Read the current ABN encoder position in counts and preserve read failures.
    async fn try_read_mechanical_encoder_position_counts(&mut self) -> Result<u32, Self::Error> {
        Ok(self.read_mechanical_encoder_position_counts().await)
    }
}

/// Cooperative timing and shutdown checks for the commissioning engine.
#[allow(async_fn_in_trait)]
pub trait CommissionTiming {
    /// Microseconds since an arbitrary reference point; strictly monotonic across a session.
    fn elapsed_us(&self) -> u64;

    fn is_shutdown_requested(&self) -> bool;

    /// Cooperative microsecond delay that yields to higher-priority tasks.
    ///
    /// Implementations must override; the default panics.
    async fn delay_us(&mut self, _us: u32) {
        panic!(
            "CommissioningBackend impl missing delay_us override; \
             firmware-side cooperative waits would not yield"
        );
    }
}

/// Whether any board-owned shutdown signal (disable request, lease expiry, shutdown, fault) is active.
pub const fn shutdown_requested_from(
    disable_signal: bool,
    lease_expired: bool,
    shutdown_active: bool,
    fault_active: bool,
) -> bool {
    disable_signal || lease_expired || shutdown_active || fault_active
}

/// Board and channel configuration values consumed by commissioning.
pub trait CommissionBoardConfig {
    fn current_limit(&self) -> PidTorqueFluxLimit;

    fn pwm_maxcnt(&self) -> u16;

    fn bbm_register(&self) -> u32;

    fn dsadc_mdec_register(&self) -> u32;

    /// Test excitation voltage magnitude.
    fn excitation_voltage(&self) -> u16;

    fn tmc_config(&self) -> &TmcConfig;

    /// The channel's captured `ADC_VM` offset, if the board sampled one at init.
    fn adc_vm_offset(&self) -> Option<crate::state::tmc::AdcVmOffsetSnapshot>;

    fn feedback_config(&self) -> crate::config::FeedbackConfig;

    /// Host-configured torque filter cutoff, including explicit zero disable.
    fn configured_torque_filter_hz(&self) -> Option<u16> {
        None
    }

    /// Host-configured flux filter cutoff, including explicit zero disable.
    fn configured_flux_filter_hz(&self) -> Option<u16> {
        None
    }

    /// Published TMC4671 position units per physical emitted pulse.
    ///
    /// Returns `None` until the complete motion scale has been configured.
    fn configured_physical_step_width(&self) -> Option<u32>;

    fn mechanical_encoder_quadrature_counts_per_rev(&self) -> u32;

    /// Configured motor electrical pole-pair count.
    ///
    /// Zero means not yet calibrated; callers must treat that as a failure.
    fn motor_pole_pairs(&self) -> u16;

    /// Whether ADC current-offset calibration has been established.
    fn adc_offsets_ready(&self) -> bool;

    fn operational_velocity_limit(&self) -> u32;

    fn operational_voltage_limit(&self) -> u32;

    /// Configured stall-detection threshold, in position-error units.
    ///
    /// Zero means unconfigured.
    fn stall_threshold_units(&self) -> u32 {
        0
    }

    /// Configured velocity feedforward gain, in permille (1000 = unity), or
    /// zero when the channel's velocity feedforward is not enabled.
    fn velocity_feedforward_gain_permille(&self) -> u32 {
        0
    }

    /// Feedforward paths currently live on this channel: bit 0 velocity,
    /// 2 acceleration. Bits 1, 3 and 4 are reserved and never set.
    fn feedforward_path_flags(&self) -> u8 {
        0
    }
}

/// Commissioning trace emission capability.
pub trait CommissionTrace {
    fn record_commission_trace_event(
        &mut self,
        _event: crate::trace::events::CommissionTraceEvent,
    ) {
    }

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation(
        &mut self,
        _observation: crate::trace::stream::VelocityTraceObservation,
    ) {
    }

    fn record_velocity_sweep_trace_record(
        &mut self,
        _record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
    }
}

/// Connect-time configuration drain capability.
#[allow(async_fn_in_trait)]
pub trait CommissionPendingConfig {
    /// Push every recorded-but-unapplied configuration item to the TMC4671.
    ///
    /// # Errors
    ///
    /// Returns the [`PendingConfigItem`] whose write failed. That item's
    /// pending bit stays set so a later drain retries it.
    async fn drain_pending_config(
        &mut self,
        sink: PendingConfigSink<'_>,
    ) -> Result<(), PendingConfigItem>;
}

/// Why a stimulus operation was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StimulusError {
    Unsupported,
    InvalidChannel,
    /// The stepper is trigger-blocked by an unhandled homing stop.
    TriggerBlocked,
    /// The step timer is active, so the clock cannot be reset.
    TimerActive,
    /// A stop left the stepper needing a clock reset before it can queue.
    NeedsReset,
    QueueRefused {
        queue_len: u32,
    },
    NotEnergized,
}

/// Step-queue stimulus a commissioning phase drives on its own channel.
pub trait CommissionStepStimulus {
    /// Step timer frequency in Hz; 0 when no stimulus is available.
    ///
    /// A board must keep this in sync with `TmcRequestBoard::stimulus_step_timer_hz`.
    fn stimulus_step_timer_hz(&self) -> u32;
    fn stimulus_step_clock_now(&self) -> u32;
    /// Free segment slots on the channel's step queue right now.
    fn stimulus_queue_free(&self) -> usize;
    /// Whether an unhandled trigger stop would discard queued segments.
    fn stimulus_is_trigger_blocked(&self) -> bool;
    fn stimulus_set_direction(&mut self, forward: bool);
    /// Clear `need_reset` and set the base clock the next segment starts from.
    fn stimulus_reset_step_clock(&mut self, clock: u32) -> Result<(), StimulusError>;
    fn stimulus_queue_segment(&mut self, segment: StepMove) -> Result<(), StimulusError>;
    /// Mark the channel armed for steps and arm the timer for the queued profile.
    fn stimulus_arm(&mut self) -> Result<(), StimulusError>;
    /// Halt an in-flight segment, drain the queue, mark `need_reset`, and put
    /// `motor_armed` back to what `stimulus_arm` found. Trigger-block state
    /// is untouched. Idempotent.
    fn stimulus_stop(&mut self);
    fn set_stimulus_feedforward(&mut self, enabled: bool);
    /// Whether the control tick reports every feedforward surface restored.
    fn stimulus_feedforward_idle(&self) -> bool;
}

/// Opt-in access to a production TMC4671 SPI driver for shared backend impls.
pub trait CommissionTmcSpiAccess {
    type Spi: SpiDevice<u8>;

    fn tmc_spi(&mut self) -> &mut Tmc4671Spi<Self::Spi>;
}

/// Opt-in access to a production TMC channel state for shared backend impls.
pub trait CommissionTmcStateAccess {
    fn tmc_state(&self) -> &'static crate::state::tmc::TmcChannelState;
}

/// Opt-in enable-pin hook for shared motor-control backend impls.
pub trait CommissionEnablePinAccess {
    fn set_commission_enable_pin(&mut self, state: bool);
}

/// Opt-in board configuration access for shared backend impls.
pub trait CommissionConfigAccess {
    fn commission_tmc_config(&self) -> &'static TmcConfig;

    fn commission_feedback_config(&self) -> crate::config::FeedbackConfig;
}

/// Opt-in timing hooks for shared commissioning timing impls.
#[allow(async_fn_in_trait)]
pub trait CommissionTimingAccess {
    fn commission_elapsed_us(&self) -> u64;

    fn commission_is_shutdown_requested(&self) -> bool;

    async fn commission_delay_us(&mut self, us: u32);
}

/// Opt-in trace hook for shared commissioning trace impls.
pub trait CommissionTraceAccess {
    fn record_commission_trace_event_impl(
        &mut self,
        _event: crate::trace::events::CommissionTraceEvent,
    ) {
    }

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation_impl(
        &mut self,
        _observation: crate::trace::stream::VelocityTraceObservation,
    ) {
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record_impl(
        &mut self,
        _record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
    }
}

/// Per-instance step-stimulus hooks implemented by the shared adapter.
pub trait CommissionStepStimulusAccess {
    fn commission_step_timer_hz_impl(&self) -> u32;
    fn commission_step_clock_now_impl(&self) -> u32;
    fn commission_stimulus_queue_free_impl(&self) -> usize;
    fn commission_stimulus_trigger_blocked_impl(&self) -> bool;
    fn commission_stimulus_set_direction_impl(&mut self, forward: bool);
    fn commission_stimulus_reset_step_clock_impl(
        &mut self,
        clock: u32,
    ) -> Result<(), StimulusError>;
    fn commission_stimulus_queue_segment_impl(
        &mut self,
        segment: StepMove,
    ) -> Result<(), StimulusError>;
    fn commission_stimulus_arm_impl(&mut self) -> Result<(), StimulusError>;
    fn commission_stimulus_stop_impl(&mut self);
}

/// Board-specific hooks used by [`SharedCommissioningBackend`].
#[allow(async_fn_in_trait)]
pub trait CommissionBackendHooks {
    fn commission_tmc_config() -> &'static TmcConfig;

    /// Board-specific per-channel feedback wiring, indexed by channel.
    fn commission_feedback_configs() -> &'static [crate::config::FeedbackConfig];

    fn commission_elapsed_us() -> u64;

    fn commission_is_shutdown_requested() -> bool;

    async fn commission_delay_us(us: u32);

    /// Step timer frequency; 0 means this board ships no step stimulus.
    fn commission_step_timer_hz() -> u32 {
        0
    }

    fn commission_step_clock_now() -> u32 {
        0
    }

    fn commission_stimulus_queue_free(_channel: u8) -> usize {
        0
    }

    fn commission_stimulus_trigger_blocked(_channel: u8) -> bool {
        true
    }

    fn commission_stimulus_set_direction(_channel: u8, _forward: bool) {}

    fn commission_stimulus_reset_step_clock(
        _channel: u8,
        _clock: u32,
    ) -> Result<(), StimulusError> {
        Err(StimulusError::Unsupported)
    }

    fn commission_stimulus_queue_segment(
        _channel: u8,
        _segment: StepMove,
    ) -> Result<(), StimulusError> {
        Err(StimulusError::Unsupported)
    }

    fn commission_stimulus_arm(_channel: u8) -> Result<(), StimulusError> {
        Err(StimulusError::Unsupported)
    }

    fn commission_stimulus_stop(_channel: u8) {}

    fn record_commission_trace_event(
        _channel: u8,
        _event: crate::trace::events::CommissionTraceEvent,
    ) {
    }

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation(
        _channel: u8,
        _observation: crate::trace::stream::VelocityTraceObservation,
    ) {
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record(
        _channel: u8,
        _record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
    }
}

/// Shared production commissioning backend core used by board crates.
pub struct SharedCommissioningBackend<'a, Spi, Pin, Hooks> {
    channel: u8,
    tmc: &'a mut Tmc4671Spi<Spi>,
    enable_pin: &'a mut Pin,
    tmc_state: &'static crate::state::tmc::TmcChannelState,
    _hooks: PhantomData<fn() -> Hooks>,
}

impl<'a, Spi, Pin, Hooks> SharedCommissioningBackend<'a, Spi, Pin, Hooks> {
    /// Create a shared commissioning backend from board-owned hardware handles.
    #[must_use]
    pub fn new(
        channel: u8,
        tmc: &'a mut Tmc4671Spi<Spi>,
        enable_pin: &'a mut Pin,
        tmc_state: &'static crate::state::tmc::TmcChannelState,
    ) -> Self {
        Self {
            channel,
            tmc,
            enable_pin,
            tmc_state,
            _hooks: PhantomData,
        }
    }
}

impl<'a, Spi, Pin, Hooks> CommissionTmcSpiAccess for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Spi: SpiDevice<u8>,
{
    type Spi = Spi;

    fn tmc_spi(&mut self) -> &mut Tmc4671Spi<Self::Spi> {
        self.tmc
    }
}

impl<'a, Spi, Pin, Hooks> CommissionTmcStateAccess
    for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
{
    fn tmc_state(&self) -> &'static crate::state::tmc::TmcChannelState {
        self.tmc_state
    }
}

impl<'a, Spi, Pin, Hooks> CommissionEnablePinAccess
    for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Pin: OutputPin<Error = Infallible>,
{
    fn set_commission_enable_pin(&mut self, state: bool) {
        if state {
            let _ = self.enable_pin.set_high();
        } else {
            let _ = self.enable_pin.set_low();
        }
        self.tmc_state()
            .enable_pin_asserted
            .store(state, core::sync::atomic::Ordering::Relaxed);
    }
}

impl<'a, Spi, Pin, Hooks> CommissionConfigAccess for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Hooks: CommissionBackendHooks,
{
    fn commission_tmc_config(&self) -> &'static TmcConfig {
        Hooks::commission_tmc_config()
    }

    fn commission_feedback_config(&self) -> crate::config::FeedbackConfig {
        Hooks::commission_feedback_configs()[self.channel as usize]
    }
}

impl<'a, Spi, Pin, Hooks> CommissionPendingConfig
    for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Spi: SpiDevice<u8>,
    Hooks: CommissionBackendHooks,
{
    async fn drain_pending_config(
        &mut self,
        sink: PendingConfigSink<'_>,
    ) -> Result<(), PendingConfigItem> {
        crate::tmc_control::pending_config::drain_pending_config(
            self.tmc,
            self.tmc_state,
            *Hooks::commission_tmc_config(),
            sink,
        )
        .await
    }
}

impl<'a, Spi, Pin, Hooks> CommissionTimingAccess for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Hooks: CommissionBackendHooks,
{
    fn commission_elapsed_us(&self) -> u64 {
        Hooks::commission_elapsed_us()
    }

    fn commission_is_shutdown_requested(&self) -> bool {
        Hooks::commission_is_shutdown_requested()
    }

    async fn commission_delay_us(&mut self, us: u32) {
        Hooks::commission_delay_us(us).await;
    }
}

impl<'a, Spi, Pin, Hooks> CommissionStepStimulusAccess
    for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Hooks: CommissionBackendHooks,
{
    fn commission_step_timer_hz_impl(&self) -> u32 {
        Hooks::commission_step_timer_hz()
    }

    fn commission_step_clock_now_impl(&self) -> u32 {
        Hooks::commission_step_clock_now()
    }

    fn commission_stimulus_queue_free_impl(&self) -> usize {
        Hooks::commission_stimulus_queue_free(self.channel)
    }

    fn commission_stimulus_trigger_blocked_impl(&self) -> bool {
        Hooks::commission_stimulus_trigger_blocked(self.channel)
    }

    fn commission_stimulus_set_direction_impl(&mut self, forward: bool) {
        Hooks::commission_stimulus_set_direction(self.channel, forward);
    }

    fn commission_stimulus_reset_step_clock_impl(
        &mut self,
        clock: u32,
    ) -> Result<(), StimulusError> {
        Hooks::commission_stimulus_reset_step_clock(self.channel, clock)
    }

    fn commission_stimulus_queue_segment_impl(
        &mut self,
        segment: StepMove,
    ) -> Result<(), StimulusError> {
        Hooks::commission_stimulus_queue_segment(self.channel, segment)
    }

    fn commission_stimulus_arm_impl(&mut self) -> Result<(), StimulusError> {
        Hooks::commission_stimulus_arm(self.channel)
    }

    fn commission_stimulus_stop_impl(&mut self) {
        Hooks::commission_stimulus_stop(self.channel);
    }
}

impl<'a, Spi, Pin, Hooks> CommissionTraceAccess for SharedCommissioningBackend<'a, Spi, Pin, Hooks>
where
    Hooks: CommissionBackendHooks,
{
    fn record_commission_trace_event_impl(
        &mut self,
        event: crate::trace::events::CommissionTraceEvent,
    ) {
        Hooks::record_commission_trace_event(self.channel, event);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation_impl(
        &mut self,
        observation: crate::trace::stream::VelocityTraceObservation,
    ) {
        Hooks::record_velocity_trace_observation(self.channel, observation);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record_impl(
        &mut self,
        record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        Hooks::record_velocity_sweep_trace_record(self.channel, record);
    }
}

impl<T> CommissionRegisterIo for T
where
    T: CommissionTmcSpiAccess,
    T::Spi: embedded_hal::spi::SpiDevice<u8>,
{
    type Error = SpiError<<T::Spi as ErrorType>::Error>;

    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        self.tmc_spi().read_register(reg).await
    }

    async fn read_capture_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        self.tmc_spi().read_register_blocking(reg)
    }

    async fn read_register_pair(
        &mut self,
        first: Register,
        second: Register,
    ) -> Result<(u32, u32), Self::Error> {
        self.tmc_spi().read_register_pair_blocking(first, second)
    }

    async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), Self::Error> {
        self.tmc_spi().write_register(reg, val).await
    }

    async fn write_register_verified(
        &mut self,
        reg: Register,
        val: u32,
    ) -> Result<(), Self::Error> {
        self.tmc_spi().write_register_verified(reg, val).await
    }

    async fn read_register_verified(
        &mut self,
        reg: Register,
        expected: u32,
    ) -> Result<u32, Self::Error> {
        self.tmc_spi().read_register_verified(reg, expected).await
    }
}

impl<T> CommissionEncoderRead for T
where
    T: CommissionTmcSpiAccess,
    T::Spi: embedded_hal::spi::SpiDevice<u8>,
{
    async fn read_mechanical_encoder_position_counts(&mut self) -> u32 {
        self.tmc_spi()
            .read_register(ABN_DECODER_COUNT)
            .await
            .unwrap_or(0)
    }

    async fn try_read_mechanical_encoder_position_counts(&mut self) -> Result<u32, Self::Error> {
        self.tmc_spi().read_register(ABN_DECODER_COUNT).await
    }
}

impl<T> CommissionMotorControl for T
where
    T: CommissionTmcStateAccess + CommissionEnablePinAccess,
{
    fn store_motor_sentinel(&self, value: u32) {
        self.tmc_state()
            .motor_sentinel
            .store(value, core::sync::atomic::Ordering::Relaxed);
    }

    fn set_enable_pin(&mut self, state: bool) {
        self.set_commission_enable_pin(state);
    }

    fn interlock_tripped(&self) -> bool {
        crate::tmc_control::interlock::fault_interlock_tripped(self.tmc_state())
    }

    fn record_safe_state_report(
        &mut self,
        report: crate::tmc_safe_state::SafeStateRestoreReport,
        timestamp_ms: u32,
    ) {
        use core::sync::atomic::Ordering::Relaxed;

        let tmc = self.tmc_state();
        let last_pass_failed = report.failed_writes > 0;
        tmc.safe_state_incomplete.store(true, Relaxed);
        tmc.safe_state_last_pass_failed
            .store(last_pass_failed, Relaxed);
        tmc.safe_state_failed_writes
            .store(report.failed_writes, Relaxed);
        tmc.safe_state_first_failed_register
            .store(report.first_failed_register.unwrap_or(0xFF), Relaxed);
        tmc.safe_state_first_failed_sub_addr
            .store(report.first_failed_sub_addr.unwrap_or(0xFF), Relaxed);
        tmc.safe_state_failure_timestamp_ms
            .store(timestamp_ms, Relaxed);
        tmc.safe_state_pass_count.store(
            tmc.safe_state_pass_count.load(Relaxed).saturating_add(1),
            Relaxed,
        );
    }
}

impl<T> CommissionBoardConfig for T
where
    T: CommissionTmcStateAccess + CommissionConfigAccess,
{
    fn current_limit(&self) -> PidTorqueFluxLimit {
        self.tmc_state()
            .load_current_limit(core::sync::atomic::Ordering::Relaxed)
    }

    fn pwm_maxcnt(&self) -> u16 {
        self.commission_tmc_config().pwm_maxcnt
    }

    fn bbm_register(&self) -> u32 {
        self.commission_tmc_config().bbm_register()
    }

    fn dsadc_mdec_register(&self) -> u32 {
        self.commission_tmc_config().dsadc_mdec_register()
    }

    fn excitation_voltage(&self) -> u16 {
        self.commission_tmc_config().excitation_voltage
    }

    fn tmc_config(&self) -> &TmcConfig {
        self.commission_tmc_config()
    }

    fn adc_vm_offset(&self) -> Option<crate::state::tmc::AdcVmOffsetSnapshot> {
        self.tmc_state().adc_vm_offset_snapshot()
    }

    fn feedback_config(&self) -> crate::config::FeedbackConfig {
        self.commission_feedback_config()
    }

    fn configured_torque_filter_hz(&self) -> Option<u16> {
        self.tmc_state()
            .torque_filter_hz
            .load(core::sync::atomic::Ordering::Relaxed)
    }

    fn configured_flux_filter_hz(&self) -> Option<u16> {
        self.tmc_state()
            .flux_filter_hz
            .load(core::sync::atomic::Ordering::Relaxed)
    }

    fn configured_physical_step_width(&self) -> Option<u32> {
        self.tmc_state().physical_step_width()
    }

    fn mechanical_encoder_quadrature_counts_per_rev(&self) -> u32 {
        self.tmc_state()
            .encoder_ppr
            .load(core::sync::atomic::Ordering::Relaxed)
            .saturating_mul(4)
    }

    fn motor_pole_pairs(&self) -> u16 {
        MotorTypeNPolePairs::from(
            self.tmc_state()
                .motor_sentinel
                .load(core::sync::atomic::Ordering::Relaxed),
        )
        .n_pole_pairs()
    }

    fn adc_offsets_ready(&self) -> bool {
        self.tmc_state()
            .motor_sentinel
            .load(core::sync::atomic::Ordering::Relaxed)
            != 0
    }

    fn operational_velocity_limit(&self) -> u32 {
        self.tmc_state()
            .velocity_limit_cfg
            .load(core::sync::atomic::Ordering::Relaxed)
    }

    fn operational_voltage_limit(&self) -> u32 {
        self.tmc_state()
            .voltage_limit_cfg
            .load(core::sync::atomic::Ordering::Relaxed)
    }

    fn stall_threshold_units(&self) -> u32 {
        self.tmc_state()
            .stall_ceiling_units
            .load(core::sync::atomic::Ordering::Relaxed)
    }

    fn velocity_feedforward_gain_permille(&self) -> u32 {
        let channel = self.tmc_state();
        if channel
            .velocity_feedforward
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            u32::from(
                channel
                    .velocity_feedforward_gain_permille
                    .load(core::sync::atomic::Ordering::Relaxed),
            )
        } else {
            0
        }
    }

    fn feedforward_path_flags(&self) -> u8 {
        let channel = self.tmc_state();
        let mut flags = 0u8;
        if channel
            .velocity_feedforward
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            flags |= 0b0_0001;
        }
        if channel
            .accel_feedforward
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            flags |= 0b0_0100;
        }
        flags
    }
}

impl<T> CommissionTiming for T
where
    T: CommissionTimingAccess,
{
    fn elapsed_us(&self) -> u64 {
        self.commission_elapsed_us()
    }

    fn is_shutdown_requested(&self) -> bool {
        self.commission_is_shutdown_requested()
    }

    async fn delay_us(&mut self, us: u32) {
        self.commission_delay_us(us).await;
    }
}

impl<T> CommissionTrace for T
where
    T: CommissionTraceAccess,
{
    fn record_commission_trace_event(&mut self, event: crate::trace::events::CommissionTraceEvent) {
        self.record_commission_trace_event_impl(event);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation(
        &mut self,
        observation: crate::trace::stream::VelocityTraceObservation,
    ) {
        self.record_velocity_trace_observation_impl(observation);
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record(
        &mut self,
        record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        self.record_velocity_sweep_trace_record_impl(record);
    }
}

impl<T> CommissionStepStimulus for T
where
    T: CommissionStepStimulusAccess + CommissionTmcStateAccess,
{
    fn stimulus_step_timer_hz(&self) -> u32 {
        self.commission_step_timer_hz_impl()
    }

    fn stimulus_step_clock_now(&self) -> u32 {
        self.commission_step_clock_now_impl()
    }

    fn stimulus_queue_free(&self) -> usize {
        self.commission_stimulus_queue_free_impl()
    }

    fn stimulus_is_trigger_blocked(&self) -> bool {
        self.commission_stimulus_trigger_blocked_impl()
    }

    fn stimulus_set_direction(&mut self, forward: bool) {
        self.commission_stimulus_set_direction_impl(forward)
    }

    fn stimulus_reset_step_clock(&mut self, clock: u32) -> Result<(), StimulusError> {
        self.commission_stimulus_reset_step_clock_impl(clock)
    }

    fn stimulus_queue_segment(&mut self, segment: StepMove) -> Result<(), StimulusError> {
        self.commission_stimulus_queue_segment_impl(segment)
    }

    fn stimulus_arm(&mut self) -> Result<(), StimulusError> {
        self.commission_stimulus_arm_impl()
    }

    fn stimulus_stop(&mut self) {
        self.commission_stimulus_stop_impl()
    }

    fn set_stimulus_feedforward(&mut self, enabled: bool) {
        self.tmc_state()
            .commission_stimulus_active
            .store(enabled, Relaxed);
    }

    fn stimulus_feedforward_idle(&self) -> bool {
        self.tmc_state().feedforward_surfaces_idle.load(Relaxed)
    }
}

/// Complete commissioning backend capability bundle.
pub trait CommissioningBackend:
    CommissionRegisterIo
    + CommissionMotorControl
    + CommissionEncoderRead
    + CommissionTiming
    + CommissionBoardConfig
    + CommissionTrace
    + CommissionPendingConfig
    + CommissionStepStimulus
{
}

impl<T> CommissioningBackend for T where
    T: CommissionRegisterIo
        + CommissionMotorControl
        + CommissionEncoderRead
        + CommissionTiming
        + CommissionBoardConfig
        + CommissionTrace
        + CommissionPendingConfig
        + CommissionStepStimulus
{
}

#[cfg(test)]
mod tests {
    use super::*;

    use core::cell::RefCell;
    use core::convert::Infallible;
    #[cfg(feature = "trace")]
    use core::sync::atomic::AtomicI32;
    use core::sync::atomic::{
        AtomicBool, AtomicU8, AtomicU16, AtomicU32, AtomicU64, Ordering::Relaxed,
    };

    use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embedded_hal_async::spi::Operation;
    use foci_firmware_macros::fixture_test;
    use foci_tmc4671::registers::PID_POSITION_TARGET;
    use std::sync::Mutex;
    use std::vec::Vec;

    use crate::commissioning::types::EngineKind;
    use crate::trace::events::{
        CommissionTraceEvent, EVENT_COMMISSION_DIAGNOSTIC_WARNING, TraceEventLevel,
    };

    #[test]
    fn shutdown_requested_from_false_when_all_clear() {
        assert!(!shutdown_requested_from(false, false, false, false));
    }

    #[test]
    fn shutdown_requested_from_true_when_disable_signal_set() {
        assert!(shutdown_requested_from(true, false, false, false));
    }

    #[test]
    fn shutdown_requested_from_true_when_lease_expired() {
        assert!(shutdown_requested_from(false, true, false, false));
    }

    #[test]
    fn shutdown_requested_from_true_when_shutdown_active() {
        assert!(shutdown_requested_from(false, false, true, false));
    }

    #[test]
    fn shutdown_requested_from_true_when_fault_active() {
        assert!(shutdown_requested_from(false, false, false, true));
    }

    static TEST_LOCK: Mutex<()> = Mutex::new(());
    static TEST_TMC_STATE: crate::state::tmc::TmcChannelState =
        crate::state::tmc::TmcChannelState::new();
    static TEST_CONFIG: TmcConfig = TmcConfig {
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
    static TEST_FEEDBACK_CONFIGS: [crate::config::FeedbackConfig; 8] = {
        let mut configs = [crate::config::FeedbackConfig::abn_abn(); 8];
        configs[3] = crate::config::FeedbackConfig {
            phi_e_source: crate::config::FeedbackSource::Analog,
            position_source: crate::config::FeedbackSource::Abn,
        };
        configs
    };
    static ELAPSED_US: AtomicU64 = AtomicU64::new(0);
    static SHUTDOWN_REQUESTED: AtomicBool = AtomicBool::new(false);
    static LAST_DELAY_US: AtomicU32 = AtomicU32::new(0);
    static LAST_TRACE_CHANNEL: AtomicU8 = AtomicU8::new(u8::MAX);
    static LAST_TRACE_EVENT_ID: AtomicU16 = AtomicU16::new(0);
    #[cfg(feature = "trace")]
    static LAST_VELOCITY_DELTA: AtomicI32 = AtomicI32::new(0);
    static STIMULUS_CALLS: BlockingMutex<
        CriticalSectionRawMutex,
        RefCell<heapless::Vec<(u8, &'static str), 32>>,
    > = BlockingMutex::new(RefCell::new(heapless::Vec::new()));

    #[derive(Default)]
    struct RecordingSpi {
        writes: Vec<(u8, u32)>,
    }

    impl embedded_hal_async::spi::ErrorType for RecordingSpi {
        type Error = Infallible;
    }

    impl embedded_hal_async::spi::SpiDevice<u8> for RecordingSpi {
        async fn transaction(
            &mut self,
            operations: &mut [Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for operation in operations {
                match operation {
                    Operation::Write(buf) if buf.len() == 5 && (buf[0] & 0x80) != 0 => {
                        let addr = buf[0] & 0x7F;
                        let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                        self.writes.push((addr, value));
                    }
                    Operation::Read(buf) => {
                        buf.fill(0);
                    }
                    _ => {}
                }
            }
            Ok(())
        }
    }

    impl embedded_hal::spi::SpiDevice<u8> for RecordingSpi {
        fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
            for operation in operations {
                match operation {
                    Operation::Write(buf) if buf.len() == 5 && (buf[0] & 0x80) != 0 => {
                        let addr = buf[0] & 0x7F;
                        let value = u32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]);
                        self.writes.push((addr, value));
                    }
                    Operation::Read(buf) => {
                        buf.fill(0);
                    }
                    _ => {}
                }
            }
            Ok(())
        }
    }

    #[derive(Default)]
    struct RecordingPin {
        is_high: bool,
    }

    impl embedded_hal::digital::ErrorType for RecordingPin {
        type Error = Infallible;
    }

    impl embedded_hal::digital::OutputPin for RecordingPin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.is_high = false;
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.is_high = true;
            Ok(())
        }
    }

    struct RecordingHooks;

    impl CommissionBackendHooks for RecordingHooks {
        fn commission_tmc_config() -> &'static TmcConfig {
            &TEST_CONFIG
        }

        fn commission_feedback_configs() -> &'static [crate::config::FeedbackConfig] {
            &TEST_FEEDBACK_CONFIGS
        }

        fn commission_elapsed_us() -> u64 {
            ELAPSED_US.load(Relaxed)
        }

        fn commission_is_shutdown_requested() -> bool {
            SHUTDOWN_REQUESTED.load(Relaxed)
        }

        async fn commission_delay_us(us: u32) {
            LAST_DELAY_US.store(us, Relaxed);
        }

        fn record_commission_trace_event(channel: u8, event: CommissionTraceEvent) {
            LAST_TRACE_CHANNEL.store(channel, Relaxed);
            LAST_TRACE_EVENT_ID.store(event.event_id, Relaxed);
        }

        #[cfg(feature = "trace")]
        fn record_velocity_trace_observation(
            channel: u8,
            observation: crate::trace::stream::VelocityTraceObservation,
        ) {
            LAST_TRACE_CHANNEL.store(channel, Relaxed);
            LAST_VELOCITY_DELTA.store(observation.delta_counts, Relaxed);
        }

        fn commission_step_timer_hz() -> u32 {
            42_000_000
        }

        fn commission_step_clock_now() -> u32 {
            7
        }

        fn commission_stimulus_queue_free(channel: u8) -> usize {
            STIMULUS_CALLS.lock(|calls| {
                let _ = calls.borrow_mut().push((channel, "queue_free"));
            });
            77
        }

        fn commission_stimulus_trigger_blocked(channel: u8) -> bool {
            STIMULUS_CALLS.lock(|calls| {
                let _ = calls.borrow_mut().push((channel, "trigger_blocked"));
            });
            false
        }

        fn commission_stimulus_set_direction(channel: u8, _forward: bool) {
            STIMULUS_CALLS.lock(|calls| {
                let _ = calls.borrow_mut().push((channel, "set_direction"));
            });
        }

        fn commission_stimulus_reset_step_clock(
            channel: u8,
            _clock: u32,
        ) -> Result<(), StimulusError> {
            STIMULUS_CALLS.lock(|calls| {
                let _ = calls.borrow_mut().push((channel, "reset_step_clock"));
            });
            Ok(())
        }

        fn commission_stimulus_queue_segment(
            channel: u8,
            _segment: StepMove,
        ) -> Result<(), StimulusError> {
            STIMULUS_CALLS.lock(|calls| {
                let _ = calls.borrow_mut().push((channel, "queue_segment"));
            });
            Ok(())
        }

        fn commission_stimulus_arm(channel: u8) -> Result<(), StimulusError> {
            STIMULUS_CALLS.lock(|calls| {
                let _ = calls.borrow_mut().push((channel, "arm"));
            });
            Ok(())
        }

        fn commission_stimulus_stop(channel: u8) {
            STIMULUS_CALLS.lock(|calls| {
                let _ = calls.borrow_mut().push((channel, "stop"));
            });
        }
    }

    fn reset_hooks() {
        ELAPSED_US.store(0, Relaxed);
        SHUTDOWN_REQUESTED.store(false, Relaxed);
        LAST_DELAY_US.store(0, Relaxed);
        LAST_TRACE_CHANNEL.store(u8::MAX, Relaxed);
        LAST_TRACE_EVENT_ID.store(0, Relaxed);
        #[cfg(feature = "trace")]
        LAST_VELOCITY_DELTA.store(0, Relaxed);
        TEST_TMC_STATE.motor_sentinel.store(0, Relaxed);
        TEST_TMC_STATE.safe_state_incomplete.store(false, Relaxed);
        TEST_TMC_STATE
            .safe_state_last_pass_failed
            .store(false, Relaxed);
        TEST_TMC_STATE.safe_state_failed_writes.store(0, Relaxed);
        TEST_TMC_STATE
            .safe_state_first_failed_register
            .store(0xFF, Relaxed);
        TEST_TMC_STATE
            .safe_state_first_failed_sub_addr
            .store(0xFF, Relaxed);
        TEST_TMC_STATE
            .safe_state_failure_timestamp_ms
            .store(0, Relaxed);
        TEST_TMC_STATE.safe_state_pass_count.store(0, Relaxed);
    }

    #[fixture_test]
    fn set_commission_enable_pin_updates_enable_pin_asserted() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();
        TEST_TMC_STATE.enable_pin_asserted.store(false, Relaxed);

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                0,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );

        backend.set_commission_enable_pin(true);
        assert!(TEST_TMC_STATE.enable_pin_asserted.load(Relaxed));

        backend.set_commission_enable_pin(false);
        assert!(!TEST_TMC_STATE.enable_pin_asserted.load(Relaxed));
    }

    #[fixture_test]
    fn shared_backend_persists_safe_state_report_into_tmc_state() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                0,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );

        let report = crate::tmc_safe_state::SafeStateRestoreReport {
            failed_writes: 2,
            first_failed_register: Some(0x1A),
            first_failed_sub_addr: Some(6),
        };
        backend.record_safe_state_report(report, 12_345);

        assert!(TEST_TMC_STATE.safe_state_incomplete.load(Relaxed));
        assert!(TEST_TMC_STATE.safe_state_last_pass_failed.load(Relaxed));
        assert_eq!(TEST_TMC_STATE.safe_state_failed_writes.load(Relaxed), 2);
        assert_eq!(
            TEST_TMC_STATE
                .safe_state_first_failed_register
                .load(Relaxed),
            0x1A
        );
        assert_eq!(
            TEST_TMC_STATE
                .safe_state_first_failed_sub_addr
                .load(Relaxed),
            6
        );
        assert_eq!(
            TEST_TMC_STATE.safe_state_failure_timestamp_ms.load(Relaxed),
            12_345
        );
        assert_eq!(TEST_TMC_STATE.safe_state_pass_count.load(Relaxed), 1);

        backend.record_safe_state_report(report, 12_346);
        assert_eq!(
            TEST_TMC_STATE.safe_state_pass_count.load(Relaxed),
            2,
            "every completed pass counts, not just failed ones"
        );
    }

    #[fixture_test]
    fn shared_backend_clean_report_leaves_incomplete_latch_set_but_clears_last_pass_failed() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();
        TEST_TMC_STATE.safe_state_incomplete.store(false, Relaxed);
        TEST_TMC_STATE
            .safe_state_last_pass_failed
            .store(true, Relaxed);
        TEST_TMC_STATE.safe_state_pass_count.store(3, Relaxed);

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                0,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );

        let report = crate::tmc_safe_state::SafeStateRestoreReport {
            failed_writes: 0,
            first_failed_register: None,
            first_failed_sub_addr: None,
        };
        backend.record_safe_state_report(report, 0);

        assert!(
            TEST_TMC_STATE.safe_state_incomplete.load(Relaxed),
            "only arm_motor_electrical's sentinel success may clear the compound latch"
        );
        assert!(
            !TEST_TMC_STATE.safe_state_last_pass_failed.load(Relaxed),
            "a clean pass should clear the last-pass-failed flag"
        );
        assert_eq!(
            TEST_TMC_STATE.safe_state_pass_count.load(Relaxed),
            4,
            "a clean pass still counts as a completed restore attempt"
        );
    }

    #[fixture_test]
    fn shared_backend_decodes_motor_pole_pairs_from_sentinel() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let sentinel: u32 = foci_tmc4671::registers::MotorTypeNPolePairs::new()
            .with_n_pole_pairs(50)
            .into();
        TEST_TMC_STATE.motor_sentinel.store(sentinel, Relaxed);

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let backend = SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
            1,
            &mut tmc,
            &mut enable_pin,
            &TEST_TMC_STATE,
        );

        assert_eq!(backend.motor_pole_pairs(), 50);
    }

    #[fixture_test]
    fn shared_backend_reads_feedback_config_for_its_own_channel() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let backend = SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
            3,
            &mut tmc,
            &mut enable_pin,
            &TEST_TMC_STATE,
        );

        assert_eq!(
            backend.feedback_config(),
            crate::config::FeedbackConfig {
                phi_e_source: crate::config::FeedbackSource::Analog,
                position_source: crate::config::FeedbackSource::Abn,
            }
        );
    }

    #[fixture_test]
    fn shared_backend_reports_zero_pole_pairs_for_uncalibrated_sentinel() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let backend = SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
            1,
            &mut tmc,
            &mut enable_pin,
            &TEST_TMC_STATE,
        );

        assert_eq!(backend.motor_pole_pairs(), 0);
    }

    #[fixture_test]
    fn shared_backend_drains_pending_config_through_the_real_drain() {
        use foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS;

        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        pollster::block_on(async {
            TEST_TMC_STATE.run_current_ma_cfg.store(1_000, Relaxed);
            TEST_TMC_STATE
                .config_apply_pending
                .store(crate::state::tmc::PENDING_CURRENT, Relaxed);

            let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
            let mut enable_pin = RecordingPin::default();
            {
                let mut backend = SharedCommissioningBackend::<
                    RecordingSpi,
                    RecordingPin,
                    RecordingHooks,
                >::new(
                    0, &mut tmc, &mut enable_pin, &TEST_TMC_STATE
                );
                let mut stored_inner_gains = None;
                backend
                    .drain_pending_config(PendingConfigSink {
                        stored_inner_gains: &mut stored_inner_gains,
                    })
                    .await
                    .expect("a recording SPI accepts every drain write");
            }

            let expected = crate::tmc_control::current_limit::current_limit_from_run_ma(
                1_000,
                TEST_CONFIG.current_scale_ma_per_lsb,
            )
            .expect("1000 mA at the board scale is representable")
            .register_value();
            let spi = tmc.into_inner();
            assert_eq!(
                spi.writes,
                std::vec![(PID_TORQUE_FLUX_LIMITS.addr(), expected)],
                "the delegation must reach the real drain, not report success without writing"
            );
            assert_eq!(TEST_TMC_STATE.config_apply_pending.load(Relaxed), 0);

            TEST_TMC_STATE.run_current_ma_cfg.store(0, Relaxed);
        });
    }

    #[test]
    fn shared_backend_satisfies_commissioning_backend_bundle() {
        fn assert_backend<T: CommissioningBackend>() {}

        assert_backend::<
            SharedCommissioningBackend<'static, RecordingSpi, RecordingPin, RecordingHooks>,
        >();
    }

    #[fixture_test]
    fn shared_backend_delegates_access_traits() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        pollster::block_on(async {
            ELAPSED_US.store(123_456, Relaxed);
            SHUTDOWN_REQUESTED.store(true, Relaxed);

            let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
            let mut enable_pin = RecordingPin::default();
            let trace_event = CommissionTraceEvent {
                level: TraceEventLevel::Warn,
                event_id: EVENT_COMMISSION_DIAGNOSTIC_WARNING,
                engine_kind: EngineKind::Commission,
                payload: [7, 8],
                payload_count: 2,
            };

            {
                let mut backend = SharedCommissioningBackend::<
                    RecordingSpi,
                    RecordingPin,
                    RecordingHooks,
                >::new(
                    7, &mut tmc, &mut enable_pin, &TEST_TMC_STATE
                );

                assert!(core::ptr::eq(backend.tmc_state(), &TEST_TMC_STATE));
                assert_eq!(
                    backend.commission_tmc_config().pwm_maxcnt,
                    TEST_CONFIG.pwm_maxcnt
                );
                assert_eq!(backend.commission_elapsed_us(), 123_456);
                assert!(backend.commission_is_shutdown_requested());

                backend.commission_delay_us(250).await;
                backend.record_commission_trace_event_impl(trace_event);
                backend.set_commission_enable_pin(true);
                backend.set_commission_enable_pin(false);
                backend
                    .tmc_spi()
                    .write_register(PID_POSITION_TARGET, 0x0102_0304)
                    .await
                    .expect("recording SPI write succeeds");
            }

            assert!(!enable_pin.is_high);
            assert_eq!(LAST_DELAY_US.load(Relaxed), 250);
            assert_eq!(LAST_TRACE_CHANNEL.load(Relaxed), 7);
            assert_eq!(
                LAST_TRACE_EVENT_ID.load(Relaxed),
                EVENT_COMMISSION_DIAGNOSTIC_WARNING
            );

            let spi = tmc.into_inner();
            assert_eq!(
                spi.writes,
                std::vec![(PID_POSITION_TARGET.addr(), 0x0102_0304)]
            );
        });
    }

    #[cfg(feature = "trace")]
    #[test]
    fn shared_backend_forwards_velocity_trace_observations() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();
        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                3,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );

        backend.record_velocity_trace_observation_impl(
            crate::trace::stream::VelocityTraceObservation {
                delta_counts: -7,
                ..Default::default()
            },
        );

        assert_eq!(LAST_TRACE_CHANNEL.load(Relaxed), 3);
        assert_eq!(LAST_VELOCITY_DELTA.load(Relaxed), -7);
    }

    #[fixture_test]
    fn stimulus_facet_forwards_each_operation_with_its_channel() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                1,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );
        STIMULUS_CALLS.lock(|calls| calls.borrow_mut().clear());

        backend.stimulus_set_direction(false);
        backend.stimulus_reset_step_clock(1_234).unwrap();
        backend
            .stimulus_queue_segment(StepMove::new(3_173, 10, 0))
            .unwrap();
        backend.stimulus_arm().unwrap();
        backend.stimulus_stop();
        let _ = backend.stimulus_is_trigger_blocked();
        assert_eq!(backend.stimulus_queue_free(), 77);

        STIMULUS_CALLS.lock(|calls| {
            assert_eq!(
                calls.borrow().as_slice(),
                &[
                    (1, "set_direction"),
                    (1, "reset_step_clock"),
                    (1, "queue_segment"),
                    (1, "arm"),
                    (1, "stop"),
                    (1, "trigger_blocked"),
                    (1, "queue_free"),
                ]
            );
        });
        assert_eq!(backend.stimulus_step_timer_hz(), 42_000_000);
    }

    #[fixture_test]
    fn stimulus_feedforward_opt_in_writes_the_atomic_and_reads_the_mirror() {
        let _guard = TEST_LOCK
            .lock()
            .expect("commissioning backend test lock is available");
        reset_hooks();

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend =
            SharedCommissioningBackend::<RecordingSpi, RecordingPin, RecordingHooks>::new(
                0,
                &mut tmc,
                &mut enable_pin,
                &TEST_TMC_STATE,
            );
        TEST_TMC_STATE
            .commission_stimulus_active
            .store(false, Relaxed);
        TEST_TMC_STATE
            .feedforward_surfaces_idle
            .store(false, Relaxed);

        backend.set_stimulus_feedforward(true);
        assert!(TEST_TMC_STATE.commission_stimulus_active.load(Relaxed));
        assert!(!backend.stimulus_feedforward_idle());

        backend.set_stimulus_feedforward(false);
        TEST_TMC_STATE
            .feedforward_surfaces_idle
            .store(true, Relaxed);
        assert!(!TEST_TMC_STATE.commission_stimulus_active.load(Relaxed));
        assert!(backend.stimulus_feedforward_idle());
    }

    #[test]
    fn default_hooks_report_no_stimulus() {
        struct BareHooks;
        impl CommissionBackendHooks for BareHooks {
            fn commission_tmc_config() -> &'static TmcConfig {
                &TEST_CONFIG
            }

            fn commission_feedback_configs() -> &'static [crate::config::FeedbackConfig] {
                &TEST_FEEDBACK_CONFIGS
            }

            fn commission_elapsed_us() -> u64 {
                0
            }

            fn commission_is_shutdown_requested() -> bool {
                false
            }

            async fn commission_delay_us(_us: u32) {}
        }

        let mut tmc = Tmc4671Spi::new(RecordingSpi::default());
        let mut enable_pin = RecordingPin::default();
        let mut backend = SharedCommissioningBackend::<RecordingSpi, RecordingPin, BareHooks>::new(
            0,
            &mut tmc,
            &mut enable_pin,
            &TEST_TMC_STATE,
        );
        assert_eq!(backend.stimulus_step_timer_hz(), 0);
        assert_eq!(backend.stimulus_queue_free(), 0);
        assert!(backend.stimulus_is_trigger_blocked());
        assert_eq!(
            backend.stimulus_reset_step_clock(0),
            Err(StimulusError::Unsupported)
        );
        assert_eq!(
            backend.stimulus_queue_segment(StepMove::new(1, 1, 0)),
            Err(StimulusError::Unsupported)
        );
        assert_eq!(backend.stimulus_arm(), Err(StimulusError::Unsupported));
    }
}
