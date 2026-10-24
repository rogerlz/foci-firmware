//! Resumable resistance-identification state.

mod measurement;
mod poll;

use super::profile::ResistanceProfile;
use super::runner::{ResistanceBackend, ResistanceRunMode};
use super::types::{
    ElectricalAxis, ResistanceAxisResult, ResistanceCurrentEvidence, ResistanceError,
    ResistancePoint, ResistanceRunResult, ResistanceWarningFlags, SignedAnchorEvidence,
};
use crate::autotune::Permille;
use crate::commissioning::capture_timing::CaptureTiming;

/// Sampling opportunities a window may use beyond its sample count.
const SPARE_OPPORTUNITIES_PER_WINDOW: u8 = 1;

const fn max_opportunities_per_window(profile: &ResistanceProfile) -> u8 {
    profile
        .samples_per_window
        .saturating_add(SPARE_OPPORTUNITIES_PER_WINDOW)
}
pub(super) const PHASE_DEADLINE_US: u64 = 20_000_000;
const PREDICTED_CURRENT_STOP_PERMILLE: u64 = 950;

/// Result of one bounded resistance-runner poll.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ResistancePoll {
    /// More work is ready without waiting.
    ReadyNow,
    /// Resume at this absolute monotonic deadline.
    At(u64),
    /// The run and required restoration have completed.
    Complete(Result<ResistanceRunResult, ResistanceError>),
}

/// Compact timing summary retained across all completed windows.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, defmt::Format)]
pub struct ResistanceTimingEvidence {
    /// Windows whose timing envelopes completed successfully.
    pub completed_windows: u8,
    /// Requested sample period shared by all windows.
    pub requested_period_us: u16,
    /// Total accepted logical samples.
    pub valid_samples: u16,
    /// Total skipped sample slots.
    pub missed_samples: u16,
    /// Largest consecutive skipped-slot count.
    pub max_consecutive_misses: u8,
    /// Largest sample-start lateness.
    pub max_lateness_us: u32,
    /// Largest accepted-observation interval.
    pub max_interval_us: u32,
    /// Largest complete sample-poll wall time.
    pub max_poll_wall_us: u32,
    /// Largest summed SPI-await wall time in a sample poll.
    pub max_spi_wall_us: u32,
    /// Whether any retained source or aggregate counter saturated.
    pub overflowed: bool,
}

impl ResistanceTimingEvidence {
    pub(crate) fn merge(
        &mut self,
        evidence: crate::commissioning::capture_timing::CaptureTimingEvidence,
        overflowed: bool,
    ) {
        self.requested_period_us = evidence.requested_period_us;
        self.valid_samples = match self.valid_samples.checked_add(evidence.valid_samples) {
            Some(value) => value,
            None => {
                self.overflowed = true;
                u16::MAX
            }
        };
        self.missed_samples = match self.missed_samples.checked_add(evidence.missed_samples) {
            Some(value) => value,
            None => {
                self.overflowed = true;
                u16::MAX
            }
        };
        self.max_consecutive_misses = self
            .max_consecutive_misses
            .max(evidence.max_consecutive_misses);
        self.max_lateness_us = self.max_lateness_us.max(evidence.max_lateness_us);
        self.max_interval_us = self.max_interval_us.max(evidence.max_interval_us);
        self.max_poll_wall_us = self.max_poll_wall_us.max(evidence.max_poll_wall_us);
        self.max_spi_wall_us = self.max_spi_wall_us.max(evidence.max_spi_wall_us);
        self.overflowed |= overflowed;
    }

    pub(crate) const fn capture_evidence(
        self,
    ) -> crate::commissioning::capture_timing::CaptureTimingEvidence {
        crate::commissioning::capture_timing::CaptureTimingEvidence {
            requested_period_us: self.requested_period_us,
            valid_samples: self.valid_samples,
            missed_samples: self.missed_samples,
            max_consecutive_misses: self.max_consecutive_misses,
            max_lateness_us: self.max_lateness_us,
            max_interval_us: self.max_interval_us,
            max_poll_wall_us: self.max_poll_wall_us,
            max_spi_wall_us: self.max_spi_wall_us,
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
enum WindowPurpose {
    InitialZero,
    PointEnergized(u8),
    PointTrailing(u8),
    RepeatedAnchor,
    RepeatedZero,
    NegativeAnchor,
    NegativeZero,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct WindowPlan {
    purpose: WindowPurpose,
    ud: i16,
    settle_ms: u16,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct WindowCapture {
    plan: WindowPlan,
    operation: u8,
    encoder_before: u32,
    current_sum: i32,
    status_or: u32,
    acquisition_samples: u8,
    acquisition_opportunities: u8,
    settle_deadline_us: u64,
    next_sample_us: u64,
    timing: CaptureTiming,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct SafeAnchor {
    ud: i16,
    positive_current_count: i32,
    preferred_distance: u16,
}

#[derive(Debug, Clone, Copy, defmt::Format)]
struct AxisProgress {
    points: [ResistancePoint; 8],
    point_index: u8,
    leading_zero: i32,
    energized_current: i32,
    first_anchor_current: Option<i32>,
    safe_anchor: Option<SafeAnchor>,
    last_safe_point: Option<(u16, u32)>,
    repeated_positive: i32,
    negative_current: i32,
}

impl AxisProgress {
    const fn new() -> Self {
        Self {
            points: [ResistancePoint {
                requested_ud: 0,
                voltage_delta_count: 0,
                current_delta_count: 0,
                selected: false,
            }; 8],
            point_index: 0,
            leading_zero: 0,
            energized_current: 0,
            first_anchor_current: None,
            safe_anchor: None,
            last_safe_point: None,
            repeated_positive: 0,
            negative_current: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
enum ResistanceState {
    ReadSavedLimit,
    ReadMotorType,
    ReadBusVoltage(u8),
    Setup(u8),
    SelectAxis(u8),
    Window(WindowCapture),
    ReadEncoderAfter(WindowCapture),
    EvaluateAxis,
    EvaluateRun,
    Restore(u8),
    Complete,
}

pub(super) const RESTORE_WRITE_COUNT: u8 = 4;

/// Resumable shared resistance-identification runner.
#[derive(Debug, defmt::Format)]
pub struct ResistanceRunner {
    mode: ResistanceRunMode,
    profile: ResistanceProfile,
    state: ResistanceState,
    deadline_us: u64,
    saved_voltage_limit: u32,
    saved_adc_raw_addr: u32,
    adc_vm_raw: Option<u16>,
    energized: bool,
    primary_error: Option<ResistanceError>,
    baseline_status_flags: u32,
    status_flags_or: u32,
    warning_flags: ResistanceWarningFlags,
    axis_index: u8,
    /// Excite in the rotor frame (`phi_e_abn`) instead of at a fixed
    /// angle. A free PMSM rotor turns toward a fixed excitation, and its
    /// back-EMF corrupts the measurement; along the d axis it holds still.
    rotor_frame: bool,
    axis: AxisProgress,
    axis_results: [Option<ResistanceAxisResult>; 2],
    result: Option<ResistanceRunResult>,
    timing: ResistanceTimingEvidence,
    current: ResistanceCurrentEvidence,
}

impl ResistanceRunner {
    /// Create a runner and prove its configured schedule fits the phase deadline.
    pub fn new(
        mode: ResistanceRunMode,
        profile: ResistanceProfile,
        start_us: u64,
    ) -> Result<Self, ResistanceError> {
        let deadline_us = start_us
            .checked_add(PHASE_DEADLINE_US)
            .ok_or(ResistanceError::PhaseDeadlineExceeded)?;
        if !exposure_is_supported(&profile) {
            return Err(ResistanceError::UnsupportedProfile);
        }
        if nominal_duration_us(&profile).is_none_or(|duration| duration > PHASE_DEADLINE_US) {
            return Err(ResistanceError::PhaseDeadlineExceeded);
        }
        Ok(Self {
            mode,
            profile,
            state: ResistanceState::ReadSavedLimit,
            deadline_us,
            saved_voltage_limit: 0,
            saved_adc_raw_addr: 0,
            adc_vm_raw: None,
            energized: false,
            primary_error: None,
            baseline_status_flags: 0,
            status_flags_or: 0,
            warning_flags: ResistanceWarningFlags::NONE,
            axis_index: 0,
            rotor_frame: false,
            axis: AxisProgress::new(),
            axis_results: [None, None],
            result: None,
            timing: ResistanceTimingEvidence::default(),
            current: ResistanceCurrentEvidence::new(profile.max_expected_current_count),
        })
    }

    /// Return timing evidence accumulated by completed windows.
    #[must_use]
    pub const fn timing_evidence(&self) -> ResistanceTimingEvidence {
        self.timing
    }

    /// Return retained evidence plus a non-mutating snapshot of an active window.
    #[must_use]
    pub fn timing_evidence_with_active(&self) -> ResistanceTimingEvidence {
        let mut timing = self.timing;
        if let ResistanceState::Window(capture) = self.state {
            let (evidence, overflowed) = capture.timing.snapshot();
            timing.merge(evidence, overflowed);
        }
        timing
    }

    /// Return the raw `STATUS_FLAGS` value read once before excitation.
    #[cfg(test)]
    #[must_use]
    pub const fn baseline_status_flags(&self) -> u32 {
        self.baseline_status_flags
    }

    /// Return the raw OR of all STATUS observations made after the
    /// pre-excitation clear, excluding the pre-clear baseline.
    #[must_use]
    pub const fn status_flags_or(&self) -> u32 {
        self.status_flags_or
    }

    /// Return the primary failure retained while restoration is in progress.
    #[must_use]
    pub const fn primary_error(&self) -> Option<ResistanceError> {
        self.primary_error
    }

    /// Return warning evidence accumulated so far.
    #[must_use]
    pub const fn warning_flags(&self) -> ResistanceWarningFlags {
        self.warning_flags
    }

    /// Return current evidence accumulated so far.
    #[must_use]
    pub const fn current_evidence(&self) -> ResistanceCurrentEvidence {
        self.current
    }

    /// Advance by at most one register operation or one two-read logical sample.
    pub async fn poll<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        now_us: u64,
    ) -> ResistancePoll {
        if now_us >= self.deadline_us
            && !matches!(
                self.state,
                ResistanceState::Restore(_) | ResistanceState::Complete
            )
        {
            return self.fail(backend, ResistanceError::PhaseDeadlineExceeded);
        }
        match self.state {
            ResistanceState::ReadSavedLimit => self.read_saved_limit(backend).await,
            ResistanceState::ReadMotorType => self.read_motor_type(backend).await,
            ResistanceState::ReadBusVoltage(operation) => {
                self.read_bus_voltage(backend, operation).await
            }
            ResistanceState::Setup(operation) => self.poll_setup(backend, operation).await,
            ResistanceState::SelectAxis(operation) => self.select_axis(backend, operation).await,
            ResistanceState::Window(capture) => self.poll_window(backend, capture, now_us).await,
            ResistanceState::ReadEncoderAfter(capture) => {
                self.read_encoder_after(backend, capture).await
            }
            ResistanceState::EvaluateAxis => self.evaluate_axis(backend),
            ResistanceState::EvaluateRun => self.evaluate_run(backend),
            ResistanceState::Restore(operation) => self.poll_restore(backend, operation).await,
            ResistanceState::Complete => match self.primary_error {
                Some(error) => ResistancePoll::Complete(Err(error)),
                None => {
                    ResistancePoll::Complete(Ok(self.result.expect("successful evaluated result")))
                }
            },
        }
    }
}

pub(super) fn exposure_is_supported(profile: &ResistanceProfile) -> bool {
    let largest =
        u64::from(profile.max_abs_ud().unsigned_abs()).pow(2) * u64::from(profile.settle_ms);
    profile.exposure_per_axis() <= profile.max_total_exposure
        && largest <= profile.max_window_exposure
}

pub(super) fn nominal_duration_us(profile: &ResistanceProfile) -> Option<u64> {
    let point_pairs = u64::try_from(profile.ud_points.len()).ok()?;
    let windows_per_axis = 1 + point_pairs * 2 + 4;
    let settle_ms = u64::from(profile.zero_settle_ms)
        .checked_add(point_pairs.checked_mul(
            u64::from(profile.settle_ms).checked_add(u64::from(profile.zero_settle_ms))?,
        )?)?
        .checked_add(2 * (u64::from(profile.settle_ms) + u64::from(profile.zero_settle_ms)))?;
    let sample_ms = windows_per_axis
        .checked_mul(u64::from(max_opportunities_per_window(profile)))?
        .checked_mul(u64::from(profile.sample_interval_ms))?;
    settle_ms
        .checked_add(sample_ms)?
        .checked_mul(2)?
        .checked_mul(1_000)
}

async fn write<B: ResistanceBackend>(
    backend: &mut B,
    addr: u8,
    value: u32,
) -> Result<(), ResistanceError> {
    backend
        .write_register(addr, value)
        .await
        .map_err(|_| ResistanceError::BackendIo)
}

fn pack_ud(ud: i16) -> u32 {
    u32::from(ud as u16)
}

fn phase_current_for_axis(axis: ElectricalAxis, packed: u32) -> i16 {
    if axis.index == 0 {
        crate::tmc::decode_signed_lower_half(packed)
    } else {
        crate::tmc::decode_signed_upper_half(packed)
    }
}

pub(super) fn acquisition_opportunities_through(
    due_us: u64,
    sample_deadline_us: u64,
    settle_deadline_us: u64,
    period_us: u16,
) -> Result<u32, ResistanceError> {
    if sample_deadline_us < settle_deadline_us {
        return Ok(0);
    }
    let period_us = u64::from(period_us);
    if period_us == 0 || sample_deadline_us < due_us {
        return Err(ResistanceError::TimingRejected);
    }
    let selected_index = (sample_deadline_us - due_us) / period_us;
    let first_acquisition_index = if settle_deadline_us <= due_us {
        0
    } else {
        (settle_deadline_us - due_us).div_ceil(period_us)
    };
    if first_acquisition_index > selected_index {
        return Ok(0);
    }
    u32::try_from(selected_index - first_acquisition_index + 1)
        .map_err(|_| ResistanceError::TimingRejected)
}

fn wrapped_encoder_delta(before: u32, after: u32, counts_per_rev: u32) -> i32 {
    let raw = (after as i32).wrapping_sub(before as i32);
    let counts_per_rev = counts_per_rev as i32;
    if counts_per_rev <= 0 {
        return raw;
    }
    let half = counts_per_rev / 2;
    if raw > half {
        raw - counts_per_rev
    } else if raw < -half {
        raw + counts_per_rev
    } else {
        raw
    }
}

fn next_point_predicted_unsafe(
    profile: &ResistanceProfile,
    last: Option<(u16, u32)>,
    next_ud: i16,
) -> bool {
    let Some((last_ud, last_current)) = last else {
        return false;
    };
    if last_ud == 0 {
        return false;
    }
    let predicted =
        u64::from(last_current) * u64::from(next_ud.unsigned_abs()) / u64::from(last_ud);
    predicted
        >= u64::from(profile.max_expected_current_count) * PREDICTED_CURRENT_STOP_PERMILLE / 1_000
}

fn update_safe_anchor(
    profile: &ResistanceProfile,
    candidate: &mut Option<SafeAnchor>,
    ud: i16,
    current: i32,
) {
    if current.unsigned_abs() < u32::from(profile.linear_current_threshold_count) {
        return;
    }
    let distance = ud
        .unsigned_abs()
        .abs_diff(profile.signed_anchor_ud.unsigned_abs());
    if candidate.is_none_or(|existing| distance < existing.preferred_distance) {
        *candidate = Some(SafeAnchor {
            ud,
            positive_current_count: current,
            preferred_distance: distance,
        });
    }
}

fn build_signed_anchor(
    positive: i32,
    negative: i32,
    repeated: i32,
    ud: i16,
) -> SignedAnchorEvidence {
    let current_span = i64::from(positive) - i64::from(negative);
    let slope = if current_span == 0 {
        0
    } else {
        (i64::from(ud) * 2_000 / current_span) as i32
    };
    let pos_abs = positive.unsigned_abs();
    let neg_abs = negative.unsigned_abs();
    let asymmetry = Permille::checked_ratio(
        u64::from(pos_abs.abs_diff(neg_abs)),
        u64::from(pos_abs.max(neg_abs).max(1)),
    )
    .expect("denominator is at least 1 via .max(1)");
    let drift = Permille::checked_ratio(
        u64::from(positive.abs_diff(repeated)),
        u64::from(pos_abs.max(1)),
    )
    .expect("denominator is at least 1 via .max(1)");
    SignedAnchorEvidence {
        positive_current_count: positive,
        negative_current_count: negative,
        signed_count_slope_milli: slope,
        asymmetry_permille: asymmetry,
        repeated_positive_current_count: repeated,
        drift_permille: drift,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_anchor_drift_saturates_instead_of_capping_at_100_percent() {
        let anchor = build_signed_anchor(10_000, -1_000, -100, 960);

        assert_eq!(anchor.drift_permille.raw(), 1010);
    }

    #[test]
    fn signed_anchor_asymmetry_is_bounded_by_1000_and_unaffected_by_the_migration() {
        let anchor = build_signed_anchor(10_000, -100, 10_000, 960);

        assert_eq!(anchor.asymmetry_permille.raw(), 990);
    }

    #[test]
    fn signed_anchor_reports_zero_for_perfectly_repeated_symmetric_current() {
        let anchor = build_signed_anchor(1_000, -1_000, 1_000, 960);

        assert_eq!(anchor.asymmetry_permille.raw(), 0);
        assert_eq!(anchor.drift_permille.raw(), 0);
    }
}
