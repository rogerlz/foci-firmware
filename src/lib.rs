//! Board-agnostic FOCI firmware library.

#![no_std]

#[cfg(test)]
extern crate std;

extern crate self as foci_firmware;

pub mod absolute_encoder;
pub mod adc_vm;
pub mod analog_in;
pub mod ankyra;
pub mod autotune;
pub mod biquad;
pub mod calibration;
pub mod commission_dispatch;
pub mod commissioning;
pub mod config;
pub mod context;
pub mod cpu_load;
pub mod diagnostic;
pub mod dump;
pub mod electrical_capture;
pub mod helpers;
pub mod mcu_temp;
pub mod motion_scale;
pub mod pid;
pub mod resistance_identification;
pub mod selftest;
pub mod shutdown;
pub mod stack_watermark;
pub mod state;
pub mod step_pulse;
pub mod stepper_runtime;
pub mod tmc;
pub mod tmc_control;
pub mod tmc_diagnostics;
pub mod tmc_register_values;
pub mod tmc_safe_state;
pub mod trace;
pub mod transport;
pub mod usb_pump;

/// Legacy `foci_firmware::commission::*` re-exports of [`commissioning`].
pub mod commission {
    pub use crate::commissioning::backend::CommissioningBackend;
    pub use crate::commissioning::types::{
        AdcCalResult, CommissionResult, InnerResult, OuterResult, StepResult,
    };
    pub use crate::commissioning::{
        CommissioningEngine, OuterAutotuneInputs, OuterGainRollbackSnapshot,
    };
}
