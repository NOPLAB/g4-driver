//! Hardware adapters for the bldc crate
//!
//! This module provides STM32G4-specific implementations of the bldc traits.

mod hall_adapter;
#[allow(dead_code)]
mod pwm_adapter;

pub use hall_adapter::HallSensorAdapter;
pub use hall_adapter::HallStateReaderAdapter;
#[allow(unused_imports)]
pub use pwm_adapter::PwmAdapter;
