//! Hardware adapters for the bldc crate
//!
//! This module provides STM32G4-specific implementations of the bldc traits.
//! These adapters are prepared for future migration from firmware-specific
//! implementations to the portable bldc crate.

#[allow(dead_code)]
mod hall_adapter;
#[allow(dead_code)]
mod pwm_adapter;

#[allow(unused_imports)]
pub use hall_adapter::HallSensorAdapter;
#[allow(unused_imports)]
pub use pwm_adapter::PwmAdapter;
