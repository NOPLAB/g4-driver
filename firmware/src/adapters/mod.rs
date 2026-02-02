//! Hardware adapters for the bldc crate
//!
//! This module provides STM32G4-specific implementations of the bldc traits.

mod hall_adapter;

pub use hall_adapter::HallSensorAdapter;
