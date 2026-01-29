//! Compensation modules for motor control
//!
//! This module provides various compensation algorithms:
//! - Dead time compensation: Corrects PWM dead time voltage distortion
//! - Flux weakening: Extends motor speed range by applying negative d-axis voltage

pub mod dead_time;
pub mod flux_weakening;

pub use dead_time::DeadTimeCompensation;
pub use flux_weakening::FluxWeakeningController;
