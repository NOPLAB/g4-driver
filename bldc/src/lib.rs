//! Hardware-agnostic BLDC motor control library
//!
//! This crate provides reusable motor control algorithms that can be used
//! across different microcontrollers and platforms.
//!
//! # Features
//!
//! - `hall` (default): Hall sensor support
//! - `calibration`: Motor calibration support
//! - `encoder`: Encoder support (planned)
//! - `sensorless`: Sensorless control support (planned)
//! - `std`: Standard library support for PC simulation/testing
//!
//! # Architecture
//!
//! The library is organized into several modules:
//!
//! - [`traits`]: Hardware abstraction traits (PositionSensor, SpeedSensor, PwmOutput, etc.)
//! - [`control`]: Control algorithms (PI controller, FOC, six-step)
//! - [`modulation`]: PWM modulation strategies (SVPWM)
//! - [`sensors`]: Sensor processing algorithms (Hall sensor)
//! - [`transforms`]: Coordinate transformations (Park, Clarke)
//! - [`position`]: Shaft position tracking
//! - [`compensation`]: Compensation algorithms (dead time, flux weakening)
//! - [`calibration`]: Motor calibration (requires `calibration` feature)
//!
//! # Example
//!
//! ```rust,ignore
//! use bldc::control::PiController;
//! use bldc::modulation::svpwm;
//! use bldc::transforms::{inverse_park, limit_voltage};
//!
//! // Create a PI controller for speed control
//! let mut speed_pi = PiController::new_symmetric(0.5, 0.05, 24.0);
//!
//! // In your control loop:
//! let vq = speed_pi.update(target_speed, measured_speed, dt);
//! let (vd, vq) = limit_voltage(0.0, vq, max_voltage);
//! let (v_alpha, v_beta) = inverse_park(vd, vq, electrical_angle);
//! let (du, dv, dw) = svpwm::calculate(v_alpha, v_beta, v_dc, max_duty);
//! ```

#![no_std]
#![deny(unsafe_code)]

#[cfg(feature = "calibration")]
pub mod calibration;
pub mod compensation;
pub mod control;
pub mod modulation;
pub mod position;
#[cfg(feature = "hall")]
pub mod sensors;
pub mod traits;
pub mod transforms;

// Re-export commonly used types
pub use compensation::{DeadTimeCompensation, FluxWeakeningController};
pub use control::PiController;
pub use position::ShaftPosition;
pub use traits::{CurrentSensor, HallStateReader, PositionSensor, PwmDuty, PwmOutput, SpeedSensor};
