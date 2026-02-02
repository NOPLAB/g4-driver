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
//! - [`control`]: Control algorithms (PI controller, FOC, open-loop, speed ramp, stall detector)
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
//! use bldc::control::{FocController, FocConfig};
//! use bldc::control::stall_detector::StallDetectorConfig;
//!
//! // Create a FOC controller with all features
//! let mut controller = FocController::builder(FocConfig::default())
//!     .with_stall_detection(StallDetectorConfig::default())
//!     .build();
//!
//! // In your control loop:
//! controller.set_target_speed_rpm(1000.0);
//! let output = controller.update_extended(measured_speed, electrical_angle, dt);
//!
//! if output.is_stalled {
//!     // Handle stall condition
//! }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "calibration")]
pub mod calibration;
pub mod compensation;
pub mod control;
pub mod modulation;
pub mod position;
#[cfg(feature = "hall")]
pub mod sensors;
#[cfg(feature = "state-machine")]
pub mod state_machine;
pub mod traits;
pub mod transforms;

// Re-export commonly used types
pub use compensation::{DeadTimeCompensation, FluxWeakeningController};
pub use control::{
    FocConfig, FocController, FocOutput, OpenLoopConfig, OpenLoopController, OpenLoopOutput,
    OpenLoopPhase, PiController, SpeedRamp, StallDetector, StallDetectorConfig,
};
pub use position::ShaftPosition;
pub use traits::{
    ControlInput, ControlMode, CurrentSensor, HallStateReader, PositionSensor, PwmDuty, PwmOutput,
    SpeedSensor, StatusOutput,
};

// State machine re-exports (when feature is enabled)
#[cfg(all(feature = "state-machine", feature = "calibration"))]
pub use state_machine::CalibrationMode;
#[cfg(feature = "state-machine")]
pub use state_machine::{
    ControlState, FocMode, ModeOutput, MotorStateMachine, OpenLoopMode, StateMachineBuilder,
    StateMachineConfig, StateTransition,
};
