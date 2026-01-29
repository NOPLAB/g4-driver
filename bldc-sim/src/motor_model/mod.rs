//! Motor model module
//!
//! Provides electrical and mechanical modeling of PMSM/BLDC motors
//! for simulation purposes.

mod dynamics;
mod params;
mod state;

pub use dynamics::{LoadTorque, MotorDynamics, PowerLoss, StateDerivatives, VoltageInput};
pub use params::{MotorParams, MotorParamsBuilder};
pub use state::{normalize_angle, MotorState, StateSnapshot};
