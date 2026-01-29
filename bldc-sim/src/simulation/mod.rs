//! Simulation module
//!
//! Provides the simulation engine and supporting components for
//! running BLDC motor simulations with FOC control.

mod engine;
mod hall_emulator;
mod integration;

pub use engine::{SimConfig, SimStepResult, Simulation};
pub use hall_emulator::{HallEmulator, HallOutput};
pub use integration::{IntegrationMethod, Integrator};
