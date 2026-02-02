//! Simulation module
//!
//! Provides the simulation engine and supporting components for
//! running BLDC motor simulations with FOC control.

mod adapters;
mod engine;
mod hall_emulator;
mod integration;

pub use adapters::{SimControlInput, SimStatusOutput, SimulatedHardware, StatusSnapshot};
pub use engine::{
    SimConfig, SimStepResult, Simulation, StateMachineSimulation, StateMachineStepResult,
};
pub use hall_emulator::{HallEmulator, HallOutput};
pub use integration::{IntegrationMethod, Integrator};
