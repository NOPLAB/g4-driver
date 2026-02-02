//! Test scenarios module
//!
//! Pre-built scenarios for validating motor control:
//! - Step response
//! - Ramp response
//! - Load disturbance
//! - Startup characteristics
//! - State machine scenarios (OpenLoop → FOC transition, stall recovery)

mod disturbance;
mod ramp;
mod startup;
mod step;
mod traits;

pub mod state_machine;

pub use disturbance::LoadDisturbance;
pub use ramp::RampResponse;
pub use startup::StartupScenario;
pub use step::StepResponse;
pub use traits::{LoadProfile, Scenario, ScenarioMetrics, ScenarioResult, SpeedProfile};

// State machine scenarios
pub use state_machine::{
    ModeDurations, StartupWithTransitionScenario, StateMachineLoadDisturbance,
    StateMachineScenarioResult, StateMachineStepResponse,
};
