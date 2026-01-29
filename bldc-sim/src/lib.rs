//! BLDC Motor Physics Simulation Library
//!
//! This crate provides a detailed physics-based simulation of BLDC/PMSM motors
//! for validating FOC (Field Oriented Control) implementations.
//!
//! # Features
//!
//! - **Motor Model**: Complete electrical and mechanical dynamics in dq reference frame
//! - **Nonlinear Effects**: Magnetic saturation, cogging torque, dead-time compensation
//! - **Thermal Model**: Temperature-dependent resistance and performance
//! - **Hall Sensor Emulation**: Realistic sensor behavior for FOC testing
//! - **Test Scenarios**: Pre-built scenarios for step response, ramp, disturbance, and startup
//!
//! # Example
//!
//! ```rust
//! use bldc_sim::motor_model::{MotorParams, MotorState, MotorDynamics, VoltageInput, LoadTorque};
//! use bldc_sim::simulation::{Integrator, IntegrationMethod};
//!
//! // Create motor with default parameters
//! let params = MotorParams::default_small_bldc();
//! let dynamics = MotorDynamics::new(params);
//! let integrator = Integrator::rk4();
//!
//! // Initialize state
//! let mut state = MotorState::new();
//!
//! // Simulation loop
//! let dt = 0.0001; // 100 μs time step
//! let voltage = VoltageInput::new(0.0, 1.0); // Apply q-axis voltage
//! let load = LoadTorque::zero();
//!
//! for _ in 0..1000 {
//!     integrator.step(&dynamics, &mut state, &voltage, &load, dt);
//! }
//!
//! println!("Speed: {} RPM", state.speed_rpm());
//! ```
//!
//! # Feature Flags
//!
//! - `visualization`: Enable plot generation with `plotters`
//! - `csv-output`: Enable CSV data export
//! - `full-output`: Enable both visualization and CSV output

pub mod motor_model;
pub mod simulation;

// These modules will be added in later phases
pub mod nonlinear;
pub mod output;
pub mod scenarios;
pub mod thermal;
pub mod validation;

// Re-export commonly used types
pub use motor_model::{MotorDynamics, MotorParams, MotorState, VoltageInput, LoadTorque, StateSnapshot};
pub use simulation::{Integrator, IntegrationMethod, Simulation, SimConfig, SimStepResult, HallEmulator};
