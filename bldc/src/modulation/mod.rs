//! PWM modulation strategies
//!
//! This module provides different PWM modulation techniques for
//! three-phase inverter control.

pub mod svpwm;

pub use svpwm::{calculate_svpwm, calculate_sinusoidal_pwm};
