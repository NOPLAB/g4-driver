//! Control algorithms for BLDC motor control
//!
//! This module provides various control algorithms including:
//! - PI controller for speed and current control
//! - FOC (Field Oriented Control) controller
//! - Six-step commutation controller

mod pi;

pub mod foc;
pub mod six_step;

pub use pi::PiController;
