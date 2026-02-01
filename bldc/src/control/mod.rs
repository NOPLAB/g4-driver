//! Control algorithms for BLDC motor control
//!
//! This module provides various control algorithms including:
//! - PI controller for speed and current control
//! - FOC (Field Oriented Control) controller
//! - Open-loop SVPWM controller
//! - Speed ramp (acceleration limiter)
//! - Stall detector
//! - Six-step commutation controller (legacy)

mod pi;
pub mod speed_ramp;
pub mod stall_detector;

pub mod foc;
pub mod six_step;

pub use foc::{FocConfig, FocController, FocControllerBuilder, FocOutput};
pub use pi::PiController;
pub use six_step::{
    OpenLoopConfig, OpenLoopController, OpenLoopOutput, OpenLoopPhase, SixStepController,
    SixStepState,
};
pub use speed_ramp::SpeedRamp;
pub use stall_detector::{StallDetector, StallDetectorConfig};
