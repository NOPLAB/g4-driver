//! Control mode implementations

mod foc;
mod openloop;

#[cfg(feature = "calibration")]
mod calibration;

pub use foc::{FocMode, FocModeBuilder};
pub use openloop::OpenLoopMode;

#[cfg(feature = "calibration")]
pub use calibration::CalibrationMode;

use crate::traits::{ControlMode, PwmDuty};

/// State transition request from a mode
#[derive(Debug, Clone)]
pub enum StateTransition {
    /// Transition to FOC mode
    ToFoc {
        /// Initial Vq voltage for smooth transition
        initial_vq: f32,
        /// Current motor speed in RPM
        current_rpm: f32,
        /// Whether motor is in reverse direction
        is_reverse: bool,
    },
    /// Transition to OpenLoop mode
    ToOpenLoop {
        /// Whether this is a recovery from stall
        is_recovery: bool,
    },
    /// Transition to Calibration mode
    #[cfg(feature = "calibration")]
    ToCalibration {
        /// Calibration torque (0.0 to 1.0)
        torque: f32,
    },
}

/// Common result from mode update
#[derive(Debug, Clone, Default)]
pub struct ModeOutput {
    /// PWM duty cycles
    pub duty: PwmDuty,
    /// Current speed in RPM
    pub speed_rpm: f32,
    /// Current electrical angle in radians
    pub electrical_angle: f32,
    /// State transition request (if any)
    pub transition: Option<StateTransition>,
}

/// Control state enum holding the active mode
pub enum ControlState {
    /// Open-loop control mode
    OpenLoop(OpenLoopMode),
    /// FOC closed-loop control mode
    Foc(FocMode),
    /// Calibration mode
    #[cfg(feature = "calibration")]
    Calibration(CalibrationMode),
}

impl ControlState {
    /// Get the current control mode
    pub fn mode(&self) -> ControlMode {
        match self {
            ControlState::OpenLoop(_) => ControlMode::OpenLoop,
            ControlState::Foc(_) => ControlMode::Foc,
            #[cfg(feature = "calibration")]
            ControlState::Calibration(_) => ControlMode::Calibration,
        }
    }
}
