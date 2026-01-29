// FOC (Field Oriented Control) module
// Hall sensor-based FOC implementation for BLDC motor control

pub mod hall;
pub mod openloop_six_step;
pub mod pi_controller;

// Re-export from bldc crate for backward compatibility
pub use bldc::calibration::{CalibrationResult, MotorCalibration};
pub use bldc::compensation::{DeadTimeCompensation, FluxWeakeningController};

// Re-export main types for easier access
pub use hall::HallSensor;
pub use openloop_six_step::OpenLoopSixStep;
pub use pi_controller::PiController;
// Note: calculate_svpwm, inverse_park, limit_voltage are now imported from the bldc crate
// Use `bldc::modulation::calculate_svpwm` and `bldc::transforms::*` instead

/// モーター制御モード
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlMode {
    /// オープンループ強制転流（始動時）
    OpenLoop,
    /// クローズドループFOC制御（通常運転）
    ClosedLoopFoc,
    /// キャリブレーションモード（電気角オフセット・回転方向の自動検出）
    Calibration,
}
