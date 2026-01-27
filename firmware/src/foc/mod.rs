// FOC (Field Oriented Control) module
// Hall sensor-based FOC implementation for BLDC motor control

pub mod calibration;
pub mod hall_sensor;
pub mod openloop_six_step;
pub mod pi_controller;
pub mod shaft_position;
pub mod svpwm;
pub mod transforms;

// Re-export main types for easier access
pub use calibration::{CalibrationResult, MotorCalibration};
pub use hall_sensor::HallSensor;
pub use openloop_six_step::OpenLoopSixStep;
pub use pi_controller::PiController;
pub use svpwm::calculate_svpwm;
pub use transforms::{inverse_park, limit_voltage};

// Benchmark function for performance testing
#[cfg(not(test))]
pub use transforms::benchmark_inverse_park;

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
