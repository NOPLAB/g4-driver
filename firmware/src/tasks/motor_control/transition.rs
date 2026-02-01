//! モード遷移管理
//!
//! 遷移に必要なデータを型で明示し、遷移ロジックを一箇所に集約します。

use crate::fmt::*;

use super::hardware::Hardware;
use super::modes::{CalibrationState, FocState, OpenLoopState};
use super::state_machine::MotorState;

/// モード遷移を表現（遷移に必要なデータを含む）
pub enum Transition {
    /// FOCへ遷移
    Foc {
        initial_vq: f32,
        current_rpm: f32,
        is_reverse: bool,
    },
    /// OpenLoopへ遷移
    OpenLoop {
        /// 脱調回復モードか
        is_recovery: bool,
    },
    /// キャリブレーションへ遷移
    Calibration { torque: f32 },
}

impl Transition {
    /// 遷移を適用し、新しいMotorStateを生成
    pub fn apply(self, hw: &mut Hardware) -> MotorState {
        match self {
            Transition::Foc {
                initial_vq,
                current_rpm,
                is_reverse,
            } => {
                info!(
                    "Transition to FOC: rpm={}, vq={}, reverse={}",
                    current_rpm, initial_vq, is_reverse
                );
                hw.hall_sensor.reset_speed_filter(current_rpm);
                let state = FocState::new(hw.max_duty, initial_vq, current_rpm, is_reverse);
                MotorState::Foc(state)
            }
            Transition::OpenLoop { is_recovery } => {
                if is_recovery {
                    info!("Transition to OpenLoop (recovery mode)");
                } else {
                    info!("Transition to OpenLoop");
                }
                hw.hall_sensor.reset();
                let state = OpenLoopState::new(hw.max_duty, is_recovery);
                MotorState::OpenLoop(state)
            }
            Transition::Calibration { torque } => {
                info!("Transition to Calibration: torque={}", torque);
                let state = CalibrationState::new(hw.max_duty, torque);
                MotorState::Calibration(state)
            }
        }
    }
}
