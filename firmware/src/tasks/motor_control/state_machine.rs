//! モーター制御ステートマシン
//!
//! 各制御モードを状態として保持し、統一インターフェースで制御を実行します。

use crate::state::ControlMode;

use super::hardware::Hardware;
use super::modes::{CalibrationState, FocState, OpenLoopState};
use super::transition::Transition;

/// モーター制御の状態マシン
pub enum MotorState {
    /// オープンループ制御（起動・脱調回復）
    OpenLoop(OpenLoopState),
    /// FOC閉ループ制御
    Foc(FocState),
    /// キャリブレーション
    Calibration(CalibrationState),
}

impl MotorState {
    /// 初期状態を生成
    pub fn new(max_duty: u16) -> Self {
        MotorState::OpenLoop(OpenLoopState::new(max_duty, false))
    }

    /// 1制御サイクルを実行し、遷移があればTransitionを返す
    pub async fn update(&mut self, hw: &mut Hardware, dt: f32) -> Option<Transition> {
        match self {
            MotorState::OpenLoop(state) => state.execute(hw, dt).await,
            MotorState::Foc(state) => state.execute(hw, dt).await,
            MotorState::Calibration(state) => state.execute(hw, dt).await,
        }
    }

    /// 現在のControlModeを取得（CAN送信用）
    pub fn control_mode(&self) -> ControlMode {
        match self {
            MotorState::OpenLoop(_) => ControlMode::OpenLoop,
            MotorState::Foc(_) => ControlMode::ClosedLoopFoc,
            MotorState::Calibration(_) => ControlMode::Calibration,
        }
    }
}
