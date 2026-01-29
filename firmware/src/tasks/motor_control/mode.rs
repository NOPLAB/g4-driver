//! モード遷移管理
//!
//! 制御モード間の遷移ロジックとモードの共通インターフェースを定義します。

use crate::motor_driver::MotorDriver;
use crate::state::ControlMode;

use super::resources::ControllerResources;

/// モード遷移結果
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ModeResult {
    /// 現在のモードを継続
    Continue,
    /// 次のモードへ遷移
    TransitionTo(ControlMode),
}

impl ModeResult {
    /// 遷移先モードを取得（遷移がある場合のみ）
    pub fn next_mode(self) -> Option<ControlMode> {
        match self {
            ModeResult::Continue => None,
            ModeResult::TransitionTo(mode) => Some(mode),
        }
    }
}

/// モード実行に必要な共有コンテキスト
pub struct ModeContext<'a> {
    /// 制御リソース（Hallセンサー、PIコントローラ等）
    pub resources: &'a mut ControllerResources,
    /// モータードライバー
    pub motor_driver: &'a mut MotorDriver,
    /// 制御周期 [s]
    pub dt: f32,
}

impl<'a> ModeContext<'a> {
    /// 新しいModeContextを作成
    pub fn new(
        resources: &'a mut ControllerResources,
        motor_driver: &'a mut MotorDriver,
        dt: f32,
    ) -> Self {
        Self {
            resources,
            motor_driver,
            dt,
        }
    }
}
