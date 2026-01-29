//! モード遷移管理
//!
//! 制御モード間の遷移ロジックを定義します。

use crate::foc::ControlMode;

/// モード遷移結果
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TransitionResult {
    /// 現在のモードを継続
    Continue,
    /// 次のモードへ遷移
    TransitionTo(ControlMode),
}

impl TransitionResult {
    /// 遷移先モードを取得（遷移がある場合のみ）
    pub fn next_mode(self) -> Option<ControlMode> {
        match self {
            TransitionResult::Continue => None,
            TransitionResult::TransitionTo(mode) => Some(mode),
        }
    }
}
