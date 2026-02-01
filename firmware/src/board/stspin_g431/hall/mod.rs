//! TIM4ベースのHallセンサーインターフェース実装
//!
//! STM32のハードウェアHall Sensor Interface Mode（XORモード）を使用して、
//! 3つのHallセンサー入力から自動的にエッジ検出とタイムスタンプキャプチャを行います。
//!
//! ## ハードウェア構成
//! - TIM4_CH1 (PB6): Hall H1
//! - TIM4_CH2 (PB7): Hall H2
//! - TIM4_CH3 (PB8): Hall H3
//! - クロック: 170MHz (APB1)
//!
//! ## 動作原理（参照: HAL_TIMEx_HallSensor_Init）
//! 1. 3つのHall入力がXORされてTI1に接続される（CR2.TI1S=1）
//! 2. TI1のエッジ検出がトリガーとして選択される（SMCR.TS=TI1F_ED）
//! 3. トリガーエッジでカウンターがリセットされる（SMCR.SMS=RESET）
//! 4. いずれかのHallセンサーがエッジを検出すると、TIM4_CCR1にカウンタ値がキャプチャされる
//! 5. CC1割り込みが発生し、エッジ間の時間から速度を計算
//! 6. UPDATE割り込みでタイムアウト（低速/停止）を検出

mod isr;
mod state;

pub use isr::init_hall_timer;
pub use state::{
    calculate_speed_rpm, get_hall_state, get_period_cycles, get_snapshot, reset_state,
};

use crate::board::traits::HallSensorInterface;

/// Hallセンサーラッパー構造体
///
/// 静的Atomic変数へのアクセサとして機能するゼロサイズ型。
/// HallSensorInterfaceトレイトを実装。
#[allow(dead_code)]
pub struct HallSensor;

impl HallSensor {
    /// 新しいHallSensorインスタンスを作成
    #[allow(dead_code)]
    pub const fn new() -> Self {
        Self
    }
}

impl Default for HallSensor {
    fn default() -> Self {
        Self::new()
    }
}

impl HallSensorInterface for HallSensor {
    #[inline(always)]
    fn get_hall_state(&self) -> u8 {
        get_hall_state()
    }

    #[inline(always)]
    fn get_period_cycles(&self) -> u32 {
        get_period_cycles()
    }

    #[inline(always)]
    fn is_timeout(&self) -> bool {
        state::is_timeout()
    }

    #[inline(always)]
    fn get_snapshot(&self) -> (u8, u32, bool) {
        get_snapshot()
    }

    #[inline(always)]
    fn calculate_speed_rpm(&self, period_cycles: u32, pole_pairs: u8) -> f32 {
        calculate_speed_rpm(period_cycles, pole_pairs)
    }

    fn reset_state(&mut self) {
        reset_state()
    }
}
