//! オープンループ制御モード
//!
//! 強制転流で起動し、Hall センサーベースの6ステップ駆動に切り替えます。

use core::sync::atomic::{AtomicU32, Ordering};

use crate::config::openloop::{
    DEFAULT_DUTY_RATIO, FORCED_COMMUTATION_CYCLES, MIN_CYCLES_BEFORE_FOC,
};
use crate::fmt::*;
use crate::foc::{HallSensor, OpenLoopSixStep};
use crate::hall_tim;
use crate::motor_driver::MotorDriver;
use crate::state;

/// オープンループログカウンタ（1Hz = 10000サイクルごと @ 10kHz）
static OPENLOOP_LOG_COUNTER: AtomicU32 = AtomicU32::new(0);

/// OpenLoop実行カウンタ（FOCへの切り替え判定用）
static OPENLOOP_EXECUTION_COUNTER: AtomicU32 = AtomicU32::new(0);

/// Hall 状態から6ステップ駆動パターンを取得
///
/// 一般的な Hall センサー配置での対応表:
/// Hall State | Step | High | Low | Off (Duty=50%)
/// -----------|------|------|-----|---------------
/// 1 (001)    | 0    | U    | V   | W
/// 3 (011)    | 1    | U    | W   | V
/// 2 (010)    | 2    | V    | W   | U
/// 6 (110)    | 3    | V    | U   | W
/// 4 (100)    | 4    | W    | U   | V
/// 5 (101)    | 5    | W    | V   | U
///
/// 注: 全チャネル有効のまま、Duty=50 でフローティング状態を実現
fn hall_to_commutation(hall_state: u8, duty: u16) -> (u16, u16, u16) {
    // (duty_u, duty_v, duty_w) - 全チャネル有効
    // High = duty, Low = 0, Off = 50 (フローティング相当)
    let off = 50; // 50% duty でフローティング
    match hall_state {
        // Hall 1: U-High, V-Low, W-Off
        1 => (duty, 0, off),
        // Hall 3: U-High, W-Low, V-Off
        3 => (duty, off, 0),
        // Hall 2: V-High, W-Low, U-Off
        2 => (off, duty, 0),
        // Hall 6: V-High, U-Low, W-Off
        6 => (0, duty, off),
        // Hall 4: W-High, U-Low, V-Off
        4 => (0, off, duty),
        // Hall 5: W-High, V-Low, U-Off
        5 => (off, 0, duty),
        // 無効な Hall 状態: 全チャネル 50% (停止)
        _ => (50, 50, 50),
    }
}

/// OpenLoop実行カウンタをリセット
pub fn reset_execution_counter() {
    OPENLOOP_EXECUTION_COUNTER.store(0, Ordering::Relaxed);
    OPENLOOP_LOG_COUNTER.store(0, Ordering::Relaxed);
}

/// オープンループ制御の実行
///
/// フェーズ1（0-500ms）：強制転流で起動
/// フェーズ2（500ms-1s）：Hall駆動に切り替え
/// フェーズ3（1s以降）：FOCに切り替え可能
///
/// # 引数
/// * `openloop` - オープンループコントローラー（強制転流用）
/// * `hall_sensor` - Hallセンサー（Hall状態確認用）
/// * `motor_driver` - モータードライバー
/// * `dt` - 制御周期 [s]
///
/// # 戻り値
/// * `(bool, u8)` - (FOCに切り替え可能か, Hall状態)
pub async fn execute(
    openloop: &mut OpenLoopSixStep,
    _hall_sensor: &HallSensor,
    motor_driver: &mut MotorDriver,
    dt: f32,
) -> (bool, u8) {
    // OpenLoop実行カウンタをインクリメント
    let exec_count = OPENLOOP_EXECUTION_COUNTER.fetch_add(1, Ordering::Relaxed);

    // Hall状態を取得
    let hall_state = hall_tim::get_hall_state();
    let is_valid_hall = (1..=6).contains(&hall_state);

    // PWM最大値を取得
    let pwm_max_duty = motor_driver.max_duty();

    // フェーズ判定
    let (scaled_duty_u, scaled_duty_v, scaled_duty_w, enable_u, enable_v, enable_w, phase_name) =
        if exec_count < FORCED_COMMUTATION_CYCLES {
            // フェーズ1：強制転流（時間ベースの6ステップ駆動）
            let state = openloop.update(dt);

            // 0-100 を PWM最大値にスケーリング
            let scaled_u = (state.duty_u as u32 * pwm_max_duty as u32 / 100) as u16;
            let scaled_v = (state.duty_v as u32 * pwm_max_duty as u32 / 100) as u16;
            let scaled_w = (state.duty_w as u32 * pwm_max_duty as u32 / 100) as u16;

            (
                scaled_u,
                scaled_v,
                scaled_w,
                state.enable_u,
                state.enable_v,
                state.enable_w,
                "Forced",
            )
        } else {
            // フェーズ2：Hall駆動（全チャンネル有効、50%フローティング）
            let duty = DEFAULT_DUTY_RATIO;
            let (du, dv, dw) = hall_to_commutation(hall_state, duty);

            let scaled_u = (du as u32 * pwm_max_duty as u32 / 100) as u16;
            let scaled_v = (dv as u32 * pwm_max_duty as u32 / 100) as u16;
            let scaled_w = (dw as u32 * pwm_max_duty as u32 / 100) as u16;

            (scaled_u, scaled_v, scaled_w, true, true, true, "Hall")
        };

    // PWM出力
    motor_driver.set_duty_uvw(scaled_duty_u, scaled_duty_v, scaled_duty_w);

    // チャンネル有効/無効設定
    motor_driver.set_channels(enable_u, enable_v, enable_w);

    // ステータス更新（Atomic変数でロックフリー）
    state::update_motor_status_atomic(openloop.get_current_rpm(), 0.0);

    // デバッグログ（1秒ごと @ 10kHz）
    let count = OPENLOOP_LOG_COUNTER.fetch_add(1, Ordering::Relaxed);
    if count >= 10000 {
        OPENLOOP_LOG_COUNTER.store(0, Ordering::Relaxed);
        info!(
            "[{}] Hall:{}, Duty:{}/{}/{}, En:{}/{}/{}, Cycle:{}",
            phase_name,
            hall_state,
            scaled_duty_u,
            scaled_duty_v,
            scaled_duty_w,
            enable_u,
            enable_v,
            enable_w,
            exec_count
        );
    }

    // フェーズ切り替えログ
    if exec_count == FORCED_COMMUTATION_CYCLES {
        info!("[OpenLoop] Switching from Forced to Hall-based commutation");
    }

    // FOC切り替え判定
    let ready_for_foc = exec_count >= MIN_CYCLES_BEFORE_FOC && is_valid_hall;

    if exec_count == MIN_CYCLES_BEFORE_FOC {
        info!(
            "[OpenLoop] Ready for FOC transition after {} cycles",
            exec_count
        );
    }

    (ready_for_foc, hall_state)
}
