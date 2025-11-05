//! オープンループ制御モード
//!
//! 始動時に6ステップ駆動（台形波）でモーターを回転させます。

use crate::fmt::*;
use crate::foc::{HallSensor, OpenLoopSixStep};
use crate::hall_tim;
use crate::motor_driver::MotorDriver;
use crate::state::MOTOR_STATUS;

/// オープンループ制御の実行
///
/// # 引数
/// * `openloop` - オープンループコントローラー
/// * `hall_sensor` - Hallセンサー（Hall状態確認用）
/// * `motor_driver` - モータードライバー
/// * `dt` - 制御周期 [s]
///
/// # 戻り値
/// * `(bool, u8)` - (目標速度に達したか, Hall状態)
pub async fn execute(
    openloop: &mut OpenLoopSixStep,
    _hall_sensor: &HallSensor,
    motor_driver: &mut MotorDriver,
    dt: f32,
) -> (bool, u8) {
    // オープンループ6ステップ駆動を更新
    let step_state = openloop.update(dt);

    // Hall状態を取得（切替判定用）
    let hall_state = hall_tim::get_hall_state();
    let is_valid_hall = (1..=6).contains(&hall_state);
    let target_reached = openloop.is_target_reached();

    // PWM出力（0-100の値を実際のPWM最大値にスケーリング）
    let pwm_max_duty = motor_driver.max_duty();
    let scaled_duty_u = (step_state.duty_u as u32 * pwm_max_duty as u32 / 100) as u16;
    let scaled_duty_v = (step_state.duty_v as u32 * pwm_max_duty as u32 / 100) as u16;
    let scaled_duty_w = (step_state.duty_w as u32 * pwm_max_duty as u32 / 100) as u16;

    motor_driver.set_duty_uvw(scaled_duty_u, scaled_duty_v, scaled_duty_w);

    // チャネル有効/無効制御
    motor_driver.set_channels(
        step_state.enable_u,
        step_state.enable_v,
        step_state.enable_w,
    );

    // ステータス更新
    {
        let mut status = MOTOR_STATUS.lock().await;
        status.speed_rpm = openloop.get_current_rpm();
        status.electrical_angle = 0.0; // OpenLoopでは電気角は不定
    }

    // デバッグログ（低頻度）
    static mut OPENLOOP_LOG_COUNTER: u32 = 0;
    unsafe {
        OPENLOOP_LOG_COUNTER += 1;
        if OPENLOOP_LOG_COUNTER >= 2500 {
            OPENLOOP_LOG_COUNTER = 0;
            debug!(
                "[OpenLoop] Step: {}, RPM: {}, Hall: {}, DutyU/V/W: {}/{}/{}",
                step_state.step,
                openloop.get_current_rpm(),
                hall_state,
                step_state.duty_u,
                step_state.duty_v,
                step_state.duty_w
            );
        }
    }

    (is_valid_hall && target_reached, hall_state)
}
