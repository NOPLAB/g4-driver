//! FOC（Field Oriented Control）制御モード
//!
//! Hallセンサーベースのクローズドループ速度制御を実行します。

use crate::config::*;
use crate::fmt::*;
use crate::foc::{calculate_svpwm, inverse_park, limit_voltage, HallSensor, PiController};
use crate::hall_tim;
use crate::motor_driver::MotorDriver;
use crate::state::{MOTOR_STATUS, SPEED_PI_GAINS, TARGET_SPEED};

/// FOC制御の実行
///
/// # 引数
/// * `hall_sensor` - Hallセンサー
/// * `speed_pi` - 速度PIコントローラー
/// * `motor_driver` - モータードライバー
/// * `ramped_target_speed` - ランプ処理後の目標速度
/// * `dt` - 制御周期 [s]
///
/// # 戻り値
/// * `(bool, f32)` - (Hall状態が有効か, 新しいランプ速度)
pub async fn execute(
    hall_sensor: &mut HallSensor,
    speed_pi: &mut PiController,
    motor_driver: &mut MotorDriver,
    ramped_target_speed: &mut f32,
    dt: f32,
) -> bool {
    // Hall状態の確認（有効な状態：1-6）
    let hall_state = hall_tim::get_hall_state();
    let is_valid_hall = (1..=6).contains(&hall_state);

    // Hallセンサが無効な場合の安全処理
    if !is_valid_hall {
        motor_driver.stop();
        speed_pi.reset();
        *ramped_target_speed = 0.0;
        return false;
    }

    // 電気角と速度を取得（TIM4ハードウェアベース、foc-simple互換計算）
    let (hall_electrical_angle, speed_rpm) = hall_sensor.update(dt);

    // PIゲイン更新チェック（非同期で更新された場合）
    {
        let (kp, ki) = *SPEED_PI_GAINS.lock().await;
        if kp != speed_pi.get_kp() || ki != speed_pi.get_ki() {
            speed_pi.set_gains(kp, ki);
            info!("PI gains updated: Kp={}, Ki={}", kp, ki);
        }
    }

    // 目標速度取得
    let target_speed = *TARGET_SPEED.lock().await;

    // 速度ランプ（加速度制限）を適用
    let speed_error = target_speed - *ramped_target_speed;
    let max_delta_speed = MAX_SPEED_ACCELERATION * dt; // 1制御周期で変化可能な最大速度

    if speed_error.abs() > max_delta_speed {
        // 加速度制限を適用
        if speed_error > 0.0 {
            *ramped_target_speed += max_delta_speed;
        } else {
            *ramped_target_speed -= max_delta_speed;
        }
    } else {
        // 目標速度に到達
        *ramped_target_speed = target_speed;
    }

    // 速度PI制御（q軸電圧指令生成）- ランプ処理後の速度を使用
    let mut vq_cmd = speed_pi.update(*ramped_target_speed, speed_rpm, dt);
    let vd_cmd = 0.0; // SPMSM: d軸電流/電圧は0

    // 停止時の処理：目標速度が0で実際に停止している場合、PI積分項をリセット
    if ramped_target_speed.abs() < 1.0 && speed_rpm.abs() < 1.0 {
        speed_pi.reset();
        vq_cmd = 0.0;
    }

    // 最小電圧適用（静止摩擦克服用）
    let speed_error_abs = (*ramped_target_speed - speed_rpm).abs();
    if speed_error_abs > MIN_VOLTAGE_ERROR_THRESHOLD && vq_cmd.abs() > 0.0 {
        // 速度誤差が大きい場合、最小電圧を適用
        if vq_cmd > 0.0 {
            vq_cmd = vq_cmd.max(MIN_VOLTAGE);
        } else {
            vq_cmd = vq_cmd.min(-MIN_VOLTAGE);
        }
    }

    // 電圧ベクトル制限
    let (vd_limited, vq_limited) = limit_voltage(vd_cmd, vq_cmd, DEFAULT_MAX_VOLTAGE);

    // Park逆変換（dq → αβ）
    let (v_alpha, v_beta) = inverse_park(vd_limited, vq_limited, hall_electrical_angle);

    // SVPWM計算（実際のPWM最大値を使用）
    let pwm_max_duty = motor_driver.max_duty();
    let (duty_u, duty_v, duty_w) = calculate_svpwm(v_alpha, v_beta, DEFAULT_V_DC_BUS, pwm_max_duty);

    // デバッグ用：FOC制御の詳細ログ（10Hz = 250回に1回）
    static mut FOC_LOG_COUNTER: u32 = 0;
    unsafe {
        FOC_LOG_COUNTER += 1;
        if FOC_LOG_COUNTER >= 250 {
            FOC_LOG_COUNTER = 0;
            let angle_deg = hall_electrical_angle * 180.0 / core::f32::consts::PI;
            trace!(
                "[FOC Detail] Hall={}, Angle={}rad ({}°), Vq={}V, Valpha={}V, Vbeta={}V, DutyU={}, DutyV={}, DutyW={}",
                hall_state, hall_electrical_angle, angle_deg, vq_limited, v_alpha, v_beta, duty_u, duty_v, duty_w
            );
        }
    }

    // PWM出力
    motor_driver.set_duty_uvw(duty_u, duty_v, duty_w);

    // FOCモードではすべてのチャネルを有効化
    motor_driver.enable_all_channels();

    // ステータス更新
    {
        let mut status = MOTOR_STATUS.lock().await;
        status.speed_rpm = speed_rpm;
        status.electrical_angle = hall_electrical_angle;
    }

    // デバッグログ（低頻度）
    static mut FOC_MODE_LOG_COUNTER: u32 = 0;
    unsafe {
        FOC_MODE_LOG_COUNTER += 1;
        if FOC_MODE_LOG_COUNTER >= 2500 {
            // 1秒ごと（2.5kHz / 2500 = 1Hz）
            FOC_MODE_LOG_COUNTER = 0;

            // TIM4ベースのHallセンサ値を取得（ログ用）
            let period_cycles = hall_tim::get_period_cycles();

            // 最新のステータスを取得
            let status = *MOTOR_STATUS.lock().await;
            let target_speed = *TARGET_SPEED.lock().await;
            debug!(
                "[FOC] Speed: {}/{} RPM (ramped: {}), Angle: {}rad, Hall: {}, Period: {} cycles",
                status.speed_rpm,
                target_speed,
                *ramped_target_speed,
                status.electrical_angle,
                hall_state,
                period_cycles
            );
        }
    }

    true
}
