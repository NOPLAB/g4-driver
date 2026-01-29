//! キャリブレーション制御モード
//!
//! モーターの電気角オフセットと回転方向を自動検出します。

use core::f32::consts::PI;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::config::*;
use crate::fmt::*;
use crate::foc::{calculate_svpwm, inverse_park, ControlMode, HallSensor, MotorCalibration};
use crate::motor_driver::MotorDriver;
use crate::state;

/// キャリブレーションデバッグログカウンタ（1Hz = 10000サイクルごと @ 10kHz）
static DEBUG_COUNTER: AtomicU32 = AtomicU32::new(0);

/// キャリブレーション制御の実行
///
/// # 引数
/// * `calibration` - キャリブレーションコントローラー
/// * `hall_sensor` - Hallセンサー
/// * `motor_driver` - モータードライバー
/// * `dt` - 制御周期 [秒]
///
/// # 戻り値
/// * `Option<ControlMode>` - 完了時は次のモード（ClosedLoopFocまたはOpenLoop）、継続中はNone
pub async fn execute(
    calibration: &mut MotorCalibration,
    hall_sensor: &mut HallSensor,
    motor_driver: &mut MotorDriver,
    dt: f32,
) -> Option<ControlMode> {
    // Hall センサーを更新して現在の角度を取得
    let (_electrical_angle, _speed_rpm, _hall_state) = hall_sensor.update(dt);
    let sensor_angle = hall_sensor.get_mechanical_angle();

    // デバッグ：Hall状態と角度を定期的にログ出力（10000サイクルごと = 1秒 @ 10kHz）
    let count = DEBUG_COUNTER.fetch_add(1, Ordering::Relaxed);
    if count >= 10000 {
        DEBUG_COUNTER.store(0, Ordering::Relaxed);
        let hall_state = crate::hall_tim::get_hall_state();
        info!(
            "[Calibration Execute] Hall state={}, sensor_angle={} rad ({} deg)",
            hall_state,
            sensor_angle,
            sensor_angle * 180.0 / PI
        );
    }

    // キャリブレーションステートマシンを更新
    match calibration.update(sensor_angle) {
        Ok((electrical_angle, torque)) => {
            // トルクから電圧指令を計算（トルク 0.0～1.0 → 電圧 0～MAX_VOLTAGE）
            let v_cmd = torque * DEFAULT_MAX_VOLTAGE;

            // d軸・q軸電圧（キャリブレーション中はシンプルにq軸のみ）
            let vd_cmd = 0.0;
            let vq_cmd = v_cmd;

            // Park逆変換
            let (v_alpha, v_beta) = inverse_park(vd_cmd, vq_cmd, electrical_angle);

            // SVPWM計算（実際のPWM最大値を使用）
            let pwm_max_duty = motor_driver.max_duty();
            let (duty_u, duty_v, duty_w) =
                calculate_svpwm(v_alpha, v_beta, DEFAULT_V_DC_BUS, pwm_max_duty);

            // PWM出力
            motor_driver.set_duty_uvw(duty_u, duty_v, duty_w);

            // すべてのチャネルを有効化
            motor_driver.enable_all_channels();

            // キャリブレーション完了チェック
            if calibration.is_completed() {
                let result = calibration.get_result();

                if result.success {
                    info!("Calibration completed successfully!");
                    info!(
                        "  Electrical offset: {} rad ({} deg)",
                        result.electrical_offset,
                        result.electrical_offset * 180.0 / PI
                    );
                    info!("  Direction inversed: {}", result.direction_inversed);

                    // 結果をグローバル状態に保存
                    state::calibration_context().await.result = result;

                    // Hall センサーに結果を適用
                    hall_sensor.set_electrical_offset(result.electrical_offset);
                    // TODO: 方向反転の適用（HallSensor に direction_inversed を追加する必要がある）

                    // ClosedLoopFocモードに切り替え
                    state::motor_context().await.control_mode = ControlMode::ClosedLoopFoc;

                    info!("Switching to ClosedLoopFoc mode");
                    return Some(ControlMode::ClosedLoopFoc);
                } else {
                    error!("Calibration failed!");
                    // エラー時はモーターを停止
                    motor_driver.stop();

                    // OpenLoopモードに戻る
                    state::motor_context().await.control_mode = ControlMode::OpenLoop;

                    return Some(ControlMode::OpenLoop);
                }
            }
        }
        Err(_) => {
            error!("Calibration update error, stopping motor");
            // エラー時はモーターを停止
            motor_driver.stop();

            // OpenLoopモードに戻る
            state::motor_context().await.control_mode = ControlMode::OpenLoop;

            return Some(ControlMode::OpenLoop);
        }
    }

    None // キャリブレーション継続中
}
