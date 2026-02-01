//! キャリブレーション制御モード
//!
//! モーターの電気角オフセットと回転方向を自動検出します。

use core::f32::consts::PI;

use bldc::calibration::MotorCalibration;
use bldc::modulation::calculate_svpwm;
use bldc::transforms::inverse_park;

use crate::adapters::HallStateReaderAdapter;
use crate::config::{motor, voltage};
use crate::fmt::*;
use crate::state::{self, ControlMode, RUNTIME};

use super::super::hardware::Hardware;
use super::super::logging::PeriodicLogger;
use super::super::transition::Transition;

/// CalibrationModeの状態
pub struct CalibrationState {
    controller: MotorCalibration,
    max_duty: u16,
    logger: PeriodicLogger,
}

impl CalibrationState {
    /// 新しいCalibrationStateを作成
    pub fn new(max_duty: u16, torque: f32) -> Self {
        let mut controller = MotorCalibration::new(motor::DEFAULT_POLE_PAIRS);
        controller.set_torque(torque);
        controller.start();

        Self {
            controller,
            max_duty,
            logger: PeriodicLogger::every_2500_cycles(),
        }
    }

    /// 1制御サイクルの実行
    pub async fn execute(&mut self, hw: &mut Hardware, dt: f32) -> Option<Transition> {
        // Hall センサーを更新して現在の角度と速度を取得
        let (electrical_angle_raw, speed_rpm, _hall_state) = hw.hall_sensor.update(dt);
        let sensor_angle = hw.hall_sensor.get_mechanical_angle();

        // ステータス更新（CAN送信用）
        RUNTIME.status.update(speed_rpm, electrical_angle_raw);

        // キャリブレーションステートマシンを更新
        let hall_reader = HallStateReaderAdapter;
        match self.controller.update(sensor_angle, &hall_reader) {
            Ok((electrical_angle, torque)) => {
                // トルクから電圧指令を計算
                let v_cmd = torque * voltage::DEFAULT_MAX;

                // d軸・q軸電圧（キャリブレーション中はシンプルにq軸のみ）
                let vd_cmd = 0.0;
                let vq_cmd = v_cmd;

                // Park逆変換
                let (v_alpha, v_beta) = inverse_park(vd_cmd, vq_cmd, electrical_angle);

                // SVPWM計算
                let (duty_u, duty_v, duty_w) =
                    calculate_svpwm(v_alpha, v_beta, voltage::DEFAULT_DC_BUS, self.max_duty);

                // デバッグログ
                if self.logger.tick() {
                    info!(
                        "[Calib] torque={}, v_cmd={}, max={}, duty=({},{},{})",
                        torque, v_cmd, self.max_duty, duty_u, duty_v, duty_w
                    );
                }

                // PWM出力
                hw.motor_driver.set_duty_uvw(duty_u, duty_v, duty_w);
                hw.motor_driver.enable_all_channels();

                // キャリブレーション完了チェック
                if self.controller.is_completed() {
                    return self.handle_completion(hw).await;
                }
            }
            Err(_) => {
                error!("Calibration update error, stopping motor");
                hw.motor_driver.stop();
                state::motor_context().await.control_mode = ControlMode::OpenLoop;
                return Some(Transition::OpenLoop { is_recovery: false });
            }
        }

        None
    }

    /// キャリブレーション完了処理
    async fn handle_completion(&mut self, hw: &mut Hardware) -> Option<Transition> {
        let result = self.controller.get_result();

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
            hw.hall_sensor
                .set_electrical_offset(result.electrical_offset);
            hw.hall_sensor
                .set_direction_inversed(result.direction_inversed);

            // キャリブレーション中にモーターは既に回転しているため、直接FOCに移行
            state::motor_context().await.control_mode = ControlMode::ClosedLoopFoc;

            info!("Switching to ClosedLoopFoc mode");

            // キャリブレーションのトルク値からVq初期値を計算
            let calib_torque = state::calibration_context().await.torque as f32 / 100.0;
            let initial_vq = calib_torque * voltage::DEFAULT_MAX;

            // 目標速度から方向を判断
            let target_speed = state::get_foc_input_params().await.target_speed;
            let is_reverse = target_speed < 0.0;

            // 現在の速度を取得
            let period = crate::board::get_period_cycles();
            let actual_rpm = crate::board::calculate_speed_rpm(period, motor::DEFAULT_POLE_PAIRS);
            let current_rpm = if is_reverse {
                -actual_rpm.abs()
            } else {
                actual_rpm.abs()
            };

            Some(Transition::Foc {
                initial_vq,
                current_rpm,
                is_reverse,
            })
        } else {
            error!("Calibration failed!");
            hw.motor_driver.stop();
            state::motor_context().await.control_mode = ControlMode::OpenLoop;
            Some(Transition::OpenLoop { is_recovery: false })
        }
    }
}
