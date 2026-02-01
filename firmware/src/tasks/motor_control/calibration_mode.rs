//! キャリブレーション制御モード
//!
//! モーターの電気角オフセットと回転方向を自動検出します。

use core::f32::consts::PI;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::adapters::HallStateReaderAdapter;
use crate::config::voltage;
use crate::fmt::*;
// Use bldc crate for portable algorithms
use bldc::modulation::calculate_svpwm;
use bldc::transforms::inverse_park;

use crate::state;
use crate::state::{ControlMode, RUNTIME};

use super::mode::{ModeContext, ModeResult};

/// キャリブレーションデバッグログカウンタ（1Hz = 10000サイクルごと @ 10kHz）
static DEBUG_COUNTER: AtomicU32 = AtomicU32::new(0);

/// CalibrationModeのハンドラ
pub struct CalibrationMode;

impl CalibrationMode {
    /// モード固有の名前
    #[allow(dead_code)]
    pub fn name(&self) -> &'static str {
        "Calibration"
    }

    /// モード開始時の初期化
    pub fn on_enter(&self, ctx: &mut ModeContext<'_>, _prev_mode: ControlMode) {
        // calibrationの準備（トルク設定と開始）はcheck_calibration_requestで既に実行済み
        DEBUG_COUNTER.store(0, Ordering::Relaxed);
        let _ = ctx; // 将来の拡張用
    }

    /// モード終了時のクリーンアップ
    pub fn on_exit(&self, _ctx: &mut ModeContext<'_>) {
        // キャリブレーション終了時：特別な処理なし
    }

    /// 1制御サイクルの実行
    pub async fn execute(&self, ctx: &mut ModeContext<'_>) -> ModeResult {
        execute_calibration_internal(ctx).await
    }
}

/// シングルトンインスタンス
pub static CALIBRATION_MODE: CalibrationMode = CalibrationMode;

/// キャリブレーション制御の内部実装
async fn execute_calibration_internal(ctx: &mut ModeContext<'_>) -> ModeResult {
    let calibration = &mut ctx.resources.calibration;
    let hall_sensor = &mut ctx.resources.hall_sensor;
    let motor_driver = &mut ctx.motor_driver;
    let dt = ctx.dt;

    // Hall センサーを更新して現在の角度を取得
    let (electrical_angle_raw, speed_rpm, _hall_state) = hall_sensor.update(dt);
    let sensor_angle = hall_sensor.get_mechanical_angle();

    // ステータス更新（CAN送信用）
    RUNTIME.status.update(speed_rpm, electrical_angle_raw);

    // デバッグ：Hall状態と角度を定期的にログ出力（10000サイクルごと = 1秒 @ 10kHz）
    let count = DEBUG_COUNTER.fetch_add(1, Ordering::Relaxed);
    if count >= 10000 {
        DEBUG_COUNTER.store(0, Ordering::Relaxed);
        let hall_state = crate::board::get_hall_state();
        info!(
            "[Calibration Execute] Hall state={}, sensor_angle={} rad ({} deg)",
            hall_state,
            sensor_angle,
            sensor_angle * 180.0 / PI
        );
    }

    // キャリブレーションステートマシンを更新（HallStateReaderアダプターを使用）
    let hall_reader = HallStateReaderAdapter;
    match calibration.update(sensor_angle, &hall_reader) {
        Ok((electrical_angle, torque)) => {
            // トルクから電圧指令を計算（トルク 0.0～1.0 → 電圧 0～MAX_VOLTAGE）
            let v_cmd = torque * voltage::DEFAULT_MAX;

            // d軸・q軸電圧（キャリブレーション中はシンプルにq軸のみ）
            let vd_cmd = 0.0;
            let vq_cmd = v_cmd;

            // Park逆変換
            let (v_alpha, v_beta) = inverse_park(vd_cmd, vq_cmd, electrical_angle);

            // SVPWM計算（実際のPWM最大値を使用）
            let pwm_max_duty = motor_driver.max_duty();
            let (duty_u, duty_v, duty_w) =
                calculate_svpwm(v_alpha, v_beta, voltage::DEFAULT_DC_BUS, pwm_max_duty);

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
                    hall_sensor.set_direction_inversed(result.direction_inversed);

                    // OpenLoopモードに切り替え（低速域が不安定なモーター向け）
                    // OpenLoopで加速してからFOCに移行する
                    state::motor_context().await.control_mode = ControlMode::OpenLoop;

                    info!("Switching to OpenLoop mode (will transition to FOC after acceleration)");
                    return ModeResult::TransitionTo(ControlMode::OpenLoop);
                } else {
                    error!("Calibration failed!");
                    // エラー時はモーターを停止
                    motor_driver.stop();

                    // OpenLoopモードに戻る
                    state::motor_context().await.control_mode = ControlMode::OpenLoop;

                    return ModeResult::TransitionTo(ControlMode::OpenLoop);
                }
            }
        }
        Err(_) => {
            error!("Calibration update error, stopping motor");
            // エラー時はモーターを停止
            motor_driver.stop();

            // OpenLoopモードに戻る
            state::motor_context().await.control_mode = ControlMode::OpenLoop;

            return ModeResult::TransitionTo(ControlMode::OpenLoop);
        }
    }

    ModeResult::Continue
}
