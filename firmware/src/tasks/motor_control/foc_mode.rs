//! FOC（Field Oriented Control）制御モード
//!
//! Hallセンサーベースのクローズドループ速度制御を実行します。
//! bldcクレートのFocControllerを使用して制御パイプラインを簡素化。

use core::sync::atomic::{AtomicU32, Ordering};

use crate::config::motor;
use crate::fmt::*;
use crate::state::{ControlMode, RUNTIME};

use crate::board;
use crate::state;

use super::mode::{ModeContext, ModeResult};

/// FOCモードログカウンタ（1Hz = 10000サイクルごと @ 10kHz）
static FOC_MODE_LOG_COUNTER: AtomicU32 = AtomicU32::new(0);

/// PIリセットまでの無効Hall状態の連続回数閾値
/// 10kHz制御で100回 = 10ms以上連続して無効な場合のみリセット
const INVALID_HALL_THRESHOLD: u32 = 100;

/// FOCモードのハンドラ
pub struct FocMode;

impl FocMode {
    /// モード固有の名前
    #[allow(dead_code)]
    pub fn name(&self) -> &'static str {
        "ClosedLoopFoc"
    }

    /// モード開始時の初期化
    pub fn on_enter(&self, ctx: &mut ModeContext<'_>, _prev_mode: ControlMode) {
        // 実際のHall速度を取得
        let period = board::get_period_cycles();
        let actual_rpm = board::calculate_speed_rpm(period, motor::DEFAULT_POLE_PAIRS);

        // 実測値が有効な場合はそれを使用、そうでなければOpenLoopの理論値を使用
        let is_reverse = ctx.resources.openloop_controller.is_reverse();
        let theoretical_rpm = ctx.resources.openloop_controller.get_theoretical_rpm();

        let current_rpm = if actual_rpm > 50.0 {
            if is_reverse {
                -actual_rpm
            } else {
                actual_rpm
            }
        } else {
            theoretical_rpm
        };

        ctx.resources.prepare_for_foc(current_rpm);
        info!("Switching to FOC mode: speed={} RPM", current_rpm);
    }

    /// モード終了時のクリーンアップ
    pub fn on_exit(&self, _ctx: &mut ModeContext<'_>) {
        // FOC終了時：特別な処理なし
    }

    /// 1制御サイクルの実行
    pub async fn execute(&self, ctx: &mut ModeContext<'_>) -> ModeResult {
        let hall_sensor = &mut ctx.resources.hall_sensor;
        let foc_controller = &mut ctx.resources.foc_controller;
        let motor_driver = &mut ctx.motor_driver;
        let dt = ctx.dt;

        // 電気角と速度とHall状態を取得（TIM4ハードウェアベース）
        let (hall_electrical_angle, speed_rpm_abs, hall_state) = hall_sensor.update(dt);
        let is_valid_hall = (1..=6).contains(&hall_state);

        // Hallセンサが無効な場合の処理
        if !is_valid_hall {
            let invalid_count = RUNTIME.foc.increment_invalid_hall();

            if invalid_count >= INVALID_HALL_THRESHOLD {
                // 長時間無効: PWMを中立に設定し、コントローラをリセット
                motor_driver.set_duty_uvw(
                    motor_driver.max_duty() / 2,
                    motor_driver.max_duty() / 2,
                    motor_driver.max_duty() / 2,
                );
                foc_controller.reset();

                if invalid_count == INVALID_HALL_THRESHOLD {
                    warn!(
                        "[FOC] Invalid Hall state persisted for {}+ cycles, PWM neutralized",
                        INVALID_HALL_THRESHOLD
                    );
                }
            }
            return ModeResult::Continue;
        }

        // 有効なHall状態: 無効カウンタをリセット
        RUNTIME.foc.reset_invalid_hall();

        // FOC入力パラメータを一括取得
        let foc_params = state::get_foc_input_params().await;

        // PIゲイン更新チェック
        if foc_params.pi_gains.0 != foc_controller.get_kp()
            || foc_params.pi_gains.1 != foc_controller.get_ki()
        {
            foc_controller.set_gains(foc_params.pi_gains.0, foc_params.pi_gains.1);
            info!(
                "PI gains updated: Kp={}, Ki={}",
                foc_params.pi_gains.0, foc_params.pi_gains.1
            );
        }

        // 目標速度設定
        foc_controller.set_target_speed_rpm(foc_params.target_speed);

        // 逆転対応：目標速度の符号に基づいてフィードバック速度の符号を推定
        let ramped_target = foc_controller
            .speed_ramp()
            .map_or(foc_params.target_speed, |r| r.get_current_speed());
        let speed_rpm = if ramped_target < 0.0 {
            -speed_rpm_abs
        } else {
            speed_rpm_abs
        };

        // FOCコントローラ更新（全パイプラインを実行）
        let output = foc_controller.update_extended(speed_rpm, hall_electrical_angle, dt);

        // PWM出力
        motor_driver.set_duty_uvw(output.duty.u, output.duty.v, output.duty.w);
        motor_driver.enable_all_channels();

        // ステータス更新
        RUNTIME.status.update(speed_rpm, hall_electrical_angle);

        // 脱調検出による遷移
        if output.is_stalled {
            warn!(
                "[FOC] Stall detected: speed={} RPM, switching to OpenLoop",
                speed_rpm.abs()
            );
            return ModeResult::TransitionTo(ControlMode::OpenLoop);
        }

        // デバッグログ（1秒ごと）
        let mode_count = FOC_MODE_LOG_COUNTER.fetch_add(1, Ordering::Relaxed);
        if mode_count >= 10000 {
            FOC_MODE_LOG_COUNTER.store(0, Ordering::Relaxed);

            let period_cycles = board::get_period_cycles();
            debug!(
                "[FOC] Speed: {}/{} RPM (ramped: {}), Vq={}V, Hall: {}, Period: {} cycles",
                speed_rpm,
                foc_params.target_speed,
                output.ramped_target_speed,
                output.vq,
                hall_state,
                period_cycles
            );
        }

        ModeResult::Continue
    }
}

/// シングルトンインスタンス
pub static FOC_MODE: FocMode = FocMode;
