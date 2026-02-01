//! オープンループ制御モード
//!
//! SVPWMベースの強制転流で起動し、Hallセンサーベースの駆動に移行してFOCへ接続します。
//! bldcクレートのOpenLoopControllerを使用して制御を簡素化。

use crate::config::motor;
use crate::fmt::*;
use crate::state::{self, ControlMode, RUNTIME};

use crate::hall_tim;

use super::mode::{ModeContext, ModeResult};

/// OpenLoopモードのハンドラ
pub struct OpenLoopMode;

impl OpenLoopMode {
    /// モード固有の名前
    #[allow(dead_code)]
    pub fn name(&self) -> &'static str {
        "OpenLoop"
    }

    /// モード開始時の初期化
    pub fn on_enter(&self, ctx: &mut ModeContext<'_>, prev_mode: ControlMode) {
        // FOCから戻ってきた場合は脱調回復モード
        let is_recovery = prev_mode == ControlMode::ClosedLoopFoc;
        if is_recovery {
            info!("Entering OpenLoop recovery mode (from FOC stall)");
        }
        ctx.resources.prepare_for_openloop_or_recovery(is_recovery);
    }

    /// モード終了時のクリーンアップ
    pub fn on_exit(&self, _ctx: &mut ModeContext<'_>) {
        // OpenLoop終了時：特別な処理なし
    }

    /// 1制御サイクルの実行
    pub async fn execute(&self, ctx: &mut ModeContext<'_>) -> ModeResult {
        let hall_sensor = &mut ctx.resources.hall_sensor;
        let openloop_controller = &mut ctx.resources.openloop_controller;
        let motor_driver = &mut ctx.motor_driver;
        let dt = ctx.dt;

        // Hall状態を取得
        let hall_state = hall_tim::get_hall_state();
        let is_valid_hall = (1..=6).contains(&hall_state);

        // 目標速度から回転方向を決定
        let foc_params = state::get_foc_input_params().await;
        let is_reverse = foc_params.target_speed < 0.0;
        openloop_controller.set_reverse(is_reverse);
        RUNTIME.openloop.set_reverse(is_reverse);

        // Hall駆動フェーズ用の電気角を取得
        let (hall_electrical_angle, _speed_rpm, _hall) = hall_sensor.update(dt);

        // OpenLoopコントローラ更新
        let hall_speed_rpm =
            hall_tim::calculate_speed_rpm(hall_tim::get_period_cycles(), motor::DEFAULT_POLE_PAIRS);
        let output = openloop_controller.update(
            Some(hall_electrical_angle),
            hall_speed_rpm,
            is_valid_hall,
            dt,
        );

        // PWM出力
        motor_driver.set_duty_uvw(output.duty.u, output.duty.v, output.duty.w);
        motor_driver.set_channels(true, true, true);

        // ステータス更新
        RUNTIME.status.update(output.current_rpm, 0.0);

        // ログ（1秒ごと）
        let log_count = RUNTIME.openloop.increment_log();
        if log_count >= 10000 {
            RUNTIME.openloop.reset_log();

            let mode = if openloop_controller.is_recovery() {
                "(R)"
            } else {
                ""
            };
            let dir = if is_reverse { " REV" } else { "" };
            let phase = match output.phase {
                bldc::OpenLoopPhase::ForcedCommutation => "Forced",
                bldc::OpenLoopPhase::HallDriven => "SVPWM",
            };

            info!(
                "[{}{}{}] Hall:{}, Speed:{} RPM, Duty:{}/{}/{}, Cycle:{}",
                phase,
                mode,
                dir,
                hall_state,
                hall_speed_rpm,
                output.duty.u,
                output.duty.v,
                output.duty.w,
                openloop_controller.get_execution_count()
            );
        }

        // フェーズ切り替えログ
        let exec_count = openloop_controller.get_execution_count();
        if exec_count == crate::config::openloop::FORCED_COMMUTATION_CYCLES
            && crate::config::openloop::FORCED_COMMUTATION_CYCLES > 0
        {
            info!("[OpenLoop] Switching to Hall-based SVPWM commutation");
        }

        // FOC切り替え判定ログ（初回のみ）
        if exec_count == crate::config::openloop::MIN_CYCLES_BEFORE_FOC {
            if !output.ready_for_foc {
                let mode = if openloop_controller.is_recovery() {
                    "(R)"
                } else {
                    ""
                };
                info!(
                    "[OpenLoop{}] Waiting for conditions: speed={} RPM, valid_hall={}",
                    mode, hall_speed_rpm, is_valid_hall
                );
            } else {
                info!("[OpenLoop] Ready for FOC, speed={} RPM", hall_speed_rpm);
            }
        }

        // FOC遷移
        if output.ready_for_foc {
            ModeResult::TransitionTo(ControlMode::ClosedLoopFoc)
        } else {
            ModeResult::Continue
        }
    }
}

/// シングルトンインスタンス
pub static OPENLOOP_MODE: OpenLoopMode = OpenLoopMode;
