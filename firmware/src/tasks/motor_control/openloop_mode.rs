//! オープンループ制御モード
//!
//! 強制転流で起動し、SVPWMベースのHall駆動に切り替えます。
//! FOCへの滑らかな移行を実現するため、Hall駆動フェーズでもSVPWMを使用します。

use crate::config::motor;
use crate::config::openloop::{
    DEFAULT_DUTY_RATIO, FORCED_COMMUTATION_CYCLES, MIN_CYCLES_BEFORE_FOC, MIN_SPEED_FOR_FOC,
};
use crate::config::voltage;
use crate::fmt::*;
use crate::state::{self, ControlMode, RUNTIME};

use crate::hall_tim;

// SVPWMとPark逆変換を使用
use bldc::modulation::calculate_svpwm;
use bldc::transforms::inverse_park;

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
        if prev_mode == ControlMode::ClosedLoopFoc {
            info!("Entering OpenLoop recovery mode (from FOC stall)");
            RUNTIME.openloop.reset_for_recovery();
            ctx.resources.prepare_for_openloop_or_recovery(true);
        } else {
            RUNTIME.openloop.reset_for_normal();
            ctx.resources.prepare_for_openloop_or_recovery(false);
        }
    }

    /// モード終了時のクリーンアップ
    pub fn on_exit(&self, _ctx: &mut ModeContext<'_>) {
        // OpenLoop終了時：特別な処理なし
    }

    /// 1制御サイクルの実行
    pub async fn execute(&self, ctx: &mut ModeContext<'_>) -> ModeResult {
        let openloop = &mut ctx.resources.openloop;
        let hall_sensor = &mut ctx.resources.hall_sensor;
        let exec_count = RUNTIME.openloop.increment_execution();
        let hall_state = hall_tim::get_hall_state();
        let is_valid_hall = (1..=6).contains(&hall_state);
        let pwm_max = ctx.motor_driver.max_duty();
        let is_recovery = RUNTIME.openloop.is_recovery();

        // 目標速度を取得して回転方向を決定
        let foc_params = state::get_foc_input_params().await;
        let is_reverse = foc_params.target_speed < 0.0;
        openloop.set_reverse(is_reverse);

        // フェーズ別処理
        let (du, dv, dw, phase) = if exec_count < FORCED_COMMUTATION_CYCLES {
            // 強制転流フェーズ（従来の6ステップ駆動）
            let state = openloop.update(ctx.dt);
            let u = (state.duty_u as u32 * pwm_max as u32 / 100) as u16;
            let v = (state.duty_v as u32 * pwm_max as u32 / 100) as u16;
            let w = (state.duty_w as u32 * pwm_max as u32 / 100) as u16;
            ctx.motor_driver
                .set_channels(state.enable_u, state.enable_v, state.enable_w);
            (u, v, w, "Forced")
        } else {
            // Hall駆動フェーズ（SVPWMベース - FOCと同じ駆動方式）
            // Hall センサーから電気角を取得
            let (electrical_angle, _speed_rpm, _hall) = hall_sensor.update(ctx.dt);

            // 固定電圧指令（OpenLoopのDuty相当）
            // 逆回転時は負のVqを使用
            let vq_base = (DEFAULT_DUTY_RATIO as f32 / 100.0) * voltage::DEFAULT_DC_BUS;
            let vq_cmd = if is_reverse { -vq_base } else { vq_base };
            let vd_cmd = 0.0;

            // Park逆変換 → SVPWM（FOCと同じ計算）
            let (v_alpha, v_beta) = inverse_park(vd_cmd, vq_cmd, electrical_angle);
            let (u, v, w) = calculate_svpwm(v_alpha, v_beta, voltage::DEFAULT_DC_BUS, pwm_max);

            ctx.motor_driver.set_channels(true, true, true);
            (u, v, w, "SVPWM")
        };

        ctx.motor_driver.set_duty_uvw(du, dv, dw);

        // ステータス更新（逆回転時は負の速度として報告）
        let current_rpm = openloop.get_current_rpm();
        let signed_rpm = if is_reverse {
            -current_rpm
        } else {
            current_rpm
        };
        RUNTIME.status.update(signed_rpm, 0.0);

        // 速度計算
        let period = hall_tim::get_period_cycles();
        let speed = hall_tim::calculate_speed_rpm(period, motor::DEFAULT_POLE_PAIRS);

        // ログ（1秒ごと）
        let log_count = RUNTIME.openloop.increment_log();
        if log_count >= 10000 {
            RUNTIME.openloop.reset_log();
            let mode = if is_recovery { "(R)" } else { "" };
            info!(
                "[{}{}] Hall:{}, Speed:{} RPM, Duty:{}/{}/{}, Cycle:{}",
                phase, mode, hall_state, speed, du, dv, dw, exec_count
            );
        }

        // フェーズ切り替えログ
        if exec_count == FORCED_COMMUTATION_CYCLES {
            info!("[OpenLoop] Switching to SVPWM-based Hall commutation");
        }

        // FOC切り替え判定
        // 低速域が不安定なモーター向けに、通常起動時も速度チェックを実施
        let time_ok = exec_count >= MIN_CYCLES_BEFORE_FOC;
        let speed_ok = speed >= MIN_SPEED_FOR_FOC;

        let ready = time_ok && is_valid_hall && speed_ok;

        // 判定ログ（初回のみ）
        if exec_count == MIN_CYCLES_BEFORE_FOC {
            if !speed_ok {
                let mode = if is_recovery { "(R)" } else { "" };
                info!(
                    "[OpenLoop{}] Waiting for speed: {} < {} RPM",
                    mode, speed, MIN_SPEED_FOR_FOC
                );
            } else {
                info!("[OpenLoop] Ready for FOC, speed={} RPM", speed);
            }
        }

        if ready {
            ModeResult::TransitionTo(ControlMode::ClosedLoopFoc)
        } else {
            ModeResult::Continue
        }
    }
}

/// シングルトンインスタンス
pub static OPENLOOP_MODE: OpenLoopMode = OpenLoopMode;
