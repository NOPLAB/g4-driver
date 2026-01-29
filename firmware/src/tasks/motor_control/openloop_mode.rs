//! オープンループ制御モード
//!
//! 強制転流で起動し、Hall センサーベースの6ステップ駆動に切り替えます。

use crate::config::motor;
use crate::config::openloop::{
    DEFAULT_DUTY_RATIO, FORCED_COMMUTATION_CYCLES, MIN_CYCLES_BEFORE_FOC, MIN_SPEED_FOR_FOC,
};
use crate::fmt::*;
use crate::state::{ControlMode, RUNTIME};

use crate::hall_tim;

use super::mode::{ModeContext, ModeResult};

/// Hall 状態から6ステップ駆動パターンを取得
fn hall_to_commutation(hall_state: u8, duty: u16) -> (u16, u16, u16) {
    let off = 50; // フローティング
    match hall_state {
        1 => (duty, 0, off),
        3 => (duty, off, 0),
        2 => (off, duty, 0),
        6 => (0, duty, off),
        4 => (0, off, duty),
        5 => (off, 0, duty),
        _ => (50, 50, 50),
    }
}

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
        let exec_count = RUNTIME.openloop.increment_execution();
        let hall_state = hall_tim::get_hall_state();
        let is_valid_hall = (1..=6).contains(&hall_state);
        let pwm_max = ctx.motor_driver.max_duty();
        let is_recovery = RUNTIME.openloop.is_recovery();

        // フェーズ別処理
        let (du, dv, dw, phase) = if exec_count < FORCED_COMMUTATION_CYCLES {
            // 強制転流フェーズ
            let state = openloop.update(ctx.dt);
            let u = (state.duty_u as u32 * pwm_max as u32 / 100) as u16;
            let v = (state.duty_v as u32 * pwm_max as u32 / 100) as u16;
            let w = (state.duty_w as u32 * pwm_max as u32 / 100) as u16;
            ctx.motor_driver
                .set_channels(state.enable_u, state.enable_v, state.enable_w);
            (u, v, w, "Forced")
        } else {
            // Hall駆動フェーズ
            let (u, v, w) = hall_to_commutation(hall_state, DEFAULT_DUTY_RATIO);
            let u = (u as u32 * pwm_max as u32 / 100) as u16;
            let v = (v as u32 * pwm_max as u32 / 100) as u16;
            let w = (w as u32 * pwm_max as u32 / 100) as u16;
            ctx.motor_driver.set_channels(true, true, true);
            (u, v, w, "Hall")
        };

        ctx.motor_driver.set_duty_uvw(du, dv, dw);
        RUNTIME.status.update(openloop.get_current_rpm(), 0.0);

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
            info!("[OpenLoop] Switching to Hall-based commutation");
        }

        // FOC切り替え判定
        let time_ok = exec_count >= MIN_CYCLES_BEFORE_FOC;
        let speed_ok = speed >= MIN_SPEED_FOR_FOC;

        let ready = if is_recovery {
            // 回復時: 速度チェックも必要
            time_ok && is_valid_hall && speed_ok
        } else {
            // 通常起動: 従来通り
            time_ok && is_valid_hall
        };

        // 判定ログ（初回のみ）
        if exec_count == MIN_CYCLES_BEFORE_FOC {
            if is_recovery && !speed_ok {
                info!(
                    "[Recovery] Waiting for speed: {} < {} RPM",
                    speed, MIN_SPEED_FOR_FOC
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
