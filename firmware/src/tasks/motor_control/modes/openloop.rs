//! オープンループ制御モード
//!
//! SVPWMベースの強制転流で起動し、Hallセンサーベースの駆動に移行してFOCへ接続します。

use bldc::control::{OpenLoopConfig, OpenLoopController};

use crate::board;
use crate::config::{motor, openloop, voltage};
use crate::fmt::*;
use crate::state::{self, RUNTIME};

use super::super::hardware::Hardware;
use super::super::logging::PeriodicLogger;
use super::super::transition::Transition;

/// OpenLoopモードの状態
pub struct OpenLoopState {
    controller: OpenLoopController,
    is_recovery: bool,
    logger: PeriodicLogger,
}

impl OpenLoopState {
    /// 新しいOpenLoopStateを作成
    pub fn new(max_duty: u16, is_recovery: bool) -> Self {
        let mut controller = OpenLoopController::new(OpenLoopConfig {
            initial_rpm: openloop::DEFAULT_INITIAL_RPM,
            target_rpm: openloop::DEFAULT_TARGET_RPM,
            acceleration: openloop::DEFAULT_ACCELERATION,
            voltage_ratio: openloop::DEFAULT_DUTY_RATIO as f32 / 100.0,
            v_dc: voltage::DEFAULT_DC_BUS,
            max_duty,
            pole_pairs: motor::DEFAULT_POLE_PAIRS,
            forced_commutation_cycles: openloop::FORCED_COMMUTATION_CYCLES,
            min_cycles_for_foc: openloop::MIN_CYCLES_BEFORE_FOC,
            min_speed_for_foc: openloop::MIN_SPEED_FOR_FOC,
        });

        if is_recovery {
            controller.reset_for_recovery();
            RUNTIME.openloop.reset_for_recovery();
        } else {
            controller.reset_for_normal();
            RUNTIME.openloop.reset_for_normal();
        }

        Self {
            controller,
            is_recovery,
            logger: PeriodicLogger::one_hz(),
        }
    }

    /// 1制御サイクルの実行
    pub async fn execute(&mut self, hw: &mut Hardware, dt: f32) -> Option<Transition> {
        // Hall状態を取得
        let hall_state = board::get_hall_state();
        let is_valid_hall = (1..=6).contains(&hall_state);

        // 目標速度から回転方向を決定
        let foc_params = state::get_foc_input_params().await;
        let is_reverse = foc_params.target_speed < 0.0;
        self.controller.set_reverse(is_reverse);
        RUNTIME.openloop.set_reverse(is_reverse);

        // Hall駆動フェーズ用の電気角と速度を取得
        let (hall_electrical_angle, speed_rpm, _hall) = hw.hall_sensor.update(dt);

        // OpenLoopコントローラ更新（速度は絶対値を使用）
        let hall_speed_rpm = speed_rpm.abs();
        let output = self.controller.update(
            Some(hall_electrical_angle),
            hall_speed_rpm,
            is_valid_hall,
            dt,
        );

        // PWM出力
        hw.motor_driver
            .set_duty_uvw(output.duty.u, output.duty.v, output.duty.w);
        hw.motor_driver.set_channels(true, true, true);

        // ステータス更新
        RUNTIME.status.update(output.current_rpm, 0.0);

        // ログ（1秒ごと）
        if self.logger.tick() {
            let mode = if self.is_recovery { "(R)" } else { "" };
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
                self.controller.get_execution_count()
            );
        }

        // フェーズ切り替えログ
        let exec_count = self.controller.get_execution_count();
        if exec_count == openloop::FORCED_COMMUTATION_CYCLES
            && openloop::FORCED_COMMUTATION_CYCLES > 0
        {
            info!("[OpenLoop] Switching to Hall-based SVPWM commutation");
        }

        // FOC切り替え判定ログ（初回のみ）
        if exec_count == openloop::MIN_CYCLES_BEFORE_FOC && !output.ready_for_foc {
            let mode = if self.is_recovery { "(R)" } else { "" };
            info!(
                "[OpenLoop{}] Waiting for conditions: speed={} RPM, valid_hall={}",
                mode, hall_speed_rpm, is_valid_hall
            );
        }

        // FOC遷移
        if output.ready_for_foc {
            info!("[OpenLoop] Ready for FOC, speed={} RPM", hall_speed_rpm);
            let initial_vq =
                (openloop::DEFAULT_DUTY_RATIO as f32 / 100.0) * voltage::DEFAULT_DC_BUS;
            Some(Transition::Foc {
                initial_vq,
                current_rpm: if is_reverse {
                    -hall_speed_rpm
                } else {
                    hall_speed_rpm
                },
                is_reverse,
            })
        } else {
            None
        }
    }
}
