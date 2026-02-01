//! FOC（Field Oriented Control）制御モード
//!
//! Hallセンサーベースのクローズドループ速度制御を実行します。

use bldc::compensation::{DeadTimeCompensation, FluxWeakeningController};
use bldc::control::stall_detector::StallDetectorConfig;
use bldc::control::{FocConfig, FocController};

use crate::board;
use crate::config::{dead_time_compensation, flux_weakening, foc_stall, pwm, speed, voltage};
use crate::fmt::*;
use crate::state::{self, RUNTIME};

use super::super::hardware::Hardware;
use super::super::logging::PeriodicLogger;
use super::super::transition::Transition;

/// PIリセットまでの無効Hall状態の連続回数閾値
/// 10kHz制御で100回 = 10ms以上連続して無効な場合のみリセット
const INVALID_HALL_THRESHOLD: u32 = 100;

/// FOCモードの状態
pub struct FocState {
    controller: FocController,
    invalid_hall_count: u32,
    logger: PeriodicLogger,
}

impl FocState {
    /// 新しいFocStateを作成
    pub fn new(max_duty: u16, initial_vq: f32, current_rpm: f32, _is_reverse: bool) -> Self {
        let mut controller = create_foc_controller(max_duty);
        controller.initialize_for_foc(current_rpm, initial_vq);
        RUNTIME.foc.reset_all();

        Self {
            controller,
            invalid_hall_count: 0,
            logger: PeriodicLogger::one_hz(),
        }
    }

    /// 1制御サイクルの実行
    pub async fn execute(&mut self, hw: &mut Hardware, dt: f32) -> Option<Transition> {
        // 電気角と速度とHall状態を取得
        let (hall_electrical_angle, speed_rpm, hall_state) = hw.hall_sensor.update(dt);
        let is_valid_hall = (1..=6).contains(&hall_state);

        // Hallセンサが無効な場合の処理
        if !is_valid_hall {
            self.invalid_hall_count += 1;

            if self.invalid_hall_count >= INVALID_HALL_THRESHOLD {
                // 長時間無効: PWMを中立に設定し、コントローラをリセット
                hw.motor_driver
                    .set_duty_uvw(hw.max_duty / 2, hw.max_duty / 2, hw.max_duty / 2);
                self.controller.reset();

                if self.invalid_hall_count == INVALID_HALL_THRESHOLD {
                    warn!(
                        "[FOC] Invalid Hall state persisted for {}+ cycles, PWM neutralized",
                        INVALID_HALL_THRESHOLD
                    );
                }
            }
            return None;
        }

        // 有効なHall状態: 無効カウンタをリセット
        self.invalid_hall_count = 0;

        // FOC入力パラメータを一括取得
        let foc_params = state::get_foc_input_params().await;

        // PIゲイン更新チェック
        if foc_params.pi_gains.0 != self.controller.get_kp()
            || foc_params.pi_gains.1 != self.controller.get_ki()
        {
            self.controller
                .set_gains(foc_params.pi_gains.0, foc_params.pi_gains.1);
            info!(
                "PI gains updated: Kp={}, Ki={}",
                foc_params.pi_gains.0, foc_params.pi_gains.1
            );
        }

        // 目標速度設定
        self.controller
            .set_target_speed_rpm(foc_params.target_speed);

        // 速度ランプが実速度を下回らないように調整（加速中の逆トルク防止）
        // 正方向加速中（ランプ < 実速度）または負方向加速中（ランプ > 実速度）の場合、
        // ランプを実速度に追従させてPI制御が減速方向のトルクを出すのを防ぐ
        if let Some(ramp) = self.controller.speed_ramp_mut() {
            let target = foc_params.target_speed;
            let current_ramp = ramp.get_current_speed();

            let should_catch_up = (target > 0.0 && current_ramp >= 0.0 && current_ramp < speed_rpm)
                || (target < 0.0 && current_ramp <= 0.0 && current_ramp > speed_rpm);

            if should_catch_up {
                ramp.set_current_speed(speed_rpm);
            }
        }

        // FOCコントローラ更新
        let output = self
            .controller
            .update_extended(speed_rpm, hall_electrical_angle, dt);

        // PWM出力
        hw.motor_driver
            .set_duty_uvw(output.duty.u, output.duty.v, output.duty.w);
        hw.motor_driver.enable_all_channels();

        // ステータス更新
        RUNTIME.status.update(speed_rpm, hall_electrical_angle);

        // 脱調検出による遷移
        if output.is_stalled {
            warn!(
                "[FOC] Stall detected: speed={} RPM, switching to OpenLoop",
                speed_rpm.abs()
            );
            return Some(Transition::OpenLoop { is_recovery: true });
        }

        // デバッグログ（1秒ごと）
        if self.logger.tick() {
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

        None
    }
}

/// FOCコントローラを作成
fn create_foc_controller(max_duty: u16) -> FocController {
    FocController::builder(FocConfig {
        speed_kp: speed::DEFAULT_KP,
        speed_ki: speed::DEFAULT_KI,
        max_voltage: voltage::DEFAULT_MAX,
        v_dc: voltage::DEFAULT_DC_BUS,
        max_duty,
        vd: 0.0,
        max_acceleration: speed::MAX_ACCELERATION,
        min_voltage: voltage::MIN,
        min_voltage_error_threshold: voltage::MIN_ERROR_THRESHOLD,
        pi_integral_limit: speed::PI_INTEGRAL_LIMIT,
        anti_windup_enabled: true,
    })
    .with_dead_time_compensation(create_dead_time_compensation(max_duty))
    .with_flux_weakening(create_flux_weakening_controller())
    .with_stall_detection(StallDetectorConfig {
        speed_threshold: foc_stall::SPEED_THRESHOLD,
        count_threshold: foc_stall::COUNT_THRESHOLD,
    })
    .build()
}

/// デッドタイム補償器を初期化
fn create_dead_time_compensation(max_duty: u16) -> DeadTimeCompensation {
    let mut comp = DeadTimeCompensation::new(
        dead_time_compensation::DEAD_TIME_NS,
        pwm::DEFAULT_FREQUENCY.0,
        voltage::DEFAULT_DC_BUS,
        max_duty,
    );
    comp.set_enabled(dead_time_compensation::ENABLED);
    comp
}

/// フラックス弱め制御器を初期化
fn create_flux_weakening_controller() -> FluxWeakeningController {
    let mut fw = FluxWeakeningController::new(
        flux_weakening::MIN_SPEED,
        flux_weakening::MAX_SPEED,
        flux_weakening::MAX_WEAKENING_RATIO,
        flux_weakening::VD_RATE_LIMIT,
    );
    fw.set_enabled(flux_weakening::ENABLED);
    fw
}
