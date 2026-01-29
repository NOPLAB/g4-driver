//! モーター制御タスク
//!
//! 10kHz FOCループ + オープンループ始動制御を実行します。
//! 各制御モードは独立したモジュールに分離されています。

mod calibration_mode;
mod foc_mode;
mod mode;
mod openloop_mode;
mod resources;

use embassy_stm32::{peripherals, timer::complementary_pwm::ComplementaryPwm};
use embassy_time::{Duration, Timer};

use crate::config::*;
use crate::fmt::*;
use crate::foc::ControlMode;
use crate::motor_driver::MotorDriver;
use crate::state;

use mode::TransitionResult;
use resources::ControllerResources;

/// モーター制御タスク（10kHz FOC制御ループ）
#[embassy_executor::task]
pub async fn motor_control_task(uvw_pwm: ComplementaryPwm<'static, peripherals::TIM1>) {
    // 電源投入後、ハードウェア安定待ち
    Timer::after(Duration::from_millis(500)).await;

    info!("Motor control task started (OpenLoop + FOC mode)");

    // モータードライバー初期化
    let mut motor_driver = MotorDriver::new(uvw_pwm);

    // 制御リソース初期化
    let mut resources = ControllerResources::new(motor_driver.max_duty());

    // 制御モード
    let mut control_mode = ControlMode::OpenLoop;

    // 制御周期
    let dt = DEFAULT_CONTROL_PERIOD_US as f32 / 1_000_000.0; // 秒に変換

    info!(
        "FOC parameters: Pole pairs={}, Control freq={}Hz, dt={}s",
        DEFAULT_POLE_PAIRS,
        1_000_000 / DEFAULT_CONTROL_PERIOD_US,
        dt
    );
    info!(
        "PWM configuration: Frequency={}Hz, Max duty={}",
        pwm::DEFAULT_FREQUENCY.0,
        motor_driver.max_duty()
    );

    // モーター有効状態の追跡（PWMチャネル制御用）
    let mut was_enabled = false;

    loop {
        // 1. モーター使能チェック
        let motor_enabled = state::motor_context().await.enabled;
        if !motor_enabled {
            if was_enabled {
                info!("Motor control loop: Disabling PWM channels");
                was_enabled = false;
            }

            // モーター停止：PWMチャネルを完全無効化
            motor_driver.stop();

            // 全リソースをリセット
            resources.reset_all();
            control_mode = ControlMode::OpenLoop;

            Timer::after(Duration::from_micros(DEFAULT_CONTROL_PERIOD_US)).await;
            continue;
        }

        // モーター有効化時の処理
        if !was_enabled {
            info!("Motor control loop: Starting with OpenLoop mode");
            motor_driver.enable_all_channels();
            was_enabled = true;
        }

        // 2. キャリブレーションリクエストをチェック
        if let Some(torque_f32) = check_calibration_request().await {
            info!("Starting motor calibration...");
            info!("  Pole pairs: {}", DEFAULT_POLE_PAIRS);
            info!("  Torque: {}", torque_f32);

            resources.prepare_for_calibration(torque_f32);
            control_mode = ControlMode::Calibration;
            state::motor_context().await.control_mode = ControlMode::Calibration;
        }

        // 3. 制御モード別処理
        let transition =
            execute_control_mode(control_mode, &mut resources, &mut motor_driver, dt).await;

        // 4. モード遷移処理
        if let Some(next_mode) = transition.next_mode() {
            on_mode_exit(control_mode, &mut resources);
            control_mode = next_mode;
            on_mode_enter(next_mode, &mut resources);
        }

        Timer::after(Duration::from_micros(DEFAULT_CONTROL_PERIOD_US)).await;
    }
}

/// キャリブレーションリクエストをチェック
///
/// # Returns
/// `Some(torque)` - キャリブレーション開始時のトルク値（0.0-1.0）
/// `None` - キャリブレーションリクエストなし
async fn check_calibration_request() -> Option<f32> {
    let mut calib_ctx = state::calibration_context().await;
    if calib_ctx.request {
        info!("Calibration requested, switching to Calibration mode");
        calib_ctx.request = false;
        let torque_f32 = calib_ctx.torque as f32 / 100.0;
        Some(torque_f32)
    } else {
        None
    }
}

/// 制御モード別処理を実行
async fn execute_control_mode(
    mode: ControlMode,
    resources: &mut ControllerResources,
    motor_driver: &mut MotorDriver,
    dt: f32,
) -> TransitionResult {
    match mode {
        ControlMode::OpenLoop => {
            let (should_switch, _hall_state) = openloop_mode::execute(
                &mut resources.openloop,
                &resources.hall_sensor,
                motor_driver,
                dt,
            )
            .await;

            if should_switch {
                TransitionResult::TransitionTo(ControlMode::ClosedLoopFoc)
            } else {
                TransitionResult::Continue
            }
        }

        ControlMode::ClosedLoopFoc => {
            let result = foc_mode::execute(
                &mut resources.hall_sensor,
                &mut resources.speed_pi,
                motor_driver,
                &resources.dead_time_comp,
                &mut resources.flux_weakening,
                &mut resources.ramped_target_speed,
                dt,
            )
            .await;

            match result {
                foc_mode::FocResult::Continue | foc_mode::FocResult::InvalidHall => {
                    TransitionResult::Continue
                }
                foc_mode::FocResult::Stalled => {
                    info!("FOC stalled, restarting from OpenLoop mode");
                    TransitionResult::TransitionTo(ControlMode::OpenLoop)
                }
            }
        }

        ControlMode::Calibration => {
            if let Some(next_mode) = calibration_mode::execute(
                &mut resources.calibration,
                &mut resources.hall_sensor,
                motor_driver,
                dt,
            )
            .await
            {
                TransitionResult::TransitionTo(next_mode)
            } else {
                TransitionResult::Continue
            }
        }
    }
}

/// モード退出時の処理
fn on_mode_exit(mode: ControlMode, resources: &mut ControllerResources) {
    match mode {
        ControlMode::OpenLoop => {
            // OpenLoop終了時：現在の速度を保存
            // （FOC移行時に使用）
        }
        ControlMode::ClosedLoopFoc => {
            // FOC終了時：特別な処理なし
        }
        ControlMode::Calibration => {
            // キャリブレーション終了時：特別な処理なし
        }
    }
    let _ = (mode, resources); // 将来の拡張用にパラメータを保持
}

/// モード開始時の処理
fn on_mode_enter(mode: ControlMode, resources: &mut ControllerResources) {
    match mode {
        ControlMode::OpenLoop => {
            resources.prepare_for_openloop();
        }
        ControlMode::ClosedLoopFoc => {
            let current_rpm = resources.openloop.get_current_rpm();
            resources.prepare_for_foc(current_rpm);
            info!("Switching to FOC mode: speed={} RPM", current_rpm);
        }
        ControlMode::Calibration => {
            // calibrationの準備はcheck_calibration_requestで既に実行済み
        }
    }
}
