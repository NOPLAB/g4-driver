//! モーター制御タスク
//!
//! ステートマシンベースのFOC制御を実行します。
//! 各制御モード（OpenLoop、FOC、Calibration）は独立した状態として管理されます。

mod hardware;
mod logging;
mod modes;
mod state_machine;
mod transition;

use embassy_time::{Duration, Timer};

use crate::board::MotorDriver;
use crate::config::{control, motor, pwm};
use crate::fmt::*;
use crate::state;

use hardware::Hardware;
use state_machine::MotorState;
use transition::Transition;

/// モーター制御タスク（10kHz FOC制御ループ）
#[embassy_executor::task]
pub async fn motor_control_task(motor_driver: MotorDriver) {
    // 電源投入後、ハードウェア安定待ち
    Timer::after(Duration::from_millis(500)).await;

    info!("Motor control task started");
    info!(
        "  Control freq={}Hz, Pole pairs={}",
        1_000_000 / control::DEFAULT_PERIOD_US,
        motor::DEFAULT_POLE_PAIRS
    );
    info!(
        "  PWM freq={}Hz, Max duty={}",
        pwm::DEFAULT_FREQUENCY.0,
        motor_driver.max_duty()
    );

    let dt = control::DEFAULT_PERIOD_US as f32 / 1_000_000.0;
    let mut hw = Hardware::new(motor_driver);
    let mut motor_state = MotorState::new(hw.max_duty);
    let mut was_enabled = false;

    loop {
        // 1. モーター有効チェック
        if !state::motor_context().await.enabled {
            if was_enabled {
                info!("Motor disabled");
                was_enabled = false;
            }
            hw.stop();
            motor_state = MotorState::new(hw.max_duty); // 状態リセット
            Timer::after(Duration::from_micros(control::DEFAULT_PERIOD_US)).await;
            continue;
        }

        if !was_enabled {
            info!("Motor enabled, starting with OpenLoop mode");
            hw.motor_driver.enable_all_channels();
            was_enabled = true;
        }

        // 2. キャリブレーションリクエストチェック
        if let Some(trans) = check_calibration_request().await {
            motor_state = trans.apply(&mut hw);
        }

        // 3. 制御実行
        if let Some(trans) = motor_state.update(&mut hw, dt).await {
            motor_state = trans.apply(&mut hw);
        }

        // 4. グローバル状態を更新（CAN送信用）
        state::motor_context().await.control_mode = motor_state.control_mode();

        Timer::after(Duration::from_micros(control::DEFAULT_PERIOD_US)).await;
    }
}

/// キャリブレーションリクエストをチェック
async fn check_calibration_request() -> Option<Transition> {
    let mut ctx = state::calibration_context().await;
    if ctx.request {
        ctx.request = false;
        let torque = ctx.torque as f32 / 100.0;
        info!("Calibration requested");
        info!("  Pole pairs: {}", motor::DEFAULT_POLE_PAIRS);
        info!("  Torque: {}", torque);
        Some(Transition::Calibration { torque })
    } else {
        None
    }
}
