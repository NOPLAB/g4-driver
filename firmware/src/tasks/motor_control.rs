//! モーター制御タスク
//!
//! 2.5kHz FOCループ + オープンループ始動制御を実行します。
//! 各制御モードは独立したモジュールに分離されています。

mod calibration_mode;
mod foc_mode;
mod openloop_mode;

use embassy_stm32::{peripherals, timer::complementary_pwm::ComplementaryPwm};
use embassy_time::{Duration, Timer};

use crate::config::*;
use crate::fmt::*;
use crate::foc::{ControlMode, HallSensor, MotorCalibration, OpenLoopSixStep, PiController};
use crate::hall_tim;
use crate::motor_driver::MotorDriver;
use crate::state::{CALIBRATION_REQUEST, CALIBRATION_TORQUE, CONTROL_MODE, MOTOR_ENABLE};
use core::f32::consts::PI;

/// モーター制御タスク（2.5kHz FOC制御ループ）
#[embassy_executor::task]
pub async fn motor_control_task(uvw_pwm: ComplementaryPwm<'static, peripherals::TIM1>) {
    info!("Motor control task started (OpenLoop + FOC mode)");

    // モータードライバー初期化
    let mut motor_driver = MotorDriver::new(uvw_pwm);

    // ホールセンサ初期化（foc-simple互換の機械角ベース計算）
    let mut hall_sensor = HallSensor::new(DEFAULT_POLE_PAIRS, DEFAULT_SPEED_FILTER_ALPHA);
    hall_sensor.set_interpolation(true); // 角度補間を有効化（FOC制御の安定性向上）

    // 電気オフセットを設定（キャリブレーション値）
    let offset_rad = DEFAULT_HALL_ANGLE_OFFSET_DEG * PI / 180.0;
    hall_sensor.set_electrical_offset(offset_rad);

    // 速度PIコントローラ初期化
    let mut speed_pi =
        PiController::new_symmetric(DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, DEFAULT_MAX_VOLTAGE);

    // オープンループ始動コントローラ初期化
    let mut openloop = OpenLoopSixStep::new(
        openloop::DEFAULT_INITIAL_RPM,
        openloop::DEFAULT_TARGET_RPM,
        openloop::DEFAULT_ACCELERATION_RPM_PER_S,
        openloop::DEFAULT_DUTY_RATIO,
        DEFAULT_POLE_PAIRS,
    );

    // キャリブレーション初期化（トルク0.1 = 10%、電力消費を抑える）
    let mut calibration = MotorCalibration::new(DEFAULT_POLE_PAIRS, 0.1);

    // 制御モード
    let mut control_mode = ControlMode::OpenLoop;

    // 速度ランプ（加速度制限）用の現在指令速度
    let mut ramped_target_speed: f32 = 0.0;

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
        let motor_enabled = *MOTOR_ENABLE.lock().await;
        if !motor_enabled {
            if was_enabled {
                info!("Motor control loop: Disabling PWM channels");
                was_enabled = false;
            }

            // モーター停止：PWMチャネルを完全無効化
            motor_driver.stop();

            // 各コントローラとセンサーをリセット
            speed_pi.reset();
            hall_sensor.reset();
            openloop.reset();
            hall_tim::reset_state(); // TIM4の状態もリセット
            ramped_target_speed = 0.0; // 速度ランプもリセット
            control_mode = ControlMode::OpenLoop; // OpenLoopに戻す

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
        {
            let mut calibration_request = CALIBRATION_REQUEST.lock().await;
            if *calibration_request {
                info!("Calibration requested, switching to Calibration mode");
                *calibration_request = false; // リクエストをクリア

                // トルク値を取得（0-100 → 0.0-1.0に変換）
                let torque_u8 = *CALIBRATION_TORQUE.lock().await;
                let torque_f32 = torque_u8 as f32 / 100.0;
                info!("Starting motor calibration...");
                info!("  Pole pairs: {}", DEFAULT_POLE_PAIRS);
                info!("  Torque: {}", torque_f32);
                calibration.set_torque(torque_f32);

                control_mode = ControlMode::Calibration;
                calibration.start();

                // 制御モードをグローバル状態に反映
                let mut mode = CONTROL_MODE.lock().await;
                *mode = ControlMode::Calibration;
            }
        }

        // 3. 制御モード別処理
        match control_mode {
            ControlMode::OpenLoop => {
                // オープンループ制御を実行
                let (should_switch, _hall_state) =
                    openloop_mode::execute(&mut openloop, &hall_sensor, &mut motor_driver, dt)
                        .await;

                // OpenLoopからFOCへの切り替え判定
                if should_switch {
                    control_mode = ControlMode::ClosedLoopFoc;
                    info!("Switching to FOC mode: Hall state valid, target speed reached");

                    // Hall センサーの速度フィルタを現在の速度で初期化
                    let current_rpm = openloop.get_current_rpm();
                    hall_sensor.reset_speed_filter(current_rpm);
                    ramped_target_speed = current_rpm;
                    info!("FOC mode initialized with speed: {} RPM", current_rpm);
                }
            }

            ControlMode::ClosedLoopFoc => {
                // FOC制御を実行
                let success = foc_mode::execute(
                    &mut hall_sensor,
                    &mut speed_pi,
                    &mut motor_driver,
                    &mut ramped_target_speed,
                    dt,
                )
                .await;

                // Hall状態が無効な場合は処理をスキップ
                if !success {
                    Timer::after(Duration::from_micros(DEFAULT_CONTROL_PERIOD_US)).await;
                    continue;
                }
            }

            ControlMode::Calibration => {
                // キャリブレーション制御を実行
                if let Some(next_mode) = calibration_mode::execute(
                    &mut calibration,
                    &mut hall_sensor,
                    &mut motor_driver,
                    dt,
                )
                .await
                {
                    // キャリブレーション完了、次のモードに移行
                    control_mode = next_mode;
                }
            }
        }

        Timer::after(Duration::from_micros(DEFAULT_CONTROL_PERIOD_US)).await;
    }
}
