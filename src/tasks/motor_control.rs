//! モーター制御タスク
//!
//! 2.5kHz FOCループ + オープンループ始動制御を実行します。

use embassy_stm32::{
    peripherals,
    timer::{complementary_pwm::ComplementaryPwm, Channel},
};
use embassy_time::{Duration, Timer};

use crate::config::*;
use crate::foc::{
    calculate_svpwm, inverse_park, limit_voltage, ControlMode, HallSensor, OpenLoopSixStep,
    PiController,
};
use crate::fmt::*;
use crate::hall_tim;
use crate::state::{MOTOR_ENABLE, MOTOR_STATUS, SPEED_PI_GAINS, TARGET_SPEED};

/// モーター制御タスク（2.5kHz FOCループ + オープンループ始動）
#[embassy_executor::task]
pub async fn motor_control_task(mut uvw_pwm: ComplementaryPwm<'static, peripherals::TIM1>) {
    info!("Motor control task started");

    // ホールセンサ初期化
    let mut hall_sensor = HallSensor::new(POLE_PAIRS, SPEED_FILTER_ALPHA);

    // 速度PIコントローラ初期化
    let mut speed_pi = PiController::new_symmetric(DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, MAX_VOLTAGE);

    // オープンループ6ステップ駆動初期化
    let mut open_loop = OpenLoopSixStep::new(
        openloop::INITIAL_RPM,
        openloop::TARGET_RPM,
        openloop::ACCELERATION_RPM_PER_S,
        openloop::DUTY_RATIO,
        POLE_PAIRS,
    );

    // 制御モード（初期はオープンループ）
    let mut control_mode = ControlMode::OpenLoop;

    // 制御周期
    let dt = CONTROL_PERIOD_US as f32 / 1_000_000.0; // 秒に変換

    info!(
        "FOC parameters: Pole pairs={}, Control freq={}Hz, dt={}s",
        POLE_PAIRS,
        1_000_000 / CONTROL_PERIOD_US,
        dt
    );
    info!(
        "OpenLoop 6-Step startup: Initial={}RPM, Target={}RPM, Accel={}RPM/s, Duty={}%",
        openloop::INITIAL_RPM,
        openloop::TARGET_RPM,
        openloop::ACCELERATION_RPM_PER_S,
        openloop::DUTY_RATIO
    );

    // モーター有効状態の追跡（PWMチャネル制御用）
    let mut was_enabled = false;

    loop {
        // 1. モーター使能チェック
        let motor_enabled = *MOTOR_ENABLE.lock().await;
        if !motor_enabled {
            // 状態が変化した場合のみログとPWM停止処理
            if was_enabled {
                info!("Motor control loop: Disabling PWM channels");
                was_enabled = false;
                // 制御モードをオープンループに戻す
                control_mode = ControlMode::OpenLoop;
            }

            // モーター停止：PWMチャネルを完全無効化
            uvw_pwm.disable(Channel::Ch1);
            uvw_pwm.disable(Channel::Ch2);
            uvw_pwm.disable(Channel::Ch3);

            // Duty比も0にセット
            uvw_pwm.set_duty(Channel::Ch1, 0);
            uvw_pwm.set_duty(Channel::Ch2, 0);
            uvw_pwm.set_duty(Channel::Ch3, 0);

            // 各コントローラをリセット
            speed_pi.reset();
            hall_sensor.reset();
            open_loop.reset();

            Timer::after(Duration::from_micros(CONTROL_PERIOD_US)).await;
            continue;
        }

        // モーター有効化時の処理
        if !was_enabled {
            info!("Motor control loop: Starting with OpenLoop mode");
            uvw_pwm.enable(Channel::Ch1);
            uvw_pwm.enable(Channel::Ch2);
            uvw_pwm.enable(Channel::Ch3);
            was_enabled = true;
        }

        // 2. 制御モードに応じた処理
        match control_mode {
            ControlMode::OpenLoop => {
                // オープンループモード時はホールセンサを使用しない（チャタリング回避）
                // オープンループ6ステップ駆動
                let step_state = open_loop.update(dt);
                let openloop_rpm = open_loop.get_current_rpm();

                // しきい値速度に達したらFOCモードに切り替え
                if open_loop.is_target_reached() {
                    info!(
                        "Switching to FOC mode: OpenLoop reached {}RPM",
                        openloop_rpm
                    );
                    control_mode = ControlMode::ClosedLoopFoc;
                    // FOCモードに切り替え時、ホールセンサをリセット
                    hall_sensor.reset();
                }

                // 6ステップ駆動のPWM設定
                uvw_pwm.set_duty(Channel::Ch1, step_state.duty_u);
                uvw_pwm.set_duty(Channel::Ch2, step_state.duty_v);
                uvw_pwm.set_duty(Channel::Ch3, step_state.duty_w);

                if step_state.enable_u {
                    uvw_pwm.enable(Channel::Ch1);
                } else {
                    uvw_pwm.disable(Channel::Ch1);
                }

                if step_state.enable_v {
                    uvw_pwm.enable(Channel::Ch2);
                } else {
                    uvw_pwm.disable(Channel::Ch2);
                }

                if step_state.enable_w {
                    uvw_pwm.enable(Channel::Ch3);
                } else {
                    uvw_pwm.disable(Channel::Ch3);
                }

                // ステータス更新
                {
                    let mut status = MOTOR_STATUS.lock().await;
                    status.speed_rpm = openloop_rpm;
                    status.electrical_angle = 0.0; // 6ステップでは電気角は使わない
                }
            }
            ControlMode::ClosedLoopFoc => {
                // クローズドループFOC制御

                // TIM4ハードウェアからHallセンサ状態と速度を取得
                let hall_state = hall_tim::get_hall_state();
                let period_cycles = hall_tim::get_period_cycles();
                let is_timeout = hall_tim::is_timeout();

                // 速度計算（TIM4キャプチャベース）
                let speed_rpm = if is_timeout {
                    0.0
                } else {
                    hall_tim::calculate_speed_rpm(period_cycles, POLE_PAIRS)
                };

                // 電気角推定（Hallセンサーから）
                let (hall_electrical_angle, _) = hall_sensor.update(hall_state, dt);

                // PIゲイン更新チェック（非同期で更新された場合）
                {
                    let (kp, ki) = *SPEED_PI_GAINS.lock().await;
                    if kp != speed_pi.get_kp() || ki != speed_pi.get_ki() {
                        speed_pi.set_gains(kp, ki);
                        info!("PI gains updated: Kp={}, Ki={}", kp, ki);
                    }
                }

                // 目標速度取得
                let target_speed = *TARGET_SPEED.lock().await;

                // 速度PI制御（q軸電圧指令生成）
                let vq_cmd = speed_pi.update(target_speed, speed_rpm, dt);
                let vd_cmd = 0.0; // SPMSM: d軸電流/電圧は0

                // 電圧ベクトル制限
                let (vd_limited, vq_limited) = limit_voltage(vd_cmd, vq_cmd, MAX_VOLTAGE);

                // Park逆変換（dq → αβ）
                let (v_alpha, v_beta) = inverse_park(vd_limited, vq_limited, hall_electrical_angle);

                // SVPWM計算
                let (duty_u, duty_v, duty_w) = calculate_svpwm(v_alpha, v_beta, V_DC_BUS, MAX_DUTY);

                // デバッグ用：FOC制御の詳細ログ（10Hz = 250回に1回）
                static mut FOC_LOG_COUNTER: u32 = 0;
                unsafe {
                    FOC_LOG_COUNTER += 1;
                    if FOC_LOG_COUNTER >= 250 {
                        FOC_LOG_COUNTER = 0;
                        trace!(
                            "[FOC Detail] Vq={}V, Valpha={}V, Vbeta={}V, DutyU={}, DutyV={}, DutyW={}, Angle={}rad",
                            vq_limited, v_alpha, v_beta, duty_u, duty_v, duty_w, hall_electrical_angle
                        );
                    }
                }

                // PWM出力
                uvw_pwm.set_duty(Channel::Ch1, duty_u);
                uvw_pwm.set_duty(Channel::Ch2, duty_v);
                uvw_pwm.set_duty(Channel::Ch3, duty_w);

                // FOCモードではすべてのチャネルを有効化
                uvw_pwm.enable(Channel::Ch1);
                uvw_pwm.enable(Channel::Ch2);
                uvw_pwm.enable(Channel::Ch3);

                // ステータス更新
                {
                    let mut status = MOTOR_STATUS.lock().await;
                    status.speed_rpm = speed_rpm;
                    status.electrical_angle = hall_electrical_angle;
                }
            }
        }

        // 5. デバッグログ（低頻度）
        static mut LOG_COUNTER: u32 = 0;
        unsafe {
            LOG_COUNTER += 1;
            if LOG_COUNTER >= 2500 {
                // 1秒ごと（2.5kHz / 2500 = 1Hz）
                LOG_COUNTER = 0;
                match control_mode {
                    ControlMode::OpenLoop => {
                        let openloop_rpm = open_loop.get_current_rpm();
                        debug!(
                            "[OpenLoop 6-Step] RPM: {}, Step: {}, Target: {} RPM (no Hall sensor)",
                            openloop_rpm,
                            open_loop.get_current_step(),
                            openloop::TARGET_RPM
                        );
                    }
                    ControlMode::ClosedLoopFoc => {
                        // TIM4ベースのHallセンサ値を取得（ログ用）
                        let hall_state = hall_tim::get_hall_state();
                        let period_cycles = hall_tim::get_period_cycles();

                        // 最新のステータスを取得
                        let status = *MOTOR_STATUS.lock().await;
                        let target_speed = *TARGET_SPEED.lock().await;
                        debug!(
                            "[FOC TIM4] Speed: {}/{} RPM, Angle: {}rad, Hall: {}, Period: {} cycles",
                            status.speed_rpm, target_speed, status.electrical_angle, hall_state, period_cycles
                        );
                    }
                }
            }
        }

        Timer::after(Duration::from_micros(CONTROL_PERIOD_US)).await;
    }
}
