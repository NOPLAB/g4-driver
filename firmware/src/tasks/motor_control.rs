//! モーター制御タスク
//!
//! 2.5kHz FOCループ + オープンループ始動制御を実行します。

use embassy_stm32::{
    peripherals,
    timer::{complementary_pwm::ComplementaryPwm, Channel},
};
use embassy_time::{Duration, Timer};

use crate::config::*;
use crate::fmt::*;
use crate::foc::{
    calculate_svpwm, inverse_park, limit_voltage, ControlMode, HallSensor, MotorCalibration,
    OpenLoopSixStep, PiController,
};
use crate::hall_tim;
use crate::state::{
    CALIBRATION_REQUEST, CALIBRATION_RESULT, CONTROL_MODE, MOTOR_ENABLE, MOTOR_STATUS,
    SPEED_PI_GAINS, TARGET_SPEED,
};
use core::f32::consts::PI;

/// モーター制御タスク（2.5kHz FOC制御ループ）
#[embassy_executor::task]
pub async fn motor_control_task(mut uvw_pwm: ComplementaryPwm<'static, peripherals::TIM1>) {
    info!("Motor control task started (OpenLoop + FOC mode)");

    // ホールセンサ初期化（foc-simple互換の機械角ベース計算）
    let mut hall_sensor = HallSensor::new(DEFAULT_POLE_PAIRS, DEFAULT_SPEED_FILTER_ALPHA);
    // 角度補間を有効化（FOC制御の安定性向上）
    hall_sensor.set_interpolation(true);
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

    // キャリブレーション初期化（トルク0.2 = 20%）
    let mut calibration = MotorCalibration::new(DEFAULT_POLE_PAIRS, 0.2);

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
            }

            // モーター停止：PWMチャネルを完全無効化
            uvw_pwm.disable(Channel::Ch1);
            uvw_pwm.disable(Channel::Ch2);
            uvw_pwm.disable(Channel::Ch3);

            // Duty比も0にセット
            uvw_pwm.set_duty(Channel::Ch1, 0);
            uvw_pwm.set_duty(Channel::Ch2, 0);
            uvw_pwm.set_duty(Channel::Ch3, 0);

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
            uvw_pwm.enable(Channel::Ch1);
            uvw_pwm.enable(Channel::Ch2);
            uvw_pwm.enable(Channel::Ch3);
            was_enabled = true;
        }

        // 1.5. キャリブレーションリクエストをチェック
        {
            let mut calibration_request = CALIBRATION_REQUEST.lock().await;
            if *calibration_request {
                info!("Calibration requested, switching to Calibration mode");
                *calibration_request = false; // リクエストをクリア
                control_mode = ControlMode::Calibration;
                calibration.start();

                // 制御モードをグローバル状態に反映
                let mut mode = CONTROL_MODE.lock().await;
                *mode = ControlMode::Calibration;
            }
        }

        // 2. 制御モード判定とHall状態取得

        // Hall状態の確認（有効な状態：1-6）
        let hall_state = hall_tim::get_hall_state();
        let is_valid_hall = (1..=6).contains(&hall_state);

        // 電気角と速度を取得（TIM4ハードウェアベース、foc-simple互換計算）
        let (hall_electrical_angle, speed_rpm) = hall_sensor.update(dt);

        // OpenLoopからFOCへの切り替え判定
        if control_mode == ControlMode::OpenLoop {
            // Hall状態が有効かつ目標速度に達したらFOCに切り替え
            if is_valid_hall && openloop.is_target_reached() {
                control_mode = ControlMode::ClosedLoopFoc;
                info!("Switching to FOC mode: Hall state valid, target speed reached");
                // Hall センサーの速度フィルタを現在の速度で初期化
                let current_rpm = openloop.get_current_rpm();
                hall_sensor.reset_speed_filter(current_rpm);
                // 速度ランプも現在の速度で初期化（急激な速度誤差を防ぐ）
                ramped_target_speed = current_rpm;
                info!("FOC mode initialized with speed: {} RPM", current_rpm);
            }
        }

        // 3. 制御モード別処理
        match control_mode {
            ControlMode::OpenLoop => {
                // オープンループ6ステップ駆動
                let step_state = openloop.update(dt);

                // PWM出力
                uvw_pwm.set_duty(Channel::Ch1, step_state.duty_u);
                uvw_pwm.set_duty(Channel::Ch2, step_state.duty_v);
                uvw_pwm.set_duty(Channel::Ch3, step_state.duty_w);

                // チャネル有効/無効制御
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
                    status.speed_rpm = openloop.get_current_rpm();
                    status.electrical_angle = 0.0; // OpenLoopでは電気角は不定
                }

                // デバッグログ（低頻度）
                static mut OPENLOOP_LOG_COUNTER: u32 = 0;
                unsafe {
                    OPENLOOP_LOG_COUNTER += 1;
                    if OPENLOOP_LOG_COUNTER >= 2500 {
                        OPENLOOP_LOG_COUNTER = 0;
                        debug!(
                            "[OpenLoop] Step: {}, RPM: {}, Hall: {}, DutyU/V/W: {}/{}/{}",
                            step_state.step,
                            openloop.get_current_rpm(),
                            hall_state,
                            step_state.duty_u,
                            step_state.duty_v,
                            step_state.duty_w
                        );
                    }
                }
            }
            ControlMode::ClosedLoopFoc => {
                // Hallセンサが無効な場合の安全処理
                if !is_valid_hall {
                    // PWM出力を停止
                    uvw_pwm.set_duty(Channel::Ch1, 0);
                    uvw_pwm.set_duty(Channel::Ch2, 0);
                    uvw_pwm.set_duty(Channel::Ch3, 0);
                    speed_pi.reset();
                    ramped_target_speed = 0.0;

                    Timer::after(Duration::from_micros(DEFAULT_CONTROL_PERIOD_US)).await;
                    continue;
                }

                // 電気角はHallSensor内で既に計算済み（機械角 * 極対数 - 電気オフセット）
                // foc-simple互換の計算方式により、追加のオフセット適用は不要

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

                // 速度ランプ（加速度制限）を適用
                let speed_error = target_speed - ramped_target_speed;
                let max_delta_speed = MAX_SPEED_ACCELERATION * dt; // 1制御周期で変化可能な最大速度

                if speed_error.abs() > max_delta_speed {
                    // 加速度制限を適用
                    if speed_error > 0.0 {
                        ramped_target_speed += max_delta_speed;
                    } else {
                        ramped_target_speed -= max_delta_speed;
                    }
                } else {
                    // 目標速度に到達
                    ramped_target_speed = target_speed;
                }

                // 速度PI制御（q軸電圧指令生成）- ランプ処理後の速度を使用
                let mut vq_cmd = speed_pi.update(ramped_target_speed, speed_rpm, dt);
                let vd_cmd = 0.0; // SPMSM: d軸電流/電圧は0

                // 停止時の処理：目標速度が0で実際に停止している場合、PI積分項をリセット
                if ramped_target_speed.abs() < 1.0 && speed_rpm.abs() < 1.0 {
                    speed_pi.reset();
                    vq_cmd = 0.0;
                }

                // 最小電圧適用（静止摩擦克服用）
                let speed_error_abs = (ramped_target_speed - speed_rpm).abs();
                if speed_error_abs > MIN_VOLTAGE_ERROR_THRESHOLD && vq_cmd.abs() > 0.0 {
                    // 速度誤差が大きい場合、最小電圧を適用
                    if vq_cmd > 0.0 {
                        vq_cmd = vq_cmd.max(MIN_VOLTAGE);
                    } else {
                        vq_cmd = vq_cmd.min(-MIN_VOLTAGE);
                    }
                }

                // 電圧ベクトル制限
                let (vd_limited, vq_limited) = limit_voltage(vd_cmd, vq_cmd, DEFAULT_MAX_VOLTAGE);

                // Park逆変換（dq → αβ）
                let (v_alpha, v_beta) = inverse_park(vd_limited, vq_limited, hall_electrical_angle);

                // SVPWM計算
                let (duty_u, duty_v, duty_w) =
                    calculate_svpwm(v_alpha, v_beta, DEFAULT_V_DC_BUS, DEFAULT_MAX_DUTY);

                // デバッグ用：FOC制御の詳細ログ（10Hz = 250回に1回）
                static mut FOC_LOG_COUNTER: u32 = 0;
                unsafe {
                    FOC_LOG_COUNTER += 1;
                    if FOC_LOG_COUNTER >= 250 {
                        FOC_LOG_COUNTER = 0;
                        let angle_deg = hall_electrical_angle * 180.0 / core::f32::consts::PI;
                        trace!(
                            "[FOC Detail] Hall={}, Angle={}rad ({}°), Vq={}V, Valpha={}V, Vbeta={}V, DutyU={}, DutyV={}, DutyW={}",
                            hall_state, hall_electrical_angle, angle_deg, vq_limited, v_alpha, v_beta, duty_u, duty_v, duty_w
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

                // デバッグログ（低頻度）
                static mut FOC_MODE_LOG_COUNTER: u32 = 0;
                unsafe {
                    FOC_MODE_LOG_COUNTER += 1;
                    if FOC_MODE_LOG_COUNTER >= 2500 {
                        // 1秒ごと（2.5kHz / 2500 = 1Hz）
                        FOC_MODE_LOG_COUNTER = 0;

                        // TIM4ベースのHallセンサ値を取得（ログ用）
                        let period_cycles = hall_tim::get_period_cycles();

                        // 最新のステータスを取得
                        let status = *MOTOR_STATUS.lock().await;
                        let target_speed = *TARGET_SPEED.lock().await;
                        debug!(
                            "[FOC] Speed: {}/{} RPM (ramped: {}), Angle: {}rad, Hall: {}, Period: {} cycles",
                            status.speed_rpm, target_speed, ramped_target_speed, status.electrical_angle, hall_state, period_cycles
                        );
                    }
                }
            }
            ControlMode::Calibration => {
                // キャリブレーションモード
                // Hall センサーから現在の角度を取得
                let sensor_angle = hall_sensor.get_mechanical_angle();

                // キャリブレーションステートマシンを更新
                match calibration.update(sensor_angle) {
                    Ok((electrical_angle, torque)) => {
                        // トルクから電圧指令を計算（トルク 0.0～1.0 → 電圧 0～MAX_VOLTAGE）
                        let v_cmd = torque * DEFAULT_MAX_VOLTAGE;

                        // d軸・q軸電圧（キャリブレーション中はシンプルにq軸のみ）
                        let vd_cmd = 0.0;
                        let vq_cmd = v_cmd;

                        // Park逆変換
                        let (v_alpha, v_beta) = inverse_park(vd_cmd, vq_cmd, electrical_angle);

                        // SVPWM計算
                        let (duty_u, duty_v, duty_w) =
                            calculate_svpwm(v_alpha, v_beta, DEFAULT_V_DC_BUS, DEFAULT_MAX_DUTY);

                        // PWM出力
                        uvw_pwm.set_duty(Channel::Ch1, duty_u);
                        uvw_pwm.set_duty(Channel::Ch2, duty_v);
                        uvw_pwm.set_duty(Channel::Ch3, duty_w);

                        // すべてのチャネルを有効化
                        uvw_pwm.enable(Channel::Ch1);
                        uvw_pwm.enable(Channel::Ch2);
                        uvw_pwm.enable(Channel::Ch3);

                        // キャリブレーション完了チェック
                        if calibration.is_completed() {
                            let result = calibration.get_result();

                            if result.success {
                                info!("Calibration completed successfully!");
                                info!(
                                    "  Electrical offset: {} rad ({} deg)",
                                    result.electrical_offset,
                                    result.electrical_offset * 180.0 / PI
                                );
                                info!("  Direction inversed: {}", result.direction_inversed);

                                // 結果をグローバル状態に保存
                                {
                                    let mut saved_result = CALIBRATION_RESULT.lock().await;
                                    *saved_result = result;
                                }

                                // Hall センサーに結果を適用
                                hall_sensor.set_electrical_offset(result.electrical_offset);
                                // TODO: 方向反転の適用（HallSensor に direction_inversed を追加する必要がある）

                                // ClosedLoopFocモードに切り替え
                                control_mode = ControlMode::ClosedLoopFoc;
                                let mut mode = CONTROL_MODE.lock().await;
                                *mode = ControlMode::ClosedLoopFoc;

                                info!("Switching to ClosedLoopFoc mode");
                            } else {
                                error!("Calibration failed!");
                                // エラー時はモーターを停止
                                uvw_pwm.set_duty(Channel::Ch1, 0);
                                uvw_pwm.set_duty(Channel::Ch2, 0);
                                uvw_pwm.set_duty(Channel::Ch3, 0);

                                // OpenLoopモードに戻る
                                control_mode = ControlMode::OpenLoop;
                                let mut mode = CONTROL_MODE.lock().await;
                                *mode = ControlMode::OpenLoop;
                            }
                        }
                    }
                    Err(_) => {
                        error!("Calibration update error, stopping motor");
                        // エラー時はモーターを停止
                        uvw_pwm.set_duty(Channel::Ch1, 0);
                        uvw_pwm.set_duty(Channel::Ch2, 0);
                        uvw_pwm.set_duty(Channel::Ch3, 0);

                        // OpenLoopモードに戻る
                        control_mode = ControlMode::OpenLoop;
                        let mut mode = CONTROL_MODE.lock().await;
                        *mode = ControlMode::OpenLoop;
                    }
                }
            }
        }

        Timer::after(Duration::from_micros(DEFAULT_CONTROL_PERIOD_US)).await;
    }
}
