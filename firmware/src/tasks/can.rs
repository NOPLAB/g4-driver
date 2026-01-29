//! CAN通信タスク
//!
//! モーター制御コマンドの受信とステータス送信を行います。

use embassy_stm32::{
    can,
    crc::Crc,
    flash::{Blocking, Flash},
};
use embassy_time::{Duration, Ticker};
use embedded_can::Id;

use crate::config;
use crate::fmt::*;
use crate::state::{self, SYSTEM_CONTEXT};
use g4_driver_protocol::{
    can_ids, parse_angle_interpolation, parse_can_config, parse_control_timing,
    parse_enable_command, parse_hall_sensor_params, parse_motor_basic_params,
    parse_motor_voltage_params, parse_openloop_accel_duty_params, parse_openloop_rpm_params,
    parse_pi_gains, parse_pwm_config, parse_speed_command,
};

/// CAN通信タスク - モーター制御コマンド処理とステータス送信
#[embassy_executor::task]
pub async fn can_task(
    can: can::Can<'static>,
    mut flash: Flash<'static, Blocking>,
    mut crc: Crc<'static>,
) {
    let (_tx, mut rx, _properties) = can.split();

    info!("CAN motor control task started");

    // ステータス送信用タイマー（100ms周期）
    let mut status_ticker = Ticker::every(Duration::from_millis(100));

    loop {
        // CANフレーム受信とステータス送信を並行処理
        embassy_futures::select::select(
            async {
                // CANフレーム受信処理
                match rx.read().await {
                    Ok(envelope) => {
                        let frame = envelope.frame;
                        let data = frame.data();
                        let header = frame.header();

                        // IDを数値として取得
                        let id_raw = match header.id() {
                            Id::Standard(std_id) => std_id.as_raw() as u32,
                            Id::Extended(ext_id) => ext_id.as_raw(),
                        };

                        match id_raw {
                            can_ids::SPEED_CMD => {
                                if let Some(speed) = parse_speed_command(data) {
                                    state::set_target_speed(speed).await;
                                }
                            }
                            can_ids::PI_GAINS => {
                                if let Some((kp, ki)) = parse_pi_gains(data) {
                                    state::set_pi_gains(kp, ki).await;
                                }
                            }
                            can_ids::ENABLE_CMD => {
                                if let Some(enable) = parse_enable_command(data) {
                                    state::set_motor_enabled(enable).await;
                                    if enable {
                                        info!("Motor ENABLED via CAN");
                                    } else {
                                        info!("Motor DISABLED via CAN");
                                    }
                                }
                            }
                            can_ids::START_CALIBRATION => {
                                info!("Start calibration command received");
                                // トルク値をパース（1バイト, 0-100, デフォルト20）
                                let torque = if !data.is_empty() {
                                    data[0].min(100) // 0-100に制限
                                } else {
                                    20 // デフォルト値
                                };
                                info!("Calibration torque: {}", torque);
                                state::set_calibration_torque(torque).await;
                                // キャリブレーションリクエストフラグを設定
                                state::set_calibration_request(true).await;
                                info!("Calibration request flag set");
                            }
                            can_ids::SAVE_CONFIG => {
                                info!("Save config command received");

                                // 現在の設定を取得
                                let mut cfg = state::get_runtime_config().await;

                                // キャリブレーション結果を設定に反映
                                let calib_result = state::get_calibration_result().await;
                                cfg.calibration_electrical_offset = calib_result.electrical_offset;
                                cfg.calibration_direction_inversed = calib_result.direction_inversed;
                                cfg.calibration_success = calib_result.success;

                                // フラッシュに保存
                                match config::write_config(&mut flash, &mut crc, &mut cfg).await {
                                    Ok(_) => {
                                        info!("Config saved successfully");
                                        state::set_config_crc_valid(true).await;
                                    }
                                    Err(e) => {
                                        error!("Failed to save config: {:?}", e);
                                        state::set_config_crc_valid(false).await;
                                    }
                                }
                            }
                            can_ids::RELOAD_CONFIG => {
                                info!("Reload config command received");

                                // フラッシュから設定を読み込み
                                match config::read_config(&mut flash, &mut crc) {
                                    Ok(loaded_config) => {
                                        info!("Config reloaded successfully");

                                        // グローバル状態に適用
                                        state::update_system_config(loaded_config, loaded_config.version, true).await;

                                        // PIゲインを更新
                                        state::set_pi_gains(loaded_config.speed_kp, loaded_config.speed_ki).await;

                                        info!("  PI gains: Kp={}, Ki={}", loaded_config.speed_kp, loaded_config.speed_ki);
                                    }
                                    Err(e) => {
                                        error!("Failed to reload config: {:?}", e);
                                        state::set_config_crc_valid(false).await;
                                    }
                                }
                            }
                            can_ids::RESET_CONFIG => {
                                info!("Reset config command received");

                                // デフォルト設定を作成
                                match config::initialize_default_config(&mut flash, &mut crc).await {
                                    Ok(default_config) => {
                                        info!("Config reset to defaults successfully");

                                        // グローバル状態に適用
                                        state::update_system_config(default_config, default_config.version, true).await;

                                        // PIゲインを更新
                                        state::set_pi_gains(default_config.speed_kp, default_config.speed_ki).await;

                                        info!("  PI gains: Kp={}, Ki={}", default_config.speed_kp, default_config.speed_ki);
                                    }
                                    Err(e) => {
                                        error!("Failed to reset config: {:?}", e);
                                        state::set_config_crc_valid(false).await;
                                    }
                                }
                            }
                            // === Motor Control Parameter Commands ===
                            can_ids::MOTOR_VOLTAGE_PARAMS => {
                                if let Some((max_voltage, v_dc_bus)) = parse_motor_voltage_params(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.max_voltage = max_voltage;
                                    ctx.runtime_config.v_dc_bus = v_dc_bus;
                                    info!("Updated motor voltage params: max={}, vdc={}", max_voltage, v_dc_bus);
                                }
                            }
                            can_ids::MOTOR_BASIC_PARAMS => {
                                if let Some((pole_pairs, max_duty)) = parse_motor_basic_params(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.pole_pairs = pole_pairs;
                                    ctx.runtime_config.max_duty = max_duty;
                                    info!("Updated motor basic params: pole_pairs={}, max_duty={}", pole_pairs, max_duty);
                                }
                            }
                            can_ids::HALL_SENSOR_PARAMS => {
                                if let Some((alpha, offset)) = parse_hall_sensor_params(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.speed_filter_alpha = alpha;
                                    ctx.runtime_config.hall_angle_offset = offset;
                                    info!("Updated hall sensor params: alpha={}, offset={}", alpha, offset);
                                }
                            }
                            can_ids::ANGLE_INTERPOLATION => {
                                if let Some(enable) = parse_angle_interpolation(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.enable_angle_interpolation = enable;
                                    info!("Updated angle interpolation: {}", enable);
                                }
                            }
                            // === OpenLoop Parameter Commands ===
                            can_ids::OPENLOOP_RPM_PARAMS => {
                                if let Some((initial_rpm, target_rpm)) = parse_openloop_rpm_params(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.openloop_initial_rpm = initial_rpm;
                                    ctx.runtime_config.openloop_target_rpm = target_rpm;
                                    info!("Updated openloop RPM params: initial={}, target={}", initial_rpm, target_rpm);
                                }
                            }
                            can_ids::OPENLOOP_ACCEL_DUTY_PARAMS => {
                                if let Some((acceleration, duty_ratio)) = parse_openloop_accel_duty_params(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.openloop_acceleration = acceleration;
                                    ctx.runtime_config.openloop_duty_ratio = duty_ratio;
                                    info!("Updated openloop accel/duty: accel={}, duty={}", acceleration, duty_ratio);
                                }
                            }
                            // === PWM/CAN/Timing Configuration ===
                            can_ids::PWM_CONFIG => {
                                if let Some((frequency, dead_time)) = parse_pwm_config(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.pwm_frequency = frequency;
                                    ctx.runtime_config.pwm_dead_time = dead_time;
                                    info!("Updated PWM config: freq={}Hz, dead_time={}", frequency, dead_time);
                                    info!("⚠ PWM changes require reboot to take effect. Save config and restart.");
                                }
                            }
                            can_ids::CAN_CONFIG => {
                                if let Some(bitrate) = parse_can_config(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.can_bitrate = bitrate;
                                    info!("Updated CAN config: bitrate={}", bitrate);
                                    info!("⚠ CAN bitrate changes require reboot to take effect. Save config and restart.");
                                }
                            }
                            can_ids::CONTROL_TIMING => {
                                if let Some(period_us) = parse_control_timing(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.control_period_us = period_us;
                                    info!("Updated control timing: {}us", period_us);
                                    info!("⚠ Control period changes require reboot to take effect. Save config and restart.");
                                }
                            }
                            can_ids::EMERGENCY_STOP => {
                                info!("Emergency stop received!");
                                state::emergency_stop().await;
                            }
                            _ => {
                                debug!("Unknown CAN ID: 0x{:03X}", id_raw);
                            }
                        }
                    }
                    Err(_e) => {
                        // error!("CAN RX Error: {:?}", _e);
                    }
                }
            },
            async {
                // ステータス送信（100ms周期）- 現在は無効化
                status_ticker.next().await;
            },
        )
        .await;
    }
}
