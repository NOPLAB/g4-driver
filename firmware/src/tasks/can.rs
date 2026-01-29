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
    can_ids, parse_advance_angle_params, parse_advance_angle_speed, parse_angle_interpolation,
    parse_can_config, parse_control_timing, parse_dead_time_comp_params, parse_enable_command,
    parse_flux_weakening_enable, parse_flux_weakening_params, parse_flux_weakening_vd,
    parse_foc_stall_params, parse_hall_sensor_params, parse_max_speed_accel,
    parse_min_voltage_params, parse_motor_basic_params, parse_motor_voltage_params,
    parse_openloop_accel_duty_params, parse_openloop_cycles_params, parse_openloop_rpm_params,
    parse_pi_gains, parse_pwm_config, parse_speed_command, parse_voltage_monitor_filter,
    parse_voltage_monitor_thresholds,
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
                                    state::motor_context().await.target_speed = speed;
                                }
                            }
                            can_ids::PI_GAINS => {
                                if let Some((kp, ki)) = parse_pi_gains(data) {
                                    state::motor_context().await.pi_gains = (kp, ki);
                                }
                            }
                            can_ids::ENABLE_CMD => {
                                if let Some(enable) = parse_enable_command(data) {
                                    state::motor_context().await.enabled = enable;
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
                                {
                                    let mut ctx = state::calibration_context().await;
                                    ctx.torque = torque;
                                    ctx.request = true;
                                }
                                info!("Calibration request flag set");
                            }
                            can_ids::SAVE_CONFIG => {
                                info!("Save config command received");

                                // 現在の設定とキャリブレーション結果を取得
                                let mut cfg = state::system_context().await.runtime_config;
                                let calib_result = state::calibration_context().await.result;

                                // キャリブレーション結果を設定に反映
                                cfg.calibration_electrical_offset = calib_result.electrical_offset;
                                cfg.calibration_direction_inversed = calib_result.direction_inversed;
                                cfg.calibration_success = calib_result.success;

                                // フラッシュに保存
                                match config::write_config(&mut flash, &mut crc, &mut cfg).await {
                                    Ok(_) => {
                                        info!("Config saved successfully");
                                        state::system_context().await.config_crc_valid = true;
                                    }
                                    Err(e) => {
                                        error!("Failed to save config: {:?}", e);
                                        state::system_context().await.config_crc_valid = false;
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
                                        state::motor_context().await.pi_gains = (loaded_config.speed_kp, loaded_config.speed_ki);

                                        info!("  PI gains: Kp={}, Ki={}", loaded_config.speed_kp, loaded_config.speed_ki);
                                    }
                                    Err(e) => {
                                        error!("Failed to reload config: {:?}", e);
                                        state::system_context().await.config_crc_valid = false;
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
                                        state::motor_context().await.pi_gains = (default_config.speed_kp, default_config.speed_ki);

                                        info!("  PI gains: Kp={}, Ki={}", default_config.speed_kp, default_config.speed_ki);
                                    }
                                    Err(e) => {
                                        error!("Failed to reset config: {:?}", e);
                                        state::system_context().await.config_crc_valid = false;
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
                            // === Advance Angle Parameters ===
                            can_ids::ADVANCE_ANGLE_PARAMS => {
                                if let Some((base_deg, max_deg)) = parse_advance_angle_params(data)
                                {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.advance_base_deg = base_deg;
                                    ctx.runtime_config.advance_max_deg = max_deg;
                                    info!(
                                        "Updated advance angle params: base={}°, max={}°",
                                        base_deg, max_deg
                                    );
                                }
                            }
                            can_ids::ADVANCE_ANGLE_SPEED => {
                                if let Some((min_speed, max_speed)) =
                                    parse_advance_angle_speed(data)
                                {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.advance_min_speed = min_speed;
                                    ctx.runtime_config.advance_max_speed = max_speed;
                                    info!(
                                        "Updated advance angle speed: min={} RPM, max={} RPM",
                                        min_speed, max_speed
                                    );
                                }
                            }
                            // === Min Voltage Parameters ===
                            can_ids::MIN_VOLTAGE_PARAMS => {
                                if let Some((min_voltage, error_threshold)) =
                                    parse_min_voltage_params(data)
                                {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.min_voltage = min_voltage;
                                    ctx.runtime_config.min_voltage_error_threshold = error_threshold;
                                    info!(
                                        "Updated min voltage params: min={}V, threshold={}",
                                        min_voltage, error_threshold
                                    );
                                }
                            }
                            can_ids::MAX_SPEED_ACCEL => {
                                if let Some(max_accel) = parse_max_speed_accel(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.max_speed_acceleration = max_accel;
                                    info!("Updated max speed acceleration: {} RPM/s", max_accel);
                                }
                            }
                            // === FOC Stall Parameters ===
                            can_ids::FOC_STALL_PARAMS => {
                                if let Some((speed_threshold, count_threshold)) =
                                    parse_foc_stall_params(data)
                                {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.foc_stall_speed_threshold = speed_threshold;
                                    ctx.runtime_config.foc_stall_count_threshold = count_threshold;
                                    info!(
                                        "Updated FOC stall params: speed={} RPM, count={}",
                                        speed_threshold, count_threshold
                                    );
                                }
                            }
                            // === OpenLoop Cycles Parameters ===
                            can_ids::OPENLOOP_CYCLES_PARAMS => {
                                if let Some((forced_cycles, min_cycles)) =
                                    parse_openloop_cycles_params(data)
                                {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.forced_commutation_cycles = forced_cycles;
                                    ctx.runtime_config.min_cycles_before_foc = min_cycles;
                                    info!(
                                        "Updated openloop cycles: forced={}, min={}",
                                        forced_cycles, min_cycles
                                    );
                                }
                            }
                            // === Dead Time Compensation Parameters ===
                            can_ids::DEAD_TIME_COMP_PARAMS => {
                                if let Some((enabled, dead_time_ns)) =
                                    parse_dead_time_comp_params(data)
                                {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.dead_time_comp_enabled = enabled;
                                    ctx.runtime_config.dead_time_ns = dead_time_ns;
                                    info!(
                                        "Updated dead time comp: enabled={}, ns={}",
                                        enabled, dead_time_ns
                                    );
                                }
                            }
                            // === Flux Weakening Parameters ===
                            can_ids::FLUX_WEAKENING_ENABLE => {
                                if let Some((enabled, min_speed)) = parse_flux_weakening_enable(data)
                                {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.flux_weakening_enabled = enabled;
                                    ctx.runtime_config.flux_weakening_min_speed = min_speed;
                                    info!(
                                        "Updated flux weakening: enabled={}, min_speed={} RPM",
                                        enabled, min_speed
                                    );
                                }
                            }
                            can_ids::FLUX_WEAKENING_PARAMS => {
                                if let Some((max_speed, max_ratio)) =
                                    parse_flux_weakening_params(data)
                                {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.flux_weakening_max_speed = max_speed;
                                    ctx.runtime_config.flux_weakening_max_ratio = max_ratio;
                                    info!(
                                        "Updated flux weakening params: max_speed={} RPM, ratio={}",
                                        max_speed, max_ratio
                                    );
                                }
                            }
                            can_ids::FLUX_WEAKENING_VD => {
                                if let Some(vd_rate_limit) = parse_flux_weakening_vd(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.flux_weakening_vd_rate_limit = vd_rate_limit;
                                    info!("Updated flux weakening Vd rate: {} V/s", vd_rate_limit);
                                }
                            }
                            // === Voltage Monitor Parameters ===
                            can_ids::VOLTAGE_MONITOR_THRESHOLDS => {
                                if let Some((overvoltage, undervoltage)) =
                                    parse_voltage_monitor_thresholds(data)
                                {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.voltage_overvoltage_threshold = overvoltage;
                                    ctx.runtime_config.voltage_undervoltage_threshold = undervoltage;
                                    info!(
                                        "Updated voltage thresholds: OV={}V, UV={}V",
                                        overvoltage, undervoltage
                                    );
                                }
                            }
                            can_ids::VOLTAGE_MONITOR_FILTER => {
                                if let Some(alpha) = parse_voltage_monitor_filter(data) {
                                    let mut ctx = SYSTEM_CONTEXT.lock().await;
                                    ctx.runtime_config.voltage_filter_alpha = alpha;
                                    info!("Updated voltage filter alpha: {}", alpha);
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
                    Err(e) => {
                        warn!("CAN RX error: {:?}", e);
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
