//! CAN通信タスク
//!
//! モーター制御コマンドの受信とステータス送信を行います。

use embassy_stm32::{can, crc::Crc, flash::Flash};
use embassy_time::{Duration, Ticker};
use embedded_can::{Id, StandardId};

use crate::can_protocol::{
    can_ids, encode_config_status, encode_status, encode_voltage_status, parse_enable_command,
    parse_pi_gains, parse_speed_command,
};
use crate::config_storage::StoredConfig;
use crate::eeprom;
use crate::fmt::*;
use crate::state::{
    CONFIG_CRC_VALID, CONFIG_VERSION, MOTOR_ENABLE, MOTOR_STATUS, RUNTIME_CONFIG, SPEED_PI_GAINS,
    TARGET_SPEED, VOLTAGE_STATE,
};

/// CAN通信タスク - モーター制御コマンド処理とステータス送信
#[embassy_executor::task]
pub async fn can_task(
    can: can::Can<'static>,
    mut flash: Flash<'static>,
    mut crc: Crc<'static>,
) {
    let (mut tx, mut rx, _properties) = can.split();

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
                                    *TARGET_SPEED.lock().await = speed;
                                }
                            }
                            can_ids::PI_GAINS => {
                                if let Some((kp, ki)) = parse_pi_gains(data) {
                                    *SPEED_PI_GAINS.lock().await = (kp, ki);
                                }
                            }
                            can_ids::ENABLE_CMD => {
                                if let Some(enable) = parse_enable_command(data) {
                                    *MOTOR_ENABLE.lock().await = enable;
                                    if enable {
                                        info!("Motor ENABLED via CAN");
                                    } else {
                                        info!("Motor DISABLED via CAN");
                                    }
                                }
                            }
                            can_ids::SAVE_CONFIG => {
                                info!("Save config command received");

                                // 現在の設定を取得
                                let mut config = RUNTIME_CONFIG.lock().await.clone();

                                // フラッシュに保存
                                match eeprom::write_config(&mut flash, &mut crc, &mut config).await {
                                    Ok(_) => {
                                        info!("Config saved successfully");
                                        *CONFIG_CRC_VALID.lock().await = true;
                                    }
                                    Err(e) => {
                                        error!("Failed to save config: {:?}", e);
                                        *CONFIG_CRC_VALID.lock().await = false;
                                    }
                                }
                            }
                            can_ids::RELOAD_CONFIG => {
                                info!("Reload config command received");

                                // フラッシュから設定を読み込み
                                match eeprom::read_config(&mut flash, &mut crc) {
                                    Ok(loaded_config) => {
                                        info!("Config reloaded successfully");

                                        // グローバル状態に適用
                                        *RUNTIME_CONFIG.lock().await = loaded_config;
                                        *CONFIG_VERSION.lock().await = loaded_config.version;
                                        *CONFIG_CRC_VALID.lock().await = true;

                                        // PIゲインを更新
                                        *SPEED_PI_GAINS.lock().await =
                                            (loaded_config.speed_kp, loaded_config.speed_ki);

                                        info!("  PI gains: Kp={}, Ki={}", loaded_config.speed_kp, loaded_config.speed_ki);
                                    }
                                    Err(e) => {
                                        error!("Failed to reload config: {:?}", e);
                                        *CONFIG_CRC_VALID.lock().await = false;
                                    }
                                }
                            }
                            can_ids::RESET_CONFIG => {
                                info!("Reset config command received");

                                // デフォルト設定を作成
                                match eeprom::initialize_default_config(&mut flash, &mut crc).await {
                                    Ok(default_config) => {
                                        info!("Config reset to defaults successfully");

                                        // グローバル状態に適用
                                        *RUNTIME_CONFIG.lock().await = default_config;
                                        *CONFIG_VERSION.lock().await = default_config.version;
                                        *CONFIG_CRC_VALID.lock().await = true;

                                        // PIゲインを更新
                                        *SPEED_PI_GAINS.lock().await =
                                            (default_config.speed_kp, default_config.speed_ki);

                                        info!("  PI gains: Kp={}, Ki={}", default_config.speed_kp, default_config.speed_ki);
                                    }
                                    Err(e) => {
                                        error!("Failed to reset config: {:?}", e);
                                        *CONFIG_CRC_VALID.lock().await = false;
                                    }
                                }
                            }
                            can_ids::EMERGENCY_STOP => {
                                info!("Emergency stop received!");
                                *MOTOR_ENABLE.lock().await = false;
                                *TARGET_SPEED.lock().await = 0.0;
                            }
                            _ => {
                                debug!("Unknown CAN ID: 0x{:03X}", id_raw);
                            }
                        }
                    }
                    Err(e) => {
                        error!("CAN RX Error: {:?}", e);
                    }
                }
            },
            async {
                // ステータス送信（100ms周期）
                status_ticker.next().await;

                // モーターステータス送信 (ID 0x200)
                let status = *MOTOR_STATUS.lock().await;
                let data = encode_status(status.speed_rpm, status.electrical_angle);

                if let Some(std_id) = StandardId::new(can_ids::STATUS as u16) {
                    let id = Id::Standard(std_id);
                    if let Ok(frame) = can::frame::Frame::new_data(id, &data) {
                        let _ = tx.write(&frame).await;
                    }
                }

                // 電圧ステータス送信 (ID 0x201)
                let voltage_state = *VOLTAGE_STATE.lock().await;
                let voltage_data = encode_voltage_status(
                    voltage_state.voltage,
                    voltage_state.overvoltage,
                    voltage_state.undervoltage,
                );

                if let Some(std_id) = StandardId::new(can_ids::VOLTAGE_STATUS as u16) {
                    let id = Id::Standard(std_id);
                    if let Ok(frame) = can::frame::Frame::new_data(id, &voltage_data) {
                        let _ = tx.write(&frame).await;
                    }
                }

                // 設定ステータス送信 (ID 0x202)
                let version = *CONFIG_VERSION.lock().await;
                let crc_valid = *CONFIG_CRC_VALID.lock().await;
                let config_data = encode_config_status(version, crc_valid);

                if let Some(std_id) = StandardId::new(can_ids::CONFIG_STATUS as u16) {
                    let id = Id::Standard(std_id);
                    if let Ok(frame) = can::frame::Frame::new_data(id, &config_data) {
                        let _ = tx.write(&frame).await;
                    }
                }
            },
        )
        .await;
    }
}
