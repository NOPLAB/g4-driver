//! グローバル共有状態管理
//!
//! タスク間で共有される状態をMutexで保護して管理します。

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

use crate::can_protocol::MotorStatus;
use crate::config::{StoredConfig, DEFAULT_SPEED_KI, DEFAULT_SPEED_KP};
use crate::foc::{CalibrationResult, ControlMode};
use crate::voltage_monitor::VoltageMonitorState;

/// 目標速度 [RPM]
/// デバッグ用: 起動時に1000 RPMに設定
pub static TARGET_SPEED: Mutex<ThreadModeRawMutex, f32> = Mutex::new(1000.0);

/// 速度PIコントローラのゲイン (Kp, Ki)
pub static SPEED_PI_GAINS: Mutex<ThreadModeRawMutex, (f32, f32)> =
    Mutex::new((DEFAULT_SPEED_KP, DEFAULT_SPEED_KI));

/// モーター有効/無効フラグ
/// デバッグ用: 起動時に有効化
pub static MOTOR_ENABLE: Mutex<ThreadModeRawMutex, bool> = Mutex::new(true);

/// モーターステータス（CAN送信用）
pub static MOTOR_STATUS: Mutex<ThreadModeRawMutex, MotorStatus> = Mutex::new(MotorStatus::new());

/// 電圧監視ステータス（CAN送信用）
pub static VOLTAGE_STATE: Mutex<ThreadModeRawMutex, VoltageMonitorState> =
    Mutex::new(VoltageMonitorState::new());

/// ランタイム設定（フラッシュから読み込まれた設定）
pub static RUNTIME_CONFIG: Mutex<ThreadModeRawMutex, StoredConfig> =
    Mutex::new(StoredConfig::default());

/// 設定バージョン番号（CAN送信用）
pub static CONFIG_VERSION: Mutex<ThreadModeRawMutex, u16> = Mutex::new(0);

/// CRC検証フラグ（CAN送信用）
pub static CONFIG_CRC_VALID: Mutex<ThreadModeRawMutex, bool> = Mutex::new(false);

/// モーター制御モード（ClosedLoopFoc / Calibration等）
pub static CONTROL_MODE: Mutex<ThreadModeRawMutex, ControlMode> =
    Mutex::new(ControlMode::ClosedLoopFoc);

/// キャリブレーション開始フラグ
pub static CALIBRATION_REQUEST: Mutex<ThreadModeRawMutex, bool> = Mutex::new(false);

/// キャリブレーション結果
pub static CALIBRATION_RESULT: Mutex<ThreadModeRawMutex, CalibrationResult> =
    Mutex::new(CalibrationResult {
        electrical_offset: 0.0,
        direction_inversed: false,
        success: false,
    });
