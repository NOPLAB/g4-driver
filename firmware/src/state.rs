//! グローバル共有状態管理
//!
//! タスク間で共有される状態をMutexで保護して管理します。
//! 状態は論理的にグループ化されたコンテキストに整理されています。

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

use crate::can_protocol::MotorStatus;
use crate::config::{StoredConfig, DEFAULT_SPEED_KI, DEFAULT_SPEED_KP};
use crate::foc::{CalibrationResult, ControlMode};
use crate::voltage_monitor::VoltageMonitorState;

/// モーター制御コンテキスト
///
/// モーター制御に関連する全ての状態を一つの構造体にまとめます。
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub struct MotorContext {
    /// 目標速度 [RPM]
    pub target_speed: f32,
    /// 速度PIコントローラのゲイン (Kp, Ki)
    pub pi_gains: (f32, f32),
    /// モーター有効/無効フラグ
    pub enabled: bool,
    /// モーターステータス（CAN送信用）
    pub status: MotorStatus,
    /// モーター制御モード（ClosedLoopFoc / Calibration等）
    pub control_mode: ControlMode,
}

impl MotorContext {
    /// デフォルト値で新しいモーターコンテキストを作成
    #[allow(dead_code)]
    pub const fn new() -> Self {
        Self {
            target_speed: 2000.0, // デバッグ用: 起動時に2000 RPM
            pi_gains: (DEFAULT_SPEED_KP, DEFAULT_SPEED_KI),
            enabled: true, // デバッグ用: 起動時に有効化
            status: MotorStatus::new(),
            control_mode: ControlMode::OpenLoop,
        }
    }
}

/// キャリブレーションコンテキスト
///
/// キャリブレーションに関連する全ての状態を一つの構造体にまとめます。
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub struct CalibrationContext {
    /// キャリブレーション開始フラグ
    pub request: bool,
    /// キャリブレーション用トルク値 (0-100)
    pub torque: u8,
    /// キャリブレーション結果
    pub result: CalibrationResult,
}

impl CalibrationContext {
    /// デフォルト値で新しいキャリブレーションコンテキストを作成
    #[allow(dead_code)]
    pub const fn new() -> Self {
        Self {
            request: false,
            torque: 10,
            result: CalibrationResult {
                electrical_offset: 0.0,
                direction_inversed: false,
                success: false,
            },
        }
    }
}

/// システムコンテキスト
///
/// システム全体の状態を一つの構造体にまとめます。
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub struct SystemContext {
    /// 電圧監視ステータス（CAN送信用）
    pub voltage_state: VoltageMonitorState,
    /// ランタイム設定（フラッシュから読み込まれた設定）
    pub runtime_config: StoredConfig,
    /// 設定バージョン番号（CAN送信用）
    pub config_version: u16,
    /// CRC検証フラグ（CAN送信用）
    pub config_crc_valid: bool,
}

impl SystemContext {
    /// デフォルト値で新しいシステムコンテキストを作成
    #[allow(dead_code)]
    pub const fn new() -> Self {
        Self {
            voltage_state: VoltageMonitorState::new(),
            runtime_config: StoredConfig::default(),
            config_version: 0,
            config_crc_valid: false,
        }
    }
}

/// グローバルモーターコンテキスト
#[allow(dead_code)]
pub static MOTOR_CONTEXT: Mutex<ThreadModeRawMutex, MotorContext> = Mutex::new(MotorContext::new());

/// グローバルキャリブレーションコンテキスト
#[allow(dead_code)]
pub static CALIBRATION_CONTEXT: Mutex<ThreadModeRawMutex, CalibrationContext> =
    Mutex::new(CalibrationContext::new());

/// グローバルシステムコンテキスト
#[allow(dead_code)]
pub static SYSTEM_CONTEXT: Mutex<ThreadModeRawMutex, SystemContext> =
    Mutex::new(SystemContext::new());

// ========================================
// 後方互換性のための旧API
// 段階的な移行のため、既存のAPIを維持
// ========================================

/// 目標速度 [RPM]
/// デバッグ用: 起動時に2000 RPMに設定
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

/// キャリブレーション用トルク値 (0-100)
pub static CALIBRATION_TORQUE: Mutex<ThreadModeRawMutex, u8> = Mutex::new(10);

/// キャリブレーション結果
pub static CALIBRATION_RESULT: Mutex<ThreadModeRawMutex, CalibrationResult> =
    Mutex::new(CalibrationResult {
        electrical_offset: 0.0,
        direction_inversed: false,
        success: false,
    });
