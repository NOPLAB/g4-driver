//! グローバル共有状態管理
//!
//! タスク間で共有される状態をMutexで保護して管理します。
//! 状態は論理的にグループ化されたコンテキストに整理されています。
//!
//! パフォーマンス最適化:
//! - ステータス更新はAtomic変数を使用（FOC/OpenLoopの高頻度更新用）
//! - 複合操作関数で複数のMutexロックを1回に統合

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

use crate::can_protocol::MotorStatus;
use crate::config::{StoredConfig, DEFAULT_SPEED_KI, DEFAULT_SPEED_KP};
use crate::foc::{CalibrationResult, ControlMode};
use crate::voltage_monitor::VoltageMonitorState;

/// モーター制御コンテキスト
///
/// モーター制御に関連する全ての状態を一つの構造体にまとめます。
#[derive(Clone, Copy)]
pub struct MotorContext {
    /// 目標速度 [RPM]
    pub target_speed: f32,
    /// 速度PIコントローラのゲイン (Kp, Ki)
    pub pi_gains: (f32, f32),
    /// モーター有効/無効フラグ
    pub enabled: bool,
    /// モーターステータス（CAN送信用）
    /// 注: 高速制御ループではAtomic変数を使用（update_motor_status_atomic）
    #[allow(dead_code)]
    pub status: MotorStatus,
    /// モーター制御モード（ClosedLoopFoc / Calibration等）
    pub control_mode: ControlMode,
}

impl MotorContext {
    /// デフォルト値で新しいモーターコンテキストを作成
    pub const fn new() -> Self {
        Self {
            target_speed: 2000.0,
            pi_gains: (DEFAULT_SPEED_KP, DEFAULT_SPEED_KI),
            enabled: true,
            status: MotorStatus::new(),
            control_mode: ControlMode::OpenLoop,
        }
    }
}

/// キャリブレーションコンテキスト
///
/// キャリブレーションに関連する全ての状態を一つの構造体にまとめます。
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
    pub const fn new() -> Self {
        Self {
            request: true,
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
pub static MOTOR_CONTEXT: Mutex<ThreadModeRawMutex, MotorContext> = Mutex::new(MotorContext::new());

// ========================================
// Atomic変数（高速ステータス更新用）
// ========================================
// FOC/OpenLoopの制御ループ内で高頻度にステータスを更新するため、
// Mutexのオーバーヘッドを回避するためにAtomic変数を使用

/// モーター速度（RPM）のビット表現
static STATUS_SPEED_RPM_BITS: AtomicU32 = AtomicU32::new(0);

/// 電気角（ラジアン）のビット表現
static STATUS_ELECTRICAL_ANGLE_BITS: AtomicU32 = AtomicU32::new(0);

/// グローバルキャリブレーションコンテキスト
pub static CALIBRATION_CONTEXT: Mutex<ThreadModeRawMutex, CalibrationContext> =
    Mutex::new(CalibrationContext::new());

/// グローバルシステムコンテキスト
pub static SYSTEM_CONTEXT: Mutex<ThreadModeRawMutex, SystemContext> =
    Mutex::new(SystemContext::new());

// ========================================
// MotorContext ヘルパー関数
// ========================================

/// 目標速度を取得
#[allow(dead_code)]
pub async fn get_target_speed() -> f32 {
    MOTOR_CONTEXT.lock().await.target_speed
}

/// 目標速度を設定
pub async fn set_target_speed(speed: f32) {
    MOTOR_CONTEXT.lock().await.target_speed = speed;
}

/// PIゲインを取得
#[allow(dead_code)]
pub async fn get_pi_gains() -> (f32, f32) {
    MOTOR_CONTEXT.lock().await.pi_gains
}

/// PIゲインを設定
pub async fn set_pi_gains(kp: f32, ki: f32) {
    MOTOR_CONTEXT.lock().await.pi_gains = (kp, ki);
}

/// モーター有効フラグを取得
pub async fn get_motor_enabled() -> bool {
    MOTOR_CONTEXT.lock().await.enabled
}

/// モーター有効フラグを設定
pub async fn set_motor_enabled(enabled: bool) {
    MOTOR_CONTEXT.lock().await.enabled = enabled;
}

/// モーターステータスを取得
#[allow(dead_code)]
pub async fn get_motor_status() -> MotorStatus {
    MOTOR_CONTEXT.lock().await.status
}

/// 制御モードを設定
pub async fn set_control_mode(mode: ControlMode) {
    MOTOR_CONTEXT.lock().await.control_mode = mode;
}

// ========================================
// CalibrationContext ヘルパー関数
// ========================================

/// キャリブレーションリクエストを取得
pub async fn get_calibration_request() -> bool {
    CALIBRATION_CONTEXT.lock().await.request
}

/// キャリブレーションリクエストを設定
pub async fn set_calibration_request(request: bool) {
    CALIBRATION_CONTEXT.lock().await.request = request;
}

/// キャリブレーショントルクを取得
pub async fn get_calibration_torque() -> u8 {
    CALIBRATION_CONTEXT.lock().await.torque
}

/// キャリブレーショントルクを設定
pub async fn set_calibration_torque(torque: u8) {
    CALIBRATION_CONTEXT.lock().await.torque = torque;
}

/// キャリブレーション結果を取得
pub async fn get_calibration_result() -> CalibrationResult {
    CALIBRATION_CONTEXT.lock().await.result
}

/// キャリブレーション結果を設定
pub async fn set_calibration_result(result: CalibrationResult) {
    CALIBRATION_CONTEXT.lock().await.result = result;
}

// ========================================
// SystemContext ヘルパー関数
// ========================================

/// 電圧状態を取得
#[allow(dead_code)]
pub async fn get_voltage_state() -> VoltageMonitorState {
    SYSTEM_CONTEXT.lock().await.voltage_state
}

/// 電圧状態を設定
pub async fn set_voltage_state(state: VoltageMonitorState) {
    SYSTEM_CONTEXT.lock().await.voltage_state = state;
}

/// ランタイム設定を取得
pub async fn get_runtime_config() -> StoredConfig {
    SYSTEM_CONTEXT.lock().await.runtime_config
}

/// 設定バージョンを取得
#[allow(dead_code)]
pub async fn get_config_version() -> u16 {
    SYSTEM_CONTEXT.lock().await.config_version
}

/// CRC検証フラグを取得
#[allow(dead_code)]
pub async fn get_config_crc_valid() -> bool {
    SYSTEM_CONTEXT.lock().await.config_crc_valid
}

/// CRC検証フラグを設定
pub async fn set_config_crc_valid(valid: bool) {
    SYSTEM_CONTEXT.lock().await.config_crc_valid = valid;
}

// ========================================
// 複合操作ヘルパー関数
// ========================================

/// モーターを緊急停止（有効フラグをfalseにし、目標速度を0に設定）
pub async fn emergency_stop() {
    let mut ctx = MOTOR_CONTEXT.lock().await;
    ctx.enabled = false;
    ctx.target_speed = 0.0;
}

/// システム設定を一括更新
pub async fn update_system_config(config: StoredConfig, version: u16, crc_valid: bool) {
    let mut ctx = SYSTEM_CONTEXT.lock().await;
    ctx.runtime_config = config;
    ctx.config_version = version;
    ctx.config_crc_valid = crc_valid;
}

/// キャリブレーション結果をランタイム設定から適用
pub async fn apply_calibration_from_config(config: &StoredConfig) {
    let mut ctx = CALIBRATION_CONTEXT.lock().await;
    ctx.result.electrical_offset = config.calibration_electrical_offset;
    ctx.result.direction_inversed = config.calibration_direction_inversed;
    ctx.result.success = config.calibration_success;
}

// ========================================
// Atomic ステータス更新関数（高速制御ループ用）
// ========================================

/// モーターステータスをAtomic変数で更新（ロックフリー）
///
/// FOC/OpenLoopの制御ループ内で高頻度に呼び出されるため、
/// Mutexのオーバーヘッドを回避するためにAtomic変数を使用します。
#[inline(always)]
pub fn update_motor_status_atomic(speed_rpm: f32, electrical_angle: f32) {
    STATUS_SPEED_RPM_BITS.store(speed_rpm.to_bits(), Ordering::Relaxed);
    STATUS_ELECTRICAL_ANGLE_BITS.store(electrical_angle.to_bits(), Ordering::Relaxed);
}

/// モーターステータスをAtomic変数から取得（ロックフリー）
///
/// CAN送信タスクなど、ステータスを読み取るタスクで使用します。
#[inline(always)]
#[allow(dead_code)]
pub fn get_motor_status_atomic() -> (f32, f32) {
    (
        f32::from_bits(STATUS_SPEED_RPM_BITS.load(Ordering::Relaxed)),
        f32::from_bits(STATUS_ELECTRICAL_ANGLE_BITS.load(Ordering::Relaxed)),
    )
}

// ========================================
// FOC制御ループ用複合操作関数
// ========================================

/// FOC制御ループで必要な入力パラメータ
#[derive(Clone, Copy)]
pub struct FocInputParams {
    /// 目標速度 [RPM]
    pub target_speed: f32,
    /// PIゲイン (Kp, Ki)
    pub pi_gains: (f32, f32),
}

/// FOC制御ループの入力パラメータを一括取得（1回のロックで複数値取得）
///
/// 従来は `get_target_speed()` と `get_pi_gains()` を個別に呼び出していたが、
/// この関数で1回のMutexロックに統合することでオーバーヘッドを削減します。
pub async fn get_foc_input_params() -> FocInputParams {
    let ctx = MOTOR_CONTEXT.lock().await;
    FocInputParams {
        target_speed: ctx.target_speed,
        pi_gains: ctx.pi_gains,
    }
}
