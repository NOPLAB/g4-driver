//! グローバル共有状態管理
//!
//! タスク間で共有される状態をMutexで保護して管理します。
//! 状態は論理的にグループ化されたコンテキストに整理されています。
//!
//! ## 構造
//! - `MotorState`: 制御設定・キャリブレーション・システム設定をMutexで保護
//! - `RuntimeCounters`: 制御ループ用カウンタをAtomicでロックフリー管理

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

use bldc::calibration::CalibrationResult;

use crate::config::{speed, StoredConfig};
use crate::voltage_monitor::VoltageMonitorState;

// ========================================
// モーター制御モード
// ========================================

/// モーター制御モード
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlMode {
    /// オープンループ強制転流（始動時）
    OpenLoop,
    /// クローズドループFOC制御（通常運転）
    ClosedLoopFoc,
    /// キャリブレーションモード（電気角オフセット・回転方向の自動検出）
    Calibration,
}

// ========================================
// 統合状態構造体: MotorState
// ========================================

/// 制御パラメータ
#[derive(Clone, Copy)]
pub struct ControlParams {
    /// 目標速度 [RPM]
    pub target_speed: f32,
    /// 速度PIコントローラのゲイン (Kp, Ki)
    pub pi_gains: (f32, f32),
    /// モーター有効/無効フラグ
    pub enabled: bool,
    /// モーター制御モード
    pub control_mode: ControlMode,
}

impl ControlParams {
    /// デフォルト値で新しい制御パラメータを作成
    pub const fn new() -> Self {
        Self {
            target_speed: 3000.0,
            pi_gains: (speed::DEFAULT_KP, speed::DEFAULT_KI),
            enabled: true,
            control_mode: ControlMode::OpenLoop,
        }
    }
}

/// キャリブレーションパラメータ
#[derive(Clone, Copy)]
pub struct CalibrationParams {
    /// キャリブレーション開始フラグ
    pub request: bool,
    /// キャリブレーション用トルク値 (0-100)
    pub torque: u8,
    /// キャリブレーション結果
    pub result: CalibrationResult,
}

impl CalibrationParams {
    /// デフォルト値で新しいキャリブレーションパラメータを作成
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

/// システムパラメータ
#[derive(Clone, Copy)]
pub struct SystemParams {
    /// 電圧監視ステータス（CAN送信用）
    pub voltage_state: VoltageMonitorState,
    /// ランタイム設定（フラッシュから読み込まれた設定）
    pub runtime_config: StoredConfig,
    /// 設定バージョン番号（CAN送信用）
    pub config_version: u16,
    /// CRC検証フラグ（CAN送信用）
    pub config_crc_valid: bool,
}

impl SystemParams {
    /// デフォルト値で新しいシステムパラメータを作成
    pub const fn new() -> Self {
        Self {
            voltage_state: VoltageMonitorState::new(),
            runtime_config: StoredConfig::default(),
            config_version: 0,
            config_crc_valid: false,
        }
    }
}

/// モーター状態（統合）
///
/// 制御・キャリブレーション・システムの全状態を1つの構造体に統合
#[derive(Clone, Copy)]
pub struct MotorState {
    /// 制御パラメータ
    pub control: ControlParams,
    /// キャリブレーションパラメータ
    pub calibration: CalibrationParams,
    /// システムパラメータ
    pub system: SystemParams,
}

impl MotorState {
    /// デフォルト値で新しいモーター状態を作成
    pub const fn new() -> Self {
        Self {
            control: ControlParams::new(),
            calibration: CalibrationParams::new(),
            system: SystemParams::new(),
        }
    }
}

/// グローバルモーター状態（統合）
pub static MOTOR_STATE: Mutex<ThreadModeRawMutex, MotorState> = Mutex::new(MotorState::new());

// ========================================
// RuntimeCounters: ロックフリーカウンタ
// ========================================

/// FOC制御用カウンタ
pub struct FocCounters {
    /// 脱調（速度低下）の連続カウンタ
    stall_counter: AtomicU32,
    /// 無効Hall状態の連続カウンタ
    invalid_hall_counter: AtomicU32,
}

impl FocCounters {
    /// 新しいFocCountersを作成
    pub const fn new() -> Self {
        Self {
            stall_counter: AtomicU32::new(0),
            invalid_hall_counter: AtomicU32::new(0),
        }
    }

    /// 脱調カウンタをインクリメントし、新しい値を返す
    #[inline(always)]
    pub fn increment_stall(&self) -> u32 {
        self.stall_counter.fetch_add(1, Ordering::Relaxed) + 1
    }

    /// 脱調カウンタをリセット
    #[inline(always)]
    pub fn reset_stall(&self) {
        self.stall_counter.store(0, Ordering::Relaxed);
    }

    /// 無効Hallカウンタをインクリメントし、新しい値を返す
    #[inline(always)]
    pub fn increment_invalid_hall(&self) -> u32 {
        self.invalid_hall_counter.fetch_add(1, Ordering::Relaxed) + 1
    }

    /// 無効Hallカウンタをリセット
    #[inline(always)]
    pub fn reset_invalid_hall(&self) {
        self.invalid_hall_counter.store(0, Ordering::Relaxed);
    }

    /// 全カウンタをリセット
    #[inline(always)]
    pub fn reset_all(&self) {
        self.stall_counter.store(0, Ordering::Relaxed);
        self.invalid_hall_counter.store(0, Ordering::Relaxed);
    }
}

/// ステータス出力（CAN送信用）
pub struct StatusOutput {
    /// モーター速度（RPM）のビット表現
    speed_rpm_bits: AtomicU32,
    /// 電気角（ラジアン）のビット表現
    electrical_angle_bits: AtomicU32,
}

impl StatusOutput {
    /// 新しいStatusOutputを作成
    pub const fn new() -> Self {
        Self {
            speed_rpm_bits: AtomicU32::new(0),
            electrical_angle_bits: AtomicU32::new(0),
        }
    }

    /// ステータスを更新
    #[inline(always)]
    pub fn update(&self, speed_rpm: f32, electrical_angle: f32) {
        self.speed_rpm_bits
            .store(speed_rpm.to_bits(), Ordering::Relaxed);
        self.electrical_angle_bits
            .store(electrical_angle.to_bits(), Ordering::Relaxed);
    }

    /// ステータスを取得
    #[inline(always)]
    #[allow(dead_code)]
    pub fn get(&self) -> (f32, f32) {
        (
            f32::from_bits(self.speed_rpm_bits.load(Ordering::Relaxed)),
            f32::from_bits(self.electrical_angle_bits.load(Ordering::Relaxed)),
        )
    }
}

/// OpenLoop制御用カウンタ
pub struct OpenLoopCounters {
    /// 実行カウンタ
    execution_counter: AtomicU32,
    /// ログカウンタ
    log_counter: AtomicU32,
    /// 脱調回復モードフラグ（0=通常、1=回復）
    recovery_mode: AtomicU32,
}

impl OpenLoopCounters {
    /// 新しいOpenLoopCountersを作成
    pub const fn new() -> Self {
        Self {
            execution_counter: AtomicU32::new(0),
            log_counter: AtomicU32::new(0),
            recovery_mode: AtomicU32::new(0),
        }
    }

    /// 実行カウンタをインクリメントし、前の値を返す
    #[inline(always)]
    pub fn increment_execution(&self) -> u32 {
        self.execution_counter.fetch_add(1, Ordering::Relaxed)
    }

    /// ログカウンタをインクリメントし、前の値を返す
    #[inline(always)]
    pub fn increment_log(&self) -> u32 {
        self.log_counter.fetch_add(1, Ordering::Relaxed)
    }

    /// ログカウンタをリセット
    #[inline(always)]
    pub fn reset_log(&self) {
        self.log_counter.store(0, Ordering::Relaxed);
    }

    /// 回復モードかどうかを取得
    #[inline(always)]
    pub fn is_recovery(&self) -> bool {
        self.recovery_mode.load(Ordering::Relaxed) != 0
    }

    /// 通常起動用リセット
    #[inline(always)]
    pub fn reset_for_normal(&self) {
        self.execution_counter.store(0, Ordering::Relaxed);
        self.log_counter.store(0, Ordering::Relaxed);
        self.recovery_mode.store(0, Ordering::Relaxed);
    }

    /// 脱調回復用リセット
    #[inline(always)]
    pub fn reset_for_recovery(&self) {
        self.execution_counter.store(0, Ordering::Relaxed);
        self.log_counter.store(0, Ordering::Relaxed);
        self.recovery_mode.store(1, Ordering::Relaxed);
    }
}

/// 制御ループ用ランタイムカウンタ（ロックフリー）
pub struct RuntimeCounters {
    /// FOC関連カウンタ
    pub foc: FocCounters,
    /// OpenLoop関連カウンタ
    pub openloop: OpenLoopCounters,
    /// ステータス出力
    pub status: StatusOutput,
}

impl RuntimeCounters {
    /// 新しいRuntimeCountersを作成
    pub const fn new() -> Self {
        Self {
            foc: FocCounters::new(),
            openloop: OpenLoopCounters::new(),
            status: StatusOutput::new(),
        }
    }
}

/// グローバルランタイムカウンタ
pub static RUNTIME: RuntimeCounters = RuntimeCounters::new();

// ========================================
// 直接アクセス関数
// ========================================

pub use embassy_sync::mutex::MutexGuard;

/// モーター状態への直接アクセスを取得
///
/// # Example
/// ```ignore
/// {
///     let mut state = state::motor_state().await;
///     let speed = state.control.target_speed;
///     state.control.target_speed = 3000.0;
/// }
/// ```
#[allow(dead_code)]
pub async fn motor_state() -> MutexGuard<'static, ThreadModeRawMutex, MotorState> {
    MOTOR_STATE.lock().await
}

/// 制御パラメータへのアクセス（motor_contextの代替）
pub async fn motor_context() -> MotorControlGuard {
    MotorControlGuard(MOTOR_STATE.lock().await)
}

/// キャリブレーションパラメータへのアクセス（calibration_contextの代替）
pub async fn calibration_context() -> CalibrationGuard {
    CalibrationGuard(MOTOR_STATE.lock().await)
}

/// システムパラメータへのアクセス（system_contextの代替）
pub async fn system_context() -> SystemGuard {
    SystemGuard(MOTOR_STATE.lock().await)
}

/// 制御パラメータへのアクセスを提供するガード
pub struct MotorControlGuard(MutexGuard<'static, ThreadModeRawMutex, MotorState>);

impl core::ops::Deref for MotorControlGuard {
    type Target = ControlParams;

    fn deref(&self) -> &Self::Target {
        &self.0.control
    }
}

impl core::ops::DerefMut for MotorControlGuard {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0.control
    }
}

/// キャリブレーションパラメータへのアクセスを提供するガード
pub struct CalibrationGuard(MutexGuard<'static, ThreadModeRawMutex, MotorState>);

impl core::ops::Deref for CalibrationGuard {
    type Target = CalibrationParams;

    fn deref(&self) -> &Self::Target {
        &self.0.calibration
    }
}

impl core::ops::DerefMut for CalibrationGuard {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0.calibration
    }
}

/// システムパラメータへのアクセスを提供するガード
pub struct SystemGuard(MutexGuard<'static, ThreadModeRawMutex, MotorState>);

impl core::ops::Deref for SystemGuard {
    type Target = SystemParams;

    fn deref(&self) -> &Self::Target {
        &self.0.system
    }
}

impl core::ops::DerefMut for SystemGuard {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0.system
    }
}

// ========================================
// 複合操作ヘルパー関数
// ========================================

/// モーターを緊急停止（有効フラグをfalseにし、目標速度を0に設定）
pub async fn emergency_stop() {
    let mut state = MOTOR_STATE.lock().await;
    state.control.enabled = false;
    state.control.target_speed = 0.0;
}

/// システム設定を一括更新
pub async fn update_system_config(config: StoredConfig, version: u16, crc_valid: bool) {
    let mut state = MOTOR_STATE.lock().await;
    state.system.runtime_config = config;
    state.system.config_version = version;
    state.system.config_crc_valid = crc_valid;
}

/// キャリブレーション結果をランタイム設定から適用
pub async fn apply_calibration_from_config(config: &StoredConfig) {
    let mut state = MOTOR_STATE.lock().await;
    state.calibration.result.electrical_offset = config.calibration_electrical_offset;
    state.calibration.result.direction_inversed = config.calibration_direction_inversed;
    state.calibration.result.success = config.calibration_success;
}

// ========================================
// Atomic ステータス更新関数（高速制御ループ用）
// ========================================

/// モーターステータスをAtomic変数で更新（ロックフリー）
///
/// FOC/OpenLoopの制御ループ内で高頻度に呼び出されるため、
/// Mutexのオーバーヘッドを回避するためにAtomic変数を使用します。
#[inline(always)]
#[allow(dead_code)]
pub fn update_motor_status_atomic(speed_rpm: f32, electrical_angle: f32) {
    RUNTIME.status.update(speed_rpm, electrical_angle);
}

/// モーターステータスをAtomic変数から取得（ロックフリー）
///
/// CAN送信タスクなど、ステータスを読み取るタスクで使用します。
#[inline(always)]
#[allow(dead_code)]
pub fn get_motor_status_atomic() -> (f32, f32) {
    RUNTIME.status.get()
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

/// FOC制御ループの入力パラメータを一括取得（1回のMutexロックで統合）
///
/// 従来は `get_target_speed()` と `get_pi_gains()` を個別に呼び出していたが、
/// この関数で1回のMutexロックに統合することでオーバーヘッドを削減します。
pub async fn get_foc_input_params() -> FocInputParams {
    let state = MOTOR_STATE.lock().await;
    FocInputParams {
        target_speed: state.control.target_speed,
        pi_gains: state.control.pi_gains,
    }
}
