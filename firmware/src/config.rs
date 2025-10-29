//! モーター制御とハードウェアの設定パラメータ

/// モーター制御パラメータ（デフォルト値）
/// 角度補間により制御精度が向上したため、ゲインを最適化
pub const DEFAULT_SPEED_KP: f32 = 0.5; // 比例ゲイン（オーバーシュート抑制）
pub const DEFAULT_SPEED_KI: f32 = 0.05; // 積分ゲイン（ワインドアップ抑制）

/// 最大電圧 [V]
pub const MAX_VOLTAGE: f32 = 24.0;

/// DCバス電圧 [V]
pub const V_DC_BUS: f32 = 24.0;

/// モーターの極対数（ポール数12 / 2 = 6）
pub const POLE_PAIRS: u8 = 6;

/// 制御周期 [μs]（2.5kHz = 400μs）
pub const CONTROL_PERIOD_US: u64 = 400;

/// 最大デューティ比
pub const MAX_DUTY: u16 = 100;

/// ホールセンサ速度フィルタ係数（滑らかな速度推定のため低減）
pub const SPEED_FILTER_ALPHA: f32 = 0.1;

/// オープンループ始動パラメータ（6ステップ駆動）
pub mod openloop {
    /// 初期回転数 [RPM]
    pub const INITIAL_RPM: f32 = 30.0;

    /// FOC切替回転数 [RPM]
    pub const TARGET_RPM: f32 = 500.0;

    /// 加速度 [RPM/s]
    pub const ACCELERATION_RPM_PER_S: f32 = 50.0;

    /// デューティ比 (0-100)
    pub const DUTY_RATIO: u16 = 50;
}

/// PWM設定
pub mod pwm {
    use embassy_stm32::time::Hertz;

    /// PWM周波数（50kHz）
    pub const FREQUENCY: Hertz = Hertz(50_000);

    /// デッドタイム
    pub const DEAD_TIME: u16 = 1;
}

/// CAN設定
pub mod can {
    /// CANビットレート（250kbps）
    pub const BITRATE: u32 = 250_000;
}
