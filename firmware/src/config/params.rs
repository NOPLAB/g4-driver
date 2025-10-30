//! モーター制御とハードウェアの設定パラメータ

/// モーター制御パラメータ（デフォルト値）
/// 低速FOC起動のため、ゲインと最小電圧を最適化
pub const DEFAULT_SPEED_KP: f32 = 0.8; // 比例ゲイン（低速応答性向上）
pub const DEFAULT_SPEED_KI: f32 = 0.1; // 積分ゲイン（定常偏差低減）

/// 最大電圧 [V]（デフォルト値）
pub const DEFAULT_MAX_VOLTAGE: f32 = 24.0;

/// DCバス電圧 [V]（デフォルト値）
pub const DEFAULT_V_DC_BUS: f32 = 24.0;

/// モーターの極対数（ポール数12 / 2 = 6）（デフォルト値）
pub const DEFAULT_POLE_PAIRS: u8 = 6;

/// 制御周期 [μs]（2.5kHz = 400μs）（デフォルト値）
pub const DEFAULT_CONTROL_PERIOD_US: u64 = 400;

/// 最大デューティ比（デフォルト値）
pub const DEFAULT_MAX_DUTY: u16 = 100;

/// ホールセンサ速度フィルタ係数（foc-simple互換: α=0.05でより滑らかな速度推定）（デフォルト値）
pub const DEFAULT_SPEED_FILTER_ALPHA: f32 = 0.05;

/// Hall角度オフセット [度]（ハードウェアに応じて調整、モーターが正しく回転しない場合は調整が必要）（デフォルト値）
pub const DEFAULT_HALL_ANGLE_OFFSET_DEG: f32 = 0.0;

/// 最小出力電圧 [V]（静止摩擦を克服するための最小電圧）
pub const MIN_VOLTAGE: f32 = 5.0;

/// 最小電圧適用のしきい値 [RPM]（速度誤差がこの値を超える場合に最小電圧を適用）
pub const MIN_VOLTAGE_ERROR_THRESHOLD: f32 = 10.0;

/// 速度指令の最大加速度 [RPM/s]（急激な速度変化を抑制してPI制御を安定化）
pub const MAX_SPEED_ACCELERATION: f32 = 200.0;

/// オープンループ始動パラメータ（6ステップ駆動）
pub mod openloop {
    /// 初期回転数 [RPM]（十分なトルクを得るため高めに設定）（デフォルト値）
    pub const DEFAULT_INITIAL_RPM: f32 = 50.0;

    /// FOC切替回転数 [RPM]（実際のモーターが追従できる速度）（デフォルト値）
    pub const DEFAULT_TARGET_RPM: f32 = 500.0;

    /// 加速度 [RPM/s]（モーターが追従できる緩やかな加速）（デフォルト値）
    pub const DEFAULT_ACCELERATION_RPM_PER_S: f32 = 50.0;

    /// デューティ比 (0-100)（十分なトルクを確保）（デフォルト値）
    pub const DEFAULT_DUTY_RATIO: u16 = 80;
}

/// PWM設定
pub mod pwm {
    use embassy_stm32::time::Hertz;

    /// PWM周波数（50kHz）（デフォルト値）
    pub const DEFAULT_FREQUENCY: Hertz = Hertz(50_000);

    /// デッドタイム（デフォルト値）
    pub const DEFAULT_DEAD_TIME: u16 = 1;
}

/// CAN設定
pub mod can {
    /// CANビットレート（250kbps）（デフォルト値）
    pub const DEFAULT_BITRATE: u32 = 250_000;
}
