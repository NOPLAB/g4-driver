//! モーター制御とハードウェアの設定パラメータ

/// モーター制御パラメータ（デフォルト値）
/// 低速FOC起動のため、ゲインと最小電圧を最適化
pub const DEFAULT_SPEED_KP: f32 = 0.2; // 比例ゲイン（低速応答性向上）
pub const DEFAULT_SPEED_KI: f32 = 0.05; // 積分ゲイン（定常偏差低減）

/// 最大電圧 [V]（デフォルト値）
pub const DEFAULT_MAX_VOLTAGE: f32 = 24.0;

/// DCバス電圧 [V]（デフォルト値）
pub const DEFAULT_V_DC_BUS: f32 = 24.0;

/// モーターの極対数（ポール数12 / 2 = 6）（デフォルト値）
pub const DEFAULT_POLE_PAIRS: u8 = 6;

/// 制御周期 [μs]（10kHz = 100μs）（デフォルト値）
pub const DEFAULT_CONTROL_PERIOD_US: u64 = 100;

/// ホールセンサ速度フィルタ係数（foc-simple互換: α=0.05でより滑らかな速度推定）（デフォルト値）
pub const DEFAULT_SPEED_FILTER_ALPHA: f32 = 0.05;

/// Hall角度オフセット [度]（ハードウェアに応じて調整、モーターが正しく回転しない場合は調整が必要）
/// テーブル方式で基本電気角を設定済み。オフセットは微調整用。
pub const DEFAULT_HALL_ANGLE_OFFSET_DEG: f32 = 0.0;

/// 進角（Advance Angle）設定
/// 高速回転時にトルク効率を最大化するため、電気角に進角を加える
pub mod advance_angle {
    /// 基本進角 [度]（低速時から適用される固定進角）
    pub const BASE_ADVANCE_DEG: f32 = 10.0;

    /// 最大進角 [度]（高速時の最大進角）
    pub const MAX_ADVANCE_DEG: f32 = 30.0;

    /// 進角が最大になる速度 [RPM]
    pub const MAX_SPEED_FOR_ADVANCE: f32 = 3000.0;

    /// 進角を適用し始める速度 [RPM]（これ以下では基本進角のみ）
    pub const MIN_SPEED_FOR_ADVANCE: f32 = 100.0;
}

/// 最小出力電圧 [V]（静止摩擦を克服するための最小電圧）
pub const MIN_VOLTAGE: f32 = 2.0;

/// 最小電圧適用のしきい値 [RPM]（速度誤差がこの値を超える場合に最小電圧を適用）
pub const MIN_VOLTAGE_ERROR_THRESHOLD: f32 = 2.0;

/// 速度指令の最大加速度 [RPM/s]（急激な速度変化を抑制してPI制御を安定化）
pub const MAX_SPEED_ACCELERATION: f32 = 100.0;

/// FOC脱落検出パラメータ
pub mod foc_stall {
    /// FOC脱落判定の速度閾値 [RPM]
    /// 実測速度がこの値以下になると脱落カウンタが増加
    pub const STALL_SPEED_THRESHOLD: f32 = 50.0;

    /// FOC脱落判定の連続回数閾値
    /// 10kHz制御で1000回 = 100ms以上連続して低速ならOpenLoopに戻る
    pub const STALL_COUNT_THRESHOLD: u32 = 1000;
}

/// オープンループ始動パラメータ（6ステップ駆動）
pub mod openloop {
    /// 初期回転数 [RPM]（起動用：100RPMから開始）
    pub const DEFAULT_INITIAL_RPM: f32 = 100.0;

    /// FOC切替回転数 [RPM]（目標速度）
    pub const DEFAULT_TARGET_RPM: f32 = 300.0;

    /// 加速度 [RPM/s]（起動用：適度な加速）
    pub const DEFAULT_ACCELERATION_RPM_PER_S: f32 = 200.0;

    /// デューティ比 (0-100)（Hall ベース駆動用：15%）
    pub const DEFAULT_DUTY_RATIO: u16 = 15;

    /// 強制転流フェーズの実行回数（10000 = 1秒 @ 10kHz）
    pub const FORCED_COMMUTATION_CYCLES: u32 = 10000;

    /// FOC切り替えまでの最小実行回数（10000 = 1秒 @ 10kHz）
    pub const MIN_CYCLES_BEFORE_FOC: u32 = 10000;
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

/// デッドタイム補償パラメータ
pub mod dead_time_compensation {
    /// 補償有効/無効（デフォルト: 無効）
    pub const ENABLED: bool = false;

    /// 補償対象のデッドタイム [ns]
    pub const DEAD_TIME_NS: f32 = 100.0;
}

/// フラックス弱め制御パラメータ
pub mod flux_weakening {
    /// 制御有効/無効（デフォルト: 無効）
    pub const ENABLED: bool = false;

    /// 弱め制御開始速度 [RPM]
    pub const MIN_SPEED: f32 = 2000.0;

    /// 最大弱め速度 [RPM]
    pub const MAX_SPEED: f32 = 4000.0;

    /// 最大弱め率 (0.0-1.0)
    pub const MAX_WEAKENING_RATIO: f32 = 0.5;

    /// Vdレート制限 [V/s]
    pub const VD_RATE_LIMIT: f32 = 100.0;
}
