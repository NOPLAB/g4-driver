//! 設定パラメータの永続化構造体
//!
//! params.rsのすべてのパラメータをフラッシュメモリに保存するための構造体

use super::params;

/// 設定データのマジックナンバー（"CFG1"のASCII）
pub const CONFIG_MAGIC: u32 = 0x31474643;

/// 現在の設定バージョン
pub const CONFIG_VERSION: u16 = 2;

/// 永続化される設定構造体
///
/// すべてのconfig.rsパラメータをこの構造体に含める
/// サイズ制約：2KB（フラッシュページサイズ）以内
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct StoredConfig {
    /// マジックナンバー（データ識別用）
    pub magic: u32,

    /// 設定バージョン番号
    pub version: u16,

    /// パディング（アライメント調整）
    _padding: u16,

    // === モーター制御パラメータ ===
    /// 速度PI制御の比例ゲイン
    pub speed_kp: f32,

    /// 速度PI制御の積分ゲイン
    pub speed_ki: f32,

    /// 最大電圧 [V]
    pub max_voltage: f32,

    /// DCバス電圧 [V]
    pub v_dc_bus: f32,

    /// モーターの極対数
    pub pole_pairs: u8,

    /// 最大デューティ比
    pub max_duty: u16,

    /// Hallセンサ速度フィルタ係数
    pub speed_filter_alpha: f32,

    /// Hallセンサ角度オフセット [rad]
    pub hall_angle_offset: f32,

    /// 角度補間有効フラグ
    pub enable_angle_interpolation: bool,

    /// パディング
    _padding2: [u8; 2],

    // === キャリブレーション結果 ===
    /// キャリブレーション済み電気角オフセット [rad] (0～2π)
    pub calibration_electrical_offset: f32,

    /// キャリブレーション済み方向反転フラグ
    pub calibration_direction_inversed: bool,

    /// キャリブレーション成功フラグ
    pub calibration_success: bool,

    /// パディング
    _padding_calib: [u8; 2],

    // === オープンループ始動パラメータ ===
    /// 初期回転数 [RPM]
    pub openloop_initial_rpm: f32,

    /// FOC切替回転数 [RPM]
    pub openloop_target_rpm: f32,

    /// 加速度 [RPM/s]
    pub openloop_acceleration: f32,

    /// デューティ比 (0-100)
    pub openloop_duty_ratio: u16,

    /// パディング
    _padding3: u16,

    // === PWM設定 ===
    /// PWM周波数 [Hz]
    pub pwm_frequency: u32,

    /// デッドタイム
    pub pwm_dead_time: u16,

    /// パディング
    _padding4: u16,

    // === CAN設定 ===
    /// CANビットレート [bps]
    pub can_bitrate: u32,

    // === 制御タイミング ===
    /// 制御周期 [μs]
    pub control_period_us: u64,

    // === 進角パラメータ ===
    /// 基本進角 [度]
    pub advance_base_deg: f32,
    /// 最大進角 [度]
    pub advance_max_deg: f32,
    /// 進角適用最大速度 [RPM]
    pub advance_max_speed: f32,
    /// 進角適用最小速度 [RPM]
    pub advance_min_speed: f32,

    // === 最小電圧関連 ===
    /// 最小出力電圧 [V]
    pub min_voltage: f32,
    /// 最小電圧適用の速度誤差しきい値 [RPM]
    pub min_voltage_error_threshold: f32,
    /// 速度指令の最大加速度 [RPM/s]
    pub max_speed_acceleration: f32,

    // === FOC脱落検出 ===
    /// FOC脱落判定の速度閾値 [RPM]
    pub foc_stall_speed_threshold: f32,
    /// FOC脱落判定の連続回数閾値
    pub foc_stall_count_threshold: u32,

    // === オープンループサイクル ===
    /// 強制転流フェーズの実行回数
    pub forced_commutation_cycles: u32,
    /// FOC切り替えまでの最小実行回数
    pub min_cycles_before_foc: u32,

    // === デッドタイム補償 ===
    /// 補償有効/無効フラグ
    pub dead_time_comp_enabled: bool,
    /// パディング
    _padding_dtc: [u8; 3],
    /// 補償対象のデッドタイム [ns]
    pub dead_time_ns: f32,

    // === フラックス弱め制御 ===
    /// 制御有効/無効フラグ
    pub flux_weakening_enabled: bool,
    /// パディング
    _padding_fw: [u8; 3],
    /// 弱め制御開始速度 [RPM]
    pub flux_weakening_min_speed: f32,
    /// 最大弱め速度 [RPM]
    pub flux_weakening_max_speed: f32,
    /// 最大弱め率 (0.0-1.0)
    pub flux_weakening_max_ratio: f32,
    /// Vdレート制限 [V/s]
    pub flux_weakening_vd_rate_limit: f32,

    // === 電圧監視 ===
    /// 過電圧しきい値 [V]
    pub voltage_overvoltage_threshold: f32,
    /// 低電圧しきい値 [V]
    pub voltage_undervoltage_threshold: f32,
    /// ローパスフィルタ係数
    pub voltage_filter_alpha: f32,

    /// CRC32チェックサム（最後に配置）
    pub crc32: u32,
}

impl StoredConfig {
    /// デフォルト設定を生成（params.rsの値を使用）
    pub const fn default() -> Self {
        Self {
            magic: CONFIG_MAGIC,
            version: CONFIG_VERSION,
            _padding: 0,
            speed_kp: params::speed::DEFAULT_KP,
            speed_ki: params::speed::DEFAULT_KI,
            max_voltage: params::voltage::DEFAULT_MAX,
            v_dc_bus: params::voltage::DEFAULT_DC_BUS,
            pole_pairs: params::motor::DEFAULT_POLE_PAIRS,
            max_duty: 100, // 注: この値は使用されず、実行時にPWMから取得される
            speed_filter_alpha: params::speed::DEFAULT_FILTER_ALPHA,
            hall_angle_offset: params::hall::DEFAULT_ANGLE_OFFSET_DEG, // デフォルトはオフセットなし
            enable_angle_interpolation: true,                          // デフォルトで有効
            _padding2: [0; 2],
            calibration_electrical_offset: 0.0, // キャリブレーション未実施
            calibration_direction_inversed: false,
            calibration_success: false,
            _padding_calib: [0; 2],
            openloop_initial_rpm: params::openloop::DEFAULT_INITIAL_RPM,
            openloop_target_rpm: params::openloop::DEFAULT_TARGET_RPM,
            openloop_acceleration: params::openloop::DEFAULT_ACCELERATION,
            openloop_duty_ratio: params::openloop::DEFAULT_DUTY_RATIO,
            _padding3: 0,
            pwm_frequency: params::pwm::DEFAULT_FREQUENCY.0,
            pwm_dead_time: params::pwm::DEFAULT_DEAD_TIME,
            _padding4: 0,
            can_bitrate: params::can::DEFAULT_BITRATE,
            control_period_us: params::control::DEFAULT_PERIOD_US,
            // 進角パラメータ
            advance_base_deg: params::advance_angle::BASE_ADVANCE_DEG,
            advance_max_deg: params::advance_angle::MAX_ADVANCE_DEG,
            advance_max_speed: params::advance_angle::MAX_SPEED_FOR_ADVANCE,
            advance_min_speed: params::advance_angle::MIN_SPEED_FOR_ADVANCE,
            // 最小電圧関連
            min_voltage: params::voltage::MIN,
            min_voltage_error_threshold: params::voltage::MIN_ERROR_THRESHOLD,
            max_speed_acceleration: params::speed::MAX_ACCELERATION,
            // FOC脱落検出
            foc_stall_speed_threshold: params::foc_stall::SPEED_THRESHOLD,
            foc_stall_count_threshold: params::foc_stall::COUNT_THRESHOLD,
            // オープンループサイクル
            forced_commutation_cycles: params::openloop::FORCED_COMMUTATION_CYCLES,
            min_cycles_before_foc: params::openloop::MIN_CYCLES_BEFORE_FOC,
            // デッドタイム補償
            dead_time_comp_enabled: params::dead_time_compensation::ENABLED,
            _padding_dtc: [0; 3],
            dead_time_ns: params::dead_time_compensation::DEAD_TIME_NS,
            // フラックス弱め制御
            flux_weakening_enabled: params::flux_weakening::ENABLED,
            _padding_fw: [0; 3],
            flux_weakening_min_speed: params::flux_weakening::MIN_SPEED,
            flux_weakening_max_speed: params::flux_weakening::MAX_SPEED,
            flux_weakening_max_ratio: params::flux_weakening::MAX_WEAKENING_RATIO,
            flux_weakening_vd_rate_limit: params::flux_weakening::VD_RATE_LIMIT,
            // 電圧監視
            voltage_overvoltage_threshold: 30.0,
            voltage_undervoltage_threshold: 10.0,
            voltage_filter_alpha: 0.1,
            crc32: 0, // CRC計算前は0
        }
    }

    /// バイト配列として参照を取得（CRC計算用）
    ///
    /// CRC32フィールドを除くすべてのバイトを返す
    pub fn as_bytes_for_crc(&self) -> &[u8] {
        let ptr = self as *const Self as *const u8;
        let total_size = core::mem::size_of::<Self>();
        let crc_size = core::mem::size_of::<u32>();
        unsafe { core::slice::from_raw_parts(ptr, total_size - crc_size) }
    }

    /// バイト配列として可変参照を取得（シリアライズ用）
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        let ptr = self as *mut Self as *mut u8;
        let size = core::mem::size_of::<Self>();
        unsafe { core::slice::from_raw_parts_mut(ptr, size) }
    }

    /// バイト配列から構造体を復元
    ///
    /// # Safety
    /// バイト配列が正しい構造体レイアウトであることを確認する必要がある
    pub unsafe fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < core::mem::size_of::<Self>() {
            return None;
        }

        let ptr = bytes.as_ptr() as *const Self;
        Some(*ptr)
    }

    /// マジックナンバーとバージョンを検証
    pub fn validate_header(&self) -> bool {
        self.magic == CONFIG_MAGIC && self.version == CONFIG_VERSION
    }

    /// CRC32チェックサムを計算
    ///
    /// # Arguments
    /// * `crc` - embassy-stm32のCRCペリフェラル
    pub fn calculate_crc(&self, crc: &mut embassy_stm32::crc::Crc) -> u32 {
        let data = self.as_bytes_for_crc();

        // 4バイト境界に合わせてデータを準備
        let mut aligned_data = [0u32; 64]; // 最大256バイト分
        let word_count = data.len().div_ceil(4);

        for (i, aligned_word) in aligned_data.iter_mut().enumerate().take(word_count) {
            let offset = i * 4;
            if offset + 4 <= data.len() {
                *aligned_word = u32::from_le_bytes([
                    data[offset],
                    data[offset + 1],
                    data[offset + 2],
                    data[offset + 3],
                ]);
            } else {
                // 最後の不完全なワード
                let mut bytes = [0u8; 4];
                let remaining = data.len() - offset;
                bytes[..remaining].copy_from_slice(&data[offset..offset + remaining]);
                *aligned_word = u32::from_le_bytes(bytes);
            }
        }

        crc.reset();
        crc.feed_words(&aligned_data[..word_count])
    }

    /// CRC32チェックサムを検証
    pub fn verify_crc(&self, crc: &mut embassy_stm32::crc::Crc) -> bool {
        let calculated = self.calculate_crc(crc);
        calculated == self.crc32
    }
}

// コンパイル時サイズチェック（2KB以内であることを確認）
const _: () = {
    const SIZE: usize = core::mem::size_of::<StoredConfig>();
    const MAX_SIZE: usize = 2048; // 2KB
    assert!(
        SIZE <= MAX_SIZE,
        "StoredConfig size exceeds flash page size"
    );
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = StoredConfig::default();
        assert_eq!(config.magic, CONFIG_MAGIC);
        assert_eq!(config.version, CONFIG_VERSION);
        assert_eq!(config.speed_kp, params::speed::DEFAULT_KP);
        assert_eq!(config.speed_ki, params::speed::DEFAULT_KI);
        // Extended parameters
        assert_eq!(
            config.advance_base_deg,
            params::advance_angle::BASE_ADVANCE_DEG
        );
        assert_eq!(
            config.advance_max_deg,
            params::advance_angle::MAX_ADVANCE_DEG
        );
        assert_eq!(config.min_voltage, params::voltage::MIN);
        assert_eq!(
            config.foc_stall_speed_threshold,
            params::foc_stall::SPEED_THRESHOLD
        );
        assert!(!config.dead_time_comp_enabled);
        assert!(!config.flux_weakening_enabled);
    }

    #[test]
    fn test_size_constraint() {
        let size = core::mem::size_of::<StoredConfig>();
        assert!(size <= 2048, "Config size {} exceeds 2KB", size);
    }
}
