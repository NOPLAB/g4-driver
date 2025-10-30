//! DC Bus Voltage Monitoring
//!
//! M1_BUS_VOLTAGE (PC1ピン) からDCバス電圧を監視し、過電圧/低電圧保護を提供します。

use crate::fmt::*;

/// 電圧監視パラメータ
pub struct VoltageMonitorConfig {
    /// 分圧抵抗の上側 [Ω]
    pub r_upper: f32,
    /// 分圧抵抗の下側 [Ω]
    pub r_lower: f32,
    /// ADC分解能（12ビット = 4096）
    pub adc_max: u16,
    /// ADC基準電圧 [V]
    pub vref: f32,
    /// ローパスフィルタ係数（0.0-1.0、大きいほど応答速度が速い）
    pub filter_alpha: f32,
    /// 過電圧しきい値 [V]
    pub overvoltage_threshold: f32,
    /// 低電圧しきい値 [V]
    pub undervoltage_threshold: f32,
}

impl Default for VoltageMonitorConfig {
    fn default() -> Self {
        Self {
            // 分圧回路: 33.3kΩ + 3.3kΩ（抵抗比 10.09:1、電圧変換係数 11.09倍）
            // V_bus = V_adc * (33.3k + 3.3k) / 3.3k = V_adc * 11.09
            // 最大入力電圧: 3.3V * 11.09 ≈ 36.6V
            r_upper: 33_300.0, // 上側抵抗 33.3kΩ
            r_lower: 3_300.0,  // 下側抵抗 3.3kΩ
            adc_max: 4096,
            vref: 3.3,
            filter_alpha: 0.1,            // 緩やかなフィルタ
            overvoltage_threshold: 30.0,  // 30V以上で過電圧
            undervoltage_threshold: 10.0, // 10V以下で低電圧
        }
    }
}

/// 電圧監視状態
#[derive(Copy, Clone)]
pub struct VoltageMonitorState {
    /// 現在の電圧 [V]（フィルタ済み）
    pub voltage: f32,
    /// 過電圧フラグ
    pub overvoltage: bool,
    /// 低電圧フラグ
    pub undervoltage: bool,
}

impl VoltageMonitorState {
    pub const fn new() -> Self {
        Self {
            voltage: 0.0,
            overvoltage: false,
            undervoltage: false,
        }
    }

    /// 電圧が正常範囲内かチェック
    pub fn is_voltage_ok(&self) -> bool {
        !self.overvoltage && !self.undervoltage
    }
}

/// 電圧監視コントローラ
pub struct VoltageMonitor {
    config: VoltageMonitorConfig,
    state: VoltageMonitorState,
}

impl VoltageMonitor {
    /// 新しい電圧監視コントローラを作成
    pub fn new(config: VoltageMonitorConfig) -> Self {
        Self {
            config,
            state: VoltageMonitorState::new(),
        }
    }

    /// ADC生値から実電圧を計算
    ///
    /// # Arguments
    /// * `adc_raw` - ADC生値（0-4095）
    ///
    /// # Returns
    /// DCバス電圧 [V]
    fn adc_to_voltage(&self, adc_raw: u16) -> f32 {
        // ADC電圧計算: V_adc = (adc_raw / adc_max) * Vref
        let v_adc = (adc_raw as f32 / self.config.adc_max as f32) * self.config.vref;

        // 分圧回路から元の電圧を逆算
        // V_bus = V_adc * (R_upper + R_lower) / R_lower
        let divider_ratio = (self.config.r_upper + self.config.r_lower) / self.config.r_lower;
        v_adc * divider_ratio
    }

    /// 電圧を更新し、過電圧/低電圧をチェック
    ///
    /// # Arguments
    /// * `adc_raw` - ADC生値（0-4095）
    ///
    /// # Returns
    /// 更新後の電圧監視状態
    pub fn update(&mut self, adc_raw: u16) -> VoltageMonitorState {
        // ADC生値から実電圧を計算
        let voltage_raw = self.adc_to_voltage(adc_raw);

        // ローパスフィルタ適用
        // filtered = alpha * raw + (1 - alpha) * filtered_prev
        self.state.voltage = self.config.filter_alpha * voltage_raw
            + (1.0 - self.config.filter_alpha) * self.state.voltage;

        // 過電圧/低電圧チェック
        self.state.overvoltage = self.state.voltage > self.config.overvoltage_threshold;
        self.state.undervoltage = self.state.voltage < self.config.undervoltage_threshold;

        // 異常検出時のログ
        if self.state.overvoltage {
            error!(
                "OVERVOLTAGE detected! Bus voltage: {}V (threshold: {}V)",
                self.state.voltage, self.config.overvoltage_threshold
            );
        }
        if self.state.undervoltage {
            error!(
                "UNDERVOLTAGE detected! Bus voltage: {}V (threshold: {}V)",
                self.state.voltage, self.config.undervoltage_threshold
            );
        }

        self.state
    }

    /// 現在の状態を取得
    #[allow(dead_code)]
    pub fn get_state(&self) -> VoltageMonitorState {
        self.state
    }

    /// 現在の電圧を取得 [V]
    #[allow(dead_code)]
    pub fn get_voltage(&self) -> f32 {
        self.state.voltage
    }

    /// しきい値を更新
    #[allow(dead_code)]
    pub fn set_thresholds(&mut self, overvoltage: f32, undervoltage: f32) {
        self.config.overvoltage_threshold = overvoltage;
        self.config.undervoltage_threshold = undervoltage;
        info!(
            "Voltage thresholds updated: OV={}V, UV={}V",
            overvoltage, undervoltage
        );
    }

    /// フィルタ係数を更新
    #[allow(dead_code)]
    pub fn set_filter_alpha(&mut self, alpha: f32) {
        self.config.filter_alpha = alpha.clamp(0.0, 1.0);
    }

    /// リセット（フィルタ状態をクリア）
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.state = VoltageMonitorState::new();
    }

    /// フィルタを初期電圧で初期化（起動時の誤検出防止用）
    pub fn initialize_with_adc(&mut self, adc_raw: u16) {
        let voltage = self.adc_to_voltage(adc_raw);
        self.state.voltage = voltage;
        self.state.overvoltage = voltage > self.config.overvoltage_threshold;
        self.state.undervoltage = voltage < self.config.undervoltage_threshold;
    }
}
