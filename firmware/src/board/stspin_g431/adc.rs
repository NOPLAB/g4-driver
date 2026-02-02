//! ADC初期化モジュール
//!
//! ADC2を使用した電圧監視

use embassy_stm32::{
    adc::{Adc, AdcChannel, AdcConfig, AnyAdcChannel},
    peripherals, Peri,
};

/// ADC2と電圧監視ピンを初期化
///
/// # Returns
/// - ADC2インスタンス
/// - 電圧監視ピン（PC1 = ADC2_IN7）
pub fn init_adc(
    adc_peri: Peri<'static, peripherals::ADC2>,
    voltage_pin: Peri<'static, peripherals::PC1>,
) -> (
    Adc<'static, peripherals::ADC2>,
    AnyAdcChannel<'static, peripherals::ADC2>,
) {
    let adc = Adc::new(adc_peri, AdcConfig::default());

    let pin = voltage_pin.degrade_adc();

    (adc, pin)
}
