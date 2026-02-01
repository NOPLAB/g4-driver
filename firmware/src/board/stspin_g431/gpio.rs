//! GPIO初期化モジュール
//!
//! LED、ドライバーイネーブル等のGPIO初期化

use embassy_stm32::{
    gpio::{Level, Output, Speed},
    Peri,
};

use super::LedPins;

/// ドライバーイネーブルピンを初期化（PE7: Active High）
pub fn init_driver_enable(pin: Peri<'static, embassy_stm32::peripherals::PE7>) -> Output<'static> {
    Output::new(pin, Level::High, Speed::Low)
}

/// LEDピンを初期化（PC13/PC14/PC15）
pub fn init_leds(
    pc13: Peri<'static, embassy_stm32::peripherals::PC13>,
    pc14: Peri<'static, embassy_stm32::peripherals::PC14>,
    pc15: Peri<'static, embassy_stm32::peripherals::PC15>,
) -> LedPins<'static> {
    LedPins {
        led1: Output::new(pc13, Level::High, Speed::Low),
        led2: Output::new(pc14, Level::High, Speed::Low),
        led3: Output::new(pc15, Level::High, Speed::Low),
    }
}
