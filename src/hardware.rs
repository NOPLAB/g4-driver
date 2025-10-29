//! ハードウェア初期化モジュール
//!
//! ペリフェラルの初期化ロジックを集約します。

use embassy_stm32::{
    adc::{Adc, AdcChannel, SampleTime},
    bind_interrupts, can,
    gpio::{Level, Output, Speed},
    opamp::{OpAmp, OpAmpSpeed},
    peripherals,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        low_level::CountingMode,
        simple_pwm::PwmPin,
        Channel,
    },
    Config, Peripherals,
};

use crate::config;
use crate::fmt::*;
use crate::hall_tim;

// CANの割り込みをバインド
bind_interrupts!(pub struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

/// RCCクロック設定を初期化
///
/// HSI → PLL（÷4 × 85 ÷ 2）で170MHz生成
pub fn create_clock_config() -> Config {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::mux::{Adcsel, ClockMux, Fdcansel};
        use embassy_stm32::rcc::{Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk};

        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // FDCANクロック用
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R; // システムクロックをPLLに設定

        let mut clock_mux = ClockMux::default();
        clock_mux.adc12sel = Adcsel::SYS;
        clock_mux.fdcansel = Fdcansel::PLL1_Q; // FDCANクロックをPLL1_Qに設定
        config.rcc.mux = clock_mux;
    }
    config
}

/// TIM4 Hallセンサーインターフェース初期化
///
/// PB6=H1、PB7=H2、PB8=H3（XORモード）
///
/// # Safety
/// PACを使用した直接レジスタ操作を含む
pub unsafe fn init_hall_sensor() {
    info!("Initializing TIM4 Hall Sensor Interface (XOR mode)...");
    hall_tim::init_hall_timer();
    info!("TIM4 Hall Sensor Interface initialized");
}
