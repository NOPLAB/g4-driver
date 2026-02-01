//! クロック設定モジュール
//!
//! HSI → PLL（÷4 × 85 ÷ 2）で170MHz生成

use embassy_stm32::Config;

/// RCCクロック設定を作成
///
/// HSI → PLL（÷4 × 85 ÷ 2）で170MHz生成
pub fn create_config() -> Config {
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
