//! STSPIN32G4 + STM32G431VBT ボード固有実装
//!
//! STSPIN32G4評価ボード向けのモータードライバー実装。
//! - TIM1: 3相補完PWM出力
//! - TIM4: Hallセンサーインターフェース（XORモード）
//! - I2C3: ゲートドライバーIC制御
//! - ADC2: 電圧監視
//! - FDCAN1: CAN通信

pub mod adc;
pub mod can;
pub mod clock;
pub mod gate;
pub mod gpio;
pub mod hall;
pub mod pwm;
pub mod registers;

use embassy_stm32::{
    adc::{Adc, AnyAdcChannel},
    can::Can,
    gpio::Output,
    peripherals, Peri,
};

use crate::board::traits::GateDriverControl;

pub use gate::{bootstrap_charge, GateDriver, GateDriverError};
pub use hall::{
    calculate_speed_rpm, get_hall_state, get_period_cycles, get_snapshot, init_hall_timer,
    reset_state,
};
pub use pwm::MotorDriver;

use crate::fmt::*;

/// LEDピン構造体
pub struct LedPins<'d> {
    pub led1: Output<'d>,
    pub led2: Output<'d>,
    pub led3: Output<'d>,
}

/// ボード全体のペリフェラル
pub struct Peripherals<'d> {
    /// モータードライバー（TIM1 PWM）
    pub motor_driver: MotorDriver,
    /// CAN通信
    pub can: Can<'d>,
    /// ADC
    pub adc: Adc<'d, peripherals::ADC2>,
    /// 電圧監視ピン
    pub voltage_pin: AnyAdcChannel<peripherals::ADC2>,
    /// LEDピン
    pub leds: LedPins<'d>,
    /// Flash（設定保存用）
    pub flash: Peri<'d, peripherals::FLASH>,
    /// CRC（設定検証用）
    pub crc: Peri<'d, peripherals::CRC>,
}

/// ボード初期化エラー
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InitError {
    /// ゲートドライバーでフォルト検出
    GateDriverFault,
    /// ゲートドライバーのREADY待機タイムアウト
    GateDriverTimeout,
    /// ゲートドライバーI2C通信エラー
    GateDriverI2c,
}

impl From<GateDriverError> for InitError {
    fn from(e: GateDriverError) -> Self {
        match e {
            GateDriverError::FaultDetected => InitError::GateDriverFault,
            GateDriverError::ReadyTimeout => InitError::GateDriverTimeout,
            GateDriverError::I2cError | GateDriverError::ClearFaultFailed => {
                InitError::GateDriverI2c
            }
        }
    }
}

/// STSPIN32G4ボード初期化
pub struct StspinG431Board;

impl StspinG431Board {
    /// ボード全体を初期化
    ///
    /// # 初期化順序
    /// 1. GPIO (ドライバーイネーブル)
    /// 2. I2C + ゲートドライバー
    /// 3. TIM1 PWM + ブートストラップ充電
    /// 4. TIM4 Hall
    /// 5. ADC
    /// 6. CAN
    /// 7. GPIO (LED)
    /// 8. Flash/CRC
    pub async fn init(p: embassy_stm32::Peripherals) -> Result<Peripherals<'static>, InitError> {
        info!("Initializing STSPIN32G4 board...");

        // 1. ドライバーイネーブルピン（PE7: Active High）
        let _driver_enable = gpio::init_driver_enable(p.PE7);
        info!("Driver enable pin (PE7) set HIGH");

        // 2. ゲートドライバーIC初期化（I2C3経由）
        let mut gate_driver = GateDriver::new(p.I2C3, p.PC8, p.PC9, p.PE14, p.PE15);
        gate_driver.initialize().await?;

        // 3. PWM初期化（TIM1、3相補完PWM）
        let mut motor_driver =
            pwm::init_motor_driver(p.TIM1, p.PE9, p.PE8, p.PE11, p.PE10, p.PE13, p.PE12);

        // 4. ブートストラップ充電
        bootstrap_charge(&mut motor_driver).await;

        // 5. TIM4 Hallセンサーインターフェース初期化
        // Safety: ハードウェア初期化は一度だけ呼び出される
        unsafe { init_hall_timer() };

        // 6. ADC初期化
        let (adc, voltage_pin) = adc::init_adc(p.ADC2, p.PC1);
        info!("Voltage monitoring ready on PC1 (ADC2_IN7)");

        // 7. CAN初期化
        let can = can::init_can(p.FDCAN1, p.PA11, p.PA12);

        // 8. LED初期化
        let leds = gpio::init_leds(p.PC13, p.PC14, p.PC15);

        info!("Board initialization complete");

        // ゲートドライバーの所有権は不要（初期化のみで役割完了）
        let _ = gate_driver;

        Ok(Peripherals {
            motor_driver,
            can,
            adc,
            voltage_pin,
            leds,
            flash: p.FLASH,
            crc: p.CRC,
        })
    }
}
