#![no_std]
#![no_main]

mod adapters;
mod config;
mod fmt;
mod foc;
mod hall_tim;
mod hardware;
mod init;
mod motor_driver;
mod state;
mod tasks;
mod voltage_monitor;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, SampleTime},
    can,
    gpio::{Level, Output, Speed},
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        low_level::CountingMode,
        simple_pwm::PwmPin,
        Channel,
    },
};
use embassy_time::{Duration, Timer};

use fmt::*;
use hardware::Irqs;
use init::ConfigLoader;
use tasks::{can_task, led_task, motor_control_task, voltage_monitor_task};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ハードウェア初期化
    let hw_config = hardware::create_clock_config();
    let p = embassy_stm32::init(hw_config);

    info!("═══════════════════════════════════════════════════════════════════");
    info!("");
    info!("    ██████╗ ██╗  ██╗    ██████╗ ██████╗ ██╗██╗   ██╗███████╗██████╗ ");
    info!("   ██╔════╝ ██║  ██║    ██╔══██╗██╔══██╗██║██║   ██║██╔════╝██╔══██╗");
    info!("   ██║  ███╗███████║    ██║  ██║██████╔╝██║██║   ██║█████╗  ██████╔╝");
    info!("   ██║   ██║╚════██║    ██║  ██║██╔══██╗██║╚██╗ ██╔╝██╔══╝  ██╔══██╗");
    info!("   ╚██████╔╝     ██║    ██████╔╝██║  ██║██║ ╚████╔╝ ███████╗██║  ██║");
    info!("    ╚═════╝      ╚═╝    ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚══════╝╚═╝  ╚═╝");
    info!("");
    info!("        BLDC Motor Controller • STM32G431VB @ 170MHz");
    info!("");
    info!("═══════════════════════════════════════════════════════════════════");

    // 設定ロード（Flash/CRCの所有権はConfigLoaderが管理）
    let mut config_loader = ConfigLoader::new(p.FLASH, p.CRC);
    let needs_calibration = config_loader.load_and_apply().await;

    // 自動キャリブレーション設定（設定が未保存の場合に実行）
    if needs_calibration {
        state::calibration_context().await.request = true;
        info!("Auto-calibration enabled (no saved config found)");
    }

    // Flash/CRCをcan_task用に取得（Peripherals::steal()を使わずに所有権を移譲）
    let (flash, crc) = config_loader.into_peripherals();

    // LED初期化＆タスク起動
    let led1 = Output::new(p.PC13, Level::High, Speed::Low);
    let led2 = Output::new(p.PC14, Level::High, Speed::Low);
    let led3 = Output::new(p.PC15, Level::High, Speed::Low);
    if spawner.spawn(led_task(led1, led2, led3)).is_err() {
        error!("Failed to spawn led_task");
    }

    // CAN初期化＆タスク起動
    let mut can_configurator = can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);
    can_configurator.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );
    can_configurator.properties().set_standard_filter(
        can::filter::StandardFilterSlot::_0,
        can::filter::StandardFilter::accept_all_into_fifo0(),
    );
    can_configurator.set_bitrate(config::can::DEFAULT_BITRATE);
    let can = can_configurator.start(can::OperatingMode::NormalOperationMode);
    if spawner.spawn(can_task(can, flash, crc)).is_err() {
        error!("Failed to spawn can_task");
    }

    // ADC初期化
    let mut adc2 = Adc::new(p.ADC2);
    adc2.set_sample_time(SampleTime::CYCLES640_5);

    // 電圧監視タスク起動（PC1 = ADC2_IN7）
    let voltage_pin = p.PC1.degrade_adc();
    if spawner
        .spawn(voltage_monitor_task(adc2, voltage_pin))
        .is_err()
    {
        error!("Failed to spawn voltage_monitor_task");
    } else {
        info!("Voltage monitoring started on PC1 (ADC2_IN7)");
    }

    // PWM初期化（TIM1、3相補完PWM）
    let mut uvw_pwm = ComplementaryPwm::new(
        p.TIM1,
        Some(PwmPin::new(
            p.PE9,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(ComplementaryPwmPin::new(
            p.PE8,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(PwmPin::new(
            p.PE11,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(ComplementaryPwmPin::new(
            p.PE10,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(PwmPin::new(
            p.PE13,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(ComplementaryPwmPin::new(
            p.PE12,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        None,
        None,
        config::pwm::DEFAULT_FREQUENCY,
        CountingMode::EdgeAlignedUp,
    );
    uvw_pwm.disable(Channel::Ch1);
    uvw_pwm.disable(Channel::Ch2);
    uvw_pwm.disable(Channel::Ch3);
    uvw_pwm.set_dead_time(config::pwm::DEFAULT_DEAD_TIME);
    uvw_pwm.enable(Channel::Ch1);
    uvw_pwm.enable(Channel::Ch2);
    uvw_pwm.enable(Channel::Ch3);

    // TIM4 Hallセンサーインターフェース初期化
    unsafe {
        hardware::init_hall_sensor();
    }

    info!("Starting FOC motor control...");

    // モーター制御タスクを起動
    if spawner.spawn(motor_control_task(uvw_pwm)).is_err() {
        error!("Failed to spawn motor_control_task - CRITICAL");
    }

    // メインループ（将来の拡張用）
    loop {
        Timer::after(Duration::from_millis(100)).await;
    }
}
