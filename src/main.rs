#![no_std]
#![no_main]

mod benchmark;
mod can_protocol;
mod config;
mod fmt;
mod foc;
mod hall_tim;
mod hardware;
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
    opamp::{OpAmp, OpAmpSpeed},
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
use tasks::{can_task, led_task, motor_control_task, voltage_monitor_task};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // ハードウェア初期化
    let config = hardware::create_clock_config();
    let p = embassy_stm32::init(config);

    // LED初期化＆タスク起動
    let led1 = Output::new(p.PC13, Level::High, Speed::Low);
    let led2 = Output::new(p.PC14, Level::High, Speed::Low);
    let led3 = Output::new(p.PC15, Level::High, Speed::Low);
    spawner.spawn(led_task(led1, led2, led3)).unwrap();

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
    can_configurator.set_bitrate(config::can::BITRATE);
    let can = can_configurator.start(can::OperatingMode::NormalOperationMode);
    spawner.spawn(can_task(can)).unwrap();

    // ADC初期化
    let mut adc1 = Adc::new(p.ADC1);
    adc1.set_sample_time(SampleTime::CYCLES640_5);
    let mut adc2 = Adc::new(p.ADC2);
    adc2.set_sample_time(SampleTime::CYCLES640_5);

    // 電圧監視タスク起動（PC1 = ADC2_IN7）
    let voltage_pin = p.PC1.degrade_adc();
    spawner
        .spawn(voltage_monitor_task(adc2, voltage_pin))
        .unwrap();
    info!("Voltage monitoring started on PC1 (ADC2_IN7)");

    // OPAMP初期化（将来の電流リミット監視用、現在は未使用）
    let mut _op1 = OpAmp::new(p.OPAMP1, OpAmpSpeed::HighSpeed);
    let _op1_sa = _op1.pga_ext(p.PA1, p.PA2, embassy_stm32::opamp::OpAmpGain::Mul4);
    let mut _op2 = OpAmp::new(p.OPAMP2, OpAmpSpeed::Normal);
    let _op2_sa = _op2.standalone_ext(p.PA7, p.PC5, p.PA6);
    let mut _op3 = OpAmp::new(p.OPAMP3, OpAmpSpeed::Normal);
    let _op3_sa = _op3.standalone_ext(p.PB0, p.PB2, p.PB1);

    // PWM初期化（TIM1、3相補完PWM）
    let mut uvw_pwm = ComplementaryPwm::new(
        p.TIM1,
        Some(PwmPin::new(p.PE9, embassy_stm32::gpio::OutputType::PushPull)),
        Some(ComplementaryPwmPin::new(p.PE8, embassy_stm32::gpio::OutputType::PushPull)),
        Some(PwmPin::new(p.PE11, embassy_stm32::gpio::OutputType::PushPull)),
        Some(ComplementaryPwmPin::new(p.PE10, embassy_stm32::gpio::OutputType::PushPull)),
        Some(PwmPin::new(p.PE13, embassy_stm32::gpio::OutputType::PushPull)),
        Some(ComplementaryPwmPin::new(p.PE12, embassy_stm32::gpio::OutputType::PushPull)),
        None,
        None,
        config::pwm::FREQUENCY,
        CountingMode::EdgeAlignedUp,
    );
    uvw_pwm.disable(Channel::Ch1);
    uvw_pwm.disable(Channel::Ch2);
    uvw_pwm.disable(Channel::Ch3);
    uvw_pwm.set_dead_time(config::pwm::DEAD_TIME);
    uvw_pwm.enable(Channel::Ch1);
    uvw_pwm.enable(Channel::Ch2);
    uvw_pwm.enable(Channel::Ch3);

    // TIM4 Hallセンサーインターフェース初期化
    unsafe {
        hardware::init_hall_sensor();
    }

    // ベンチマーク実行
    unsafe {
        benchmark::enable_cycle_counter();
    }
    benchmark::run_inverse_park_benchmark(1000);

    info!("Starting FOC motor control...");

    // モーター制御タスクを起動
    spawner.spawn(motor_control_task(uvw_pwm)).unwrap();

    info!("System initialized successfully");
    info!("Send CAN commands to control motor:");
    info!("  - 0x100: Speed command (f32 RPM)");
    info!("  - 0x101: PI gains (Kp, Ki as f32)");
    info!("  - 0x102: Enable motor (u8: 0=off, 1=on)");
    info!("  - 0x000: Emergency stop");

    // メインループ（将来の拡張用）
    loop {
        Timer::after(Duration::from_millis(100)).await;
    }
}
