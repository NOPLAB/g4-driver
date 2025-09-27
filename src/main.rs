#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    time::Hertz,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        low_level::CountingMode,
        simple_pwm::PwmPin,
        Channel,
    },
};
use embassy_time::{Duration, Timer};
use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led1 = Output::new(p.PC13, Level::High, Speed::Low);
    let mut led2 = Output::new(p.PC14, Level::High, Speed::Low);
    let mut led3 = Output::new(p.PC15, Level::High, Speed::Low);

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
        Hertz(50_000),
        CountingMode::EdgeAlignedUp,
    );

    uvw_pwm.disable(Channel::Ch1);
    uvw_pwm.disable(Channel::Ch2);
    uvw_pwm.disable(Channel::Ch3);
    uvw_pwm.set_dead_time(1);

    loop {
        // Step 1: U-High, V-Low, W-Off
        uvw_pwm.set_duty(Channel::Ch1, 100);
        uvw_pwm.set_duty(Channel::Ch2, 0);
        uvw_pwm.set_duty(Channel::Ch3, 0);
        uvw_pwm.enable(Channel::Ch1);
        uvw_pwm.enable(Channel::Ch2);
        uvw_pwm.disable(Channel::Ch3);
        Timer::after(Duration::from_millis(100)).await;

        // Step 2: U-High, V-Off, W-Low
        uvw_pwm.set_duty(Channel::Ch1, 100);
        uvw_pwm.set_duty(Channel::Ch2, 0);
        uvw_pwm.set_duty(Channel::Ch3, 0);
        uvw_pwm.enable(Channel::Ch1);
        uvw_pwm.disable(Channel::Ch2);
        uvw_pwm.enable(Channel::Ch3);
        Timer::after(Duration::from_millis(100)).await;

        // Step 3: U-Off, V-High, W-Low
        uvw_pwm.set_duty(Channel::Ch1, 0);
        uvw_pwm.set_duty(Channel::Ch2, 100);
        uvw_pwm.set_duty(Channel::Ch3, 0);
        uvw_pwm.disable(Channel::Ch1);
        uvw_pwm.enable(Channel::Ch2);
        uvw_pwm.enable(Channel::Ch3);
        Timer::after(Duration::from_millis(100)).await;

        // Step 4: U-Low, V-High, W-Off
        uvw_pwm.set_duty(Channel::Ch1, 0);
        uvw_pwm.set_duty(Channel::Ch2, 100);
        uvw_pwm.set_duty(Channel::Ch3, 0);
        uvw_pwm.enable(Channel::Ch1);
        uvw_pwm.enable(Channel::Ch2);
        uvw_pwm.disable(Channel::Ch3);
        Timer::after(Duration::from_millis(100)).await;

        // Step 5: U-Low, V-Off, W-High
        uvw_pwm.set_duty(Channel::Ch1, 0);
        uvw_pwm.set_duty(Channel::Ch2, 0);
        uvw_pwm.set_duty(Channel::Ch3, 100);
        uvw_pwm.enable(Channel::Ch1);
        uvw_pwm.disable(Channel::Ch2);
        uvw_pwm.enable(Channel::Ch3);
        Timer::after(Duration::from_millis(100)).await;

        // Step 6: U-Off, V-Low, W-High
        uvw_pwm.set_duty(Channel::Ch1, 0);
        uvw_pwm.set_duty(Channel::Ch2, 0);
        uvw_pwm.set_duty(Channel::Ch3, 100);
        uvw_pwm.disable(Channel::Ch1);
        uvw_pwm.enable(Channel::Ch2);
        uvw_pwm.enable(Channel::Ch3);
        Timer::after(Duration::from_millis(100)).await;

    }
}
