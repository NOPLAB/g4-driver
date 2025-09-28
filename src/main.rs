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
use libm::sin;

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

    uvw_pwm.enable(Channel::Ch1);
    uvw_pwm.enable(Channel::Ch2);
    uvw_pwm.enable(Channel::Ch3);

    let mut angle = 0.0f32;
    let max_duty = 99u16;
    let mut angular_velocity = 0.01f32; // 初期角速度（ラジアン/ループ）
    let max_angular_velocity = 1f32; // 最大角速度
    let acceleration_rate = 1.02f32; // 加速率（2%増加）
    let amplitude = (max_duty as f32) / 2.0;
    let offset = amplitude;

    loop {
        // 3相正弦波PWMデューティ計算（120度位相差）
        let duty_u = (amplitude * sin(angle as f64) as f32 + offset) as u16;
        let duty_v = (amplitude * sin((angle + 2.094395) as f64) as f32 + offset) as u16; // +120度
        let duty_w = (amplitude * sin((angle + 4.188790) as f64) as f32 + offset) as u16; // +240度

        // 各相のPWMデューティ設定
        uvw_pwm.set_duty(Channel::Ch1, duty_u);
        uvw_pwm.set_duty(Channel::Ch2, duty_v);
        uvw_pwm.set_duty(Channel::Ch3, duty_w);

        // 徐々に加速
        if angular_velocity < max_angular_velocity {
            angular_velocity *= acceleration_rate;
            if angular_velocity > max_angular_velocity {
                angular_velocity = max_angular_velocity;
            }
        }

        // 角度更新
        angle += angular_velocity;
        if angle >= 6.283185 { // 2π
            angle -= 6.283185;
        }

        Timer::after(Duration::from_millis(1)).await;
    }
}
