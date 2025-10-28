#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use fmt::*;

use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, SampleTime},
    gpio::{Level, Output, Speed},
    opamp::{OpAmp, OpAmpSpeed},
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

#[embassy_executor::task]
pub async fn led_task(
    mut led1: Output<'static>,
    mut led2: Output<'static>,
    mut led3: Output<'static>,
) {
    loop {
        led1.set_high();
        led2.set_low();
        led3.set_low();
        Timer::after(Duration::from_millis(500)).await;

        led1.set_low();
        led2.set_high();
        led3.set_low();
        Timer::after(Duration::from_millis(500)).await;

        led1.set_low();
        led2.set_low();
        led3.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
pub async fn motor_task() {}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::Pll;
        use embassy_stm32::rcc::PllMul;
        use embassy_stm32::rcc::PllPreDiv;
        use embassy_stm32::rcc::PllRDiv;
        use embassy_stm32::rcc::PllSource;

        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });

        use embassy_stm32::rcc::mux::Adcsel;
        use embassy_stm32::rcc::mux::ClockMux;

        let mut clock_mux = ClockMux::default();
        clock_mux.adc12sel = Adcsel::SYS;
        config.rcc.mux = clock_mux;
    }

    let p = embassy_stm32::init(config);

    let led1 = Output::new(p.PC13, Level::High, Speed::Low);
    let led2 = Output::new(p.PC14, Level::High, Speed::Low);
    let led3 = Output::new(p.PC15, Level::High, Speed::Low);

    spawner.spawn(led_task(led1, led2, led3)).unwrap();

    let mut adc1 = Adc::new(p.ADC1);
    adc1.set_sample_time(SampleTime::CYCLES640_5);
    let mut adc2 = Adc::new(p.ADC2);
    adc2.set_sample_time(SampleTime::CYCLES640_5);

    let mut op1 = OpAmp::new(p.OPAMP1, OpAmpSpeed::HighSpeed);
    let op1_sa = op1.pga_ext(p.PA1, p.PA2, embassy_stm32::opamp::OpAmpGain::Mul4);
    let mut op1_adc_ch = op1_sa.degrade_adc();

    let mut op2 = OpAmp::new(p.OPAMP2, OpAmpSpeed::Normal);
    let op2_sa = op2.standalone_ext(p.PA7, p.PC5, p.PA6);
    let mut op2_adc_ch = op2_sa.degrade_adc();

    let mut op3 = OpAmp::new(p.OPAMP3, OpAmpSpeed::Normal);
    let op3_sa = op3.standalone_ext(p.PB0, p.PB2, p.PB1);
    let mut op3_adc_ch = op3_sa.degrade_adc();

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
        if angle >= 6.283185 {
            // 2π
            angle -= 6.283185;
        }

        let op1_val = adc1.blocking_read(&mut op1_adc_ch);
        let op2_val = adc2.blocking_read(&mut op2_adc_ch);
        let op3_val = adc1.blocking_read(&mut op3_adc_ch);

        info!("{}, {}, {}", op1_val, op2_val, op3_val);

        Timer::after(Duration::from_millis(1)).await;
    }
}
