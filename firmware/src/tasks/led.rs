//! LED制御タスク
//!
//! 3つのLEDを順次点灯させて動作確認を行います。

use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

use crate::fmt::*;

/// LED制御タスク
///
/// 3つのLEDを500msごとに順次点灯させます。
#[embassy_executor::task]
pub async fn led_task(
    mut led1: Output<'static>,
    mut led2: Output<'static>,
    mut led3: Output<'static>,
) {
    info!("LED task started");

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
