//! LED制御タスク
//!
//! 3つのLEDを順次点灯させて動作確認を行います。

use embassy_time::{Duration, Timer};

use crate::board::LedPins;
use crate::fmt::*;

/// LED制御タスク
///
/// 3つのLEDを500msごとに順次点灯させます。
#[embassy_executor::task]
pub async fn led_task(mut leds: LedPins<'static>) {
    info!("LED task started");

    loop {
        leds.led1.set_high();
        leds.led2.set_low();
        leds.led3.set_low();
        Timer::after(Duration::from_millis(500)).await;

        leds.led1.set_low();
        leds.led2.set_high();
        leds.led3.set_low();
        Timer::after(Duration::from_millis(500)).await;

        leds.led1.set_low();
        leds.led2.set_low();
        leds.led3.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}
