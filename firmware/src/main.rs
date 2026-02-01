#![no_std]
#![no_main]

mod adapters;
mod board;
mod config;
mod fmt;
mod init;
mod state;
mod tasks;
mod voltage_monitor;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use board::StspinG431Board;
use fmt::*;
use init::ConfigLoader;
use tasks::{can_task, led_task, motor_control_task, voltage_monitor_task};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // クロック初期化
    let hw_config = board::stspin_g431::clock::create_config();
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

    // ボード初期化（全ペリフェラル一括）
    let peripherals = match StspinG431Board::init(p).await {
        Ok(p) => p,
        Err(e) => {
            error!("Board init failed: {:?}", e);
            loop {
                Timer::after(Duration::from_millis(1000)).await;
            }
        }
    };

    // 設定ロード（Flash/CRCの所有権はConfigLoaderが管理）
    let mut config_loader = ConfigLoader::new(peripherals.flash, peripherals.crc);
    let needs_calibration = config_loader.load_and_apply().await;

    // 自動キャリブレーション設定（設定が未保存の場合に実行）
    if needs_calibration {
        state::calibration_context().await.request = true;
        info!("Auto-calibration enabled (no saved config found)");
    }

    // Flash/CRCをcan_task用に取得（Peripherals::steal()を使わずに所有権を移譲）
    let (flash, crc) = config_loader.into_peripherals();

    // タスク起動
    if spawner.spawn(led_task(peripherals.leds)).is_err() {
        error!("Failed to spawn led_task");
    }

    if spawner
        .spawn(can_task(peripherals.can, flash, crc))
        .is_err()
    {
        error!("Failed to spawn can_task");
    }

    if spawner
        .spawn(voltage_monitor_task(
            peripherals.adc,
            peripherals.voltage_pin,
        ))
        .is_err()
    {
        error!("Failed to spawn voltage_monitor_task");
    } else {
        info!("Voltage monitoring started on PC1 (ADC2_IN7)");
    }

    // モーター制御タスクを起動
    info!("Starting FOC motor control...");
    if spawner
        .spawn(motor_control_task(peripherals.motor_driver))
        .is_err()
    {
        error!("Failed to spawn motor_control_task - CRITICAL");
    }

    // メインループ（将来の拡張用）
    loop {
        Timer::after(Duration::from_millis(100)).await;
    }
}
