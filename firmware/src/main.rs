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
    crc::{Config as CrcConfig, Crc},
    flash::Flash,
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

    // フラッシュとCRC初期化（設定ロード用）
    // Blocking版で設定を読み込む
    let mut flash_blocking = Flash::new_blocking(p.FLASH);

    // CRC初期化（STM32デフォルト設定: CRC-32、poly=0x04C11DB7）
    let crc_peripheral = p.CRC;
    let crc_config = CrcConfig::new(
        embassy_stm32::crc::InputReverseConfig::None,
        false, // reverse_out
        embassy_stm32::crc::PolySize::Width32,
        0xFFFFFFFF, // crc_init_value
        0x04C11DB7, // crc_poly (CRC-32)
    )
    .unwrap();
    let mut crc_blocking = Crc::new(crc_peripheral, crc_config);

    // 設定をフラッシュから読み込み（失敗時はデフォルト初期化）
    info!("Loading configuration from flash...");
    let loaded_config =
        config::load_or_initialize_config(&mut flash_blocking, &mut crc_blocking).await;

    // グローバル状態に設定を適用
    {
        let mut runtime_config = state::RUNTIME_CONFIG.lock().await;
        *runtime_config = loaded_config;

        let mut version = state::CONFIG_VERSION.lock().await;
        *version = loaded_config.version;

        let mut crc_valid = state::CONFIG_CRC_VALID.lock().await;
        *crc_valid = true; // load_or_initialize_configが成功したらCRC有効

        info!("Config loaded: version={}", loaded_config.version);
        info!(
            "  PI gains: Kp={}, Ki={}",
            loaded_config.speed_kp, loaded_config.speed_ki
        );
        info!("  Max voltage: {}V", loaded_config.max_voltage);
        info!("  Pole pairs: {}", loaded_config.pole_pairs);
    }

    // PIゲインをSPEED_PI_GAINSに適用
    {
        let mut gains = state::SPEED_PI_GAINS.lock().await;
        *gains = (loaded_config.speed_kp, loaded_config.speed_ki);
    }

    // キャリブレーション結果をCALIBRATION_RESULTに適用
    {
        let mut calib_result = state::CALIBRATION_RESULT.lock().await;
        calib_result.electrical_offset = loaded_config.calibration_electrical_offset;
        calib_result.direction_inversed = loaded_config.calibration_direction_inversed;
        calib_result.success = loaded_config.calibration_success;

        if loaded_config.calibration_success {
            info!("  Calibration data loaded:");
            info!(
                "    Electrical offset: {} rad",
                loaded_config.calibration_electrical_offset
            );
            info!(
                "    Direction inversed: {}",
                loaded_config.calibration_direction_inversed
            );
        } else {
            info!("  No calibration data found (calibration not performed)");
        }
    }

    // CAN task用にFlash/CRCをAsync版で再初期化
    // Peripheralsを再取得（flash_blockingとcrc_blockingはdrop）
    drop(flash_blocking);
    drop(crc_blocking);

    let p2 = unsafe { embassy_stm32::Peripherals::steal() };
    let flash = Flash::new_blocking(p2.FLASH); // new_blockingしか使えない
    let crc_config2 = CrcConfig::new(
        embassy_stm32::crc::InputReverseConfig::None,
        false,
        embassy_stm32::crc::PolySize::Width32,
        0xFFFFFFFF,
        0x04C11DB7,
    )
    .unwrap();
    let crc = Crc::new(p2.CRC, crc_config2);

    // LED初期化＆タスク起動
    let led1 = Output::new(p.PC13, Level::High, Speed::Low);
    let led2 = Output::new(p.PC14, Level::High, Speed::Low);
    let led3 = Output::new(p.PC15, Level::High, Speed::Low);
    spawner.spawn(led_task(led1, led2, led3)).unwrap();

    // CAN初期化＆タスク起動（FlashとCRCも渡す）
    // 注: flash と crc の所有権がcan_taskに移る
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
    spawner.spawn(can_task(can, flash, crc)).unwrap();

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

    // ベンチマーク実行
    unsafe {
        benchmark::enable_cycle_counter();
    }
    benchmark::run_inverse_park_benchmark(1000);

    info!("Starting FOC motor control...");

    // モーター制御タスクを起動
    spawner.spawn(motor_control_task(uvw_pwm)).unwrap();

    // メインループ（将来の拡張用）
    loop {
        Timer::after(Duration::from_millis(100)).await;
    }
}
