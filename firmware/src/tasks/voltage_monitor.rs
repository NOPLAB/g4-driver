//! 電圧監視タスク
//!
//! DCバス電圧を監視し、過電圧/低電圧を検出してモーターを保護します。

use embassy_stm32::{adc::Adc, peripherals};
use embassy_time::{Duration, Ticker};

use crate::fmt::*;
use crate::state::{MOTOR_ENABLE, TARGET_SPEED, VOLTAGE_STATE};
use crate::voltage_monitor::{VoltageMonitor, VoltageMonitorConfig};

/// 電圧監視タスク - DCバス電圧を監視し、過電圧/低電圧を検出
#[embassy_executor::task]
pub async fn voltage_monitor_task(
    mut adc: Adc<'static, peripherals::ADC2>,
    mut voltage_pin: embassy_stm32::adc::AnyAdcChannel<peripherals::ADC2>,
) {
    info!("Voltage monitor task started");

    // 電圧監視コントローラ初期化
    let mut monitor = VoltageMonitor::new(VoltageMonitorConfig {
        r_upper: 33_300.0, // 33.3kΩ（分圧回路の上側抵抗）
        r_lower: 3_300.0,  // 3.3kΩ（分圧回路の下側抵抗）
        adc_max: 4096,
        vref: 3.3,
        filter_alpha: 0.1,
        overvoltage_threshold: 30.0,  // 30V以上で過電圧警告
        undervoltage_threshold: 10.0, // 10V以下で低電圧警告
    });

    info!("Voltage monitor initialized: OV=30V, UV=10V");

    // 初回ADC読み取りでフィルタを初期化（起動時のUNDERVOLTAGE誤検出を防ぐ）
    let initial_adc = adc.blocking_read(&mut voltage_pin);
    monitor.initialize_with_adc(initial_adc);  // フィルタを実電圧で初期化
    let state = monitor.get_state();
    info!(
        "Initial voltage: {}V (ADC raw: {}), OV={}, UV={}",
        state.voltage, initial_adc, state.overvoltage, state.undervoltage
    );

    // 監視周期（100ms）
    let mut ticker = Ticker::every(Duration::from_millis(100));

    // デバッグログ用カウンタ（1秒ごとにログ）
    let mut log_counter = 0u32;

    loop {
        ticker.next().await;

        // ADCから電圧を読み取り
        let adc_raw = adc.blocking_read(&mut voltage_pin);

        // デバッグ: ADC生値とADC電圧を計算（33.3kΩ + 3.3kΩ分圧）
        let v_adc = (adc_raw as f32 / 4096.0) * 3.3;
        let divider_ratio = (33_300.0 + 3_300.0) / 3_300.0;  // = 11.09倍
        let v_bus_calc = v_adc * divider_ratio;

        // 電圧監視更新
        let state = monitor.update(adc_raw);

        // グローバル状態を更新（CAN送信用）
        *VOLTAGE_STATE.lock().await = state;

        // 過電圧/低電圧時はモーターを自動停止
        if !state.is_voltage_ok() {
            let was_enabled = *MOTOR_ENABLE.lock().await;
            if was_enabled {
                error!(
                    "Voltage fault detected! Disabling motor. Voltage: {}V, OV: {}, UV: {}",
                    state.voltage, state.overvoltage, state.undervoltage
                );
                *MOTOR_ENABLE.lock().await = false;
                *TARGET_SPEED.lock().await = 0.0;
            }
        }

        // デバッグログ（1秒ごと = 10回に1回）
        log_counter += 1;
        if log_counter >= 10 {
            log_counter = 0;
            info!(
                "[Voltage] ADC_raw={}, V_adc={}V, V_bus_calc={}V, V_filtered={}V, ratio={}, OV={}, UV={}",
                adc_raw, v_adc, v_bus_calc, state.voltage, divider_ratio, state.overvoltage, state.undervoltage
            );
        }
    }
}
