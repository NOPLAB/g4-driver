//! CAN初期化モジュール
//!
//! FDCAN1の初期化

use embassy_stm32::{bind_interrupts, can, peripherals, Peri};

use crate::config;

// CANの割り込みをバインド
bind_interrupts!(pub struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

/// FDCAN1を初期化
///
/// # Arguments
/// * `fdcan` - FDCAN1ペリフェラル
/// * `rx` - RXピン（PA11）
/// * `tx` - TXピン（PA12）
///
/// # Returns
/// 初期化されたCANインスタンス
pub fn init_can(
    fdcan: Peri<'static, peripherals::FDCAN1>,
    rx: Peri<'static, peripherals::PA11>,
    tx: Peri<'static, peripherals::PA12>,
) -> can::Can<'static> {
    let mut can_configurator = can::CanConfigurator::new(fdcan, rx, tx, Irqs);

    // フィルター設定
    can_configurator.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );
    can_configurator.properties().set_standard_filter(
        can::filter::StandardFilterSlot::_0,
        can::filter::StandardFilter::accept_all_into_fifo0(),
    );

    // ビットレート設定
    can_configurator.set_bitrate(config::can::DEFAULT_BITRATE);

    // CAN開始
    can_configurator.start(can::OperatingMode::NormalOperationMode)
}
