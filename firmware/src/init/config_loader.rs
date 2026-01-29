//! 設定ローダー
//!
//! フラッシュから設定を読み込み、グローバル状態に適用します。

use embassy_stm32::{
    crc::{Config as CrcConfig, Crc},
    flash::{Blocking, Flash},
    Peri,
};

use crate::config;
use crate::fmt::*;
use crate::state;

/// 設定ローダー
///
/// Flash/CRCペリフェラルの所有権を管理し、設定のロード後にcan_taskへ渡せるようにします。
pub struct ConfigLoader {
    flash: Flash<'static, Blocking>,
    crc: Crc<'static>,
}

impl ConfigLoader {
    /// 新しいConfigLoaderを作成
    pub fn new(
        flash_peripheral: Peri<'static, embassy_stm32::peripherals::FLASH>,
        crc_peripheral: Peri<'static, embassy_stm32::peripherals::CRC>,
    ) -> Self {
        let flash = Flash::new_blocking(flash_peripheral);

        // CRC初期化（STM32デフォルト設定: CRC-32、poly=0x04C11DB7）
        let crc_config = CrcConfig::new(
            embassy_stm32::crc::InputReverseConfig::None,
            false, // reverse_out
            embassy_stm32::crc::PolySize::Width32,
            0xFFFFFFFF, // crc_init_value
            0x04C11DB7, // crc_poly (CRC-32)
        )
        .unwrap();
        let crc = Crc::new(crc_peripheral, crc_config);

        Self { flash, crc }
    }

    /// 設定をフラッシュから読み込み、グローバル状態に適用
    ///
    /// 戻り値: キャリブレーションが必要かどうか
    pub async fn load_and_apply(&mut self) -> bool {
        info!("Loading configuration from flash...");

        // デバッグ用: フラッシュを無視してデフォルト値を使用
        // TODO: デバッグ完了後に元に戻す
        // let loaded_config = config::load_or_initialize_config(&mut self.flash, &mut self.crc).await;
        let loaded_config = config::StoredConfig::default();
        info!("Using DEFAULT config (flash ignored for debugging)");

        // グローバル状態に設定を適用
        state::update_system_config(loaded_config, loaded_config.version, true).await;
        info!("Config loaded: version={}", loaded_config.version);
        info!(
            "  PI gains: Kp={}, Ki={}",
            loaded_config.speed_kp, loaded_config.speed_ki
        );
        info!("  Max voltage: {}V", loaded_config.max_voltage);
        info!("  Pole pairs: {}", loaded_config.pole_pairs);

        // PIゲインをMotorContextに適用
        state::motor_context().await.pi_gains = (loaded_config.speed_kp, loaded_config.speed_ki);

        // キャリブレーション結果をCalibrationContextに適用
        state::apply_calibration_from_config(&loaded_config).await;

        let needs_calibration = !loaded_config.calibration_success;

        if !needs_calibration {
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
            info!("  Auto-calibration will start after motor control task initialization");
        }

        needs_calibration
    }

    /// Flash/CRCの所有権を消費してcan_task用に返す
    pub fn into_peripherals(self) -> (Flash<'static, Blocking>, Crc<'static>) {
        (self.flash, self.crc)
    }
}
