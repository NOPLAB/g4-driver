//! フラッシュメモリベースのEEPROM実装
//!
//! STM32G431VBの最終フラッシュページ（ページ63）を使用して設定を保存

use embassy_stm32::{crc::Crc, flash::Flash};

use crate::config_storage::StoredConfig;
use crate::fmt::*;

/// STM32G431VBのフラッシュページサイズ（2KB）
pub const FLASH_PAGE_SIZE: usize = 2048;

/// 最終ページ番号（ページ63、0ベース）
pub const LAST_PAGE_NUMBER: u8 = 63;

/// 最終ページの開始アドレス（128KB - 2KB = 0x0801F800）
pub const LAST_PAGE_START: u32 = 0x0801F800;

/// EEPROM操作のエラー型
#[derive(Debug, Clone, Copy)]
pub enum EepromError {
    /// フラッシュ書き込みエラー
    FlashWriteError,

    /// フラッシュ消去エラー
    FlashEraseError,

    /// フラッシュ読み取りエラー
    FlashReadError,

    /// CRC検証エラー
    CrcMismatch,

    /// マジックナンバー不一致
    InvalidMagic,

    /// バージョン不一致
    VersionMismatch,

    /// データサイズエラー
    InvalidSize,
}

/// フラッシュメモリから設定を読み込む
///
/// # Arguments
/// * `flash` - Flashペリフェラル
/// * `crc` - CRCペリフェラル
///
/// # Returns
/// * `Ok(StoredConfig)` - 読み込み成功
/// * `Err(EepromError)` - 読み込み失敗（CRCエラー、バージョン不一致など）
pub fn read_config(flash: &mut Flash, crc: &mut Crc) -> Result<StoredConfig, EepromError> {
    info!("Reading config from flash at 0x{:08X}", LAST_PAGE_START);

    // フラッシュからバイト列を読み込み
    let mut buffer = [0u8; core::mem::size_of::<StoredConfig>()];

    // embassy-stm32のFlash::readを使用
    let src_addr = LAST_PAGE_START as usize;
    for (i, byte) in buffer.iter_mut().enumerate() {
        let addr = (src_addr + i) as *const u8;
        *byte = unsafe { core::ptr::read_volatile(addr) };
    }

    // バイト列から構造体に変換
    let config = unsafe { StoredConfig::from_bytes(&buffer) }
        .ok_or(EepromError::InvalidSize)?;

    // マジックナンバーとバージョンを検証
    if !config.validate_header() {
        error!("Config header validation failed: magic=0x{:08X}, version={}",
               config.magic, config.version);
        return Err(EepromError::InvalidMagic);
    }

    // CRC検証
    if !config.verify_crc(crc) {
        error!("CRC verification failed: stored=0x{:08X}", config.crc32);
        return Err(EepromError::CrcMismatch);
    }

    info!("Config loaded successfully: version={}", config.version);
    Ok(config)
}

/// フラッシュメモリに設定を書き込む
///
/// # Arguments
/// * `flash` - Flashペリフェラル
/// * `crc` - CRCペリフェラル
/// * `config` - 保存する設定
///
/// # Returns
/// * `Ok(())` - 書き込み成功
/// * `Err(EepromError)` - 書き込み失敗
pub async fn write_config(
    flash: &mut Flash<'_>,
    crc: &mut Crc,
    config: &mut StoredConfig,
) -> Result<(), EepromError> {
    info!("Writing config to flash at 0x{:08X}", LAST_PAGE_START);

    // CRC計算
    config.crc32 = config.calculate_crc(crc);
    info!("Calculated CRC32: 0x{:08X}", config.crc32);

    // 最終ページを消去
    info!("Erasing flash page {}", LAST_PAGE_NUMBER);
    flash
        .blocking_erase(LAST_PAGE_START, LAST_PAGE_START + FLASH_PAGE_SIZE as u32)
        .map_err(|e| {
            error!("Flash erase failed: {:?}", e);
            EepromError::FlashEraseError
        })?;

    // バイト列に変換
    let data = config.as_bytes_mut();

    // フラッシュに書き込み
    info!("Writing {} bytes to flash", data.len());
    flash
        .blocking_write(LAST_PAGE_START, data)
        .map_err(|e| {
            error!("Flash write failed: {:?}", e);
            EepromError::FlashWriteError
        })?;

    info!("Config saved successfully");
    Ok(())
}

/// フラッシュメモリをデフォルト設定で初期化
///
/// # Arguments
/// * `flash` - Flashペリフェラル
/// * `crc` - CRCペリフェラル
///
/// # Returns
/// * `Ok(StoredConfig)` - 初期化されたデフォルト設定
/// * `Err(EepromError)` - 初期化失敗
pub async fn initialize_default_config(
    flash: &mut Flash<'_>,
    crc: &mut Crc,
) -> Result<StoredConfig, EepromError> {
    info!("Initializing flash with default config");

    let mut config = StoredConfig::default();
    write_config(flash, crc, &mut config).await?;

    Ok(config)
}

/// 設定を読み込み、失敗時はデフォルト設定で初期化
///
/// # Arguments
/// * `flash` - Flashペリフェラル
/// * `crc` - CRCペリフェラル
///
/// # Returns
/// * 有効な設定（読み込み成功 or デフォルト設定）
pub async fn load_or_initialize_config(
    flash: &mut Flash<'_>,
    crc: &mut Crc,
) -> StoredConfig {
    match read_config(flash, crc) {
        Ok(config) => {
            info!("Loaded config from flash");
            config
        }
        Err(e) => {
            error!("Failed to load config: {:?}, initializing with defaults", e);
            match initialize_default_config(flash, crc).await {
                Ok(config) => config,
                Err(e) => {
                    error!("Failed to initialize default config: {:?}, using in-memory defaults", e);
                    StoredConfig::default()
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flash_addresses() {
        // 128KB = 0x20000
        // 最終ページ = 0x08000000 + 0x20000 - 0x800 = 0x0801F800
        assert_eq!(LAST_PAGE_START, 0x0801F800);
    }

    #[test]
    fn test_page_size() {
        assert_eq!(FLASH_PAGE_SIZE, 2048);
    }
}
