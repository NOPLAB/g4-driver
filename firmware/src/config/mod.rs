//! Configuration module
//!
//! このモジュールはモーター制御とハードウェアの設定、
//! および設定の永続化機能を提供します。

pub mod eeprom;
pub mod params;
pub mod storage;

// params.rsから主要な定数を再エクスポート
pub use params::*;

// storage.rsから構造体を再エクスポート
pub use storage::StoredConfig;

// eepromモジュールの主要な関数を再エクスポート
pub use eeprom::{initialize_default_config, load_or_initialize_config, read_config, write_config};
