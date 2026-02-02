//! ボード抽象化レイヤー
//!
//! ペリフェラルの初期化とハードウェア抽象化を集約するモジュール。
//!
//! ## サポートボード
//!
//! - `stspin_g431`: STSPIN32G4 + STM32G431VBT 評価ボード
//!
//! ## 使用方法
//!
//! ```rust,ignore
//! use crate::board::{StspinG431Board, GateDriverControl};
//!
//! let peripherals = StspinG431Board::init(p).await?;
//! // peripherals.motor_driver, peripherals.can, etc...
//! ```

pub mod stspin_g431;
pub mod traits;

// トレイトをre-export（他の実装でも使用可能）
#[allow(unused_imports)]
pub use traits::{BootstrapChargeable, GateDriverControl, HallSensorInterface, PwmDriver};

// STSPIN32G4ボード固有実装をデフォルトとしてre-export
pub use stspin_g431::{
    calculate_speed_rpm, get_hall_state, get_snapshot, reset_state, LedPins, MotorDriver,
    StspinG431Board,
};
