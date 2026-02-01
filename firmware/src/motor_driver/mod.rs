//! モータードライバー抽象化レイヤー
//!
//! PWMハードウェアへの直接アクセスを隠蔽し、
//! モーター制御に必要な高レベルインターフェースを提供します。
//!
//! ## ボード固有実装
//!
//! 現在サポートされているボード:
//! - `stspin_g431`: STSPIN32G4 + STM32G431VBT 評価ボード
//!
//! ## 使用方法
//!
//! ```rust,ignore
//! use crate::motor_driver::{MotorDriver, traits::PwmDriver};
//!
//! let mut driver = MotorDriver::new(pwm);
//! driver.set_duty_uvw(100, 200, 300);
//! driver.enable_all_channels();
//! ```

pub mod stspin_g431;
pub mod traits;

// STSPIN32G4ボード固有実装をデフォルトとしてre-export
pub use stspin_g431::{
    bootstrap_charge, calculate_speed_rpm, get_hall_state, get_period_cycles, get_snapshot,
    init_hall_timer, reset_state, GateDriver, MotorDriver,
};

// トレイトもre-export
pub use traits::GateDriverControl;
