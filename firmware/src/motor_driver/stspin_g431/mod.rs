//! STSPIN32G4 + STM32G431VBT ボード固有実装
//!
//! STSPIN32G4評価ボード向けのモータードライバー実装。
//! - TIM1: 3相補完PWM出力
//! - TIM4: Hallセンサーインターフェース（XORモード）
//! - I2C3: ゲートドライバーIC制御

pub mod gate;
pub mod hall;
pub mod pwm;
pub mod registers;

pub use gate::{bootstrap_charge, GateDriver};
pub use hall::{
    calculate_speed_rpm, get_hall_state, get_period_cycles, get_snapshot, init_hall_timer,
    reset_state,
};
pub use pwm::MotorDriver;
