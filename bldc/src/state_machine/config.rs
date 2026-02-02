//! Configuration for the motor state machine

use crate::control::{FocConfig, OpenLoopConfig};

/// Configuration for the state machine
#[derive(Debug, Clone)]
pub struct StateMachineConfig {
    /// Open-loop controller configuration
    pub openloop: OpenLoopConfig,
    /// FOC controller configuration
    pub foc: FocConfig,
    /// Number of motor pole pairs
    pub pole_pairs: u8,
    /// Maximum PWM duty value
    pub max_duty: u16,
    /// Invalid Hall state threshold (cycles before PI reset)
    pub invalid_hall_threshold: u32,
}

impl Default for StateMachineConfig {
    fn default() -> Self {
        Self {
            openloop: OpenLoopConfig::default(),
            foc: FocConfig::default(),
            pole_pairs: 6,
            max_duty: 100,
            invalid_hall_threshold: 100,
        }
    }
}
