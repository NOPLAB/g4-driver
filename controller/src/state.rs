use std::sync::Arc;
use tokio::sync::Mutex;

use crate::can::{CanManager, MotorStatus, VoltageStatus};

/// Connection state
#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Error(String),
}

impl Default for ConnectionState {
    fn default() -> Self {
        Self::Disconnected
    }
}

/// User settings
#[derive(Debug, Clone)]
pub struct UserSettings {
    /// Target speed in RPM
    pub target_speed: f32,
    /// Proportional gain
    pub kp: f32,
    /// Integral gain
    pub ki: f32,
    /// Motor enable flag
    pub motor_enabled: bool,
}

impl Default for UserSettings {
    fn default() -> Self {
        Self {
            target_speed: 0.0,
            kp: 0.5,      // Default from firmware config
            ki: 0.05,     // Default from firmware config
            motor_enabled: false,
        }
    }
}

/// Application state
#[derive(Clone)]
pub struct AppState {
    /// CAN manager
    pub can_manager: Arc<Mutex<CanManager>>,
    /// Connection state
    pub connection_state: ConnectionState,
    /// Selected CAN interface
    pub interface: String,
    /// Motor status
    pub motor_status: MotorStatus,
    /// Voltage status
    pub voltage_status: VoltageStatus,
    /// User settings
    pub settings: UserSettings,
    /// Last status update timestamp (milliseconds)
    pub last_status_update: u64,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            can_manager: Arc::new(Mutex::new(CanManager::new())),
            connection_state: ConnectionState::Disconnected,
            interface: "can0".to_string(),
            motor_status: MotorStatus::default(),
            voltage_status: VoltageStatus::default(),
            settings: UserSettings::default(),
            last_status_update: 0,
        }
    }
}

impl AppState {
    pub fn new() -> Self {
        Self::default()
    }
}
