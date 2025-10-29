use std::sync::Arc;
use tokio::sync::Mutex;

use crate::can::{CanInterface, CanManager, MotorStatus, UsbCanDevice, VoltageStatus};

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

/// User settings (matches firmware StoredConfig)
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

    // === Motor Control Parameters ===
    /// Maximum voltage [V]
    pub max_voltage: f32,
    /// DC bus voltage [V]
    pub v_dc_bus: f32,
    /// Motor pole pairs
    pub pole_pairs: u8,
    /// Maximum duty cycle
    pub max_duty: u16,
    /// Hall sensor speed filter alpha
    pub speed_filter_alpha: f32,
    /// Hall sensor angle offset [rad]
    pub hall_angle_offset: f32,
    /// Enable angle interpolation
    pub enable_angle_interpolation: bool,

    // === OpenLoop Parameters ===
    /// OpenLoop initial RPM
    pub openloop_initial_rpm: f32,
    /// OpenLoop target RPM
    pub openloop_target_rpm: f32,
    /// OpenLoop acceleration [RPM/s]
    pub openloop_acceleration: f32,
    /// OpenLoop duty ratio (0-100)
    pub openloop_duty_ratio: u16,

    // === PWM Configuration ===
    /// PWM frequency [Hz]
    pub pwm_frequency: u32,
    /// PWM dead time
    pub pwm_dead_time: u16,

    // === CAN Configuration ===
    /// CAN bitrate [bps]
    pub can_bitrate: u32,

    // === Control Timing ===
    /// Control period [μs]
    pub control_period_us: u64,
}

impl Default for UserSettings {
    fn default() -> Self {
        Self {
            target_speed: 0.0,
            kp: 0.5,
            ki: 0.05,
            motor_enabled: false,

            // Motor Control defaults (from firmware config.rs)
            max_voltage: 24.0,
            v_dc_bus: 24.0,
            pole_pairs: 6,
            max_duty: 100,
            speed_filter_alpha: 0.1,
            hall_angle_offset: 0.0,
            enable_angle_interpolation: true,

            // OpenLoop defaults
            openloop_initial_rpm: 100.0,
            openloop_target_rpm: 500.0,
            openloop_acceleration: 100.0,
            openloop_duty_ratio: 50,

            // PWM defaults
            pwm_frequency: 50000,
            pwm_dead_time: 100,

            // CAN defaults
            can_bitrate: 250000,

            // Control timing defaults
            control_period_us: 400,
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
    /// Available CAN interfaces (detected)
    pub available_interfaces: Vec<CanInterface>,
    /// Available USB-CAN devices (detected)
    pub available_usb_devices: Vec<UsbCanDevice>,
    /// Motor status
    pub motor_status: MotorStatus,
    /// Voltage status
    pub voltage_status: VoltageStatus,
    /// User settings
    pub settings: UserSettings,
    /// Last status update timestamp (milliseconds)
    pub last_status_update: u64,
    /// Config version number (from driver)
    pub config_version: u16,
    /// Config CRC valid flag (from driver)
    pub config_crc_valid: bool,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            can_manager: Arc::new(Mutex::new(CanManager::new())),
            connection_state: ConnectionState::Disconnected,
            interface: "can0".to_string(),
            available_interfaces: Vec::new(),
            available_usb_devices: Vec::new(),
            motor_status: MotorStatus::default(),
            voltage_status: VoltageStatus::default(),
            settings: UserSettings::default(),
            last_status_update: 0,
            config_version: 0,
            config_crc_valid: false,
        }
    }
}

impl AppState {
    pub fn new() -> Self {
        Self::default()
    }
}
