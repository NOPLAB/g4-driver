use std::sync::Arc;
use tokio::sync::Mutex;

use crate::can::{
    CalibrationStatus, CanInterface, CanManager, MotorStatus, UsbCanDevice, VoltageStatus,
};

/// Connection state
#[derive(Debug, Clone, PartialEq, Default)]
pub enum ConnectionState {
    #[default]
    Disconnected,
    Connecting,
    Connected,
    Error(String),
}

// ============================================================================
// Settings Sub-structures
// ============================================================================

/// PI controller settings
#[derive(Debug, Clone)]
pub struct PiSettings {
    /// Proportional gain
    pub kp: f32,
    /// Integral gain
    pub ki: f32,
}

impl Default for PiSettings {
    fn default() -> Self {
        Self { kp: 0.5, ki: 0.05 }
    }
}

/// Motor control settings
#[derive(Debug, Clone)]
pub struct MotorSettings {
    /// Maximum voltage [V]
    pub max_voltage: f32,
    /// DC bus voltage [V]
    pub v_dc_bus: f32,
    /// Motor pole pairs
    pub pole_pairs: u8,
    /// Maximum duty cycle
    pub max_duty: u16,
}

impl Default for MotorSettings {
    fn default() -> Self {
        Self {
            max_voltage: 24.0,
            v_dc_bus: 24.0,
            pole_pairs: 6,
            max_duty: 100,
        }
    }
}

/// Hall sensor settings
#[derive(Debug, Clone)]
pub struct HallSensorSettings {
    /// Speed filter alpha coefficient
    pub speed_filter_alpha: f32,
    /// Hall sensor angle offset [rad]
    pub angle_offset: f32,
    /// Enable angle interpolation
    pub enable_interpolation: bool,
}

impl Default for HallSensorSettings {
    fn default() -> Self {
        Self {
            speed_filter_alpha: 0.1,
            angle_offset: 0.0,
            enable_interpolation: true,
        }
    }
}

/// OpenLoop startup settings
#[derive(Debug, Clone)]
pub struct OpenLoopSettings {
    /// Initial RPM for ramp-up
    pub initial_rpm: f32,
    /// Target RPM for FOC transition
    pub target_rpm: f32,
    /// Acceleration [RPM/s]
    pub acceleration: f32,
    /// Duty ratio (0-100)
    pub duty_ratio: u16,
    /// Forced commutation cycles
    pub forced_commutation_cycles: u32,
    /// Min cycles before FOC transition
    pub min_cycles_before_foc: u32,
}

impl Default for OpenLoopSettings {
    fn default() -> Self {
        Self {
            initial_rpm: 100.0,
            target_rpm: 500.0,
            acceleration: 100.0,
            duty_ratio: 50,
            forced_commutation_cycles: 10000,
            min_cycles_before_foc: 10000,
        }
    }
}

/// Advance angle settings
#[derive(Debug, Clone)]
pub struct AdvanceAngleSettings {
    /// Base advance angle [degrees]
    pub base_deg: f32,
    /// Maximum advance angle [degrees]
    pub max_deg: f32,
    /// Minimum speed for advance [RPM]
    pub min_speed: f32,
    /// Maximum speed for advance [RPM]
    pub max_speed: f32,
}

impl Default for AdvanceAngleSettings {
    fn default() -> Self {
        Self {
            base_deg: 10.0,
            max_deg: 30.0,
            min_speed: 100.0,
            max_speed: 3000.0,
        }
    }
}

/// Minimum voltage settings
#[derive(Debug, Clone)]
pub struct MinVoltageSettings {
    /// Minimum output voltage [V]
    pub voltage: f32,
    /// Error threshold [RPM]
    pub error_threshold: f32,
    /// Max speed acceleration [RPM/s]
    pub max_speed_acceleration: f32,
}

impl Default for MinVoltageSettings {
    fn default() -> Self {
        Self {
            voltage: 2.0,
            error_threshold: 2.0,
            max_speed_acceleration: 100.0,
        }
    }
}

/// FOC stall detection settings
#[derive(Debug, Clone)]
pub struct FocStallSettings {
    /// Speed threshold [RPM]
    pub speed_threshold: f32,
    /// Count threshold
    pub count_threshold: u32,
}

impl Default for FocStallSettings {
    fn default() -> Self {
        Self {
            speed_threshold: 50.0,
            count_threshold: 1000,
        }
    }
}

/// Dead time compensation settings
#[derive(Debug, Clone)]
pub struct DeadTimeCompSettings {
    /// Enabled flag
    pub enabled: bool,
    /// Dead time [ns]
    pub dead_time_ns: f32,
}

impl Default for DeadTimeCompSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            dead_time_ns: 100.0,
        }
    }
}

/// Flux weakening settings
#[derive(Debug, Clone)]
pub struct FluxWeakeningSettings {
    /// Enabled flag
    pub enabled: bool,
    /// Min speed [RPM]
    pub min_speed: f32,
    /// Max speed [RPM]
    pub max_speed: f32,
    /// Max ratio (0-1)
    pub max_ratio: f32,
    /// Vd rate limit [V/s]
    pub vd_rate_limit: f32,
}

impl Default for FluxWeakeningSettings {
    fn default() -> Self {
        Self {
            enabled: false,
            min_speed: 2000.0,
            max_speed: 4000.0,
            max_ratio: 0.5,
            vd_rate_limit: 100.0,
        }
    }
}

/// Voltage monitor settings
#[derive(Debug, Clone)]
pub struct VoltageMonitorSettings {
    /// Overvoltage threshold [V]
    pub overvoltage_threshold: f32,
    /// Undervoltage threshold [V]
    pub undervoltage_threshold: f32,
    /// Filter alpha
    pub filter_alpha: f32,
}

impl Default for VoltageMonitorSettings {
    fn default() -> Self {
        Self {
            overvoltage_threshold: 30.0,
            undervoltage_threshold: 10.0,
            filter_alpha: 0.1,
        }
    }
}

/// Hardware configuration settings
#[derive(Debug, Clone)]
pub struct HardwareSettings {
    /// PWM frequency [Hz]
    pub pwm_frequency: u32,
    /// PWM dead time
    pub pwm_dead_time: u16,
    /// CAN bitrate [bps]
    pub can_bitrate: u32,
    /// Control period [us]
    pub control_period_us: u64,
}

impl Default for HardwareSettings {
    fn default() -> Self {
        Self {
            pwm_frequency: 50000,
            pwm_dead_time: 100,
            can_bitrate: 250000,
            control_period_us: 400,
        }
    }
}

// ============================================================================
// Main UserSettings structure
// ============================================================================

/// User settings (matches firmware StoredConfig)
#[derive(Debug, Clone)]
pub struct UserSettings {
    /// Target speed in RPM
    pub target_speed: f32,
    /// Motor enable flag
    pub motor_enabled: bool,

    /// PI controller settings
    pub pi: PiSettings,
    /// Motor control settings
    pub motor: MotorSettings,
    /// Hall sensor settings
    pub hall_sensor: HallSensorSettings,
    /// OpenLoop settings
    pub openloop: OpenLoopSettings,
    /// Advance angle settings
    pub advance_angle: AdvanceAngleSettings,
    /// Minimum voltage settings
    pub min_voltage: MinVoltageSettings,
    /// FOC stall detection settings
    pub foc_stall: FocStallSettings,
    /// Dead time compensation settings
    pub dead_time_comp: DeadTimeCompSettings,
    /// Flux weakening settings
    pub flux_weakening: FluxWeakeningSettings,
    /// Voltage monitor settings
    pub voltage_monitor: VoltageMonitorSettings,
    /// Hardware settings
    pub hardware: HardwareSettings,
}

impl Default for UserSettings {
    fn default() -> Self {
        Self {
            target_speed: 0.0,
            motor_enabled: false,
            pi: PiSettings::default(),
            motor: MotorSettings::default(),
            hall_sensor: HallSensorSettings::default(),
            openloop: OpenLoopSettings::default(),
            advance_angle: AdvanceAngleSettings::default(),
            min_voltage: MinVoltageSettings::default(),
            foc_stall: FocStallSettings::default(),
            dead_time_comp: DeadTimeCompSettings::default(),
            flux_weakening: FluxWeakeningSettings::default(),
            voltage_monitor: VoltageMonitorSettings::default(),
            hardware: HardwareSettings::default(),
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
    /// Calibration status (from driver)
    pub calibration_status: Option<CalibrationStatus>,
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
            calibration_status: None,
        }
    }
}

impl AppState {
    pub fn new() -> Self {
        Self::default()
    }
}
