//! Motor command definitions for unified CAN API
//!
//! This module provides a unified enum-based API for sending motor commands,
//! replacing the 26+ individual send_* methods with a single `send(cmd)` method.

use g4_driver_protocol::{self as protocol, can_ids};

/// Motor command for unified API
///
/// All motor commands can be sent using `CanManager::send(cmd)`.
#[derive(Debug, Clone)]
pub enum MotorCommand {
    // ========================================================================
    // Basic Commands
    // ========================================================================
    /// Set target speed in RPM
    Speed(f32),

    /// Set PI gains (Kp, Ki)
    PiGains { kp: f32, ki: f32 },

    /// Enable or disable motor
    Enable(bool),

    /// Emergency stop
    EmergencyStop,

    // ========================================================================
    // Config Commands
    // ========================================================================
    /// Save config to flash
    SaveConfig,

    /// Reload config from flash
    ReloadConfig,

    /// Reset config to defaults
    ResetConfig,

    /// Start calibration with optional torque (0-100)
    StartCalibration { torque: Option<u8> },

    // ========================================================================
    // Motor Parameters
    // ========================================================================
    /// Motor voltage parameters (max_voltage, v_dc_bus)
    MotorVoltage { max_voltage: f32, v_dc_bus: f32 },

    /// Motor basic parameters (pole_pairs, max_duty)
    MotorBasic { pole_pairs: u8, max_duty: u16 },

    // ========================================================================
    // Hall Sensor Parameters
    // ========================================================================
    /// Hall sensor parameters (speed_filter_alpha, angle_offset)
    HallSensor {
        speed_filter_alpha: f32,
        angle_offset: f32,
    },

    /// Enable/disable angle interpolation
    AngleInterpolation(bool),

    // ========================================================================
    // OpenLoop Parameters
    // ========================================================================
    /// OpenLoop RPM parameters (initial_rpm, target_rpm)
    OpenLoopRpm { initial_rpm: f32, target_rpm: f32 },

    /// OpenLoop acceleration and duty (acceleration, duty_ratio)
    OpenLoopAccelDuty { acceleration: f32, duty_ratio: u16 },

    /// OpenLoop cycles (forced_cycles, min_cycles)
    OpenLoopCycles { forced_cycles: u32, min_cycles: u32 },

    // ========================================================================
    // Advance Angle Parameters
    // ========================================================================
    /// Advance angle parameters (base_deg, max_deg)
    AdvanceAngle { base_deg: f32, max_deg: f32 },

    /// Advance angle speed range (min_speed, max_speed)
    AdvanceAngleSpeed { min_speed: f32, max_speed: f32 },

    // ========================================================================
    // Min Voltage Parameters
    // ========================================================================
    /// Min voltage parameters (min_voltage, error_threshold)
    MinVoltage {
        min_voltage: f32,
        error_threshold: f32,
    },

    /// Max speed acceleration
    MaxSpeedAccel(f32),

    // ========================================================================
    // FOC Stall Detection
    // ========================================================================
    /// FOC stall parameters (speed_threshold, count_threshold)
    FocStall {
        speed_threshold: f32,
        count_threshold: u32,
    },

    // ========================================================================
    // Compensation Parameters
    // ========================================================================
    /// Dead time compensation (enabled, dead_time_ns)
    DeadTimeComp { enabled: bool, dead_time_ns: f32 },

    /// Flux weakening enable (enabled, min_speed)
    FluxWeakeningEnable { enabled: bool, min_speed: f32 },

    /// Flux weakening parameters (max_speed, max_ratio)
    FluxWeakeningParams { max_speed: f32, max_ratio: f32 },

    /// Flux weakening Vd rate limit
    FluxWeakeningVd(f32),

    // ========================================================================
    // Voltage Monitor
    // ========================================================================
    /// Voltage monitor thresholds (overvoltage, undervoltage)
    VoltageMonitorThresholds { overvoltage: f32, undervoltage: f32 },

    /// Voltage monitor filter alpha
    VoltageMonitorFilter(f32),

    // ========================================================================
    // Hardware Configuration
    // ========================================================================
    /// PWM config (frequency, dead_time)
    PwmConfig { frequency: u32, dead_time: u16 },

    /// CAN config (bitrate)
    CanConfig(u32),

    /// Control timing (control_period_us)
    ControlTiming(u64),
}

impl MotorCommand {
    /// Get the CAN ID for this command
    pub fn can_id(&self) -> u32 {
        match self {
            Self::Speed(_) => can_ids::SPEED_CMD,
            Self::PiGains { .. } => can_ids::PI_GAINS,
            Self::Enable(_) => can_ids::ENABLE_CMD,
            Self::EmergencyStop => can_ids::EMERGENCY_STOP,
            Self::SaveConfig => can_ids::SAVE_CONFIG,
            Self::ReloadConfig => can_ids::RELOAD_CONFIG,
            Self::ResetConfig => can_ids::RESET_CONFIG,
            Self::StartCalibration { .. } => can_ids::START_CALIBRATION,
            Self::MotorVoltage { .. } => can_ids::MOTOR_VOLTAGE_PARAMS,
            Self::MotorBasic { .. } => can_ids::MOTOR_BASIC_PARAMS,
            Self::HallSensor { .. } => can_ids::HALL_SENSOR_PARAMS,
            Self::AngleInterpolation(_) => can_ids::ANGLE_INTERPOLATION,
            Self::OpenLoopRpm { .. } => can_ids::OPENLOOP_RPM_PARAMS,
            Self::OpenLoopAccelDuty { .. } => can_ids::OPENLOOP_ACCEL_DUTY_PARAMS,
            Self::OpenLoopCycles { .. } => can_ids::OPENLOOP_CYCLES_PARAMS,
            Self::AdvanceAngle { .. } => can_ids::ADVANCE_ANGLE_PARAMS,
            Self::AdvanceAngleSpeed { .. } => can_ids::ADVANCE_ANGLE_SPEED,
            Self::MinVoltage { .. } => can_ids::MIN_VOLTAGE_PARAMS,
            Self::MaxSpeedAccel(_) => can_ids::MAX_SPEED_ACCEL,
            Self::FocStall { .. } => can_ids::FOC_STALL_PARAMS,
            Self::DeadTimeComp { .. } => can_ids::DEAD_TIME_COMP_PARAMS,
            Self::FluxWeakeningEnable { .. } => can_ids::FLUX_WEAKENING_ENABLE,
            Self::FluxWeakeningParams { .. } => can_ids::FLUX_WEAKENING_PARAMS,
            Self::FluxWeakeningVd(_) => can_ids::FLUX_WEAKENING_VD,
            Self::VoltageMonitorThresholds { .. } => can_ids::VOLTAGE_MONITOR_THRESHOLDS,
            Self::VoltageMonitorFilter(_) => can_ids::VOLTAGE_MONITOR_FILTER,
            Self::PwmConfig { .. } => can_ids::PWM_CONFIG,
            Self::CanConfig(_) => can_ids::CAN_CONFIG,
            Self::ControlTiming(_) => can_ids::CONTROL_TIMING,
        }
    }

    /// Encode this command to CAN data bytes
    pub fn encode(&self) -> Vec<u8> {
        match self {
            Self::Speed(rpm) => protocol::encode_speed_command(*rpm).to_vec(),
            Self::PiGains { kp, ki } => protocol::encode_pi_gains(*kp, *ki).to_vec(),
            Self::Enable(enabled) => protocol::encode_enable_command(*enabled).to_vec(),
            Self::EmergencyStop => Vec::new(),
            Self::SaveConfig => Vec::new(),
            Self::ReloadConfig => Vec::new(),
            Self::ResetConfig => Vec::new(),
            Self::StartCalibration { torque } => {
                protocol::encode_start_calibration(*torque).to_vec()
            }
            Self::MotorVoltage {
                max_voltage,
                v_dc_bus,
            } => protocol::encode_motor_voltage_params(*max_voltage, *v_dc_bus).to_vec(),
            Self::MotorBasic {
                pole_pairs,
                max_duty,
            } => protocol::encode_motor_basic_params(*pole_pairs, *max_duty).to_vec(),
            Self::HallSensor {
                speed_filter_alpha,
                angle_offset,
            } => protocol::encode_hall_sensor_params(*speed_filter_alpha, *angle_offset).to_vec(),
            Self::AngleInterpolation(enabled) => {
                protocol::encode_angle_interpolation(*enabled).to_vec()
            }
            Self::OpenLoopRpm {
                initial_rpm,
                target_rpm,
            } => protocol::encode_openloop_rpm_params(*initial_rpm, *target_rpm).to_vec(),
            Self::OpenLoopAccelDuty {
                acceleration,
                duty_ratio,
            } => protocol::encode_openloop_accel_duty_params(*acceleration, *duty_ratio).to_vec(),
            Self::OpenLoopCycles {
                forced_cycles,
                min_cycles,
            } => protocol::encode_openloop_cycles_params(*forced_cycles, *min_cycles).to_vec(),
            Self::AdvanceAngle { base_deg, max_deg } => {
                protocol::encode_advance_angle_params(*base_deg, *max_deg).to_vec()
            }
            Self::AdvanceAngleSpeed {
                min_speed,
                max_speed,
            } => protocol::encode_advance_angle_speed(*min_speed, *max_speed).to_vec(),
            Self::MinVoltage {
                min_voltage,
                error_threshold,
            } => protocol::encode_min_voltage_params(*min_voltage, *error_threshold).to_vec(),
            Self::MaxSpeedAccel(accel) => protocol::encode_max_speed_accel(*accel).to_vec(),
            Self::FocStall {
                speed_threshold,
                count_threshold,
            } => protocol::encode_foc_stall_params(*speed_threshold, *count_threshold).to_vec(),
            Self::DeadTimeComp {
                enabled,
                dead_time_ns,
            } => protocol::encode_dead_time_comp_params(*enabled, *dead_time_ns).to_vec(),
            Self::FluxWeakeningEnable { enabled, min_speed } => {
                protocol::encode_flux_weakening_enable(*enabled, *min_speed).to_vec()
            }
            Self::FluxWeakeningParams {
                max_speed,
                max_ratio,
            } => protocol::encode_flux_weakening_params(*max_speed, *max_ratio).to_vec(),
            Self::FluxWeakeningVd(rate_limit) => {
                protocol::encode_flux_weakening_vd(*rate_limit).to_vec()
            }
            Self::VoltageMonitorThresholds {
                overvoltage,
                undervoltage,
            } => protocol::encode_voltage_monitor_thresholds(*overvoltage, *undervoltage).to_vec(),
            Self::VoltageMonitorFilter(alpha) => {
                protocol::encode_voltage_monitor_filter(*alpha).to_vec()
            }
            Self::PwmConfig {
                frequency,
                dead_time,
            } => protocol::encode_pwm_config(*frequency, *dead_time).to_vec(),
            Self::CanConfig(bitrate) => protocol::encode_can_config(*bitrate).to_vec(),
            Self::ControlTiming(period_us) => protocol::encode_control_timing(*period_us).to_vec(),
        }
    }
}
