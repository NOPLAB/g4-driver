//! CAN communication protocol definitions for G4 BLDC motor driver
//!
//! This crate provides CAN protocol definitions shared between
//! firmware (no_std) and controller (std) applications.

#![no_std]

/// CAN message IDs
pub mod can_ids {
    /// Speed command (f32 RPM, 4 bytes)
    pub const SPEED_CMD: u32 = 0x100;

    /// PI gains setting (Kp: f32, Ki: f32, 8 bytes)
    pub const PI_GAINS: u32 = 0x101;

    /// Motor enable command (u8, 1 byte: 0=disable, 1=enable)
    pub const ENABLE_CMD: u32 = 0x102;

    /// Save config to flash command (no data)
    pub const SAVE_CONFIG: u32 = 0x103;

    /// Reload config from flash command (no data)
    pub const RELOAD_CONFIG: u32 = 0x104;

    /// Reset config to defaults command (no data)
    pub const RESET_CONFIG: u32 = 0x105;

    /// Start calibration command (no data, or optionally 1 byte for torque 0-100)
    pub const START_CALIBRATION: u32 = 0x106;

    // === Motor Control Parameter Commands (0x110-0x113) ===
    /// Motor voltage params (max_voltage: f32, v_dc_bus: f32, 8 bytes)
    pub const MOTOR_VOLTAGE_PARAMS: u32 = 0x110;

    /// Motor basic params (pole_pairs: u8, max_duty: u16, 3 bytes)
    pub const MOTOR_BASIC_PARAMS: u32 = 0x111;

    /// Hall sensor params (speed_filter_alpha: f32, hall_angle_offset: f32, 8 bytes)
    pub const HALL_SENSOR_PARAMS: u32 = 0x112;

    /// Angle interpolation (enable_angle_interpolation: bool, 1 byte)
    pub const ANGLE_INTERPOLATION: u32 = 0x113;

    // === OpenLoop Parameter Commands (0x120-0x121) ===
    /// OpenLoop RPM params (initial_rpm: f32, target_rpm: f32, 8 bytes)
    pub const OPENLOOP_RPM_PARAMS: u32 = 0x120;

    /// OpenLoop accel/duty params (acceleration: f32, duty_ratio: u16, 6 bytes)
    pub const OPENLOOP_ACCEL_DUTY_PARAMS: u32 = 0x121;

    // === PWM Configuration (0x130) ===
    /// PWM config (frequency: u32, dead_time: u16, 6 bytes)
    pub const PWM_CONFIG: u32 = 0x130;

    // === CAN Configuration (0x140) ===
    /// CAN config (bitrate: u32, 4 bytes)
    pub const CAN_CONFIG: u32 = 0x140;

    // === Control Timing (0x150) ===
    /// Control timing (control_period_us: u64, 8 bytes)
    pub const CONTROL_TIMING: u32 = 0x150;

    // === Advance Angle (0x160-0x161) ===
    /// Advance angle params (base_deg: f32, max_deg: f32, 8 bytes)
    pub const ADVANCE_ANGLE_PARAMS: u32 = 0x160;
    /// Advance angle speed range (min_speed: f32, max_speed: f32, 8 bytes)
    pub const ADVANCE_ANGLE_SPEED: u32 = 0x161;

    // === Min Voltage (0x162-0x163) ===
    /// Min voltage params (min_voltage: f32, error_threshold: f32, 8 bytes)
    pub const MIN_VOLTAGE_PARAMS: u32 = 0x162;
    /// Max speed acceleration (max_speed_accel: f32, 4 bytes)
    pub const MAX_SPEED_ACCEL: u32 = 0x163;

    // === FOC Stall Detection (0x164) ===
    /// FOC stall params (speed_threshold: f32, count_threshold: u32, 8 bytes)
    pub const FOC_STALL_PARAMS: u32 = 0x164;

    // === OpenLoop Cycles (0x165) ===
    /// OpenLoop cycles params (forced_cycles: u32, min_cycles: u32, 8 bytes)
    pub const OPENLOOP_CYCLES_PARAMS: u32 = 0x165;

    // === Dead Time Compensation (0x166) ===
    /// Dead time compensation params (enabled: u8, dead_time_ns: f32, 5 bytes)
    pub const DEAD_TIME_COMP_PARAMS: u32 = 0x166;

    // === Flux Weakening (0x167-0x169) ===
    /// Flux weakening enable (enabled: u8, min_speed: f32, 5 bytes)
    pub const FLUX_WEAKENING_ENABLE: u32 = 0x167;
    /// Flux weakening params (max_speed: f32, max_ratio: f32, 8 bytes)
    pub const FLUX_WEAKENING_PARAMS: u32 = 0x168;
    /// Flux weakening Vd rate (vd_rate_limit: f32, 4 bytes)
    pub const FLUX_WEAKENING_VD: u32 = 0x169;

    // === Voltage Monitor (0x16A-0x16B) ===
    /// Voltage monitor thresholds (overvoltage: f32, undervoltage: f32, 8 bytes)
    pub const VOLTAGE_MONITOR_THRESHOLDS: u32 = 0x16A;
    /// Voltage monitor filter (filter_alpha: f32, 4 bytes)
    pub const VOLTAGE_MONITOR_FILTER: u32 = 0x16B;

    /// Motor status feedback (speed: f32, angle: f32, 8 bytes)
    pub const STATUS: u32 = 0x200;

    /// Voltage status feedback (voltage: f32, flags: u8, 5 bytes)
    pub const VOLTAGE_STATUS: u32 = 0x201;

    /// Config status feedback (version: u16, crc_valid: u8, 3 bytes)
    pub const CONFIG_STATUS: u32 = 0x202;

    /// Calibration status feedback (electrical_offset: f32, direction_inversed: u8, success: u8, 6 bytes)
    pub const CALIBRATION_STATUS: u32 = 0x203;

    /// Emergency stop (any data length)
    pub const EMERGENCY_STOP: u32 = 0x000;
}

/// Motor status structure
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorStatus {
    /// Current motor speed in RPM
    pub speed_rpm: f32,
    /// Current electrical angle in radians
    pub electrical_angle: f32,
}

impl MotorStatus {
    /// Creates a new MotorStatus with default values
    pub const fn new() -> Self {
        Self {
            speed_rpm: 0.0,
            electrical_angle: 0.0,
        }
    }
}

impl Default for MotorStatus {
    fn default() -> Self {
        Self::new()
    }
}

/// Voltage status structure
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct VoltageStatus {
    /// DC bus voltage in volts
    pub voltage: f32,
    /// Overvoltage condition flag
    pub overvoltage: bool,
    /// Undervoltage condition flag
    pub undervoltage: bool,
}

impl VoltageStatus {
    /// Creates a new VoltageStatus with default values
    pub const fn new() -> Self {
        Self {
            voltage: 0.0,
            overvoltage: false,
            undervoltage: false,
        }
    }
}

impl Default for VoltageStatus {
    fn default() -> Self {
        Self::new()
    }
}

/// Calibration status structure
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CalibrationStatus {
    /// Electrical offset in radians (0〜2π)
    pub electrical_offset: f32,
    /// Direction inversion flag
    pub direction_inversed: bool,
    /// Calibration success flag
    pub success: bool,
}

impl CalibrationStatus {
    /// Creates a new CalibrationStatus with default values
    pub const fn new() -> Self {
        Self {
            electrical_offset: 0.0,
            direction_inversed: false,
            success: false,
        }
    }
}

impl Default for CalibrationStatus {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Parse Functions (decode incoming CAN messages)
// ============================================================================

/// Parse speed command from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 4 bytes)
///
/// # Returns
/// * `Some(speed_rpm)` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_speed_command(data: &[u8]) -> Option<f32> {
    if data.len() < 4 {
        return None;
    }

    let speed_bytes = [data[0], data[1], data[2], data[3]];
    Some(f32::from_le_bytes(speed_bytes))
}

/// Parse PI gains from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 8 bytes)
///
/// # Returns
/// * `Some((kp, ki))` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_pi_gains(data: &[u8]) -> Option<(f32, f32)> {
    if data.len() < 8 {
        return None;
    }

    let kp_bytes = [data[0], data[1], data[2], data[3]];
    let ki_bytes = [data[4], data[5], data[6], data[7]];

    let kp = f32::from_le_bytes(kp_bytes);
    let ki = f32::from_le_bytes(ki_bytes);

    Some((kp, ki))
}

/// Parse enable command from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be at least 1 byte)
///
/// # Returns
/// * `Some(true)` if enable command (data[0] != 0)
/// * `Some(false)` if disable command (data[0] == 0)
/// * `None` if data length is incorrect
pub fn parse_enable_command(data: &[u8]) -> Option<bool> {
    if data.is_empty() {
        return None;
    }

    Some(data[0] != 0)
}

/// Parse motor voltage parameters from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 8 bytes)
///
/// # Returns
/// * `Some((max_voltage, v_dc_bus))` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_motor_voltage_params(data: &[u8]) -> Option<(f32, f32)> {
    if data.len() < 8 {
        return None;
    }

    let max_voltage_bytes = [data[0], data[1], data[2], data[3]];
    let v_dc_bus_bytes = [data[4], data[5], data[6], data[7]];

    let max_voltage = f32::from_le_bytes(max_voltage_bytes);
    let v_dc_bus = f32::from_le_bytes(v_dc_bus_bytes);

    Some((max_voltage, v_dc_bus))
}

/// Parse motor basic parameters from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 3 bytes)
///
/// # Returns
/// * `Some((pole_pairs, max_duty))` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_motor_basic_params(data: &[u8]) -> Option<(u8, u16)> {
    if data.len() < 3 {
        return None;
    }

    let pole_pairs = data[0];
    let max_duty_bytes = [data[1], data[2]];
    let max_duty = u16::from_le_bytes(max_duty_bytes);

    Some((pole_pairs, max_duty))
}

/// Parse hall sensor parameters from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 8 bytes)
///
/// # Returns
/// * `Some((speed_filter_alpha, hall_angle_offset))` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_hall_sensor_params(data: &[u8]) -> Option<(f32, f32)> {
    if data.len() < 8 {
        return None;
    }

    let alpha_bytes = [data[0], data[1], data[2], data[3]];
    let offset_bytes = [data[4], data[5], data[6], data[7]];

    let speed_filter_alpha = f32::from_le_bytes(alpha_bytes);
    let hall_angle_offset = f32::from_le_bytes(offset_bytes);

    Some((speed_filter_alpha, hall_angle_offset))
}

/// Parse angle interpolation setting from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be at least 1 byte)
///
/// # Returns
/// * `Some(enable)` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_angle_interpolation(data: &[u8]) -> Option<bool> {
    if data.is_empty() {
        return None;
    }

    Some(data[0] != 0)
}

/// Parse openloop RPM parameters from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 8 bytes)
///
/// # Returns
/// * `Some((initial_rpm, target_rpm))` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_openloop_rpm_params(data: &[u8]) -> Option<(f32, f32)> {
    if data.len() < 8 {
        return None;
    }

    let initial_bytes = [data[0], data[1], data[2], data[3]];
    let target_bytes = [data[4], data[5], data[6], data[7]];

    let initial_rpm = f32::from_le_bytes(initial_bytes);
    let target_rpm = f32::from_le_bytes(target_bytes);

    Some((initial_rpm, target_rpm))
}

/// Parse openloop acceleration/duty parameters from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 6 bytes)
///
/// # Returns
/// * `Some((acceleration, duty_ratio))` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_openloop_accel_duty_params(data: &[u8]) -> Option<(f32, u16)> {
    if data.len() < 6 {
        return None;
    }

    let accel_bytes = [data[0], data[1], data[2], data[3]];
    let duty_bytes = [data[4], data[5]];

    let acceleration = f32::from_le_bytes(accel_bytes);
    let duty_ratio = u16::from_le_bytes(duty_bytes);

    Some((acceleration, duty_ratio))
}

/// Parse PWM configuration from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 6 bytes)
///
/// # Returns
/// * `Some((frequency, dead_time))` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_pwm_config(data: &[u8]) -> Option<(u32, u16)> {
    if data.len() < 6 {
        return None;
    }

    let freq_bytes = [data[0], data[1], data[2], data[3]];
    let dead_time_bytes = [data[4], data[5]];

    let frequency = u32::from_le_bytes(freq_bytes);
    let dead_time = u16::from_le_bytes(dead_time_bytes);

    Some((frequency, dead_time))
}

/// Parse CAN configuration from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 4 bytes)
///
/// # Returns
/// * `Some(bitrate)` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_can_config(data: &[u8]) -> Option<u32> {
    if data.len() < 4 {
        return None;
    }

    let bitrate_bytes = [data[0], data[1], data[2], data[3]];
    Some(u32::from_le_bytes(bitrate_bytes))
}

/// Parse control timing from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 8 bytes)
///
/// # Returns
/// * `Some(control_period_us)` if parsing successful
/// * `None` if data length is incorrect
pub fn parse_control_timing(data: &[u8]) -> Option<u64> {
    if data.len() < 8 {
        return None;
    }

    let period_bytes = [
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
    ];
    Some(u64::from_le_bytes(period_bytes))
}

// ============================================================================
// Parse Functions for Extended Parameters
// ============================================================================

/// Parse advance angle parameters from CAN data
///
/// # Returns
/// * `Some((base_deg, max_deg))` if parsing successful
pub fn parse_advance_angle_params(data: &[u8]) -> Option<(f32, f32)> {
    if data.len() < 8 {
        return None;
    }

    let base_bytes = [data[0], data[1], data[2], data[3]];
    let max_bytes = [data[4], data[5], data[6], data[7]];

    Some((
        f32::from_le_bytes(base_bytes),
        f32::from_le_bytes(max_bytes),
    ))
}

/// Parse advance angle speed range from CAN data
///
/// # Returns
/// * `Some((min_speed, max_speed))` if parsing successful
pub fn parse_advance_angle_speed(data: &[u8]) -> Option<(f32, f32)> {
    if data.len() < 8 {
        return None;
    }

    let min_bytes = [data[0], data[1], data[2], data[3]];
    let max_bytes = [data[4], data[5], data[6], data[7]];

    Some((f32::from_le_bytes(min_bytes), f32::from_le_bytes(max_bytes)))
}

/// Parse min voltage parameters from CAN data
///
/// # Returns
/// * `Some((min_voltage, error_threshold))` if parsing successful
pub fn parse_min_voltage_params(data: &[u8]) -> Option<(f32, f32)> {
    if data.len() < 8 {
        return None;
    }

    let min_bytes = [data[0], data[1], data[2], data[3]];
    let threshold_bytes = [data[4], data[5], data[6], data[7]];

    Some((
        f32::from_le_bytes(min_bytes),
        f32::from_le_bytes(threshold_bytes),
    ))
}

/// Parse max speed acceleration from CAN data
///
/// # Returns
/// * `Some(max_speed_accel)` if parsing successful
pub fn parse_max_speed_accel(data: &[u8]) -> Option<f32> {
    if data.len() < 4 {
        return None;
    }

    let accel_bytes = [data[0], data[1], data[2], data[3]];
    Some(f32::from_le_bytes(accel_bytes))
}

/// Parse FOC stall parameters from CAN data
///
/// # Returns
/// * `Some((speed_threshold, count_threshold))` if parsing successful
pub fn parse_foc_stall_params(data: &[u8]) -> Option<(f32, u32)> {
    if data.len() < 8 {
        return None;
    }

    let speed_bytes = [data[0], data[1], data[2], data[3]];
    let count_bytes = [data[4], data[5], data[6], data[7]];

    Some((
        f32::from_le_bytes(speed_bytes),
        u32::from_le_bytes(count_bytes),
    ))
}

/// Parse openloop cycles parameters from CAN data
///
/// # Returns
/// * `Some((forced_cycles, min_cycles))` if parsing successful
pub fn parse_openloop_cycles_params(data: &[u8]) -> Option<(u32, u32)> {
    if data.len() < 8 {
        return None;
    }

    let forced_bytes = [data[0], data[1], data[2], data[3]];
    let min_bytes = [data[4], data[5], data[6], data[7]];

    Some((
        u32::from_le_bytes(forced_bytes),
        u32::from_le_bytes(min_bytes),
    ))
}

/// Parse dead time compensation parameters from CAN data
///
/// # Returns
/// * `Some((enabled, dead_time_ns))` if parsing successful
pub fn parse_dead_time_comp_params(data: &[u8]) -> Option<(bool, f32)> {
    if data.len() < 5 {
        return None;
    }

    let enabled = data[0] != 0;
    let dead_time_bytes = [data[1], data[2], data[3], data[4]];

    Some((enabled, f32::from_le_bytes(dead_time_bytes)))
}

/// Parse flux weakening enable from CAN data
///
/// # Returns
/// * `Some((enabled, min_speed))` if parsing successful
pub fn parse_flux_weakening_enable(data: &[u8]) -> Option<(bool, f32)> {
    if data.len() < 5 {
        return None;
    }

    let enabled = data[0] != 0;
    let min_speed_bytes = [data[1], data[2], data[3], data[4]];

    Some((enabled, f32::from_le_bytes(min_speed_bytes)))
}

/// Parse flux weakening parameters from CAN data
///
/// # Returns
/// * `Some((max_speed, max_ratio))` if parsing successful
pub fn parse_flux_weakening_params(data: &[u8]) -> Option<(f32, f32)> {
    if data.len() < 8 {
        return None;
    }

    let max_speed_bytes = [data[0], data[1], data[2], data[3]];
    let max_ratio_bytes = [data[4], data[5], data[6], data[7]];

    Some((
        f32::from_le_bytes(max_speed_bytes),
        f32::from_le_bytes(max_ratio_bytes),
    ))
}

/// Parse flux weakening Vd rate limit from CAN data
///
/// # Returns
/// * `Some(vd_rate_limit)` if parsing successful
pub fn parse_flux_weakening_vd(data: &[u8]) -> Option<f32> {
    if data.len() < 4 {
        return None;
    }

    let vd_bytes = [data[0], data[1], data[2], data[3]];
    Some(f32::from_le_bytes(vd_bytes))
}

/// Parse voltage monitor thresholds from CAN data
///
/// # Returns
/// * `Some((overvoltage, undervoltage))` if parsing successful
pub fn parse_voltage_monitor_thresholds(data: &[u8]) -> Option<(f32, f32)> {
    if data.len() < 8 {
        return None;
    }

    let over_bytes = [data[0], data[1], data[2], data[3]];
    let under_bytes = [data[4], data[5], data[6], data[7]];

    Some((
        f32::from_le_bytes(over_bytes),
        f32::from_le_bytes(under_bytes),
    ))
}

/// Parse voltage monitor filter from CAN data
///
/// # Returns
/// * `Some(filter_alpha)` if parsing successful
pub fn parse_voltage_monitor_filter(data: &[u8]) -> Option<f32> {
    if data.len() < 4 {
        return None;
    }

    let alpha_bytes = [data[0], data[1], data[2], data[3]];
    Some(f32::from_le_bytes(alpha_bytes))
}

// ============================================================================
// Encode Functions (create outgoing CAN messages)
// All functions return fixed-size arrays for no_std compatibility
// ============================================================================

/// Encode speed command into CAN data
///
/// # Arguments
/// * `speed_rpm` - Target speed in RPM
///
/// # Returns
/// 4-byte array containing encoded speed
pub fn encode_speed_command(speed_rpm: f32) -> [u8; 4] {
    speed_rpm.to_le_bytes()
}

/// Encode PI gains into CAN data
///
/// # Arguments
/// * `kp` - Proportional gain
/// * `ki` - Integral gain
///
/// # Returns
/// 8-byte array containing encoded PI gains
pub fn encode_pi_gains(kp: f32, ki: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&kp.to_le_bytes());
    data[4..8].copy_from_slice(&ki.to_le_bytes());
    data
}

/// Encode enable command into CAN data
///
/// # Arguments
/// * `enable` - Motor enable flag
///
/// # Returns
/// 1-byte array containing encoded enable command
pub fn encode_enable_command(enable: bool) -> [u8; 1] {
    [if enable { 1 } else { 0 }]
}

/// Encode motor status into CAN data
///
/// # Arguments
/// * `speed_rpm` - Current motor speed in RPM
/// * `electrical_angle` - Current electrical angle in radians
///
/// # Returns
/// 8-byte array containing encoded status
pub fn encode_status(speed_rpm: f32, electrical_angle: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&speed_rpm.to_le_bytes());
    data[4..8].copy_from_slice(&electrical_angle.to_le_bytes());
    data
}

/// Encode voltage status into CAN data
///
/// # Arguments
/// * `voltage` - DC bus voltage in volts
/// * `overvoltage` - Overvoltage flag
/// * `undervoltage` - Undervoltage flag
///
/// # Returns
/// 5-byte array containing encoded voltage status
pub fn encode_voltage_status(voltage: f32, overvoltage: bool, undervoltage: bool) -> [u8; 5] {
    let mut data = [0u8; 5];
    data[0..4].copy_from_slice(&voltage.to_le_bytes());

    let mut flags = 0u8;
    if overvoltage {
        flags |= 0x01;
    }
    if undervoltage {
        flags |= 0x02;
    }
    data[4] = flags;

    data
}

/// Encode config status into CAN data
///
/// # Arguments
/// * `version` - Config version number
/// * `crc_valid` - Whether CRC validation passed
///
/// # Returns
/// 3-byte array containing encoded config status
pub fn encode_config_status(version: u16, crc_valid: bool) -> [u8; 3] {
    let mut data = [0u8; 3];
    data[0..2].copy_from_slice(&version.to_le_bytes());
    data[2] = if crc_valid { 1 } else { 0 };
    data
}

/// Encode calibration status into CAN data
///
/// # Arguments
/// * `electrical_offset` - Electrical offset in radians (0〜2π)
/// * `direction_inversed` - Direction inversion flag
/// * `success` - Calibration success flag
///
/// # Returns
/// 6-byte array containing encoded calibration status
pub fn encode_calibration_status(
    electrical_offset: f32,
    direction_inversed: bool,
    success: bool,
) -> [u8; 6] {
    let mut data = [0u8; 6];
    data[0..4].copy_from_slice(&electrical_offset.to_le_bytes());
    data[4] = if direction_inversed { 1 } else { 0 };
    data[5] = if success { 1 } else { 0 };
    data
}

/// Encode motor voltage parameters into CAN data
pub fn encode_motor_voltage_params(max_voltage: f32, v_dc_bus: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&max_voltage.to_le_bytes());
    data[4..8].copy_from_slice(&v_dc_bus.to_le_bytes());
    data
}

/// Encode motor basic parameters into CAN data
pub fn encode_motor_basic_params(pole_pairs: u8, max_duty: u16) -> [u8; 3] {
    let mut data = [0u8; 3];
    data[0] = pole_pairs;
    data[1..3].copy_from_slice(&max_duty.to_le_bytes());
    data
}

/// Encode hall sensor parameters into CAN data
pub fn encode_hall_sensor_params(speed_filter_alpha: f32, hall_angle_offset: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&speed_filter_alpha.to_le_bytes());
    data[4..8].copy_from_slice(&hall_angle_offset.to_le_bytes());
    data
}

/// Encode angle interpolation setting into CAN data
pub fn encode_angle_interpolation(enable: bool) -> [u8; 1] {
    [if enable { 1 } else { 0 }]
}

/// Encode openloop RPM parameters into CAN data
pub fn encode_openloop_rpm_params(initial_rpm: f32, target_rpm: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&initial_rpm.to_le_bytes());
    data[4..8].copy_from_slice(&target_rpm.to_le_bytes());
    data
}

/// Encode openloop acceleration/duty parameters into CAN data
pub fn encode_openloop_accel_duty_params(acceleration: f32, duty_ratio: u16) -> [u8; 6] {
    let mut data = [0u8; 6];
    data[0..4].copy_from_slice(&acceleration.to_le_bytes());
    data[4..6].copy_from_slice(&duty_ratio.to_le_bytes());
    data
}

/// Encode PWM configuration into CAN data
pub fn encode_pwm_config(frequency: u32, dead_time: u16) -> [u8; 6] {
    let mut data = [0u8; 6];
    data[0..4].copy_from_slice(&frequency.to_le_bytes());
    data[4..6].copy_from_slice(&dead_time.to_le_bytes());
    data
}

/// Encode CAN configuration into CAN data
pub fn encode_can_config(bitrate: u32) -> [u8; 4] {
    bitrate.to_le_bytes()
}

/// Encode control timing into CAN data
pub fn encode_control_timing(control_period_us: u64) -> [u8; 8] {
    control_period_us.to_le_bytes()
}

/// Encode start calibration command into CAN data
///
/// # Arguments
/// * `torque` - Optional torque value (0-100). If None, returns empty array.
///
/// # Returns
/// 1-byte array with torque value, or empty array if None
pub fn encode_start_calibration(torque: Option<u8>) -> [u8; 1] {
    [torque.map(|t| t.min(100)).unwrap_or(0)]
}

/// Encode start calibration command without torque (for firmware use)
pub fn encode_start_calibration_empty() -> [u8; 0] {
    []
}

// ============================================================================
// Encode Functions for Extended Parameters
// ============================================================================

/// Encode advance angle parameters into CAN data
pub fn encode_advance_angle_params(base_deg: f32, max_deg: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&base_deg.to_le_bytes());
    data[4..8].copy_from_slice(&max_deg.to_le_bytes());
    data
}

/// Encode advance angle speed range into CAN data
pub fn encode_advance_angle_speed(min_speed: f32, max_speed: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&min_speed.to_le_bytes());
    data[4..8].copy_from_slice(&max_speed.to_le_bytes());
    data
}

/// Encode min voltage parameters into CAN data
pub fn encode_min_voltage_params(min_voltage: f32, error_threshold: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&min_voltage.to_le_bytes());
    data[4..8].copy_from_slice(&error_threshold.to_le_bytes());
    data
}

/// Encode max speed acceleration into CAN data
pub fn encode_max_speed_accel(max_speed_accel: f32) -> [u8; 4] {
    max_speed_accel.to_le_bytes()
}

/// Encode FOC stall parameters into CAN data
pub fn encode_foc_stall_params(speed_threshold: f32, count_threshold: u32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&speed_threshold.to_le_bytes());
    data[4..8].copy_from_slice(&count_threshold.to_le_bytes());
    data
}

/// Encode openloop cycles parameters into CAN data
pub fn encode_openloop_cycles_params(forced_cycles: u32, min_cycles: u32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&forced_cycles.to_le_bytes());
    data[4..8].copy_from_slice(&min_cycles.to_le_bytes());
    data
}

/// Encode dead time compensation parameters into CAN data
pub fn encode_dead_time_comp_params(enabled: bool, dead_time_ns: f32) -> [u8; 5] {
    let mut data = [0u8; 5];
    data[0] = if enabled { 1 } else { 0 };
    data[1..5].copy_from_slice(&dead_time_ns.to_le_bytes());
    data
}

/// Encode flux weakening enable into CAN data
pub fn encode_flux_weakening_enable(enabled: bool, min_speed: f32) -> [u8; 5] {
    let mut data = [0u8; 5];
    data[0] = if enabled { 1 } else { 0 };
    data[1..5].copy_from_slice(&min_speed.to_le_bytes());
    data
}

/// Encode flux weakening parameters into CAN data
pub fn encode_flux_weakening_params(max_speed: f32, max_ratio: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&max_speed.to_le_bytes());
    data[4..8].copy_from_slice(&max_ratio.to_le_bytes());
    data
}

/// Encode flux weakening Vd rate limit into CAN data
pub fn encode_flux_weakening_vd(vd_rate_limit: f32) -> [u8; 4] {
    vd_rate_limit.to_le_bytes()
}

/// Encode voltage monitor thresholds into CAN data
pub fn encode_voltage_monitor_thresholds(overvoltage: f32, undervoltage: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&overvoltage.to_le_bytes());
    data[4..8].copy_from_slice(&undervoltage.to_le_bytes());
    data
}

/// Encode voltage monitor filter into CAN data
pub fn encode_voltage_monitor_filter(filter_alpha: f32) -> [u8; 4] {
    filter_alpha.to_le_bytes()
}

// ============================================================================
// Decode Functions (parse response CAN messages into structures)
// ============================================================================

/// Decode motor status from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 8 bytes)
///
/// # Returns
/// * `Some(MotorStatus)` if parsing successful
/// * `None` if data length is incorrect
pub fn decode_status(data: &[u8]) -> Option<MotorStatus> {
    if data.len() < 8 {
        return None;
    }

    let speed_bytes = [data[0], data[1], data[2], data[3]];
    let angle_bytes = [data[4], data[5], data[6], data[7]];

    let speed_rpm = f32::from_le_bytes(speed_bytes);
    let electrical_angle = f32::from_le_bytes(angle_bytes);

    Some(MotorStatus {
        speed_rpm,
        electrical_angle,
    })
}

/// Decode voltage status from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be at least 5 bytes)
///
/// # Returns
/// * `Some(VoltageStatus)` if parsing successful
/// * `None` if data length is incorrect
pub fn decode_voltage_status(data: &[u8]) -> Option<VoltageStatus> {
    if data.len() < 5 {
        return None;
    }

    let voltage_bytes = [data[0], data[1], data[2], data[3]];
    let voltage = f32::from_le_bytes(voltage_bytes);

    let flags = data[4];
    let overvoltage = (flags & 0x01) != 0;
    let undervoltage = (flags & 0x02) != 0;

    Some(VoltageStatus {
        voltage,
        overvoltage,
        undervoltage,
    })
}

/// Decode config status from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be at least 3 bytes)
///
/// # Returns
/// * `Some((version, crc_valid))` if parsing successful
/// * `None` if data length is incorrect
pub fn decode_config_status(data: &[u8]) -> Option<(u16, bool)> {
    if data.len() < 3 {
        return None;
    }

    let version_bytes = [data[0], data[1]];
    let version = u16::from_le_bytes(version_bytes);
    let crc_valid = data[2] != 0;

    Some((version, crc_valid))
}

/// Decode calibration status from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be at least 6 bytes)
///
/// # Returns
/// * `Some(CalibrationStatus)` if parsing successful
/// * `None` if data length is incorrect
pub fn decode_calibration_status(data: &[u8]) -> Option<CalibrationStatus> {
    if data.len() < 6 {
        return None;
    }

    let offset_bytes = [data[0], data[1], data[2], data[3]];
    let electrical_offset = f32::from_le_bytes(offset_bytes);

    let direction_inversed = data[4] != 0;
    let success = data[5] != 0;

    Some(CalibrationStatus {
        electrical_offset,
        direction_inversed,
        success,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_speed_command() {
        let speed = 1234.5f32;
        let encoded = encode_speed_command(speed);
        let decoded = parse_speed_command(&encoded);
        assert_eq!(decoded, Some(speed));
    }

    #[test]
    fn test_encode_decode_pi_gains() {
        let kp = 0.1f32;
        let ki = 0.01f32;
        let encoded = encode_pi_gains(kp, ki);
        let decoded = parse_pi_gains(&encoded);
        assert_eq!(decoded, Some((kp, ki)));
    }

    #[test]
    fn test_encode_decode_enable_command() {
        let enable = true;
        let encoded = encode_enable_command(enable);
        let decoded = parse_enable_command(&encoded);
        assert_eq!(decoded, Some(enable));

        let disable = false;
        let encoded = encode_enable_command(disable);
        let decoded = parse_enable_command(&encoded);
        assert_eq!(decoded, Some(disable));
    }

    #[test]
    fn test_encode_decode_status() {
        let speed = 1500.0f32;
        let angle = 2.5f32;

        let encoded = encode_status(speed, angle);
        let decoded = decode_status(&encoded).unwrap();

        assert_eq!(decoded.speed_rpm, speed);
        assert_eq!(decoded.electrical_angle, angle);
    }

    #[test]
    fn test_encode_decode_voltage_status() {
        let voltage = 24.5f32;
        let overvoltage = true;
        let undervoltage = false;

        let encoded = encode_voltage_status(voltage, overvoltage, undervoltage);
        let decoded = decode_voltage_status(&encoded).unwrap();

        assert_eq!(decoded.voltage, voltage);
        assert_eq!(decoded.overvoltage, overvoltage);
        assert_eq!(decoded.undervoltage, undervoltage);
    }

    #[test]
    fn test_encode_decode_config_status() {
        let version = 1u16;
        let crc_valid = true;

        let encoded = encode_config_status(version, crc_valid);
        let decoded = decode_config_status(&encoded).unwrap();

        assert_eq!(decoded.0, version);
        assert_eq!(decoded.1, crc_valid);
    }

    #[test]
    fn test_encode_decode_calibration_status() {
        let offset = 1.57f32;
        let inversed = true;
        let success = true;

        let encoded = encode_calibration_status(offset, inversed, success);
        let decoded = decode_calibration_status(&encoded).unwrap();

        assert_eq!(decoded.electrical_offset, offset);
        assert_eq!(decoded.direction_inversed, inversed);
        assert_eq!(decoded.success, success);
    }

    #[test]
    fn test_encode_decode_motor_voltage_params() {
        let max_voltage = 24.0f32;
        let v_dc_bus = 24.0f32;

        let encoded = encode_motor_voltage_params(max_voltage, v_dc_bus);
        let decoded = parse_motor_voltage_params(&encoded).unwrap();

        assert_eq!(decoded.0, max_voltage);
        assert_eq!(decoded.1, v_dc_bus);
    }

    #[test]
    fn test_encode_decode_motor_basic_params() {
        let pole_pairs = 6u8;
        let max_duty = 100u16;

        let encoded = encode_motor_basic_params(pole_pairs, max_duty);
        let decoded = parse_motor_basic_params(&encoded).unwrap();

        assert_eq!(decoded.0, pole_pairs);
        assert_eq!(decoded.1, max_duty);
    }

    #[test]
    fn test_encode_decode_hall_sensor_params() {
        let alpha = 0.1f32;
        let offset = 1.57f32;

        let encoded = encode_hall_sensor_params(alpha, offset);
        let decoded = parse_hall_sensor_params(&encoded).unwrap();

        assert_eq!(decoded.0, alpha);
        assert_eq!(decoded.1, offset);
    }

    #[test]
    fn test_encode_decode_angle_interpolation() {
        let enable = true;

        let encoded = encode_angle_interpolation(enable);
        let decoded = parse_angle_interpolation(&encoded).unwrap();

        assert_eq!(decoded, enable);
    }

    #[test]
    fn test_encode_decode_openloop_rpm_params() {
        let initial = 100.0f32;
        let target = 500.0f32;

        let encoded = encode_openloop_rpm_params(initial, target);
        let decoded = parse_openloop_rpm_params(&encoded).unwrap();

        assert_eq!(decoded.0, initial);
        assert_eq!(decoded.1, target);
    }

    #[test]
    fn test_encode_decode_openloop_accel_duty_params() {
        let accel = 100.0f32;
        let duty = 50u16;

        let encoded = encode_openloop_accel_duty_params(accel, duty);
        let decoded = parse_openloop_accel_duty_params(&encoded).unwrap();

        assert_eq!(decoded.0, accel);
        assert_eq!(decoded.1, duty);
    }

    #[test]
    fn test_encode_decode_pwm_config() {
        let freq = 50000u32;
        let dead_time = 100u16;

        let encoded = encode_pwm_config(freq, dead_time);
        let decoded = parse_pwm_config(&encoded).unwrap();

        assert_eq!(decoded.0, freq);
        assert_eq!(decoded.1, dead_time);
    }

    #[test]
    fn test_encode_decode_can_config() {
        let bitrate = 250000u32;

        let encoded = encode_can_config(bitrate);
        let decoded = parse_can_config(&encoded).unwrap();

        assert_eq!(decoded, bitrate);
    }

    #[test]
    fn test_encode_decode_control_timing() {
        let period = 400u64;

        let encoded = encode_control_timing(period);
        let decoded = parse_control_timing(&encoded).unwrap();

        assert_eq!(decoded, period);
    }

    #[test]
    fn test_parse_invalid_data_length() {
        // Test with insufficient data length
        assert_eq!(parse_speed_command(&[0, 1, 2]), None);
        assert_eq!(parse_pi_gains(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_enable_command(&[]), None);
        assert_eq!(parse_motor_voltage_params(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_motor_basic_params(&[0, 1]), None);
        assert_eq!(parse_hall_sensor_params(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_angle_interpolation(&[]), None);
        assert_eq!(parse_openloop_rpm_params(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_openloop_accel_duty_params(&[0, 1, 2, 3, 4]), None);
        assert_eq!(parse_pwm_config(&[0, 1, 2, 3, 4]), None);
        assert_eq!(parse_can_config(&[0, 1, 2]), None);
        assert_eq!(parse_control_timing(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(decode_status(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(decode_voltage_status(&[0, 1, 2, 3]), None);
        assert_eq!(decode_config_status(&[0, 1]), None);
        assert_eq!(decode_calibration_status(&[0, 1, 2, 3, 4]), None);
    }

    // ========================================================================
    // Tests for Extended Parameters
    // ========================================================================

    #[test]
    fn test_encode_decode_advance_angle_params() {
        let base = 10.0f32;
        let max = 30.0f32;

        let encoded = encode_advance_angle_params(base, max);
        let decoded = parse_advance_angle_params(&encoded).unwrap();

        assert_eq!(decoded.0, base);
        assert_eq!(decoded.1, max);
    }

    #[test]
    fn test_encode_decode_advance_angle_speed() {
        let min_speed = 100.0f32;
        let max_speed = 3000.0f32;

        let encoded = encode_advance_angle_speed(min_speed, max_speed);
        let decoded = parse_advance_angle_speed(&encoded).unwrap();

        assert_eq!(decoded.0, min_speed);
        assert_eq!(decoded.1, max_speed);
    }

    #[test]
    fn test_encode_decode_min_voltage_params() {
        let min_voltage = 2.0f32;
        let error_threshold = 2.0f32;

        let encoded = encode_min_voltage_params(min_voltage, error_threshold);
        let decoded = parse_min_voltage_params(&encoded).unwrap();

        assert_eq!(decoded.0, min_voltage);
        assert_eq!(decoded.1, error_threshold);
    }

    #[test]
    fn test_encode_decode_max_speed_accel() {
        let accel = 100.0f32;

        let encoded = encode_max_speed_accel(accel);
        let decoded = parse_max_speed_accel(&encoded).unwrap();

        assert_eq!(decoded, accel);
    }

    #[test]
    fn test_encode_decode_foc_stall_params() {
        let speed = 50.0f32;
        let count = 1000u32;

        let encoded = encode_foc_stall_params(speed, count);
        let decoded = parse_foc_stall_params(&encoded).unwrap();

        assert_eq!(decoded.0, speed);
        assert_eq!(decoded.1, count);
    }

    #[test]
    fn test_encode_decode_openloop_cycles_params() {
        let forced = 10000u32;
        let min = 10000u32;

        let encoded = encode_openloop_cycles_params(forced, min);
        let decoded = parse_openloop_cycles_params(&encoded).unwrap();

        assert_eq!(decoded.0, forced);
        assert_eq!(decoded.1, min);
    }

    #[test]
    fn test_encode_decode_dead_time_comp_params() {
        let enabled = true;
        let dead_time = 100.0f32;

        let encoded = encode_dead_time_comp_params(enabled, dead_time);
        let decoded = parse_dead_time_comp_params(&encoded).unwrap();

        assert_eq!(decoded.0, enabled);
        assert_eq!(decoded.1, dead_time);
    }

    #[test]
    fn test_encode_decode_flux_weakening_enable() {
        let enabled = true;
        let min_speed = 2000.0f32;

        let encoded = encode_flux_weakening_enable(enabled, min_speed);
        let decoded = parse_flux_weakening_enable(&encoded).unwrap();

        assert_eq!(decoded.0, enabled);
        assert_eq!(decoded.1, min_speed);
    }

    #[test]
    fn test_encode_decode_flux_weakening_params() {
        let max_speed = 4000.0f32;
        let max_ratio = 0.5f32;

        let encoded = encode_flux_weakening_params(max_speed, max_ratio);
        let decoded = parse_flux_weakening_params(&encoded).unwrap();

        assert_eq!(decoded.0, max_speed);
        assert_eq!(decoded.1, max_ratio);
    }

    #[test]
    fn test_encode_decode_flux_weakening_vd() {
        let vd_rate = 100.0f32;

        let encoded = encode_flux_weakening_vd(vd_rate);
        let decoded = parse_flux_weakening_vd(&encoded).unwrap();

        assert_eq!(decoded, vd_rate);
    }

    #[test]
    fn test_encode_decode_voltage_monitor_thresholds() {
        let overvoltage = 30.0f32;
        let undervoltage = 10.0f32;

        let encoded = encode_voltage_monitor_thresholds(overvoltage, undervoltage);
        let decoded = parse_voltage_monitor_thresholds(&encoded).unwrap();

        assert_eq!(decoded.0, overvoltage);
        assert_eq!(decoded.1, undervoltage);
    }

    #[test]
    fn test_encode_decode_voltage_monitor_filter() {
        let alpha = 0.1f32;

        let encoded = encode_voltage_monitor_filter(alpha);
        let decoded = parse_voltage_monitor_filter(&encoded).unwrap();

        assert_eq!(decoded, alpha);
    }

    #[test]
    fn test_parse_extended_params_invalid_length() {
        assert_eq!(parse_advance_angle_params(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_advance_angle_speed(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_min_voltage_params(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_max_speed_accel(&[0, 1, 2]), None);
        assert_eq!(parse_foc_stall_params(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_openloop_cycles_params(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_dead_time_comp_params(&[0, 1, 2, 3]), None);
        assert_eq!(parse_flux_weakening_enable(&[0, 1, 2, 3]), None);
        assert_eq!(parse_flux_weakening_params(&[0, 1, 2, 3, 4, 5, 6]), None);
        assert_eq!(parse_flux_weakening_vd(&[0, 1, 2]), None);
        assert_eq!(
            parse_voltage_monitor_thresholds(&[0, 1, 2, 3, 4, 5, 6]),
            None
        );
        assert_eq!(parse_voltage_monitor_filter(&[0, 1, 2]), None);
    }
}
