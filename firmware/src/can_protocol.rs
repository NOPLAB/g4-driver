// CAN communication protocol definitions for motor control

use crate::fmt::*;

/// CAN message IDs
pub mod can_ids {
    /// Speed command (f32 RPM, 4 bytes)
    pub const SPEED_CMD: u32 = 0x100;

    /// PI gains setting (Kp: f32, Ki: f32, 8 bytes)
    pub const PI_GAINS: u32 = 0x101;

    /// Motor enable command (u8, 1 byte: 0=disable, 1=enable)
    pub const ENABLE_CMD: u32 = 0x102;

    /// Start calibration command (no data, or optionally 1 byte for torque 0-100)
    pub const START_CALIBRATION: u32 = 0x106;

    /// Save config to flash command (no data)
    pub const SAVE_CONFIG: u32 = 0x103;

    /// Reload config from flash command (no data)
    pub const RELOAD_CONFIG: u32 = 0x104;

    /// Reset config to defaults command (no data)
    pub const RESET_CONFIG: u32 = 0x105;

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
#[derive(Debug, Clone, Copy)]
pub struct MotorStatus {
    pub speed_rpm: f32,
    pub electrical_angle: f32,
}

impl MotorStatus {
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
#[derive(Debug, Clone, Copy)]
pub struct VoltageStatus {
    pub voltage: f32,
    pub overvoltage: bool,
    pub undervoltage: bool,
}

impl VoltageStatus {
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
        error!("Speed command: invalid data length {}", data.len());
        return None;
    }

    // Parse as little-endian f32
    let speed_bytes = [data[0], data[1], data[2], data[3]];
    let speed_rpm = f32::from_le_bytes(speed_bytes);

    info!("Speed command received: {} RPM", speed_rpm);
    Some(speed_rpm)
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
        error!("PI gains: invalid data length {}", data.len());
        return None;
    }

    // Parse as little-endian f32 values
    let kp_bytes = [data[0], data[1], data[2], data[3]];
    let ki_bytes = [data[4], data[5], data[6], data[7]];

    let kp = f32::from_le_bytes(kp_bytes);
    let ki = f32::from_le_bytes(ki_bytes);

    info!("PI gains received: Kp={}, Ki={}", kp, ki);
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
        error!("Enable command: no data");
        return None;
    }

    let enable = data[0] != 0;
    info!("Motor enable command: {}", enable);
    Some(enable)
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

    // Encode speed as little-endian f32 (bytes 0-3)
    let speed_bytes = speed_rpm.to_le_bytes();
    data[0..4].copy_from_slice(&speed_bytes);

    // Encode angle as little-endian f32 (bytes 4-7)
    let angle_bytes = electrical_angle.to_le_bytes();
    data[4..8].copy_from_slice(&angle_bytes);

    data
}

/// Decode motor status from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be 8 bytes)
///
/// # Returns
/// * `Some(MotorStatus)` if parsing successful
/// * `None` if data length is incorrect
#[allow(dead_code)]
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

    // Encode voltage as little-endian f32 (bytes 0-3)
    let voltage_bytes = voltage.to_le_bytes();
    data[0..4].copy_from_slice(&voltage_bytes);

    // Encode flags (byte 4)
    // Bit 0: overvoltage
    // Bit 1: undervoltage
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

/// Decode voltage status from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be at least 5 bytes)
///
/// # Returns
/// * `Some(VoltageStatus)` if parsing successful
/// * `None` if data length is incorrect
#[allow(dead_code)]
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

    // Encode version as little-endian u16 (bytes 0-1)
    let version_bytes = version.to_le_bytes();
    data[0..2].copy_from_slice(&version_bytes);

    // Encode CRC valid flag (byte 2)
    data[2] = if crc_valid { 1 } else { 0 };

    data
}

/// Decode config status from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be at least 3 bytes)
///
/// # Returns
/// * `Some((version, crc_valid))` if parsing successful
/// * `None` if data length is incorrect
#[allow(dead_code)]
pub fn decode_config_status(data: &[u8]) -> Option<(u16, bool)> {
    if data.len() < 3 {
        return None;
    }

    let version_bytes = [data[0], data[1]];
    let version = u16::from_le_bytes(version_bytes);
    let crc_valid = data[2] != 0;

    Some((version, crc_valid))
}

// ============================================================================
// Motor Control Parameter Commands
// ============================================================================

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
        error!("Motor voltage params: invalid data length {}", data.len());
        return None;
    }

    let max_voltage_bytes = [data[0], data[1], data[2], data[3]];
    let v_dc_bus_bytes = [data[4], data[5], data[6], data[7]];

    let max_voltage = f32::from_le_bytes(max_voltage_bytes);
    let v_dc_bus = f32::from_le_bytes(v_dc_bus_bytes);

    info!(
        "Motor voltage params received: max_voltage={}, v_dc_bus={}",
        max_voltage, v_dc_bus
    );
    Some((max_voltage, v_dc_bus))
}

/// Encode motor voltage parameters into CAN data
pub fn encode_motor_voltage_params(max_voltage: f32, v_dc_bus: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&max_voltage.to_le_bytes());
    data[4..8].copy_from_slice(&v_dc_bus.to_le_bytes());
    data
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
        error!("Motor basic params: invalid data length {}", data.len());
        return None;
    }

    let pole_pairs = data[0];
    let max_duty_bytes = [data[1], data[2]];
    let max_duty = u16::from_le_bytes(max_duty_bytes);

    info!(
        "Motor basic params received: pole_pairs={}, max_duty={}",
        pole_pairs, max_duty
    );
    Some((pole_pairs, max_duty))
}

/// Encode motor basic parameters into CAN data
pub fn encode_motor_basic_params(pole_pairs: u8, max_duty: u16) -> [u8; 3] {
    let mut data = [0u8; 3];
    data[0] = pole_pairs;
    data[1..3].copy_from_slice(&max_duty.to_le_bytes());
    data
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
        error!("Hall sensor params: invalid data length {}", data.len());
        return None;
    }

    let alpha_bytes = [data[0], data[1], data[2], data[3]];
    let offset_bytes = [data[4], data[5], data[6], data[7]];

    let speed_filter_alpha = f32::from_le_bytes(alpha_bytes);
    let hall_angle_offset = f32::from_le_bytes(offset_bytes);

    info!(
        "Hall sensor params received: alpha={}, offset={}",
        speed_filter_alpha, hall_angle_offset
    );
    Some((speed_filter_alpha, hall_angle_offset))
}

/// Encode hall sensor parameters into CAN data
pub fn encode_hall_sensor_params(speed_filter_alpha: f32, hall_angle_offset: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&speed_filter_alpha.to_le_bytes());
    data[4..8].copy_from_slice(&hall_angle_offset.to_le_bytes());
    data
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
        error!("Angle interpolation: no data");
        return None;
    }

    let enable = data[0] != 0;
    info!("Angle interpolation received: {}", enable);
    Some(enable)
}

/// Encode angle interpolation setting into CAN data
pub fn encode_angle_interpolation(enable: bool) -> [u8; 1] {
    [if enable { 1 } else { 0 }]
}

// ============================================================================
// OpenLoop Parameter Commands
// ============================================================================

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
        error!("OpenLoop RPM params: invalid data length {}", data.len());
        return None;
    }

    let initial_bytes = [data[0], data[1], data[2], data[3]];
    let target_bytes = [data[4], data[5], data[6], data[7]];

    let initial_rpm = f32::from_le_bytes(initial_bytes);
    let target_rpm = f32::from_le_bytes(target_bytes);

    info!(
        "OpenLoop RPM params received: initial={}, target={}",
        initial_rpm, target_rpm
    );
    Some((initial_rpm, target_rpm))
}

/// Encode openloop RPM parameters into CAN data
pub fn encode_openloop_rpm_params(initial_rpm: f32, target_rpm: f32) -> [u8; 8] {
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&initial_rpm.to_le_bytes());
    data[4..8].copy_from_slice(&target_rpm.to_le_bytes());
    data
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
        error!(
            "OpenLoop accel/duty params: invalid data length {}",
            data.len()
        );
        return None;
    }

    let accel_bytes = [data[0], data[1], data[2], data[3]];
    let duty_bytes = [data[4], data[5]];

    let acceleration = f32::from_le_bytes(accel_bytes);
    let duty_ratio = u16::from_le_bytes(duty_bytes);

    info!(
        "OpenLoop accel/duty params received: accel={}, duty={}",
        acceleration, duty_ratio
    );
    Some((acceleration, duty_ratio))
}

/// Encode openloop acceleration/duty parameters into CAN data
pub fn encode_openloop_accel_duty_params(acceleration: f32, duty_ratio: u16) -> [u8; 6] {
    let mut data = [0u8; 6];
    data[0..4].copy_from_slice(&acceleration.to_le_bytes());
    data[4..6].copy_from_slice(&duty_ratio.to_le_bytes());
    data
}

// ============================================================================
// PWM Configuration Commands
// ============================================================================

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
        error!("PWM config: invalid data length {}", data.len());
        return None;
    }

    let freq_bytes = [data[0], data[1], data[2], data[3]];
    let dead_time_bytes = [data[4], data[5]];

    let frequency = u32::from_le_bytes(freq_bytes);
    let dead_time = u16::from_le_bytes(dead_time_bytes);

    info!(
        "PWM config received: freq={}Hz, dead_time={}",
        frequency, dead_time
    );
    Some((frequency, dead_time))
}

/// Encode PWM configuration into CAN data
pub fn encode_pwm_config(frequency: u32, dead_time: u16) -> [u8; 6] {
    let mut data = [0u8; 6];
    data[0..4].copy_from_slice(&frequency.to_le_bytes());
    data[4..6].copy_from_slice(&dead_time.to_le_bytes());
    data
}

// ============================================================================
// CAN Configuration Commands
// ============================================================================

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
        error!("CAN config: invalid data length {}", data.len());
        return None;
    }

    let bitrate_bytes = [data[0], data[1], data[2], data[3]];
    let bitrate = u32::from_le_bytes(bitrate_bytes);

    info!("CAN config received: bitrate={}", bitrate);
    Some(bitrate)
}

/// Encode CAN configuration into CAN data
pub fn encode_can_config(bitrate: u32) -> [u8; 4] {
    bitrate.to_le_bytes()
}

// ============================================================================
// Control Timing Commands
// ============================================================================

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
        error!("Control timing: invalid data length {}", data.len());
        return None;
    }

    let period_bytes = [
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
    ];
    let control_period_us = u64::from_le_bytes(period_bytes);

    info!("Control timing received: {}us", control_period_us);
    Some(control_period_us)
}

/// Encode control timing into CAN data
pub fn encode_control_timing(control_period_us: u64) -> [u8; 8] {
    control_period_us.to_le_bytes()
}

// ============================================================================
// Calibration Commands
// ============================================================================

/// Encode calibration status into CAN data
///
/// # Arguments
/// * `electrical_offset` - Electrical offset in radians (0～2π)
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

    // Encode electrical offset as little-endian f32 (bytes 0-3)
    let offset_bytes = electrical_offset.to_le_bytes();
    data[0..4].copy_from_slice(&offset_bytes);

    // Encode direction inversed flag (byte 4)
    data[4] = if direction_inversed { 1 } else { 0 };

    // Encode success flag (byte 5)
    data[5] = if success { 1 } else { 0 };

    data
}

/// Decode calibration status from CAN data
///
/// # Arguments
/// * `data` - CAN frame data (should be at least 6 bytes)
///
/// # Returns
/// * `Some((electrical_offset, direction_inversed, success))` if parsing successful
/// * `None` if data length is incorrect
#[allow(dead_code)]
pub fn decode_calibration_status(data: &[u8]) -> Option<(f32, bool, bool)> {
    if data.len() < 6 {
        return None;
    }

    let offset_bytes = [data[0], data[1], data[2], data[3]];
    let electrical_offset = f32::from_le_bytes(offset_bytes);

    let direction_inversed = data[4] != 0;
    let success = data[5] != 0;

    Some((electrical_offset, direction_inversed, success))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_speed_command() {
        let speed = 1234.5f32;
        let data = speed.to_le_bytes();
        let parsed = parse_speed_command(&data);
        assert_eq!(parsed, Some(speed));
    }

    #[test]
    fn test_parse_pi_gains() {
        let kp = 0.1f32;
        let ki = 0.01f32;
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&kp.to_le_bytes());
        data[4..8].copy_from_slice(&ki.to_le_bytes());

        let parsed = parse_pi_gains(&data);
        assert_eq!(parsed, Some((kp, ki)));
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
}
