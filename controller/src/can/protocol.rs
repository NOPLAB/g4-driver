// CAN communication protocol definitions for motor control
use tracing::{error, info};

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

/// Calibration status structure
#[derive(Debug, Clone, Copy)]
pub struct CalibrationStatus {
    pub electrical_offset: f32,
    pub direction_inversed: bool,
    pub success: bool,
}

impl CalibrationStatus {
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

// ============================================================================
// Motor Control Parameter Commands (Encode functions)
// ============================================================================

/// Encode motor voltage parameters into CAN data
pub fn encode_motor_voltage_params(max_voltage: f32, v_dc_bus: f32) -> Vec<u8> {
    let mut data = Vec::with_capacity(8);
    data.extend_from_slice(&max_voltage.to_le_bytes());
    data.extend_from_slice(&v_dc_bus.to_le_bytes());
    data
}

/// Encode motor basic parameters into CAN data
pub fn encode_motor_basic_params(pole_pairs: u8, max_duty: u16) -> Vec<u8> {
    let mut data = Vec::with_capacity(3);
    data.push(pole_pairs);
    data.extend_from_slice(&max_duty.to_le_bytes());
    data
}

/// Encode hall sensor parameters into CAN data
pub fn encode_hall_sensor_params(speed_filter_alpha: f32, hall_angle_offset: f32) -> Vec<u8> {
    let mut data = Vec::with_capacity(8);
    data.extend_from_slice(&speed_filter_alpha.to_le_bytes());
    data.extend_from_slice(&hall_angle_offset.to_le_bytes());
    data
}

/// Encode angle interpolation setting into CAN data
pub fn encode_angle_interpolation(enable: bool) -> Vec<u8> {
    vec![if enable { 1 } else { 0 }]
}

// ============================================================================
// OpenLoop Parameter Commands (Encode functions)
// ============================================================================

/// Encode openloop RPM parameters into CAN data
pub fn encode_openloop_rpm_params(initial_rpm: f32, target_rpm: f32) -> Vec<u8> {
    let mut data = Vec::with_capacity(8);
    data.extend_from_slice(&initial_rpm.to_le_bytes());
    data.extend_from_slice(&target_rpm.to_le_bytes());
    data
}

/// Encode openloop acceleration/duty parameters into CAN data
pub fn encode_openloop_accel_duty_params(acceleration: f32, duty_ratio: u16) -> Vec<u8> {
    let mut data = Vec::with_capacity(6);
    data.extend_from_slice(&acceleration.to_le_bytes());
    data.extend_from_slice(&duty_ratio.to_le_bytes());
    data
}

// ============================================================================
// PWM Configuration Commands (Encode functions)
// ============================================================================

/// Encode PWM configuration into CAN data
pub fn encode_pwm_config(frequency: u32, dead_time: u16) -> Vec<u8> {
    let mut data = Vec::with_capacity(6);
    data.extend_from_slice(&frequency.to_le_bytes());
    data.extend_from_slice(&dead_time.to_le_bytes());
    data
}

// ============================================================================
// CAN Configuration Commands (Encode functions)
// ============================================================================

/// Encode CAN configuration into CAN data
pub fn encode_can_config(bitrate: u32) -> Vec<u8> {
    bitrate.to_le_bytes().to_vec()
}

// ============================================================================
// Control Timing Commands (Encode functions)
// ============================================================================

/// Encode control timing into CAN data
pub fn encode_control_timing(control_period_us: u64) -> Vec<u8> {
    control_period_us.to_le_bytes().to_vec()
}

// ============================================================================
// Calibration Commands (Encode functions)
// ============================================================================

/// Encode start calibration command into CAN data
///
/// # Arguments
/// * `torque` - Optional torque value (0-100). If None, sends empty data.
///
/// # Returns
/// Vec<u8> containing encoded calibration command (0 or 1 byte)
pub fn encode_start_calibration(torque: Option<u8>) -> Vec<u8> {
    if let Some(t) = torque {
        vec![t.min(100)]
    } else {
        vec![]
    }
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
}
