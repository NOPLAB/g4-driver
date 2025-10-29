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

    /// Motor status feedback (speed: f32, angle: f32, 8 bytes)
    pub const STATUS: u32 = 0x200;

    /// Voltage status feedback (voltage: f32, flags: u8, 5 bytes)
    pub const VOLTAGE_STATUS: u32 = 0x201;

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
/// * `Some((voltage, overvoltage, undervoltage))` if parsing successful
/// * `None` if data length is incorrect
pub fn decode_voltage_status(data: &[u8]) -> Option<(f32, bool, bool)> {
    if data.len() < 5 {
        return None;
    }

    let voltage_bytes = [data[0], data[1], data[2], data[3]];
    let voltage = f32::from_le_bytes(voltage_bytes);

    let flags = data[4];
    let overvoltage = (flags & 0x01) != 0;
    let undervoltage = (flags & 0x02) != 0;

    Some((voltage, overvoltage, undervoltage))
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
}
