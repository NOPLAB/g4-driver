//! SLCAN protocol encoder/decoder
//!
//! Implements the Serial Line CAN (SLCAN) protocol for communication
//! with USB-CAN adapters like CANUSB.
//!
//! ## Protocol Format
//!
//! Standard frame: `tiiildd...\r`
//! - `t` = standard frame command
//! - `iii` = 3-digit hex ID
//! - `l` = data length (0-8)
//! - `dd...` = data bytes in hex
//! - `\r` = carriage return terminator
//!
//! Extended frame: `Tiiiiiiiildd...\r`
//! - `T` = extended frame command
//! - `iiiiiiii` = 8-digit hex ID (29-bit)
//!
//! RTR frame: `riiil\r` or `Riiiiiiiil\r`

use super::frame::CanFrame;

/// SLCAN bitrate settings
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(dead_code)]
pub enum SlcanBitrate {
    /// 10 kbps
    B10K = 0,
    /// 20 kbps
    B20K = 1,
    /// 50 kbps
    B50K = 2,
    /// 100 kbps
    B100K = 3,
    /// 125 kbps
    B125K = 4,
    /// 250 kbps
    B250K = 5,
    /// 500 kbps
    B500K = 6,
    /// 800 kbps
    B800K = 7,
    /// 1000 kbps (1 Mbps)
    B1M = 8,
}

impl SlcanBitrate {
    /// Get the bitrate value in bps
    pub fn to_bps(self) -> u32 {
        match self {
            Self::B10K => 10_000,
            Self::B20K => 20_000,
            Self::B50K => 50_000,
            Self::B100K => 100_000,
            Self::B125K => 125_000,
            Self::B250K => 250_000,
            Self::B500K => 500_000,
            Self::B800K => 800_000,
            Self::B1M => 1_000_000,
        }
    }

    /// Create from bitrate value in bps
    #[allow(dead_code)]
    pub fn from_bps(bps: u32) -> Option<Self> {
        match bps {
            10_000 => Some(Self::B10K),
            20_000 => Some(Self::B20K),
            50_000 => Some(Self::B50K),
            100_000 => Some(Self::B100K),
            125_000 => Some(Self::B125K),
            250_000 => Some(Self::B250K),
            500_000 => Some(Self::B500K),
            800_000 => Some(Self::B800K),
            1_000_000 => Some(Self::B1M),
            _ => None,
        }
    }
}

/// Encode a CAN frame to SLCAN format
///
/// # Arguments
/// * `frame` - The CAN frame to encode
///
/// # Returns
/// SLCAN formatted string with carriage return terminator
pub fn encode_frame(frame: &CanFrame) -> String {
    let mut result = String::with_capacity(32);

    if frame.is_rtr() {
        // RTR frame
        if frame.is_extended() {
            result.push('R');
            result.push_str(&format!("{:08X}", frame.id()));
        } else {
            result.push('r');
            result.push_str(&format!("{:03X}", frame.id()));
        }
        result.push_str(&format!("{}", frame.len()));
    } else {
        // Data frame
        if frame.is_extended() {
            result.push('T');
            result.push_str(&format!("{:08X}", frame.id()));
        } else {
            result.push('t');
            result.push_str(&format!("{:03X}", frame.id()));
        }
        result.push_str(&format!("{}", frame.len()));

        // Append data bytes as hex
        for byte in frame.data() {
            result.push_str(&format!("{:02X}", byte));
        }
    }

    result.push('\r');
    result
}

/// Decode an SLCAN frame from a string
///
/// # Arguments
/// * `line` - SLCAN formatted string (with or without terminator)
///
/// # Returns
/// * `Some(CanFrame)` if successfully decoded
/// * `None` if invalid format
pub fn decode_frame(line: &str) -> Option<CanFrame> {
    let line = line.trim_end_matches('\r').trim_end_matches('\n');

    if line.is_empty() {
        return None;
    }

    let chars: Vec<char> = line.chars().collect();
    let cmd = chars[0];

    match cmd {
        't' => decode_standard_frame(&chars[1..]),
        'T' => decode_extended_frame(&chars[1..]),
        'r' => decode_standard_rtr(&chars[1..]),
        'R' => decode_extended_rtr(&chars[1..]),
        _ => None,
    }
}

/// Decode a standard data frame
fn decode_standard_frame(chars: &[char]) -> Option<CanFrame> {
    // Format: iiildd... (min 4 chars: 3 for ID + 1 for length)
    if chars.len() < 4 {
        return None;
    }

    // Parse 3-digit hex ID
    let id_str: String = chars[0..3].iter().collect();
    let id = u32::from_str_radix(&id_str, 16).ok()?;

    // Parse length
    let len = chars[3].to_digit(10)? as usize;
    if len > 8 {
        return None;
    }

    // Parse data bytes
    let data_chars = &chars[4..];
    if data_chars.len() < len * 2 {
        return None;
    }

    let mut data = Vec::with_capacity(len);
    for i in 0..len {
        let byte_str: String = data_chars[i * 2..i * 2 + 2].iter().collect();
        let byte = u8::from_str_radix(&byte_str, 16).ok()?;
        data.push(byte);
    }

    CanFrame::new(id, &data)
}

/// Decode an extended data frame
fn decode_extended_frame(chars: &[char]) -> Option<CanFrame> {
    // Format: iiiiiiiildd... (min 9 chars: 8 for ID + 1 for length)
    if chars.len() < 9 {
        return None;
    }

    // Parse 8-digit hex ID
    let id_str: String = chars[0..8].iter().collect();
    let id = u32::from_str_radix(&id_str, 16).ok()?;

    // Parse length
    let len = chars[8].to_digit(10)? as usize;
    if len > 8 {
        return None;
    }

    // Parse data bytes
    let data_chars = &chars[9..];
    if data_chars.len() < len * 2 {
        return None;
    }

    let mut data = Vec::with_capacity(len);
    for i in 0..len {
        let byte_str: String = data_chars[i * 2..i * 2 + 2].iter().collect();
        let byte = u8::from_str_radix(&byte_str, 16).ok()?;
        data.push(byte);
    }

    CanFrame::new_extended(id, &data)
}

/// Decode a standard RTR frame
fn decode_standard_rtr(chars: &[char]) -> Option<CanFrame> {
    // Format: iiil (4 chars: 3 for ID + 1 for DLC)
    if chars.len() < 4 {
        return None;
    }

    let id_str: String = chars[0..3].iter().collect();
    let id = u32::from_str_radix(&id_str, 16).ok()?;

    let dlc = chars[3].to_digit(10)? as usize;
    if dlc > 8 {
        return None;
    }

    CanFrame::new_rtr(id, dlc)
}

/// Decode an extended RTR frame
fn decode_extended_rtr(chars: &[char]) -> Option<CanFrame> {
    // Format: iiiiiiiil (9 chars: 8 for ID + 1 for DLC)
    if chars.len() < 9 {
        return None;
    }

    let id_str: String = chars[0..8].iter().collect();
    let id = u32::from_str_radix(&id_str, 16).ok()?;

    let dlc = chars[8].to_digit(10)? as usize;
    if dlc > 8 {
        return None;
    }

    // Extended RTR is not directly supported by our CanFrame,
    // we'd need to add support for it
    // For now, return None for extended RTR
    let _ = id;
    let _ = dlc;
    None
}

/// Generate SLCAN initialization sequence
///
/// # Arguments
/// * `bitrate` - CAN bitrate setting
///
/// # Returns
/// Vector of commands to send for initialization
pub fn init_sequence(bitrate: SlcanBitrate) -> Vec<String> {
    vec![
        "C\r".to_string(),               // Close any existing connection
        format!("S{}\r", bitrate as u8), // Set bitrate
        "O\r".to_string(),               // Open channel
    ]
}

/// Generate SLCAN close sequence
pub fn close_sequence() -> Vec<String> {
    vec!["C\r".to_string()]
}

/// Check if a response indicates success
#[allow(dead_code)]
pub fn is_ok_response(response: &str) -> bool {
    response.trim() == "\r" || response.contains('\r')
}

/// Check if a response indicates error
pub fn is_error_response(response: &str) -> bool {
    response.trim().starts_with('\x07') || response.contains('\x07')
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_standard_frame() {
        let frame = CanFrame::new(0x100, &[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();
        let encoded = encode_frame(&frame);
        assert_eq!(encoded, "t1004DEADBEEF\r");
    }

    #[test]
    fn test_encode_empty_frame() {
        let frame = CanFrame::new(0x000, &[]).unwrap();
        let encoded = encode_frame(&frame);
        assert_eq!(encoded, "t0000\r");
    }

    #[test]
    fn test_encode_extended_frame() {
        let frame = CanFrame::new_extended(0x12345678, &[0x01, 0x02]).unwrap();
        let encoded = encode_frame(&frame);
        assert_eq!(encoded, "T1234567820102\r");
    }

    #[test]
    fn test_decode_standard_frame() {
        let frame = decode_frame("t1004DEADBEEF\r").unwrap();
        assert_eq!(frame.id(), 0x100);
        assert_eq!(frame.data(), &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(frame.len(), 4);
        assert!(!frame.is_extended());
    }

    #[test]
    fn test_decode_extended_frame() {
        let frame = decode_frame("T1234567820102\r").unwrap();
        assert_eq!(frame.id(), 0x12345678);
        assert!(frame.is_extended());
        assert_eq!(frame.data(), &[0x01, 0x02]);
    }

    #[test]
    fn test_decode_empty_frame() {
        let frame = decode_frame("t1000\r").unwrap();
        assert_eq!(frame.id(), 0x100);
        assert!(frame.is_empty());
    }

    #[test]
    fn test_decode_invalid() {
        assert!(decode_frame("").is_none());
        assert!(decode_frame("x1234").is_none());
        assert!(decode_frame("t10").is_none()); // Too short
    }

    #[test]
    fn test_roundtrip() {
        let original =
            CanFrame::new(0x200, &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]).unwrap();
        let encoded = encode_frame(&original);
        let decoded = decode_frame(&encoded).unwrap();
        assert_eq!(original, decoded);
    }

    #[test]
    fn test_bitrate_conversion() {
        assert_eq!(SlcanBitrate::B250K.to_bps(), 250_000);
        assert_eq!(SlcanBitrate::from_bps(250_000), Some(SlcanBitrate::B250K));
        assert_eq!(SlcanBitrate::from_bps(123_456), None);
    }

    #[test]
    fn test_init_sequence() {
        let seq = init_sequence(SlcanBitrate::B250K);
        assert_eq!(seq.len(), 3);
        assert_eq!(seq[0], "C\r");
        assert_eq!(seq[1], "S5\r");
        assert_eq!(seq[2], "O\r");
    }
}
