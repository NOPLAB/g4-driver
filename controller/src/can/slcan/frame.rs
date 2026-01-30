//! Platform-independent CAN frame structure
//!
//! This module provides a CAN frame structure that works across all platforms,
//! replacing the Linux-specific tokio-socketcan CANFrame.

/// CAN frame structure (platform-independent)
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CanFrame {
    /// CAN message ID (11-bit standard or 29-bit extended)
    id: u32,
    /// Frame data (up to 8 bytes)
    data: [u8; 8],
    /// Data length (0-8)
    len: usize,
    /// Extended frame flag (29-bit ID)
    extended: bool,
    /// Remote Transmission Request flag
    rtr: bool,
}

impl CanFrame {
    /// Create a new standard CAN frame
    ///
    /// # Arguments
    /// * `id` - CAN ID (11-bit, 0x000-0x7FF)
    /// * `data` - Frame data slice (up to 8 bytes)
    ///
    /// # Returns
    /// * `Some(CanFrame)` if valid
    /// * `None` if ID > 0x7FF or data > 8 bytes
    pub fn new(id: u32, data: &[u8]) -> Option<Self> {
        if id > 0x7FF || data.len() > 8 {
            return None;
        }

        let mut frame_data = [0u8; 8];
        frame_data[..data.len()].copy_from_slice(data);

        Some(Self {
            id,
            data: frame_data,
            len: data.len(),
            extended: false,
            rtr: false,
        })
    }

    /// Create a new extended CAN frame (29-bit ID)
    ///
    /// # Arguments
    /// * `id` - Extended CAN ID (29-bit, 0x00000000-0x1FFFFFFF)
    /// * `data` - Frame data slice (up to 8 bytes)
    ///
    /// # Returns
    /// * `Some(CanFrame)` if valid
    /// * `None` if ID > 0x1FFFFFFF or data > 8 bytes
    pub fn new_extended(id: u32, data: &[u8]) -> Option<Self> {
        if id > 0x1FFFFFFF || data.len() > 8 {
            return None;
        }

        let mut frame_data = [0u8; 8];
        frame_data[..data.len()].copy_from_slice(data);

        Some(Self {
            id,
            data: frame_data,
            len: data.len(),
            extended: true,
            rtr: false,
        })
    }

    /// Create a Remote Transmission Request frame
    pub fn new_rtr(id: u32, dlc: usize) -> Option<Self> {
        if id > 0x7FF || dlc > 8 {
            return None;
        }

        Some(Self {
            id,
            data: [0u8; 8],
            len: dlc,
            extended: false,
            rtr: true,
        })
    }

    /// Get the CAN ID
    #[inline]
    pub fn id(&self) -> u32 {
        self.id
    }

    /// Get the data slice
    #[inline]
    pub fn data(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// Get the data length
    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if frame has no data
    #[inline]
    #[allow(dead_code)]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Check if this is an extended frame (29-bit ID)
    #[inline]
    pub fn is_extended(&self) -> bool {
        self.extended
    }

    /// Check if this is a Remote Transmission Request
    #[inline]
    pub fn is_rtr(&self) -> bool {
        self.rtr
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_standard_frame() {
        let frame = CanFrame::new(0x100, &[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();
        assert_eq!(frame.id(), 0x100);
        assert_eq!(frame.data(), &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(frame.len(), 4);
        assert!(!frame.is_extended());
        assert!(!frame.is_rtr());
    }

    #[test]
    fn test_new_extended_frame() {
        let frame = CanFrame::new_extended(0x12345678, &[0x01, 0x02]).unwrap();
        assert_eq!(frame.id(), 0x12345678);
        assert!(frame.is_extended());
    }

    #[test]
    fn test_invalid_standard_id() {
        assert!(CanFrame::new(0x800, &[]).is_none());
    }

    #[test]
    fn test_invalid_data_length() {
        assert!(CanFrame::new(0x100, &[0; 9]).is_none());
    }

    #[test]
    fn test_empty_frame() {
        let frame = CanFrame::new(0x100, &[]).unwrap();
        assert!(frame.is_empty());
        assert_eq!(frame.len(), 0);
    }
}
