//! Serial port detection utilities
//!
//! Provides cross-platform serial port detection for USB-CAN adapters.

use tracing::{debug, info};

/// Information about a detected serial port
#[derive(Debug, Clone)]
pub struct SerialPortInfo {
    /// Port path (e.g., "/dev/ttyACM0", "COM3")
    pub port_name: String,
    /// Human-readable description
    pub description: String,
    /// Whether this is a USB serial device
    pub is_usb: bool,
    /// USB Vendor ID (if available)
    pub vid: Option<u16>,
    /// USB Product ID (if available)
    pub pid: Option<u16>,
}

impl SerialPortInfo {
    /// Create a new SerialPortInfo
    pub fn new(port_name: String, description: String) -> Self {
        Self {
            port_name,
            description,
            is_usb: false,
            vid: None,
            pid: None,
        }
    }

    /// Check if this might be a CAN adapter based on common VID/PIDs
    #[allow(dead_code)]
    pub fn is_likely_can_adapter(&self) -> bool {
        match (self.vid, self.pid) {
            // FTDI chips (common in CANUSB and similar)
            (Some(0x0403), _) => true,
            // Microchip/MCHP
            (Some(0x04D8), _) => true,
            // STMicroelectronics
            (Some(0x0483), _) => true,
            // Canable/Candlelight
            (Some(0x1D50), Some(0x606F)) => true,
            // Generic CDC ACM devices might be CAN adapters
            _ => self.port_name.contains("ACM") || self.port_name.contains("USB"),
        }
    }
}

/// Detect available serial ports
///
/// Uses tokio-serial's port enumeration on all platforms.
pub fn detect_serial_ports() -> Vec<SerialPortInfo> {
    let mut ports = Vec::new();

    match tokio_serial::available_ports() {
        Ok(available_ports) => {
            for port in available_ports {
                let mut info = SerialPortInfo::new(port.port_name.clone(), String::new());

                // Extract USB info if available
                match &port.port_type {
                    tokio_serial::SerialPortType::UsbPort(usb_info) => {
                        info.is_usb = true;
                        info.vid = Some(usb_info.vid);
                        info.pid = Some(usb_info.pid);

                        // Build description
                        let manufacturer = usb_info.manufacturer.as_deref().unwrap_or("Unknown");
                        let product = usb_info.product.as_deref().unwrap_or("USB Serial");

                        info.description = format!(
                            "{} - {} (VID:{:04X} PID:{:04X})",
                            manufacturer, product, usb_info.vid, usb_info.pid
                        );
                    }
                    tokio_serial::SerialPortType::PciPort => {
                        info.description = "PCI Serial Port".to_string();
                    }
                    tokio_serial::SerialPortType::BluetoothPort => {
                        info.description = "Bluetooth Serial".to_string();
                    }
                    tokio_serial::SerialPortType::Unknown => {
                        info.description = "Serial Port".to_string();
                    }
                }

                debug!(
                    "Found serial port: {} - {}",
                    info.port_name, info.description
                );
                ports.push(info);
            }
        }
        Err(e) => {
            info!("Failed to enumerate serial ports: {}", e);
        }
    }

    // Sort by port name
    ports.sort_by(|a, b| a.port_name.cmp(&b.port_name));

    info!("Detected {} serial ports", ports.len());
    ports
}

/// Filter ports to only show likely CAN adapters
#[allow(dead_code)]
pub fn filter_can_adapters(ports: Vec<SerialPortInfo>) -> Vec<SerialPortInfo> {
    ports
        .into_iter()
        .filter(|p| p.is_usb && p.is_likely_can_adapter())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_serial_port_info() {
        let info = SerialPortInfo {
            port_name: "/dev/ttyACM0".to_string(),
            description: "Test Device".to_string(),
            is_usb: true,
            vid: Some(0x0403),
            pid: Some(0x6001),
        };

        assert!(info.is_likely_can_adapter());
    }

    #[test]
    fn test_detect_ports() {
        // This test just checks that the function doesn't panic
        let ports = detect_serial_ports();
        println!("Found {} ports", ports.len());
        for port in &ports {
            println!("  - {}: {}", port.port_name, port.description);
        }
    }
}
