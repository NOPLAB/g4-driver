// CAN interface setup and detection utilities
// Provides functionality similar to scripts/can.sh for CANUSB adapters

use anyhow::{Context, Result};
use std::path::Path;
use std::process::Command;
use tracing::{debug, error, info, warn};

/// USB serial device for slcan
#[derive(Debug, Clone)]
pub struct UsbCanDevice {
    pub device_path: String,
    pub description: String,
}

/// CAN interface information
#[derive(Debug, Clone)]
pub struct CanInterface {
    pub name: String,
    pub is_up: bool,
    pub interface_type: InterfaceType,
}

#[derive(Debug, Clone, PartialEq)]
pub enum InterfaceType {
    Hardware,  // Hardware CAN (can0, can1)
    Virtual,   // Virtual CAN (vcan0, vcan1)
    Slcan,     // Serial line CAN (slcan0, slcan1)
}

/// Detect available USB-CAN adapters (serial devices)
///
/// Scans /dev/ttyACM* and /dev/ttyUSB* for potential CANUSB adapters
pub fn detect_usb_can_devices() -> Vec<UsbCanDevice> {
    let mut devices = Vec::new();

    // Check /dev/ttyACM* devices
    for i in 0..10 {
        let path = format!("/dev/ttyACM{}", i);
        if Path::new(&path).exists() {
            devices.push(UsbCanDevice {
                device_path: path.clone(),
                description: format!("USB CAN Adapter (ACM{})", i),
            });
            debug!("Found USB device: {}", path);
        }
    }

    // Check /dev/ttyUSB* devices
    for i in 0..10 {
        let path = format!("/dev/ttyUSB{}", i);
        if Path::new(&path).exists() {
            devices.push(UsbCanDevice {
                device_path: path.clone(),
                description: format!("USB CAN Adapter (USB{})", i),
            });
            debug!("Found USB device: {}", path);
        }
    }

    info!("Detected {} USB-CAN devices", devices.len());
    devices
}

/// Detect available CAN interfaces on the system
///
/// Uses `ip link show` to list all network interfaces and filters for CAN types
pub fn detect_can_interfaces() -> Result<Vec<CanInterface>> {
    let output = Command::new("ip")
        .args(["link", "show"])
        .output()
        .context("Failed to execute 'ip link show'")?;

    if !output.status.success() {
        return Err(anyhow::anyhow!(
            "ip link show failed: {}",
            String::from_utf8_lossy(&output.stderr)
        ));
    }

    let output_str = String::from_utf8_lossy(&output.stdout);
    let mut interfaces = Vec::new();

    // Parse output line by line
    // Format: "3: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP"
    for line in output_str.lines() {
        // Look for lines that start with a number (interface definition)
        if let Some(colon_pos) = line.find(':') {
            let after_number = &line[colon_pos + 1..];
            if let Some(second_colon) = after_number.find(':') {
                let name = after_number[..second_colon].trim();

                // Filter for CAN interfaces
                if name.starts_with("can")
                    || name.starts_with("vcan")
                    || name.starts_with("slcan")
                {
                    let is_up = line.contains("UP");
                    let interface_type = if name.starts_with("vcan") {
                        InterfaceType::Virtual
                    } else if name.starts_with("slcan") {
                        InterfaceType::Slcan
                    } else {
                        InterfaceType::Hardware
                    };

                    interfaces.push(CanInterface {
                        name: name.to_string(),
                        is_up,
                        interface_type,
                    });

                    debug!("Found CAN interface: {} (UP: {})", name, is_up);
                }
            }
        }
    }

    info!("Detected {} CAN interfaces", interfaces.len());
    Ok(interfaces)
}

/// Setup slcan interface from USB serial device
///
/// # Arguments
/// * `device_path` - Path to USB serial device (e.g., "/dev/ttyACM0")
/// * `interface_name` - Desired slcan interface name (e.g., "slcan0")
/// * `bitrate` - CAN bitrate (250000 for 250kbps)
///
/// This function:
/// 1. Runs slcand to create the slcan interface
/// 2. Brings up the interface with ip link set up
pub fn setup_slcan_interface(
    device_path: &str,
    interface_name: &str,
    bitrate: u32,
) -> Result<()> {
    info!(
        "Setting up slcan interface {} from device {} at {} bps",
        interface_name, device_path, bitrate
    );

    // Check if device exists
    if !Path::new(device_path).exists() {
        return Err(anyhow::anyhow!("Device {} does not exist", device_path));
    }

    // Check if interface already exists
    if let Ok(interfaces) = detect_can_interfaces() {
        if interfaces.iter().any(|i| i.name == interface_name) {
            warn!("Interface {} already exists, cleaning up first", interface_name);
            // Try to clean up existing interface
            let _ = cleanup_slcan_interface(interface_name);
        }
    }

    // Determine slcand speed parameter
    // S0=10kbps, S1=20kbps, S2=50kbps, S3=100kbps, S4=125kbps, S5=250kbps, S6=500kbps, S7=800kbps, S8=1Mbps
    let speed_param = match bitrate {
        10000 => "0",
        20000 => "1",
        50000 => "2",
        100000 => "3",
        125000 => "4",
        250000 => "5",
        500000 => "6",
        800000 => "7",
        1000000 => "8",
        _ => return Err(anyhow::anyhow!("Unsupported bitrate: {}", bitrate)),
    };

    // Run slcand command
    // -o: open device
    // -c: close device on exit
    // -s: speed parameter
    // -S: baudrate (default 115200)
    info!("Running: slcand -o -c -s{} {} {}", speed_param, device_path, interface_name);

    let output = Command::new("sudo")
        .args([
            "slcand",
            "-o",
            "-c",
            &format!("-s{}", speed_param),
            device_path,
            interface_name,
        ])
        .output()
        .context("Failed to execute slcand command")?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        error!("slcand failed: {}", stderr);
        return Err(anyhow::anyhow!("slcand failed: {}", stderr));
    }

    info!("slcand executed successfully");

    // Wait a bit for interface to be created
    std::thread::sleep(std::time::Duration::from_millis(500));

    // Bring up the interface
    bring_up_interface(interface_name)?;

    info!("slcan interface {} is ready", interface_name);
    Ok(())
}

/// Bring up a CAN interface
///
/// # Arguments
/// * `interface_name` - CAN interface name (e.g., "slcan0", "can0")
pub fn bring_up_interface(interface_name: &str) -> Result<()> {
    info!("Bringing up interface {}", interface_name);

    let output = Command::new("sudo")
        .args(["ip", "link", "set", interface_name, "up"])
        .output()
        .context("Failed to execute 'ip link set up'")?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        error!("Failed to bring up interface: {}", stderr);
        return Err(anyhow::anyhow!("Failed to bring up interface: {}", stderr));
    }

    info!("Interface {} is now UP", interface_name);
    Ok(())
}

/// Bring down a CAN interface
///
/// # Arguments
/// * `interface_name` - CAN interface name
pub fn bring_down_interface(interface_name: &str) -> Result<()> {
    info!("Bringing down interface {}", interface_name);

    let output = Command::new("sudo")
        .args(["ip", "link", "set", interface_name, "down"])
        .output()
        .context("Failed to execute 'ip link set down'")?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        warn!("Failed to bring down interface: {}", stderr);
        // Don't return error, interface might not exist
    }

    Ok(())
}

/// Cleanup slcan interface
///
/// # Arguments
/// * `interface_name` - slcan interface name (e.g., "slcan0")
///
/// This function:
/// 1. Brings down the interface
/// 2. Kills the slcand daemon
pub fn cleanup_slcan_interface(interface_name: &str) -> Result<()> {
    info!("Cleaning up slcan interface {}", interface_name);

    // Bring down interface
    let _ = bring_down_interface(interface_name);

    // Kill slcand process
    // Note: This kills ALL slcand processes, not just for this interface
    // A more sophisticated approach would parse ps output to find the specific PID
    let output = Command::new("sudo")
        .args(["pkill", "slcand"])
        .output()
        .context("Failed to execute pkill")?;

    if !output.status.success() {
        debug!("pkill slcand returned non-zero (might not be running)");
    }

    info!("Cleaned up slcan interface");
    Ok(())
}

/// Check if an interface is up
pub fn is_interface_up(interface_name: &str) -> Result<bool> {
    let interfaces = detect_can_interfaces()?;
    Ok(interfaces
        .iter()
        .find(|i| i.name == interface_name)
        .map(|i| i.is_up)
        .unwrap_or(false))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_usb_devices() {
        // This test just checks that the function doesn't panic
        let devices = detect_usb_can_devices();
        println!("Found {} USB devices", devices.len());
    }

    #[test]
    fn test_detect_can_interfaces() {
        // This test requires 'ip' command to be available
        match detect_can_interfaces() {
            Ok(interfaces) => {
                println!("Found {} CAN interfaces", interfaces.len());
                for iface in interfaces {
                    println!("  - {} (up: {})", iface.name, iface.is_up);
                }
            }
            Err(e) => {
                eprintln!("Error detecting interfaces: {}", e);
            }
        }
    }
}
