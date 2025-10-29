use anyhow::{Context, Result};
use futures::StreamExt;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{timeout, Duration};
use tokio_socketcan::{CANSocket, CANFrame};
use tracing::{debug, info};

use super::protocol::{self, can_ids, MotorStatus, VoltageStatus};

/// CAN Manager for handling CAN communication
pub struct CanManager {
    socket: Arc<Mutex<Option<CANSocket>>>,
    interface_name: String,
}

impl CanManager {
    /// Create a new CAN manager
    pub fn new() -> Self {
        Self {
            socket: Arc::new(Mutex::new(None)),
            interface_name: String::new(),
        }
    }

    /// Connect to CAN interface
    ///
    /// # Arguments
    /// * `interface` - CAN interface name (e.g., "can0", "vcan0")
    pub async fn connect(&mut self, interface: &str) -> Result<()> {
        info!("Connecting to CAN interface: {}", interface);

        let socket = CANSocket::open(interface)
            .with_context(|| format!("Failed to open CAN interface: {}", interface))?;

        *self.socket.lock().await = Some(socket);
        self.interface_name = interface.to_string();

        info!("Successfully connected to {}", interface);
        Ok(())
    }

    /// Disconnect from CAN interface
    pub async fn disconnect(&mut self) {
        info!("Disconnecting from CAN interface");
        *self.socket.lock().await = None;
        self.interface_name.clear();
    }

    /// Check if connected
    pub async fn is_connected(&self) -> bool {
        self.socket.lock().await.is_some()
    }

    /// Get current interface name
    pub fn interface_name(&self) -> &str {
        &self.interface_name
    }

    /// Send speed command
    ///
    /// # Arguments
    /// * `speed_rpm` - Target speed in RPM
    pub async fn send_speed_command(&self, speed_rpm: f32) -> Result<()> {
        let data = protocol::encode_speed_command(speed_rpm);
        self.send_frame(can_ids::SPEED_CMD, &data).await
    }

    /// Send PI gains
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    pub async fn send_pi_gains(&self, kp: f32, ki: f32) -> Result<()> {
        let data = protocol::encode_pi_gains(kp, ki);
        self.send_frame(can_ids::PI_GAINS, &data).await
    }

    /// Send motor enable command
    ///
    /// # Arguments
    /// * `enable` - Motor enable flag
    pub async fn send_enable_command(&self, enable: bool) -> Result<()> {
        let data = protocol::encode_enable_command(enable);
        self.send_frame(can_ids::ENABLE_CMD, &data).await
    }

    /// Send emergency stop command
    pub async fn send_emergency_stop(&self) -> Result<()> {
        info!("Sending emergency stop");
        self.send_frame(can_ids::EMERGENCY_STOP, &[]).await
    }

    /// Receive next CAN frame with timeout
    ///
    /// # Arguments
    /// * `timeout_ms` - Timeout in milliseconds
    ///
    /// # Returns
    /// * `Ok(Some(frame))` if frame received
    /// * `Ok(None)` if timeout occurred
    /// * `Err` if receive error
    pub async fn receive_frame(&self, timeout_ms: u64) -> Result<Option<CANFrame>> {
        let mut socket_guard = self.socket.lock().await;
        if let Some(socket) = socket_guard.as_mut() {
            match timeout(Duration::from_millis(timeout_ms), socket.next()).await {
                Ok(Some(Ok(frame))) => Ok(Some(frame)),
                Ok(Some(Err(e))) => Err(anyhow::anyhow!("CAN receive error: {}", e)),
                Ok(None) => Err(anyhow::anyhow!("CAN socket closed")),
                Err(_) => Ok(None), // Timeout
            }
        } else {
            Err(anyhow::anyhow!("Not connected to CAN interface"))
        }
    }

    /// Parse motor status from CAN frame
    pub fn parse_motor_status(frame: &CANFrame) -> Option<MotorStatus> {
        if frame.id() == can_ids::STATUS {
            protocol::decode_status(frame.data())
        } else {
            None
        }
    }

    /// Parse voltage status from CAN frame
    pub fn parse_voltage_status(frame: &CANFrame) -> Option<VoltageStatus> {
        if frame.id() == can_ids::VOLTAGE_STATUS {
            protocol::decode_voltage_status(frame.data())
        } else {
            None
        }
    }

    /// Send a CAN frame
    ///
    /// # Arguments
    /// * `id` - CAN message ID
    /// * `data` - CAN frame data
    async fn send_frame(&self, id: u32, data: &[u8]) -> Result<()> {
        let socket_guard = self.socket.lock().await;
        if let Some(socket) = socket_guard.as_ref() {
            let frame = CANFrame::new(id, data, false, false)
                .with_context(|| format!("Failed to create CAN frame with ID 0x{:X}", id))?;

            debug!("Sending CAN frame: ID=0x{:X}, len={}", id, data.len());

            socket
                .write_frame(frame)?
                .await
                .with_context(|| format!("Failed to send CAN frame with ID 0x{:X}", id))?;

            Ok(())
        } else {
            Err(anyhow::anyhow!("Not connected to CAN interface"))
        }
    }
}

impl Default for CanManager {
    fn default() -> Self {
        Self::new()
    }
}
