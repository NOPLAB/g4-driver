//! CAN communication manager using SLCAN
//!
//! This module provides CAN communication through USB-CAN adapters
//! using the SLCAN protocol for cross-platform compatibility.

use anyhow::{Context, Result};
use std::sync::Arc;
use tokio::sync::Mutex;
use tracing::{debug, info};

use g4_driver_protocol::{
    self as protocol, can_ids, CalibrationStatus, MotorStatus, VoltageStatus,
};

use super::commands::MotorCommand;
use super::slcan::{CanFrame, SlcanBitrate, SlcanStream};

/// CAN Manager for handling CAN communication via SLCAN
pub struct CanManager {
    stream: Arc<Mutex<Option<SlcanStream>>>,
    port_path: String,
}

impl CanManager {
    /// Create a new CAN manager
    pub fn new() -> Self {
        Self {
            stream: Arc::new(Mutex::new(None)),
            port_path: String::new(),
        }
    }

    /// Connect to a serial port with SLCAN adapter
    ///
    /// # Arguments
    /// * `port_path` - Serial port path (e.g., "/dev/ttyACM0", "COM3")
    /// * `bitrate` - CAN bus bitrate (default: 250kbps)
    pub async fn connect(&mut self, port_path: &str) -> Result<()> {
        self.connect_with_bitrate(port_path, SlcanBitrate::B250K)
            .await
    }

    /// Connect to a serial port with specified CAN bitrate
    ///
    /// # Arguments
    /// * `port_path` - Serial port path
    /// * `bitrate` - CAN bus bitrate
    pub async fn connect_with_bitrate(
        &mut self,
        port_path: &str,
        bitrate: SlcanBitrate,
    ) -> Result<()> {
        info!(
            "Connecting to SLCAN adapter: {} at {} bps",
            port_path,
            bitrate.to_bps()
        );

        let stream = SlcanStream::connect(port_path, bitrate)
            .await
            .with_context(|| format!("Failed to connect to SLCAN adapter: {}", port_path))?;

        *self.stream.lock().await = Some(stream);
        self.port_path = port_path.to_string();

        info!("Successfully connected to {}", port_path);
        Ok(())
    }

    /// Disconnect from CAN interface
    pub async fn disconnect(&mut self) {
        info!("Disconnecting from SLCAN adapter");

        if let Some(mut stream) = self.stream.lock().await.take() {
            let _ = stream.disconnect().await;
        }
        self.port_path.clear();
    }

    /// Check if connected
    #[allow(dead_code)]
    pub async fn is_connected(&self) -> bool {
        self.stream.lock().await.is_some()
    }

    /// Get current port path
    #[allow(dead_code)]
    pub fn port_path(&self) -> &str {
        &self.port_path
    }

    // ========================================================================
    // Unified API
    // ========================================================================

    /// Send a motor command using the unified API
    ///
    /// # Example
    /// ```ignore
    /// mgr.send(MotorCommand::Speed(1000.0)).await?;
    /// mgr.send(MotorCommand::PiGains { kp: 0.5, ki: 0.05 }).await?;
    /// mgr.send(MotorCommand::Enable(true)).await?;
    /// ```
    pub async fn send(&self, cmd: MotorCommand) -> Result<()> {
        let id = cmd.can_id();
        let data = cmd.encode();
        debug!("Sending command {:?}", cmd);
        self.send_frame(id, &data).await
    }

    // ========================================================================
    // Receive Methods
    // ========================================================================

    /// Receive next CAN frame with timeout
    ///
    /// # Arguments
    /// * `timeout_ms` - Timeout in milliseconds
    ///
    /// # Returns
    /// * `Ok(Some(frame))` if frame received
    /// * `Ok(None)` if timeout occurred
    /// * `Err` if receive error
    pub async fn receive_frame(&self, timeout_ms: u64) -> Result<Option<CanFrame>> {
        let mut stream_guard = self.stream.lock().await;
        if let Some(stream) = stream_guard.as_mut() {
            stream.receive_frame(timeout_ms).await
        } else {
            Err(anyhow::anyhow!("Not connected to SLCAN adapter"))
        }
    }

    /// Parse motor status from CAN frame
    pub fn parse_motor_status(frame: &CanFrame) -> Option<MotorStatus> {
        if frame.id() == can_ids::STATUS {
            protocol::decode_status(frame.data())
        } else {
            None
        }
    }

    /// Parse voltage status from CAN frame
    pub fn parse_voltage_status(frame: &CanFrame) -> Option<VoltageStatus> {
        if frame.id() == can_ids::VOLTAGE_STATUS {
            protocol::decode_voltage_status(frame.data())
        } else {
            None
        }
    }

    /// Parse config status from CAN frame
    ///
    /// # Returns
    /// * `Some((version, crc_valid))` if config status frame
    /// * `None` if not a config status frame
    pub fn parse_config_status(frame: &CanFrame) -> Option<(u16, bool)> {
        if frame.id() == can_ids::CONFIG_STATUS {
            protocol::decode_config_status(frame.data())
        } else {
            None
        }
    }

    /// Parse calibration status from CAN frame
    ///
    /// # Returns
    /// * `Some(CalibrationStatus)` if calibration status frame
    /// * `None` if not a calibration status frame
    pub fn parse_calibration_status(frame: &CanFrame) -> Option<CalibrationStatus> {
        if frame.id() == can_ids::CALIBRATION_STATUS {
            protocol::decode_calibration_status(frame.data())
        } else {
            None
        }
    }

    // ========================================================================
    // Internal Methods
    // ========================================================================

    /// Send a CAN frame
    ///
    /// # Arguments
    /// * `id` - CAN message ID
    /// * `data` - CAN frame data
    async fn send_frame(&self, id: u32, data: &[u8]) -> Result<()> {
        let mut stream_guard = self.stream.lock().await;
        if let Some(stream) = stream_guard.as_mut() {
            let frame = CanFrame::new(id, data)
                .with_context(|| format!("Failed to create CAN frame with ID 0x{:X}", id))?;

            debug!("Sending CAN frame: ID=0x{:X}, len={}", id, data.len());

            stream
                .send_frame(&frame)
                .await
                .with_context(|| format!("Failed to send CAN frame with ID 0x{:X}", id))?;

            Ok(())
        } else {
            Err(anyhow::anyhow!("Not connected to SLCAN adapter"))
        }
    }
}

impl Default for CanManager {
    fn default() -> Self {
        Self::new()
    }
}
