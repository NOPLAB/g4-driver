// Allow dead code for deprecated legacy methods
#![allow(dead_code)]

use anyhow::{Context, Result};
use futures::StreamExt;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{timeout, Duration};
use tokio_socketcan::{CANFrame, CANSocket};
use tracing::{debug, info};

use g4_driver_protocol::{
    self as protocol, can_ids, CalibrationStatus, MotorStatus, VoltageStatus,
};

use super::commands::MotorCommand;

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
    #[allow(dead_code)]
    pub async fn is_connected(&self) -> bool {
        self.socket.lock().await.is_some()
    }

    /// Get current interface name
    #[allow(dead_code)]
    pub fn interface_name(&self) -> &str {
        &self.interface_name
    }

    // ========================================================================
    // New Unified API
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
    // Legacy API (deprecated - use send() instead)
    // ========================================================================

    /// Send speed command
    #[deprecated(since = "0.2.0", note = "Use send(MotorCommand::Speed(rpm)) instead")]
    pub async fn send_speed_command(&self, speed_rpm: f32) -> Result<()> {
        self.send(MotorCommand::Speed(speed_rpm)).await
    }

    /// Send PI gains
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::PiGains { kp, ki }) instead"
    )]
    pub async fn send_pi_gains(&self, kp: f32, ki: f32) -> Result<()> {
        self.send(MotorCommand::PiGains { kp, ki }).await
    }

    /// Send motor enable command
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::Enable(enabled)) instead"
    )]
    pub async fn send_enable_command(&self, enable: bool) -> Result<()> {
        self.send(MotorCommand::Enable(enable)).await
    }

    /// Send emergency stop command
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::EmergencyStop) instead"
    )]
    pub async fn send_emergency_stop(&self) -> Result<()> {
        info!("Sending emergency stop");
        self.send(MotorCommand::EmergencyStop).await
    }

    /// Send save config command
    #[deprecated(since = "0.2.0", note = "Use send(MotorCommand::SaveConfig) instead")]
    pub async fn send_save_config(&self) -> Result<()> {
        info!("Sending save config command");
        self.send(MotorCommand::SaveConfig).await
    }

    /// Send reload config command
    #[deprecated(since = "0.2.0", note = "Use send(MotorCommand::ReloadConfig) instead")]
    pub async fn send_reload_config(&self) -> Result<()> {
        info!("Sending reload config command");
        self.send(MotorCommand::ReloadConfig).await
    }

    /// Send reset config command
    #[deprecated(since = "0.2.0", note = "Use send(MotorCommand::ResetConfig) instead")]
    pub async fn send_reset_config(&self) -> Result<()> {
        info!("Sending reset config command");
        self.send(MotorCommand::ResetConfig).await
    }

    /// Send motor voltage parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::MotorVoltage { max_voltage, v_dc_bus }) instead"
    )]
    pub async fn send_motor_voltage_params(&self, max_voltage: f32, v_dc_bus: f32) -> Result<()> {
        self.send(MotorCommand::MotorVoltage {
            max_voltage,
            v_dc_bus,
        })
        .await
    }

    /// Send motor basic parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::MotorBasic { pole_pairs, max_duty }) instead"
    )]
    pub async fn send_motor_basic_params(&self, pole_pairs: u8, max_duty: u16) -> Result<()> {
        self.send(MotorCommand::MotorBasic {
            pole_pairs,
            max_duty,
        })
        .await
    }

    /// Send hall sensor parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::HallSensor { speed_filter_alpha, angle_offset }) instead"
    )]
    pub async fn send_hall_sensor_params(
        &self,
        speed_filter_alpha: f32,
        hall_angle_offset: f32,
    ) -> Result<()> {
        self.send(MotorCommand::HallSensor {
            speed_filter_alpha,
            angle_offset: hall_angle_offset,
        })
        .await
    }

    /// Send angle interpolation enable/disable
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::AngleInterpolation(enabled)) instead"
    )]
    pub async fn send_angle_interpolation(&self, enable: bool) -> Result<()> {
        self.send(MotorCommand::AngleInterpolation(enable)).await
    }

    /// Send openloop RPM parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::OpenLoopRpm { initial_rpm, target_rpm }) instead"
    )]
    pub async fn send_openloop_rpm_params(&self, initial_rpm: f32, target_rpm: f32) -> Result<()> {
        self.send(MotorCommand::OpenLoopRpm {
            initial_rpm,
            target_rpm,
        })
        .await
    }

    /// Send openloop acceleration and duty parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::OpenLoopAccelDuty { acceleration, duty_ratio }) instead"
    )]
    pub async fn send_openloop_accel_duty_params(
        &self,
        acceleration: f32,
        duty_ratio: u16,
    ) -> Result<()> {
        self.send(MotorCommand::OpenLoopAccelDuty {
            acceleration,
            duty_ratio,
        })
        .await
    }

    /// Send PWM configuration
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::PwmConfig { frequency, dead_time }) instead"
    )]
    pub async fn send_pwm_config(&self, frequency: u32, dead_time: u16) -> Result<()> {
        self.send(MotorCommand::PwmConfig {
            frequency,
            dead_time,
        })
        .await
    }

    /// Send CAN configuration
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::CanConfig(bitrate)) instead"
    )]
    pub async fn send_can_config(&self, bitrate: u32) -> Result<()> {
        self.send(MotorCommand::CanConfig(bitrate)).await
    }

    /// Send control timing configuration
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::ControlTiming(period_us)) instead"
    )]
    pub async fn send_control_timing(&self, control_period_us: u64) -> Result<()> {
        self.send(MotorCommand::ControlTiming(control_period_us))
            .await
    }

    /// Send start calibration command
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::StartCalibration { torque }) instead"
    )]
    pub async fn send_start_calibration(&self, torque: Option<u8>) -> Result<()> {
        info!("Sending start calibration command");
        self.send(MotorCommand::StartCalibration { torque }).await
    }

    /// Send advance angle parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::AdvanceAngle { base_deg, max_deg }) instead"
    )]
    pub async fn send_advance_angle_params(&self, base_deg: f32, max_deg: f32) -> Result<()> {
        self.send(MotorCommand::AdvanceAngle { base_deg, max_deg })
            .await
    }

    /// Send advance angle speed range
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::AdvanceAngleSpeed { min_speed, max_speed }) instead"
    )]
    pub async fn send_advance_angle_speed(&self, min_speed: f32, max_speed: f32) -> Result<()> {
        self.send(MotorCommand::AdvanceAngleSpeed {
            min_speed,
            max_speed,
        })
        .await
    }

    /// Send min voltage parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::MinVoltage { min_voltage, error_threshold }) instead"
    )]
    pub async fn send_min_voltage_params(
        &self,
        min_voltage: f32,
        error_threshold: f32,
    ) -> Result<()> {
        self.send(MotorCommand::MinVoltage {
            min_voltage,
            error_threshold,
        })
        .await
    }

    /// Send max speed acceleration
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::MaxSpeedAccel(accel)) instead"
    )]
    pub async fn send_max_speed_accel(&self, max_accel: f32) -> Result<()> {
        self.send(MotorCommand::MaxSpeedAccel(max_accel)).await
    }

    /// Send FOC stall detection parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::FocStall { speed_threshold, count_threshold }) instead"
    )]
    pub async fn send_foc_stall_params(
        &self,
        speed_threshold: f32,
        count_threshold: u32,
    ) -> Result<()> {
        self.send(MotorCommand::FocStall {
            speed_threshold,
            count_threshold,
        })
        .await
    }

    /// Send openloop cycles parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::OpenLoopCycles { forced_cycles, min_cycles }) instead"
    )]
    pub async fn send_openloop_cycles_params(
        &self,
        forced_cycles: u32,
        min_cycles: u32,
    ) -> Result<()> {
        self.send(MotorCommand::OpenLoopCycles {
            forced_cycles,
            min_cycles,
        })
        .await
    }

    /// Send dead time compensation parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::DeadTimeComp { enabled, dead_time_ns }) instead"
    )]
    pub async fn send_dead_time_comp_params(&self, enabled: bool, dead_time_ns: f32) -> Result<()> {
        self.send(MotorCommand::DeadTimeComp {
            enabled,
            dead_time_ns,
        })
        .await
    }

    /// Send flux weakening enable and min speed
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::FluxWeakeningEnable { enabled, min_speed }) instead"
    )]
    pub async fn send_flux_weakening_enable(&self, enabled: bool, min_speed: f32) -> Result<()> {
        self.send(MotorCommand::FluxWeakeningEnable { enabled, min_speed })
            .await
    }

    /// Send flux weakening parameters
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::FluxWeakeningParams { max_speed, max_ratio }) instead"
    )]
    pub async fn send_flux_weakening_params(&self, max_speed: f32, max_ratio: f32) -> Result<()> {
        self.send(MotorCommand::FluxWeakeningParams {
            max_speed,
            max_ratio,
        })
        .await
    }

    /// Send flux weakening Vd rate limit
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::FluxWeakeningVd(rate_limit)) instead"
    )]
    pub async fn send_flux_weakening_vd(&self, vd_rate_limit: f32) -> Result<()> {
        self.send(MotorCommand::FluxWeakeningVd(vd_rate_limit))
            .await
    }

    /// Send voltage monitor thresholds
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::VoltageMonitorThresholds { overvoltage, undervoltage }) instead"
    )]
    pub async fn send_voltage_monitor_thresholds(
        &self,
        overvoltage: f32,
        undervoltage: f32,
    ) -> Result<()> {
        self.send(MotorCommand::VoltageMonitorThresholds {
            overvoltage,
            undervoltage,
        })
        .await
    }

    /// Send voltage monitor filter alpha
    #[deprecated(
        since = "0.2.0",
        note = "Use send(MotorCommand::VoltageMonitorFilter(alpha)) instead"
    )]
    pub async fn send_voltage_monitor_filter(&self, alpha: f32) -> Result<()> {
        self.send(MotorCommand::VoltageMonitorFilter(alpha)).await
    }

    // ========================================================================
    // Receive Methods (not deprecated)
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

    /// Parse config status from CAN frame
    ///
    /// # Returns
    /// * `Some((version, crc_valid))` if config status frame
    /// * `None` if not a config status frame
    pub fn parse_config_status(frame: &CANFrame) -> Option<(u16, bool)> {
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
    pub fn parse_calibration_status(frame: &CANFrame) -> Option<CalibrationStatus> {
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
