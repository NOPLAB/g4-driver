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

    /// Send save config command
    pub async fn send_save_config(&self) -> Result<()> {
        info!("Sending save config command");
        self.send_frame(can_ids::SAVE_CONFIG, &[]).await
    }

    /// Send reload config command
    pub async fn send_reload_config(&self) -> Result<()> {
        info!("Sending reload config command");
        self.send_frame(can_ids::RELOAD_CONFIG, &[]).await
    }

    /// Send reset config command
    pub async fn send_reset_config(&self) -> Result<()> {
        info!("Sending reset config command");
        self.send_frame(can_ids::RESET_CONFIG, &[]).await
    }

    // ========================================================================
    // Motor Control Parameter Commands
    // ========================================================================

    /// Send motor voltage parameters
    ///
    /// # Arguments
    /// * `max_voltage` - Maximum voltage in volts
    /// * `v_dc_bus` - DC bus voltage in volts
    pub async fn send_motor_voltage_params(&self, max_voltage: f32, v_dc_bus: f32) -> Result<()> {
        let data = protocol::encode_motor_voltage_params(max_voltage, v_dc_bus);
        self.send_frame(can_ids::MOTOR_VOLTAGE_PARAMS, &data).await
    }

    /// Send motor basic parameters
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of pole pairs
    /// * `max_duty` - Maximum duty cycle
    pub async fn send_motor_basic_params(&self, pole_pairs: u8, max_duty: u16) -> Result<()> {
        let data = protocol::encode_motor_basic_params(pole_pairs, max_duty);
        self.send_frame(can_ids::MOTOR_BASIC_PARAMS, &data).await
    }

    /// Send hall sensor parameters
    ///
    /// # Arguments
    /// * `speed_filter_alpha` - Speed filter alpha coefficient
    /// * `hall_angle_offset` - Hall angle offset in radians
    pub async fn send_hall_sensor_params(
        &self,
        speed_filter_alpha: f32,
        hall_angle_offset: f32,
    ) -> Result<()> {
        let data = protocol::encode_hall_sensor_params(speed_filter_alpha, hall_angle_offset);
        self.send_frame(can_ids::HALL_SENSOR_PARAMS, &data).await
    }

    /// Send angle interpolation enable/disable
    ///
    /// # Arguments
    /// * `enable` - Enable angle interpolation
    pub async fn send_angle_interpolation(&self, enable: bool) -> Result<()> {
        let data = protocol::encode_angle_interpolation(enable);
        self.send_frame(can_ids::ANGLE_INTERPOLATION, &data).await
    }

    // ========================================================================
    // OpenLoop Parameter Commands
    // ========================================================================

    /// Send openloop RPM parameters
    ///
    /// # Arguments
    /// * `initial_rpm` - Initial RPM for openloop ramp-up
    /// * `target_rpm` - Target RPM for switching to FOC
    pub async fn send_openloop_rpm_params(&self, initial_rpm: f32, target_rpm: f32) -> Result<()> {
        let data = protocol::encode_openloop_rpm_params(initial_rpm, target_rpm);
        self.send_frame(can_ids::OPENLOOP_RPM_PARAMS, &data).await
    }

    /// Send openloop acceleration and duty parameters
    ///
    /// # Arguments
    /// * `acceleration` - Acceleration in RPM/s
    /// * `duty_ratio` - Duty ratio (0-100)
    pub async fn send_openloop_accel_duty_params(
        &self,
        acceleration: f32,
        duty_ratio: u16,
    ) -> Result<()> {
        let data = protocol::encode_openloop_accel_duty_params(acceleration, duty_ratio);
        self.send_frame(can_ids::OPENLOOP_ACCEL_DUTY_PARAMS, &data)
            .await
    }

    // ========================================================================
    // PWM/CAN/Timing Configuration Commands
    // ========================================================================

    /// Send PWM configuration
    ///
    /// # Arguments
    /// * `frequency` - PWM frequency in Hz
    /// * `dead_time` - Dead time value
    pub async fn send_pwm_config(&self, frequency: u32, dead_time: u16) -> Result<()> {
        let data = protocol::encode_pwm_config(frequency, dead_time);
        self.send_frame(can_ids::PWM_CONFIG, &data).await
    }

    /// Send CAN configuration
    ///
    /// # Arguments
    /// * `bitrate` - CAN bitrate in bps
    pub async fn send_can_config(&self, bitrate: u32) -> Result<()> {
        let data = protocol::encode_can_config(bitrate);
        self.send_frame(can_ids::CAN_CONFIG, &data).await
    }

    /// Send control timing configuration
    ///
    /// # Arguments
    /// * `control_period_us` - Control period in microseconds
    pub async fn send_control_timing(&self, control_period_us: u64) -> Result<()> {
        let data = protocol::encode_control_timing(control_period_us);
        self.send_frame(can_ids::CONTROL_TIMING, &data).await
    }

    // ========================================================================
    // Calibration Commands
    // ========================================================================

    /// Send start calibration command
    ///
    /// # Arguments
    /// * `torque` - Optional torque value (0-100). If None, uses default.
    pub async fn send_start_calibration(&self, torque: Option<u8>) -> Result<()> {
        info!("Sending start calibration command");
        let data = protocol::encode_start_calibration(torque);
        self.send_frame(can_ids::START_CALIBRATION, &data).await
    }

    // ========================================================================
    // Extended Parameter Commands
    // ========================================================================

    /// Send advance angle parameters
    pub async fn send_advance_angle_params(&self, base_deg: f32, max_deg: f32) -> Result<()> {
        let data = protocol::encode_advance_angle_params(base_deg, max_deg);
        self.send_frame(can_ids::ADVANCE_ANGLE_PARAMS, &data).await
    }

    /// Send advance angle speed range
    pub async fn send_advance_angle_speed(&self, min_speed: f32, max_speed: f32) -> Result<()> {
        let data = protocol::encode_advance_angle_speed(min_speed, max_speed);
        self.send_frame(can_ids::ADVANCE_ANGLE_SPEED, &data).await
    }

    /// Send min voltage parameters
    pub async fn send_min_voltage_params(
        &self,
        min_voltage: f32,
        error_threshold: f32,
    ) -> Result<()> {
        let data = protocol::encode_min_voltage_params(min_voltage, error_threshold);
        self.send_frame(can_ids::MIN_VOLTAGE_PARAMS, &data).await
    }

    /// Send max speed acceleration
    pub async fn send_max_speed_accel(&self, max_accel: f32) -> Result<()> {
        let data = protocol::encode_max_speed_accel(max_accel);
        self.send_frame(can_ids::MAX_SPEED_ACCEL, &data).await
    }

    /// Send FOC stall detection parameters
    pub async fn send_foc_stall_params(
        &self,
        speed_threshold: f32,
        count_threshold: u32,
    ) -> Result<()> {
        let data = protocol::encode_foc_stall_params(speed_threshold, count_threshold);
        self.send_frame(can_ids::FOC_STALL_PARAMS, &data).await
    }

    /// Send openloop cycles parameters
    pub async fn send_openloop_cycles_params(
        &self,
        forced_cycles: u32,
        min_cycles: u32,
    ) -> Result<()> {
        let data = protocol::encode_openloop_cycles_params(forced_cycles, min_cycles);
        self.send_frame(can_ids::OPENLOOP_CYCLES_PARAMS, &data)
            .await
    }

    /// Send dead time compensation parameters
    pub async fn send_dead_time_comp_params(&self, enabled: bool, dead_time_ns: f32) -> Result<()> {
        let data = protocol::encode_dead_time_comp_params(enabled, dead_time_ns);
        self.send_frame(can_ids::DEAD_TIME_COMP_PARAMS, &data).await
    }

    /// Send flux weakening enable and min speed
    pub async fn send_flux_weakening_enable(&self, enabled: bool, min_speed: f32) -> Result<()> {
        let data = protocol::encode_flux_weakening_enable(enabled, min_speed);
        self.send_frame(can_ids::FLUX_WEAKENING_ENABLE, &data).await
    }

    /// Send flux weakening parameters
    pub async fn send_flux_weakening_params(&self, max_speed: f32, max_ratio: f32) -> Result<()> {
        let data = protocol::encode_flux_weakening_params(max_speed, max_ratio);
        self.send_frame(can_ids::FLUX_WEAKENING_PARAMS, &data).await
    }

    /// Send flux weakening Vd rate limit
    pub async fn send_flux_weakening_vd(&self, vd_rate_limit: f32) -> Result<()> {
        let data = protocol::encode_flux_weakening_vd(vd_rate_limit);
        self.send_frame(can_ids::FLUX_WEAKENING_VD, &data).await
    }

    /// Send voltage monitor thresholds
    pub async fn send_voltage_monitor_thresholds(
        &self,
        overvoltage: f32,
        undervoltage: f32,
    ) -> Result<()> {
        let data = protocol::encode_voltage_monitor_thresholds(overvoltage, undervoltage);
        self.send_frame(can_ids::VOLTAGE_MONITOR_THRESHOLDS, &data)
            .await
    }

    /// Send voltage monitor filter alpha
    pub async fn send_voltage_monitor_filter(&self, alpha: f32) -> Result<()> {
        let data = protocol::encode_voltage_monitor_filter(alpha);
        self.send_frame(can_ids::VOLTAGE_MONITOR_FILTER, &data)
            .await
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
