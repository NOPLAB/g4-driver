//! Hardware abstraction traits for BLDC motor control
//!
//! These traits define the interface between the motor control algorithms
//! and the hardware-specific implementations.

/// Hall state reader trait for reading Hall sensor state
///
/// This trait is used by the calibration module to read the current Hall sector.
pub trait HallStateReader {
    /// Get the current Hall state (1-6 for valid states)
    fn get_hall_state(&self) -> u8;
}

/// Position sensor trait for reading rotor position
pub trait PositionSensor {
    /// Get the electrical angle in radians (0.0 to 2*PI)
    fn electrical_angle(&self) -> f32;

    /// Get the mechanical angle in radians (0.0 to 2*PI)
    fn mechanical_angle(&self) -> f32;
}

/// Speed sensor trait for reading rotor speed
pub trait SpeedSensor {
    /// Get the current speed in radians per second
    fn speed_rad_s(&self) -> f32;

    /// Get the current speed in RPM
    fn speed_rpm(&self) -> f32 {
        self.speed_rad_s() * 60.0 / (2.0 * core::f32::consts::PI)
    }
}

/// PWM output trait for controlling the three-phase inverter
pub trait PwmOutput {
    /// Set the duty cycle for each phase (values in range 0.0 to 1.0)
    ///
    /// # Arguments
    /// * `u` - U phase duty cycle
    /// * `v` - V phase duty cycle
    /// * `w` - W phase duty cycle
    fn set_duty(&mut self, u: f32, v: f32, w: f32);

    /// Enable PWM output
    fn enable(&mut self);

    /// Disable PWM output
    fn disable(&mut self);
}

/// Current sensor trait for reading phase currents
pub trait CurrentSensor {
    /// Get the current in the alpha-beta reference frame
    ///
    /// # Returns
    /// Tuple of (i_alpha, i_beta) in Amperes
    fn current_alpha_beta(&self) -> (f32, f32);

    /// Get the phase currents
    ///
    /// # Returns
    /// Tuple of (i_u, i_v, i_w) in Amperes
    fn phase_currents(&self) -> (f32, f32, f32);
}

/// PWM duty cycle output from motor control algorithms
#[derive(Debug, Clone, Copy, Default)]
pub struct PwmDuty {
    /// U phase duty cycle (0 to max_duty)
    pub u: u16,
    /// V phase duty cycle (0 to max_duty)
    pub v: u16,
    /// W phase duty cycle (0 to max_duty)
    pub w: u16,
}

impl PwmDuty {
    /// Create a new PWM duty cycle
    pub fn new(u: u16, v: u16, w: u16) -> Self {
        Self { u, v, w }
    }

    /// Convert to normalized duty cycles (0.0 to 1.0)
    pub fn to_normalized(&self, max_duty: u16) -> (f32, f32, f32) {
        let max = max_duty as f32;
        (
            self.u as f32 / max,
            self.v as f32 / max,
            self.w as f32 / max,
        )
    }

    /// Create from normalized duty cycles (0.0 to 1.0)
    pub fn from_normalized(u: f32, v: f32, w: f32, max_duty: u16) -> Self {
        let max = max_duty as f32;
        Self {
            u: (u * max).clamp(0.0, max) as u16,
            v: (v * max).clamp(0.0, max) as u16,
            w: (w * max).clamp(0.0, max) as u16,
        }
    }
}

/// Control mode for the motor state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ControlMode {
    /// Open-loop control (startup/recovery)
    #[default]
    OpenLoop,
    /// Field Oriented Control (closed-loop speed control)
    Foc,
    /// Motor calibration
    Calibration,
}

/// Control input trait for providing control commands to the state machine
///
/// This trait abstracts the source of control commands (e.g., CAN bus, simulation).
pub trait ControlInput {
    /// Get the target speed in RPM
    fn target_speed(&self) -> f32;

    /// Get the PI gains (Kp, Ki)
    fn pi_gains(&self) -> (f32, f32);

    /// Check if calibration is requested
    fn calibration_requested(&self) -> bool;

    /// Get the calibration torque (0.0 to 1.0)
    fn calibration_torque(&self) -> f32;

    /// Check if motor should be enabled
    fn motor_enabled(&self) -> bool {
        true
    }
}

/// Status output trait for reporting motor status
///
/// This trait abstracts the destination of status updates (e.g., CAN bus, simulation logging).
pub trait StatusOutput {
    /// Update motor status (speed and electrical angle)
    fn update_status(&mut self, speed_rpm: f32, electrical_angle: f32);

    /// Called when control mode changes
    fn on_mode_change(&mut self, mode: ControlMode);

    /// Called when stall is detected
    fn on_stall_detected(&mut self);

    /// Called when calibration completes
    fn on_calibration_complete(&mut self, success: bool, offset: f32, inversed: bool) {
        let _ = (success, offset, inversed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pwm_duty_to_normalized() {
        let duty = PwmDuty::new(50, 75, 100);
        let (u, v, w) = duty.to_normalized(100);
        assert!((u - 0.5).abs() < 0.001);
        assert!((v - 0.75).abs() < 0.001);
        assert!((w - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_pwm_duty_from_normalized() {
        let duty = PwmDuty::from_normalized(0.5, 0.75, 1.0, 100);
        assert_eq!(duty.u, 50);
        assert_eq!(duty.v, 75);
        assert_eq!(duty.w, 100);
    }
}
