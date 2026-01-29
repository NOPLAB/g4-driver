//! Field Oriented Control (FOC) for BLDC motors
//!
//! This module provides the main FOC controller that coordinates
//! position sensing, speed control, and PWM generation.

use crate::control::PiController;
use crate::modulation::calculate_svpwm;
use crate::traits::PwmDuty;
use crate::transforms::{inverse_park, limit_voltage};

/// Configuration for the FOC controller
#[derive(Debug, Clone)]
pub struct FocConfig {
    /// Proportional gain for speed PI controller
    pub speed_kp: f32,
    /// Integral gain for speed PI controller
    pub speed_ki: f32,
    /// Maximum voltage output
    pub max_voltage: f32,
    /// DC bus voltage
    pub v_dc: f32,
    /// Maximum PWM duty cycle value
    pub max_duty: u16,
    /// d-axis voltage (typically 0 for SPMSM)
    pub vd: f32,
}

impl Default for FocConfig {
    fn default() -> Self {
        Self {
            speed_kp: 0.5,
            speed_ki: 0.05,
            max_voltage: 24.0,
            v_dc: 24.0,
            max_duty: 100,
            vd: 0.0,
        }
    }
}

/// FOC controller for BLDC motor speed control
///
/// This controller implements sensorless FOC using:
/// - Speed PI controller for outer loop
/// - Inverse Park transform for voltage vector rotation
/// - SVPWM for PWM generation
///
/// Note: This is a simplified version that works with external position
/// sensing. The actual position sensor is managed by the firmware layer.
#[derive(Debug)]
pub struct FocController {
    /// Speed PI controller
    speed_pi: PiController,
    /// Target speed in rad/s
    target_speed: f32,
    /// Configuration
    config: FocConfig,
}

impl FocController {
    /// Create a new FOC controller
    ///
    /// # Arguments
    /// * `config` - FOC configuration
    pub fn new(config: FocConfig) -> Self {
        let speed_pi = PiController::new_symmetric(
            config.speed_kp,
            config.speed_ki,
            config.max_voltage,
        );

        Self {
            speed_pi,
            target_speed: 0.0,
            config,
        }
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(FocConfig::default())
    }

    /// Update the FOC controller
    ///
    /// # Arguments
    /// * `measured_speed` - Current motor speed in rad/s
    /// * `electrical_angle` - Current electrical angle in radians
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// PWM duty cycles for U, V, W phases
    pub fn update(&mut self, measured_speed: f32, electrical_angle: f32, dt: f32) -> PwmDuty {
        // Speed PI controller - outputs vq voltage
        let vq = self.speed_pi.update(self.target_speed, measured_speed, dt);

        // Voltage limiting
        let (vd, vq) = limit_voltage(self.config.vd, vq, self.config.max_voltage);

        // Inverse Park transform to get alpha-beta voltages
        let (v_alpha, v_beta) = inverse_park(vd, vq, electrical_angle);

        // SVPWM to get duty cycles
        let (du, dv, dw) = calculate_svpwm(
            v_alpha,
            v_beta,
            self.config.v_dc,
            self.config.max_duty,
        );

        PwmDuty::new(du, dv, dw)
    }

    /// Set the target speed
    ///
    /// # Arguments
    /// * `speed` - Target speed in rad/s
    pub fn set_target_speed(&mut self, speed: f32) {
        self.target_speed = speed;
    }

    /// Set the target speed in RPM
    ///
    /// # Arguments
    /// * `rpm` - Target speed in RPM
    pub fn set_target_speed_rpm(&mut self, rpm: f32) {
        self.target_speed = rpm * core::f32::consts::TAU / 60.0;
    }

    /// Get the current target speed in rad/s
    #[allow(dead_code)]
    pub fn get_target_speed(&self) -> f32 {
        self.target_speed
    }

    /// Get the current target speed in RPM
    #[allow(dead_code)]
    pub fn get_target_speed_rpm(&self) -> f32 {
        self.target_speed * 60.0 / core::f32::consts::TAU
    }

    /// Set the PI controller gains
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.speed_pi.set_gains(kp, ki);
        self.config.speed_kp = kp;
        self.config.speed_ki = ki;
    }

    /// Get the proportional gain
    pub fn get_kp(&self) -> f32 {
        self.speed_pi.get_kp()
    }

    /// Get the integral gain
    pub fn get_ki(&self) -> f32 {
        self.speed_pi.get_ki()
    }

    /// Set the DC bus voltage
    ///
    /// # Arguments
    /// * `v_dc` - DC bus voltage in volts
    pub fn set_vdc(&mut self, v_dc: f32) {
        self.config.v_dc = v_dc;
    }

    /// Reset the controller state
    pub fn reset(&mut self) {
        self.speed_pi.reset();
        self.target_speed = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_controller() {
        let controller = FocController::with_defaults();
        assert!((controller.target_speed - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_set_target_speed() {
        let mut controller = FocController::with_defaults();

        controller.set_target_speed(10.0);
        assert!((controller.get_target_speed() - 10.0).abs() < 0.001);

        controller.set_target_speed_rpm(600.0);
        let expected = 600.0 * core::f32::consts::TAU / 60.0;
        assert!((controller.get_target_speed() - expected).abs() < 0.001);
    }

    #[test]
    fn test_update_output_range() {
        let mut controller = FocController::new(FocConfig {
            max_duty: 100,
            ..Default::default()
        });

        controller.set_target_speed_rpm(1000.0);

        let duty = controller.update(0.0, 0.0, 0.001);

        // Duty cycles should be within valid range
        assert!(duty.u <= 100);
        assert!(duty.v <= 100);
        assert!(duty.w <= 100);
    }

    #[test]
    fn test_zero_speed() {
        let mut controller = FocController::with_defaults();

        // With zero target and measured speed, output should be ~50% duty
        let duty = controller.update(0.0, 0.0, 0.001);

        // Should be around 50% for all phases
        assert!(duty.u > 30 && duty.u < 70);
        assert!(duty.v > 30 && duty.v < 70);
        assert!(duty.w > 30 && duty.w < 70);
    }

    #[test]
    fn test_set_gains() {
        let mut controller = FocController::with_defaults();

        controller.set_gains(1.0, 0.1);

        assert!((controller.get_kp() - 1.0).abs() < 0.001);
        assert!((controller.get_ki() - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_reset() {
        let mut controller = FocController::with_defaults();

        controller.set_target_speed(10.0);
        controller.update(0.0, 0.0, 0.001);

        controller.reset();

        assert!((controller.target_speed - 0.0).abs() < 0.001);
    }
}
