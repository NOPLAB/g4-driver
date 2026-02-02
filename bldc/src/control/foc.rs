//! Field Oriented Control (FOC) for BLDC motors
//!
//! This module provides the main FOC controller that coordinates
//! position sensing, speed control, and PWM generation.

use crate::compensation::{DeadTimeCompensation, FluxWeakeningController};
use crate::control::speed_ramp::SpeedRamp;
use crate::control::stall_detector::{StallDetector, StallDetectorConfig};
use crate::control::PiController;
use crate::modulation::calculate_svpwm;
use crate::traits::PwmDuty;
use crate::transforms::{inverse_park, limit_voltage};
use core::f32::consts::TAU;

/// Conversion factor from RPM to rad/s: TAU / 60
const RPM_TO_RAD_S: f32 = TAU / 60.0;
/// Conversion factor from rad/s to RPM: 60 / TAU
const RAD_S_TO_RPM: f32 = 60.0 / TAU;

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
    /// Maximum acceleration rate [RPM/s] (0 = disabled)
    pub max_acceleration: f32,
    /// Minimum voltage for stall prevention [V] (0 = disabled)
    pub min_voltage: f32,
    /// Speed error threshold for minimum voltage application [RPM]
    pub min_voltage_error_threshold: f32,
    /// PI integral limit [V]
    pub pi_integral_limit: f32,
    /// Enable anti-windup for PI controller
    pub anti_windup_enabled: bool,
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
            max_acceleration: 500.0,
            min_voltage: 2.0,
            min_voltage_error_threshold: 10.0,
            pi_integral_limit: 12.0,
            anti_windup_enabled: true,
        }
    }
}

/// Output from FOC controller update
#[derive(Debug, Clone, Copy, Default)]
pub struct FocOutput {
    /// PWM duty cycles for U, V, W phases
    pub duty: PwmDuty,
    /// Whether stall is detected
    pub is_stalled: bool,
    /// q-axis voltage command (limited)
    pub vq: f32,
    /// d-axis voltage command (limited)
    pub vd: f32,
    /// Ramped target speed [RPM]
    pub ramped_target_speed: f32,
}

/// FOC controller builder for optional components
pub struct FocControllerBuilder {
    config: FocConfig,
    dead_time_comp: Option<DeadTimeCompensation>,
    flux_weakening: Option<FluxWeakeningController>,
    stall_config: Option<StallDetectorConfig>,
}

impl FocControllerBuilder {
    /// Create a new builder with configuration
    pub fn new(config: FocConfig) -> Self {
        Self {
            config,
            dead_time_comp: None,
            flux_weakening: None,
            stall_config: None,
        }
    }

    /// Add dead time compensation
    pub fn with_dead_time_compensation(mut self, comp: DeadTimeCompensation) -> Self {
        self.dead_time_comp = Some(comp);
        self
    }

    /// Add flux weakening controller
    pub fn with_flux_weakening(mut self, fw: FluxWeakeningController) -> Self {
        self.flux_weakening = Some(fw);
        self
    }

    /// Add stall detection
    pub fn with_stall_detection(mut self, config: StallDetectorConfig) -> Self {
        self.stall_config = Some(config);
        self
    }

    /// Build the FOC controller
    pub fn build(self) -> FocController {
        let mut speed_pi = PiController::new_symmetric(
            self.config.speed_kp,
            self.config.speed_ki,
            self.config.max_voltage,
        );
        speed_pi.set_anti_windup(self.config.anti_windup_enabled);
        speed_pi.set_integral_limit(self.config.pi_integral_limit);

        let speed_ramp = if self.config.max_acceleration > 0.0 {
            Some(SpeedRamp::new(self.config.max_acceleration))
        } else {
            None
        };

        let stall_detector = self.stall_config.map(StallDetector::new);

        FocController {
            speed_pi,
            target_speed: 0.0,
            config: self.config,
            speed_ramp,
            stall_detector,
            dead_time_comp: self.dead_time_comp,
            flux_weakening: self.flux_weakening,
        }
    }
}

/// FOC controller for BLDC motor speed control
///
/// This controller implements sensorless FOC using:
/// - Speed PI controller for outer loop
/// - Inverse Park transform for voltage vector rotation
/// - SVPWM for PWM generation
/// - Optional: speed ramp, stall detection, dead time compensation, flux weakening
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
    /// Speed ramp (optional)
    speed_ramp: Option<SpeedRamp>,
    /// Stall detector (optional)
    stall_detector: Option<StallDetector>,
    /// Dead time compensation (optional)
    dead_time_comp: Option<DeadTimeCompensation>,
    /// Flux weakening controller (optional)
    flux_weakening: Option<FluxWeakeningController>,
}

impl FocController {
    /// Create a new FOC controller
    ///
    /// # Arguments
    /// * `config` - FOC configuration
    pub fn new(config: FocConfig) -> Self {
        FocControllerBuilder::new(config).build()
    }

    /// Create a builder for FOC controller
    pub fn builder(config: FocConfig) -> FocControllerBuilder {
        FocControllerBuilder::new(config)
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(FocConfig::default())
    }

    /// Update the FOC controller (simple version without optional features)
    ///
    /// # Arguments
    /// * `measured_speed` - Current motor speed in rad/s
    /// * `electrical_angle` - Current electrical angle in radians
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// PWM duty cycles for U, V, W phases
    pub fn update(&mut self, measured_speed: f32, electrical_angle: f32, dt: f32) -> PwmDuty {
        self.update_extended(measured_speed, electrical_angle, dt)
            .duty
    }

    /// Update the FOC controller with all optional features
    ///
    /// # Arguments
    /// * `measured_speed` - Current motor speed in RPM
    /// * `electrical_angle` - Current electrical angle in radians
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// FocOutput containing duty cycles and status information
    pub fn update_extended(
        &mut self,
        measured_speed: f32,
        electrical_angle: f32,
        dt: f32,
    ) -> FocOutput {
        // Speed ramp processing
        let ramped_target = if let Some(ref mut ramp) = self.speed_ramp {
            ramp.update(self.target_speed, dt)
        } else {
            self.target_speed
        };

        // Speed PI controller - outputs vq voltage
        let mut vq_cmd = self.speed_pi.update(ramped_target, measured_speed, dt);

        // Flux weakening (optional)
        let vd_cmd = if let Some(ref mut fw) = self.flux_weakening {
            fw.calculate_vd(measured_speed, vq_cmd, self.config.v_dc, dt)
        } else {
            self.config.vd
        };

        // Stop handling: reset PI when target and measured are near zero
        if ramped_target.abs() < 1.0 && measured_speed.abs() < 1.0 {
            self.speed_pi.reset();
            vq_cmd = 0.0;
        }

        // Minimum voltage application (stall prevention)
        if self.config.min_voltage > 0.0 {
            let speed_error = ramped_target - measured_speed;
            if speed_error > self.config.min_voltage_error_threshold {
                // Accelerating forward
                vq_cmd = vq_cmd.max(self.config.min_voltage);
            } else if speed_error < -self.config.min_voltage_error_threshold {
                // Accelerating reverse
                vq_cmd = vq_cmd.min(-self.config.min_voltage);
            }
        }

        // Voltage vector limiting
        let (vd_limited, vq_limited) = limit_voltage(vd_cmd, vq_cmd, self.config.max_voltage);

        // Inverse Park transform to get alpha-beta voltages
        let (v_alpha, v_beta) = inverse_park(vd_limited, vq_limited, electrical_angle);

        // SVPWM to get duty cycles
        let (du, dv, dw) = calculate_svpwm(v_alpha, v_beta, self.config.v_dc, self.config.max_duty);

        // Dead time compensation (optional)
        let (du, dv, dw) = if let Some(ref comp) = self.dead_time_comp {
            comp.compensate(
                du,
                dv,
                dw,
                vq_limited,
                electrical_angle,
                self.config.max_duty,
            )
        } else {
            (du, dv, dw)
        };

        // Stall detection (optional)
        let is_stalled = if let Some(ref mut detector) = self.stall_detector {
            detector.update(ramped_target, measured_speed)
        } else {
            false
        };

        FocOutput {
            duty: PwmDuty::new(du, dv, dw),
            is_stalled,
            vq: vq_limited,
            vd: vd_limited,
            ramped_target_speed: ramped_target,
        }
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
        self.target_speed = rpm * RPM_TO_RAD_S;
    }

    /// Get the current target speed in rad/s
    #[allow(dead_code)]
    pub fn get_target_speed(&self) -> f32 {
        self.target_speed
    }

    /// Get the current target speed in RPM
    #[allow(dead_code)]
    pub fn get_target_speed_rpm(&self) -> f32 {
        self.target_speed * RAD_S_TO_RPM
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
        if let Some(ref mut ramp) = self.speed_ramp {
            ramp.reset();
        }
        if let Some(ref mut detector) = self.stall_detector {
            detector.reset();
        }
        if let Some(ref mut fw) = self.flux_weakening {
            fw.reset();
        }
    }

    /// Initialize for FOC mode transition
    ///
    /// Sets initial values for smooth transition from open-loop to FOC
    ///
    /// # Arguments
    /// * `current_speed` - Current motor speed [RPM]
    /// * `initial_vq` - Initial Vq output (for PI initialization)
    pub fn initialize_for_foc(&mut self, current_speed: f32, initial_vq: f32) {
        self.speed_pi.initialize_output(initial_vq);
        if let Some(ref mut ramp) = self.speed_ramp {
            ramp.set_current_speed(current_speed);
        }
        if let Some(ref mut detector) = self.stall_detector {
            detector.reset();
        }
        if let Some(ref mut fw) = self.flux_weakening {
            fw.reset();
        }
    }

    /// Get reference to the speed ramp (if configured)
    pub fn speed_ramp(&self) -> Option<&SpeedRamp> {
        self.speed_ramp.as_ref()
    }

    /// Get mutable reference to the speed ramp (if configured)
    pub fn speed_ramp_mut(&mut self) -> Option<&mut SpeedRamp> {
        self.speed_ramp.as_mut()
    }

    /// Get reference to the stall detector (if configured)
    pub fn stall_detector(&self) -> Option<&StallDetector> {
        self.stall_detector.as_ref()
    }

    /// Get mutable reference to the stall detector (if configured)
    pub fn stall_detector_mut(&mut self) -> Option<&mut StallDetector> {
        self.stall_detector.as_mut()
    }

    /// Get reference to the dead time compensator (if configured)
    pub fn dead_time_comp(&self) -> Option<&DeadTimeCompensation> {
        self.dead_time_comp.as_ref()
    }

    /// Get reference to the flux weakening controller (if configured)
    pub fn flux_weakening(&self) -> Option<&FluxWeakeningController> {
        self.flux_weakening.as_ref()
    }

    /// Get mutable reference to the flux weakening controller (if configured)
    pub fn flux_weakening_mut(&mut self) -> Option<&mut FluxWeakeningController> {
        self.flux_weakening.as_mut()
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
        assert!((controller.get_target_speed_rpm() - 600.0).abs() < 0.001);
    }

    #[test]
    fn test_update_output_range() {
        let mut controller = FocController::new(FocConfig {
            max_duty: 100,
            max_acceleration: 0.0, // Disable ramp for this test
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
        let mut controller = FocController::new(FocConfig {
            max_acceleration: 0.0, // Disable ramp for this test
            ..Default::default()
        });

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

    #[test]
    fn test_builder_basic() {
        let controller = FocController::builder(FocConfig::default()).build();
        assert!(controller.stall_detector.is_none());
        assert!(controller.dead_time_comp.is_none());
        assert!(controller.flux_weakening.is_none());
    }

    #[test]
    fn test_builder_with_stall_detection() {
        let controller = FocController::builder(FocConfig::default())
            .with_stall_detection(StallDetectorConfig {
                speed_threshold: 100.0,
                count_threshold: 500,
            })
            .build();

        assert!(controller.stall_detector.is_some());
    }

    #[test]
    fn test_update_extended_stall_detection() {
        let mut controller = FocController::builder(FocConfig {
            max_acceleration: 0.0, // Disable ramp
            ..Default::default()
        })
        .with_stall_detection(StallDetectorConfig {
            speed_threshold: 50.0,
            count_threshold: 5,
        })
        .build();

        controller.set_target_speed_rpm(500.0);

        // Simulate stall condition
        for _ in 0..4 {
            let output = controller.update_extended(10.0, 0.0, 0.001);
            assert!(!output.is_stalled);
        }

        // 5th cycle should trigger stall
        let output = controller.update_extended(10.0, 0.0, 0.001);
        assert!(output.is_stalled);
    }

    #[test]
    fn test_speed_ramp() {
        let mut controller = FocController::new(FocConfig {
            max_acceleration: 100.0, // 100 RPM/s
            ..Default::default()
        });

        controller.set_target_speed_rpm(1000.0);

        // With dt=0.1s, max delta is 10 RPM
        let output = controller.update_extended(0.0, 0.0, 0.1);
        assert!((output.ramped_target_speed - 10.0).abs() < 0.1);

        let output = controller.update_extended(0.0, 0.0, 0.1);
        assert!((output.ramped_target_speed - 20.0).abs() < 0.1);
    }
}
