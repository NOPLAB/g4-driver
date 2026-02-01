//! SVPWM-based open-loop controller for BLDC motors
//!
//! This module provides open-loop driving for motor startup using SVPWM.
//! It operates in two phases:
//! 1. Forced commutation: Time-based electrical angle progression
//! 2. Hall-driven: Hall sensor-based electrical angle with fixed voltage
//!
//! After reaching target speed, it can transition to FOC control.

use crate::modulation::calculate_svpwm;
use crate::traits::PwmDuty;
use crate::transforms::inverse_park;

/// Configuration for the open-loop controller
#[derive(Debug, Clone)]
pub struct OpenLoopConfig {
    /// Initial rotation speed [RPM]
    pub initial_rpm: f32,
    /// Target rotation speed [RPM] (for FOC transition)
    pub target_rpm: f32,
    /// Acceleration rate [RPM/s]
    pub acceleration: f32,
    /// Voltage command ratio (0.0 - 1.0)
    pub voltage_ratio: f32,
    /// DC bus voltage [V]
    pub v_dc: f32,
    /// Maximum PWM duty cycle value
    pub max_duty: u16,
    /// Number of pole pairs
    pub pole_pairs: u8,
    /// Number of cycles for forced commutation phase
    pub forced_commutation_cycles: u32,
    /// Minimum cycles before FOC transition
    pub min_cycles_for_foc: u32,
    /// Minimum speed for FOC transition [RPM]
    pub min_speed_for_foc: f32,
}

impl Default for OpenLoopConfig {
    fn default() -> Self {
        Self {
            initial_rpm: 50.0,
            target_rpm: 100.0,
            acceleration: 200.0,
            voltage_ratio: 0.1, // 10%
            v_dc: 24.0,
            max_duty: 100,
            pole_pairs: 6,
            forced_commutation_cycles: 50000,
            min_cycles_for_foc: 50000,
            min_speed_for_foc: 100.0,
        }
    }
}

/// Open-loop phase
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OpenLoopPhase {
    /// Time-based forced commutation
    #[default]
    ForcedCommutation,
    /// Hall sensor-driven commutation
    HallDriven,
}

/// Output from open-loop controller update
#[derive(Debug, Clone, Copy, Default)]
pub struct OpenLoopOutput {
    /// PWM duty cycles for U, V, W phases
    pub duty: PwmDuty,
    /// Whether ready to transition to FOC
    pub ready_for_foc: bool,
    /// Current theoretical/measured speed [RPM]
    pub current_rpm: f32,
    /// Current phase
    pub phase: OpenLoopPhase,
}

/// SVPWM-based open-loop controller for motor startup
///
/// Drives the motor using SVPWM with:
/// 1. Forced commutation phase: Time-based angle progression
/// 2. Hall-driven phase: Hall sensor angle with fixed voltage
///
/// Compatible with FOC's SVPWM-based driving for smooth transition.
#[derive(Debug)]
pub struct OpenLoopController {
    /// Configuration
    config: OpenLoopConfig,
    /// Current electrical angle [rad]
    electrical_angle: f32,
    /// Current angular velocity [rad/s]
    angular_velocity: f32,
    /// Execution counter
    execution_count: u32,
    /// Reverse direction flag
    reverse: bool,
    /// Recovery mode flag (from FOC stall)
    is_recovery: bool,
}

impl OpenLoopController {
    /// Create a new open-loop controller
    pub fn new(config: OpenLoopConfig) -> Self {
        let initial_angular_velocity =
            config.initial_rpm * core::f32::consts::TAU / 60.0 * config.pole_pairs as f32;

        Self {
            config,
            electrical_angle: 0.0,
            angular_velocity: initial_angular_velocity,
            execution_count: 0,
            reverse: false,
            is_recovery: false,
        }
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(OpenLoopConfig::default())
    }

    /// Update the open-loop controller
    ///
    /// # Arguments
    /// * `hall_electrical_angle` - Hall sensor electrical angle [rad] (if available)
    /// * `hall_speed_rpm` - Hall sensor measured speed [RPM] (absolute value)
    /// * `is_valid_hall` - Whether Hall sensor reading is valid
    /// * `dt` - Time step [s]
    ///
    /// # Returns
    /// OpenLoopOutput containing duty cycles and status
    pub fn update(
        &mut self,
        hall_electrical_angle: Option<f32>,
        hall_speed_rpm: f32,
        is_valid_hall: bool,
        dt: f32,
    ) -> OpenLoopOutput {
        self.execution_count += 1;

        let (duty, phase, current_rpm) =
            if self.execution_count < self.config.forced_commutation_cycles {
                // Phase 1: Forced commutation (time-based)
                self.update_forced_commutation(dt)
            } else {
                // Phase 2: Hall-driven commutation
                self.update_hall_driven(hall_electrical_angle, hall_speed_rpm, is_valid_hall)
            };

        // FOC transition check
        let time_ok = self.execution_count >= self.config.min_cycles_for_foc;
        let speed_ok = hall_speed_rpm >= self.config.min_speed_for_foc;
        let ready_for_foc = time_ok && is_valid_hall && speed_ok;

        OpenLoopOutput {
            duty,
            ready_for_foc,
            current_rpm,
            phase,
        }
    }

    /// Update during forced commutation phase
    fn update_forced_commutation(&mut self, dt: f32) -> (PwmDuty, OpenLoopPhase, f32) {
        // Progress electrical angle based on time
        let angle_delta = self.angular_velocity * dt;
        if self.reverse {
            self.electrical_angle -= angle_delta;
        } else {
            self.electrical_angle += angle_delta;
        }

        // Normalize angle to [0, 2π)
        self.electrical_angle = normalize_angle(self.electrical_angle);

        // Accelerate if not at target
        let target_angular_velocity =
            self.config.target_rpm * core::f32::consts::TAU / 60.0 * self.config.pole_pairs as f32;

        if self.angular_velocity < target_angular_velocity {
            let accel_rad = self.config.acceleration * core::f32::consts::TAU / 60.0
                * self.config.pole_pairs as f32;
            self.angular_velocity += accel_rad * dt;
            if self.angular_velocity > target_angular_velocity {
                self.angular_velocity = target_angular_velocity;
            }
        }

        // Calculate PWM using SVPWM
        let duty = self.calculate_svpwm(self.electrical_angle);

        // Calculate theoretical RPM
        let theoretical_rpm =
            self.angular_velocity * 60.0 / (core::f32::consts::TAU * self.config.pole_pairs as f32);
        let signed_rpm = if self.reverse {
            -theoretical_rpm
        } else {
            theoretical_rpm
        };

        (duty, OpenLoopPhase::ForcedCommutation, signed_rpm)
    }

    /// Update during Hall-driven phase
    fn update_hall_driven(
        &self,
        hall_electrical_angle: Option<f32>,
        hall_speed_rpm: f32,
        _is_valid_hall: bool,
    ) -> (PwmDuty, OpenLoopPhase, f32) {
        // Use Hall sensor angle if available, otherwise use last known angle
        let angle = hall_electrical_angle.unwrap_or(self.electrical_angle);

        // Calculate PWM using SVPWM
        let duty = self.calculate_svpwm(angle);

        // Use measured speed with direction
        let signed_rpm = if self.reverse {
            -hall_speed_rpm
        } else {
            hall_speed_rpm
        };

        (duty, OpenLoopPhase::HallDriven, signed_rpm)
    }

    /// Calculate SVPWM duty cycles
    fn calculate_svpwm(&self, electrical_angle: f32) -> PwmDuty {
        // Fixed voltage command
        let vq_base = self.config.voltage_ratio * self.config.v_dc;
        let vq_cmd = if self.reverse { -vq_base } else { vq_base };
        let vd_cmd = 0.0;

        // Inverse Park transform → SVPWM
        let (v_alpha, v_beta) = inverse_park(vd_cmd, vq_cmd, electrical_angle);
        let (du, dv, dw) = calculate_svpwm(v_alpha, v_beta, self.config.v_dc, self.config.max_duty);

        PwmDuty::new(du, dv, dw)
    }

    /// Set the target speed in RPM
    pub fn set_target_speed_rpm(&mut self, rpm: f32) {
        self.config.target_rpm = rpm.abs();
        self.reverse = rpm < 0.0;
    }

    /// Get the target speed in RPM (signed)
    pub fn get_target_speed_rpm(&self) -> f32 {
        if self.reverse {
            -self.config.target_rpm
        } else {
            self.config.target_rpm
        }
    }

    /// Set the rotation direction
    pub fn set_reverse(&mut self, reverse: bool) {
        self.reverse = reverse;
    }

    /// Get the current rotation direction
    pub fn is_reverse(&self) -> bool {
        self.reverse
    }

    /// Set recovery mode (from FOC stall)
    pub fn set_recovery_mode(&mut self, is_recovery: bool) {
        self.is_recovery = is_recovery;
    }

    /// Check if in recovery mode
    pub fn is_recovery(&self) -> bool {
        self.is_recovery
    }

    /// Get the theoretical speed [RPM] based on current angular velocity
    pub fn get_theoretical_rpm(&self) -> f32 {
        let rpm =
            self.angular_velocity * 60.0 / (core::f32::consts::TAU * self.config.pole_pairs as f32);
        if self.reverse {
            -rpm
        } else {
            rpm
        }
    }

    /// Get the current execution count
    pub fn get_execution_count(&self) -> u32 {
        self.execution_count
    }

    /// Get the current phase
    pub fn get_current_phase(&self) -> OpenLoopPhase {
        if self.execution_count < self.config.forced_commutation_cycles {
            OpenLoopPhase::ForcedCommutation
        } else {
            OpenLoopPhase::HallDriven
        }
    }

    /// Reset for normal startup
    pub fn reset_for_normal(&mut self) {
        let initial_angular_velocity =
            self.config.initial_rpm * core::f32::consts::TAU / 60.0 * self.config.pole_pairs as f32;

        self.electrical_angle = 0.0;
        self.angular_velocity = initial_angular_velocity;
        self.execution_count = 0;
        self.is_recovery = false;
    }

    /// Reset for recovery from stall
    pub fn reset_for_recovery(&mut self) {
        self.reset_for_normal();
        self.is_recovery = true;
    }

    /// Set voltage ratio (0.0 - 1.0)
    pub fn set_voltage_ratio(&mut self, ratio: f32) {
        self.config.voltage_ratio = ratio.clamp(0.0, 1.0);
    }

    /// Get voltage ratio
    pub fn get_voltage_ratio(&self) -> f32 {
        self.config.voltage_ratio
    }

    /// Set the DC bus voltage
    pub fn set_vdc(&mut self, v_dc: f32) {
        self.config.v_dc = v_dc;
    }
}

/// Normalize angle to [0, 2π) range
fn normalize_angle(angle: f32) -> f32 {
    let tau = core::f32::consts::TAU;
    let mut a = angle % tau;
    if a < 0.0 {
        a += tau;
    }
    a
}

// ============================================================================
// Legacy six-step support (for backward compatibility)
// ============================================================================

/// State information for six-step driving (legacy)
#[derive(Debug, Clone, Copy, Default)]
pub struct SixStepState {
    /// Current step (0-5)
    pub step: u8,
    /// U phase duty cycle (0 to max_duty)
    pub duty_u: u16,
    /// V phase duty cycle (0 to max_duty)
    pub duty_v: u16,
    /// W phase duty cycle (0 to max_duty)
    pub duty_w: u16,
    /// Whether U phase is enabled
    pub enable_u: bool,
    /// Whether V phase is enabled
    pub enable_v: bool,
    /// Whether W phase is enabled
    pub enable_w: bool,
}

/// Legacy six-step commutation controller
///
/// Maintained for backward compatibility. For new implementations,
/// use `OpenLoopController` which provides SVPWM-based driving
/// compatible with FOC transition.
#[derive(Debug)]
pub struct SixStepController {
    /// Current step (0-5)
    current_step: u8,
    /// Step switching period [s]
    step_period: f32,
    /// Initial step period [s]
    initial_step_period: f32,
    /// Acceleration rate (period multiplier per step, < 1.0)
    acceleration_rate: f32,
    /// Minimum step period [s] (corresponds to target speed)
    min_step_period: f32,
    /// Elapsed time since last step change [s]
    elapsed_time: f32,
    /// PWM duty ratio (0 to max_duty)
    duty_ratio: u16,
    /// Number of pole pairs
    pole_pairs: u8,
    /// Reverse direction flag
    reverse: bool,
}

impl SixStepController {
    /// Create a new open-loop six-step controller
    ///
    /// # Arguments
    /// * `initial_rpm` - Initial rotation speed [RPM]
    /// * `target_rpm` - Target rotation speed [RPM] (switch to FOC when reached)
    /// * `acceleration_rpm_per_s` - Acceleration rate [RPM/s]
    /// * `duty_ratio` - PWM duty ratio (0 to max_duty)
    /// * `pole_pairs` - Number of pole pairs in the motor
    pub fn new(
        initial_rpm: f32,
        target_rpm: f32,
        acceleration_rpm_per_s: f32,
        duty_ratio: u16,
        pole_pairs: u8,
    ) -> Self {
        // Calculate step period from RPM
        // 1 rotation = 6 steps * pole_pairs
        let steps_per_rotation = 6.0 * pole_pairs as f32;
        let initial_step_period = 60.0 / (initial_rpm * steps_per_rotation);
        let min_step_period = 60.0 / (target_rpm * steps_per_rotation);

        // Calculate acceleration rate
        let acceleration_rate = if acceleration_rpm_per_s > 0.0 {
            1.0 - (acceleration_rpm_per_s * initial_step_period / initial_rpm)
        } else {
            0.98 // Default
        };

        Self {
            current_step: 0,
            step_period: initial_step_period,
            initial_step_period,
            acceleration_rate,
            min_step_period,
            elapsed_time: 0.0,
            duty_ratio,
            pole_pairs,
            reverse: false,
        }
    }

    /// Set the rotation direction
    ///
    /// # Arguments
    /// * `reverse` - If true, rotate in reverse direction
    pub fn set_reverse(&mut self, reverse: bool) {
        self.reverse = reverse;
    }

    /// Get the current rotation direction
    pub fn is_reverse(&self) -> bool {
        self.reverse
    }

    /// Get the state for a specific step
    fn get_step_state(step: u8, duty: u16) -> SixStepState {
        match step % 6 {
            // Step 1: U-High (PWM), V-Low (0), W-Open (Off)
            0 => SixStepState {
                step,
                duty_u: duty,
                duty_v: 0,
                duty_w: 0,
                enable_u: true,
                enable_v: true,
                enable_w: false,
            },
            // Step 2: U-High (PWM), W-Low (0), V-Open (Off)
            1 => SixStepState {
                step,
                duty_u: duty,
                duty_v: 0,
                duty_w: 0,
                enable_u: true,
                enable_v: false,
                enable_w: true,
            },
            // Step 3: V-High (PWM), W-Low (0), U-Open (Off)
            2 => SixStepState {
                step,
                duty_u: 0,
                duty_v: duty,
                duty_w: 0,
                enable_u: false,
                enable_v: true,
                enable_w: true,
            },
            // Step 4: V-High (PWM), U-Low (0), W-Open (Off)
            3 => SixStepState {
                step,
                duty_u: 0,
                duty_v: duty,
                duty_w: 0,
                enable_u: true,
                enable_v: true,
                enable_w: false,
            },
            // Step 5: W-High (PWM), U-Low (0), V-Open (Off)
            4 => SixStepState {
                step,
                duty_u: 0,
                duty_v: 0,
                duty_w: duty,
                enable_u: true,
                enable_v: false,
                enable_w: true,
            },
            // Step 6: W-High (PWM), V-Low (0), U-Open (Off)
            5 => SixStepState {
                step,
                duty_u: 0,
                duty_v: 0,
                duty_w: duty,
                enable_u: false,
                enable_v: true,
                enable_w: true,
            },
            _ => unreachable!(),
        }
    }

    /// Update the six-step controller
    ///
    /// # Arguments
    /// * `dt` - Control period [s]
    ///
    /// # Returns
    /// Current step state with duty cycles and enable flags
    pub fn update(&mut self, dt: f32) -> SixStepState {
        self.elapsed_time += dt;

        // Check if it's time to switch steps
        if self.elapsed_time >= self.step_period {
            self.elapsed_time = 0.0;

            // Step progression (always forward for now - reverse support disabled)
            self.current_step = (self.current_step + 1) % 6;

            // Accelerate (shorten step period)
            if self.step_period > self.min_step_period {
                self.step_period *= self.acceleration_rate;
                if self.step_period < self.min_step_period {
                    self.step_period = self.min_step_period;
                }
            }
        }

        Self::get_step_state(self.current_step, self.duty_ratio)
    }

    /// Check if target speed has been reached
    #[allow(dead_code)]
    pub fn is_target_reached(&self) -> bool {
        self.step_period <= self.min_step_period
    }

    /// Reset the controller to initial state
    pub fn reset(&mut self) {
        self.current_step = 0;
        self.step_period = self.initial_step_period;
        self.elapsed_time = 0.0;
    }

    /// Get current speed [RPM]
    pub fn get_current_rpm(&self) -> f32 {
        let steps_per_rotation = 6.0 * self.pole_pairs as f32;
        60.0 / (self.step_period * steps_per_rotation)
    }

    /// Get current step (0-5)
    #[allow(dead_code)]
    pub fn get_current_step(&self) -> u8 {
        self.current_step
    }

    /// Set the duty ratio
    #[allow(dead_code)]
    pub fn set_duty_ratio(&mut self, duty: u16) {
        self.duty_ratio = duty;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_openloop_controller() {
        let controller = OpenLoopController::with_defaults();
        assert_eq!(controller.execution_count, 0);
        assert!(!controller.reverse);
    }

    #[test]
    fn test_forced_commutation_phase() {
        let mut controller = OpenLoopController::new(OpenLoopConfig {
            forced_commutation_cycles: 100,
            ..Default::default()
        });

        // Should be in forced commutation phase
        let output = controller.update(None, 0.0, false, 0.001);
        assert_eq!(output.phase, OpenLoopPhase::ForcedCommutation);
        assert!(!output.ready_for_foc);
    }

    #[test]
    fn test_hall_driven_phase() {
        let mut controller = OpenLoopController::new(OpenLoopConfig {
            forced_commutation_cycles: 10,
            min_cycles_for_foc: 20,
            ..Default::default()
        });

        // Progress through forced commutation
        for _ in 0..15 {
            controller.update(None, 0.0, false, 0.001);
        }

        // Should now be in Hall-driven phase
        let output = controller.update(Some(1.0), 50.0, true, 0.001);
        assert_eq!(output.phase, OpenLoopPhase::HallDriven);
    }

    #[test]
    fn test_foc_ready_detection() {
        let mut controller = OpenLoopController::new(OpenLoopConfig {
            forced_commutation_cycles: 10,
            min_cycles_for_foc: 20,
            min_speed_for_foc: 50.0,
            ..Default::default()
        });

        // Run until min cycles reached
        for _ in 0..25 {
            let output = controller.update(Some(1.0), 60.0, true, 0.001);
            if controller.execution_count >= 20 {
                assert!(output.ready_for_foc);
            }
        }
    }

    #[test]
    fn test_reverse_direction() {
        let mut controller = OpenLoopController::with_defaults();
        controller.set_reverse(true);

        let output = controller.update(None, 0.0, false, 0.001);
        assert!(output.current_rpm < 0.0);
    }

    #[test]
    fn test_reset_for_normal() {
        let mut controller = OpenLoopController::with_defaults();

        // Progress some cycles
        for _ in 0..100 {
            controller.update(None, 0.0, false, 0.001);
        }

        controller.reset_for_normal();
        assert_eq!(controller.execution_count, 0);
        assert!(!controller.is_recovery);
    }

    #[test]
    fn test_reset_for_recovery() {
        let mut controller = OpenLoopController::with_defaults();

        controller.reset_for_recovery();
        assert_eq!(controller.execution_count, 0);
        assert!(controller.is_recovery);
    }

    // Legacy SixStepController tests
    #[test]
    fn test_legacy_new_controller() {
        let controller = SixStepController::new(60.0, 600.0, 100.0, 50, 6);

        assert_eq!(controller.current_step, 0);
        assert_eq!(controller.duty_ratio, 50);
        assert_eq!(controller.pole_pairs, 6);
    }

    #[test]
    fn test_legacy_step_state() {
        // Test all 6 steps
        for step in 0..6 {
            let state = SixStepController::get_step_state(step, 100);
            assert_eq!(state.step, step);

            // Exactly one phase should be high
            let high_count = [state.duty_u, state.duty_v, state.duty_w]
                .iter()
                .filter(|&&d| d > 0)
                .count();
            assert_eq!(high_count, 1);

            // Exactly two phases should be enabled
            let enabled_count = [state.enable_u, state.enable_v, state.enable_w]
                .iter()
                .filter(|&&e| e)
                .count();
            assert_eq!(enabled_count, 2);
        }
    }
}
