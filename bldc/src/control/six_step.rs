//! Six-step (trapezoidal) commutation for BLDC motors
//!
//! This module provides open-loop six-step driving for motor startup.
//! Six-step commutation is simpler than FOC and useful for starting
//! motors before transitioning to closed-loop FOC control.

/// State information for six-step driving
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

/// Open-loop six-step commutation controller
///
/// Drives the motor using six-step (trapezoidal) commutation,
/// which is useful for motor startup before transitioning to FOC.
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
        }
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
    fn test_new_controller() {
        let controller = SixStepController::new(60.0, 600.0, 100.0, 50, 6);

        assert_eq!(controller.current_step, 0);
        assert_eq!(controller.duty_ratio, 50);
        assert_eq!(controller.pole_pairs, 6);
    }

    #[test]
    fn test_step_state() {
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

    #[test]
    fn test_step_progression() {
        let mut controller = SixStepController::new(600.0, 600.0, 0.0, 50, 6);

        // With a very short step period, updates should progress steps
        let initial_step = controller.current_step;

        // Simulate several updates (step period for 600 RPM, 6 pole pairs = ~2.78ms per step)
        for _ in 0..1000 {
            controller.update(0.001); // 1ms per update
        }

        // After many updates, step should have changed
        assert_ne!(controller.current_step, initial_step);
    }

    #[test]
    fn test_speed_calculation() {
        let controller = SixStepController::new(600.0, 1200.0, 100.0, 50, 6);

        let rpm = controller.get_current_rpm();
        // Should start close to initial RPM
        assert!((rpm - 600.0).abs() < 1.0);
    }

    #[test]
    fn test_reset() {
        let mut controller = SixStepController::new(60.0, 600.0, 100.0, 50, 6);

        // Advance some steps
        for _ in 0..100 {
            controller.update(0.01);
        }

        controller.reset();

        assert_eq!(controller.current_step, 0);
        assert!((controller.elapsed_time - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_acceleration() {
        let mut controller = SixStepController::new(60.0, 600.0, 200.0, 50, 6);

        let initial_period = controller.step_period;

        // Run for a while to trigger acceleration
        for _ in 0..1000 {
            controller.update(0.01);
        }

        // Period should decrease (speed should increase)
        assert!(controller.step_period < initial_period);
    }
}
