//! FOC (Field Oriented Control) mode
//!
//! Hall sensor-based closed-loop speed control using field oriented control.

use crate::compensation::{DeadTimeCompensation, FluxWeakeningController};
use crate::control::stall_detector::StallDetectorConfig;
use crate::control::{FocConfig, FocController};
use crate::traits::{
    ControlInput, HallStateReader, PositionSensor, PwmDuty, PwmOutput, SpeedSensor,
};

use super::{ModeOutput, StateTransition};

/// FOC mode state
pub struct FocMode {
    controller: FocController,
    invalid_hall_count: u32,
    invalid_hall_threshold: u32,
    max_duty: u16,
    last_kp: f32,
    last_ki: f32,
}

/// Builder for FocMode with optional components
pub struct FocModeBuilder {
    config: FocConfig,
    initial_vq: f32,
    current_rpm: f32,
    invalid_hall_threshold: u32,
    dead_time_comp: Option<DeadTimeCompensation>,
    flux_weakening: Option<FluxWeakeningController>,
    stall_config: Option<StallDetectorConfig>,
}

impl FocModeBuilder {
    /// Create a new builder
    pub fn new(config: FocConfig) -> Self {
        Self {
            config,
            initial_vq: 0.0,
            current_rpm: 0.0,
            invalid_hall_threshold: 100,
            dead_time_comp: None,
            flux_weakening: None,
            stall_config: None,
        }
    }

    /// Set initial Vq for smooth transition
    pub fn with_initial_vq(mut self, vq: f32) -> Self {
        self.initial_vq = vq;
        self
    }

    /// Set current RPM for ramp initialization
    pub fn with_current_rpm(mut self, rpm: f32) -> Self {
        self.current_rpm = rpm;
        self
    }

    /// Set invalid Hall threshold
    pub fn with_invalid_hall_threshold(mut self, threshold: u32) -> Self {
        self.invalid_hall_threshold = threshold;
        self
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

    /// Build the FocMode
    pub fn build(self) -> FocMode {
        let max_duty = self.config.max_duty;
        let kp = self.config.speed_kp;
        let ki = self.config.speed_ki;

        let mut builder = FocController::builder(self.config);

        if let Some(comp) = self.dead_time_comp {
            builder = builder.with_dead_time_compensation(comp);
        }
        if let Some(fw) = self.flux_weakening {
            builder = builder.with_flux_weakening(fw);
        }
        if let Some(stall) = self.stall_config {
            builder = builder.with_stall_detection(stall);
        }

        let mut controller = builder.build();
        controller.initialize_for_foc(self.current_rpm, self.initial_vq);

        FocMode {
            controller,
            invalid_hall_count: 0,
            invalid_hall_threshold: self.invalid_hall_threshold,
            max_duty,
            last_kp: kp,
            last_ki: ki,
        }
    }
}

impl FocMode {
    /// Create a new FOC mode with basic configuration
    pub fn new(config: FocConfig, initial_vq: f32, current_rpm: f32) -> Self {
        FocModeBuilder::new(config)
            .with_initial_vq(initial_vq)
            .with_current_rpm(current_rpm)
            .build()
    }

    /// Create a builder for FOC mode
    pub fn builder(config: FocConfig) -> FocModeBuilder {
        FocModeBuilder::new(config)
    }

    /// Update the FOC mode
    ///
    /// # Arguments
    /// * `hw` - Hardware abstraction providing sensor readings
    /// * `input` - Control input providing target speed and PI gains
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// Mode output with PWM duty and possible transition
    pub fn update<H, I>(&mut self, hw: &mut H, input: &I, dt: f32) -> ModeOutput
    where
        H: HallStateReader + PositionSensor + SpeedSensor + PwmOutput,
        I: ControlInput,
    {
        // Get Hall state and sensor readings
        let hall_state = hw.get_hall_state();
        let is_valid_hall = (1..=6).contains(&hall_state);
        let hall_electrical_angle = hw.electrical_angle();
        let speed_rpm = hw.speed_rpm();

        // Handle invalid Hall state
        if !is_valid_hall {
            self.invalid_hall_count += 1;

            if self.invalid_hall_count >= self.invalid_hall_threshold {
                // Long-term invalid: neutralize PWM and reset controller
                let neutral = self.max_duty / 2;
                self.controller.reset();

                return ModeOutput {
                    duty: PwmDuty::new(neutral, neutral, neutral),
                    speed_rpm,
                    electrical_angle: hall_electrical_angle,
                    transition: None,
                };
            }

            // Short-term invalid: hold last output
            return ModeOutput {
                duty: PwmDuty::default(),
                speed_rpm,
                electrical_angle: hall_electrical_angle,
                transition: None,
            };
        }

        // Valid Hall state: reset invalid counter
        self.invalid_hall_count = 0;

        // Check for PI gain updates
        let (kp, ki) = input.pi_gains();
        if (kp - self.last_kp).abs() > f32::EPSILON || (ki - self.last_ki).abs() > f32::EPSILON {
            self.controller.set_gains(kp, ki);
            self.last_kp = kp;
            self.last_ki = ki;
        }

        // Set target speed
        let target_speed = input.target_speed();
        self.controller.set_target_speed_rpm(target_speed);

        // Adjust speed ramp to prevent reverse torque during acceleration
        if let Some(ramp) = self.controller.speed_ramp_mut() {
            let current_ramp = ramp.get_current_speed();

            let should_catch_up =
                (target_speed > 0.0 && current_ramp >= 0.0 && current_ramp < speed_rpm)
                    || (target_speed < 0.0 && current_ramp <= 0.0 && current_ramp > speed_rpm);

            if should_catch_up {
                ramp.set_current_speed(speed_rpm);
            }
        }

        // Run FOC controller
        let output = self
            .controller
            .update_extended(speed_rpm, hall_electrical_angle, dt);

        // Check for stall transition
        let transition = if output.is_stalled {
            Some(StateTransition::ToOpenLoop { is_recovery: true })
        } else {
            None
        };

        ModeOutput {
            duty: output.duty,
            speed_rpm,
            electrical_angle: hall_electrical_angle,
            transition,
        }
    }

    /// Get Vq output
    pub fn get_vq(&self) -> f32 {
        // Note: This would require storing last output or adding getter to FocController
        0.0
    }

    /// Reset the controller
    pub fn reset(&mut self) {
        self.controller.reset();
        self.invalid_hall_count = 0;
    }

    /// Get reference to the underlying FOC controller
    pub fn controller(&self) -> &FocController {
        &self.controller
    }

    /// Get mutable reference to the underlying FOC controller
    pub fn controller_mut(&mut self) -> &mut FocController {
        &mut self.controller
    }
}
