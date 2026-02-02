//! Open-loop control mode
//!
//! SVPWM-based forced commutation for motor startup,
//! transitioning to Hall-driven commutation before FOC.

use crate::control::{OpenLoopConfig, OpenLoopController, OpenLoopPhase};
use crate::traits::{ControlInput, HallStateReader, PositionSensor, PwmOutput, SpeedSensor};

use super::{ModeOutput, StateTransition};

/// Open-loop mode state
pub struct OpenLoopMode {
    controller: OpenLoopController,
    is_recovery: bool,
    max_duty: u16,
    v_dc: f32,
}

impl OpenLoopMode {
    /// Create a new open-loop mode
    pub fn new(config: OpenLoopConfig, is_recovery: bool) -> Self {
        let max_duty = config.max_duty;
        let v_dc = config.v_dc;
        let mut controller = OpenLoopController::new(config);

        if is_recovery {
            controller.reset_for_recovery();
        } else {
            controller.reset_for_normal();
        }

        Self {
            controller,
            is_recovery,
            max_duty,
            v_dc,
        }
    }

    /// Update the open-loop mode
    ///
    /// # Arguments
    /// * `hw` - Hardware abstraction providing sensor readings
    /// * `input` - Control input providing target speed
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// Mode output with PWM duty and possible transition
    pub fn update<H, I>(&mut self, hw: &mut H, input: &I, dt: f32) -> ModeOutput
    where
        H: HallStateReader + PositionSensor + SpeedSensor + PwmOutput,
        I: ControlInput,
    {
        // Get Hall state
        let hall_state = hw.get_hall_state();
        let is_valid_hall = (1..=6).contains(&hall_state);

        // Determine direction from target speed
        let target_speed = input.target_speed();
        let is_reverse = target_speed < 0.0;
        self.controller.set_reverse(is_reverse);

        // Get Hall-based electrical angle and speed
        let hall_electrical_angle = hw.electrical_angle();
        let speed_rpm = hw.speed_rpm().abs();

        // Update controller
        let output =
            self.controller
                .update(Some(hall_electrical_angle), speed_rpm, is_valid_hall, dt);

        // Check for FOC transition
        let transition = if output.ready_for_foc {
            let voltage_ratio = self.controller.get_voltage_ratio();
            let initial_vq = voltage_ratio * self.v_dc;
            let current_rpm = if is_reverse { -speed_rpm } else { speed_rpm };

            Some(StateTransition::ToFoc {
                initial_vq,
                current_rpm,
                is_reverse,
            })
        } else {
            None
        };

        ModeOutput {
            duty: output.duty,
            speed_rpm: output.current_rpm,
            electrical_angle: hall_electrical_angle,
            transition,
        }
    }

    /// Get the current phase
    pub fn phase(&self) -> OpenLoopPhase {
        self.controller.get_current_phase()
    }

    /// Get execution count
    pub fn execution_count(&self) -> u32 {
        self.controller.get_execution_count()
    }

    /// Check if in recovery mode
    pub fn is_recovery(&self) -> bool {
        self.is_recovery
    }

    /// Get max duty
    pub fn max_duty(&self) -> u16 {
        self.max_duty
    }
}
