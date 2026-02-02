//! Calibration control mode
//!
//! Automatic detection of electrical angle offset and rotation direction.

use crate::calibration::MotorCalibration;
use crate::modulation::calculate_svpwm;
use crate::traits::{
    ControlInput, HallStateReader, PositionSensor, PwmDuty, PwmOutput, SpeedSensor,
};
use crate::transforms::inverse_park;

use super::{ModeOutput, StateTransition};

/// Calibration mode state
pub struct CalibrationMode {
    controller: MotorCalibration,
    max_duty: u16,
    v_dc: f32,
    max_voltage: f32,
}

impl CalibrationMode {
    /// Create a new calibration mode
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of motor pole pairs
    /// * `max_duty` - Maximum PWM duty value
    /// * `v_dc` - DC bus voltage
    /// * `max_voltage` - Maximum voltage output
    /// * `torque` - Calibration torque (0.0 to 1.0)
    pub fn new(pole_pairs: u8, max_duty: u16, v_dc: f32, max_voltage: f32, torque: f32) -> Self {
        let mut controller = MotorCalibration::new(pole_pairs);
        controller.set_torque(torque);
        controller.start();

        Self {
            controller,
            max_duty,
            v_dc,
            max_voltage,
        }
    }

    /// Update the calibration mode
    ///
    /// # Arguments
    /// * `hw` - Hardware abstraction providing sensor readings
    /// * `input` - Control input (not used during calibration)
    /// * `dt` - Time step in seconds (not used in current implementation)
    ///
    /// # Returns
    /// Mode output with PWM duty and possible transition
    pub fn update<H, I>(&mut self, hw: &mut H, _input: &I, _dt: f32) -> ModeOutput
    where
        H: HallStateReader + PositionSensor + SpeedSensor + PwmOutput,
        I: ControlInput,
    {
        // Get sensor readings
        let speed_rpm = hw.speed_rpm();
        let sensor_angle = hw.mechanical_angle();
        let electrical_angle_raw = hw.electrical_angle();

        // Update calibration state machine
        match self.controller.update(sensor_angle, hw) {
            Ok((electrical_angle, torque)) => {
                // Calculate voltage command from torque
                let v_cmd = torque * self.max_voltage;

                // d-axis and q-axis voltage (simple q-axis only during calibration)
                let vd_cmd = 0.0;
                let vq_cmd = v_cmd;

                // Inverse Park transform
                let (v_alpha, v_beta) = inverse_park(vd_cmd, vq_cmd, electrical_angle);

                // SVPWM calculation
                let (duty_u, duty_v, duty_w) =
                    calculate_svpwm(v_alpha, v_beta, self.v_dc, self.max_duty);

                // Check for completion
                let transition = if self.controller.is_completed() {
                    self.handle_completion()
                } else {
                    None
                };

                ModeOutput {
                    duty: PwmDuty::new(duty_u, duty_v, duty_w),
                    speed_rpm,
                    electrical_angle: electrical_angle_raw,
                    transition,
                }
            }
            Err(_) => {
                // Error during calibration: transition to OpenLoop
                ModeOutput {
                    duty: PwmDuty::default(),
                    speed_rpm,
                    electrical_angle: electrical_angle_raw,
                    transition: Some(StateTransition::ToOpenLoop { is_recovery: false }),
                }
            }
        }
    }

    /// Handle calibration completion
    fn handle_completion(&self) -> Option<StateTransition> {
        let result = self.controller.get_result();

        if result.success {
            // On success, transition to FOC
            // Note: The caller should apply calibration results to the Hall sensor
            Some(StateTransition::ToFoc {
                initial_vq: self.controller.get_torque() * self.max_voltage,
                current_rpm: 0.0, // Speed is typically low after calibration
                is_reverse: false,
            })
        } else {
            // On failure, transition to OpenLoop
            Some(StateTransition::ToOpenLoop { is_recovery: false })
        }
    }

    /// Check if calibration is completed
    pub fn is_completed(&self) -> bool {
        self.controller.is_completed()
    }

    /// Get calibration result
    pub fn get_result(&self) -> crate::calibration::CalibrationResult {
        self.controller.get_result()
    }

    /// Get reference to the underlying calibration controller
    pub fn controller(&self) -> &MotorCalibration {
        &self.controller
    }
}
