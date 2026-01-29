//! Motor auto-calibration module
//!
//! This module provides calibration functionality to automatically detect
//! motor electrical angle offset and rotation direction.
//!
//! Module structure:
//! - sector_sampler: Angle sampling for each Hall sector
//! - offset_calc: Electrical angle offset calculation

mod offset_calc;
mod sector_sampler;

use crate::position::ShaftPosition;
use crate::traits::HallStateReader;
use core::f32::consts::TAU;

pub use offset_calc::calculate_electrical_offset;
pub use sector_sampler::SectorSampler;

/// Calibration error
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationError {
    /// Motor did not move during calibration
    MotorNotMoving,
}

/// Calibration state
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationState {
    /// Initialization state
    Init,
    /// Detecting rotation direction (checking relationship between motor and sensor direction)
    FindDirection,
    /// Measuring angle at each Hall sector
    MeasureSectors,
    /// Returning to start position
    ReturnToStart,
    /// Calibration complete
    Completed,
}

/// Calibration result
#[derive(Debug, Clone, Copy)]
pub struct CalibrationResult {
    /// Electrical angle offset [rad] (0 to 2π)
    pub electrical_offset: f32,
    /// Sensor direction inversion flag (true: opposite to motor, false: same direction)
    pub direction_inversed: bool,
    /// Calibration success flag
    pub success: bool,
}

impl CalibrationResult {
    /// Create new calibration result (failed state)
    pub fn new() -> Self {
        Self {
            electrical_offset: 0.0,
            direction_inversed: false,
            success: false,
        }
    }
}

impl Default for CalibrationResult {
    fn default() -> Self {
        Self::new()
    }
}

/// Motor auto-calibration
pub struct MotorCalibration {
    /// Current state
    state: CalibrationState,
    /// Pole pairs
    pole_pairs: u8,
    /// Torque (0.0 to 1.0)
    torque: f32,
    /// Requested shaft position
    shaft_position_req: ShaftPosition,
    /// Actual shaft position
    shaft_position_act: ShaftPosition,
    /// Calibration result
    result: CalibrationResult,
    /// Sector sampler
    sector_sampler: SectorSampler,
}

impl MotorCalibration {
    /// Create new motor calibration
    ///
    /// # Arguments
    /// * `pole_pairs` - Motor pole pairs
    /// * `torque` - Calibration torque (0.0 to 1.0, recommended: 0.15 to 0.25)
    pub fn new(pole_pairs: u8, torque: f32) -> Self {
        Self {
            state: CalibrationState::Init,
            pole_pairs,
            torque: torque.clamp(0.1, 0.5), // Limit to 0.1 to 0.5 for safety
            shaft_position_req: ShaftPosition::new(),
            shaft_position_act: ShaftPosition::new(),
            result: CalibrationResult::new(),
            sector_sampler: SectorSampler::new(),
        }
    }

    /// Start calibration
    pub fn start(&mut self) {
        self.state = CalibrationState::Init;
        self.shaft_position_req.reset();
        self.shaft_position_act.reset();
        self.result = CalibrationResult::new();
        self.sector_sampler.reset();
    }

    /// Get current state
    #[allow(dead_code)]
    pub fn get_state(&self) -> CalibrationState {
        self.state
    }

    /// Get calibration result
    pub fn get_result(&self) -> CalibrationResult {
        self.result
    }

    /// Check if calibration is completed
    pub fn is_completed(&self) -> bool {
        self.state == CalibrationState::Completed
    }

    /// Update calibration state machine
    ///
    /// # Arguments
    /// * `sensor_angle` - Angle from sensor [rad]
    /// * `hall_reader` - Hall state reader implementation
    ///
    /// # Returns
    /// * `Ok((electrical_angle, torque))` - Electrical angle [rad] and torque (0.0 to 1.0)
    /// * `Err(CalibrationError)` - Error (motor did not move, etc.)
    pub fn update<H: HallStateReader>(
        &mut self,
        sensor_angle: f32,
        hall_reader: &H,
    ) -> Result<(f32, f32), CalibrationError> {
        // Update actual shaft position
        self.shaft_position_act.update_shaft_angle(sensor_angle);

        match self.state {
            CalibrationState::Init => self.handle_init(),
            CalibrationState::FindDirection => self.handle_find_direction(),
            CalibrationState::MeasureSectors => self.handle_measure_sectors(hall_reader),
            CalibrationState::ReturnToStart => self.handle_return_to_start(),
            CalibrationState::Completed => Ok((0.0, 0.0)),
        }
    }

    /// Set torque
    #[allow(dead_code)]
    pub fn set_torque(&mut self, torque: f32) {
        self.torque = torque.clamp(0.1, 0.5);
    }

    // === State handlers ===

    fn handle_init(&mut self) -> Result<(f32, f32), CalibrationError> {
        self.shaft_position_req.reset();
        self.shaft_position_act.reset();
        self.result.electrical_offset = 0.0;
        self.sector_sampler.reset();
        self.state = CalibrationState::FindDirection;
        Ok((0.0, 0.0))
    }

    fn handle_find_direction(&mut self) -> Result<(f32, f32), CalibrationError> {
        // Target: 1 or more rotations (1 electrical angle rotation)
        if self.shaft_position_req.rotations >= 1 {
            // Check if motor moved
            if self.shaft_position_act.rotations == 0 && self.shaft_position_act.angle < 0.1 {
                self.state = CalibrationState::Completed;
                self.result.success = false;
                return Err(CalibrationError::MotorNotMoving);
            }

            // Check rotation direction
            let actual_position = self.shaft_position_act.get_position();
            if actual_position < 0.0 {
                // Sensor is reversed
                self.shaft_position_act.set_inversed(true);
                self.result.direction_inversed = true;
            } else {
                self.shaft_position_act.set_inversed(false);
                self.result.direction_inversed = false;
            }

            self.state = CalibrationState::MeasureSectors;
        } else {
            // Slow rotation (2.5 rad/s ≈ 24 RPM)
            // At 10kHz update, per step: 2.5 / 10000 = 0.00025 rad
            self.shaft_position_req.increment(0.00025);
        }

        // Return requested position electrical angle (without offset)
        let electrical_angle = self.shaft_position_req.get_angle() * self.pole_pairs as f32;
        Ok((electrical_angle, self.torque))
    }

    fn handle_measure_sectors<H: HallStateReader>(
        &mut self,
        hall_reader: &H,
    ) -> Result<(f32, f32), CalibrationError> {
        // Get current Hall sector (1-6)
        let current_hall = hall_reader.get_hall_state();

        // Sample at valid Hall sector
        if (1..=6).contains(&current_hall) {
            let angle = self.shaft_position_act.get_angle();
            self.sector_sampler.record_sample(current_hall, angle);

            // Check if all sector angles are recorded
            if self.sector_sampler.is_complete() {
                // Calculate offset
                self.result.electrical_offset =
                    calculate_electrical_offset(self.sector_sampler.get_angles(), self.pole_pairs);
                self.state = CalibrationState::ReturnToStart;
            }
        }

        // Continue slow rotation
        self.shaft_position_req.increment(0.00025);

        let electrical_angle = self.shaft_position_req.get_angle() * self.pole_pairs as f32;
        Ok((electrical_angle, self.torque))
    }

    fn handle_return_to_start(&mut self) -> Result<(f32, f32), CalibrationError> {
        // Target: 0 rotations, angle < π/2
        if self.shaft_position_req.rotations == 0 && self.shaft_position_req.angle < TAU / 4.0 {
            self.state = CalibrationState::Completed;
            self.result.success = true;
            Ok((0.0, 0.0)) // Stop with torque 0
        } else {
            // Rotate in reverse direction (return to start position) - slowly
            // At 10kHz update, per step: 5.0 / 10000 = 0.0005 rad
            self.shaft_position_req.increment(-0.0005);

            let electrical_angle = self.shaft_position_req.get_angle() * self.pole_pairs as f32;
            Ok((electrical_angle, self.torque))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockHallReader {
        state: u8,
    }

    impl HallStateReader for MockHallReader {
        fn get_hall_state(&self) -> u8 {
            self.state
        }
    }

    #[test]
    fn test_calibration_new() {
        let cal = MotorCalibration::new(6, 0.2);
        assert_eq!(cal.get_state(), CalibrationState::Init);
        assert!(!cal.is_completed());
    }

    #[test]
    fn test_calibration_start() {
        let mut cal = MotorCalibration::new(6, 0.2);
        cal.start();
        assert_eq!(cal.get_state(), CalibrationState::Init);
        assert!(!cal.get_result().success);
    }

    #[test]
    fn test_torque_clamping() {
        let mut cal = MotorCalibration::new(6, 0.8); // 0.8 is too high
        assert!(cal.torque <= 0.5);

        cal.set_torque(0.05); // 0.05 is too low
        assert!(cal.torque >= 0.1);
    }

    #[test]
    fn test_init_transitions_to_find_direction() {
        let mut cal = MotorCalibration::new(6, 0.2);
        let hall_reader = MockHallReader { state: 1 };
        cal.start();

        // First update should transition from Init to FindDirection
        let _ = cal.update(0.0, &hall_reader);
        assert_eq!(cal.get_state(), CalibrationState::FindDirection);
    }
}
