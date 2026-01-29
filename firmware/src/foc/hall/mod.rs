// Hall sensor processing for BLDC motor position and speed estimation
// Uses TIM4 hardware Hall interface for high-precision edge detection and speed calculation
// Implements foc-simple compatible mechanical angle based calculation
//
// Module structure:
// - constants: Hall state tables and angle mappings
// - state_machine: Hall state transition logic
// - speed_filter: Low-pass filter for speed smoothing
// - interpolator: Angle interpolation between Hall edges
// - angle: Electrical angle calculation with advance angle
// - utils: Shared utility functions

mod angle;
mod constants;
mod interpolator;
mod speed_filter;
mod state_machine;
mod utils;

// Re-export constants for external use (e.g., calibration)
#[allow(unused_imports)]
pub use constants::{HALL_STATE_TABLE, HALL_TO_ELECTRICAL_ANGLE, MECHANICAL_TO_ELECTRICAL_OFFSET};
use utils::normalize_angle;

use crate::hall_tim;
use angle::ElectricalAngle;
use interpolator::AngleInterpolator;
use speed_filter::SpeedFilter;
use state_machine::HallStateMachine;

/// Hall sensor state machine for position and speed estimation
/// Implements foc-simple compatible mechanical angle based calculation
/// Relies on hall_tim (TIM4 hardware) for edge detection and speed calculation
pub struct HallSensor {
    /// State machine for Hall state transitions
    state_machine: HallStateMachine,
    /// Low-pass filter for speed smoothing
    speed_filter: SpeedFilter,
    /// Angle interpolator between Hall edges
    interpolator: AngleInterpolator,
    /// Electrical angle calculator
    electrical_angle: ElectricalAngle,
    /// Number of pole pairs
    pole_pairs: u8,
}

impl HallSensor {
    /// Create a new Hall sensor instance
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of pole pairs in the motor
    /// * `speed_filter_alpha` - Low-pass filter coefficient (0.0-1.0, foc-simple uses 0.05)
    pub fn new(pole_pairs: u8, speed_filter_alpha: f32) -> Self {
        Self {
            state_machine: HallStateMachine::new(pole_pairs),
            speed_filter: SpeedFilter::new(speed_filter_alpha),
            interpolator: AngleInterpolator::new(),
            electrical_angle: ElectricalAngle::new(pole_pairs),
            pole_pairs,
        }
    }

    /// Check if a hall state is valid
    ///
    /// # Arguments
    /// * `state` - Hall state (0-7)
    ///
    /// # Returns
    /// `true` if state is valid (1-6), `false` otherwise
    #[allow(dead_code)]
    pub fn is_valid_state(state: u8) -> bool {
        HallStateMachine::is_valid_state(state)
    }

    /// Update hall sensor state and estimate position/speed
    /// Uses foc-simple compatible mechanical angle based calculation
    /// Uses TIM4 hardware for both speed calculation and Hall state reading
    ///
    /// # Arguments
    /// * `dt` - Time step since last update (seconds) - used for angle interpolation
    ///
    /// # Returns
    /// Tuple of (electrical_angle in radians, speed in RPM, raw_hall_state)
    /// raw_hall_state is included to avoid re-reading Hall state after this call
    pub fn update(&mut self, dt: f32) -> (f32, f32, u8) {
        // Get Hall state and period from TIM4 (read once for consistency)
        let raw_hall_state = hall_tim::get_hall_state();
        let period_cycles = hall_tim::get_period_cycles();
        let is_timeout = hall_tim::is_timeout();

        // Process Hall state through state machine
        let state_result = self.state_machine.process_state(raw_hall_state);

        // Handle invalid Hall state
        if state_result.is_none() {
            return self.handle_invalid_state(dt, is_timeout, period_cycles, raw_hall_state);
        }

        let state = state_result.unwrap();

        // Handle timeout (1秒以上Hallエッジがない場合のみ速度を0に)
        if is_timeout && period_cycles == 0 {
            return self.handle_timeout(state.normalized_state, raw_hall_state);
        }

        // Handle period_cycles == 0 (maintain previous speed, continue interpolation)
        if period_cycles == 0 {
            return self.handle_zero_period(dt, state.normalized_state, raw_hall_state);
        }

        // Calculate instant speed from TIM4 period
        let instant_rpm = hall_tim::calculate_speed_rpm(period_cycles, self.pole_pairs);

        // Update speed filter and interpolator based on state change
        if state.state_changed {
            // Apply low-pass filter to speed
            self.speed_filter.update(instant_rpm);
            // Reset edge timer
            self.interpolator.reset_time();
        } else {
            // Accumulate time since last edge
            self.interpolator.accumulate_time(dt);

            // Initialize speed on first reading
            if state.is_first_reading && instant_rpm > 0.0 {
                self.speed_filter.initialize(instant_rpm);
            }
        }

        // Calculate mechanical angle with optional interpolation
        let speed_rpm = self.speed_filter.get_speed();
        let base_mechanical_angle = self
            .state_machine
            .get_base_mechanical_angle(state.normalized_state);
        let mechanical_angle = self
            .interpolator
            .interpolate(base_mechanical_angle, speed_rpm);
        self.state_machine.set_mechanical_angle(mechanical_angle);

        // Calculate electrical angle with optional advance
        let use_interpolation = self.interpolator.should_interpolate(speed_rpm);
        let electrical_angle = self.electrical_angle.calculate_with_advance(
            mechanical_angle,
            speed_rpm,
            use_interpolation,
            raw_hall_state,
        );

        (electrical_angle, speed_rpm, raw_hall_state)
    }

    /// Handle invalid Hall state reading
    fn handle_invalid_state(
        &mut self,
        dt: f32,
        is_timeout: bool,
        period_cycles: u32,
        raw_hall_state: u8,
    ) -> (f32, f32, u8) {
        if is_timeout && period_cycles == 0 {
            self.speed_filter.reset();
            self.interpolator.reset_time();
        } else {
            self.interpolator.accumulate_time(dt);
        }

        let electrical_angle = self
            .electrical_angle
            .to_electrical(self.state_machine.get_mechanical_angle());

        (
            electrical_angle,
            self.speed_filter.get_speed(),
            raw_hall_state,
        )
    }

    /// Handle timeout condition (no Hall edges for >1 second)
    fn handle_timeout(&mut self, normalized_state: u8, raw_hall_state: u8) -> (f32, f32, u8) {
        self.speed_filter.reset();
        self.interpolator.reset_time();

        // Use discrete mechanical angle (no interpolation)
        let mechanical_angle = self
            .state_machine
            .get_base_mechanical_angle(normalized_state);
        self.state_machine
            .set_mechanical_angle(normalize_angle(mechanical_angle));

        let electrical_angle = self.electrical_angle.to_electrical(mechanical_angle);

        (electrical_angle, 0.0, raw_hall_state)
    }

    /// Handle zero period cycles (maintain previous speed)
    fn handle_zero_period(
        &mut self,
        dt: f32,
        normalized_state: u8,
        raw_hall_state: u8,
    ) -> (f32, f32, u8) {
        self.interpolator.accumulate_time(dt);

        let speed_rpm = self.speed_filter.get_speed();
        let base_mechanical_angle = self
            .state_machine
            .get_base_mechanical_angle(normalized_state);
        let mechanical_angle = self
            .interpolator
            .interpolate(base_mechanical_angle, speed_rpm);
        self.state_machine.set_mechanical_angle(mechanical_angle);

        let use_interpolation = self.interpolator.should_interpolate(speed_rpm);
        let electrical_angle = self.electrical_angle.calculate_with_advance(
            mechanical_angle,
            speed_rpm,
            use_interpolation,
            raw_hall_state,
        );

        (electrical_angle, speed_rpm, raw_hall_state)
    }

    /// Get current electrical angle in radians
    #[allow(dead_code)]
    pub fn get_electrical_angle(&self) -> f32 {
        self.electrical_angle
            .to_electrical(self.state_machine.get_mechanical_angle())
    }

    /// Get current mechanical angle in radians
    pub fn get_mechanical_angle(&self) -> f32 {
        self.state_machine.get_mechanical_angle()
    }

    /// Get current speed in RPM
    #[allow(dead_code)]
    pub fn get_speed_rpm(&self) -> f32 {
        self.speed_filter.get_speed()
    }

    /// Reset the hall sensor state
    pub fn reset(&mut self) {
        self.state_machine.reset();
        self.speed_filter.reset();
        self.interpolator.reset();
    }

    /// Reset speed filter and interpolation timer to a specific speed value
    /// This is useful when transitioning from OpenLoop to FOC mode to avoid
    /// transient effects from the low-pass filter
    ///
    /// # Arguments
    /// * `new_speed` - Speed value to set in RPM
    pub fn reset_speed_filter(&mut self, new_speed: f32) {
        self.speed_filter.initialize(new_speed);
        self.interpolator.reset_time();
    }

    /// Enable or disable angle interpolation
    ///
    /// # Arguments
    /// * `enable` - True to enable interpolation, false for discrete Hall angles only
    #[allow(dead_code)]
    pub fn set_interpolation(&mut self, enable: bool) {
        self.interpolator.set_enabled(enable);
    }

    /// Check if interpolation is enabled
    #[allow(dead_code)]
    pub fn is_interpolation_enabled(&self) -> bool {
        self.interpolator.is_enabled()
    }

    /// Set the speed filter coefficient
    ///
    /// # Arguments
    /// * `alpha` - Filter coefficient (0.0-1.0)
    ///   - Lower values = more filtering (smoother but slower response)
    ///   - Higher values = less filtering (faster but noisier)
    #[allow(dead_code)]
    pub fn set_filter_alpha(&mut self, alpha: f32) {
        self.speed_filter.set_alpha(alpha);
    }

    /// Set the electrical offset (calibration value)
    ///
    /// # Arguments
    /// * `offset_rad` - Electrical offset in radians
    ///
    /// This is used to calibrate the motor. The electrical offset is the difference
    /// between the Hall sensor zero position and the motor's magnetic zero position.
    #[allow(dead_code)]
    pub fn set_electrical_offset(&mut self, offset_rad: f32) {
        self.electrical_angle.set_electrical_offset(offset_rad);
    }

    /// Get the electrical offset
    #[allow(dead_code)]
    pub fn get_electrical_offset(&self) -> f32 {
        self.electrical_angle.get_electrical_offset()
    }

    /// Enable or disable advance angle
    ///
    /// # Arguments
    /// * `enable` - True to enable advance angle, false to disable
    #[allow(dead_code)]
    pub fn set_advance_angle(&mut self, enable: bool) {
        self.electrical_angle.set_advance_angle(enable);
    }

    /// Check if advance angle is enabled
    #[allow(dead_code)]
    pub fn is_advance_angle_enabled(&self) -> bool {
        self.electrical_angle.is_advance_angle_enabled()
    }

    /// Get current advance angle in degrees for the current speed
    #[allow(dead_code)]
    pub fn get_current_advance_deg(&self) -> f32 {
        self.electrical_angle
            .get_advance_deg_for_speed(self.speed_filter.get_speed())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_states() {
        assert!(!HallSensor::is_valid_state(0));
        assert!(HallSensor::is_valid_state(1));
        assert!(HallSensor::is_valid_state(6));
        assert!(!HallSensor::is_valid_state(7));
    }

    #[test]
    fn test_hall_state_table() {
        // Test state mapping (foc-simple compatible)
        assert_eq!(HALL_STATE_TABLE[0], 255); // Invalid
        assert_eq!(HALL_STATE_TABLE[1], 0); // State 1 -> index 0
        assert_eq!(HALL_STATE_TABLE[2], 2); // State 2 -> index 2
        assert_eq!(HALL_STATE_TABLE[3], 1); // State 3 -> index 1
        assert_eq!(HALL_STATE_TABLE[4], 4); // State 4 -> index 4
        assert_eq!(HALL_STATE_TABLE[5], 5); // State 5 -> index 5
        assert_eq!(HALL_STATE_TABLE[6], 3); // State 6 -> index 3
        assert_eq!(HALL_STATE_TABLE[7], 255); // Invalid
    }

    #[test]
    fn test_angle_calculation() {
        use core::f32::consts::TAU;

        // For pole_pairs = 6, hall_idx_max = 36
        // angle_per_state = TAU / 36 = 0.174533 rad (10 degrees)
        let pole_pairs = 6;
        let hall_idx_max = (pole_pairs as u32) * 6; // 36
        let angle_per_state = TAU / (hall_idx_max as f32);

        // Expected: ~0.174533 rad per state (10 degrees mechanical)
        let expected_deg = 360.0 / 36.0; // 10 degrees
        let expected_rad = expected_deg * core::f32::consts::PI / 180.0;

        assert!((angle_per_state - expected_rad).abs() < 0.001);
    }

    #[test]
    fn test_electrical_angle_calculation() {
        // Test electrical angle = mechanical_angle * pole_pairs + offset
        let sensor = HallSensor::new(6, 0.05);

        // With zero mechanical angle
        // electrical_angle = 0 * 6 + MECHANICAL_TO_ELECTRICAL_OFFSET + 0
        let expected = normalize_angle(MECHANICAL_TO_ELECTRICAL_OFFSET);
        assert!((sensor.get_electrical_angle() - expected).abs() < 0.001);
    }
}
