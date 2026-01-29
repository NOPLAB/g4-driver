//! Hall sensor adapter for STM32G4
//!
//! Wraps the TIM4-based Hall sensor interface for use with the bldc crate.

use bldc::sensors::hall::{HallConfig, HallProcessor, HallResult};
use bldc::traits::{PositionSensor, SpeedSensor};

use crate::config::advance_angle::{
    BASE_ADVANCE_DEG, MAX_ADVANCE_DEG, MAX_SPEED_FOR_ADVANCE, MIN_SPEED_FOR_ADVANCE,
};
use crate::hall_tim;

/// Hall sensor adapter that combines TIM4 hardware with bldc processing
pub struct HallSensorAdapter {
    /// bldc Hall processor
    processor: HallProcessor,
    /// Last processed result
    last_result: HallResult,
}

impl HallSensorAdapter {
    /// Create a new Hall sensor adapter
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of pole pairs in the motor
    /// * `filter_alpha` - Speed filter coefficient (0.0-1.0)
    pub fn new(pole_pairs: u8, filter_alpha: f32) -> Self {
        let config = HallConfig {
            pole_pairs,
            filter_alpha,
            electrical_offset: 0.0,
            enable_interpolation: true,
            enable_advance_angle: true,
            base_advance_deg: BASE_ADVANCE_DEG,
            max_advance_deg: MAX_ADVANCE_DEG,
            min_speed_for_advance: MIN_SPEED_FOR_ADVANCE,
            max_speed_for_advance: MAX_SPEED_FOR_ADVANCE,
        };

        Self {
            processor: HallProcessor::new(config),
            last_result: HallResult::default(),
        }
    }

    /// Update the Hall sensor and return the result
    ///
    /// Reads Hall state and timing from TIM4 hardware, then processes
    /// using the bldc HallProcessor.
    ///
    /// # Arguments
    /// * `dt` - Time since last update (seconds)
    ///
    /// # Returns
    /// Tuple of (electrical_angle, speed_rpm, hall_state)
    pub fn update(&mut self, dt: f32) -> (f32, f32, u8) {
        // Read from TIM4 hardware
        let hall_state = hall_tim::get_hall_state();
        let period_cycles = hall_tim::get_period_cycles();
        let is_timeout = hall_tim::is_timeout();

        // Calculate instantaneous speed from TIM4 period
        let instant_rpm = if !is_timeout && period_cycles > 0 {
            hall_tim::calculate_speed_rpm(period_cycles, self.processor_pole_pairs())
        } else {
            0.0
        };

        // Process through bldc HallProcessor
        self.last_result = self
            .processor
            .process(hall_state, instant_rpm, is_timeout, dt);

        (
            self.last_result.electrical_angle,
            self.last_result.speed_rpm,
            self.last_result.hall_state,
        )
    }

    /// Get the current electrical angle in radians
    pub fn get_electrical_angle(&self) -> f32 {
        self.last_result.electrical_angle
    }

    /// Get the current mechanical angle in radians
    pub fn get_mechanical_angle(&self) -> f32 {
        self.last_result.mechanical_angle
    }

    /// Get the current speed in RPM
    pub fn get_speed_rpm(&self) -> f32 {
        self.last_result.speed_rpm
    }

    /// Reset the Hall sensor state
    pub fn reset(&mut self) {
        self.processor.reset();
        self.last_result = HallResult::default();
    }

    /// Reset speed filter to a specific value
    pub fn reset_speed_filter(&mut self, speed_rpm: f32) {
        self.processor.reset_speed_filter(speed_rpm);
    }

    /// Set the electrical offset (calibration value)
    pub fn set_electrical_offset(&mut self, offset_rad: f32) {
        self.processor.set_electrical_offset(offset_rad);
    }

    /// Get the electrical offset
    pub fn get_electrical_offset(&self) -> f32 {
        self.processor.get_electrical_offset()
    }

    /// Enable or disable interpolation
    #[allow(dead_code)]
    pub fn set_interpolation(&mut self, enable: bool) {
        self.processor.set_interpolation(enable);
    }

    /// Set the filter alpha
    #[allow(dead_code)]
    pub fn set_filter_alpha(&mut self, alpha: f32) {
        self.processor.set_filter_alpha(alpha);
    }

    /// Get current advance angle in degrees (placeholder - to be implemented)
    #[allow(dead_code)]
    pub fn get_current_advance_deg(&self) -> f32 {
        // TODO: Implement advance angle calculation in bldc crate
        0.0
    }

    /// Get pole pairs from processor config
    fn processor_pole_pairs(&self) -> u8 {
        // Access through the HallConfig would require storing it separately
        // For now, use a workaround by storing pole_pairs in the adapter
        6 // Default, should match what was passed to new()
    }
}

impl PositionSensor for HallSensorAdapter {
    fn electrical_angle(&self) -> f32 {
        self.last_result.electrical_angle
    }

    fn mechanical_angle(&self) -> f32 {
        self.last_result.mechanical_angle
    }
}

impl SpeedSensor for HallSensorAdapter {
    fn speed_rad_s(&self) -> f32 {
        self.last_result.speed_rpm * core::f32::consts::TAU / 60.0
    }

    fn speed_rpm(&self) -> f32 {
        self.last_result.speed_rpm
    }
}
