//! Hall sensor adapter for STM32G4
//!
//! Wraps the TIM4-based Hall sensor interface for use with the bldc crate.

use bldc::sensors::hall::{HallConfig, HallProcessor, HallResult};
use bldc::traits::{HallStateReader, PositionSensor, SpeedSensor};

use crate::config::advance_angle::{
    BASE_ADVANCE_DEG, MAX_ADVANCE_DEG, MAX_SPEED_FOR_ADVANCE, MIN_SPEED_FOR_ADVANCE,
};

// Note: advance_angle config imports are used by HallConfig initialization
use crate::board::{calculate_speed_rpm, get_hall_state, get_snapshot};

/// Hall sensor adapter that combines TIM4 hardware with bldc processing
pub struct HallSensorAdapter {
    /// bldc Hall processor
    processor: HallProcessor,
    /// Last processed result
    last_result: HallResult,
    /// Last valid Hall state (for noise filtering)
    last_valid_hall_state: u8,
    /// Number of pole pairs (cached for speed calculation)
    pole_pairs: u8,
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
            direction_inversed: false,
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
            last_valid_hall_state: 1, // Default to a valid state
            pole_pairs,
        }
    }

    /// Update the Hall sensor and return the result
    ///
    /// Reads Hall state and timing from TIM4 hardware using atomic snapshot,
    /// then processes using the bldc HallProcessor.
    ///
    /// # Arguments
    /// * `dt` - Time since last update (seconds)
    ///
    /// # Returns
    /// Tuple of (electrical_angle, speed_rpm, hall_state)
    /// Note: Invalid Hall states (0, 7) are filtered as noise; last valid state is returned
    pub fn update(&mut self, dt: f32) -> (f32, f32, u8) {
        // Read consistent snapshot from TIM4 hardware (sequence-locked)
        let (raw_hall_state, period_cycles, is_timeout) = get_snapshot();

        // Filter noise: states 0 and 7 are invalid, use last valid state
        let hall_state = if (1..=6).contains(&raw_hall_state) {
            self.last_valid_hall_state = raw_hall_state;
            raw_hall_state
        } else {
            // Use last valid state instead of invalid noise
            self.last_valid_hall_state
        };

        // Calculate instantaneous speed from TIM4 period
        let instant_rpm = if !is_timeout && period_cycles > 0 {
            calculate_speed_rpm(period_cycles, self.processor_pole_pairs())
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
    #[allow(dead_code)]
    pub fn get_electrical_angle(&self) -> f32 {
        self.last_result.electrical_angle
    }

    /// Get the current mechanical angle in radians
    pub fn get_mechanical_angle(&self) -> f32 {
        self.last_result.mechanical_angle
    }

    /// Get the current speed in RPM
    #[allow(dead_code)]
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
    #[allow(dead_code)]
    pub fn get_electrical_offset(&self) -> f32 {
        self.processor.get_electrical_offset()
    }

    /// Set whether motor direction is inverted (calibration value)
    pub fn set_direction_inversed(&mut self, inversed: bool) {
        self.processor.set_direction_inversed(inversed);
    }

    /// Get whether motor direction is inverted
    #[allow(dead_code)]
    pub fn get_direction_inversed(&self) -> bool {
        self.processor.get_direction_inversed()
    }

    /// Enable or disable interpolation
    #[allow(dead_code)]
    pub fn set_interpolation(&mut self, enable: bool) {
        self.processor.set_interpolation(enable);
    }

    /// Enable or disable advance angle
    #[allow(dead_code)]
    pub fn set_advance_angle(&mut self, enable: bool) {
        self.processor.set_advance_angle(enable);
    }

    /// Set the filter alpha
    #[allow(dead_code)]
    pub fn set_filter_alpha(&mut self, alpha: f32) {
        self.processor.set_filter_alpha(alpha);
    }

    /// Get pole pairs
    fn processor_pole_pairs(&self) -> u8 {
        self.pole_pairs
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

impl HallStateReader for HallSensorAdapter {
    fn get_hall_state(&self) -> u8 {
        get_hall_state()
    }
}

/// Simple Hall state reader that directly reads from TIM4 hardware
///
/// This is a zero-sized type that provides HallStateReader implementation
/// without requiring a full HallSensorAdapter.
#[allow(dead_code)]
pub struct HallStateReaderAdapter;

impl HallStateReader for HallStateReaderAdapter {
    fn get_hall_state(&self) -> u8 {
        get_hall_state()
    }
}
