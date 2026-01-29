//! Hall sensor processing for BLDC motor position and speed estimation
//!
//! Provides hardware-agnostic Hall sensor processing including:
//! - State validation and normalization
//! - Mechanical and electrical angle calculation
//! - Speed filtering with exponential moving average
//! - Angle interpolation between Hall edges

use crate::transforms::normalize_angle;
use core::f32::consts::{FRAC_PI_2, FRAC_PI_6, PI, TAU};

/// Hall state lookup table
/// Maps raw Hall state (1-6) to normalized index (0-5)
/// Valid transition sequence: 1 -> 3 -> 2 -> 6 -> 4 -> 5 -> 1 (CW rotation)
const HALL_STATE_TABLE: [u8; 8] = [
    255, // 0b000: Invalid state
    0,   // 0b001: State 1 -> index 0
    2,   // 0b010: State 2 -> index 2
    1,   // 0b011: State 3 -> index 1
    4,   // 0b100: State 4 -> index 4
    5,   // 0b101: State 5 -> index 5
    3,   // 0b110: State 6 -> index 3
    255, // 0b111: Invalid state
];

/// Hall state to electrical angle lookup table (radians)
/// Each Hall state corresponds to the center electrical angle + 180 degrees
const HALL_TO_ELECTRICAL_ANGLE: [f32; 8] = [
    0.0,              // 0b000: Invalid
    7.0 * FRAC_PI_6,  // 0b001: Hall 1 -> 210 deg = 7*PI/6
    11.0 * FRAC_PI_6, // 0b010: Hall 2 -> 330 deg = 11*PI/6
    3.0 * FRAC_PI_2,  // 0b011: Hall 3 -> 270 deg = 3*PI/2
    FRAC_PI_2,        // 0b100: Hall 4 -> 90 deg = PI/2
    5.0 * FRAC_PI_6,  // 0b101: Hall 5 -> 150 deg = 5*PI/6
    FRAC_PI_6,        // 0b110: Hall 6 -> 30 deg = PI/6
    0.0,              // 0b111: Invalid
];

/// Mechanical to electrical angle offset
/// Adjusts so Hall state 1 (normalized_state=0) has electrical angle of 210 deg
const MECHANICAL_TO_ELECTRICAL_OFFSET: f32 = 7.0 * PI / 6.0; // 210 deg

/// Invalid state marker
const INVALID_STATE: u8 = 255;

/// Minimum speed threshold for interpolation (RPM)
const MIN_INTERPOLATION_SPEED: f32 = 1.0;

/// Configuration for Hall sensor processing
#[derive(Debug, Clone)]
pub struct HallConfig {
    /// Number of pole pairs in the motor
    pub pole_pairs: u8,
    /// Speed filter coefficient (0.0-1.0, lower = more filtering)
    pub filter_alpha: f32,
    /// Electrical offset in radians (calibration value)
    pub electrical_offset: f32,
    /// Enable angle interpolation between Hall edges
    pub enable_interpolation: bool,
    /// Enable advance angle for improved efficiency
    pub enable_advance_angle: bool,
    /// Base advance angle in degrees
    pub base_advance_deg: f32,
    /// Maximum advance angle in degrees
    pub max_advance_deg: f32,
    /// Minimum speed for advance angle (RPM)
    pub min_speed_for_advance: f32,
    /// Maximum speed for advance angle (RPM)
    pub max_speed_for_advance: f32,
}

impl Default for HallConfig {
    fn default() -> Self {
        Self {
            pole_pairs: 6,
            filter_alpha: 0.05,
            electrical_offset: 0.0,
            enable_interpolation: true,
            enable_advance_angle: true,
            base_advance_deg: 15.0,
            max_advance_deg: 30.0,
            min_speed_for_advance: 100.0,
            max_speed_for_advance: 3000.0,
        }
    }
}

/// Result from Hall sensor processing
#[derive(Debug, Clone, Copy, Default)]
pub struct HallResult {
    /// Electrical angle in radians (0 to 2*PI)
    pub electrical_angle: f32,
    /// Mechanical angle in radians (0 to 2*PI)
    pub mechanical_angle: f32,
    /// Filtered speed in RPM
    pub speed_rpm: f32,
    /// Raw Hall state (1-6, or invalid)
    pub hall_state: u8,
    /// Whether the state is valid
    pub is_valid: bool,
    /// Whether the state changed since last update
    pub state_changed: bool,
}

/// Hall sensor processor for position and speed estimation
///
/// This is a hardware-agnostic implementation that processes Hall sensor
/// readings and calculates motor position and speed.
#[derive(Debug)]
pub struct HallProcessor {
    config: HallConfig,
    /// Previous normalized Hall state (0-5), 255 = invalid/initial
    prev_state: u8,
    /// Hall index base (increments by 6 each electrical revolution)
    hall_idx_base: u32,
    /// Maximum Hall index (pole_pairs * 6)
    hall_idx_max: u32,
    /// Angle per Hall state (mechanical angle)
    angle_per_state: f32,
    /// Current mechanical angle in radians
    mechanical_angle: f32,
    /// Filtered speed in RPM
    filtered_speed: f32,
    /// Time since last Hall edge (seconds)
    time_since_edge: f32,
    /// Whether this is the first valid reading
    is_first_reading: bool,
}

impl HallProcessor {
    /// Create a new Hall sensor processor
    ///
    /// # Arguments
    /// * `config` - Hall sensor configuration
    pub fn new(config: HallConfig) -> Self {
        let hall_idx_max = (config.pole_pairs as u32) * 6;
        let angle_per_state = TAU / (hall_idx_max as f32);

        Self {
            config,
            prev_state: INVALID_STATE,
            hall_idx_base: 0,
            hall_idx_max,
            angle_per_state,
            mechanical_angle: 0.0,
            filtered_speed: 0.0,
            time_since_edge: 0.0,
            is_first_reading: true,
        }
    }

    /// Create with default configuration for a given number of pole pairs
    pub fn with_pole_pairs(pole_pairs: u8) -> Self {
        Self::new(HallConfig {
            pole_pairs,
            ..Default::default()
        })
    }

    /// Process a Hall sensor reading
    ///
    /// # Arguments
    /// * `hall_state` - Raw Hall state (3-bit value, 1-6 are valid)
    /// * `instant_speed_rpm` - Instantaneous speed calculated from Hall edge timing (0 if timeout)
    /// * `is_timeout` - Whether a timeout occurred (no Hall edges for extended period)
    /// * `dt` - Time since last call (seconds)
    ///
    /// # Returns
    /// Hall sensor result with electrical angle, speed, etc.
    pub fn process(
        &mut self,
        hall_state: u8,
        instant_speed_rpm: f32,
        is_timeout: bool,
        dt: f32,
    ) -> HallResult {
        // Validate Hall state
        if !Self::is_valid_state(hall_state) {
            return self.handle_invalid_state(hall_state, is_timeout, dt);
        }

        // Get normalized state from lookup table
        let normalized_state = HALL_STATE_TABLE[hall_state as usize];
        if normalized_state == INVALID_STATE {
            return self.handle_invalid_state(hall_state, is_timeout, dt);
        }

        // Handle timeout
        if is_timeout && instant_speed_rpm == 0.0 {
            return self.handle_timeout(hall_state, normalized_state);
        }

        // Detect state change
        let state_changed = normalized_state != self.prev_state && !self.is_first_reading;

        if state_changed {
            self.handle_state_change(normalized_state);
        } else if self.is_first_reading {
            self.prev_state = normalized_state;
            self.is_first_reading = false;
        }

        // Update speed filter
        if instant_speed_rpm > 0.0 {
            if state_changed {
                self.update_speed_filter(instant_speed_rpm);
                self.time_since_edge = 0.0;
            } else {
                self.time_since_edge += dt;
            }
        } else {
            self.time_since_edge += dt;
        }

        // Calculate mechanical angle with interpolation
        let base_mechanical_angle = self.get_base_mechanical_angle(normalized_state);
        let mechanical_angle = self.interpolate_angle(base_mechanical_angle);
        self.mechanical_angle = mechanical_angle;

        // Calculate electrical angle
        let electrical_angle =
            self.calculate_electrical_angle(mechanical_angle, hall_state);

        HallResult {
            electrical_angle,
            mechanical_angle,
            speed_rpm: self.filtered_speed,
            hall_state,
            is_valid: true,
            state_changed,
        }
    }

    /// Check if a Hall state is valid
    pub fn is_valid_state(state: u8) -> bool {
        (1..=6).contains(&state)
    }

    /// Set the electrical offset (calibration value)
    pub fn set_electrical_offset(&mut self, offset_rad: f32) {
        self.config.electrical_offset = offset_rad;
    }

    /// Get the electrical offset
    pub fn get_electrical_offset(&self) -> f32 {
        self.config.electrical_offset
    }

    /// Reset the processor state
    pub fn reset(&mut self) {
        self.prev_state = INVALID_STATE;
        self.hall_idx_base = 0;
        self.mechanical_angle = 0.0;
        self.filtered_speed = 0.0;
        self.time_since_edge = 0.0;
        self.is_first_reading = true;
    }

    /// Reset speed filter to a specific value
    pub fn reset_speed_filter(&mut self, speed_rpm: f32) {
        self.filtered_speed = speed_rpm;
        self.time_since_edge = 0.0;
    }

    /// Enable or disable interpolation
    pub fn set_interpolation(&mut self, enable: bool) {
        self.config.enable_interpolation = enable;
    }

    /// Set the speed filter coefficient
    pub fn set_filter_alpha(&mut self, alpha: f32) {
        self.config.filter_alpha = alpha.clamp(0.0, 1.0);
    }

    // --- Private methods ---

    fn handle_invalid_state(&mut self, hall_state: u8, is_timeout: bool, dt: f32) -> HallResult {
        if is_timeout {
            self.filtered_speed = 0.0;
            self.time_since_edge = 0.0;
        } else {
            self.time_since_edge += dt;
        }

        HallResult {
            electrical_angle: self.calculate_electrical_angle_simple(),
            mechanical_angle: self.mechanical_angle,
            speed_rpm: self.filtered_speed,
            hall_state,
            is_valid: false,
            state_changed: false,
        }
    }

    fn handle_timeout(&mut self, hall_state: u8, normalized_state: u8) -> HallResult {
        self.filtered_speed = 0.0;
        self.time_since_edge = 0.0;

        let mechanical_angle = normalize_angle(self.get_base_mechanical_angle(normalized_state));
        self.mechanical_angle = mechanical_angle;

        let electrical_angle = self.calculate_electrical_angle(mechanical_angle, hall_state);

        HallResult {
            electrical_angle,
            mechanical_angle,
            speed_rpm: 0.0,
            hall_state,
            is_valid: true,
            state_changed: false,
        }
    }

    fn handle_state_change(&mut self, normalized_state: u8) {
        // Handle Hall index wrapping
        if normalized_state == 0 && self.prev_state == 5 {
            self.hall_idx_base += 6;
            if self.hall_idx_base >= self.hall_idx_max {
                self.hall_idx_base = 0;
            }
        } else if normalized_state == 5 && self.prev_state == 0 {
            if self.hall_idx_base < 6 {
                self.hall_idx_base = self.hall_idx_max - 6;
            } else {
                self.hall_idx_base -= 6;
            }
        }

        self.prev_state = normalized_state;
    }

    fn update_speed_filter(&mut self, instant_rpm: f32) {
        if instant_rpm > 0.0 {
            self.filtered_speed = self.config.filter_alpha * instant_rpm
                + (1.0 - self.config.filter_alpha) * self.filtered_speed;
        }
    }

    fn get_base_mechanical_angle(&self, normalized_state: u8) -> f32 {
        let hall_state_idx = self.hall_idx_base + (normalized_state as u32);
        (hall_state_idx as f32) * self.angle_per_state
    }

    fn interpolate_angle(&self, base_angle: f32) -> f32 {
        if !self.config.enable_interpolation
            || self.filtered_speed.abs() <= MIN_INTERPOLATION_SPEED
        {
            return base_angle;
        }

        // Calculate mechanical angular velocity (rad/s)
        let mechanical_omega = self.filtered_speed * (TAU / 60.0);

        // Interpolate angle
        let angle_increment = mechanical_omega * self.time_since_edge;
        normalize_angle(base_angle + angle_increment)
    }

    fn calculate_electrical_angle(&self, mechanical_angle: f32, hall_state: u8) -> f32 {
        let use_interpolation = self.config.enable_interpolation
            && self.filtered_speed.abs() > MIN_INTERPOLATION_SPEED;

        // Calculate base electrical angle
        let base_electrical_angle = if use_interpolation {
            mechanical_angle * (self.config.pole_pairs as f32) + MECHANICAL_TO_ELECTRICAL_OFFSET
        } else {
            HALL_TO_ELECTRICAL_ANGLE[hall_state as usize]
        };

        // Add calibration offset
        let mut electrical_angle = base_electrical_angle + self.config.electrical_offset;

        // Apply advance angle if enabled
        if self.config.enable_advance_angle
            && self.filtered_speed > self.config.min_speed_for_advance
        {
            electrical_angle += self.calculate_advance_angle();
        }

        normalize_angle(electrical_angle)
    }

    fn calculate_electrical_angle_simple(&self) -> f32 {
        normalize_angle(
            self.mechanical_angle * (self.config.pole_pairs as f32)
                + MECHANICAL_TO_ELECTRICAL_OFFSET
                + self.config.electrical_offset,
        )
    }

    fn calculate_advance_angle(&self) -> f32 {
        const DEG_TO_RAD: f32 = PI / 180.0;

        let base_advance_rad = self.config.base_advance_deg * DEG_TO_RAD;

        if self.filtered_speed <= self.config.min_speed_for_advance {
            return base_advance_rad;
        }

        let speed_ratio = ((self.filtered_speed - self.config.min_speed_for_advance)
            / (self.config.max_speed_for_advance - self.config.min_speed_for_advance))
            .clamp(0.0, 1.0);

        let additional_advance_rad =
            (self.config.max_advance_deg - self.config.base_advance_deg) * DEG_TO_RAD * speed_ratio;

        base_advance_rad + additional_advance_rad
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_states() {
        assert!(!HallProcessor::is_valid_state(0));
        assert!(HallProcessor::is_valid_state(1));
        assert!(HallProcessor::is_valid_state(6));
        assert!(!HallProcessor::is_valid_state(7));
    }

    #[test]
    fn test_hall_state_table() {
        assert_eq!(HALL_STATE_TABLE[0], 255); // Invalid
        assert_eq!(HALL_STATE_TABLE[1], 0);   // State 1 -> index 0
        assert_eq!(HALL_STATE_TABLE[2], 2);   // State 2 -> index 2
        assert_eq!(HALL_STATE_TABLE[3], 1);   // State 3 -> index 1
        assert_eq!(HALL_STATE_TABLE[4], 4);   // State 4 -> index 4
        assert_eq!(HALL_STATE_TABLE[5], 5);   // State 5 -> index 5
        assert_eq!(HALL_STATE_TABLE[6], 3);   // State 6 -> index 3
        assert_eq!(HALL_STATE_TABLE[7], 255); // Invalid
    }

    #[test]
    fn test_new_processor() {
        let processor = HallProcessor::with_pole_pairs(6);
        assert_eq!(processor.config.pole_pairs, 6);
        assert!(processor.is_first_reading);
    }

    #[test]
    fn test_process_valid_state() {
        let mut processor = HallProcessor::with_pole_pairs(6);
        let result = processor.process(1, 100.0, false, 0.001);

        assert!(result.is_valid);
        assert_eq!(result.hall_state, 1);
        assert!(!result.state_changed); // First reading doesn't count as change
    }

    #[test]
    fn test_process_invalid_state() {
        let mut processor = HallProcessor::with_pole_pairs(6);
        let result = processor.process(0, 100.0, false, 0.001);

        assert!(!result.is_valid);
        assert_eq!(result.hall_state, 0);
    }

    #[test]
    fn test_state_change_detection() {
        let mut processor = HallProcessor::with_pole_pairs(6);

        // First reading
        processor.process(1, 100.0, false, 0.001);

        // Same state - no change
        let result = processor.process(1, 100.0, false, 0.001);
        assert!(!result.state_changed);

        // Different state - change detected
        let result = processor.process(3, 100.0, false, 0.001);
        assert!(result.state_changed);
    }

    #[test]
    fn test_speed_filtering() {
        let mut processor = HallProcessor::new(HallConfig {
            filter_alpha: 0.5,
            ..Default::default()
        });

        // First reading
        processor.process(1, 100.0, false, 0.001);
        // State change triggers speed update
        processor.process(3, 100.0, false, 0.001);

        // With alpha=0.5: filtered = 0.5 * 100 + 0.5 * 0 = 50 first time
        // Second update: filtered = 0.5 * 100 + 0.5 * 50 = 75
        let result = processor.process(2, 100.0, false, 0.001);
        assert!(result.speed_rpm > 0.0);
    }

    #[test]
    fn test_timeout_handling() {
        let mut processor = HallProcessor::with_pole_pairs(6);

        // Normal reading
        processor.process(1, 100.0, false, 0.001);
        processor.process(3, 100.0, false, 0.001);

        // Timeout
        let result = processor.process(3, 0.0, true, 0.001);

        assert!(result.is_valid);
        assert!((result.speed_rpm - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_reset() {
        let mut processor = HallProcessor::with_pole_pairs(6);

        processor.process(1, 100.0, false, 0.001);
        processor.process(3, 100.0, false, 0.001);

        processor.reset();

        assert!(processor.is_first_reading);
        assert!((processor.filtered_speed - 0.0).abs() < 0.001);
        assert!((processor.mechanical_angle - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_electrical_offset() {
        let mut processor = HallProcessor::with_pole_pairs(6);

        processor.set_electrical_offset(0.5);
        assert!((processor.get_electrical_offset() - 0.5).abs() < 0.001);
    }
}
