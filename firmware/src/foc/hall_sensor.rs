// Hall sensor processing for BLDC motor position and speed estimation
// Uses TIM4 hardware Hall interface for high-precision edge detection and speed calculation
// Implements foc-simple compatible mechanical angle based calculation

use crate::fmt::*;
use crate::hall_tim;
use core::f32::consts::TAU;

/// Hall state lookup table (foc-simple compatible)
/// Maps raw hall state (1-6) to normalized index (0-5)
/// Valid transition sequence: 1 -> 3 -> 2 -> 6 -> 4 -> 5 -> 1 (CW rotation)
/// Index mapping: [invalid, 0, 2, 1, 4, 5, 3, invalid]
/// Raw state:      [0,       1, 2, 3, 4, 5, 6, 7]
const HALL_STATE_TABLE: [u8; 8] = [
    255, // 0b000: Invalid state (use 255 as marker)
    0,   // 0b001: State 1 -> index 0
    2,   // 0b010: State 2 -> index 2
    1,   // 0b011: State 3 -> index 1
    4,   // 0b100: State 4 -> index 4
    5,   // 0b101: State 5 -> index 5
    3,   // 0b110: State 6 -> index 3
    255, // 0b111: Invalid state (use 255 as marker)
];

/// Hall sensor state machine for position and speed estimation
/// Implements foc-simple compatible mechanical angle based calculation
/// Relies on hall_tim (TIM4 hardware) for edge detection and speed calculation
pub struct HallSensor {
    /// Previous normalized hall state (0-5)
    prev_state: u8,
    /// Current mechanical angle in radians (shaft angle)
    mechanical_angle: f32,
    /// Hall index base (increments by 6 each electrical revolution)
    hall_idx_base: u32,
    /// Maximum hall index (pole_pairs * 6)
    hall_idx_max: u32,
    /// Angle per hall state (mechanical angle) = TAU / hall_idx_max
    angle_per_state: f32,
    /// Current speed in RPM (from TIM4)
    speed_rpm: f32,
    /// Time since last edge (for interpolation)
    time_since_edge: f32,
    /// Low-pass filter coefficient for speed (0.0 - 1.0)
    /// Lower value = more filtering
    speed_filter_alpha: f32,
    /// Number of pole pairs
    pole_pairs: u8,
    /// Enable angle interpolation between Hall edges
    enable_interpolation: bool,
    /// Electrical offset in radians (calibration value)
    electrical_offset: f32,
}

impl HallSensor {
    /// Create a new Hall sensor instance
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of pole pairs in the motor
    /// * `speed_filter_alpha` - Low-pass filter coefficient (0.0-1.0, foc-simple uses 0.05)
    pub fn new(pole_pairs: u8, speed_filter_alpha: f32) -> Self {
        let hall_idx_max = (pole_pairs as u32) * 6;
        let angle_per_state = TAU / (hall_idx_max as f32);

        Self {
            prev_state: 255, // Invalid initial state
            mechanical_angle: 0.0,
            hall_idx_base: 0,
            hall_idx_max,
            angle_per_state,
            speed_rpm: 0.0,
            time_since_edge: 0.0,
            speed_filter_alpha: speed_filter_alpha.clamp(0.0, 1.0),
            pole_pairs,
            enable_interpolation: true, // Enable angle interpolation by default
            electrical_offset: 0.0,
        }
    }

    /// Check if a hall state is valid
    ///
    /// # Arguments
    /// * `state` - Hall state (0-7)
    ///
    /// # Returns
    /// `true` if state is valid (1-6), `false` otherwise
    pub fn is_valid_state(state: u8) -> bool {
        (1..=6).contains(&state)
    }

    /// Update hall sensor state and estimate position/speed
    /// Uses foc-simple compatible mechanical angle based calculation
    /// Uses TIM4 hardware for both speed calculation and Hall state reading
    ///
    /// # Arguments
    /// * `dt` - Time step since last update (seconds) - used for angle interpolation
    ///
    /// # Returns
    /// Tuple of (electrical_angle in radians, speed in RPM)
    pub fn update(&mut self, dt: f32) -> (f32, f32) {
        // Get Hall state from TIM4 interrupt handler (captured on edge)
        let raw_hall_state = hall_tim::get_hall_state();

        // Validate hall state (throttle error logging to avoid flooding)
        if !Self::is_valid_state(raw_hall_state) {
            static mut ERROR_LOG_COUNTER: u32 = 0;
            unsafe {
                ERROR_LOG_COUNTER += 1;
                // Log error only once every 2500 calls (1 second at 2.5kHz)
                if ERROR_LOG_COUNTER >= 2500 {
                    ERROR_LOG_COUNTER = 0;
                    error!(
                        "Invalid hall state: {} (repeated, throttling log)",
                        raw_hall_state
                    );
                }
            }

            // Check timeout from TIM4
            if hall_tim::is_timeout() {
                self.speed_rpm = 0.0;
                self.time_since_edge = 0.0;
            } else {
                self.time_since_edge += dt;
            }

            // Calculate electrical angle from mechanical angle
            let electrical_angle =
                self.mechanical_angle * (self.pole_pairs as f32) - self.electrical_offset;
            return (electrical_angle, self.speed_rpm);
        }

        // Convert raw hall state to normalized index using lookup table (foc-simple compatible)
        let normalized_state = HALL_STATE_TABLE[raw_hall_state as usize];
        if normalized_state == 255 {
            // Invalid state (should not happen after is_valid_state check, but safety check)
            let electrical_angle =
                self.mechanical_angle * (self.pole_pairs as f32) - self.electrical_offset;
            return (electrical_angle, self.speed_rpm);
        }

        // Get period from TIM4 and calculate instant speed
        let period_cycles = hall_tim::get_period_cycles();

        // Check for timeout
        if hall_tim::is_timeout() || period_cycles == 0 {
            self.speed_rpm = 0.0;
            self.time_since_edge = 0.0;

            // Calculate mechanical angle from hall_idx (discrete, no interpolation)
            let hall_state_idx = self.hall_idx_base + (normalized_state as u32);
            self.mechanical_angle = (hall_state_idx as f32) * self.angle_per_state;

            // Normalize mechanical angle to [0, TAU)
            while self.mechanical_angle >= TAU {
                self.mechanical_angle -= TAU;
            }

            // Calculate electrical angle: mechanical_angle * pole_pairs - offset
            let electrical_angle =
                self.mechanical_angle * (self.pole_pairs as f32) - self.electrical_offset;

            return (electrical_angle, self.speed_rpm);
        }

        // Calculate instant speed from TIM4 period
        let instant_rpm = hall_tim::calculate_speed_rpm(period_cycles, self.pole_pairs);

        // Detect state change (hall edge)
        let state_changed = normalized_state != self.prev_state && self.prev_state != 255;

        if state_changed {
            // Handle hall index wrapping (foc-simple compatible)
            // State 0 after state 5 means we completed an electrical revolution
            if normalized_state == 0 && self.prev_state == 5 {
                self.hall_idx_base += 6;
                if self.hall_idx_base >= self.hall_idx_max {
                    self.hall_idx_base = 0; // Wrap around after full mechanical revolution
                }
            }
            // State 5 after state 0 means we're going backwards
            else if normalized_state == 5 && self.prev_state == 0 {
                if self.hall_idx_base < 6 {
                    self.hall_idx_base = self.hall_idx_max - 6;
                } else {
                    self.hall_idx_base -= 6;
                }
            }

            // Apply low-pass filter to speed (foc-simple formula: new = (instant + 19*old)/20 for alpha=0.05)
            // Equivalent to: new = alpha*instant + (1-alpha)*old where alpha = 1/20 = 0.05
            self.speed_rpm = self.speed_filter_alpha * instant_rpm
                + (1.0 - self.speed_filter_alpha) * self.speed_rpm;

            trace!(
                "Hall edge: {} -> {} (normalized: {} -> {}), period={} cycles, instant_rpm={}, filtered_rpm={}",
                self.prev_state,
                normalized_state,
                self.prev_state,
                normalized_state,
                period_cycles,
                instant_rpm,
                self.speed_rpm
            );

            // Reset edge timer
            self.time_since_edge = 0.0;

            // Update previous state
            self.prev_state = normalized_state;
        } else {
            // Accumulate time since last edge
            self.time_since_edge += dt;

            // Update filtered speed even without edge (for smoother response)
            self.speed_rpm = self.speed_filter_alpha * instant_rpm
                + (1.0 - self.speed_filter_alpha) * self.speed_rpm;
        }

        // Calculate mechanical angle from hall index (foc-simple method)
        let hall_state_idx = self.hall_idx_base + (normalized_state as u32);
        let base_mechanical_angle = (hall_state_idx as f32) * self.angle_per_state;

        // Apply angle interpolation if enabled and motor is moving
        if self.enable_interpolation && self.speed_rpm.abs() > 1.0 {
            // Calculate mechanical angular velocity (rad/s)
            let mechanical_omega = self.speed_rpm * (TAU / 60.0); // RPM to rad/s (2*PI/60)

            // Interpolate angle based on time since last edge
            let angle_increment = mechanical_omega * self.time_since_edge;
            self.mechanical_angle = base_mechanical_angle + angle_increment;
        } else {
            // No interpolation or very low speed: use discrete Hall sensor angle
            self.mechanical_angle = base_mechanical_angle;
        }

        // Normalize mechanical angle to [0, TAU)
        while self.mechanical_angle >= TAU {
            self.mechanical_angle -= TAU;
        }
        while self.mechanical_angle < 0.0 {
            self.mechanical_angle += TAU;
        }

        // Calculate electrical angle: mechanical_angle * pole_pairs - offset (foc-simple formula)
        let electrical_angle =
            self.mechanical_angle * (self.pole_pairs as f32) - self.electrical_offset;

        (electrical_angle, self.speed_rpm)
    }

    /// Get current electrical angle in radians
    #[allow(dead_code)]
    pub fn get_electrical_angle(&self) -> f32 {
        self.mechanical_angle * (self.pole_pairs as f32) - self.electrical_offset
    }

    /// Get current mechanical angle in radians
    pub fn get_mechanical_angle(&self) -> f32 {
        self.mechanical_angle
    }

    /// Get current speed in RPM
    #[allow(dead_code)]
    pub fn get_speed_rpm(&self) -> f32 {
        self.speed_rpm
    }

    /// Reset the hall sensor state
    pub fn reset(&mut self) {
        self.prev_state = 255; // Invalid state
        self.mechanical_angle = 0.0;
        self.hall_idx_base = 0;
        self.speed_rpm = 0.0;
        self.time_since_edge = 0.0;
    }

    /// Reset speed filter and interpolation timer to a specific speed value
    /// This is useful when transitioning from OpenLoop to FOC mode to avoid
    /// transient effects from the low-pass filter
    ///
    /// # Arguments
    /// * `new_speed` - Speed value to set in RPM
    pub fn reset_speed_filter(&mut self, new_speed: f32) {
        self.speed_rpm = new_speed;
        self.time_since_edge = 0.0;
    }

    /// Enable or disable angle interpolation
    ///
    /// # Arguments
    /// * `enable` - True to enable interpolation, false for discrete Hall angles only
    #[allow(dead_code)]
    pub fn set_interpolation(&mut self, enable: bool) {
        self.enable_interpolation = enable;
    }

    /// Check if interpolation is enabled
    #[allow(dead_code)]
    pub fn is_interpolation_enabled(&self) -> bool {
        self.enable_interpolation
    }

    /// Set the speed filter coefficient
    ///
    /// # Arguments
    /// * `alpha` - Filter coefficient (0.0-1.0)
    ///   - Lower values = more filtering (smoother but slower response)
    ///   - Higher values = less filtering (faster but noisier)
    #[allow(dead_code)]
    pub fn set_filter_alpha(&mut self, alpha: f32) {
        self.speed_filter_alpha = alpha.clamp(0.0, 1.0);
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
        self.electrical_offset = offset_rad;
    }

    /// Get the electrical offset
    #[allow(dead_code)]
    pub fn get_electrical_offset(&self) -> f32 {
        self.electrical_offset
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
        // Test electrical angle = mechanical_angle * pole_pairs - offset
        let sensor = HallSensor::new(6, 0.05);

        // With zero mechanical angle and zero offset
        assert_eq!(sensor.get_electrical_angle(), 0.0);

        // mechanical_angle = 0.174533 rad (10 deg), pole_pairs = 6
        // electrical_angle should be 1.047198 rad (60 deg)
        // This is because: 10 deg mechanical * 6 pole_pairs = 60 deg electrical
    }
}
