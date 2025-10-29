// Hall sensor processing for BLDC motor position and speed estimation

use crate::fmt::*;

/// Hall sensor states to electrical angle mapping (in degrees)
/// Hall state format: (H3 << 2) | (H2 << 1) | H1
/// Valid states are 1-6 (0b001 to 0b110), invalid states are 0 (0b000) and 7 (0b111)
/// Using sector CENTER angles for better FOC accuracy
const HALL_ANGLE_TABLE: [Option<f32>; 8] = [
    None,        // 0b000: Invalid state
    Some(30.0),  // 0b001: Sector 1 center
    Some(90.0),  // 0b010: Sector 2 center
    Some(150.0), // 0b011: Sector 3 center
    Some(210.0), // 0b100: Sector 4 center
    Some(270.0), // 0b101: Sector 5 center
    Some(330.0), // 0b110: Sector 6 center
    None,        // 0b111: Invalid state
];

/// Hall sensor state machine for position and speed estimation
pub struct HallSensor {
    /// Previous hall state (0-7)
    prev_state: u8,
    /// Current electrical angle in radians
    electrical_angle: f32,
    /// Hall sector base angle in radians (updated at each edge)
    hall_sector_angle: f32,
    /// Current speed in RPM
    speed_rpm: f32,
    /// Accumulated time since last hall edge (seconds)
    time_since_edge: f32,
    /// Low-pass filter coefficient for speed (0.0 - 1.0)
    /// Lower value = more filtering
    speed_filter_alpha: f32,
    /// Number of pole pairs
    pole_pairs: u8,
    /// Flag indicating if this is the first update
    first_update: bool,
    /// Timeout threshold for speed estimation (seconds)
    /// If no edge detected within this time, speed is set to 0
    edge_timeout: f32,
    /// Enable angle interpolation between Hall edges
    enable_interpolation: bool,
}

impl HallSensor {
    /// Create a new Hall sensor instance
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of pole pairs in the motor
    /// * `speed_filter_alpha` - Low-pass filter coefficient (0.0-1.0, default 0.1)
    pub fn new(pole_pairs: u8, speed_filter_alpha: f32) -> Self {
        Self {
            prev_state: 0,
            electrical_angle: 0.0,
            hall_sector_angle: 0.0,
            speed_rpm: 0.0,
            time_since_edge: 0.0,
            speed_filter_alpha: speed_filter_alpha.clamp(0.0, 1.0),
            pole_pairs,
            first_update: true,
            edge_timeout: 1.0, // 1 second timeout (< 60 RPM for 7 pole pairs)
            enable_interpolation: true, // Enable angle interpolation by default
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
    ///
    /// # Arguments
    /// * `hall_state` - Current hall state (0-7)
    /// * `dt` - Time step since last update (seconds)
    ///
    /// # Returns
    /// Tuple of (electrical_angle in radians, speed in RPM)
    pub fn update(&mut self, hall_state: u8, dt: f32) -> (f32, f32) {
        // Validate hall state
        if !Self::is_valid_state(hall_state) {
            error!("Invalid hall state: {}", hall_state);

            // Even with invalid state, accumulate time for timeout detection
            self.time_since_edge += dt;

            // Check for timeout and set speed to 0 if motor likely stopped
            if self.time_since_edge > self.edge_timeout {
                self.speed_rpm = 0.0;
            }

            // Keep previous electrical angle and current speed
            return (self.electrical_angle, self.speed_rpm);
        }

        // Get electrical angle from lookup table
        if let Some(angle_deg) = HALL_ANGLE_TABLE[hall_state as usize] {
            // Convert degrees to radians
            let angle_rad = angle_deg * 0.017453293; // PI / 180

            // Detect state change (hall edge)
            if hall_state != self.prev_state && !self.first_update {
                // Calculate speed based on time between edges
                // Each hall edge represents 60 electrical degrees
                let electrical_degrees_per_edge = 60.0;
                let mechanical_degrees_per_edge =
                    electrical_degrees_per_edge / self.pole_pairs as f32;

                if self.time_since_edge > 0.0 {
                    // RPM = (degrees per edge / time) * (60 sec/min) / (360 deg/rev)
                    let instant_rpm =
                        (mechanical_degrees_per_edge / self.time_since_edge) * (60.0 / 360.0);

                    // Apply low-pass filter to smooth speed
                    self.speed_rpm = self.speed_filter_alpha * instant_rpm
                        + (1.0 - self.speed_filter_alpha) * self.speed_rpm;

                    trace!(
                        "Hall edge: {} -> {}, dt={}s, instant_rpm={}, filtered_rpm={}",
                        self.prev_state,
                        hall_state,
                        self.time_since_edge,
                        instant_rpm,
                        self.speed_rpm
                    );
                }

                // Reset edge timer
                self.time_since_edge = 0.0;

                // Update hall sector base angle
                self.hall_sector_angle = angle_rad;
            } else {
                // Accumulate time since last edge
                self.time_since_edge += dt;

                // If too much time has passed without an edge, assume motor stopped
                if self.time_since_edge > self.edge_timeout {
                    self.speed_rpm = 0.0;
                }
            }

            // Update electrical angle with interpolation
            if self.enable_interpolation && self.speed_rpm > 0.0 && !self.first_update {
                // Calculate electrical angular velocity (rad/s)
                let electrical_rpm = self.speed_rpm * self.pole_pairs as f32;
                let electrical_omega = electrical_rpm * 0.104719755; // RPM to rad/s (2*PI/60)

                // Interpolate angle based on time since last edge
                let angle_increment = electrical_omega * self.time_since_edge;
                self.electrical_angle = self.hall_sector_angle + angle_increment;

                // Normalize angle to [0, 2π)
                const TWO_PI: f32 = 6.283185307;
                while self.electrical_angle >= TWO_PI {
                    self.electrical_angle -= TWO_PI;
                }
                while self.electrical_angle < 0.0 {
                    self.electrical_angle += TWO_PI;
                }
            } else {
                // No interpolation: use discrete Hall sensor angle
                self.electrical_angle = angle_rad;
                self.hall_sector_angle = angle_rad;
            }

            self.prev_state = hall_state;
            self.first_update = false;
        }

        (self.electrical_angle, self.speed_rpm)
    }

    /// Get current electrical angle in radians
    pub fn get_electrical_angle(&self) -> f32 {
        self.electrical_angle
    }

    /// Get current speed in RPM
    pub fn get_speed_rpm(&self) -> f32 {
        self.speed_rpm
    }

    /// Reset the hall sensor state
    pub fn reset(&mut self) {
        self.prev_state = 0;
        self.electrical_angle = 0.0;
        self.hall_sector_angle = 0.0;
        self.speed_rpm = 0.0;
        self.time_since_edge = 0.0;
        self.first_update = true;
    }

    /// Enable or disable angle interpolation
    ///
    /// # Arguments
    /// * `enable` - True to enable interpolation, false for discrete Hall angles only
    pub fn set_interpolation(&mut self, enable: bool) {
        self.enable_interpolation = enable;
    }

    /// Check if interpolation is enabled
    pub fn is_interpolation_enabled(&self) -> bool {
        self.enable_interpolation
    }

    /// Set the speed filter coefficient
    ///
    /// # Arguments
    /// * `alpha` - Filter coefficient (0.0-1.0)
    ///   - Lower values = more filtering (smoother but slower response)
    ///   - Higher values = less filtering (faster but noisier)
    pub fn set_filter_alpha(&mut self, alpha: f32) {
        self.speed_filter_alpha = alpha.clamp(0.0, 1.0);
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
    fn test_angle_lookup() {
        assert_eq!(HALL_ANGLE_TABLE[1], Some(30.0));
        assert_eq!(HALL_ANGLE_TABLE[2], Some(90.0));
        assert_eq!(HALL_ANGLE_TABLE[6], Some(330.0));
        assert_eq!(HALL_ANGLE_TABLE[0], None);
        assert_eq!(HALL_ANGLE_TABLE[7], None);
    }
}
