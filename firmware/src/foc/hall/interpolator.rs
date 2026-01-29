// Angle interpolation between Hall edges
// Provides smooth angle estimation between discrete Hall sensor readings

use super::utils::normalize_angle;
use core::f32::consts::TAU;

/// Minimum speed threshold for interpolation (RPM)
/// Below this speed, discrete Hall angles are used
const MIN_INTERPOLATION_SPEED: f32 = 1.0;

/// Angle interpolator for smooth position estimation between Hall edges
pub struct AngleInterpolator {
    /// Time since last Hall edge (seconds)
    time_since_edge: f32,
    /// Whether interpolation is enabled
    enabled: bool,
}

impl AngleInterpolator {
    /// Create a new angle interpolator
    pub fn new() -> Self {
        Self {
            time_since_edge: 0.0,
            enabled: true,
        }
    }

    /// Update time since last edge
    ///
    /// # Arguments
    /// * `dt` - Time step since last update (seconds)
    pub fn accumulate_time(&mut self, dt: f32) {
        self.time_since_edge += dt;
    }

    /// Reset the edge timer (call on Hall edge detection)
    pub fn reset_time(&mut self) {
        self.time_since_edge = 0.0;
    }

    /// Interpolate angle based on speed and time since last edge
    ///
    /// # Arguments
    /// * `base_angle` - Base mechanical angle from Hall state (radians)
    /// * `speed_rpm` - Current filtered speed (RPM)
    ///
    /// # Returns
    /// Interpolated mechanical angle (radians)
    pub fn interpolate(&self, base_angle: f32, speed_rpm: f32) -> f32 {
        if !self.enabled || speed_rpm.abs() <= MIN_INTERPOLATION_SPEED {
            return base_angle;
        }

        // Calculate mechanical angular velocity (rad/s)
        let mechanical_omega = speed_rpm * (TAU / 60.0); // RPM to rad/s (2*PI/60)

        // Interpolate angle based on time since last edge
        let angle_increment = mechanical_omega * self.time_since_edge;
        normalize_angle(base_angle + angle_increment)
    }

    /// Check if interpolation should be applied
    ///
    /// # Arguments
    /// * `speed_rpm` - Current filtered speed (RPM)
    ///
    /// # Returns
    /// `true` if interpolation should be applied
    pub fn should_interpolate(&self, speed_rpm: f32) -> bool {
        self.enabled && speed_rpm.abs() > MIN_INTERPOLATION_SPEED
    }

    /// Get time since last Hall edge
    #[allow(dead_code)]
    pub fn get_time_since_edge(&self) -> f32 {
        self.time_since_edge
    }

    /// Enable or disable interpolation
    ///
    /// # Arguments
    /// * `enable` - True to enable interpolation, false for discrete Hall angles only
    #[allow(dead_code)]
    pub fn set_enabled(&mut self, enable: bool) {
        self.enabled = enable;
    }

    /// Check if interpolation is enabled
    #[allow(dead_code)]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Reset the interpolator state
    pub fn reset(&mut self) {
        self.time_since_edge = 0.0;
    }
}

impl Default for AngleInterpolator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    #[test]
    fn test_interpolation_disabled() {
        let mut interp = AngleInterpolator::new();
        interp.set_enabled(false);
        interp.accumulate_time(0.01);

        // Should return base angle when disabled
        let result = interp.interpolate(0.0, 1000.0);
        assert_eq!(result, 0.0);
    }

    #[test]
    fn test_interpolation_low_speed() {
        let mut interp = AngleInterpolator::new();
        interp.accumulate_time(0.01);

        // Should return base angle at low speed
        let result = interp.interpolate(0.0, 0.5);
        assert_eq!(result, 0.0);
    }

    #[test]
    fn test_interpolation_calculation() {
        let mut interp = AngleInterpolator::new();
        interp.accumulate_time(0.01); // 10ms

        // At 600 RPM = 10 rev/s = 62.83 rad/s mechanical
        // In 10ms: 62.83 * 0.01 = 0.6283 rad = ~36 degrees
        let result = interp.interpolate(0.0, 600.0);
        let expected = 600.0 * TAU / 60.0 * 0.01;
        assert!((result - expected).abs() < 0.001);
    }

    #[test]
    fn test_angle_wrapping() {
        let mut interp = AngleInterpolator::new();
        interp.accumulate_time(0.5); // Long time to cause wrap

        let result = interp.interpolate(PI, 600.0);
        // Result should be normalized to [0, TAU)
        assert!(result >= 0.0 && result < TAU);
    }
}
