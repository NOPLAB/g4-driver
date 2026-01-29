// Electrical angle calculation and advance angle computation
// Converts mechanical angle to electrical angle with calibration offset

use super::constants::{HALL_TO_ELECTRICAL_ANGLE, MECHANICAL_TO_ELECTRICAL_OFFSET};
use super::utils::normalize_angle;
use crate::config::advance_angle::{
    BASE_ADVANCE_DEG, MAX_ADVANCE_DEG, MAX_SPEED_FOR_ADVANCE, MIN_SPEED_FOR_ADVANCE,
};
use core::f32::consts::PI;

/// Degrees to radians conversion factor
const DEG_TO_RAD: f32 = PI / 180.0;

/// Electrical angle calculator
/// Handles conversion from mechanical to electrical angle with calibration
pub struct ElectricalAngle {
    /// Number of pole pairs
    pole_pairs: u8,
    /// Electrical offset in radians (calibration value)
    electrical_offset: f32,
    /// Enable advance angle for improved efficiency
    enable_advance_angle: bool,
}

impl ElectricalAngle {
    /// Create a new electrical angle calculator
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of pole pairs in the motor
    pub fn new(pole_pairs: u8) -> Self {
        Self {
            pole_pairs,
            electrical_offset: 0.0,
            enable_advance_angle: true,
        }
    }

    /// Calculate electrical angle from mechanical angle
    ///
    /// # Arguments
    /// * `mechanical_angle` - Mechanical angle in radians
    ///
    /// # Returns
    /// Electrical angle in radians (normalized to [0, TAU))
    pub fn to_electrical(&self, mechanical_angle: f32) -> f32 {
        normalize_angle(
            mechanical_angle * (self.pole_pairs as f32)
                + MECHANICAL_TO_ELECTRICAL_OFFSET
                + self.electrical_offset,
        )
    }

    /// Calculate electrical angle from interpolated mechanical angle with advance
    ///
    /// # Arguments
    /// * `mechanical_angle` - Mechanical angle in radians
    /// * `speed_rpm` - Current speed for advance angle calculation
    /// * `use_interpolation` - Whether interpolation is being used
    /// * `raw_hall_state` - Raw Hall state for discrete angle fallback
    ///
    /// # Returns
    /// Electrical angle in radians with optional advance angle applied
    pub fn calculate_with_advance(
        &self,
        mechanical_angle: f32,
        speed_rpm: f32,
        use_interpolation: bool,
        raw_hall_state: u8,
    ) -> f32 {
        // Calculate base electrical angle
        let base_electrical_angle = if use_interpolation {
            // Use interpolated mechanical angle
            mechanical_angle * (self.pole_pairs as f32) + MECHANICAL_TO_ELECTRICAL_OFFSET
        } else {
            // Use discrete Hall table for low speed
            HALL_TO_ELECTRICAL_ANGLE[raw_hall_state as usize]
        };

        // Add calibration offset
        let mut electrical_angle = base_electrical_angle + self.electrical_offset;

        // Apply advance angle if enabled
        if self.enable_advance_angle && speed_rpm > MIN_SPEED_FOR_ADVANCE {
            electrical_angle += self.calculate_advance_angle(speed_rpm);
        }

        normalize_angle(electrical_angle)
    }

    /// Calculate advance angle based on speed (linear interpolation)
    ///
    /// # Arguments
    /// * `speed_rpm` - Current speed in RPM
    ///
    /// # Returns
    /// Advance angle in radians
    fn calculate_advance_angle(&self, speed_rpm: f32) -> f32 {
        // Base advance angle (always applied)
        let base_advance_rad = BASE_ADVANCE_DEG * DEG_TO_RAD;

        // If speed is below threshold, return base advance only
        if speed_rpm <= MIN_SPEED_FOR_ADVANCE {
            return base_advance_rad;
        }

        // Calculate speed-proportional additional advance
        let speed_ratio = ((speed_rpm - MIN_SPEED_FOR_ADVANCE)
            / (MAX_SPEED_FOR_ADVANCE - MIN_SPEED_FOR_ADVANCE))
            .clamp(0.0, 1.0);

        let additional_advance_rad =
            (MAX_ADVANCE_DEG - BASE_ADVANCE_DEG) * DEG_TO_RAD * speed_ratio;

        base_advance_rad + additional_advance_rad
    }

    /// Set the electrical offset (calibration value)
    ///
    /// # Arguments
    /// * `offset_rad` - Electrical offset in radians
    #[allow(dead_code)]
    pub fn set_electrical_offset(&mut self, offset_rad: f32) {
        self.electrical_offset = offset_rad;
    }

    /// Get the electrical offset
    #[allow(dead_code)]
    pub fn get_electrical_offset(&self) -> f32 {
        self.electrical_offset
    }

    /// Enable or disable advance angle
    ///
    /// # Arguments
    /// * `enable` - True to enable advance angle, false to disable
    #[allow(dead_code)]
    pub fn set_advance_angle(&mut self, enable: bool) {
        self.enable_advance_angle = enable;
    }

    /// Check if advance angle is enabled
    #[allow(dead_code)]
    pub fn is_advance_angle_enabled(&self) -> bool {
        self.enable_advance_angle
    }

    /// Get current advance angle in degrees for a given speed
    #[allow(dead_code)]
    pub fn get_advance_deg_for_speed(&self, speed_rpm: f32) -> f32 {
        if !self.enable_advance_angle || speed_rpm <= MIN_SPEED_FOR_ADVANCE {
            return BASE_ADVANCE_DEG;
        }
        let advance_rad = self.calculate_advance_angle(speed_rpm);
        advance_rad * 180.0 / PI
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::TAU;

    #[test]
    fn test_electrical_angle_to_electrical() {
        let angle_calc = ElectricalAngle::new(6);

        // With zero mechanical angle: electrical = 0 * 6 + offset + 0 = offset
        let result = angle_calc.to_electrical(0.0);
        let expected = normalize_angle(MECHANICAL_TO_ELECTRICAL_OFFSET);
        assert!((result - expected).abs() < 0.001);
    }

    #[test]
    fn test_electrical_offset() {
        let mut angle_calc = ElectricalAngle::new(6);
        angle_calc.set_electrical_offset(0.5);

        let result = angle_calc.to_electrical(0.0);
        let expected = normalize_angle(MECHANICAL_TO_ELECTRICAL_OFFSET + 0.5);
        assert!((result - expected).abs() < 0.001);
    }

    #[test]
    fn test_advance_angle_low_speed() {
        let angle_calc = ElectricalAngle::new(6);

        // At low speed, should return base advance only
        let advance = angle_calc.get_advance_deg_for_speed(MIN_SPEED_FOR_ADVANCE);
        assert!((advance - BASE_ADVANCE_DEG).abs() < 0.001);
    }

    #[test]
    fn test_angle_normalization() {
        let angle_calc = ElectricalAngle::new(6);

        // Result should always be in [0, TAU)
        let result = angle_calc.to_electrical(100.0);
        assert!(result >= 0.0 && result < TAU);
    }
}
