//! Shaft position management module
//!
//! This module manages the motor shaft position.
//! It tracks multiple rotations and maintains both angle and rotation count.

use core::f32::consts::TAU;

/// Structure representing shaft position
/// Holds angle (0 to 2π rad) and rotation count (positive or negative integer)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShaftPosition {
    /// Current angle [rad] (0 ≤ angle < TAU)
    pub angle: f32,
    /// Rotation count (positive: forward, negative: reverse)
    pub rotations: i32,
    /// Direction inversion flag (true: inverted, false: normal)
    inversed: bool,
    /// Previous angle (for speed calculation)
    prev_angle: f32,
}

impl ShaftPosition {
    /// Create a new ShaftPosition (zero position)
    pub fn new() -> Self {
        Self {
            angle: 0.0,
            rotations: 0,
            inversed: false,
            prev_angle: 0.0,
        }
    }

    /// Normalize angle to 0 to TAU (0 to 2π) range
    #[inline]
    pub fn clamp(angle: f32) -> f32 {
        let mut normalized = angle % TAU;
        if normalized < 0.0 {
            normalized += TAU;
        }
        normalized
    }

    /// Reset position (return to zero position)
    pub fn reset(&mut self) {
        self.angle = 0.0;
        self.rotations = 0;
        self.prev_angle = 0.0;
    }

    /// Set direction inversion flag
    pub fn set_inversed(&mut self, inversed: bool) {
        self.inversed = inversed;
    }

    /// Get direction inversion flag
    #[allow(dead_code)]
    pub fn is_inversed(&self) -> bool {
        self.inversed
    }

    /// Update shaft position from sensor angle
    ///
    /// # Arguments
    /// * `sensor_angle` - Angle from sensor [rad]
    pub fn update_shaft_angle(&mut self, mut sensor_angle: f32) {
        // Direction inversion processing
        if self.inversed {
            sensor_angle = TAU - sensor_angle;
        }

        // Normalize angle to 0 to TAU
        sensor_angle = Self::clamp(sensor_angle);

        // Update rotation count (detect angle jump)
        let delta = sensor_angle - self.prev_angle;

        // Check if crossed boundary near 2π
        // Forward: from large value (e.g., 6.0) to small value (e.g., 0.2)
        if delta < -TAU / 2.0 {
            // Forward rotation (crossed 0→2π boundary)
            self.rotations += 1;
        }
        // Reverse: from small value to large value
        else if delta > TAU / 2.0 {
            // Reverse rotation (crossed 2π→0 boundary)
            self.rotations -= 1;
        }

        self.prev_angle = sensor_angle;
        self.angle = sensor_angle;
    }

    /// Advance position by specified angle increment
    ///
    /// # Arguments
    /// * `delta_angle` - Angle increment [rad] (positive: forward, negative: reverse)
    pub fn increment(&mut self, delta_angle: f32) {
        let mut new_angle = self.angle + delta_angle;

        // Update rotation count
        while new_angle >= TAU {
            new_angle -= TAU;
            self.rotations += 1;
        }
        while new_angle < 0.0 {
            new_angle += TAU;
            self.rotations -= 1;
        }

        self.angle = new_angle;
        self.prev_angle = new_angle;
    }

    /// Get current angle (0 to TAU)
    #[inline]
    pub fn get_angle(&self) -> f32 {
        self.angle
    }

    /// Get total position (rotations × TAU + angle)
    ///
    /// # Returns
    /// Cumulative position [rad] (including multiple rotations)
    pub fn get_position(&self) -> f32 {
        self.rotations as f32 * TAU + self.angle
    }

    /// Get angle change from previous update
    ///
    /// # Returns
    /// Angle change [rad]
    #[allow(dead_code)]
    pub fn delta(&self) -> f32 {
        // Calculate previous position
        let prev_position = self.rotations as f32 * TAU + self.prev_angle;
        let current_position = self.get_position();
        current_position - prev_position
    }

    /// Calculate position difference with another ShaftPosition
    ///
    /// # Arguments
    /// * `other` - ShaftPosition to compare
    ///
    /// # Returns
    /// Position difference [rad] (self - other)
    #[allow(dead_code)]
    pub fn compare(&self, other: &ShaftPosition) -> f32 {
        self.get_position() - other.get_position()
    }

    /// Calculate difference between two positions in -π to +π range (shortest path)
    ///
    /// # Arguments
    /// * `other` - ShaftPosition to compare
    ///
    /// # Returns
    /// Angle difference [rad] (-π ≤ diff ≤ +π)
    #[allow(dead_code)]
    pub fn angular_distance(&self, other: &ShaftPosition) -> f32 {
        let diff = self.angle - other.angle;

        // Normalize to -π to +π
        if diff > core::f32::consts::PI {
            diff - TAU
        } else if diff < -core::f32::consts::PI {
            diff + TAU
        } else {
            diff
        }
    }
}

impl Default for ShaftPosition {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clamp() {
        assert_eq!(ShaftPosition::clamp(0.0), 0.0);
        assert_eq!(ShaftPosition::clamp(TAU), 0.0);
        assert_eq!(ShaftPosition::clamp(TAU + 1.0), 1.0);
        assert!((ShaftPosition::clamp(-1.0) - (TAU - 1.0)).abs() < 1e-6);
    }

    #[test]
    fn test_update_forward() {
        let mut pos = ShaftPosition::new();

        // 0.0 → 1.0 → 2.0
        pos.update_shaft_angle(1.0);
        assert_eq!(pos.angle, 1.0);
        assert_eq!(pos.rotations, 0);

        pos.update_shaft_angle(2.0);
        assert_eq!(pos.angle, 2.0);
        assert_eq!(pos.rotations, 0);

        // Continue forward in smaller steps to avoid false boundary detection
        // 2.0 → 4.0 → 5.5 → 6.0 → 0.5 (1 rotation complete)
        pos.update_shaft_angle(4.0);
        assert_eq!(pos.rotations, 0);

        pos.update_shaft_angle(5.5);
        assert_eq!(pos.rotations, 0);

        pos.update_shaft_angle(6.0);
        assert_eq!(pos.rotations, 0);

        pos.update_shaft_angle(0.5);
        assert_eq!(pos.rotations, 1);
    }

    #[test]
    fn test_update_backward() {
        let mut pos = ShaftPosition::new();
        pos.update_shaft_angle(1.0);

        // Reverse: 1.0 → 0.5 → 6.0 (reverse)
        pos.update_shaft_angle(0.5);
        assert_eq!(pos.rotations, 0);

        pos.update_shaft_angle(6.0);
        assert_eq!(pos.rotations, -1);
    }

    #[test]
    fn test_increment() {
        let mut pos = ShaftPosition::new();

        // 0.0 + 1.0 = 1.0
        pos.increment(1.0);
        assert_eq!(pos.angle, 1.0);
        assert_eq!(pos.rotations, 0);

        // 1.0 + 6.0 = 7.0 → 7.0 - TAU ≈ 0.717 (1 rotation)
        pos.increment(6.0);
        assert!((pos.angle - (7.0 - TAU)).abs() < 1e-6);
        assert_eq!(pos.rotations, 1);
    }

    #[test]
    fn test_position() {
        let mut pos = ShaftPosition::new();
        pos.update_shaft_angle(1.0);
        assert!((pos.get_position() - 1.0).abs() < 1e-6);

        // Use smaller steps to avoid false boundary detection
        pos.update_shaft_angle(3.0);
        pos.update_shaft_angle(5.0);
        pos.update_shaft_angle(6.0);
        pos.update_shaft_angle(0.5);
        // 1 rotation + 0.5rad
        assert!((pos.get_position() - (TAU + 0.5)).abs() < 1e-6);
    }

    #[test]
    fn test_inversed() {
        let mut pos = ShaftPosition::new();
        pos.set_inversed(true);

        // Inverted mode: 1.0 → TAU - 1.0
        pos.update_shaft_angle(1.0);
        assert!((pos.angle - (TAU - 1.0)).abs() < 1e-6);
    }
}
