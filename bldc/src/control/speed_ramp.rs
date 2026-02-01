//! Speed ramp (acceleration limiter) for smooth motor control
//!
//! Limits the rate of speed command changes to prevent sudden motor jerks
//! and ensure smooth acceleration/deceleration.

/// Speed ramp controller for acceleration limiting
///
/// Smoothly transitions from current speed to target speed
/// while respecting maximum acceleration limits.
#[derive(Debug, Clone)]
pub struct SpeedRamp {
    /// Current ramped speed [RPM]
    current_speed: f32,
    /// Maximum acceleration rate [RPM/s]
    max_acceleration: f32,
}

impl SpeedRamp {
    /// Create a new speed ramp controller
    ///
    /// # Arguments
    /// * `max_acceleration` - Maximum acceleration rate [RPM/s]
    pub fn new(max_acceleration: f32) -> Self {
        Self {
            current_speed: 0.0,
            max_acceleration,
        }
    }

    /// Update the speed ramp
    ///
    /// # Arguments
    /// * `target_speed` - Target speed [RPM]
    /// * `dt` - Time step [s]
    ///
    /// # Returns
    /// Ramped speed command [RPM]
    pub fn update(&mut self, target_speed: f32, dt: f32) -> f32 {
        let speed_error = target_speed - self.current_speed;
        let max_delta = self.max_acceleration * dt;

        if speed_error.abs() > max_delta {
            // Apply acceleration limit
            if speed_error > 0.0 {
                self.current_speed += max_delta;
            } else {
                self.current_speed -= max_delta;
            }
        } else {
            // Target reached
            self.current_speed = target_speed;
        }

        self.current_speed
    }

    /// Get the current ramped speed
    pub fn get_current_speed(&self) -> f32 {
        self.current_speed
    }

    /// Set the current speed directly (for initialization)
    ///
    /// Use this when transitioning from another control mode
    /// to ensure smooth output continuity.
    pub fn set_current_speed(&mut self, speed: f32) {
        self.current_speed = speed;
    }

    /// Set the maximum acceleration rate
    pub fn set_max_acceleration(&mut self, max_acceleration: f32) {
        self.max_acceleration = max_acceleration;
    }

    /// Get the maximum acceleration rate
    pub fn get_max_acceleration(&self) -> f32 {
        self.max_acceleration
    }

    /// Reset the ramp to zero
    pub fn reset(&mut self) {
        self.current_speed = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 0.001;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    #[test]
    fn test_new_ramp() {
        let ramp = SpeedRamp::new(100.0);
        assert!(approx_eq(ramp.get_current_speed(), 0.0));
        assert!(approx_eq(ramp.get_max_acceleration(), 100.0));
    }

    #[test]
    fn test_acceleration_limited() {
        let mut ramp = SpeedRamp::new(100.0); // 100 RPM/s

        // Target 1000 RPM, dt = 0.1s -> max change = 10 RPM
        let speed = ramp.update(1000.0, 0.1);
        assert!(approx_eq(speed, 10.0));

        // Another step
        let speed = ramp.update(1000.0, 0.1);
        assert!(approx_eq(speed, 20.0));
    }

    #[test]
    fn test_deceleration_limited() {
        let mut ramp = SpeedRamp::new(100.0);
        ramp.set_current_speed(500.0);

        // Target 0 RPM, dt = 0.1s -> max change = 10 RPM
        let speed = ramp.update(0.0, 0.1);
        assert!(approx_eq(speed, 490.0));
    }

    #[test]
    fn test_target_reached() {
        let mut ramp = SpeedRamp::new(100.0);
        ramp.set_current_speed(95.0);

        // Target 100 RPM, dt = 0.1s, delta = 5 < max_delta = 10
        let speed = ramp.update(100.0, 0.1);
        assert!(approx_eq(speed, 100.0));
    }

    #[test]
    fn test_reverse_direction() {
        let mut ramp = SpeedRamp::new(100.0);

        // Accelerate to negative speed
        let speed = ramp.update(-1000.0, 0.1);
        assert!(approx_eq(speed, -10.0));

        let speed = ramp.update(-1000.0, 0.1);
        assert!(approx_eq(speed, -20.0));
    }

    #[test]
    fn test_reset() {
        let mut ramp = SpeedRamp::new(100.0);
        ramp.set_current_speed(500.0);

        ramp.reset();
        assert!(approx_eq(ramp.get_current_speed(), 0.0));
    }
}
