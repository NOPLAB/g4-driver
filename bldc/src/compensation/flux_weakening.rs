//! Flux weakening control
//!
//! At high speeds, apply negative voltage to d-axis to suppress back EMF
//! and improve the motor's maximum speed.
//!
//! Since there is no current sensing, this is implemented as a speed-based feedforward method.

use libm::sqrtf;

/// Flux weakening controller
#[derive(Debug, Clone)]
pub struct FluxWeakeningController {
    /// Control enabled/disabled
    enabled: bool,
    /// Weakening start speed [RPM]
    min_speed: f32,
    /// Maximum weakening speed [RPM]
    max_speed: f32,
    /// Maximum weakening ratio (0.0-1.0)
    max_weakening_ratio: f32,
    /// Vd rate limit [V/s]
    vd_rate_limit: f32,
    /// Current Vd value [V] (for rate limiting)
    current_vd: f32,
}

impl FluxWeakeningController {
    /// Create new instance
    ///
    /// # Arguments
    /// * `min_speed` - Weakening start speed [RPM]
    /// * `max_speed` - Maximum weakening speed [RPM]
    /// * `max_weakening_ratio` - Maximum weakening ratio (0.0-1.0)
    /// * `vd_rate_limit` - Vd rate limit [V/s]
    pub fn new(
        min_speed: f32,
        max_speed: f32,
        max_weakening_ratio: f32,
        vd_rate_limit: f32,
    ) -> Self {
        Self {
            enabled: false,
            min_speed,
            max_speed,
            max_weakening_ratio: max_weakening_ratio.clamp(0.0, 1.0),
            vd_rate_limit,
            current_vd: 0.0,
        }
    }

    /// Set control enabled/disabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Return whether control is enabled
    #[allow(dead_code)]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Reset internal state
    pub fn reset(&mut self) {
        self.current_vd = 0.0;
    }

    /// Calculate d-axis voltage according to speed
    ///
    /// # Arguments
    /// * `speed_rpm` - Current speed [RPM]
    /// * `vq` - q-axis voltage command [V]
    /// * `v_dc` - DC bus voltage [V]
    /// * `dt` - Control period [s]
    ///
    /// # Returns
    /// d-axis voltage command [V] (negative value)
    pub fn calculate_vd(&mut self, speed_rpm: f32, vq: f32, v_dc: f32, dt: f32) -> f32 {
        if !self.enabled {
            self.current_vd = 0.0;
            return 0.0;
        }

        let speed_abs = speed_rpm.abs();

        // No weakening below start speed
        if speed_abs < self.min_speed {
            // Apply rate limit and return to 0
            return self.apply_rate_limit(0.0, dt);
        }

        // Calculate weakening ratio according to speed (linear interpolation)
        let speed_range = self.max_speed - self.min_speed;
        let weakening_ratio = if speed_range > 0.0 {
            let ratio = (speed_abs - self.min_speed) / speed_range;
            (ratio * self.max_weakening_ratio).clamp(0.0, self.max_weakening_ratio)
        } else {
            0.0
        };

        // Calculate available d-axis voltage
        // |Vd|^2 + |Vq|^2 <= Vdc^2, so Vd_max = sqrt(Vdc^2 - Vq^2)
        let vq_abs = vq.abs();
        let vd_available = if vq_abs < v_dc {
            sqrtf(v_dc * v_dc - vq_abs * vq_abs)
        } else {
            0.0
        };

        // Target Vd (negative value, according to weakening ratio)
        let target_vd = -vd_available * weakening_ratio;

        // Apply rate limit
        self.apply_rate_limit(target_vd, dt)
    }

    /// Apply rate limit and update Vd
    fn apply_rate_limit(&mut self, target_vd: f32, dt: f32) -> f32 {
        let max_delta = self.vd_rate_limit * dt;
        let delta = target_vd - self.current_vd;

        if delta.abs() > max_delta {
            if delta > 0.0 {
                self.current_vd += max_delta;
            } else {
                self.current_vd -= max_delta;
            }
        } else {
            self.current_vd = target_vd;
        }

        self.current_vd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_disabled() {
        let mut fw = FluxWeakeningController::new(2000.0, 4000.0, 0.5, 100.0);
        // When disabled, always 0
        let vd = fw.calculate_vd(3000.0, 10.0, 24.0, 0.0001);
        assert_eq!(vd, 0.0);
    }

    #[test]
    fn test_below_min_speed() {
        let mut fw = FluxWeakeningController::new(2000.0, 4000.0, 0.5, 100.0);
        fw.set_enabled(true);

        // Below weakening start speed, Vd = 0
        let vd = fw.calculate_vd(1000.0, 10.0, 24.0, 0.001);
        assert_eq!(vd, 0.0);
    }

    #[test]
    fn test_weakening_at_mid_speed() {
        let mut fw = FluxWeakeningController::new(2000.0, 4000.0, 0.5, 1000.0);
        fw.set_enabled(true);

        // At mid speed (3000 RPM), weakening ratio = 0.5 * (3000-2000)/(4000-2000) = 0.25
        // Allow enough time to reach target
        for _ in 0..100 {
            fw.calculate_vd(3000.0, 10.0, 24.0, 0.01);
        }
        let vd = fw.calculate_vd(3000.0, 10.0, 24.0, 0.01);

        // Vd is negative
        assert!(vd < 0.0);
    }

    #[test]
    fn test_rate_limit() {
        let mut fw = FluxWeakeningController::new(2000.0, 4000.0, 0.5, 10.0);
        fw.set_enabled(true);

        // Rate limit 10V/s, dt=0.1s, max change is 1V
        let vd1 = fw.calculate_vd(4000.0, 10.0, 24.0, 0.1);
        assert!((-1.1..=0.0).contains(&vd1));

        // Second call also rate limited
        let vd2 = fw.calculate_vd(4000.0, 10.0, 24.0, 0.1);
        assert!((-2.1..=0.0).contains(&vd2));
        assert!(vd2 < vd1); // More negative
    }
}
