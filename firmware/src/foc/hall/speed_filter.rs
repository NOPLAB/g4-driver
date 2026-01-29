// Speed low-pass filter for Hall sensor speed estimation
// Implements exponential moving average filter for noise reduction

/// Low-pass filter for speed smoothing
/// Uses exponential moving average: new = alpha * instant + (1-alpha) * old
pub struct SpeedFilter {
    /// Current filtered speed in RPM
    speed_rpm: f32,
    /// Filter coefficient (0.0-1.0)
    /// Lower values = more filtering (smoother but slower response)
    /// Higher values = less filtering (faster but noisier)
    alpha: f32,
}

impl SpeedFilter {
    /// Create a new speed filter
    ///
    /// # Arguments
    /// * `alpha` - Filter coefficient (0.0-1.0, foc-simple uses 0.05)
    pub fn new(alpha: f32) -> Self {
        Self {
            speed_rpm: 0.0,
            alpha: alpha.clamp(0.0, 1.0),
        }
    }

    /// Update the filter with a new instant speed value
    ///
    /// # Arguments
    /// * `instant_rpm` - Instantaneous speed measurement in RPM
    ///
    /// # Returns
    /// Filtered speed in RPM
    pub fn update(&mut self, instant_rpm: f32) -> f32 {
        // Apply low-pass filter (foc-simple formula: new = (instant + 19*old)/20 for alpha=0.05)
        // Equivalent to: new = alpha*instant + (1-alpha)*old
        // instant_rpm が 0 の場合はノイズ判定なので速度を更新しない
        if instant_rpm > 0.0 {
            self.speed_rpm = self.alpha * instant_rpm + (1.0 - self.alpha) * self.speed_rpm;
        }
        self.speed_rpm
    }

    /// Initialize the filter to a specific speed value
    /// Useful for avoiding transient effects when starting from a known speed
    ///
    /// # Arguments
    /// * `speed_rpm` - Speed value to initialize to
    pub fn initialize(&mut self, speed_rpm: f32) {
        self.speed_rpm = speed_rpm;
    }

    /// Reset the filter to zero
    pub fn reset(&mut self) {
        self.speed_rpm = 0.0;
    }

    /// Get the current filtered speed
    pub fn get_speed(&self) -> f32 {
        self.speed_rpm
    }

    /// Set the filter coefficient
    ///
    /// # Arguments
    /// * `alpha` - Filter coefficient (0.0-1.0)
    #[allow(dead_code)]
    pub fn set_alpha(&mut self, alpha: f32) {
        self.alpha = alpha.clamp(0.0, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_filter_initialization() {
        let filter = SpeedFilter::new(0.05);
        assert_eq!(filter.get_speed(), 0.0);
    }

    #[test]
    fn test_filter_update() {
        let mut filter = SpeedFilter::new(0.5);
        filter.update(100.0);
        // With alpha=0.5: new = 0.5 * 100 + 0.5 * 0 = 50
        assert!((filter.get_speed() - 50.0).abs() < 0.001);

        filter.update(100.0);
        // new = 0.5 * 100 + 0.5 * 50 = 75
        assert!((filter.get_speed() - 75.0).abs() < 0.001);
    }

    #[test]
    fn test_filter_reset() {
        let mut filter = SpeedFilter::new(0.5);
        filter.update(100.0);
        filter.reset();
        assert_eq!(filter.get_speed(), 0.0);
    }

    #[test]
    fn test_zero_instant_rpm_ignored() {
        let mut filter = SpeedFilter::new(0.5);
        filter.initialize(100.0);
        filter.update(0.0); // Should be ignored
        assert_eq!(filter.get_speed(), 100.0);
    }
}
