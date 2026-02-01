//! Stall detection for BLDC motor control
//!
//! Detects motor stall conditions based on speed thresholds and count.
//! Used to trigger recovery actions (e.g., switching to open-loop mode).

/// Stall detector configuration
#[derive(Debug, Clone)]
pub struct StallDetectorConfig {
    /// Speed threshold below which stall is detected [RPM]
    pub speed_threshold: f32,
    /// Number of consecutive cycles below threshold to trigger stall
    pub count_threshold: u32,
}

impl Default for StallDetectorConfig {
    fn default() -> Self {
        Self {
            speed_threshold: 50.0,
            count_threshold: 2500, // 50ms @ 50kHz
        }
    }
}

/// Stall detector for motor control
///
/// Monitors motor speed and detects stall conditions when:
/// 1. Target speed is above threshold (motor should be running)
/// 2. Measured speed is below threshold (motor is not running)
/// 3. This condition persists for the count threshold duration
#[derive(Debug, Clone)]
pub struct StallDetector {
    /// Configuration
    config: StallDetectorConfig,
    /// Current stall count
    stall_count: u32,
}

impl StallDetector {
    /// Create a new stall detector with configuration
    pub fn new(config: StallDetectorConfig) -> Self {
        Self {
            config,
            stall_count: 0,
        }
    }

    /// Create with default configuration
    pub fn with_defaults() -> Self {
        Self::new(StallDetectorConfig::default())
    }

    /// Update the stall detector and check for stall condition
    ///
    /// # Arguments
    /// * `target_speed` - Target speed command [RPM] (absolute value)
    /// * `measured_speed` - Measured motor speed [RPM] (absolute value)
    ///
    /// # Returns
    /// `true` if stall is detected, `false` otherwise
    pub fn update(&mut self, target_speed: f32, measured_speed: f32) -> bool {
        let target_magnitude = target_speed.abs();
        let measured_magnitude = measured_speed.abs();

        // Check if motor should be running but isn't
        if target_magnitude > self.config.speed_threshold
            && measured_magnitude < self.config.speed_threshold
        {
            self.stall_count += 1;

            if self.stall_count >= self.config.count_threshold {
                return true;
            }
        } else {
            // Speed recovered, reset counter
            self.stall_count = 0;
        }

        false
    }

    /// Get the current stall count
    pub fn get_stall_count(&self) -> u32 {
        self.stall_count
    }

    /// Reset the stall detector
    pub fn reset(&mut self) {
        self.stall_count = 0;
    }

    /// Set the speed threshold
    pub fn set_speed_threshold(&mut self, threshold: f32) {
        self.config.speed_threshold = threshold;
    }

    /// Set the count threshold
    pub fn set_count_threshold(&mut self, count: u32) {
        self.config.count_threshold = count;
    }

    /// Get the configuration
    pub fn get_config(&self) -> &StallDetectorConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_detector() {
        let detector = StallDetector::with_defaults();
        assert_eq!(detector.get_stall_count(), 0);
    }

    #[test]
    fn test_no_stall_normal_operation() {
        let mut detector = StallDetector::new(StallDetectorConfig {
            speed_threshold: 50.0,
            count_threshold: 100,
        });

        // Target and measured both above threshold
        for _ in 0..200 {
            let stalled = detector.update(500.0, 450.0);
            assert!(!stalled);
        }
    }

    #[test]
    fn test_no_stall_target_zero() {
        let mut detector = StallDetector::new(StallDetectorConfig {
            speed_threshold: 50.0,
            count_threshold: 100,
        });

        // Target is zero (motor should be stopped)
        for _ in 0..200 {
            let stalled = detector.update(0.0, 10.0);
            assert!(!stalled);
        }
    }

    #[test]
    fn test_stall_detection() {
        let mut detector = StallDetector::new(StallDetectorConfig {
            speed_threshold: 50.0,
            count_threshold: 100,
        });

        // Simulate stall: target high but measured low
        for i in 0..99 {
            let stalled = detector.update(500.0, 20.0);
            assert!(!stalled, "Should not stall at count {}", i);
        }

        // 100th cycle should trigger stall
        let stalled = detector.update(500.0, 20.0);
        assert!(stalled, "Should stall at count 100");
    }

    #[test]
    fn test_stall_count_reset() {
        let mut detector = StallDetector::new(StallDetectorConfig {
            speed_threshold: 50.0,
            count_threshold: 100,
        });

        // Build up some stall count
        for _ in 0..50 {
            detector.update(500.0, 20.0);
        }
        assert_eq!(detector.get_stall_count(), 50);

        // Speed recovers
        detector.update(500.0, 100.0);
        assert_eq!(detector.get_stall_count(), 0);
    }

    #[test]
    fn test_reverse_speed() {
        let mut detector = StallDetector::new(StallDetectorConfig {
            speed_threshold: 50.0,
            count_threshold: 100,
        });

        // Negative target and measured (reverse direction)
        // Should work with absolute values
        for i in 0..100 {
            let stalled = detector.update(-500.0, -20.0);
            if i < 99 {
                assert!(!stalled);
            } else {
                assert!(stalled);
            }
        }
    }

    #[test]
    fn test_reset() {
        let mut detector = StallDetector::with_defaults();

        for _ in 0..1000 {
            detector.update(500.0, 20.0);
        }

        detector.reset();
        assert_eq!(detector.get_stall_count(), 0);
    }
}
