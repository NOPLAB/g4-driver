//! Performance criteria definitions

/// Performance criteria for control validation
#[derive(Debug, Clone)]
pub struct PerformanceCriteria {
    /// Maximum allowed overshoot [%]
    pub max_overshoot_percent: f32,
    /// Maximum settling time [ms]
    pub settling_time_ms: f32,
    /// Maximum steady-state error [RPM]
    pub steady_state_error_rpm: f32,
    /// Maximum rise time [ms]
    pub rise_time_ms: f32,
}

impl PerformanceCriteria {
    /// Create new criteria
    pub fn new(max_overshoot: f32, settling_time: f32, ss_error: f32, rise_time: f32) -> Self {
        Self {
            max_overshoot_percent: max_overshoot,
            settling_time_ms: settling_time,
            steady_state_error_rpm: ss_error,
            rise_time_ms: rise_time,
        }
    }

    /// Relaxed criteria for initial testing
    pub fn relaxed() -> Self {
        Self {
            max_overshoot_percent: 50.0,
            settling_time_ms: 1000.0,
            steady_state_error_rpm: 50.0,
            rise_time_ms: 500.0,
        }
    }

    /// Strict criteria for production
    pub fn strict() -> Self {
        Self {
            max_overshoot_percent: 10.0,
            settling_time_ms: 200.0,
            steady_state_error_rpm: 5.0,
            rise_time_ms: 100.0,
        }
    }

    /// Builder-style setters
    pub fn with_overshoot(mut self, percent: f32) -> Self {
        self.max_overshoot_percent = percent;
        self
    }

    pub fn with_settling_time(mut self, time_ms: f32) -> Self {
        self.settling_time_ms = time_ms;
        self
    }

    pub fn with_ss_error(mut self, error_rpm: f32) -> Self {
        self.steady_state_error_rpm = error_rpm;
        self
    }

    pub fn with_rise_time(mut self, time_ms: f32) -> Self {
        self.rise_time_ms = time_ms;
        self
    }
}

impl Default for PerformanceCriteria {
    fn default() -> Self {
        Self {
            max_overshoot_percent: 20.0,
            settling_time_ms: 500.0,
            steady_state_error_rpm: 10.0,
            rise_time_ms: 200.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_criteria() {
        let criteria = PerformanceCriteria::default();
        assert!((criteria.max_overshoot_percent - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_relaxed_criteria() {
        let criteria = PerformanceCriteria::relaxed();
        assert!(criteria.max_overshoot_percent > 20.0);
        assert!(criteria.settling_time_ms > 500.0);
    }

    #[test]
    fn test_strict_criteria() {
        let criteria = PerformanceCriteria::strict();
        assert!(criteria.max_overshoot_percent < 20.0);
        assert!(criteria.settling_time_ms < 500.0);
    }

    #[test]
    fn test_builder() {
        let criteria = PerformanceCriteria::default()
            .with_overshoot(15.0)
            .with_settling_time(300.0);

        assert!((criteria.max_overshoot_percent - 15.0).abs() < 0.1);
        assert!((criteria.settling_time_ms - 300.0).abs() < 0.1);
    }
}
