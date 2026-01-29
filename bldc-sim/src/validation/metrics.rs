//! Performance metrics calculation

use crate::motor_model::StateSnapshot;

/// Calculate performance metrics from state history
pub struct MetricsCalculator {
    /// Target speed [RPM]
    target_speed: f32,
    /// Settling tolerance [RPM]
    tolerance: f32,
}

impl MetricsCalculator {
    /// Create new calculator
    pub fn new(target_speed: f32) -> Self {
        Self {
            target_speed,
            tolerance: target_speed.abs() * 0.02, // 2% default
        }
    }

    /// Set tolerance
    pub fn with_tolerance(mut self, tolerance: f32) -> Self {
        self.tolerance = tolerance;
        self
    }

    /// Calculate rise time (time to reach 90% of target)
    pub fn rise_time(&self, history: &[StateSnapshot]) -> Option<f32> {
        let threshold = 0.9 * self.target_speed;

        for snapshot in history {
            if snapshot.speed_rpm >= threshold {
                return Some(snapshot.time * 1000.0); // Convert to ms
            }
        }
        None
    }

    /// Calculate settling time (last time outside tolerance)
    pub fn settling_time(&self, history: &[StateSnapshot]) -> Option<f32> {
        let mut last_outside = None;

        for snapshot in history {
            if (snapshot.speed_rpm - self.target_speed).abs() > self.tolerance {
                last_outside = Some(snapshot.time * 1000.0);
            }
        }

        last_outside
    }

    /// Calculate overshoot percentage
    pub fn overshoot_percent(&self, history: &[StateSnapshot]) -> f32 {
        if self.target_speed <= 0.0 {
            return 0.0;
        }

        let max_speed = history
            .iter()
            .map(|s| s.speed_rpm)
            .fold(0.0f32, |a, b| a.max(b));

        ((max_speed - self.target_speed) / self.target_speed * 100.0).max(0.0)
    }

    /// Calculate steady-state error (average of last N samples)
    pub fn steady_state_error(&self, history: &[StateSnapshot]) -> f32 {
        if history.is_empty() {
            return 0.0;
        }

        let n = (history.len() / 10).max(1); // Last 10% or at least 1
        let last_samples: Vec<_> = history.iter().rev().take(n).collect();

        let avg_speed: f32 = last_samples.iter().map(|s| s.speed_rpm).sum::<f32>()
            / last_samples.len() as f32;

        (avg_speed - self.target_speed).abs()
    }

    /// Calculate peak current
    pub fn peak_current(&self, history: &[StateSnapshot]) -> f32 {
        history
            .iter()
            .map(|s| libm::sqrtf(s.i_d * s.i_d + s.i_q * s.i_q))
            .fold(0.0f32, |a, b| a.max(b))
    }

    /// Calculate RMS current
    pub fn rms_current(&self, history: &[StateSnapshot]) -> f32 {
        if history.is_empty() {
            return 0.0;
        }

        let sum_sq: f32 = history
            .iter()
            .map(|s| s.i_d * s.i_d + s.i_q * s.i_q)
            .sum();

        libm::sqrtf(sum_sq / history.len() as f32)
    }

    /// Calculate average torque
    pub fn average_torque(&self, history: &[StateSnapshot]) -> f32 {
        if history.is_empty() {
            return 0.0;
        }

        history.iter().map(|s| s.torque).sum::<f32>() / history.len() as f32
    }

    /// Calculate total rotations
    pub fn total_rotations(&self, history: &[StateSnapshot]) -> i32 {
        history.last().map(|s| s.rotations).unwrap_or(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_history(speeds: &[(f32, f32)]) -> Vec<StateSnapshot> {
        speeds
            .iter()
            .map(|(time, speed)| StateSnapshot {
                time: *time,
                speed_rpm: *speed,
                i_d: 0.0,
                i_q: 0.0,
                theta_m: 0.0,
                omega_m: 0.0,
                theta_e: 0.0,
                torque: 0.0,
                rotations: 0,
            })
            .collect()
    }

    #[test]
    fn test_rise_time() {
        let history = make_history(&[
            (0.0, 0.0),
            (0.1, 300.0),
            (0.2, 600.0),
            (0.3, 900.0),
            (0.4, 1000.0),
        ]);

        let calc = MetricsCalculator::new(1000.0);
        let rise_time = calc.rise_time(&history);

        assert!(rise_time.is_some());
        assert!((rise_time.unwrap() - 300.0).abs() < 1.0); // 0.3s = 300ms
    }

    #[test]
    fn test_overshoot() {
        let history = make_history(&[
            (0.0, 0.0),
            (0.1, 500.0),
            (0.2, 1200.0), // 20% overshoot
            (0.3, 1000.0),
            (0.4, 1000.0),
        ]);

        let calc = MetricsCalculator::new(1000.0);
        let overshoot = calc.overshoot_percent(&history);

        assert!((overshoot - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_steady_state_error() {
        let history = make_history(&[
            (0.0, 0.0),
            (0.1, 500.0),
            (0.2, 900.0),
            (0.3, 990.0),
            (0.4, 995.0),
        ]);

        let calc = MetricsCalculator::new(1000.0);
        let error = calc.steady_state_error(&history);

        // Last sample is 995, so error is 5
        assert!(error < 10.0);
    }
}
