//! Cogging torque model
//!
//! Models the position-dependent torque ripple caused by interaction
//! between permanent magnets and stator slots.

#[cfg(test)]
use core::f32::consts::TAU;

/// Cogging torque model
///
/// Models cogging torque as a sum of harmonics:
/// Tcog = Σ An * sin(n * θe + φn)
///
/// Dominant harmonics are typically 6th and 12th order (for 3-phase motors).
#[derive(Debug, Clone)]
pub struct CoggingTorque {
    /// 6th harmonic amplitude [N⋅m]
    a6: f32,
    /// 12th harmonic amplitude [N⋅m]
    a12: f32,
    /// 6th harmonic phase [rad]
    phi6: f32,
    /// 12th harmonic phase [rad]
    phi12: f32,
    /// Enable/disable flag
    enabled: bool,
}

impl CoggingTorque {
    /// Create new cogging torque model
    ///
    /// # Arguments
    /// * `amplitude_6th` - 6th harmonic amplitude [N⋅m]
    /// * `amplitude_12th` - 12th harmonic amplitude [N⋅m]
    pub fn new(amplitude_6th: f32, amplitude_12th: f32) -> Self {
        Self {
            a6: amplitude_6th,
            a12: amplitude_12th,
            phi6: 0.0,
            phi12: 0.0,
            enabled: true,
        }
    }

    /// Create disabled cogging model
    pub fn disabled() -> Self {
        Self {
            a6: 0.0,
            a12: 0.0,
            phi6: 0.0,
            phi12: 0.0,
            enabled: false,
        }
    }

    /// Set phase offsets
    pub fn with_phases(mut self, phi6: f32, phi12: f32) -> Self {
        self.phi6 = phi6;
        self.phi12 = phi12;
        self
    }

    /// Create from typical motor parameters
    ///
    /// # Arguments
    /// * `cogging_ratio` - Cogging torque as fraction of rated torque (typical: 0.01-0.05)
    /// * `rated_torque` - Motor rated torque [N⋅m]
    pub fn from_ratio(cogging_ratio: f32, rated_torque: f32) -> Self {
        let total_amplitude = cogging_ratio * rated_torque;
        // Typical distribution: 70% 6th harmonic, 30% 12th harmonic
        Self::new(total_amplitude * 0.7, total_amplitude * 0.3)
    }

    /// Enable or disable cogging
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Check if enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Calculate cogging torque at given electrical angle
    ///
    /// # Arguments
    /// * `theta_e` - Electrical angle [rad]
    ///
    /// # Returns
    /// Cogging torque [N⋅m]
    pub fn calculate(&self, theta_e: f32) -> f32 {
        if !self.enabled {
            return 0.0;
        }

        // Cogging is related to slot/magnet interaction
        // 6th and 12th harmonics are dominant for 3-phase motors
        let t6 = self.a6 * libm::sinf(6.0 * theta_e + self.phi6);
        let t12 = self.a12 * libm::sinf(12.0 * theta_e + self.phi12);

        t6 + t12
    }

    /// Get peak cogging torque
    pub fn peak_torque(&self) -> f32 {
        self.a6 + self.a12
    }
}

impl Default for CoggingTorque {
    fn default() -> Self {
        // Small default cogging (typical for quality motors)
        Self::new(0.0005, 0.0002) // 0.5 mN⋅m 6th, 0.2 mN⋅m 12th
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cogging_is_periodic() {
        let cog = CoggingTorque::default();

        // Cogging should repeat every 60 degrees electrical (π/3)
        let period = TAU / 6.0;

        let t0 = cog.calculate(0.0);
        let t1 = cog.calculate(period);

        assert!(
            (t0 - t1).abs() < 0.0001,
            "Cogging should be periodic with 6th harmonic"
        );
    }

    #[test]
    fn test_cogging_zero_average() {
        let cog = CoggingTorque::new(0.001, 0.0);

        // Integrate over one electrical cycle
        let steps = 360;
        let mut sum = 0.0;
        for i in 0..steps {
            let theta = (i as f32 / steps as f32) * TAU;
            sum += cog.calculate(theta);
        }
        let average = sum / steps as f32;

        assert!(
            average.abs() < 0.0001,
            "Average cogging torque should be zero"
        );
    }

    #[test]
    fn test_disabled() {
        let cog = CoggingTorque::disabled();

        let t = cog.calculate(1.0);
        assert!((t - 0.0).abs() < 0.0001);
    }

    #[test]
    fn test_from_ratio() {
        let rated_torque = 1.0; // 1 N⋅m
        let cog = CoggingTorque::from_ratio(0.02, rated_torque); // 2% cogging

        assert!((cog.peak_torque() - 0.02).abs() < 0.001);
    }
}
