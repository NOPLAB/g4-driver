//! Magnetic saturation model
//!
//! Models the reduction of flux linkage due to magnetic saturation
//! at high current levels.

/// Magnetic saturation model
///
/// Models flux linkage reduction: λm_eff = λm_nominal * (1 - k * |Iq|²)
#[derive(Debug, Clone)]
pub struct MagneticSaturation {
    /// Saturation coefficient (higher = more saturation)
    k_sat: f32,
    /// Minimum flux linkage ratio (prevents complete demagnetization)
    min_ratio: f32,
    /// Enable/disable flag
    enabled: bool,
}

impl MagneticSaturation {
    /// Create new saturation model
    ///
    /// # Arguments
    /// * `k_sat` - Saturation coefficient (typical: 0.001 to 0.01)
    pub fn new(k_sat: f32) -> Self {
        Self {
            k_sat,
            min_ratio: 0.5, // Minimum 50% flux linkage
            enabled: true,
        }
    }

    /// Create disabled saturation model
    pub fn disabled() -> Self {
        Self {
            k_sat: 0.0,
            min_ratio: 1.0,
            enabled: false,
        }
    }

    /// Set minimum flux ratio
    pub fn with_min_ratio(mut self, ratio: f32) -> Self {
        self.min_ratio = ratio.clamp(0.1, 1.0);
        self
    }

    /// Enable or disable saturation
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Check if enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Calculate effective flux linkage considering saturation
    ///
    /// # Arguments
    /// * `lambda_m_nominal` - Nominal flux linkage [Wb]
    /// * `i_q` - q-axis current [A]
    ///
    /// # Returns
    /// Effective flux linkage [Wb]
    pub fn apply(&self, lambda_m_nominal: f32, i_q: f32) -> f32 {
        if !self.enabled {
            return lambda_m_nominal;
        }

        // Saturation model: λm_eff = λm * (1 - k * Iq²)
        let saturation_factor = 1.0 - self.k_sat * i_q * i_q;
        let clamped_factor = saturation_factor.max(self.min_ratio);

        lambda_m_nominal * clamped_factor
    }

    /// Calculate saturation factor (for diagnostics)
    pub fn saturation_factor(&self, i_q: f32) -> f32 {
        if !self.enabled {
            return 1.0;
        }
        let factor = 1.0 - self.k_sat * i_q * i_q;
        factor.max(self.min_ratio)
    }
}

impl Default for MagneticSaturation {
    fn default() -> Self {
        Self::new(0.005) // Moderate saturation
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_saturation_at_zero_current() {
        let sat = MagneticSaturation::default();
        let lambda_m = 0.01;

        let effective = sat.apply(lambda_m, 0.0);
        assert!((effective - lambda_m).abs() < 0.0001);
    }

    #[test]
    fn test_saturation_increases_with_current() {
        let sat = MagneticSaturation::new(0.01);
        let lambda_m = 0.01;

        let eff_low = sat.apply(lambda_m, 1.0);
        let eff_high = sat.apply(lambda_m, 5.0);

        assert!(eff_high < eff_low, "Higher current should cause more saturation");
    }

    #[test]
    fn test_minimum_ratio_clamp() {
        let sat = MagneticSaturation::new(1.0).with_min_ratio(0.5); // Very high saturation
        let lambda_m = 0.01;

        let effective = sat.apply(lambda_m, 10.0);

        // Should be clamped to minimum ratio
        assert!(
            (effective - lambda_m * 0.5).abs() < 0.0001,
            "Should be clamped to min ratio"
        );
    }

    #[test]
    fn test_disabled() {
        let sat = MagneticSaturation::disabled();
        let lambda_m = 0.01;

        let effective = sat.apply(lambda_m, 10.0);
        assert!((effective - lambda_m).abs() < 0.0001);
    }
}
