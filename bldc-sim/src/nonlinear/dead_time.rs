//! Dead-time effect model
//!
//! Models the voltage distortion caused by inverter dead-time,
//! which depends on current direction.

/// Dead-time effect model
///
/// Dead-time causes voltage distortion: ΔV = ±Vdc * (t_dead / T_pwm)
/// The sign depends on current direction in each phase.
#[derive(Debug, Clone)]
pub struct DeadTimeEffect {
    /// Dead-time duration [s]
    t_dead: f32,
    /// PWM period [s]
    t_pwm: f32,
    /// DC bus voltage [V]
    v_dc: f32,
    /// Enable/disable flag
    enabled: bool,
}

impl DeadTimeEffect {
    /// Create new dead-time effect model
    ///
    /// # Arguments
    /// * `t_dead` - Dead-time duration [s] (typical: 0.5-2 μs)
    /// * `t_pwm` - PWM period [s] (1/f_pwm)
    /// * `v_dc` - DC bus voltage [V]
    pub fn new(t_dead: f32, t_pwm: f32, v_dc: f32) -> Self {
        Self {
            t_dead,
            t_pwm,
            v_dc,
            enabled: true,
        }
    }

    /// Create from PWM frequency
    ///
    /// # Arguments
    /// * `t_dead` - Dead-time duration [s]
    /// * `f_pwm` - PWM frequency [Hz]
    /// * `v_dc` - DC bus voltage [V]
    pub fn from_frequency(t_dead: f32, f_pwm: f32, v_dc: f32) -> Self {
        Self::new(t_dead, 1.0 / f_pwm, v_dc)
    }

    /// Create disabled model
    pub fn disabled() -> Self {
        Self {
            t_dead: 0.0,
            t_pwm: 1.0,
            v_dc: 0.0,
            enabled: false,
        }
    }

    /// Enable or disable
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Check if enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Calculate voltage distortion for a single phase
    ///
    /// # Arguments
    /// * `phase_current` - Phase current [A]
    ///
    /// # Returns
    /// Voltage distortion [V]
    pub fn phase_distortion(&self, phase_current: f32) -> f32 {
        if !self.enabled {
            return 0.0;
        }

        // Dead-time voltage error depends on current direction
        let delta_v = self.v_dc * self.t_dead / self.t_pwm;

        // Smooth transition near zero current to avoid discontinuity
        let threshold = 0.1; // 100 mA threshold
        if phase_current.abs() < threshold {
            // Linear interpolation
            delta_v * phase_current / threshold
        } else {
            // Full effect with sign
            delta_v * libm::copysignf(1.0, phase_current)
        }
    }

    /// Calculate αβ-frame voltage distortion
    ///
    /// # Arguments
    /// * `i_u`, `i_v`, `i_w` - Phase currents [A]
    ///
    /// # Returns
    /// (v_alpha_distortion, v_beta_distortion) [V]
    pub fn alpha_beta_distortion(&self, i_u: f32, i_v: f32, i_w: f32) -> (f32, f32) {
        if !self.enabled {
            return (0.0, 0.0);
        }

        let delta_u = self.phase_distortion(i_u);
        let delta_v = self.phase_distortion(i_v);
        let delta_w = self.phase_distortion(i_w);

        // Clarke transform
        let delta_alpha = delta_u;
        let delta_beta = (delta_v - delta_w) / libm::sqrtf(3.0);

        (delta_alpha, delta_beta)
    }

    /// Calculate dq-frame voltage distortion
    ///
    /// # Arguments
    /// * `i_u`, `i_v`, `i_w` - Phase currents [A]
    /// * `theta_e` - Electrical angle [rad]
    ///
    /// # Returns
    /// (v_d_distortion, v_q_distortion) [V]
    pub fn dq_distortion(&self, i_u: f32, i_v: f32, i_w: f32, theta_e: f32) -> (f32, f32) {
        if !self.enabled {
            return (0.0, 0.0);
        }

        let (delta_alpha, delta_beta) = self.alpha_beta_distortion(i_u, i_v, i_w);

        // Park transform
        let cos_theta = libm::cosf(theta_e);
        let sin_theta = libm::sinf(theta_e);

        let delta_d = delta_alpha * cos_theta + delta_beta * sin_theta;
        let delta_q = -delta_alpha * sin_theta + delta_beta * cos_theta;

        (delta_d, delta_q)
    }

    /// Get dead-time ratio (t_dead / t_pwm)
    pub fn dead_time_ratio(&self) -> f32 {
        self.t_dead / self.t_pwm
    }

    /// Get maximum voltage distortion
    pub fn max_distortion(&self) -> f32 {
        self.v_dc * self.dead_time_ratio()
    }
}

impl Default for DeadTimeEffect {
    fn default() -> Self {
        // Typical values: 1 μs dead-time, 50 kHz PWM, 24V DC
        Self::from_frequency(1e-6, 50e3, 24.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_positive_current_distortion() {
        let dt = DeadTimeEffect::default();

        let distortion = dt.phase_distortion(5.0);

        // Should be positive (opposes positive current)
        assert!(distortion > 0.0);
    }

    #[test]
    fn test_negative_current_distortion() {
        let dt = DeadTimeEffect::default();

        let distortion = dt.phase_distortion(-5.0);

        // Should be negative
        assert!(distortion < 0.0);
    }

    #[test]
    fn test_symmetry() {
        let dt = DeadTimeEffect::default();

        let pos = dt.phase_distortion(5.0);
        let neg = dt.phase_distortion(-5.0);

        assert!((pos + neg).abs() < 0.0001, "Distortion should be symmetric");
    }

    #[test]
    fn test_smooth_zero_crossing() {
        let dt = DeadTimeEffect::default();

        // Near zero, distortion should be small
        let near_zero = dt.phase_distortion(0.01);
        assert!(near_zero.abs() < dt.max_distortion() * 0.2);
    }

    #[test]
    fn test_disabled() {
        let dt = DeadTimeEffect::disabled();

        let distortion = dt.phase_distortion(10.0);
        assert!((distortion - 0.0).abs() < 0.0001);
    }

    #[test]
    fn test_max_distortion() {
        let dt = DeadTimeEffect::from_frequency(1e-6, 50e3, 24.0);

        let max = dt.max_distortion();
        let expected = 24.0 * 1e-6 * 50e3; // = 1.2V

        assert!((max - expected).abs() < 0.01);
    }
}
