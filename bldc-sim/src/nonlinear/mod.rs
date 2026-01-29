//! Nonlinear effects module
//!
//! Models real-world nonlinearities in BLDC motors:
//! - Magnetic saturation
//! - Cogging torque
//! - Dead-time effects

mod cogging;
mod dead_time;
mod saturation;

pub use cogging::CoggingTorque;
pub use dead_time::DeadTimeEffect;
pub use saturation::MagneticSaturation;

/// Combined nonlinear effects container
#[derive(Debug, Clone)]
pub struct NonlinearEffects {
    /// Magnetic saturation model
    pub saturation: MagneticSaturation,
    /// Cogging torque model
    pub cogging: CoggingTorque,
    /// Dead-time effect model
    pub dead_time: DeadTimeEffect,
}

impl NonlinearEffects {
    /// Create with all effects disabled
    pub fn new() -> Self {
        Self {
            saturation: MagneticSaturation::disabled(),
            cogging: CoggingTorque::disabled(),
            dead_time: DeadTimeEffect::disabled(),
        }
    }

    /// Create with all effects enabled (default parameters)
    pub fn all_enabled() -> Self {
        Self {
            saturation: MagneticSaturation::default(),
            cogging: CoggingTorque::default(),
            dead_time: DeadTimeEffect::default(),
        }
    }

    /// Create with custom models
    pub fn with_models(
        saturation: MagneticSaturation,
        cogging: CoggingTorque,
        dead_time: DeadTimeEffect,
    ) -> Self {
        Self {
            saturation,
            cogging,
            dead_time,
        }
    }

    /// Check if any effect is enabled
    pub fn any_enabled(&self) -> bool {
        self.saturation.is_enabled()
            || self.cogging.is_enabled()
            || self.dead_time.is_enabled()
    }

    /// Enable or disable all effects
    pub fn set_all_enabled(&mut self, enabled: bool) {
        self.saturation.set_enabled(enabled);
        self.cogging.set_enabled(enabled);
        self.dead_time.set_enabled(enabled);
    }

    /// Calculate total additional torque from nonlinear effects
    ///
    /// # Arguments
    /// * `theta_e` - Electrical angle [rad]
    ///
    /// # Returns
    /// Additional torque [N⋅m] (cogging contribution)
    pub fn additional_torque(&self, theta_e: f32) -> f32 {
        self.cogging.calculate(theta_e)
    }

    /// Calculate effective flux linkage with saturation
    ///
    /// # Arguments
    /// * `lambda_m_nominal` - Nominal flux linkage [Wb]
    /// * `i_q` - q-axis current [A]
    ///
    /// # Returns
    /// Effective flux linkage [Wb]
    pub fn effective_flux(&self, lambda_m_nominal: f32, i_q: f32) -> f32 {
        self.saturation.apply(lambda_m_nominal, i_q)
    }
}

impl Default for NonlinearEffects {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_disabled() {
        let effects = NonlinearEffects::new();
        assert!(!effects.any_enabled());
    }

    #[test]
    fn test_all_enabled() {
        let effects = NonlinearEffects::all_enabled();
        assert!(effects.any_enabled());
    }

    #[test]
    fn test_toggle_all() {
        let mut effects = NonlinearEffects::all_enabled();
        effects.set_all_enabled(false);
        assert!(!effects.any_enabled());
    }
}
