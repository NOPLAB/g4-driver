//! Thermal model for motor simulation
//!
//! Models temperature dynamics and temperature-dependent parameters.

/// Thermal model for motor windings
///
/// Uses a simplified first-order thermal model:
/// dT/dt = (P_loss - (T - T_ambient) / R_th) / C_th
#[derive(Debug, Clone)]
pub struct ThermalModel {
    /// Thermal resistance [K/W]
    r_th: f32,
    /// Thermal capacitance [J/K]
    c_th: f32,
    /// Temperature coefficient of resistance [1/K]
    alpha: f32,
    /// Reference temperature [°C]
    t_ref: f32,
    /// Maximum allowed temperature [°C]
    t_max: f32,
    /// Enable/disable flag
    enabled: bool,
}

impl ThermalModel {
    /// Create new thermal model
    ///
    /// # Arguments
    /// * `r_th` - Thermal resistance [K/W]
    /// * `c_th` - Thermal capacitance [J/K]
    pub fn new(r_th: f32, c_th: f32) -> Self {
        Self {
            r_th,
            c_th,
            alpha: 0.00393, // Copper temperature coefficient
            t_ref: 20.0,    // Reference at 20°C
            t_max: 120.0,   // Typical max winding temp
            enabled: true,
        }
    }

    /// Create disabled thermal model
    pub fn disabled() -> Self {
        Self {
            r_th: 1.0,
            c_th: 1.0,
            alpha: 0.0,
            t_ref: 20.0,
            t_max: 200.0,
            enabled: false,
        }
    }

    /// Set temperature coefficient of resistance
    pub fn with_alpha(mut self, alpha: f32) -> Self {
        self.alpha = alpha;
        self
    }

    /// Set maximum temperature
    pub fn with_max_temp(mut self, t_max: f32) -> Self {
        self.t_max = t_max;
        self
    }

    /// Enable or disable
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Check if enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Calculate resistance at given temperature
    ///
    /// R(T) = R_ref * (1 + α * (T - T_ref))
    ///
    /// # Arguments
    /// * `r_ref` - Reference resistance at T_ref [Ω]
    /// * `temperature` - Current temperature [°C]
    ///
    /// # Returns
    /// Resistance at current temperature [Ω]
    pub fn resistance_at_temp(&self, r_ref: f32, temperature: f32) -> f32 {
        if !self.enabled {
            return r_ref;
        }
        r_ref * (1.0 + self.alpha * (temperature - self.t_ref))
    }

    /// Calculate temperature derivative
    ///
    /// dT/dt = (P_loss - (T - T_ambient) / R_th) / C_th
    ///
    /// # Arguments
    /// * `p_loss` - Power loss [W]
    /// * `temperature` - Current winding temperature [°C]
    /// * `ambient` - Ambient temperature [°C]
    ///
    /// # Returns
    /// Rate of temperature change [°C/s]
    pub fn temperature_derivative(&self, p_loss: f32, temperature: f32, ambient: f32) -> f32 {
        if !self.enabled {
            return 0.0;
        }

        let heat_dissipation = (temperature - ambient) / self.r_th;
        (p_loss - heat_dissipation) / self.c_th
    }

    /// Integrate temperature over time step
    ///
    /// # Arguments
    /// * `temperature` - Current temperature [°C]
    /// * `p_loss` - Power loss [W]
    /// * `ambient` - Ambient temperature [°C]
    /// * `dt` - Time step [s]
    ///
    /// # Returns
    /// New temperature [°C]
    pub fn step(&self, temperature: f32, p_loss: f32, ambient: f32, dt: f32) -> f32 {
        if !self.enabled {
            return temperature;
        }

        let d_temp = self.temperature_derivative(p_loss, temperature, ambient);
        let new_temp = temperature + d_temp * dt;

        // Clamp to physical limits
        new_temp.clamp(ambient, self.t_max)
    }

    /// Calculate steady-state temperature for given power loss
    ///
    /// T_ss = T_ambient + P_loss * R_th
    pub fn steady_state_temp(&self, p_loss: f32, ambient: f32) -> f32 {
        ambient + p_loss * self.r_th
    }

    /// Calculate thermal time constant
    ///
    /// τ = R_th * C_th
    pub fn time_constant(&self) -> f32 {
        self.r_th * self.c_th
    }

    /// Check if temperature exceeds limit
    pub fn is_over_temp(&self, temperature: f32) -> bool {
        temperature > self.t_max
    }
}

impl Default for ThermalModel {
    fn default() -> Self {
        // Typical small motor values
        Self::new(
            5.0,   // 5 K/W thermal resistance
            100.0, // 100 J/K thermal capacitance
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_resistance_at_ref_temp() {
        let model = ThermalModel::default();
        let r_ref = 1.0;

        let r = model.resistance_at_temp(r_ref, 20.0);
        assert!((r - r_ref).abs() < 0.001);
    }

    #[test]
    fn test_resistance_increases_with_temp() {
        let model = ThermalModel::default();
        let r_ref = 1.0;

        let r_20 = model.resistance_at_temp(r_ref, 20.0);
        let r_100 = model.resistance_at_temp(r_ref, 100.0);

        assert!(r_100 > r_20, "Resistance should increase with temperature");
    }

    #[test]
    fn test_temperature_rise() {
        let model = ThermalModel::default();

        // Apply constant power loss
        let p_loss = 10.0; // 10W
        let ambient = 25.0;
        let mut temp = ambient;
        let dt = 0.1;

        // Simulate for some time
        for _ in 0..100 {
            temp = model.step(temp, p_loss, ambient, dt);
        }

        // Temperature should have risen
        assert!(temp > ambient);
    }

    #[test]
    fn test_steady_state() {
        let model = ThermalModel::default();

        let p_loss = 10.0;
        let ambient = 25.0;

        let t_ss = model.steady_state_temp(p_loss, ambient);

        // T_ss = 25 + 10 * 5 = 75°C
        assert!((t_ss - 75.0).abs() < 0.1);
    }

    #[test]
    fn test_time_constant() {
        let model = ThermalModel::new(5.0, 100.0);

        let tau = model.time_constant();

        // τ = 5 * 100 = 500s
        assert!((tau - 500.0).abs() < 0.1);
    }

    #[test]
    fn test_disabled() {
        let model = ThermalModel::disabled();

        let temp = 25.0;
        let new_temp = model.step(temp, 100.0, 20.0, 1.0);

        // Temperature should not change
        assert!((new_temp - temp).abs() < 0.001);
    }

    #[test]
    fn test_cooling() {
        let model = ThermalModel::default();

        // Start hot, no power loss
        let mut temp = 100.0;
        let ambient = 25.0;
        let p_loss = 0.0;
        let dt = 1.0;

        for _ in 0..100 {
            temp = model.step(temp, p_loss, ambient, dt);
        }

        // Should cool down towards ambient
        assert!(temp < 100.0);
    }
}
