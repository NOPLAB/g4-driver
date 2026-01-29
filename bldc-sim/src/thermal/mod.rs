//! Thermal model module
//!
//! Models temperature effects on motor performance:
//! - Winding temperature dynamics
//! - Temperature-dependent resistance
//! - Thermal protection limits

mod model;

pub use model::ThermalModel;

/// Thermal state of the motor
#[derive(Debug, Clone, Copy)]
pub struct ThermalState {
    /// Winding temperature [°C]
    pub winding_temp: f32,
    /// Ambient temperature [°C]
    pub ambient_temp: f32,
}

impl ThermalState {
    /// Create new thermal state at ambient
    pub fn new(ambient_temp: f32) -> Self {
        Self {
            winding_temp: ambient_temp,
            ambient_temp,
        }
    }

    /// Create at specific ambient temperature
    pub fn at_ambient(temp: f32) -> Self {
        Self::new(temp)
    }

    /// Update winding temperature
    pub fn update(&mut self, model: &ThermalModel, p_loss: f32, dt: f32) {
        self.winding_temp = model.step(
            self.winding_temp,
            p_loss,
            self.ambient_temp,
            dt,
        );
    }

    /// Get temperature rise above ambient
    pub fn temp_rise(&self) -> f32 {
        self.winding_temp - self.ambient_temp
    }
}

impl Default for ThermalState {
    fn default() -> Self {
        Self::new(25.0) // 25°C default ambient
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_state() {
        let state = ThermalState::new(30.0);
        assert!((state.winding_temp - 30.0).abs() < 0.001);
        assert!((state.ambient_temp - 30.0).abs() < 0.001);
    }

    #[test]
    fn test_temp_rise() {
        let mut state = ThermalState::new(25.0);
        state.winding_temp = 75.0;

        assert!((state.temp_rise() - 50.0).abs() < 0.001);
    }
}
