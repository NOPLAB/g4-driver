//! Motor electrical and mechanical parameters

/// Motor electrical and mechanical parameters for simulation
#[derive(Debug, Clone)]
pub struct MotorParams {
    // Electrical parameters
    /// Stator resistance per phase [Ohm]
    pub r_s: f32,
    /// d-axis inductance [H]
    pub l_d: f32,
    /// q-axis inductance [H]
    pub l_q: f32,
    /// Permanent magnet flux linkage [Wb]
    pub lambda_m: f32,

    // Mechanical parameters
    /// Rotor inertia [kg⋅m²]
    pub j: f32,
    /// Viscous friction coefficient [N⋅m⋅s/rad]
    pub b: f32,
    /// Coulomb friction torque [N⋅m]
    pub t_friction: f32,

    // Motor construction
    /// Number of pole pairs
    pub pole_pairs: u8,

    // Operating limits
    /// Maximum phase current [A]
    pub i_max: f32,
    /// DC bus voltage [V]
    pub v_dc: f32,
    /// Maximum speed [rad/s mechanical]
    pub omega_max: f32,
}

impl MotorParams {
    /// Create motor parameters based on Maxon EC-i 40 100W 48V
    /// High-precision brushless DC motor with Hall sensors
    /// Datasheet: https://www.maxongroup.com/maxon/view/product/488607
    pub fn default_small_bldc() -> Self {
        Self {
            // Electrical (Maxon EC-i 40 100W 48V)
            // Terminal resistance 0.994 Ω (phase-to-phase) → 0.497 Ω per phase (Y-connection)
            r_s: 0.497,
            // Terminal inductance 0.995 mH (phase-to-phase) → 0.4975 mH per phase
            l_d: 0.0004975,
            l_q: 0.0004975, // SPMSM: Ld ≈ Lq
            // Flux linkage calculated from torque constant: Kt = 91 mNm/A
            // λm = Kt / (1.5 * P) = 0.091 / (1.5 * 7) ≈ 8.67 mWb
            lambda_m: 0.00867,

            // Mechanical
            j: 0.0000044,      // 44 g⋅cm² = 44e-7 kg⋅m² rotor inertia
            b: 0.00001,        // Small viscous friction (estimated)
            t_friction: 0.001, // 1 mN⋅m Coulomb friction (estimated)

            // Construction (Maxon EC-i 40)
            pole_pairs: 7,

            // Limits (Maxon EC-i 40 100W 48V)
            i_max: 10.0, // 10A max continuous (conservative)
            v_dc: 48.0,  // 48V DC bus
            omega_max: 8000.0 * core::f32::consts::TAU / 60.0, // 8000 RPM max
        }
    }

    /// Create a builder for custom motor parameters
    pub fn builder() -> MotorParamsBuilder {
        MotorParamsBuilder::new()
    }

    /// Calculate electrical angular velocity from mechanical
    #[inline]
    pub fn omega_e(&self, omega_m: f32) -> f32 {
        omega_m * self.pole_pairs as f32
    }

    /// Calculate mechanical angular velocity from electrical
    #[inline]
    pub fn omega_m(&self, omega_e: f32) -> f32 {
        omega_e / self.pole_pairs as f32
    }

    /// Calculate electrical torque constant Kt = (3/2) * P * lambda_m
    /// For SPMSM (Ld = Lq), torque is proportional to Iq only
    #[inline]
    pub fn kt(&self) -> f32 {
        1.5 * self.pole_pairs as f32 * self.lambda_m
    }

    /// Calculate back-EMF constant Ke = P * lambda_m [V/(rad/s electrical)]
    #[inline]
    pub fn ke(&self) -> f32 {
        self.pole_pairs as f32 * self.lambda_m
    }

    /// Convert mechanical speed in RPM to rad/s
    #[inline]
    pub fn rpm_to_rad_s(rpm: f32) -> f32 {
        rpm * core::f32::consts::TAU / 60.0
    }

    /// Convert mechanical speed in rad/s to RPM
    #[inline]
    pub fn rad_s_to_rpm(rad_s: f32) -> f32 {
        rad_s * 60.0 / core::f32::consts::TAU
    }
}

impl Default for MotorParams {
    fn default() -> Self {
        Self::default_small_bldc()
    }
}

/// Builder for MotorParams with validation
#[derive(Debug, Clone)]
pub struct MotorParamsBuilder {
    params: MotorParams,
}

impl MotorParamsBuilder {
    pub fn new() -> Self {
        Self {
            params: MotorParams::default_small_bldc(),
        }
    }

    pub fn resistance(mut self, r_s: f32) -> Self {
        self.params.r_s = r_s;
        self
    }

    pub fn inductance_d(mut self, l_d: f32) -> Self {
        self.params.l_d = l_d;
        self
    }

    pub fn inductance_q(mut self, l_q: f32) -> Self {
        self.params.l_q = l_q;
        self
    }

    /// Set both d and q axis inductance (for SPMSM where Ld = Lq)
    pub fn inductance(mut self, l: f32) -> Self {
        self.params.l_d = l;
        self.params.l_q = l;
        self
    }

    pub fn flux_linkage(mut self, lambda_m: f32) -> Self {
        self.params.lambda_m = lambda_m;
        self
    }

    pub fn inertia(mut self, j: f32) -> Self {
        self.params.j = j;
        self
    }

    pub fn viscous_friction(mut self, b: f32) -> Self {
        self.params.b = b;
        self
    }

    pub fn coulomb_friction(mut self, t_friction: f32) -> Self {
        self.params.t_friction = t_friction;
        self
    }

    pub fn pole_pairs(mut self, pole_pairs: u8) -> Self {
        self.params.pole_pairs = pole_pairs;
        self
    }

    pub fn max_current(mut self, i_max: f32) -> Self {
        self.params.i_max = i_max;
        self
    }

    pub fn dc_voltage(mut self, v_dc: f32) -> Self {
        self.params.v_dc = v_dc;
        self
    }

    pub fn max_speed_rpm(mut self, rpm: f32) -> Self {
        self.params.omega_max = MotorParams::rpm_to_rad_s(rpm);
        self
    }

    pub fn build(self) -> MotorParams {
        self.params
    }
}

impl Default for MotorParamsBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_params() {
        let params = MotorParams::default();
        // Maxon EC-i 40 has 7 pole pairs
        assert_eq!(params.pole_pairs, 7);
        assert!(params.r_s > 0.0);
        assert!(params.l_d > 0.0);
        assert!(params.lambda_m > 0.0);
    }

    #[test]
    fn test_speed_conversion() {
        let rpm = 1000.0;
        let rad_s = MotorParams::rpm_to_rad_s(rpm);
        let back_to_rpm = MotorParams::rad_s_to_rpm(rad_s);
        assert!((rpm - back_to_rpm).abs() < 0.001);
    }

    #[test]
    fn test_omega_conversion() {
        let params = MotorParams::default();
        let omega_m = 100.0; // rad/s mechanical
        let omega_e = params.omega_e(omega_m);
        assert!((omega_e - omega_m * params.pole_pairs as f32).abs() < 0.001);

        let back_to_omega_m = params.omega_m(omega_e);
        assert!((omega_m - back_to_omega_m).abs() < 0.001);
    }

    #[test]
    fn test_builder() {
        let params = MotorParams::builder()
            .resistance(1.0)
            .inductance(0.001)
            .pole_pairs(4)
            .build();

        assert_eq!(params.r_s, 1.0);
        assert_eq!(params.l_d, 0.001);
        assert_eq!(params.l_q, 0.001);
        assert_eq!(params.pole_pairs, 4);
    }
}
