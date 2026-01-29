//! Motor electrical and mechanical dynamics equations

use super::{MotorParams, MotorState};

/// Motor dynamics calculator
///
/// Implements the electrical and mechanical equations of a PMSM/BLDC motor
/// in the dq reference frame.
#[derive(Debug, Clone)]
pub struct MotorDynamics {
    params: MotorParams,
}

/// State derivatives for numerical integration
#[derive(Debug, Clone, Copy, Default)]
pub struct StateDerivatives {
    /// Rate of change of d-axis current [A/s]
    pub di_d_dt: f32,
    /// Rate of change of q-axis current [A/s]
    pub di_q_dt: f32,
    /// Rate of change of mechanical angle [rad/s]
    pub dtheta_dt: f32,
    /// Rate of change of mechanical angular velocity [rad/s²]
    pub domega_dt: f32,
}

/// Applied voltages in dq frame
#[derive(Debug, Clone, Copy, Default)]
pub struct VoltageInput {
    /// d-axis voltage [V]
    pub v_d: f32,
    /// q-axis voltage [V]
    pub v_q: f32,
}

impl VoltageInput {
    pub fn new(v_d: f32, v_q: f32) -> Self {
        Self { v_d, v_q }
    }

    pub fn zero() -> Self {
        Self { v_d: 0.0, v_q: 0.0 }
    }
}

/// External load and disturbance torques
#[derive(Debug, Clone, Copy, Default)]
pub struct LoadTorque {
    /// Constant load torque [N⋅m]
    pub t_load: f32,
    /// Additional disturbance torque [N⋅m]
    pub t_disturbance: f32,
}

impl LoadTorque {
    pub fn new(t_load: f32) -> Self {
        Self {
            t_load,
            t_disturbance: 0.0,
        }
    }

    pub fn with_disturbance(t_load: f32, t_disturbance: f32) -> Self {
        Self {
            t_load,
            t_disturbance,
        }
    }

    pub fn zero() -> Self {
        Self::default()
    }

    pub fn total(&self) -> f32 {
        self.t_load + self.t_disturbance
    }
}

impl MotorDynamics {
    pub fn new(params: MotorParams) -> Self {
        Self { params }
    }

    pub fn params(&self) -> &MotorParams {
        &self.params
    }

    /// Calculate electromagnetic torque from dq currents
    ///
    /// Te = (3/2) * P * (λm * Iq + (Ld - Lq) * Id * Iq)
    ///
    /// For SPMSM (Ld = Lq), this simplifies to:
    /// Te = (3/2) * P * λm * Iq = Kt * Iq
    pub fn electromagnetic_torque(&self, state: &MotorState) -> f32 {
        let p = self.params.pole_pairs as f32;
        let lambda_m = self.params.lambda_m;
        let l_d = self.params.l_d;
        let l_q = self.params.l_q;

        // Full torque equation including reluctance torque
        1.5 * p * (lambda_m * state.i_q + (l_d - l_q) * state.i_d * state.i_q)
    }

    /// Calculate friction torque based on speed
    ///
    /// Combines viscous friction (proportional to speed) and Coulomb friction (constant)
    pub fn friction_torque(&self, omega_m: f32) -> f32 {
        let viscous = self.params.b * omega_m;

        // Coulomb friction with sign function, but smooth near zero
        let coulomb = if omega_m.abs() > 0.01 {
            self.params.t_friction * libm::copysignf(1.0, omega_m)
        } else {
            // Linear interpolation near zero to avoid discontinuity
            self.params.t_friction * omega_m / 0.01
        };

        viscous + coulomb
    }

    /// Calculate state derivatives (electrical and mechanical dynamics)
    ///
    /// Electrical equations (dq frame):
    /// Vd = R*Id + Ld*(dId/dt) - ωe*Lq*Iq
    /// Vq = R*Iq + Lq*(dIq/dt) + ωe*Ld*Id + ωe*λm
    ///
    /// Solving for current derivatives:
    /// dId/dt = (Vd - R*Id + ωe*Lq*Iq) / Ld
    /// dIq/dt = (Vq - R*Iq - ωe*Ld*Id - ωe*λm) / Lq
    ///
    /// Mechanical equation:
    /// J*(dω/dt) = Te - Tload - Tfriction
    pub fn calculate_derivatives(
        &self,
        state: &MotorState,
        voltage: &VoltageInput,
        load: &LoadTorque,
    ) -> StateDerivatives {
        let r = self.params.r_s;
        let l_d = self.params.l_d;
        let l_q = self.params.l_q;
        let lambda_m = self.params.lambda_m;
        let j = self.params.j;

        let omega_e = state.omega_e;

        // Electrical dynamics (dq frame)
        // dId/dt = (Vd - R*Id + ωe*Lq*Iq) / Ld
        let di_d_dt = (voltage.v_d - r * state.i_d + omega_e * l_q * state.i_q) / l_d;

        // dIq/dt = (Vq - R*Iq - ωe*Ld*Id - ωe*λm) / Lq
        let di_q_dt =
            (voltage.v_q - r * state.i_q - omega_e * l_d * state.i_d - omega_e * lambda_m) / l_q;

        // Mechanical dynamics
        let t_e = self.electromagnetic_torque(state);
        let t_friction = self.friction_torque(state.omega_m);
        let t_total_load = load.total();

        // J*(dω/dt) = Te - Tload - Tfriction
        let domega_dt = (t_e - t_total_load - t_friction) / j;

        // Position is simply velocity
        let dtheta_dt = state.omega_m;

        StateDerivatives {
            di_d_dt,
            di_q_dt,
            dtheta_dt,
            domega_dt,
        }
    }

    /// Calculate back-EMF voltage in q-axis
    ///
    /// E = ωe * λm
    pub fn back_emf(&self, omega_e: f32) -> f32 {
        omega_e * self.params.lambda_m
    }

    /// Calculate power loss components
    pub fn power_loss(&self, state: &MotorState) -> PowerLoss {
        // Copper loss: Pcu = 3/2 * R * (Id² + Iq²)
        let p_copper = 1.5 * self.params.r_s * (state.i_d * state.i_d + state.i_q * state.i_q);

        // Friction loss: Pf = Tfriction * ω
        let p_friction = self.friction_torque(state.omega_m).abs() * state.omega_m.abs();

        PowerLoss {
            copper: p_copper,
            friction: p_friction,
            total: p_copper + p_friction,
        }
    }

    /// Calculate electrical power input
    ///
    /// P = 3/2 * (Vd*Id + Vq*Iq)
    pub fn electrical_power(&self, state: &MotorState, voltage: &VoltageInput) -> f32 {
        1.5 * (voltage.v_d * state.i_d + voltage.v_q * state.i_q)
    }

    /// Calculate mechanical power output
    ///
    /// P = Te * ωm
    pub fn mechanical_power(&self, state: &MotorState) -> f32 {
        self.electromagnetic_torque(state) * state.omega_m
    }
}

/// Power loss breakdown
#[derive(Debug, Clone, Copy, Default)]
pub struct PowerLoss {
    /// Copper (I²R) losses [W]
    pub copper: f32,
    /// Friction losses [W]
    pub friction: f32,
    /// Total losses [W]
    pub total: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_params() -> MotorParams {
        MotorParams::default_small_bldc()
    }

    #[test]
    fn test_electromagnetic_torque_spmsm() {
        let params = test_params();
        let dynamics = MotorDynamics::new(params.clone());

        let mut state = MotorState::new();
        state.i_q = 1.0; // 1A q-axis current

        let torque = dynamics.electromagnetic_torque(&state);
        let expected_kt = 1.5 * params.pole_pairs as f32 * params.lambda_m;

        assert!(
            (torque - expected_kt * 1.0).abs() < 0.0001,
            "Torque should equal Kt * Iq for SPMSM"
        );
    }

    #[test]
    fn test_derivatives_at_rest() {
        let params = test_params();
        let dynamics = MotorDynamics::new(params.clone());

        let state = MotorState::new();
        let voltage = VoltageInput::new(1.0, 0.0); // Apply Vd only
        let load = LoadTorque::zero();

        let deriv = dynamics.calculate_derivatives(&state, &voltage, &load);

        // At rest with Vd applied, current should increase
        assert!(
            deriv.di_d_dt > 0.0,
            "d-axis current should increase with Vd"
        );
        // No motion yet
        assert_eq!(deriv.dtheta_dt, 0.0);
    }

    #[test]
    fn test_derivatives_with_load() {
        let params = test_params();
        let dynamics = MotorDynamics::new(params.clone());

        let mut state = MotorState::new();
        state.i_q = 1.0; // Producing torque
        state.omega_m = 10.0; // Moving
        state.update_electrical(params.pole_pairs);

        let voltage = VoltageInput::zero();
        let load = LoadTorque::new(0.1); // Large load

        let deriv = dynamics.calculate_derivatives(&state, &voltage, &load);

        // Motor should decelerate under heavy load
        // (electromagnetic torque from 1A Iq is small for these params)
        assert!(deriv.domega_dt < 0.0, "Motor should decelerate under load");
    }

    #[test]
    fn test_back_emf() {
        let params = test_params();
        let dynamics = MotorDynamics::new(params.clone());

        let omega_e = 100.0; // 100 rad/s electrical
        let emf = dynamics.back_emf(omega_e);

        assert!(
            (emf - omega_e * params.lambda_m).abs() < 0.0001,
            "Back-EMF should equal ωe * λm"
        );
    }

    #[test]
    fn test_power_balance() {
        let params = test_params();
        let dynamics = MotorDynamics::new(params.clone());

        let mut state = MotorState::new();
        state.i_d = 0.0;
        state.i_q = 2.0;
        state.omega_m = 50.0;
        state.update_electrical(params.pole_pairs);

        // Steady state voltage that maintains current
        let back_emf = dynamics.back_emf(state.omega_e);
        let voltage = VoltageInput::new(params.r_s * state.i_d, params.r_s * state.i_q + back_emf);

        let p_elec = dynamics.electrical_power(&state, &voltage);
        let p_mech = dynamics.mechanical_power(&state);
        let p_loss = dynamics.power_loss(&state);

        // Power balance: Pelec = Pmech + Ploss
        let balance_error = (p_elec - p_mech - p_loss.total).abs();
        assert!(
            balance_error < 0.1,
            "Power balance should be satisfied: Pelec={}, Pmech={}, Ploss={}",
            p_elec,
            p_mech,
            p_loss.total
        );
    }

    #[test]
    fn test_friction_sign() {
        let params = test_params();
        let dynamics = MotorDynamics::new(params);

        // Friction should oppose motion
        let friction_pos = dynamics.friction_torque(10.0);
        let friction_neg = dynamics.friction_torque(-10.0);

        assert!(friction_pos > 0.0, "Friction should oppose positive speed");
        assert!(friction_neg < 0.0, "Friction should oppose negative speed");
    }
}
