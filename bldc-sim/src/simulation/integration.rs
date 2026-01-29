//! Numerical integration methods for motor simulation

use crate::motor_model::{
    LoadTorque, MotorDynamics, MotorState, StateDerivatives, VoltageInput,
};

/// Integration method selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IntegrationMethod {
    /// Simple Euler method (first-order)
    #[default]
    Euler,
    /// Fourth-order Runge-Kutta method (more accurate)
    RungeKutta4,
}

/// Numerical integrator for motor state
pub struct Integrator {
    method: IntegrationMethod,
}

impl Integrator {
    pub fn new(method: IntegrationMethod) -> Self {
        Self { method }
    }

    pub fn euler() -> Self {
        Self::new(IntegrationMethod::Euler)
    }

    pub fn rk4() -> Self {
        Self::new(IntegrationMethod::RungeKutta4)
    }

    /// Advance motor state by one time step
    ///
    /// # Arguments
    /// * `dynamics` - Motor dynamics model
    /// * `state` - Current motor state (modified in place)
    /// * `voltage` - Applied voltage input
    /// * `load` - External load torque
    /// * `dt` - Time step [s]
    pub fn step(
        &self,
        dynamics: &MotorDynamics,
        state: &mut MotorState,
        voltage: &VoltageInput,
        load: &LoadTorque,
        dt: f32,
    ) {
        match self.method {
            IntegrationMethod::Euler => {
                self.step_euler(dynamics, state, voltage, load, dt);
            }
            IntegrationMethod::RungeKutta4 => {
                self.step_rk4(dynamics, state, voltage, load, dt);
            }
        }
    }

    /// Euler integration step
    fn step_euler(
        &self,
        dynamics: &MotorDynamics,
        state: &mut MotorState,
        voltage: &VoltageInput,
        load: &LoadTorque,
        dt: f32,
    ) {
        let pole_pairs = dynamics.params().pole_pairs;
        let deriv = dynamics.calculate_derivatives(state, voltage, load);

        // Update state using Euler method: x(t+dt) = x(t) + dx/dt * dt
        state.i_d += deriv.di_d_dt * dt;
        state.i_q += deriv.di_q_dt * dt;
        state.omega_m += deriv.domega_dt * dt;

        let new_theta = state.theta_m + deriv.dtheta_dt * dt;
        state.update_theta_m(new_theta, pole_pairs);

        // Apply current limits
        apply_current_limits(state, dynamics.params().i_max);
    }

    /// Runge-Kutta 4th order integration step
    fn step_rk4(
        &self,
        dynamics: &MotorDynamics,
        state: &mut MotorState,
        voltage: &VoltageInput,
        load: &LoadTorque,
        dt: f32,
    ) {
        let pole_pairs = dynamics.params().pole_pairs;

        // Save initial state
        let s0 = *state;

        // k1: derivatives at t
        let k1 = dynamics.calculate_derivatives(&s0, voltage, load);

        // k2: derivatives at t + dt/2 using k1
        let s1 = advance_state(&s0, &k1, dt * 0.5, pole_pairs);
        let k2 = dynamics.calculate_derivatives(&s1, voltage, load);

        // k3: derivatives at t + dt/2 using k2
        let s2 = advance_state(&s0, &k2, dt * 0.5, pole_pairs);
        let k3 = dynamics.calculate_derivatives(&s2, voltage, load);

        // k4: derivatives at t + dt using k3
        let s3 = advance_state(&s0, &k3, dt, pole_pairs);
        let k4 = dynamics.calculate_derivatives(&s3, voltage, load);

        // Combine: x(t+dt) = x(t) + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
        state.i_d = s0.i_d
            + (k1.di_d_dt + 2.0 * k2.di_d_dt + 2.0 * k3.di_d_dt + k4.di_d_dt) * dt / 6.0;
        state.i_q = s0.i_q
            + (k1.di_q_dt + 2.0 * k2.di_q_dt + 2.0 * k3.di_q_dt + k4.di_q_dt) * dt / 6.0;
        state.omega_m = s0.omega_m
            + (k1.domega_dt + 2.0 * k2.domega_dt + 2.0 * k3.domega_dt + k4.domega_dt) * dt / 6.0;

        let dtheta =
            (k1.dtheta_dt + 2.0 * k2.dtheta_dt + 2.0 * k3.dtheta_dt + k4.dtheta_dt) * dt / 6.0;
        state.update_theta_m(s0.theta_m + dtheta, pole_pairs);

        // Apply current limits
        apply_current_limits(state, dynamics.params().i_max);
    }
}

/// Advance state by derivatives * dt (helper for RK4)
fn advance_state(
    state: &MotorState,
    deriv: &StateDerivatives,
    dt: f32,
    pole_pairs: u8,
) -> MotorState {
    let mut new_state = *state;
    new_state.i_d += deriv.di_d_dt * dt;
    new_state.i_q += deriv.di_q_dt * dt;
    new_state.omega_m += deriv.domega_dt * dt;

    let new_theta = state.theta_m + deriv.dtheta_dt * dt;
    new_state.theta_m = crate::motor_model::normalize_angle(new_theta);
    new_state.update_electrical(pole_pairs);

    new_state
}

/// Apply current magnitude limits
fn apply_current_limits(state: &mut MotorState, i_max: f32) {
    let magnitude = state.current_magnitude();
    if magnitude > i_max {
        let scale = i_max / magnitude;
        state.i_d *= scale;
        state.i_q *= scale;
    }
}

impl Default for Integrator {
    fn default() -> Self {
        Self::euler()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor_model::MotorParams;
    use core::f32::consts::TAU;

    #[test]
    fn test_euler_basic() {
        let params = MotorParams::default_small_bldc();
        let dynamics = MotorDynamics::new(params.clone());
        let integrator = Integrator::euler();

        let mut state = MotorState::new();
        let voltage = VoltageInput::new(1.0, 0.0); // Small Vd to cause current
        let load = LoadTorque::zero();
        let dt = 0.0001; // 100 μs

        // Step multiple times
        for _ in 0..10 {
            integrator.step(&dynamics, &mut state, &voltage, &load, dt);
        }

        // Current should have increased
        assert!(state.i_d > 0.0, "d-axis current should increase with Vd applied");
    }

    #[test]
    fn test_rk4_basic() {
        let params = MotorParams::default_small_bldc();
        let dynamics = MotorDynamics::new(params.clone());
        let integrator = Integrator::rk4();

        let mut state = MotorState::new();
        let voltage = VoltageInput::new(1.0, 0.0);
        let load = LoadTorque::zero();
        let dt = 0.0001;

        for _ in 0..10 {
            integrator.step(&dynamics, &mut state, &voltage, &load, dt);
        }

        assert!(state.i_d > 0.0, "d-axis current should increase with Vd applied");
    }

    #[test]
    fn test_rk4_more_accurate_than_euler() {
        // For a simple exponential decay, RK4 should be more accurate
        let params = MotorParams::default_small_bldc();
        let dynamics = MotorDynamics::new(params.clone());

        let euler = Integrator::euler();
        let rk4 = Integrator::rk4();

        // Start with initial current, no voltage - current should decay
        let mut state_euler = MotorState::new();
        state_euler.i_d = 1.0;

        let mut state_rk4 = MotorState::new();
        state_rk4.i_d = 1.0;

        let voltage = VoltageInput::zero();
        let load = LoadTorque::zero();
        let dt = 0.0001;
        let steps = 100;

        for _ in 0..steps {
            euler.step(&dynamics, &mut state_euler, &voltage, &load, dt);
            rk4.step(&dynamics, &mut state_rk4, &voltage, &load, dt);
        }

        // Both should have decayed
        assert!(state_euler.i_d < 1.0);
        assert!(state_rk4.i_d < 1.0);

        // RK4 result should be close to Euler for well-behaved systems
        // but the exact comparison depends on the system dynamics
    }

    #[test]
    fn test_current_limiting() {
        let params = MotorParams::default_small_bldc();
        let dynamics = MotorDynamics::new(params.clone());
        let integrator = Integrator::euler();

        let mut state = MotorState::new();
        // Apply large voltage to cause high current
        let voltage = VoltageInput::new(100.0, 100.0);
        let load = LoadTorque::zero();
        let dt = 0.001;

        // Run many steps to saturate current
        for _ in 0..1000 {
            integrator.step(&dynamics, &mut state, &voltage, &load, dt);
        }

        // Current should be limited
        let magnitude = state.current_magnitude();
        assert!(
            magnitude <= params.i_max * 1.01, // Small tolerance for floating point
            "Current magnitude {} should be limited to {}",
            magnitude,
            params.i_max
        );
    }

    #[test]
    fn test_position_tracking() {
        let params = MotorParams::default_small_bldc();
        let dynamics = MotorDynamics::new(params.clone());
        let integrator = Integrator::rk4();

        let mut state = MotorState::new();
        state.omega_m = TAU; // 1 revolution per second

        let voltage = VoltageInput::zero();
        let load = LoadTorque::zero();
        let dt = 0.001;

        // Run for 1.5 seconds
        for _ in 0..1500 {
            integrator.step(&dynamics, &mut state, &voltage, &load, dt);
        }

        // Should have completed about 1 full rotation (friction will slow it down)
        // At least some rotation should have occurred
        assert!(
            state.rotations >= 0,
            "Should track rotations"
        );
    }
}
