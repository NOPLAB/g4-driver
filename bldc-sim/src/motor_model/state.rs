//! Motor state representation for simulation

use core::f32::consts::TAU;

/// Motor electrical and mechanical state
#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    // Electrical state (dq frame)
    /// d-axis current [A]
    pub i_d: f32,
    /// q-axis current [A]
    pub i_q: f32,

    // Mechanical state
    /// Mechanical angular position [rad] (0 to 2π)
    pub theta_m: f32,
    /// Mechanical angular velocity [rad/s]
    pub omega_m: f32,

    // Derived values (cached for efficiency)
    /// Electrical angle [rad] (0 to 2π)
    pub theta_e: f32,
    /// Electrical angular velocity [rad/s]
    pub omega_e: f32,

    // Cumulative position tracking
    /// Total mechanical rotations (can be negative for reverse)
    pub rotations: i32,
}

impl MotorState {
    /// Create a new motor state at rest
    pub fn new() -> Self {
        Self {
            i_d: 0.0,
            i_q: 0.0,
            theta_m: 0.0,
            omega_m: 0.0,
            theta_e: 0.0,
            omega_e: 0.0,
            rotations: 0,
        }
    }

    /// Create a motor state with initial conditions
    pub fn with_initial_conditions(
        theta_m: f32,
        omega_m: f32,
        i_d: f32,
        i_q: f32,
        pole_pairs: u8,
    ) -> Self {
        let theta_m = normalize_angle(theta_m);
        let theta_e = normalize_angle(theta_m * pole_pairs as f32);
        let omega_e = omega_m * pole_pairs as f32;

        Self {
            i_d,
            i_q,
            theta_m,
            omega_m,
            theta_e,
            omega_e,
            rotations: 0,
        }
    }

    /// Update electrical angles from mechanical state
    pub fn update_electrical(&mut self, pole_pairs: u8) {
        self.theta_e = normalize_angle(self.theta_m * pole_pairs as f32);
        self.omega_e = self.omega_m * pole_pairs as f32;
    }

    /// Update mechanical angle, handling wraparound and rotation counting
    pub fn update_theta_m(&mut self, new_theta_m: f32, pole_pairs: u8) {
        let old_theta = self.theta_m;
        self.theta_m = normalize_angle(new_theta_m);

        // Track full rotations
        if old_theta > 0.75 * TAU && self.theta_m < 0.25 * TAU {
            // Crossed 0 going forward
            self.rotations += 1;
        } else if old_theta < 0.25 * TAU && self.theta_m > 0.75 * TAU {
            // Crossed 0 going backward
            self.rotations -= 1;
        }

        self.update_electrical(pole_pairs);
    }

    /// Get current alpha-beta components from dq (requires electrical angle)
    pub fn currents_alpha_beta(&self) -> (f32, f32) {
        let cos_theta = libm::cosf(self.theta_e);
        let sin_theta = libm::sinf(self.theta_e);

        // Inverse Park transform: αβ = R(θe) * dq
        let i_alpha = self.i_d * cos_theta - self.i_q * sin_theta;
        let i_beta = self.i_d * sin_theta + self.i_q * cos_theta;

        (i_alpha, i_beta)
    }

    /// Get three-phase currents from alpha-beta
    pub fn phase_currents(&self) -> (f32, f32, f32) {
        let (i_alpha, i_beta) = self.currents_alpha_beta();

        // Inverse Clarke transform
        let i_u = i_alpha;
        let i_v = -0.5 * i_alpha + 0.866_025_4 * i_beta; // sqrt(3)/2
        let i_w = -0.5 * i_alpha - 0.866_025_4 * i_beta;

        (i_u, i_v, i_w)
    }

    /// Get speed in RPM
    pub fn speed_rpm(&self) -> f32 {
        self.omega_m * 60.0 / TAU
    }

    /// Get total position in radians (including rotations)
    pub fn total_position(&self) -> f32 {
        self.rotations as f32 * TAU + self.theta_m
    }

    /// Calculate current magnitude
    pub fn current_magnitude(&self) -> f32 {
        libm::sqrtf(self.i_d * self.i_d + self.i_q * self.i_q)
    }

    /// Reset state to initial conditions
    pub fn reset(&mut self) {
        self.i_d = 0.0;
        self.i_q = 0.0;
        self.theta_m = 0.0;
        self.omega_m = 0.0;
        self.theta_e = 0.0;
        self.omega_e = 0.0;
        self.rotations = 0;
    }
}

impl Default for MotorState {
    fn default() -> Self {
        Self::new()
    }
}

/// Normalize angle to [0, 2π)
#[inline]
pub fn normalize_angle(angle: f32) -> f32 {
    let mut a = libm::fmodf(angle, TAU);
    if a < 0.0 {
        a += TAU;
    }
    a
}

/// Snapshot of motor state for recording/analysis
#[derive(Debug, Clone, Copy)]
pub struct StateSnapshot {
    /// Simulation time [s]
    pub time: f32,
    /// d-axis current [A]
    pub i_d: f32,
    /// q-axis current [A]
    pub i_q: f32,
    /// Mechanical angle [rad]
    pub theta_m: f32,
    /// Mechanical speed [rad/s]
    pub omega_m: f32,
    /// Electrical angle [rad]
    pub theta_e: f32,
    /// Speed in RPM
    pub speed_rpm: f32,
    /// Electromagnetic torque [N⋅m]
    pub torque: f32,
    /// Total rotations
    pub rotations: i32,
}

impl StateSnapshot {
    /// Create a snapshot from current motor state
    pub fn from_state(state: &MotorState, time: f32, torque: f32) -> Self {
        Self {
            time,
            i_d: state.i_d,
            i_q: state.i_q,
            theta_m: state.theta_m,
            omega_m: state.omega_m,
            theta_e: state.theta_e,
            speed_rpm: state.speed_rpm(),
            torque,
            rotations: state.rotations,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_state() {
        let state = MotorState::new();
        assert_eq!(state.i_d, 0.0);
        assert_eq!(state.i_q, 0.0);
        assert_eq!(state.omega_m, 0.0);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(TAU) - 0.0).abs() < 0.001);
        assert!((normalize_angle(TAU + 1.0) - 1.0).abs() < 0.001);
        assert!((normalize_angle(-1.0) - (TAU - 1.0)).abs() < 0.001);
    }

    #[test]
    fn test_rotation_tracking() {
        let mut state = MotorState::new();
        let pole_pairs = 6;

        // Start at middle angle to avoid wraparound detection from 0
        state.theta_m = 0.5 * TAU;

        // Move forward to 0.9*TAU (no wraparound)
        state.update_theta_m(0.9 * TAU, pole_pairs);
        assert_eq!(state.rotations, 0, "No rotation yet, just moved forward");

        // Move from 0.9*TAU (>0.75) to 0.1*TAU (<0.25) - forward crossing 0
        state.update_theta_m(0.1 * TAU, pole_pairs);
        assert_eq!(state.rotations, 1, "Should detect forward rotation");

        // Move from 0.1*TAU (<0.25) to 0.9*TAU (>0.75) - backward crossing 0
        state.update_theta_m(0.9 * TAU, pole_pairs);
        assert_eq!(state.rotations, 0, "Should detect backward rotation");
    }

    #[test]
    fn test_speed_rpm_conversion() {
        let mut state = MotorState::new();
        state.omega_m = TAU; // 1 revolution per second = 60 RPM
        assert!((state.speed_rpm() - 60.0).abs() < 0.001);
    }

    #[test]
    fn test_current_magnitude() {
        let mut state = MotorState::new();
        state.i_d = 3.0;
        state.i_q = 4.0;
        assert!((state.current_magnitude() - 5.0).abs() < 0.001);
    }
}
