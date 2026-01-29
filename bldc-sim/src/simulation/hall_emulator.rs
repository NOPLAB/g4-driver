//! Hall sensor emulator for motor simulation
//!
//! Generates realistic Hall sensor signals from motor mechanical angle.

use core::f32::consts::TAU;

/// Hall sensor sequence for CW rotation (standard BLDC commutation)
/// Maps normalized Hall index (0-5) to Hall state (1-6)
const HALL_SEQUENCE: [u8; 6] = [
    1, // Index 0: Hall state 1
    3, // Index 1: Hall state 3
    2, // Index 2: Hall state 2
    6, // Index 3: Hall state 6
    4, // Index 4: Hall state 4
    5, // Index 5: Hall state 5
];

/// Hall sensor emulator
///
/// Generates Hall sensor states and timing information from mechanical angle
/// for use with the bldc crate's HallProcessor.
#[derive(Debug, Clone)]
pub struct HallEmulator {
    /// Number of pole pairs
    #[allow(dead_code)]
    pole_pairs: u8,
    /// Total Hall states per mechanical revolution (pole_pairs * 6)
    states_per_rev: u32,
    /// Angle covered by each Hall state [rad]
    angle_per_state: f32,
    /// Previous Hall state index
    prev_hall_index: u32,
    /// Time since last Hall edge [s]
    time_since_edge: f32,
    /// Flag indicating first sample
    is_first: bool,
}

impl HallEmulator {
    /// Create a new Hall emulator
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of motor pole pairs
    pub fn new(pole_pairs: u8) -> Self {
        let states_per_rev = pole_pairs as u32 * 6;
        Self {
            pole_pairs,
            states_per_rev,
            angle_per_state: TAU / states_per_rev as f32,
            prev_hall_index: 0,
            time_since_edge: 0.0,
            is_first: true,
        }
    }

    /// Update emulator with new mechanical angle
    ///
    /// # Arguments
    /// * `theta_m` - Mechanical angle [rad] (0 to 2π)
    /// * `omega_m` - Mechanical angular velocity [rad/s]
    /// * `dt` - Time step [s]
    ///
    /// # Returns
    /// `HallOutput` containing state, speed, and edge detection
    pub fn update(&mut self, theta_m: f32, omega_m: f32, dt: f32) -> HallOutput {
        // Calculate Hall state index from mechanical angle
        let hall_index = self.calculate_hall_index(theta_m);

        // Detect state change
        let state_changed = if self.is_first {
            self.is_first = false;
            self.prev_hall_index = hall_index;
            false
        } else {
            hall_index != self.prev_hall_index
        };

        // Calculate instantaneous speed from edge timing
        let instant_speed_rpm = if state_changed {
            let speed = self.calculate_speed_from_edge(omega_m);
            self.time_since_edge = 0.0;
            self.prev_hall_index = hall_index;
            speed
        } else {
            self.time_since_edge += dt;
            0.0 // Only report speed on edges
        };

        // Check for timeout (no edges for extended period)
        let is_timeout = self.time_since_edge > 0.1; // 100ms timeout

        // Map index to Hall state
        let hall_state = self.index_to_state(hall_index);

        HallOutput {
            hall_state,
            instant_speed_rpm,
            is_timeout,
            state_changed,
        }
    }

    /// Reset emulator state
    pub fn reset(&mut self) {
        self.prev_hall_index = 0;
        self.time_since_edge = 0.0;
        self.is_first = true;
    }

    /// Calculate Hall index from mechanical angle
    fn calculate_hall_index(&self, theta_m: f32) -> u32 {
        // Normalize angle to [0, 2π)
        let normalized = if theta_m < 0.0 {
            theta_m + TAU
        } else if theta_m >= TAU {
            theta_m - TAU
        } else {
            theta_m
        };

        // Calculate which Hall state we're in
        let index = (normalized / self.angle_per_state) as u32;
        index.min(self.states_per_rev - 1)
    }

    /// Convert Hall index to Hall state (1-6)
    fn index_to_state(&self, index: u32) -> u8 {
        // Get position within electrical cycle (0-5)
        let electrical_index = (index % 6) as usize;
        HALL_SEQUENCE[electrical_index]
    }

    /// Calculate speed from time between edges
    fn calculate_speed_from_edge(&self, omega_m: f32) -> f32 {
        // Convert mechanical angular velocity to RPM
        // omega_m is in rad/s, convert to RPM
        omega_m.abs() * 60.0 / TAU
    }
}

/// Output from Hall emulator
#[derive(Debug, Clone, Copy, Default)]
pub struct HallOutput {
    /// Hall sensor state (1-6)
    pub hall_state: u8,
    /// Instantaneous speed calculated from edge timing [RPM]
    /// Only non-zero when state_changed is true
    pub instant_speed_rpm: f32,
    /// True if no edges detected for extended period
    pub is_timeout: bool,
    /// True if state changed since last update
    pub state_changed: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_emulator() {
        let emulator = HallEmulator::new(6);
        assert_eq!(emulator.pole_pairs, 6);
        assert_eq!(emulator.states_per_rev, 36); // 6 pole pairs * 6 states
    }

    #[test]
    fn test_hall_state_range() {
        let mut emulator = HallEmulator::new(6);

        // Test across full mechanical revolution
        for i in 0..360 {
            let angle = (i as f32) * TAU / 360.0;
            let output = emulator.update(angle, 10.0, 0.001);

            assert!(
                (1..=6).contains(&output.hall_state),
                "Invalid Hall state {} at angle {}",
                output.hall_state,
                angle
            );
        }
    }

    #[test]
    fn test_state_changes() {
        let mut emulator = HallEmulator::new(6);

        let dt = 0.0001;
        let omega = TAU; // 1 rev/sec
        let mut state_changes = 0;

        // Simulate one full revolution
        let steps = 10000;
        for i in 0..steps {
            let angle = (i as f32 / steps as f32) * TAU;
            let output = emulator.update(angle, omega, dt);
            if output.state_changed {
                state_changes += 1;
            }
        }

        // Should have 36 state changes per revolution (6 pole pairs * 6 states)
        // Minus 1 because first reading doesn't count
        assert!(
            state_changes >= 35 && state_changes <= 36,
            "Expected ~36 state changes, got {}",
            state_changes
        );
    }

    #[test]
    fn test_speed_on_edges() {
        let mut emulator = HallEmulator::new(6);

        let omega = TAU * 10.0; // 10 rev/sec = 600 RPM
        let expected_rpm = 600.0;

        // First update
        emulator.update(0.0, omega, 0.001);

        // Move to next Hall state
        let angle_per_state = TAU / 36.0;
        let output = emulator.update(angle_per_state * 1.1, omega, 0.001);

        if output.state_changed {
            // Speed should be close to expected
            assert!(
                (output.instant_speed_rpm - expected_rpm).abs() < 10.0,
                "Expected ~{} RPM, got {}",
                expected_rpm,
                output.instant_speed_rpm
            );
        }
    }

    #[test]
    fn test_timeout_detection() {
        let mut emulator = HallEmulator::new(6);

        // Initial update
        emulator.update(0.0, 0.0, 0.001);

        // Many updates without state change (stationary motor)
        for _ in 0..150 {
            let output = emulator.update(0.0, 0.0, 0.001);
            if output.is_timeout {
                return; // Test passed
            }
        }

        panic!("Timeout should have been detected");
    }

    #[test]
    fn test_reset() {
        let mut emulator = HallEmulator::new(6);

        emulator.update(1.0, 10.0, 0.001);
        emulator.update(2.0, 10.0, 0.001);

        emulator.reset();

        assert!(emulator.is_first);
        assert!((emulator.time_since_edge - 0.0).abs() < 0.0001);
    }
}
