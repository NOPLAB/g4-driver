// Hall state machine for mechanical angle calculation
// Handles Hall state transitions and mechanical angle tracking

use super::constants::{HALL_STATE_TABLE, INVALID_STATE};
use super::utils::normalize_angle;
use core::f32::consts::TAU;

/// Result of processing a Hall state
pub struct StateTransition {
    /// Whether the state changed from the previous reading
    pub state_changed: bool,
    /// Whether this is the first valid state reading
    pub is_first_reading: bool,
    /// Current normalized state (0-5)
    pub normalized_state: u8,
}

/// Hall state machine for tracking motor position
/// Implements foc-simple compatible mechanical angle based calculation
pub struct HallStateMachine {
    /// Previous normalized hall state (0-5), 255 = invalid/initial
    prev_state: u8,
    /// Hall index base (increments by 6 each electrical revolution)
    hall_idx_base: u32,
    /// Maximum hall index (pole_pairs * 6)
    hall_idx_max: u32,
    /// Angle per hall state (mechanical angle) = TAU / hall_idx_max
    angle_per_state: f32,
    /// Current mechanical angle in radians
    mechanical_angle: f32,
}

impl HallStateMachine {
    /// Create a new Hall state machine
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of pole pairs in the motor
    pub fn new(pole_pairs: u8) -> Self {
        let hall_idx_max = (pole_pairs as u32) * 6;
        let angle_per_state = TAU / (hall_idx_max as f32);

        Self {
            prev_state: INVALID_STATE,
            hall_idx_base: 0,
            hall_idx_max,
            angle_per_state,
            mechanical_angle: 0.0,
        }
    }

    /// Check if a hall state is valid
    ///
    /// # Arguments
    /// * `state` - Hall state (0-7)
    ///
    /// # Returns
    /// `true` if state is valid (1-6), `false` otherwise
    pub fn is_valid_state(state: u8) -> bool {
        (1..=6).contains(&state)
    }

    /// Process a new Hall state reading
    ///
    /// # Arguments
    /// * `raw_hall_state` - Raw Hall state (0-7)
    ///
    /// # Returns
    /// `Some(StateTransition)` if state is valid, `None` otherwise
    pub fn process_state(&mut self, raw_hall_state: u8) -> Option<StateTransition> {
        // Validate hall state
        if !Self::is_valid_state(raw_hall_state) {
            return None;
        }

        // Convert raw hall state to normalized index using lookup table
        let normalized_state = HALL_STATE_TABLE[raw_hall_state as usize];
        if normalized_state == INVALID_STATE {
            return None;
        }

        // Check if this is the first valid reading
        let is_first_reading = self.prev_state == INVALID_STATE;

        // Detect state change
        let state_changed = normalized_state != self.prev_state && !is_first_reading;

        if state_changed {
            // Handle hall index wrapping (foc-simple compatible)
            // State 0 after state 5 means we completed an electrical revolution
            if normalized_state == 0 && self.prev_state == 5 {
                self.hall_idx_base += 6;
                if self.hall_idx_base >= self.hall_idx_max {
                    self.hall_idx_base = 0; // Wrap around after full mechanical revolution
                }
            }
            // State 5 after state 0 means we're going backwards
            else if normalized_state == 5 && self.prev_state == 0 {
                if self.hall_idx_base < 6 {
                    self.hall_idx_base = self.hall_idx_max - 6;
                } else {
                    self.hall_idx_base -= 6;
                }
            }

            // Update previous state
            self.prev_state = normalized_state;
        } else if is_first_reading {
            // Initialize prev_state on first valid reading
            self.prev_state = normalized_state;
        }

        // Calculate mechanical angle from hall index
        let hall_state_idx = self.hall_idx_base + (normalized_state as u32);
        self.mechanical_angle = normalize_angle((hall_state_idx as f32) * self.angle_per_state);

        Some(StateTransition {
            state_changed,
            is_first_reading,
            normalized_state,
        })
    }

    /// Get the base mechanical angle for the current state (without interpolation)
    pub fn get_base_mechanical_angle(&self, normalized_state: u8) -> f32 {
        let hall_state_idx = self.hall_idx_base + (normalized_state as u32);
        (hall_state_idx as f32) * self.angle_per_state
    }

    /// Get current mechanical angle
    pub fn get_mechanical_angle(&self) -> f32 {
        self.mechanical_angle
    }

    /// Set mechanical angle (for interpolation updates)
    pub fn set_mechanical_angle(&mut self, angle: f32) {
        self.mechanical_angle = angle;
    }

    /// Get the angle per Hall state
    #[allow(dead_code)]
    pub fn get_angle_per_state(&self) -> f32 {
        self.angle_per_state
    }

    /// Reset the state machine
    pub fn reset(&mut self) {
        self.prev_state = INVALID_STATE;
        self.hall_idx_base = 0;
        self.mechanical_angle = 0.0;
    }

    /// Check if state machine has been initialized
    #[allow(dead_code)]
    pub fn is_initialized(&self) -> bool {
        self.prev_state != INVALID_STATE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_states() {
        assert!(!HallStateMachine::is_valid_state(0));
        assert!(HallStateMachine::is_valid_state(1));
        assert!(HallStateMachine::is_valid_state(6));
        assert!(!HallStateMachine::is_valid_state(7));
    }

    #[test]
    fn test_initial_state() {
        let sm = HallStateMachine::new(6);
        assert!(!sm.is_initialized());
    }

    #[test]
    fn test_first_reading() {
        let mut sm = HallStateMachine::new(6);
        let result = sm.process_state(1).unwrap();
        assert!(result.is_first_reading);
        assert!(!result.state_changed);
        assert!(sm.is_initialized());
    }

    #[test]
    fn test_state_change() {
        let mut sm = HallStateMachine::new(6);
        sm.process_state(1); // First reading
        let result = sm.process_state(3).unwrap(); // State 1 -> 3 (CW)
        assert!(!result.is_first_reading);
        assert!(result.state_changed);
    }

    #[test]
    fn test_invalid_state() {
        let mut sm = HallStateMachine::new(6);
        assert!(sm.process_state(0).is_none());
        assert!(sm.process_state(7).is_none());
    }
}
