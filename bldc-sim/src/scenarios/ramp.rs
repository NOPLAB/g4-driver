//! Ramp response scenario

use super::traits::{ScenarioMetrics, ScenarioResult, SpeedProfile};
use crate::motor_model::StateSnapshot;
use crate::simulation::{SimConfig, Simulation};

/// Ramp response scenario
///
/// Tests motor response to a linear ramp in speed reference.
#[derive(Debug, Clone)]
pub struct RampResponse {
    /// Initial speed [RPM]
    pub initial_speed_rpm: f32,
    /// Final speed [RPM]
    pub final_speed_rpm: f32,
    /// Ramp duration [s]
    pub ramp_time: f32,
    /// Hold time after ramp [s]
    pub hold_time: f32,
    /// Load torque [N⋅m]
    pub load_torque: f32,
    /// Maximum allowed tracking error [RPM]
    pub max_tracking_error_rpm: f32,
}

impl RampResponse {
    /// Create new ramp response scenario
    pub fn new(initial: f32, final_value: f32, ramp_time: f32) -> Self {
        Self {
            initial_speed_rpm: initial,
            final_speed_rpm: final_value,
            ramp_time,
            hold_time: 0.2,
            load_torque: 0.0,
            max_tracking_error_rpm: 50.0,
        }
    }

    /// Set hold time after ramp
    pub fn with_hold_time(mut self, hold_time: f32) -> Self {
        self.hold_time = hold_time;
        self
    }

    /// Set load torque
    pub fn with_load(mut self, torque: f32) -> Self {
        self.load_torque = torque;
        self
    }

    /// Set maximum tracking error
    pub fn with_max_error(mut self, error_rpm: f32) -> Self {
        self.max_tracking_error_rpm = error_rpm;
        self
    }

    /// Run the scenario
    pub fn run(&self, sim: &mut Simulation) -> ScenarioResult {
        sim.reset();
        sim.set_load_torque(self.load_torque);

        let profile = SpeedProfile::Ramp {
            initial: self.initial_speed_rpm,
            final_value: self.final_speed_rpm,
            ramp_time: self.ramp_time,
        };

        let duration = self.ramp_time + self.hold_time;
        let config = SimConfig {
            duration,
            ..Default::default()
        };
        let total_steps = config.total_control_steps();

        let mut history: Vec<StateSnapshot> = Vec::new();
        let mut max_tracking_error = 0.0f32;
        let mut max_error_time = 0.0f32;

        for i in 0..total_steps {
            let time = i as f32 * config.control_period;

            // Update target according to profile
            let target = profile.at_time(time);
            sim.set_target_speed_rpm(target);

            let result = sim.step();

            // Track error
            let error = (result.speed_rpm - target).abs();
            if error > max_tracking_error {
                max_tracking_error = error;
                max_error_time = result.time;
            }

            if i % 10 == 0 {
                history.push(StateSnapshot::from_state(
                    sim.state(),
                    result.time,
                    result.torque,
                ));
            }
        }

        let final_speed = sim.state().speed_rpm();
        let final_error = (final_speed - self.final_speed_rpm).abs();

        let metrics = ScenarioMetrics {
            overshoot_percent: None, // Not applicable for ramp
            rise_time_ms: None,
            settling_time_ms: None,
            steady_state_error_rpm: Some(final_error),
            peak_current: Some(sim.state().current_magnitude()),
            max_torque: None,
            final_speed_rpm: Some(final_speed),
        };

        let passed = max_tracking_error <= self.max_tracking_error_rpm;
        let failure_reason = if !passed {
            Some(format!(
                "Max tracking error {:.1} RPM at t={:.3}s exceeds limit {:.1} RPM",
                max_tracking_error, max_error_time, self.max_tracking_error_rpm
            ))
        } else {
            None
        };

        ScenarioResult {
            name: format!(
                "Ramp Response {} -> {} RPM in {}s",
                self.initial_speed_rpm, self.final_speed_rpm, self.ramp_time
            ),
            passed,
            metrics,
            history,
            failure_reason,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor_model::MotorParams;

    #[test]
    fn test_ramp_response_scenario() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig::default();
        let mut sim = Simulation::new(params, config);

        let scenario = RampResponse::new(0.0, 500.0, 0.5)
            .with_hold_time(0.1)
            .with_max_error(200.0);

        let result = scenario.run(&mut sim);

        assert!(result.metrics.final_speed_rpm.is_some());
    }
}
