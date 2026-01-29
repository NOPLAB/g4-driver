//! Step response scenario

use super::traits::{ScenarioMetrics, ScenarioResult, SpeedProfile};
use crate::motor_model::StateSnapshot;
use crate::simulation::{SimConfig, Simulation};
use crate::validation::PerformanceCriteria;

/// Step response scenario
///
/// Tests motor response to a step change in speed reference.
#[derive(Debug, Clone)]
pub struct StepResponse {
    /// Target speed after step [RPM]
    pub target_speed_rpm: f32,
    /// Load torque during test [N⋅m]
    pub load_torque: f32,
    /// Simulation duration [s]
    pub duration: f32,
    /// Performance criteria for pass/fail
    pub criteria: PerformanceCriteria,
}

impl StepResponse {
    /// Create new step response scenario
    pub fn new(target_speed_rpm: f32) -> Self {
        Self {
            target_speed_rpm,
            load_torque: 0.0,
            duration: 1.0,
            criteria: PerformanceCriteria::default(),
        }
    }

    /// Set load torque
    pub fn with_load(mut self, torque: f32) -> Self {
        self.load_torque = torque;
        self
    }

    /// Set simulation duration
    pub fn with_duration(mut self, duration: f32) -> Self {
        self.duration = duration;
        self
    }

    /// Set performance criteria
    pub fn with_criteria(mut self, criteria: PerformanceCriteria) -> Self {
        self.criteria = criteria;
        self
    }

    /// Run the scenario
    pub fn run(&self, sim: &mut Simulation) -> ScenarioResult {
        // Reset simulation
        sim.reset();

        // Configure
        sim.set_load_torque(self.load_torque);

        // Create speed profile
        let profile = SpeedProfile::Step {
            initial: 0.0,
            final_value: self.target_speed_rpm,
            step_time: 0.0, // Immediate step
        };

        // Apply initial target
        sim.set_target_speed_rpm(profile.at_time(0.0));

        // Run simulation and collect data
        let mut history: Vec<StateSnapshot> = Vec::new();
        let dt = sim.motor_params().v_dc; // Just need a dummy read
        let _ = dt; // Suppress warning

        let config = SimConfig {
            duration: self.duration,
            ..Default::default()
        };
        let total_steps = config.total_control_steps();

        let mut max_speed = 0.0f32;
        let mut rise_time = None;
        let mut settling_time = None;
        let target = self.target_speed_rpm;
        let settling_tolerance = 0.02 * target.abs().max(10.0); // 2% or 10 RPM minimum

        for i in 0..total_steps {
            let result = sim.step();

            // Track maximum speed
            max_speed = max_speed.max(result.speed_rpm);

            // Track rise time (time to reach 90% of target)
            if rise_time.is_none() && result.speed_rpm >= 0.9 * target {
                rise_time = Some(result.time * 1000.0); // Convert to ms
            }

            // Track settling time (last time outside tolerance)
            if (result.speed_rpm - target).abs() > settling_tolerance {
                settling_time = Some(result.time * 1000.0);
            }

            // Record history periodically
            if i % 10 == 0 {
                history.push(StateSnapshot::from_state(
                    sim.state(),
                    result.time,
                    result.torque,
                ));
            }
        }

        // Calculate metrics
        let overshoot_percent = if target > 0.0 {
            ((max_speed - target) / target * 100.0).max(0.0)
        } else {
            0.0
        };

        let final_speed = sim.state().speed_rpm();
        let steady_state_error = (final_speed - target).abs();

        let metrics = ScenarioMetrics {
            overshoot_percent: Some(overshoot_percent),
            rise_time_ms: rise_time,
            settling_time_ms: settling_time,
            steady_state_error_rpm: Some(steady_state_error),
            peak_current: Some(sim.state().current_magnitude()),
            max_torque: None,
            final_speed_rpm: Some(final_speed),
        };

        // Check pass/fail
        let mut passed = true;
        let mut failure_reason = None;

        if overshoot_percent > self.criteria.max_overshoot_percent {
            passed = false;
            failure_reason = Some(format!(
                "Overshoot {:.1}% exceeds limit {:.1}%",
                overshoot_percent, self.criteria.max_overshoot_percent
            ));
        } else if let Some(st) = settling_time {
            if st > self.criteria.settling_time_ms {
                passed = false;
                failure_reason = Some(format!(
                    "Settling time {:.1}ms exceeds limit {:.1}ms",
                    st, self.criteria.settling_time_ms
                ));
            }
        }

        if steady_state_error > self.criteria.steady_state_error_rpm {
            passed = false;
            failure_reason = Some(format!(
                "Steady-state error {:.1} RPM exceeds limit {:.1} RPM",
                steady_state_error, self.criteria.steady_state_error_rpm
            ));
        }

        ScenarioResult {
            name: format!("Step Response to {} RPM", self.target_speed_rpm),
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
    fn test_step_response_scenario() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig {
            duration: 0.5,
            ..Default::default()
        };
        let mut sim = Simulation::new(params, config);

        let scenario =
            StepResponse::new(500.0)
                .with_duration(0.5)
                .with_criteria(PerformanceCriteria {
                    max_overshoot_percent: 50.0,
                    settling_time_ms: 1000.0,
                    steady_state_error_rpm: 100.0,
                    ..Default::default()
                });

        let result = scenario.run(&mut sim);

        // Should have some metrics
        assert!(result.metrics.overshoot_percent.is_some());
        assert!(result.metrics.final_speed_rpm.is_some());
    }
}
