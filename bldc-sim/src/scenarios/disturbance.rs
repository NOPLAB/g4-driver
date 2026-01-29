//! Load disturbance scenario

use super::traits::{LoadProfile, ScenarioMetrics, ScenarioResult};
use crate::motor_model::StateSnapshot;
use crate::simulation::{SimConfig, Simulation};

/// Load disturbance scenario
///
/// Tests motor response to a sudden change in load torque.
#[derive(Debug, Clone)]
pub struct LoadDisturbance {
    /// Target speed [RPM]
    pub target_speed_rpm: f32,
    /// Initial load [N⋅m]
    pub initial_load: f32,
    /// Disturbance load [N⋅m]
    pub disturbance_load: f32,
    /// Time of load application [s]
    pub load_time: f32,
    /// Total duration [s]
    pub duration: f32,
    /// Maximum allowed speed dip [%]
    pub max_speed_dip_percent: f32,
    /// Recovery time limit [ms]
    pub max_recovery_time_ms: f32,
}

impl LoadDisturbance {
    /// Create new load disturbance scenario
    pub fn new(target_rpm: f32, disturbance_load: f32) -> Self {
        Self {
            target_speed_rpm: target_rpm,
            initial_load: 0.0,
            disturbance_load,
            load_time: 0.3,
            duration: 1.0,
            max_speed_dip_percent: 30.0,
            max_recovery_time_ms: 300.0,
        }
    }

    /// Set initial load
    pub fn with_initial_load(mut self, load: f32) -> Self {
        self.initial_load = load;
        self
    }

    /// Set time when disturbance is applied
    pub fn with_load_time(mut self, time: f32) -> Self {
        self.load_time = time;
        self
    }

    /// Set duration
    pub fn with_duration(mut self, duration: f32) -> Self {
        self.duration = duration;
        self
    }

    /// Set maximum allowed speed dip
    pub fn with_max_dip(mut self, percent: f32) -> Self {
        self.max_speed_dip_percent = percent;
        self
    }

    /// Set recovery time limit
    pub fn with_recovery_time(mut self, time_ms: f32) -> Self {
        self.max_recovery_time_ms = time_ms;
        self
    }

    /// Run the scenario
    pub fn run(&self, sim: &mut Simulation) -> ScenarioResult {
        sim.reset();

        let load_profile = LoadProfile::Step {
            initial: self.initial_load,
            final_value: self.initial_load + self.disturbance_load,
            step_time: self.load_time,
        };

        // First, run to steady state at target speed
        sim.set_target_speed_rpm(self.target_speed_rpm);
        sim.set_load_torque(self.initial_load);

        let config = SimConfig {
            duration: self.duration,
            ..Default::default()
        };
        let total_steps = config.total_control_steps();

        let mut history: Vec<StateSnapshot> = Vec::new();
        let mut speed_before_disturbance = 0.0f32;
        let mut min_speed_after_disturbance = f32::MAX;
        let mut disturbance_applied = false;
        let mut recovery_time = None;
        let tolerance = 0.05 * self.target_speed_rpm.abs().max(10.0);

        for i in 0..total_steps {
            let time = i as f32 * config.control_period;

            // Apply load according to profile
            let load = load_profile.at_time(time);
            sim.set_load_torque(load);

            let result = sim.step();

            // Track speed before disturbance
            if time < self.load_time {
                speed_before_disturbance = result.speed_rpm;
            }

            // Track min speed and recovery after disturbance
            if time >= self.load_time {
                if !disturbance_applied {
                    disturbance_applied = true;
                }
                min_speed_after_disturbance = min_speed_after_disturbance.min(result.speed_rpm);

                // Check if recovered
                if recovery_time.is_none()
                    && (result.speed_rpm - self.target_speed_rpm).abs() <= tolerance
                {
                    recovery_time = Some((result.time - self.load_time) * 1000.0);
                }
            }

            if i % 10 == 0 {
                history.push(StateSnapshot::from_state(
                    sim.state(),
                    result.time,
                    result.torque,
                ));
            }
        }

        // Calculate speed dip
        let speed_dip = speed_before_disturbance - min_speed_after_disturbance;
        let speed_dip_percent = if speed_before_disturbance > 0.0 {
            (speed_dip / speed_before_disturbance * 100.0).max(0.0)
        } else {
            0.0
        };

        let final_speed = sim.state().speed_rpm();

        let metrics = ScenarioMetrics {
            overshoot_percent: Some(speed_dip_percent), // Using for dip
            rise_time_ms: recovery_time,                // Using for recovery
            settling_time_ms: recovery_time,
            steady_state_error_rpm: Some((final_speed - self.target_speed_rpm).abs()),
            peak_current: None,
            max_torque: None,
            final_speed_rpm: Some(final_speed),
        };

        let mut passed = true;
        let mut failure_reason = None;

        if speed_dip_percent > self.max_speed_dip_percent {
            passed = false;
            failure_reason = Some(format!(
                "Speed dip {:.1}% exceeds limit {:.1}%",
                speed_dip_percent, self.max_speed_dip_percent
            ));
        } else if let Some(rt) = recovery_time {
            if rt > self.max_recovery_time_ms {
                passed = false;
                failure_reason = Some(format!(
                    "Recovery time {:.1}ms exceeds limit {:.1}ms",
                    rt, self.max_recovery_time_ms
                ));
            }
        } else {
            // No recovery detected
            passed = false;
            failure_reason = Some("Motor did not recover to target speed".to_string());
        }

        ScenarioResult {
            name: format!(
                "Load Disturbance at {} RPM with {} N⋅m",
                self.target_speed_rpm, self.disturbance_load
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
    fn test_load_disturbance_scenario() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig::default();
        let mut sim = Simulation::new(params, config);

        let scenario = LoadDisturbance::new(500.0, 0.005)
            .with_load_time(0.2)
            .with_duration(0.6)
            .with_max_dip(80.0);

        let result = scenario.run(&mut sim);

        assert!(result.metrics.final_speed_rpm.is_some());
    }
}
