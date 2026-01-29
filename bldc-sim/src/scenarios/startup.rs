//! Startup scenario

use super::traits::{ScenarioMetrics, ScenarioResult};
use crate::motor_model::StateSnapshot;
use crate::simulation::{SimConfig, Simulation};

/// Startup scenario
///
/// Tests motor startup from standstill.
#[derive(Debug, Clone)]
pub struct StartupScenario {
    /// Target speed [RPM]
    pub target_speed_rpm: f32,
    /// Load torque during startup [N⋅m]
    pub load_torque: f32,
    /// Maximum startup time [ms]
    pub max_startup_time_ms: f32,
    /// Minimum speed to consider "started" [RPM]
    pub min_running_speed_rpm: f32,
    /// Duration to simulate [s]
    pub duration: f32,
}

impl StartupScenario {
    /// Create new startup scenario
    pub fn new(target_speed_rpm: f32) -> Self {
        Self {
            target_speed_rpm,
            load_torque: 0.0,
            max_startup_time_ms: 500.0,
            min_running_speed_rpm: 100.0,
            duration: 1.0,
        }
    }

    /// Set load torque
    pub fn with_load(mut self, torque: f32) -> Self {
        self.load_torque = torque;
        self
    }

    /// Set maximum startup time
    pub fn with_max_time(mut self, time_ms: f32) -> Self {
        self.max_startup_time_ms = time_ms;
        self
    }

    /// Set minimum running speed threshold
    pub fn with_min_speed(mut self, speed_rpm: f32) -> Self {
        self.min_running_speed_rpm = speed_rpm;
        self
    }

    /// Set duration
    pub fn with_duration(mut self, duration: f32) -> Self {
        self.duration = duration;
        self
    }

    /// Run the scenario
    pub fn run(&self, sim: &mut Simulation) -> ScenarioResult {
        sim.reset();
        sim.set_load_torque(self.load_torque);
        sim.set_target_speed_rpm(self.target_speed_rpm);

        let config = SimConfig {
            duration: self.duration,
            ..Default::default()
        };
        let total_steps = config.total_control_steps();

        let mut history: Vec<StateSnapshot> = Vec::new();
        let mut startup_time = None;
        let mut peak_current = 0.0f32;
        let mut motor_started = false;

        for i in 0..total_steps {
            let result = sim.step();

            // Track peak current
            let current = sim.state().current_magnitude();
            peak_current = peak_current.max(current);

            // Detect startup (first time speed exceeds threshold)
            if !motor_started && result.speed_rpm >= self.min_running_speed_rpm {
                motor_started = true;
                startup_time = Some(result.time * 1000.0);
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

        let metrics = ScenarioMetrics {
            overshoot_percent: None,
            rise_time_ms: startup_time,
            settling_time_ms: None,
            steady_state_error_rpm: Some((final_speed - self.target_speed_rpm).abs()),
            peak_current: Some(peak_current),
            max_torque: None,
            final_speed_rpm: Some(final_speed),
        };

        let mut passed = true;
        let mut failure_reason = None;

        if !motor_started {
            passed = false;
            failure_reason = Some(format!(
                "Motor failed to start (speed never reached {} RPM)",
                self.min_running_speed_rpm
            ));
        } else if let Some(st) = startup_time {
            if st > self.max_startup_time_ms {
                passed = false;
                failure_reason = Some(format!(
                    "Startup time {:.1}ms exceeds limit {:.1}ms",
                    st, self.max_startup_time_ms
                ));
            }
        }

        ScenarioResult {
            name: format!("Startup to {} RPM", self.target_speed_rpm),
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
    fn test_startup_scenario() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig::default();
        let mut sim = Simulation::new(params, config);

        let scenario = StartupScenario::new(500.0)
            .with_max_time(800.0)
            .with_min_speed(50.0)
            .with_duration(0.5);

        let result = scenario.run(&mut sim);

        assert!(result.metrics.final_speed_rpm.is_some());
        assert!(result.metrics.peak_current.is_some());
    }
}
