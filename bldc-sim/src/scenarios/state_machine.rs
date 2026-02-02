//! State machine scenarios
//!
//! Scenarios that test the full state machine behavior including
//! mode transitions (OpenLoop → FOC), stall recovery, etc.

use super::traits::{ScenarioMetrics, ScenarioResult};
use crate::motor_model::{MotorParams, StateSnapshot};
use crate::simulation::{SimConfig, StateMachineSimulation};
use bldc::traits::ControlMode;

/// Scenario result with state machine specific information
#[derive(Debug, Clone)]
pub struct StateMachineScenarioResult {
    /// Base scenario result
    pub base: ScenarioResult,
    /// Control mode history
    pub mode_history: Vec<ControlMode>,
    /// Number of stalls detected
    pub stall_count: u32,
    /// Time spent in each mode [s]
    pub mode_durations: ModeDurations,
}

/// Duration spent in each control mode
#[derive(Debug, Clone, Default)]
pub struct ModeDurations {
    /// Time in OpenLoop mode [s]
    pub openloop: f32,
    /// Time in FOC mode [s]
    pub foc: f32,
    /// Time in Calibration mode [s]
    pub calibration: f32,
}

/// Startup scenario with OpenLoop to FOC transition
///
/// Tests the full startup sequence including:
/// 1. OpenLoop phase for initial acceleration
/// 2. Transition to FOC when conditions are met
/// 3. FOC speed control to target
#[derive(Debug, Clone)]
pub struct StartupWithTransitionScenario {
    /// Target speed [RPM]
    pub target_speed_rpm: f32,
    /// Load torque during startup [N⋅m]
    pub load_torque: f32,
    /// Maximum time to transition to FOC [ms]
    pub max_transition_time_ms: f32,
    /// Maximum time to reach target speed [ms]
    pub max_target_time_ms: f32,
    /// Speed tolerance for reaching target [RPM]
    pub speed_tolerance_rpm: f32,
    /// Duration to simulate [s]
    pub duration: f32,
}

impl StartupWithTransitionScenario {
    /// Create a new startup with transition scenario
    pub fn new(target_speed_rpm: f32) -> Self {
        Self {
            target_speed_rpm,
            load_torque: 0.0,
            max_transition_time_ms: 1000.0,
            max_target_time_ms: 2000.0,
            speed_tolerance_rpm: 50.0,
            duration: 3.0,
        }
    }

    /// Set load torque
    pub fn with_load(mut self, torque: f32) -> Self {
        self.load_torque = torque;
        self
    }

    /// Set maximum transition time
    pub fn with_max_transition_time(mut self, time_ms: f32) -> Self {
        self.max_transition_time_ms = time_ms;
        self
    }

    /// Set maximum target time
    pub fn with_max_target_time(mut self, time_ms: f32) -> Self {
        self.max_target_time_ms = time_ms;
        self
    }

    /// Set duration
    pub fn with_duration(mut self, duration: f32) -> Self {
        self.duration = duration;
        self
    }

    /// Run the scenario
    pub fn run(&self, sim: &mut StateMachineSimulation) -> StateMachineScenarioResult {
        sim.reset();
        sim.set_load_torque(self.load_torque);
        sim.set_target_speed_rpm(self.target_speed_rpm);
        sim.set_motor_enabled(true);

        let dt = 0.0004; // Control period
        let total_steps = (self.duration / dt) as u32;

        let mut history: Vec<StateSnapshot> = Vec::new();
        let mut transition_time_ms: Option<f32> = None;
        let mut target_reached_time_ms: Option<f32> = None;
        let mut peak_current = 0.0f32;
        let mut mode_durations = ModeDurations::default();
        let mut last_mode = ControlMode::OpenLoop;

        for i in 0..total_steps {
            let result = sim.step();

            // Track peak current
            let current = sim.motor_state().current_magnitude();
            peak_current = peak_current.max(current);

            // Track mode durations
            match result.control_mode {
                ControlMode::OpenLoop => mode_durations.openloop += dt,
                ControlMode::Foc => mode_durations.foc += dt,
                ControlMode::Calibration => mode_durations.calibration += dt,
            }

            // Detect transition to FOC
            if transition_time_ms.is_none()
                && result.control_mode == ControlMode::Foc
                && last_mode == ControlMode::OpenLoop
            {
                transition_time_ms = Some(result.time * 1000.0);
            }

            // Detect target speed reached
            if target_reached_time_ms.is_none()
                && result.control_mode == ControlMode::Foc
                && (result.speed_rpm.abs() - self.target_speed_rpm).abs() < self.speed_tolerance_rpm
            {
                target_reached_time_ms = Some(result.time * 1000.0);
            }

            last_mode = result.control_mode;

            if i % 10 == 0 {
                history.push(StateSnapshot::from_state(
                    sim.motor_state(),
                    result.time,
                    result.torque,
                ));
            }
        }

        let final_speed = sim.motor_state().speed_rpm().abs();
        let mode_history = sim.mode_history().to_vec();
        let stall_count = sim.stall_count();

        let metrics = ScenarioMetrics {
            overshoot_percent: None,
            rise_time_ms: transition_time_ms,
            settling_time_ms: target_reached_time_ms,
            steady_state_error_rpm: Some((final_speed - self.target_speed_rpm).abs()),
            peak_current: Some(peak_current),
            max_torque: None,
            final_speed_rpm: Some(final_speed),
        };

        let mut passed = true;
        let mut failure_reason = None;

        // Check if transitioned to FOC
        if transition_time_ms.is_none() {
            passed = false;
            failure_reason = Some("Failed to transition to FOC mode".to_string());
        } else if let Some(tt) = transition_time_ms {
            if tt > self.max_transition_time_ms {
                passed = false;
                failure_reason = Some(format!(
                    "Transition time {:.1}ms exceeds limit {:.1}ms",
                    tt, self.max_transition_time_ms
                ));
            }
        }

        // Check if target reached
        if passed && target_reached_time_ms.is_none() {
            passed = false;
            failure_reason = Some(format!(
                "Failed to reach target speed {} RPM (final: {:.1} RPM)",
                self.target_speed_rpm, final_speed
            ));
        }

        StateMachineScenarioResult {
            base: ScenarioResult {
                name: format!("Startup with transition to {} RPM", self.target_speed_rpm),
                passed,
                metrics,
                history,
                failure_reason,
            },
            mode_history,
            stall_count,
            mode_durations,
        }
    }
}

/// Step response scenario with state machine
///
/// Tests speed step response in FOC mode
#[derive(Debug, Clone)]
pub struct StateMachineStepResponse {
    /// Initial speed [RPM]
    pub initial_speed_rpm: f32,
    /// Target speed after step [RPM]
    pub target_speed_rpm: f32,
    /// Time to apply step [s]
    pub step_time: f32,
    /// Maximum overshoot [%]
    pub max_overshoot_percent: f32,
    /// Maximum settling time [ms]
    pub max_settling_time_ms: f32,
    /// Duration to simulate [s]
    pub duration: f32,
}

impl StateMachineStepResponse {
    /// Create a new step response scenario
    pub fn new(initial_speed_rpm: f32, target_speed_rpm: f32) -> Self {
        Self {
            initial_speed_rpm,
            target_speed_rpm,
            step_time: 0.5,
            max_overshoot_percent: 20.0,
            max_settling_time_ms: 500.0,
            duration: 2.0,
        }
    }

    /// Set step time
    pub fn with_step_time(mut self, time: f32) -> Self {
        self.step_time = time;
        self
    }

    /// Set duration
    pub fn with_duration(mut self, duration: f32) -> Self {
        self.duration = duration;
        self
    }

    /// Run the scenario
    pub fn run(&self, sim: &mut StateMachineSimulation) -> StateMachineScenarioResult {
        sim.reset();
        sim.set_target_speed_rpm(self.initial_speed_rpm);
        sim.set_motor_enabled(true);

        let dt = 0.0004; // Control period
        let total_steps = (self.duration / dt) as u32;
        let step_at = (self.step_time / dt) as u32;

        let mut history: Vec<StateSnapshot> = Vec::new();
        let mut mode_durations = ModeDurations::default();
        let mut peak_speed = 0.0f32;
        let mut settling_time_ms: Option<f32> = None;
        let speed_tolerance = (self.target_speed_rpm - self.initial_speed_rpm).abs() * 0.05; // 5% tolerance

        for i in 0..total_steps {
            // Apply step at specified time
            if i == step_at {
                sim.set_target_speed_rpm(self.target_speed_rpm);
            }

            let result = sim.step();

            // Track mode durations
            match result.control_mode {
                ControlMode::OpenLoop => mode_durations.openloop += dt,
                ControlMode::Foc => mode_durations.foc += dt,
                ControlMode::Calibration => mode_durations.calibration += dt,
            }

            // Track peak speed after step
            if i > step_at {
                peak_speed = peak_speed.max(result.speed_rpm.abs());

                // Detect settling
                if settling_time_ms.is_none()
                    && (result.speed_rpm.abs() - self.target_speed_rpm).abs() < speed_tolerance
                {
                    settling_time_ms = Some((result.time - self.step_time) * 1000.0);
                }
            }

            if i % 10 == 0 {
                history.push(StateSnapshot::from_state(
                    sim.motor_state(),
                    result.time,
                    result.torque,
                ));
            }
        }

        let final_speed = sim.motor_state().speed_rpm().abs();
        let mode_history = sim.mode_history().to_vec();
        let stall_count = sim.stall_count();

        // Calculate overshoot
        let speed_change = (self.target_speed_rpm - self.initial_speed_rpm).abs();
        let overshoot = if speed_change > 0.0 {
            ((peak_speed - self.target_speed_rpm) / speed_change * 100.0).max(0.0)
        } else {
            0.0
        };

        let metrics = ScenarioMetrics {
            overshoot_percent: Some(overshoot),
            rise_time_ms: None,
            settling_time_ms,
            steady_state_error_rpm: Some((final_speed - self.target_speed_rpm).abs()),
            peak_current: None,
            max_torque: None,
            final_speed_rpm: Some(final_speed),
        };

        let mut passed = true;
        let mut failure_reason = None;

        if overshoot > self.max_overshoot_percent {
            passed = false;
            failure_reason = Some(format!(
                "Overshoot {:.1}% exceeds limit {:.1}%",
                overshoot, self.max_overshoot_percent
            ));
        }

        if let Some(st) = settling_time_ms {
            if st > self.max_settling_time_ms {
                passed = false;
                failure_reason = Some(format!(
                    "Settling time {:.1}ms exceeds limit {:.1}ms",
                    st, self.max_settling_time_ms
                ));
            }
        }

        StateMachineScenarioResult {
            base: ScenarioResult {
                name: format!(
                    "Step response {} → {} RPM",
                    self.initial_speed_rpm, self.target_speed_rpm
                ),
                passed,
                metrics,
                history,
                failure_reason,
            },
            mode_history,
            stall_count,
            mode_durations,
        }
    }
}

/// Load disturbance scenario with state machine
///
/// Tests disturbance rejection in FOC mode
#[derive(Debug, Clone)]
pub struct StateMachineLoadDisturbance {
    /// Target speed [RPM]
    pub target_speed_rpm: f32,
    /// Load torque [N⋅m]
    pub load_torque: f32,
    /// Time to apply load [s]
    pub load_time: f32,
    /// Maximum speed drop [RPM]
    pub max_speed_drop_rpm: f32,
    /// Maximum recovery time [ms]
    pub max_recovery_time_ms: f32,
    /// Duration to simulate [s]
    pub duration: f32,
}

impl StateMachineLoadDisturbance {
    /// Create a new load disturbance scenario
    pub fn new(target_speed_rpm: f32, load_torque: f32) -> Self {
        Self {
            target_speed_rpm,
            load_torque,
            load_time: 1.0,
            max_speed_drop_rpm: 100.0,
            max_recovery_time_ms: 500.0,
            duration: 3.0,
        }
    }

    /// Set load time
    pub fn with_load_time(mut self, time: f32) -> Self {
        self.load_time = time;
        self
    }

    /// Set duration
    pub fn with_duration(mut self, duration: f32) -> Self {
        self.duration = duration;
        self
    }

    /// Run the scenario
    pub fn run(&self, sim: &mut StateMachineSimulation) -> StateMachineScenarioResult {
        sim.reset();
        sim.set_target_speed_rpm(self.target_speed_rpm);
        sim.set_motor_enabled(true);

        let dt = 0.0004; // Control period
        let total_steps = (self.duration / dt) as u32;
        let load_at = (self.load_time / dt) as u32;

        let mut history: Vec<StateSnapshot> = Vec::new();
        let mut mode_durations = ModeDurations::default();
        let mut speed_before_load = 0.0f32;
        let mut min_speed_after_load = f32::MAX;
        let mut recovery_time_ms: Option<f32> = None;
        let speed_tolerance = self.target_speed_rpm * 0.05; // 5% tolerance

        for i in 0..total_steps {
            // Apply load at specified time
            if i == load_at {
                speed_before_load = sim.motor_state().speed_rpm().abs();
                sim.set_load_torque(self.load_torque);
            }

            let result = sim.step();

            // Track mode durations
            match result.control_mode {
                ControlMode::OpenLoop => mode_durations.openloop += dt,
                ControlMode::Foc => mode_durations.foc += dt,
                ControlMode::Calibration => mode_durations.calibration += dt,
            }

            // Track minimum speed after load
            if i > load_at {
                min_speed_after_load = min_speed_after_load.min(result.speed_rpm.abs());

                // Detect recovery
                if recovery_time_ms.is_none()
                    && (result.speed_rpm.abs() - self.target_speed_rpm).abs() < speed_tolerance
                {
                    recovery_time_ms = Some((result.time - self.load_time) * 1000.0);
                }
            }

            if i % 10 == 0 {
                history.push(StateSnapshot::from_state(
                    sim.motor_state(),
                    result.time,
                    result.torque,
                ));
            }
        }

        let final_speed = sim.motor_state().speed_rpm().abs();
        let mode_history = sim.mode_history().to_vec();
        let stall_count = sim.stall_count();

        let speed_drop = speed_before_load - min_speed_after_load;

        let metrics = ScenarioMetrics {
            overshoot_percent: None,
            rise_time_ms: None,
            settling_time_ms: recovery_time_ms,
            steady_state_error_rpm: Some((final_speed - self.target_speed_rpm).abs()),
            peak_current: None,
            max_torque: Some(self.load_torque),
            final_speed_rpm: Some(final_speed),
        };

        let mut passed = true;
        let mut failure_reason = None;

        if speed_drop > self.max_speed_drop_rpm {
            passed = false;
            failure_reason = Some(format!(
                "Speed drop {:.1} RPM exceeds limit {:.1} RPM",
                speed_drop, self.max_speed_drop_rpm
            ));
        }

        if stall_count > 0 {
            passed = false;
            failure_reason = Some(format!("Motor stalled {} times", stall_count));
        }

        StateMachineScenarioResult {
            base: ScenarioResult {
                name: format!(
                    "Load disturbance at {} RPM (load={} N⋅m)",
                    self.target_speed_rpm, self.load_torque
                ),
                passed,
                metrics,
                history,
                failure_reason,
            },
            mode_history,
            stall_count,
            mode_durations,
        }
    }
}

/// Helper to create a StateMachineSimulation with default config
pub fn create_simulation(duration: f32) -> StateMachineSimulation {
    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration,
        ..Default::default()
    };
    StateMachineSimulation::new(params, config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_startup_with_transition_scenario() {
        let mut sim = create_simulation(3.0);

        let scenario = StartupWithTransitionScenario::new(500.0)
            .with_max_transition_time(2000.0)
            .with_max_target_time(3000.0)
            .with_duration(3.0);

        let result = scenario.run(&mut sim);

        // Should have at least some history
        assert!(!result.base.history.is_empty());
        assert!(result.base.metrics.final_speed_rpm.is_some());

        // Should have spent time in OpenLoop at minimum
        assert!(result.mode_durations.openloop > 0.0);
    }

    #[test]
    fn test_step_response_scenario() {
        let mut sim = create_simulation(3.0);

        // First get to steady state
        sim.set_target_speed_rpm(300.0);
        sim.set_motor_enabled(true);
        for _ in 0..5000 {
            sim.step();
        }

        sim.reset();

        let scenario = StateMachineStepResponse::new(300.0, 600.0)
            .with_step_time(0.5)
            .with_duration(2.0);

        let result = scenario.run(&mut sim);

        assert!(!result.base.history.is_empty());
        assert!(result.base.metrics.overshoot_percent.is_some());
    }

    #[test]
    fn test_load_disturbance_scenario() {
        let mut sim = create_simulation(4.0);

        let scenario = StateMachineLoadDisturbance::new(500.0, 0.01)
            .with_load_time(1.5)
            .with_duration(4.0);

        let result = scenario.run(&mut sim);

        assert!(!result.base.history.is_empty());
        assert!(result.base.metrics.max_torque.is_some());
    }
}
