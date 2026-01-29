//! Scenario trait definition

use crate::motor_model::StateSnapshot;
use crate::simulation::{SimConfig, Simulation};

/// Result of running a scenario
#[derive(Debug, Clone)]
pub struct ScenarioResult {
    /// Scenario name
    pub name: String,
    /// Whether the scenario passed
    pub passed: bool,
    /// Performance metrics calculated
    pub metrics: ScenarioMetrics,
    /// State history from the simulation
    pub history: Vec<StateSnapshot>,
    /// Description of failure (if any)
    pub failure_reason: Option<String>,
}

/// Performance metrics from scenario execution
#[derive(Debug, Clone, Default)]
pub struct ScenarioMetrics {
    /// Maximum speed overshoot [%]
    pub overshoot_percent: Option<f32>,
    /// Time to reach 90% of target [ms]
    pub rise_time_ms: Option<f32>,
    /// Time to settle within tolerance [ms]
    pub settling_time_ms: Option<f32>,
    /// Steady-state error [RPM]
    pub steady_state_error_rpm: Option<f32>,
    /// Peak current during transient [A]
    pub peak_current: Option<f32>,
    /// Maximum torque [N⋅m]
    pub max_torque: Option<f32>,
    /// Final speed [RPM]
    pub final_speed_rpm: Option<f32>,
}

/// Trait for test scenarios
pub trait Scenario {
    /// Get scenario name
    fn name(&self) -> &str;

    /// Configure the simulation for this scenario
    fn configure(&self, sim: &mut Simulation);

    /// Run the scenario and return results
    fn run(&self, sim: &mut Simulation) -> ScenarioResult;

    /// Get recommended simulation configuration
    fn recommended_config(&self) -> SimConfig {
        SimConfig::default()
    }
}

/// Target profile for speed reference
#[derive(Debug, Clone)]
pub enum SpeedProfile {
    /// Constant target speed
    Constant(f32),
    /// Step change at specified time
    Step {
        initial: f32,
        final_value: f32,
        step_time: f32,
    },
    /// Linear ramp
    Ramp {
        initial: f32,
        final_value: f32,
        ramp_time: f32,
    },
    /// Custom profile function
    Custom(Vec<(f32, f32)>), // (time, speed) points
}

impl SpeedProfile {
    /// Get target speed at given time
    pub fn at_time(&self, time: f32) -> f32 {
        match self {
            SpeedProfile::Constant(speed) => *speed,
            SpeedProfile::Step {
                initial,
                final_value,
                step_time,
            } => {
                if time < *step_time {
                    *initial
                } else {
                    *final_value
                }
            }
            SpeedProfile::Ramp {
                initial,
                final_value,
                ramp_time,
            } => {
                if time <= 0.0 {
                    *initial
                } else if time >= *ramp_time {
                    *final_value
                } else {
                    initial + (final_value - initial) * time / ramp_time
                }
            }
            SpeedProfile::Custom(points) => {
                if points.is_empty() {
                    return 0.0;
                }
                // Find surrounding points and interpolate
                for i in 0..points.len() - 1 {
                    if time >= points[i].0 && time < points[i + 1].0 {
                        let t0 = points[i].0;
                        let t1 = points[i + 1].0;
                        let v0 = points[i].1;
                        let v1 = points[i + 1].1;
                        return v0 + (v1 - v0) * (time - t0) / (t1 - t0);
                    }
                }
                // Return last value if past all points
                points.last().map(|p| p.1).unwrap_or(0.0)
            }
        }
    }
}

/// Load profile for external torque
#[derive(Debug, Clone)]
pub enum LoadProfile {
    /// No load
    None,
    /// Constant load
    Constant(f32),
    /// Step change at specified time
    Step {
        initial: f32,
        final_value: f32,
        step_time: f32,
    },
    /// Custom profile
    Custom(Vec<(f32, f32)>),
}

impl LoadProfile {
    /// Get load torque at given time
    pub fn at_time(&self, time: f32) -> f32 {
        match self {
            LoadProfile::None => 0.0,
            LoadProfile::Constant(torque) => *torque,
            LoadProfile::Step {
                initial,
                final_value,
                step_time,
            } => {
                if time < *step_time {
                    *initial
                } else {
                    *final_value
                }
            }
            LoadProfile::Custom(points) => {
                if points.is_empty() {
                    return 0.0;
                }
                for i in 0..points.len() - 1 {
                    if time >= points[i].0 && time < points[i + 1].0 {
                        let t0 = points[i].0;
                        let t1 = points[i + 1].0;
                        let v0 = points[i].1;
                        let v1 = points[i + 1].1;
                        return v0 + (v1 - v0) * (time - t0) / (t1 - t0);
                    }
                }
                points.last().map(|p| p.1).unwrap_or(0.0)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constant_profile() {
        let profile = SpeedProfile::Constant(1000.0);
        assert!((profile.at_time(0.0) - 1000.0).abs() < 0.1);
        assert!((profile.at_time(5.0) - 1000.0).abs() < 0.1);
    }

    #[test]
    fn test_step_profile() {
        let profile = SpeedProfile::Step {
            initial: 0.0,
            final_value: 1000.0,
            step_time: 0.5,
        };
        assert!((profile.at_time(0.0) - 0.0).abs() < 0.1);
        assert!((profile.at_time(0.4) - 0.0).abs() < 0.1);
        assert!((profile.at_time(0.6) - 1000.0).abs() < 0.1);
    }

    #[test]
    fn test_ramp_profile() {
        let profile = SpeedProfile::Ramp {
            initial: 0.0,
            final_value: 1000.0,
            ramp_time: 1.0,
        };
        assert!((profile.at_time(0.0) - 0.0).abs() < 0.1);
        assert!((profile.at_time(0.5) - 500.0).abs() < 0.1);
        assert!((profile.at_time(1.0) - 1000.0).abs() < 0.1);
        assert!((profile.at_time(2.0) - 1000.0).abs() < 0.1);
    }
}
