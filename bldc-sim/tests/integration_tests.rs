//! Integration tests for bldc-sim

use bldc::traits::ControlMode;
use bldc_sim::motor_model::MotorParams;
use bldc_sim::scenarios::{
    LoadDisturbance, RampResponse, StartupScenario, StartupWithTransitionScenario,
    StateMachineLoadDisturbance, StateMachineStepResponse, StepResponse,
};
use bldc_sim::simulation::{SimConfig, Simulation, StateMachineSimulation};
use bldc_sim::validation::PerformanceCriteria;

#[cfg(feature = "csv-output")]
use bldc_sim::output::write_to_csv;

#[cfg(feature = "visualization")]
use bldc_sim::output::{plot_speed, PlotConfig};

/// Output directory for test results
const OUTPUT_DIR: &str = "output";

/// Helper to ensure output directory exists
fn ensure_output_dir() {
    std::fs::create_dir_all(OUTPUT_DIR).ok();
}

// ============================================================================
// Step Response Tests
// ============================================================================

#[test]
fn test_step_response_500rpm() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 0.8,
        record_interval: 0.001,
        ..Default::default()
    };
    let mut sim = Simulation::new(params, config);

    let scenario = StepResponse::new(500.0)
        .with_duration(0.8)
        .with_criteria(PerformanceCriteria {
            max_overshoot_percent: 50.0,
            settling_time_ms: 1500.0,
            steady_state_error_rpm: 100.0,
            rise_time_ms: 800.0,
        });

    let result = scenario.run(&mut sim);

    // Output files when features are enabled
    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/step_response_500rpm.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/step_response_500rpm.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Step Response to 500 RPM");
        plot_speed(&plot_path, &result.history, Some(500.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(
        result.metrics.final_speed_rpm.unwrap() > 100.0,
        "Motor should have accelerated to significant speed"
    );
    assert!(!result.history.is_empty(), "History should be recorded");
}

#[test]
fn test_step_response_1000rpm() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 1.0,
        record_interval: 0.001,
        ..Default::default()
    };
    let mut sim = Simulation::new(params, config);

    let scenario = StepResponse::new(1000.0)
        .with_duration(1.0)
        .with_criteria(PerformanceCriteria::relaxed());

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/step_response_1000rpm.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/step_response_1000rpm.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Step Response to 1000 RPM");
        plot_speed(&plot_path, &result.history, Some(1000.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(
        result.metrics.final_speed_rpm.unwrap() > 200.0,
        "Motor should reach significant speed"
    );
}

#[test]
fn test_step_response_with_load() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 0.8,
        record_interval: 0.001,
        ..Default::default()
    };
    let mut sim = Simulation::new(params, config);

    let scenario = StepResponse::new(500.0)
        .with_load(0.005)
        .with_duration(0.8)
        .with_criteria(PerformanceCriteria::relaxed());

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/step_response_with_load.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/step_response_with_load.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Step Response with 5mNm Load");
        plot_speed(&plot_path, &result.history, Some(500.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(
        result.metrics.final_speed_rpm.unwrap() > 50.0,
        "Motor should accelerate even with load"
    );
}

// ============================================================================
// Ramp Response Tests
// ============================================================================

#[test]
fn test_ramp_0_to_500() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig::default();
    let mut sim = Simulation::new(params, config);

    let scenario = RampResponse::new(0.0, 500.0, 0.5)
        .with_hold_time(0.2)
        .with_max_error(200.0);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/ramp_0_to_500.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/ramp_0_to_500.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Ramp Response 0 to 500 RPM");
        plot_speed(&plot_path, &result.history, Some(500.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(
        result.metrics.final_speed_rpm.unwrap() > 100.0,
        "Motor should reach significant speed after ramp"
    );
    assert!(!result.history.is_empty());
}

#[test]
fn test_slow_ramp() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 1.5,
        ..Default::default()
    };
    let mut sim = Simulation::new(params, config);

    let scenario = RampResponse::new(0.0, 300.0, 1.0)
        .with_hold_time(0.3)
        .with_max_error(150.0);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/slow_ramp.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/slow_ramp.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Slow Ramp to 300 RPM");
        plot_speed(&plot_path, &result.history, Some(300.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(result.metrics.final_speed_rpm.is_some());
}

// ============================================================================
// Load Disturbance Tests
// ============================================================================

#[test]
fn test_small_load_disturbance() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 1.0,
        ..Default::default()
    };
    let mut sim = Simulation::new(params, config);

    let scenario = LoadDisturbance::new(500.0, 0.003)
        .with_load_time(0.3)
        .with_duration(0.8)
        .with_max_dip(80.0)
        .with_recovery_time(1000.0);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/load_disturbance_small.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/load_disturbance_small.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Load Disturbance 3mNm at 500 RPM");
        plot_speed(&plot_path, &result.history, Some(500.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(!result.history.is_empty());
    assert!(result.metrics.final_speed_rpm.is_some());
}

#[test]
fn test_large_load_disturbance() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 1.2,
        ..Default::default()
    };
    let mut sim = Simulation::new(params, config);

    let scenario = LoadDisturbance::new(500.0, 0.008)
        .with_load_time(0.3)
        .with_duration(1.0)
        .with_max_dip(90.0)
        .with_recovery_time(1500.0);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/load_disturbance_large.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/load_disturbance_large.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Load Disturbance 8mNm at 500 RPM");
        plot_speed(&plot_path, &result.history, Some(500.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(result.metrics.final_speed_rpm.is_some());
}

// ============================================================================
// Startup Tests
// ============================================================================

#[test]
fn test_startup_basic() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 0.8,
        ..Default::default()
    };
    let mut sim = Simulation::new(params, config);

    let scenario = StartupScenario::new(500.0)
        .with_max_time(800.0)
        .with_min_speed(50.0)
        .with_duration(0.6);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/startup_basic.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/startup_basic.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Startup to 500 RPM");
        plot_speed(&plot_path, &result.history, Some(500.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(
        result.metrics.final_speed_rpm.unwrap() > 50.0,
        "Motor should have started and reached minimum speed"
    );
    assert!(
        result.metrics.peak_current.is_some(),
        "Peak current should be recorded"
    );
}

#[test]
fn test_startup_high_speed() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 1.0,
        ..Default::default()
    };
    let mut sim = Simulation::new(params, config);

    let scenario = StartupScenario::new(1000.0)
        .with_max_time(1000.0)
        .with_min_speed(100.0)
        .with_duration(0.8);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/startup_high_speed.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/startup_high_speed.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Startup to 1000 RPM");
        plot_speed(&plot_path, &result.history, Some(1000.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(result.metrics.final_speed_rpm.is_some());
}

#[test]
fn test_startup_with_load() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 1.0,
        ..Default::default()
    };
    let mut sim = Simulation::new(params, config);

    let scenario = StartupScenario::new(500.0)
        .with_load(0.003)
        .with_max_time(1200.0)
        .with_min_speed(30.0)
        .with_duration(0.8);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/startup_with_load.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    #[cfg(feature = "visualization")]
    {
        let plot_path = format!("{}/startup_with_load.png", OUTPUT_DIR);
        let config = PlotConfig::default().with_title("Startup with 3mNm Load");
        plot_speed(&plot_path, &result.history, Some(500.0), &config)
            .expect("Failed to generate plot");
        println!("Plot output: {}", plot_path);
    }

    assert!(result.metrics.final_speed_rpm.is_some());
}

// ============================================================================
// State Machine Scenario Tests
// ============================================================================

#[test]
fn test_sm_startup_with_transition() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 3.0,
        record_interval: 0.001,
        ..Default::default()
    };
    let mut sim = StateMachineSimulation::new(params, config);

    let scenario = StartupWithTransitionScenario::new(500.0)
        .with_max_transition_time(2000.0)
        .with_max_target_time(3000.0)
        .with_duration(3.0);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/sm_startup_with_transition.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.base.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    // Verify state machine specific results
    assert!(
        !result.mode_history.is_empty() || result.mode_durations.openloop > 0.0,
        "Should have mode history or spent time in OpenLoop"
    );
    assert!(
        result.mode_durations.openloop > 0.0,
        "Should spend time in OpenLoop mode"
    );
    assert!(
        result.base.metrics.final_speed_rpm.is_some(),
        "Should have final speed"
    );

    println!(
        "Mode durations - OpenLoop: {:.3}s, FOC: {:.3}s",
        result.mode_durations.openloop, result.mode_durations.foc
    );
}

#[test]
fn test_sm_step_response() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 3.0,
        record_interval: 0.001,
        ..Default::default()
    };
    let mut sim = StateMachineSimulation::new(params, config);

    let scenario = StateMachineStepResponse::new(300.0, 600.0)
        .with_step_time(1.0)
        .with_duration(3.0);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/sm_step_response.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.base.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    assert!(
        result.base.metrics.overshoot_percent.is_some(),
        "Should calculate overshoot"
    );
    assert!(
        result.base.metrics.final_speed_rpm.is_some(),
        "Should have final speed"
    );

    if let Some(overshoot) = result.base.metrics.overshoot_percent {
        println!("Overshoot: {:.1}%", overshoot);
    }
}

#[test]
fn test_sm_load_disturbance() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 4.0,
        record_interval: 0.001,
        ..Default::default()
    };
    let mut sim = StateMachineSimulation::new(params, config);

    let scenario = StateMachineLoadDisturbance::new(500.0, 0.005)
        .with_load_time(2.0)
        .with_duration(4.0);

    let result = scenario.run(&mut sim);

    #[cfg(feature = "csv-output")]
    {
        let csv_path = format!("{}/sm_load_disturbance.csv", OUTPUT_DIR);
        write_to_csv(&csv_path, &result.base.history).expect("Failed to write CSV");
        println!("CSV output: {}", csv_path);
    }

    assert!(
        result.base.metrics.max_torque.is_some(),
        "Should record max torque"
    );
    assert_eq!(result.stall_count, 0, "Should not stall with moderate load");
}

#[test]
fn test_sm_mode_transition_tracking() {
    ensure_output_dir();

    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 2.0,
        record_interval: 0.001,
        ..Default::default()
    };
    let mut sim = StateMachineSimulation::new(params, config);

    sim.set_target_speed_rpm(500.0);
    sim.set_motor_enabled(true);

    // Run simulation and track mode transitions
    let mut saw_openloop = false;
    let mut saw_foc = false;

    sim.run_until(|result| {
        if result.control_mode == ControlMode::OpenLoop {
            saw_openloop = true;
        }
        if result.control_mode == ControlMode::Foc {
            saw_foc = true;
        }
        // Stop when we've seen both modes or time exceeds
        saw_openloop && saw_foc
    });

    assert!(saw_openloop, "Should start in OpenLoop mode");

    let final_mode = sim.control_mode();
    let mode_history = sim.mode_history();

    println!(
        "Final mode: {:?}, Mode history length: {}",
        final_mode,
        mode_history.len()
    );
}

#[test]
fn test_sm_motor_enable_disable() {
    let params = MotorParams::default_small_bldc();
    let config = SimConfig {
        duration: 0.5,
        ..Default::default()
    };
    let mut sim = StateMachineSimulation::new(params, config);

    // Start disabled
    sim.set_motor_enabled(false);
    sim.set_target_speed_rpm(500.0);

    // Step while disabled
    for _ in 0..100 {
        let result = sim.step();
        assert_eq!(result.pwm_duty.u, 0, "PWM should be zero when disabled");
        assert_eq!(result.pwm_duty.v, 0, "PWM should be zero when disabled");
        assert_eq!(result.pwm_duty.w, 0, "PWM should be zero when disabled");
    }

    // Enable motor
    sim.set_motor_enabled(true);

    // Step while enabled
    let mut saw_nonzero_pwm = false;
    for _ in 0..100 {
        let result = sim.step();
        if result.pwm_duty.u != 0 || result.pwm_duty.v != 0 || result.pwm_duty.w != 0 {
            saw_nonzero_pwm = true;
            break;
        }
    }

    assert!(saw_nonzero_pwm, "Should produce PWM output when enabled");
}
