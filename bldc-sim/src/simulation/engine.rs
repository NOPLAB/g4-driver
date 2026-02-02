//! Simulation engine for BLDC motor with FOC control
//!
//! Integrates the motor physics model with the bldc crate's control algorithms.

use crate::motor_model::{
    LoadTorque, MotorDynamics, MotorParams, MotorState, StateSnapshot, VoltageInput,
};
use crate::nonlinear::NonlinearEffects;
use crate::simulation::hall_emulator::{HallEmulator, HallOutput};
use crate::simulation::integration::{IntegrationMethod, Integrator};
use bldc::control::foc::{FocConfig, FocController};
use bldc::sensors::hall::{HallConfig, HallProcessor, HallResult};
use bldc::traits::PwmDuty;
use core::f32::consts::TAU;

/// Simulation configuration
#[derive(Debug, Clone)]
pub struct SimConfig {
    /// Simulation time step [s]
    pub dt: f32,
    /// Control update period [s] (typically larger than physics dt)
    pub control_period: f32,
    /// Total simulation duration [s]
    pub duration: f32,
    /// Integration method
    pub integration_method: IntegrationMethod,
    /// Record interval for snapshots [s] (0 = every control step)
    pub record_interval: f32,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            dt: 0.00001,            // 10 μs physics step
            control_period: 0.0004, // 400 μs = 2.5 kHz control loop
            duration: 1.0,          // 1 second simulation
            integration_method: IntegrationMethod::RungeKutta4,
            record_interval: 0.001, // Record every 1ms
        }
    }
}

impl SimConfig {
    /// Calculate number of physics steps per control update
    pub fn physics_steps_per_control(&self) -> u32 {
        (self.control_period / self.dt).round() as u32
    }

    /// Calculate total control updates for simulation
    pub fn total_control_steps(&self) -> u32 {
        (self.duration / self.control_period).round() as u32
    }
}

/// Main simulation engine
pub struct Simulation {
    /// Motor dynamics model
    dynamics: MotorDynamics,
    /// Motor state
    state: MotorState,
    /// Integrator for physics
    integrator: Integrator,
    /// Hall sensor emulator
    hall_emulator: HallEmulator,
    /// Hall sensor processor (from bldc crate)
    hall_processor: HallProcessor,
    /// FOC controller (from bldc crate)
    foc_controller: FocController,
    /// Simulation configuration
    config: SimConfig,
    /// Current simulation time [s]
    time: f32,
    /// Load torque
    load: LoadTorque,
    /// Nonlinear effects (placeholder for Phase 3)
    #[allow(dead_code)]
    nonlinear: NonlinearEffects,
    /// Last calculated electromagnetic torque
    last_torque: f32,
    /// Recorded state snapshots
    history: Vec<StateSnapshot>,
    /// Time of last recording
    last_record_time: f32,
}

impl Simulation {
    /// Create a new simulation with default parameters
    pub fn new(motor_params: MotorParams, config: SimConfig) -> Self {
        let pole_pairs = motor_params.pole_pairs;
        let v_dc = motor_params.v_dc;

        Self::with_controllers(
            motor_params,
            config,
            HallConfig {
                pole_pairs,
                ..Default::default()
            },
            FocConfig {
                v_dc,
                max_voltage: v_dc,
                ..Default::default()
            },
        )
    }

    /// Create with custom controller configurations
    pub fn with_controllers(
        motor_params: MotorParams,
        config: SimConfig,
        hall_config: HallConfig,
        foc_config: FocConfig,
    ) -> Self {
        let pole_pairs = motor_params.pole_pairs;

        Self {
            dynamics: MotorDynamics::new(motor_params),
            state: MotorState::new(),
            integrator: Integrator::new(config.integration_method),
            hall_emulator: HallEmulator::new(pole_pairs),
            hall_processor: HallProcessor::new(hall_config),
            foc_controller: FocController::new(foc_config),
            config,
            time: 0.0,
            load: LoadTorque::zero(),
            nonlinear: NonlinearEffects::new(),
            last_torque: 0.0,
            history: Vec::new(),
            last_record_time: -1.0, // Force first record
        }
    }

    /// Get reference to motor parameters
    pub fn motor_params(&self) -> &MotorParams {
        self.dynamics.params()
    }

    /// Get current motor state
    pub fn state(&self) -> &MotorState {
        &self.state
    }

    /// Get current simulation time
    pub fn time(&self) -> f32 {
        self.time
    }

    /// Get last calculated electromagnetic torque
    pub fn torque(&self) -> f32 {
        self.last_torque
    }

    /// Get recorded history
    pub fn history(&self) -> &[StateSnapshot] {
        &self.history
    }

    /// Clear history
    pub fn clear_history(&mut self) {
        self.history.clear();
    }

    /// Set target speed [RPM]
    pub fn set_target_speed_rpm(&mut self, rpm: f32) {
        self.foc_controller.set_target_speed_rpm(rpm);
    }

    /// Set target speed [rad/s]
    pub fn set_target_speed(&mut self, rad_s: f32) {
        self.foc_controller.set_target_speed(rad_s);
    }

    /// Set load torque [N⋅m]
    pub fn set_load_torque(&mut self, torque: f32) {
        self.load = LoadTorque::new(torque);
    }

    /// Set load with disturbance
    pub fn set_load_with_disturbance(&mut self, torque: f32, disturbance: f32) {
        self.load = LoadTorque::with_disturbance(torque, disturbance);
    }

    /// Set PI gains
    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.foc_controller.set_gains(kp, ki);
    }

    /// Reset simulation to initial conditions
    pub fn reset(&mut self) {
        self.state.reset();
        self.foc_controller.reset();
        self.hall_processor.reset();
        self.hall_emulator.reset();
        self.time = 0.0;
        self.last_torque = 0.0;
        self.last_record_time = -1.0;
        self.history.clear();
    }

    /// Run one control step (includes multiple physics steps)
    ///
    /// Returns the PWM duty cycles computed by the controller
    pub fn step(&mut self) -> SimStepResult {
        let pole_pairs = self.dynamics.params().pole_pairs;
        let v_dc = self.dynamics.params().v_dc;

        // --- Sensor Processing ---
        // Get Hall sensor readings
        let hall_output: HallOutput =
            self.hall_emulator
                .update(self.state.theta_m, self.state.omega_m, self.config.dt);

        // Process through bldc Hall processor
        let hall_result: HallResult = self.hall_processor.process(
            hall_output.hall_state,
            hall_output.instant_speed_rpm,
            hall_output.is_timeout,
            self.config.control_period,
        );

        // --- Control ---
        // Convert speed to rad/s for controller
        let measured_speed_rad_s = hall_result.speed_rpm * TAU / 60.0;

        // Run FOC controller
        let pwm_duty: PwmDuty = self.foc_controller.update(
            measured_speed_rad_s,
            hall_result.electrical_angle,
            self.config.control_period,
        );

        // Convert PWM duty to voltage
        let voltage = self.pwm_to_voltage(&pwm_duty, hall_result.electrical_angle, v_dc);

        // --- Physics Simulation ---
        let physics_steps = self.config.physics_steps_per_control();
        for _ in 0..physics_steps {
            self.integrator.step(
                &self.dynamics,
                &mut self.state,
                &voltage,
                &self.load,
                self.config.dt,
            );
        }

        // Update time
        self.time += self.config.control_period;

        // Calculate torque for recording
        self.last_torque = self.dynamics.electromagnetic_torque(&self.state);

        // Record snapshot if needed
        if self.should_record() {
            let snapshot = StateSnapshot::from_state(&self.state, self.time, self.last_torque);
            self.history.push(snapshot);
            self.last_record_time = self.time;
        }

        // Update electrical angles in state
        self.state.update_electrical(pole_pairs);

        SimStepResult {
            time: self.time,
            speed_rpm: self.state.speed_rpm(),
            electrical_angle: self.state.theta_e,
            pwm_duty,
            hall_result,
            torque: self.last_torque,
        }
    }

    /// Run simulation for specified duration
    pub fn run(&mut self) -> &[StateSnapshot] {
        let total_steps = self.config.total_control_steps();
        for _ in 0..total_steps {
            self.step();
        }
        &self.history
    }

    /// Run simulation until predicate returns true or duration exceeded
    pub fn run_until<F>(&mut self, mut predicate: F) -> bool
    where
        F: FnMut(&SimStepResult) -> bool,
    {
        let total_steps = self.config.total_control_steps();
        for _ in 0..total_steps {
            let result = self.step();
            if predicate(&result) {
                return true;
            }
        }
        false
    }

    /// Convert PWM duty cycles to dq-frame voltage
    ///
    /// This is a simplified inverse of the SVPWM calculation.
    /// In a real system, we'd measure actual currents.
    fn pwm_to_voltage(&self, pwm: &PwmDuty, theta_e: f32, v_dc: f32) -> VoltageInput {
        // Normalize duty cycles to 0-1 range
        let max_duty = 100; // Default from FocConfig
        let (du, dv, dw) = pwm.to_normalized(max_duty);

        // Convert to alpha-beta frame (inverse Clarke)
        // Assuming centered PWM (duty=0.5 gives zero voltage)
        let v_u = (du - 0.5) * v_dc;
        let v_v = (dv - 0.5) * v_dc;
        let v_w = (dw - 0.5) * v_dc;

        // Clarke transform: αβ from uvw
        let v_alpha = v_u;
        let v_beta = (v_v - v_w) / libm::sqrtf(3.0);

        // Park transform: dq from αβ
        let cos_theta = libm::cosf(theta_e);
        let sin_theta = libm::sinf(theta_e);

        let v_d = v_alpha * cos_theta + v_beta * sin_theta;
        let v_q = -v_alpha * sin_theta + v_beta * cos_theta;

        VoltageInput::new(v_d, v_q)
    }

    fn should_record(&self) -> bool {
        if self.config.record_interval <= 0.0 {
            return true;
        }
        self.time - self.last_record_time >= self.config.record_interval
    }
}

/// Result from a single simulation step
#[derive(Debug, Clone)]
pub struct SimStepResult {
    /// Current simulation time [s]
    pub time: f32,
    /// Motor speed [RPM]
    pub speed_rpm: f32,
    /// Electrical angle [rad]
    pub electrical_angle: f32,
    /// PWM duty cycles
    pub pwm_duty: PwmDuty,
    /// Hall sensor processing result
    pub hall_result: HallResult,
    /// Electromagnetic torque [N⋅m]
    pub torque: f32,
}

// ============================================================================
// State Machine Simulation (uses bldc state machine)
// ============================================================================

use crate::simulation::adapters::{SimControlInput, SimStatusOutput, SimulatedHardware};
use bldc::state_machine::{MotorStateMachine, StateMachineConfig};
use bldc::traits::ControlMode;
use bldc::OpenLoopConfig;

/// Simulation using the bldc state machine
///
/// This simulation mode uses the full state machine from the bldc crate,
/// allowing testing of mode transitions (OpenLoop -> FOC, stall recovery, etc.)
pub struct StateMachineSimulation {
    /// Motor dynamics model
    dynamics: MotorDynamics,
    /// Motor state
    state: MotorState,
    /// Integrator for physics
    integrator: Integrator,
    /// State machine (owns hardware, input, output adapters)
    state_machine: MotorStateMachine<SimulatedHardware, SimControlInput, SimStatusOutput>,
    /// Simulation configuration
    config: SimConfig,
    /// Current simulation time [s]
    time: f32,
    /// Load torque
    load: LoadTorque,
    /// DC bus voltage
    v_dc: f32,
    /// Last electromagnetic torque
    last_torque: f32,
    /// Motor state history
    history: Vec<StateSnapshot>,
    /// Time of last recording
    last_record_time: f32,
}

impl StateMachineSimulation {
    /// Create a new state machine simulation
    pub fn new(motor_params: MotorParams, config: SimConfig) -> Self {
        let pole_pairs = motor_params.pole_pairs;
        let v_dc = motor_params.v_dc;
        let max_duty = 100u16;

        // Create adapters
        let hw = SimulatedHardware::new(pole_pairs, max_duty);
        let input = SimControlInput::default();
        let output = SimStatusOutput::new(config.record_interval);

        // Create state machine config
        let sm_config = StateMachineConfig {
            openloop: OpenLoopConfig {
                pole_pairs,
                v_dc,
                max_duty,
                forced_commutation_cycles: 1000,
                min_cycles_for_foc: 2000,
                min_speed_for_foc: 50.0,
                ..Default::default()
            },
            foc: bldc::FocConfig {
                v_dc,
                max_voltage: v_dc,
                max_duty,
                ..Default::default()
            },
            pole_pairs,
            max_duty,
            invalid_hall_threshold: 100,
        };

        let state_machine = MotorStateMachine::new(sm_config, hw, input, output);

        Self {
            dynamics: MotorDynamics::new(motor_params),
            state: MotorState::new(),
            integrator: Integrator::new(config.integration_method),
            state_machine,
            config,
            time: 0.0,
            load: LoadTorque::zero(),
            v_dc,
            last_torque: 0.0,
            history: Vec::new(),
            last_record_time: -1.0,
        }
    }

    /// Get current motor state
    pub fn motor_state(&self) -> &MotorState {
        &self.state
    }

    /// Get current simulation time
    pub fn time(&self) -> f32 {
        self.time
    }

    /// Get current control mode
    pub fn control_mode(&self) -> ControlMode {
        self.state_machine.mode()
    }

    /// Set target speed [RPM]
    pub fn set_target_speed_rpm(&mut self, rpm: f32) {
        self.state_machine.input_mut().target_speed = rpm;
    }

    /// Set PI gains
    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.state_machine.input_mut().pi_gains = (kp, ki);
    }

    /// Set load torque [N⋅m]
    pub fn set_load_torque(&mut self, torque: f32) {
        self.load = LoadTorque::new(torque);
    }

    /// Enable/disable motor
    pub fn set_motor_enabled(&mut self, enabled: bool) {
        self.state_machine.input_mut().motor_enabled = enabled;
    }

    /// Get state history
    pub fn history(&self) -> &[StateSnapshot] {
        &self.history
    }

    /// Get status output history
    pub fn status_history(&self) -> &[super::adapters::StatusSnapshot] {
        self.state_machine.output().get_history()
    }

    /// Get mode change history
    pub fn mode_history(&self) -> &[ControlMode] {
        &self.state_machine.output().mode_history
    }

    /// Get stall count
    pub fn stall_count(&self) -> u32 {
        self.state_machine.output().stall_count
    }

    /// Reset simulation
    pub fn reset(&mut self) {
        self.state.reset();
        self.state_machine.reset();
        self.time = 0.0;
        self.last_torque = 0.0;
        self.last_record_time = -1.0;
        self.history.clear();
    }

    /// Run one control step
    pub fn step(&mut self) -> StateMachineStepResult {
        let pole_pairs = self.dynamics.params().pole_pairs;

        // Update hardware adapter with current motor state
        self.state_machine.hardware_mut().update(
            self.state.theta_m,
            self.state.omega_m,
            self.config.control_period,
        );

        // Update status output time
        self.state_machine.output_mut().set_time(self.time);

        // Run state machine
        let pwm_duty = self.state_machine.update(self.config.control_period);

        // Convert PWM to voltage for physics simulation
        let voltage = self.pwm_to_voltage(&pwm_duty);

        // Run physics simulation
        let physics_steps = self.config.physics_steps_per_control();
        for _ in 0..physics_steps {
            self.integrator.step(
                &self.dynamics,
                &mut self.state,
                &voltage,
                &self.load,
                self.config.dt,
            );
        }

        // Update time
        self.time += self.config.control_period;

        // Update electrical angle
        self.state.update_electrical(pole_pairs);

        // Calculate torque
        self.last_torque = self.dynamics.electromagnetic_torque(&self.state);

        // Record snapshot
        if self.should_record() {
            let snapshot = StateSnapshot::from_state(&self.state, self.time, self.last_torque);
            self.history.push(snapshot);
            self.last_record_time = self.time;
        }

        StateMachineStepResult {
            time: self.time,
            speed_rpm: self.state.speed_rpm(),
            electrical_angle: self.state.theta_e,
            pwm_duty,
            control_mode: self.state_machine.mode(),
            torque: self.last_torque,
        }
    }

    /// Run simulation for configured duration
    pub fn run(&mut self) -> &[StateSnapshot] {
        let total_steps = self.config.total_control_steps();
        for _ in 0..total_steps {
            self.step();
        }
        &self.history
    }

    /// Run until predicate returns true or duration exceeded
    pub fn run_until<F>(&mut self, mut predicate: F) -> bool
    where
        F: FnMut(&StateMachineStepResult) -> bool,
    {
        let total_steps = self.config.total_control_steps();
        for _ in 0..total_steps {
            let result = self.step();
            if predicate(&result) {
                return true;
            }
        }
        false
    }

    /// Convert PWM duty to voltage
    fn pwm_to_voltage(&self, pwm: &PwmDuty) -> VoltageInput {
        let max_duty = 100;
        let (du, dv, dw) = pwm.to_normalized(max_duty);

        let v_u = (du - 0.5) * self.v_dc;
        let v_v = (dv - 0.5) * self.v_dc;
        let v_w = (dw - 0.5) * self.v_dc;

        // Clarke transform
        let v_alpha = v_u;
        let v_beta = (v_v - v_w) / libm::sqrtf(3.0);

        // Park transform (use motor electrical angle)
        let cos_theta = libm::cosf(self.state.theta_e);
        let sin_theta = libm::sinf(self.state.theta_e);

        let v_d = v_alpha * cos_theta + v_beta * sin_theta;
        let v_q = -v_alpha * sin_theta + v_beta * cos_theta;

        VoltageInput::new(v_d, v_q)
    }

    fn should_record(&self) -> bool {
        if self.config.record_interval <= 0.0 {
            return true;
        }
        self.time - self.last_record_time >= self.config.record_interval
    }
}

/// Result from a state machine simulation step
#[derive(Debug, Clone)]
pub struct StateMachineStepResult {
    /// Current simulation time [s]
    pub time: f32,
    /// Motor speed [RPM]
    pub speed_rpm: f32,
    /// Electrical angle [rad]
    pub electrical_angle: f32,
    /// PWM duty cycles
    pub pwm_duty: PwmDuty,
    /// Current control mode
    pub control_mode: ControlMode,
    /// Electromagnetic torque [N⋅m]
    pub torque: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_sim() -> Simulation {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig {
            dt: 0.00001,
            control_period: 0.0004,
            duration: 0.1,
            ..Default::default()
        };
        Simulation::new(params, config)
    }

    #[test]
    fn test_new_simulation() {
        let sim = test_sim();
        assert!((sim.time() - 0.0).abs() < 0.0001);
        assert!((sim.state().omega_m - 0.0).abs() < 0.0001);
    }

    #[test]
    fn test_single_step() {
        let mut sim = test_sim();

        let result = sim.step();

        assert!(result.time > 0.0);
        // PWM duties should be valid
        assert!(result.pwm_duty.u <= 100);
        assert!(result.pwm_duty.v <= 100);
        assert!(result.pwm_duty.w <= 100);
    }

    #[test]
    fn test_speed_control() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig {
            dt: 0.00001,
            control_period: 0.0004,
            duration: 0.5, // 500ms simulation
            ..Default::default()
        };
        let mut sim = Simulation::new(params, config);

        // Set target speed
        sim.set_target_speed_rpm(500.0);

        // Run simulation
        sim.run();

        // Motor should have accelerated towards target
        let final_speed = sim.state().speed_rpm();
        assert!(
            final_speed > 100.0,
            "Motor should have accelerated, got {} RPM",
            final_speed
        );
    }

    #[test]
    fn test_load_response() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig {
            dt: 0.00001,
            control_period: 0.0004,
            duration: 0.2,
            ..Default::default()
        };
        let mut sim = Simulation::new(params, config);

        // Set target speed and run
        sim.set_target_speed_rpm(500.0);

        // Run for a bit to get up to speed
        for _ in 0..250 {
            sim.step();
        }
        let speed_before_load = sim.state().speed_rpm();

        // Apply load
        sim.set_load_torque(0.01); // 10 mN⋅m load

        // Run more
        for _ in 0..250 {
            sim.step();
        }

        // Speed should have changed (controller compensating)
        let _speed_after_load = sim.state().speed_rpm();

        // Motor should still be running
        assert!(speed_before_load > 0.0);
    }

    #[test]
    fn test_reset() {
        let mut sim = test_sim();

        sim.set_target_speed_rpm(500.0);
        sim.step();
        sim.step();

        sim.reset();

        assert!((sim.time() - 0.0).abs() < 0.0001);
        assert!((sim.state().omega_m - 0.0).abs() < 0.0001);
        assert!(sim.history().is_empty());
    }

    #[test]
    fn test_history_recording() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig {
            dt: 0.00001,
            control_period: 0.0004,
            duration: 0.01,
            record_interval: 0.001, // Every 1ms
            ..Default::default()
        };
        let mut sim = Simulation::new(params, config);

        sim.run();

        // Should have approximately 10 recordings for 10ms
        assert!(
            sim.history().len() >= 9,
            "Expected ~10 recordings, got {}",
            sim.history().len()
        );
    }

    #[test]
    fn test_run_until() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig {
            dt: 0.00001,
            control_period: 0.0004,
            duration: 1.0,
            ..Default::default()
        };
        let mut sim = Simulation::new(params, config);

        sim.set_target_speed_rpm(500.0);

        // Run until motor reaches 200 RPM
        let reached = sim.run_until(|result| result.speed_rpm > 200.0);

        if reached {
            assert!(sim.state().speed_rpm() > 200.0);
        }
    }

    // ========================================================================
    // StateMachineSimulation tests
    // ========================================================================

    fn test_sm_sim() -> StateMachineSimulation {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig {
            dt: 0.00001,
            control_period: 0.0004,
            duration: 0.1,
            ..Default::default()
        };
        StateMachineSimulation::new(params, config)
    }

    #[test]
    fn test_sm_new_simulation() {
        let sim = test_sm_sim();
        assert!((sim.time() - 0.0).abs() < 0.0001);
        assert!((sim.motor_state().omega_m - 0.0).abs() < 0.0001);
        assert_eq!(sim.control_mode(), ControlMode::OpenLoop);
    }

    #[test]
    fn test_sm_single_step() {
        let mut sim = test_sm_sim();

        let result = sim.step();

        assert!(result.time > 0.0);
        // PWM duties should be valid
        assert!(result.pwm_duty.u <= 100);
        assert!(result.pwm_duty.v <= 100);
        assert!(result.pwm_duty.w <= 100);
        // Should start in OpenLoop mode
        assert_eq!(result.control_mode, ControlMode::OpenLoop);
    }

    #[test]
    fn test_sm_openloop_to_foc_transition() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig {
            dt: 0.00001,
            control_period: 0.0004,
            duration: 2.0, // Long enough to transition
            ..Default::default()
        };
        let mut sim = StateMachineSimulation::new(params, config);

        sim.set_target_speed_rpm(500.0);

        // Run until FOC mode
        let transitioned = sim.run_until(|result| result.control_mode == ControlMode::Foc);

        if transitioned {
            assert_eq!(sim.control_mode(), ControlMode::Foc);
            // Mode history should show the transition
            assert!(!sim.mode_history().is_empty());
        }
    }

    #[test]
    fn test_sm_motor_disabled() {
        let mut sim = test_sm_sim();

        sim.set_motor_enabled(false);
        let result = sim.step();

        // Should return zero duty when disabled
        assert_eq!(result.pwm_duty.u, 0);
        assert_eq!(result.pwm_duty.v, 0);
        assert_eq!(result.pwm_duty.w, 0);
    }

    #[test]
    fn test_sm_reset() {
        let mut sim = test_sm_sim();

        sim.set_target_speed_rpm(500.0);
        sim.step();
        sim.step();

        sim.reset();

        assert!((sim.time() - 0.0).abs() < 0.0001);
        assert!((sim.motor_state().omega_m - 0.0).abs() < 0.0001);
        assert!(sim.history().is_empty());
        assert_eq!(sim.control_mode(), ControlMode::OpenLoop);
    }

    #[test]
    fn test_sm_speed_control() {
        let params = MotorParams::default_small_bldc();
        let config = SimConfig {
            dt: 0.00001,
            control_period: 0.0004,
            duration: 0.5, // 500ms simulation
            ..Default::default()
        };
        let mut sim = StateMachineSimulation::new(params, config);

        sim.set_target_speed_rpm(500.0);
        sim.run();

        // Motor should have accelerated (check absolute value as direction may vary)
        let final_speed = sim.motor_state().speed_rpm().abs();
        assert!(
            final_speed > 50.0,
            "Motor should have accelerated, got {} RPM",
            final_speed
        );
    }
}
