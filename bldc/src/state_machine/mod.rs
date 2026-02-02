//! Motor control state machine
//!
//! Manages control mode transitions and provides a unified interface
//! for motor control across OpenLoop, FOC, and Calibration modes.
//!
//! # Example
//!
//! ```rust,ignore
//! use bldc::state_machine::{MotorStateMachine, StateMachineConfig};
//!
//! // Create state machine with default configuration
//! let mut state_machine = MotorStateMachine::new(
//!     StateMachineConfig::default(),
//!     hardware,
//!     control_input,
//!     status_output,
//! );
//!
//! // In control loop:
//! let duty = state_machine.update(dt);
//! // Apply duty to PWM
//! ```

mod config;
pub mod modes;

pub use config::StateMachineConfig;
pub use modes::{ControlState, FocMode, ModeOutput, OpenLoopMode, StateTransition};

#[cfg(feature = "calibration")]
pub use modes::CalibrationMode;

use crate::control::{FocConfig, OpenLoopConfig};
use crate::traits::{
    ControlInput, ControlMode, HallStateReader, PositionSensor, PwmDuty, PwmOutput, SpeedSensor,
    StatusOutput,
};

/// Motor control state machine
///
/// Coordinates control mode transitions and provides a unified interface
/// for motor control. The state machine owns the hardware, input, and output
/// adapters and manages state transitions automatically.
pub struct MotorStateMachine<H, I, O> {
    state: ControlState,
    config: StateMachineConfig,
    hw: H,
    input: I,
    output: O,
    last_mode: ControlMode,
}

impl<H, I, O> MotorStateMachine<H, I, O>
where
    H: HallStateReader + PositionSensor + SpeedSensor + PwmOutput,
    I: ControlInput,
    O: StatusOutput,
{
    /// Create a new state machine starting in OpenLoop mode
    pub fn new(config: StateMachineConfig, hw: H, input: I, output: O) -> Self {
        let openloop_config = config.openloop.clone();
        let state = ControlState::OpenLoop(OpenLoopMode::new(openloop_config, false));
        let last_mode = ControlMode::OpenLoop;

        Self {
            state,
            config,
            hw,
            input,
            output,
            last_mode,
        }
    }

    /// Update the state machine for one control cycle
    ///
    /// # Arguments
    /// * `dt` - Time step in seconds
    ///
    /// # Returns
    /// PWM duty cycles for U, V, W phases
    pub fn update(&mut self, dt: f32) -> PwmDuty {
        // Check if motor should be enabled
        if !self.input.motor_enabled() {
            return PwmDuty::default();
        }

        // Check for calibration request
        #[cfg(feature = "calibration")]
        if self.input.calibration_requested() && !matches!(self.state, ControlState::Calibration(_))
        {
            self.transition_to_calibration(self.input.calibration_torque());
        }

        // Execute current mode and get output
        let mode_output = match &mut self.state {
            ControlState::OpenLoop(mode) => mode.update(&mut self.hw, &self.input, dt),
            ControlState::Foc(mode) => mode.update(&mut self.hw, &self.input, dt),
            #[cfg(feature = "calibration")]
            ControlState::Calibration(mode) => mode.update(&mut self.hw, &self.input, dt),
        };

        // Update status output
        self.output
            .update_status(mode_output.speed_rpm, mode_output.electrical_angle);

        // Handle state transitions
        if let Some(transition) = mode_output.transition {
            self.apply_transition(transition);
        }

        // Notify on mode change
        let current_mode = self.state.mode();
        if current_mode != self.last_mode {
            self.output.on_mode_change(current_mode);
            self.last_mode = current_mode;
        }

        mode_output.duty
    }

    /// Apply a state transition
    fn apply_transition(&mut self, transition: StateTransition) {
        match transition {
            StateTransition::ToFoc {
                initial_vq,
                current_rpm,
                is_reverse: _,
            } => {
                self.transition_to_foc(initial_vq, current_rpm);
            }
            StateTransition::ToOpenLoop { is_recovery } => {
                if is_recovery {
                    self.output.on_stall_detected();
                }
                self.transition_to_openloop(is_recovery);
            }
            #[cfg(feature = "calibration")]
            StateTransition::ToCalibration { torque } => {
                self.transition_to_calibration(torque);
            }
        }
    }

    /// Transition to OpenLoop mode
    fn transition_to_openloop(&mut self, is_recovery: bool) {
        let openloop_config = self.config.openloop.clone();
        self.state = ControlState::OpenLoop(OpenLoopMode::new(openloop_config, is_recovery));
    }

    /// Transition to FOC mode
    fn transition_to_foc(&mut self, initial_vq: f32, current_rpm: f32) {
        let foc_mode = FocMode::new(self.config.foc.clone(), initial_vq, current_rpm);
        self.state = ControlState::Foc(foc_mode);
    }

    /// Transition to Calibration mode
    #[cfg(feature = "calibration")]
    fn transition_to_calibration(&mut self, torque: f32) {
        let mode = CalibrationMode::new(
            self.config.pole_pairs,
            self.config.max_duty,
            self.config.foc.v_dc,
            self.config.foc.max_voltage,
            torque,
        );
        self.state = ControlState::Calibration(mode);
    }

    /// Get current control mode
    pub fn mode(&self) -> ControlMode {
        self.state.mode()
    }

    /// Get reference to the current state
    pub fn state(&self) -> &ControlState {
        &self.state
    }

    /// Get mutable reference to the current state
    pub fn state_mut(&mut self) -> &mut ControlState {
        &mut self.state
    }

    /// Get reference to the hardware adapter
    pub fn hardware(&self) -> &H {
        &self.hw
    }

    /// Get mutable reference to the hardware adapter
    pub fn hardware_mut(&mut self) -> &mut H {
        &mut self.hw
    }

    /// Get reference to the input adapter
    pub fn input(&self) -> &I {
        &self.input
    }

    /// Get mutable reference to the input adapter
    pub fn input_mut(&mut self) -> &mut I {
        &mut self.input
    }

    /// Get reference to the output adapter
    pub fn output(&self) -> &O {
        &self.output
    }

    /// Get mutable reference to the output adapter
    pub fn output_mut(&mut self) -> &mut O {
        &mut self.output
    }

    /// Get reference to the configuration
    pub fn config(&self) -> &StateMachineConfig {
        &self.config
    }

    /// Reset state machine to initial OpenLoop state
    pub fn reset(&mut self) {
        self.transition_to_openloop(false);
        self.last_mode = ControlMode::OpenLoop;
    }

    /// Force transition to specific mode (for testing/debugging)
    pub fn force_mode(&mut self, mode: ControlMode) {
        match mode {
            ControlMode::OpenLoop => self.transition_to_openloop(false),
            ControlMode::Foc => self.transition_to_foc(0.0, 0.0),
            #[cfg(feature = "calibration")]
            ControlMode::Calibration => self.transition_to_calibration(0.1),
            #[cfg(not(feature = "calibration"))]
            ControlMode::Calibration => {} // No-op without calibration feature
        }
    }
}

/// Builder for MotorStateMachine with custom mode configurations
pub struct StateMachineBuilder<H, I, O> {
    config: StateMachineConfig,
    hw: H,
    input: I,
    output: O,
}

impl<H, I, O> StateMachineBuilder<H, I, O>
where
    H: HallStateReader + PositionSensor + SpeedSensor + PwmOutput,
    I: ControlInput,
    O: StatusOutput,
{
    /// Create a new builder
    pub fn new(hw: H, input: I, output: O) -> Self {
        Self {
            config: StateMachineConfig::default(),
            hw,
            input,
            output,
        }
    }

    /// Set the configuration
    pub fn with_config(mut self, config: StateMachineConfig) -> Self {
        self.config = config;
        self
    }

    /// Set the OpenLoop configuration
    pub fn with_openloop_config(mut self, config: OpenLoopConfig) -> Self {
        self.config.openloop = config;
        self
    }

    /// Set the FOC configuration
    pub fn with_foc_config(mut self, config: FocConfig) -> Self {
        self.config.foc = config;
        self
    }

    /// Set pole pairs
    pub fn with_pole_pairs(mut self, pole_pairs: u8) -> Self {
        self.config.pole_pairs = pole_pairs;
        self
    }

    /// Set max duty
    pub fn with_max_duty(mut self, max_duty: u16) -> Self {
        self.config.max_duty = max_duty;
        self
    }

    /// Build the state machine
    pub fn build(self) -> MotorStateMachine<H, I, O> {
        MotorStateMachine::new(self.config, self.hw, self.input, self.output)
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use std::vec::Vec;

    // Mock implementations for testing
    struct MockHardware {
        hall_state: u8,
        electrical_angle: f32,
        mechanical_angle: f32,
        speed_rpm: f32,
        duty: PwmDuty,
        enabled: bool,
    }

    impl Default for MockHardware {
        fn default() -> Self {
            Self {
                hall_state: 1,
                electrical_angle: 0.0,
                mechanical_angle: 0.0,
                speed_rpm: 0.0,
                duty: PwmDuty::default(),
                enabled: false,
            }
        }
    }

    impl HallStateReader for MockHardware {
        fn get_hall_state(&self) -> u8 {
            self.hall_state
        }
    }

    impl PositionSensor for MockHardware {
        fn electrical_angle(&self) -> f32 {
            self.electrical_angle
        }
        fn mechanical_angle(&self) -> f32 {
            self.mechanical_angle
        }
    }

    impl SpeedSensor for MockHardware {
        fn speed_rad_s(&self) -> f32 {
            self.speed_rpm * core::f32::consts::TAU / 60.0
        }
        fn speed_rpm(&self) -> f32 {
            self.speed_rpm
        }
    }

    impl PwmOutput for MockHardware {
        fn set_duty(&mut self, u: f32, v: f32, w: f32) {
            self.duty = PwmDuty::from_normalized(u, v, w, 100);
        }
        fn enable(&mut self) {
            self.enabled = true;
        }
        fn disable(&mut self) {
            self.enabled = false;
        }
    }

    struct MockInput {
        target_speed: f32,
        pi_gains: (f32, f32),
        calibration_requested: bool,
        calibration_torque: f32,
        motor_enabled: bool,
    }

    impl Default for MockInput {
        fn default() -> Self {
            Self {
                target_speed: 100.0,
                pi_gains: (0.5, 0.05),
                calibration_requested: false,
                calibration_torque: 0.1,
                motor_enabled: true,
            }
        }
    }

    impl ControlInput for MockInput {
        fn target_speed(&self) -> f32 {
            self.target_speed
        }
        fn pi_gains(&self) -> (f32, f32) {
            self.pi_gains
        }
        fn calibration_requested(&self) -> bool {
            self.calibration_requested
        }
        fn calibration_torque(&self) -> f32 {
            self.calibration_torque
        }
        fn motor_enabled(&self) -> bool {
            self.motor_enabled
        }
    }

    #[derive(Default)]
    struct MockOutput {
        last_speed: f32,
        last_angle: f32,
        mode_changes: Vec<ControlMode>,
        stall_count: u32,
    }

    impl StatusOutput for MockOutput {
        fn update_status(&mut self, speed_rpm: f32, electrical_angle: f32) {
            self.last_speed = speed_rpm;
            self.last_angle = electrical_angle;
        }
        fn on_mode_change(&mut self, mode: ControlMode) {
            self.mode_changes.push(mode);
        }
        fn on_stall_detected(&mut self) {
            self.stall_count += 1;
        }
    }

    #[test]
    fn test_new_state_machine() {
        let hw = MockHardware::default();
        let input = MockInput::default();
        let output = MockOutput::default();

        let sm = MotorStateMachine::new(StateMachineConfig::default(), hw, input, output);

        assert_eq!(sm.mode(), ControlMode::OpenLoop);
    }

    #[test]
    fn test_motor_disabled() {
        let hw = MockHardware::default();
        let input = MockInput {
            motor_enabled: false,
            ..Default::default()
        };
        let output = MockOutput::default();

        let mut sm = MotorStateMachine::new(StateMachineConfig::default(), hw, input, output);

        let duty = sm.update(0.001);

        // Should return zero duty when disabled
        assert_eq!(duty.u, 0);
        assert_eq!(duty.v, 0);
        assert_eq!(duty.w, 0);
    }

    #[test]
    fn test_builder() {
        let hw = MockHardware::default();
        let input = MockInput::default();
        let output = MockOutput::default();

        let sm = StateMachineBuilder::new(hw, input, output)
            .with_pole_pairs(8)
            .with_max_duty(1000)
            .build();

        assert_eq!(sm.config().pole_pairs, 8);
        assert_eq!(sm.config().max_duty, 1000);
    }
}
