//! Adapters for integrating bldc crate traits with simulation
//!
//! Provides implementations of bldc traits for use with the simulation engine.

use bldc::sensors::hall::{HallConfig, HallProcessor, HallResult};
use bldc::traits::{
    ControlInput, ControlMode, HallStateReader, PositionSensor, PwmDuty, PwmOutput, SpeedSensor,
    StatusOutput,
};

use super::hall_emulator::{HallEmulator, HallOutput};

/// Simulated hardware adapter combining Hall emulator and processor
///
/// Implements bldc traits by combining HallEmulator (generates signals from physics)
/// and HallProcessor (processes signals like real firmware).
pub struct SimulatedHardware {
    /// Hall sensor emulator (generates signals from motor state)
    hall_emulator: HallEmulator,
    /// Hall sensor processor (from bldc crate)
    hall_processor: HallProcessor,
    /// Last Hall emulator output
    last_hall_output: HallOutput,
    /// Last Hall processor result
    last_hall_result: HallResult,
    /// PWM duty cycles (stored for verification)
    duty: PwmDuty,
    /// PWM enabled flag
    pwm_enabled: bool,
    /// Maximum duty value
    max_duty: u16,
}

impl SimulatedHardware {
    /// Create a new simulated hardware adapter
    pub fn new(pole_pairs: u8, max_duty: u16) -> Self {
        Self {
            hall_emulator: HallEmulator::new(pole_pairs),
            hall_processor: HallProcessor::new(HallConfig {
                pole_pairs,
                ..Default::default()
            }),
            last_hall_output: HallOutput::default(),
            last_hall_result: HallResult::default(),
            duty: PwmDuty::default(),
            pwm_enabled: false,
            max_duty,
        }
    }

    /// Update with motor state from physics simulation
    ///
    /// # Arguments
    /// * `theta_m` - Mechanical angle [rad]
    /// * `omega_m` - Mechanical angular velocity [rad/s]
    /// * `dt` - Time step [s]
    pub fn update(&mut self, theta_m: f32, omega_m: f32, dt: f32) {
        // Generate Hall signals from physics
        self.last_hall_output = self.hall_emulator.update(theta_m, omega_m, dt);

        // Process through bldc Hall processor
        self.last_hall_result = self.hall_processor.process(
            self.last_hall_output.hall_state,
            self.last_hall_output.instant_speed_rpm,
            self.last_hall_output.is_timeout,
            dt,
        );
    }

    /// Get the last PWM duty cycles
    pub fn get_duty(&self) -> PwmDuty {
        self.duty
    }

    /// Check if PWM is enabled
    pub fn is_pwm_enabled(&self) -> bool {
        self.pwm_enabled
    }

    /// Reset the adapter
    pub fn reset(&mut self) {
        self.hall_emulator.reset();
        self.hall_processor.reset();
        self.last_hall_output = HallOutput::default();
        self.last_hall_result = HallResult::default();
        self.duty = PwmDuty::default();
        self.pwm_enabled = false;
    }

    /// Get reference to Hall processor result
    pub fn hall_result(&self) -> &HallResult {
        &self.last_hall_result
    }

    /// Set electrical offset for Hall processor
    pub fn set_electrical_offset(&mut self, offset: f32) {
        self.hall_processor.set_electrical_offset(offset);
    }

    /// Set direction inversion
    pub fn set_direction_inversed(&mut self, inversed: bool) {
        self.hall_processor.set_direction_inversed(inversed);
    }
}

impl HallStateReader for SimulatedHardware {
    fn get_hall_state(&self) -> u8 {
        self.last_hall_output.hall_state
    }
}

impl PositionSensor for SimulatedHardware {
    fn electrical_angle(&self) -> f32 {
        self.last_hall_result.electrical_angle
    }

    fn mechanical_angle(&self) -> f32 {
        // Convert electrical angle to mechanical (approximate)
        // In real use, this would come from the Hall processor
        self.last_hall_result.electrical_angle / 6.0 // Assuming 6 pole pairs
    }
}

impl SpeedSensor for SimulatedHardware {
    fn speed_rad_s(&self) -> f32 {
        self.last_hall_result.speed_rpm * core::f32::consts::TAU / 60.0
    }

    fn speed_rpm(&self) -> f32 {
        self.last_hall_result.speed_rpm
    }
}

impl PwmOutput for SimulatedHardware {
    fn set_duty(&mut self, u: f32, v: f32, w: f32) {
        self.duty = PwmDuty::from_normalized(u, v, w, self.max_duty);
    }

    fn enable(&mut self) {
        self.pwm_enabled = true;
    }

    fn disable(&mut self) {
        self.pwm_enabled = false;
    }
}

/// Simulated control input for testing
#[derive(Debug, Clone)]
pub struct SimControlInput {
    /// Target speed in RPM
    pub target_speed: f32,
    /// PI gains (Kp, Ki)
    pub pi_gains: (f32, f32),
    /// Calibration requested flag
    pub calibration_requested: bool,
    /// Calibration torque
    pub calibration_torque: f32,
    /// Motor enabled flag
    pub motor_enabled: bool,
}

impl Default for SimControlInput {
    fn default() -> Self {
        Self {
            target_speed: 0.0,
            pi_gains: (0.5, 0.05),
            calibration_requested: false,
            calibration_torque: 0.1,
            motor_enabled: true,
        }
    }
}

impl ControlInput for SimControlInput {
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

/// Status snapshot for history recording
#[derive(Debug, Clone, Default)]
pub struct StatusSnapshot {
    /// Time [s]
    pub time: f32,
    /// Speed [RPM]
    pub speed_rpm: f32,
    /// Electrical angle [rad]
    pub electrical_angle: f32,
    /// Control mode
    pub mode: ControlMode,
    /// Stall detected flag
    pub stall_detected: bool,
}

/// Simulated status output for recording history
#[derive(Debug, Clone, Default)]
pub struct SimStatusOutput {
    /// Last speed [RPM]
    pub last_speed: f32,
    /// Last electrical angle [rad]
    pub last_angle: f32,
    /// Current control mode
    pub current_mode: ControlMode,
    /// Stall detection count
    pub stall_count: u32,
    /// Mode change history
    pub mode_history: Vec<ControlMode>,
    /// Status history
    pub history: Vec<StatusSnapshot>,
    /// Current simulation time (for history recording)
    pub current_time: f32,
    /// Record interval [s]
    pub record_interval: f32,
    /// Last record time [s]
    pub last_record_time: f32,
}

impl SimStatusOutput {
    /// Create a new status output with recording interval
    pub fn new(record_interval: f32) -> Self {
        Self {
            record_interval,
            last_record_time: -1.0,
            ..Default::default()
        }
    }

    /// Set current time for history recording
    pub fn set_time(&mut self, time: f32) {
        self.current_time = time;
    }

    /// Get recorded history
    pub fn get_history(&self) -> &[StatusSnapshot] {
        &self.history
    }

    /// Clear history
    pub fn clear_history(&mut self) {
        self.history.clear();
        self.last_record_time = -1.0;
    }
}

impl StatusOutput for SimStatusOutput {
    fn update_status(&mut self, speed_rpm: f32, electrical_angle: f32) {
        self.last_speed = speed_rpm;
        self.last_angle = electrical_angle;

        // Record if interval elapsed
        if self.record_interval > 0.0
            && (self.current_time - self.last_record_time >= self.record_interval
                || self.last_record_time < 0.0)
        {
            self.history.push(StatusSnapshot {
                time: self.current_time,
                speed_rpm,
                electrical_angle,
                mode: self.current_mode,
                stall_detected: false,
            });
            self.last_record_time = self.current_time;
        }
    }

    fn on_mode_change(&mut self, mode: ControlMode) {
        self.current_mode = mode;
        self.mode_history.push(mode);
    }

    fn on_stall_detected(&mut self) {
        self.stall_count += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulated_hardware_new() {
        let hw = SimulatedHardware::new(6, 100);
        assert!(!hw.is_pwm_enabled());
        assert_eq!(hw.get_duty().u, 0);
    }

    #[test]
    fn test_simulated_hardware_update() {
        let mut hw = SimulatedHardware::new(6, 100);

        // Simulate motor at 1 radian mechanical angle, 100 rad/s
        hw.update(1.0, 100.0, 0.001);

        // Should have valid Hall state
        assert!((1..=6).contains(&hw.get_hall_state()));
    }

    #[test]
    fn test_simulated_hardware_pwm() {
        let mut hw = SimulatedHardware::new(6, 100);

        hw.set_duty(0.5, 0.6, 0.7);
        hw.enable();

        assert!(hw.is_pwm_enabled());
        let duty = hw.get_duty();
        assert_eq!(duty.u, 50);
        assert_eq!(duty.v, 60);
        assert_eq!(duty.w, 70);

        hw.disable();
        assert!(!hw.is_pwm_enabled());
    }

    #[test]
    fn test_sim_control_input() {
        let input = SimControlInput {
            target_speed: 500.0,
            ..Default::default()
        };

        assert_eq!(input.target_speed(), 500.0);
        assert!(input.motor_enabled());
    }

    #[test]
    fn test_sim_status_output() {
        let mut output = SimStatusOutput::new(0.001);

        output.set_time(0.0);
        output.update_status(100.0, 1.0);

        assert_eq!(output.last_speed, 100.0);
        assert_eq!(output.history.len(), 1);

        output.on_mode_change(ControlMode::Foc);
        assert_eq!(output.current_mode, ControlMode::Foc);
        assert_eq!(output.mode_history.len(), 1);

        output.on_stall_detected();
        assert_eq!(output.stall_count, 1);
    }
}
