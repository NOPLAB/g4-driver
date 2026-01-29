//! PWM output adapter for STM32G4
//!
//! Wraps the MotorDriver for use with the bldc crate traits.

use bldc::traits::{PwmDuty, PwmOutput};

use crate::motor_driver::MotorDriver;

/// PWM adapter that implements the bldc PwmOutput trait
pub struct PwmAdapter<'a> {
    driver: &'a mut MotorDriver,
}

impl<'a> PwmAdapter<'a> {
    /// Create a new PWM adapter
    ///
    /// # Arguments
    /// * `driver` - Reference to the motor driver
    pub fn new(driver: &'a mut MotorDriver) -> Self {
        Self { driver }
    }

    /// Apply PWM duty cycles from a PwmDuty struct
    pub fn apply_duty(&mut self, duty: PwmDuty) {
        self.driver.set_duty_uvw(duty.u, duty.v, duty.w);
    }

    /// Get the maximum duty value
    pub fn max_duty(&self) -> u16 {
        self.driver.max_duty()
    }
}

impl<'a> PwmOutput for PwmAdapter<'a> {
    fn set_duty(&mut self, u: f32, v: f32, w: f32) {
        let max = self.driver.max_duty() as f32;
        let duty_u = (u * max).clamp(0.0, max) as u16;
        let duty_v = (v * max).clamp(0.0, max) as u16;
        let duty_w = (w * max).clamp(0.0, max) as u16;
        self.driver.set_duty_uvw(duty_u, duty_v, duty_w);
    }

    fn enable(&mut self) {
        self.driver.enable_all_channels();
    }

    fn disable(&mut self) {
        self.driver.disable_all_channels();
    }
}
