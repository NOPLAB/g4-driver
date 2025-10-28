// PI (Proportional-Integral) controller with anti-windup

/// PI controller with anti-windup and output limiting
pub struct PiController {
    /// Proportional gain
    kp: f32,
    /// Integral gain
    ki: f32,
    /// Integral accumulator
    integral: f32,
    /// Minimum output limit
    output_min: f32,
    /// Maximum output limit
    output_max: f32,
    /// Last calculated output
    last_output: f32,
    /// Enable anti-windup (stops integral accumulation when saturated)
    anti_windup_enabled: bool,
}

impl PiController {
    /// Create a new PI controller
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    /// * `output_min` - Minimum output limit
    /// * `output_max` - Maximum output limit
    pub fn new(kp: f32, ki: f32, output_min: f32, output_max: f32) -> Self {
        Self {
            kp,
            ki,
            integral: 0.0,
            output_min,
            output_max,
            last_output: 0.0,
            anti_windup_enabled: true,
        }
    }

    /// Create a symmetric PI controller (output range: -limit to +limit)
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    /// * `output_limit` - Output limit (symmetric: ±output_limit)
    pub fn new_symmetric(kp: f32, ki: f32, output_limit: f32) -> Self {
        Self::new(kp, ki, -output_limit, output_limit)
    }

    /// Update the PI controller
    ///
    /// # Arguments
    /// * `setpoint` - Desired value
    /// * `measured` - Actual measured value
    /// * `dt` - Time step (seconds)
    ///
    /// # Returns
    /// Controller output (limited to output_min..output_max)
    pub fn update(&mut self, setpoint: f32, measured: f32, dt: f32) -> f32 {
        // Calculate error
        let error = setpoint - measured;

        // Proportional term
        let p_term = self.kp * error;

        // Integral term with anti-windup
        // Only accumulate if anti-windup is disabled, or if output is not saturated
        let should_integrate = !self.anti_windup_enabled
            || (self.last_output > self.output_min && self.last_output < self.output_max);

        if should_integrate {
            self.integral += error * dt;
        }

        let i_term = self.ki * self.integral;

        // Calculate output
        let output = p_term + i_term;

        // Apply output limits
        self.last_output = output.clamp(self.output_min, self.output_max);

        self.last_output
    }

    /// Reset the integral term to zero
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_output = 0.0;
    }

    /// Set the proportional and integral gains
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    pub fn set_gains(&mut self, kp: f32, ki: f32) {
        self.kp = kp;
        self.ki = ki;
    }

    /// Set the output limits
    ///
    /// # Arguments
    /// * `output_min` - Minimum output limit
    /// * `output_max` - Maximum output limit
    pub fn set_limits(&mut self, output_min: f32, output_max: f32) {
        self.output_min = output_min;
        self.output_max = output_max;
    }

    /// Set symmetric output limits (±limit)
    ///
    /// # Arguments
    /// * `output_limit` - Output limit (symmetric)
    pub fn set_symmetric_limit(&mut self, output_limit: f32) {
        self.output_min = -output_limit;
        self.output_max = output_limit;
    }

    /// Get the current output
    pub fn get_output(&self) -> f32 {
        self.last_output
    }

    /// Get the current integral term
    pub fn get_integral(&self) -> f32 {
        self.integral
    }

    /// Get the proportional gain
    pub fn get_kp(&self) -> f32 {
        self.kp
    }

    /// Get the integral gain
    pub fn get_ki(&self) -> f32 {
        self.ki
    }

    /// Enable or disable anti-windup
    ///
    /// # Arguments
    /// * `enabled` - True to enable anti-windup, false to disable
    pub fn set_anti_windup(&mut self, enabled: bool) {
        self.anti_windup_enabled = enabled;
    }

    /// Check if output is currently saturated
    pub fn is_saturated(&self) -> bool {
        self.last_output <= self.output_min || self.last_output >= self.output_max
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_proportional_only() {
        let mut pi = PiController::new(1.0, 0.0, -10.0, 10.0);
        let output = pi.update(5.0, 0.0, 0.1);
        assert_eq!(output, 5.0); // P term only
    }

    #[test]
    fn test_output_limiting() {
        let mut pi = PiController::new(1.0, 0.0, -10.0, 10.0);
        let output = pi.update(20.0, 0.0, 0.1);
        assert_eq!(output, 10.0); // Limited to max
    }

    #[test]
    fn test_integral_accumulation() {
        let mut pi = PiController::new(0.0, 1.0, -100.0, 100.0);
        // Error = 10, dt = 0.1, so integral should accumulate by 1.0 each step
        pi.update(10.0, 0.0, 0.1);
        assert_eq!(pi.get_integral(), 1.0);
        pi.update(10.0, 0.0, 0.1);
        assert_eq!(pi.get_integral(), 2.0);
    }
}
