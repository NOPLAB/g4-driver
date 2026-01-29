//! Dead time compensation
//!
//! Compensates for voltage distortion caused by PWM switching dead time.
//! Since there is no current sensing, the current direction of each phase is estimated
//! from the electrical angle and Vq command sign, and the duty ratio is corrected.

use libm::sinf;

/// Dead time compensator
pub struct DeadTimeCompensation {
    /// Compensation enabled/disabled
    enabled: bool,
    /// Compensation amount (duty equivalent, pre-calculated)
    compensation_duty: u16,
}

impl DeadTimeCompensation {
    /// Create new instance
    ///
    /// # Arguments
    /// * `dead_time_ns` - Dead time [ns]
    /// * `pwm_freq_hz` - PWM frequency [Hz]
    /// * `_v_dc` - DC bus voltage [V] (reserved for future use)
    /// * `max_duty` - Maximum duty value
    pub fn new(dead_time_ns: f32, pwm_freq_hz: u32, _v_dc: f32, max_duty: u16) -> Self {
        // Convert dead time voltage loss to duty ratio
        // Vdrop = Vdc * Td * Fpwm * 2 (dead time for both upper and lower arms)
        // duty_compensation = Vdrop / Vdc * max_duty = Td * Fpwm * 2 * max_duty
        let dead_time_s = dead_time_ns * 1e-9;
        let compensation_ratio = dead_time_s * pwm_freq_hz as f32 * 2.0;
        let compensation_duty = (compensation_ratio * max_duty as f32) as u16;

        Self {
            enabled: false,
            compensation_duty,
        }
    }

    /// Set compensation enabled/disabled
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Return whether compensation is enabled
    #[allow(dead_code)]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Apply dead time compensation
    ///
    /// # Arguments
    /// * `duty_u`, `duty_v`, `duty_w` - Duty values before compensation
    /// * `vq` - q-axis voltage command (for current direction estimation)
    /// * `theta` - Electrical angle [rad]
    /// * `max_duty` - Maximum duty value
    ///
    /// # Returns
    /// Compensated duty values (duty_u, duty_v, duty_w)
    pub fn compensate(
        &self,
        duty_u: u16,
        duty_v: u16,
        duty_w: u16,
        vq: f32,
        theta: f32,
        max_duty: u16,
    ) -> (u16, u16, u16) {
        if !self.enabled || self.compensation_duty == 0 {
            return (duty_u, duty_v, duty_w);
        }

        // Current direction estimation (estimate each phase current direction from Vq sign and electrical angle)
        // When Vq > 0, q-axis current is positive → each phase current is sin(theta), sin(theta - 2π/3), sin(theta + 2π/3)
        // When Vq < 0, sign is inverted
        let vq_sign = if vq >= 0.0 { 1.0 } else { -1.0 };

        // Estimate current direction of each phase (determined by sign of sin value)
        let phase_offset = core::f32::consts::FRAC_PI_3 * 2.0; // 2π/3
        let i_u_sign = sinf(theta) * vq_sign;
        let i_v_sign = sinf(theta - phase_offset) * vq_sign;
        let i_w_sign = sinf(theta + phase_offset) * vq_sign;

        // Apply compensation based on current direction
        // Current positive (high side to low side): dead time causes voltage drop → increase duty
        // Current negative (low side to high side): dead time causes voltage rise → decrease duty
        let comp = self.compensation_duty as i32;

        let new_duty_u = if i_u_sign > 0.0 {
            (duty_u as i32 + comp).min(max_duty as i32) as u16
        } else if i_u_sign < 0.0 {
            (duty_u as i32 - comp).max(0) as u16
        } else {
            duty_u
        };

        let new_duty_v = if i_v_sign > 0.0 {
            (duty_v as i32 + comp).min(max_duty as i32) as u16
        } else if i_v_sign < 0.0 {
            (duty_v as i32 - comp).max(0) as u16
        } else {
            duty_v
        };

        let new_duty_w = if i_w_sign > 0.0 {
            (duty_w as i32 + comp).min(max_duty as i32) as u16
        } else if i_w_sign < 0.0 {
            (duty_w as i32 - comp).max(0) as u16
        } else {
            duty_w
        };

        (new_duty_u, new_duty_v, new_duty_w)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compensation_disabled() {
        let comp = DeadTimeCompensation::new(100.0, 50000, 24.0, 1000);
        // When disabled, return input as is
        let (u, v, w) = comp.compensate(500, 500, 500, 10.0, 0.0, 1000);
        assert_eq!((u, v, w), (500, 500, 500));
    }

    #[test]
    fn test_compensation_enabled() {
        let mut comp = DeadTimeCompensation::new(100.0, 50000, 24.0, 1000);
        comp.set_enabled(true);

        // When theta = 0, Vq > 0
        // i_u: sin(0) = 0 → no compensation
        // i_v: sin(-2π/3) < 0 → decrease duty
        // i_w: sin(+2π/3) > 0 → increase duty
        let (u, v, w) = comp.compensate(500, 500, 500, 10.0, 0.0, 1000);

        // U phase sin(0)≈0 so no change
        assert_eq!(u, 500);
        // V phase decreases
        assert!(v < 500);
        // W phase increases
        assert!(w > 500);
    }

    #[test]
    fn test_compensation_clamp() {
        let mut comp = DeadTimeCompensation::new(1000.0, 50000, 24.0, 100);
        comp.set_enabled(true);

        // Even with extreme compensation values, clamp is applied
        let (u, v, w) = comp.compensate(95, 5, 50, 10.0, 0.5, 100);
        assert!(u <= 100);
        assert!(v <= 100);
        assert!(w <= 100);
    }
}
