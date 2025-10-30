// Space Vector PWM (SVPWM) generation
//
// This implementation is based on the calebfletcher/foc crate approach:
// https://github.com/calebfletcher/foc
//
// It uses a fast x/y/z coordinate transformation and sign-based sector
// detection instead of trigonometric functions, providing better
// performance and accuracy for embedded systems.

use libm::roundf;

const SQRT3: f32 = 1.732_050_8; // sqrt(3)

/// Calculate Space Vector PWM duty cycles
///
/// Implements SVPWM algorithm to generate three-phase PWM duty cycles
/// from alpha-beta voltage commands using fast x/y/z transformation.
/// This method avoids trigonometric functions for better performance.
///
/// Based on: https://github.com/calebfletcher/foc
///
/// # Arguments
/// * `v_alpha` - Alpha-axis voltage command (volts)
/// * `v_beta` - Beta-axis voltage command (volts)
/// * `v_dc` - DC bus voltage (volts)
/// * `max_duty` - Maximum duty cycle value (e.g., 100 for 0-100 range)
///
/// # Returns
/// Tuple of (duty_u, duty_v, duty_w) as u16 values
///
/// # Algorithm
/// 1. Normalize alpha/beta voltages by DC bus voltage
/// 2. Convert normalized alpha/beta to x/y/z coordinates
/// 3. Determine sector (1-6) based on signs of x/y/z
/// 4. Calculate duty cycles directly from x/y/z values
/// 5. Convert from range [-1, 1] to [0, max_duty]
pub fn calculate_svpwm(v_alpha: f32, v_beta: f32, v_dc: f32, max_duty: u16) -> (u16, u16, u16) {
    // Prevent division by zero
    if v_dc <= 0.0 {
        return (max_duty / 2, max_duty / 2, max_duty / 2);
    }

    // Normalize voltages by DC bus voltage
    // This maps the voltage commands to the range that can be achieved by the inverter
    let v_alpha_norm = v_alpha / v_dc;
    let v_beta_norm = v_beta / v_dc;

    // Convert normalized alpha/beta to x/y/z coordinates
    // This transformation maps the alpha-beta plane to three axes
    // that correspond to the six sectors of SVPWM
    let sqrt_3_alpha = SQRT3 * v_alpha_norm;
    let x = v_beta_norm;
    let y = (v_beta_norm + sqrt_3_alpha) / 2.0;
    let z = (v_beta_norm - sqrt_3_alpha) / 2.0;

    // Determine sector (1-6) based on signs of x, y, z
    // This is much faster than calculating angles with atan2
    let sector: u8 = match (x >= 0.0, y >= 0.0, z >= 0.0) {
        (true, true, false) => 1,
        (_, true, true) => 2,
        (true, false, true) => 3,
        (false, false, true) => 4,
        (_, false, false) => 5,
        (false, true, false) => 6,
    };

    // Calculate duty cycles for each phase based on sector
    // The ta, tb, tc values are in range [-1, 1]
    let (ta, tb, tc) = match sector {
        1 | 4 => (x - z, x + z, -x + z),
        2 | 5 => (y - z, y + z, -y - z),
        3 | 6 => (y - x, -y + x, -y - x),
        _ => (0.0, 0.0, 0.0), // Should never happen
    };

    // Convert from range [-1, 1] to [0, max_duty]
    // Formula: duty = (value + 1.0) / 2.0 * max_duty
    let duty_u = roundf((ta + 1.0) / 2.0 * max_duty as f32).clamp(0.0, max_duty as f32) as u16;
    let duty_v = roundf((tb + 1.0) / 2.0 * max_duty as f32).clamp(0.0, max_duty as f32) as u16;
    let duty_w = roundf((tc + 1.0) / 2.0 * max_duty as f32).clamp(0.0, max_duty as f32) as u16;

    (duty_u, duty_v, duty_w)
}

/// Calculate sinusoidal PWM duty cycles (simpler alternative to SVPWM)
///
/// Generates three-phase PWM duty cycles using direct sinusoidal modulation.
/// Simpler than SVPWM but provides ~15% less voltage utilization.
///
/// # Arguments
/// * `v_alpha` - Alpha-axis voltage command
/// * `v_beta` - Beta-axis voltage command
/// * `v_dc` - DC bus voltage
/// * `max_duty` - Maximum duty cycle value
///
/// # Returns
/// Tuple of (duty_u, duty_v, duty_w) as u16 values
#[allow(dead_code)]
pub fn calculate_sinusoidal_pwm(
    v_alpha: f32,
    v_beta: f32,
    v_dc: f32,
    max_duty: u16,
) -> (u16, u16, u16) {
    use super::transforms::inverse_clarke;

    // Prevent division by zero
    if v_dc <= 0.0 {
        return (0, 0, 0);
    }

    // Convert from αβ to three-phase
    let (v_u, v_v, v_w) = inverse_clarke(v_alpha, v_beta);

    // Normalize to DC bus voltage and convert to duty cycle
    // Add 0.5 offset to center around 50% duty cycle
    let duty_u = ((v_u / v_dc + 0.5) * max_duty as f32).clamp(0.0, max_duty as f32) as u16;
    let duty_v = ((v_v / v_dc + 0.5) * max_duty as f32).clamp(0.0, max_duty as f32) as u16;
    let duty_w = ((v_w / v_dc + 0.5) * max_duty as f32).clamp(0.0, max_duty as f32) as u16;

    (duty_u, duty_v, duty_w)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_svpwm_zero_voltage() {
        let (du, dv, dw) = calculate_svpwm(0.0, 0.0, 12.0, 100);
        // Zero voltage should result in ~50% duty cycle
        assert!(du > 40 && du < 60);
        assert!(dv > 40 && dv < 60);
        assert!(dw > 40 && dw < 60);
    }

    #[test]
    fn test_svpwm_sector1() {
        // Voltage vector in sector 1 (0-60 degrees)
        let (du, dv, dw) = calculate_svpwm(6.0, 0.0, 12.0, 100);
        // U phase should have highest duty cycle in sector 1
        assert!(du > dv && du > dw);
    }

    #[test]
    fn test_sinusoidal_pwm_zero_voltage() {
        let (du, dv, dw) = calculate_sinusoidal_pwm(0.0, 0.0, 12.0, 100);
        // Zero voltage should result in 50% duty cycle
        assert_eq!(du, 50);
        assert_eq!(dv, 50);
        assert_eq!(dw, 50);
    }
}
