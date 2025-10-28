// Space Vector PWM (SVPWM) generation

use libm::{atan2f, sqrtf};

const SQRT3: f32 = 1.732050808; // sqrt(3)
const ONE_DIV_SQRT3: f32 = 0.577350269; // 1 / sqrt(3)
const TWO_DIV_3: f32 = 0.666666667; // 2 / 3
const PI: f32 = 3.141592654;
const PI_DIV_3: f32 = 1.047197551; // PI / 3 (60 degrees)

/// Calculate Space Vector PWM duty cycles
///
/// Implements SVPWM algorithm to generate three-phase PWM duty cycles
/// from alpha-beta voltage commands. SVPWM provides better DC bus utilization
/// compared to sinusoidal PWM (15% improvement).
///
/// # Arguments
/// * `v_alpha` - Alpha-axis voltage command
/// * `v_beta` - Beta-axis voltage command
/// * `v_dc` - DC bus voltage
/// * `max_duty` - Maximum duty cycle value (e.g., 100 for 0-100 range)
///
/// # Returns
/// Tuple of (duty_u, duty_v, duty_w) as u16 values
pub fn calculate_svpwm(v_alpha: f32, v_beta: f32, v_dc: f32, max_duty: u16) -> (u16, u16, u16) {
    // Prevent division by zero
    if v_dc <= 0.0 {
        return (0, 0, 0);
    }

    // Normalize voltages to range [-1, 1]
    let v_alpha_norm = v_alpha / v_dc;
    let v_beta_norm = v_beta / v_dc;

    // Calculate voltage magnitude
    let magnitude = sqrtf(v_alpha_norm * v_alpha_norm + v_beta_norm * v_beta_norm);

    // Calculate angle (0 to 2π)
    let mut angle = atan2f(v_beta_norm, v_alpha_norm);
    if angle < 0.0 {
        angle += 2.0 * PI;
    }

    // Determine sector (1-6)
    let sector = ((angle / PI_DIV_3) as u8) + 1;

    // Calculate angle within sector (0 to π/3)
    let angle_in_sector = angle - ((sector - 1) as f32) * PI_DIV_3;

    // Calculate duty cycle times for adjacent vectors
    // Using the standard SVPWM equations
    let t1 = magnitude * SQRT3 * libm::sinf(PI_DIV_3 - angle_in_sector);
    let t2 = magnitude * SQRT3 * libm::sinf(angle_in_sector);
    let t0 = 1.0 - t1 - t2; // Zero vector time

    // Distribute zero vector time equally
    let t0_half = t0 / 2.0;

    // Calculate duty cycles based on sector
    let (ta, tb, tc) = match sector {
        1 => (t0_half + t1 + t2, t0_half + t2, t0_half),
        2 => (t0_half + t1, t0_half + t1 + t2, t0_half),
        3 => (t0_half, t0_half + t1 + t2, t0_half + t2),
        4 => (t0_half, t0_half + t1, t0_half + t1 + t2),
        5 => (t0_half + t2, t0_half, t0_half + t1 + t2),
        6 => (t0_half + t1 + t2, t0_half, t0_half + t1),
        _ => (0.5, 0.5, 0.5), // Default to 50% (shouldn't happen)
    };

    // Convert to duty cycle values (0 to max_duty)
    let duty_u = (ta * max_duty as f32).clamp(0.0, max_duty as f32) as u16;
    let duty_v = (tb * max_duty as f32).clamp(0.0, max_duty as f32) as u16;
    let duty_w = (tc * max_duty as f32).clamp(0.0, max_duty as f32) as u16;

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
