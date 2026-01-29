//! Utility functions for coordinate transformations

use core::f32::consts::TAU;
use libm::{fmodf, sqrtf};

/// Limit voltage vector to maximum magnitude
///
/// Applies circular limiting to the voltage vector in the dq frame
/// to ensure the magnitude doesn't exceed the maximum voltage.
///
/// # Arguments
/// * `vd` - d-axis voltage
/// * `vq` - q-axis voltage
/// * `max_voltage` - Maximum allowed voltage magnitude
///
/// # Returns
/// Tuple of (vd_limited, vq_limited)
#[inline]
pub fn limit_voltage(vd: f32, vq: f32, max_voltage: f32) -> (f32, f32) {
    // Fast path: vd == 0 (common for SPMSM), skip sqrtf
    if vd == 0.0 {
        let vq_limited = vq.clamp(-max_voltage, max_voltage);
        return (0.0, vq_limited);
    }

    let magnitude_sq = vd * vd + vq * vq;
    let max_voltage_sq = max_voltage * max_voltage;

    if magnitude_sq > max_voltage_sq {
        // Scale down both components proportionally
        let scale = max_voltage / sqrtf(magnitude_sq);
        (vd * scale, vq * scale)
    } else {
        (vd, vq)
    }
}

/// Normalize angle to range [0, 2*PI)
///
/// Uses fast fmodf calculation instead of while loops for better performance.
///
/// # Arguments
/// * `angle` - Angle in radians
///
/// # Returns
/// Normalized angle in range [0, 2*PI)
#[inline(always)]
pub fn normalize_angle(angle: f32) -> f32 {
    let a = fmodf(angle, TAU);
    if a < 0.0 {
        a + TAU
    } else {
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    const EPSILON: f32 = 0.0001;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    #[test]
    fn test_limit_voltage_no_limiting() {
        let (vd, vq) = limit_voltage(3.0, 4.0, 10.0);
        // Magnitude is 5.0, which is less than 10.0, so no limiting
        assert!(approx_eq(vd, 3.0));
        assert!(approx_eq(vq, 4.0));
    }

    #[test]
    fn test_limit_voltage_with_limiting() {
        let (vd, vq) = limit_voltage(10.0, 0.0, 5.0);
        assert!(approx_eq(vd, 5.0));
        assert!(approx_eq(vq, 0.0));
    }

    #[test]
    fn test_limit_voltage_vd_zero() {
        let (vd, vq) = limit_voltage(0.0, 10.0, 5.0);
        assert!(approx_eq(vd, 0.0));
        assert!(approx_eq(vq, 5.0));
    }

    #[test]
    fn test_limit_voltage_preserves_ratio() {
        let (vd, vq) = limit_voltage(6.0, 8.0, 5.0);
        // Magnitude was 10, now should be 5
        let magnitude = sqrtf(vd * vd + vq * vq);
        assert!(approx_eq(magnitude, 5.0));
        // Ratio should be preserved: vd/vq = 6/8 = 0.75
        assert!(approx_eq(vd / vq, 0.75));
    }

    #[test]
    fn test_normalize_angle_positive() {
        let result = normalize_angle(PI);
        assert!(approx_eq(result, PI));
    }

    #[test]
    fn test_normalize_angle_negative() {
        let result = normalize_angle(-PI);
        assert!(approx_eq(result, PI));
    }

    #[test]
    fn test_normalize_angle_wrap_positive() {
        let result = normalize_angle(TAU + 1.0);
        assert!(approx_eq(result, 1.0));
    }

    #[test]
    fn test_normalize_angle_zero() {
        let result = normalize_angle(0.0);
        assert!(result.abs() < EPSILON);
    }

    #[test]
    fn test_normalize_angle_multiple_wraps() {
        let result = normalize_angle(3.0 * TAU + 0.5);
        assert!(approx_eq(result, 0.5));
    }
}
