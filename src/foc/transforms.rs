// Coordinate transformations for FOC (Field Oriented Control)
// Includes Park and Clarke inverse transforms

use libm::{cosf, sinf, sqrtf};

/// Inverse Park transformation (dq → αβ)
///
/// Transforms from the rotating dq reference frame to the stationary αβ frame
///
/// # Arguments
/// * `vd` - d-axis voltage (aligned with rotor flux)
/// * `vq` - q-axis voltage (perpendicular to rotor flux, produces torque)
/// * `theta` - Electrical angle in radians
///
/// # Returns
/// Tuple of (v_alpha, v_beta) in the stationary frame
pub fn inverse_park(vd: f32, vq: f32, theta: f32) -> (f32, f32) {
    let cos_theta = cosf(theta);
    let sin_theta = sinf(theta);

    let v_alpha = vd * cos_theta - vq * sin_theta;
    let v_beta = vd * sin_theta + vq * cos_theta;

    (v_alpha, v_beta)
}

/// Inverse Clarke transformation (αβ → abc/uvw)
///
/// Transforms from the stationary αβ frame to three-phase voltages
///
/// # Arguments
/// * `v_alpha` - Alpha-axis voltage
/// * `v_beta` - Beta-axis voltage
///
/// # Returns
/// Tuple of (v_u, v_v, v_w) three-phase voltages
pub fn inverse_clarke(v_alpha: f32, v_beta: f32) -> (f32, f32, f32) {
    // Constants for Clarke transform
    const SQRT3_DIV_2: f32 = 0.866025404; // sqrt(3) / 2
    const ONE_DIV_2: f32 = 0.5;

    let v_u = v_alpha;
    let v_v = -ONE_DIV_2 * v_alpha + SQRT3_DIV_2 * v_beta;
    let v_w = -ONE_DIV_2 * v_alpha - SQRT3_DIV_2 * v_beta;

    (v_u, v_v, v_w)
}

/// Limit voltage vector to maximum magnitude
///
/// Applies circular limiting to the voltage vector in the dq frame
/// to ensure the magnitude doesn't exceed the maximum voltage
///
/// # Arguments
/// * `vd` - d-axis voltage
/// * `vq` - q-axis voltage
/// * `max_voltage` - Maximum allowed voltage magnitude
///
/// # Returns
/// Tuple of (vd_limited, vq_limited)
pub fn limit_voltage(vd: f32, vq: f32, max_voltage: f32) -> (f32, f32) {
    let magnitude = sqrtf(vd * vd + vq * vq);

    if magnitude > max_voltage {
        // Scale down both components proportionally
        let scale = max_voltage / magnitude;
        (vd * scale, vq * scale)
    } else {
        (vd, vq)
    }
}

/// Normalize angle to range [0, 2π)
///
/// # Arguments
/// * `angle` - Angle in radians
///
/// # Returns
/// Normalized angle in range [0, 2π)
pub fn normalize_angle(angle: f32) -> f32 {
    const TWO_PI: f32 = 6.283185307; // 2 * PI

    let mut normalized = angle;
    while normalized >= TWO_PI {
        normalized -= TWO_PI;
    }
    while normalized < 0.0 {
        normalized += TWO_PI;
    }
    normalized
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 0.0001;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    #[test]
    fn test_inverse_park_zero_angle() {
        let (v_alpha, v_beta) = inverse_park(1.0, 0.0, 0.0);
        assert!(approx_eq(v_alpha, 1.0));
        assert!(approx_eq(v_beta, 0.0));
    }

    #[test]
    fn test_inverse_clarke() {
        let (v_u, v_v, v_w) = inverse_clarke(1.0, 0.0);
        assert!(approx_eq(v_u, 1.0));
        assert!(approx_eq(v_v, -0.5));
        assert!(approx_eq(v_w, -0.5));
        // Sum should be zero for balanced three-phase
        assert!(approx_eq(v_u + v_v + v_w, 0.0));
    }

    #[test]
    fn test_limit_voltage() {
        let (vd, vq) = limit_voltage(10.0, 0.0, 5.0);
        assert!(approx_eq(vd, 5.0));
        assert!(approx_eq(vq, 0.0));

        let (vd, vq) = limit_voltage(3.0, 4.0, 10.0);
        // Magnitude is 5.0, which is less than 10.0, so no limiting
        assert!(approx_eq(vd, 3.0));
        assert!(approx_eq(vq, 4.0));
    }

    #[test]
    fn test_normalize_angle() {
        assert!(approx_eq(normalize_angle(0.0), 0.0));
        assert!(approx_eq(normalize_angle(7.0), 7.0 - 6.283185307));
        assert!(approx_eq(normalize_angle(-1.0), -1.0 + 6.283185307));
    }
}
