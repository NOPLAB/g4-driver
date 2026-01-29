//! Park transformation for FOC (Field Oriented Control)
//!
//! The Park transform converts between the rotating dq reference frame
//! and the stationary alpha-beta reference frame.

use libm::{cosf, sinf};

/// Inverse Park transformation (dq -> alpha-beta)
///
/// Transforms from the rotating dq reference frame to the stationary alpha-beta frame.
///
/// # Arguments
/// * `vd` - d-axis voltage (aligned with rotor flux)
/// * `vq` - q-axis voltage (perpendicular to rotor flux, produces torque)
/// * `theta` - Electrical angle in radians
///
/// # Returns
/// Tuple of (v_alpha, v_beta) in the stationary frame
///
/// # Formula
/// ```text
/// v_alpha = vd * cos(theta) - vq * sin(theta)
/// v_beta  = vd * sin(theta) + vq * cos(theta)
/// ```
#[inline]
pub fn inverse_park(vd: f32, vq: f32, theta: f32) -> (f32, f32) {
    let cos_theta = cosf(theta);
    let sin_theta = sinf(theta);

    let v_alpha = vd * cos_theta - vq * sin_theta;
    let v_beta = vd * sin_theta + vq * cos_theta;

    (v_alpha, v_beta)
}

/// Forward Park transformation (alpha-beta -> dq)
///
/// Transforms from the stationary alpha-beta frame to the rotating dq reference frame.
///
/// # Arguments
/// * `v_alpha` - Alpha-axis voltage
/// * `v_beta` - Beta-axis voltage
/// * `theta` - Electrical angle in radians
///
/// # Returns
/// Tuple of (vd, vq) in the rotating frame
///
/// # Formula
/// ```text
/// vd = v_alpha * cos(theta) + v_beta * sin(theta)
/// vq = -v_alpha * sin(theta) + v_beta * cos(theta)
/// ```
#[inline]
#[allow(dead_code)]
pub fn forward_park(v_alpha: f32, v_beta: f32, theta: f32) -> (f32, f32) {
    let cos_theta = cosf(theta);
    let sin_theta = sinf(theta);

    let vd = v_alpha * cos_theta + v_beta * sin_theta;
    let vq = -v_alpha * sin_theta + v_beta * cos_theta;

    (vd, vq)
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
    fn test_inverse_park_zero_angle() {
        // At theta=0: cos=1, sin=0
        // v_alpha = vd*1 - vq*0 = vd
        // v_beta = vd*0 + vq*1 = vq
        let (v_alpha, v_beta) = inverse_park(1.0, 0.0, 0.0);
        assert!(approx_eq(v_alpha, 1.0));
        assert!(approx_eq(v_beta, 0.0));
    }

    #[test]
    fn test_inverse_park_90_degrees() {
        // At theta=PI/2: cos=0, sin=1
        // v_alpha = vd*0 - vq*1 = -vq
        // v_beta = vd*1 + vq*0 = vd
        let (v_alpha, v_beta) = inverse_park(1.0, 2.0, PI / 2.0);
        assert!(approx_eq(v_alpha, -2.0));
        assert!(approx_eq(v_beta, 1.0));
    }

    #[test]
    fn test_forward_inverse_park_roundtrip() {
        let vd = 3.0;
        let vq = 4.0;
        let theta = 1.2;

        let (v_alpha, v_beta) = inverse_park(vd, vq, theta);
        let (vd_out, vq_out) = forward_park(v_alpha, v_beta, theta);

        assert!(approx_eq(vd, vd_out));
        assert!(approx_eq(vq, vq_out));
    }

    #[test]
    fn test_magnitude_preservation() {
        // Park transform should preserve vector magnitude
        let vd = 3.0;
        let vq = 4.0;
        let theta = 0.7;
        let input_mag = libm::sqrtf(vd * vd + vq * vq);

        let (v_alpha, v_beta) = inverse_park(vd, vq, theta);
        let output_mag = libm::sqrtf(v_alpha * v_alpha + v_beta * v_beta);

        assert!(approx_eq(input_mag, output_mag));
    }
}
