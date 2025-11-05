// Coordinate transformations for FOC (Field Oriented Control)
// Includes Park and Clarke inverse transforms

use libm::{cosf, sinf, sqrtf};

// Enable idsp-based fast trigonometric functions
const USE_IDSP_COSSIN: bool = true;

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
///
/// # Implementation
/// Uses idsp::cossin() for fast trigonometric calculation (~40 cycles on Cortex-M)
/// compared to libm::cosf/sinf (~100-200 cycles). Can be switched via USE_IDSP_COSSIN.
pub fn inverse_park(vd: f32, vq: f32, theta: f32) -> (f32, f32) {
    if USE_IDSP_COSSIN {
        inverse_park_idsp(vd, vq, theta)
    } else {
        inverse_park_libm(vd, vq, theta)
    }
}

/// Inverse Park using idsp::cossin() (fast, ~40 cycles on Cortex-M)
#[inline]
fn inverse_park_idsp(vd: f32, vq: f32, theta: f32) -> (f32, f32) {
    // Convert theta (radians, 0 to 2π) to idsp phase format (i32, full scale)
    // idsp uses i32::MIN (-2^31) to i32::MAX (2^31-1) to represent -π to π
    // First normalize theta from [0, 2π] to [-π, π]
    use core::f32::consts::{PI, TAU};
    let normalized_theta = if theta > PI { theta - TAU } else { theta };

    // Then scale to i32 range: phase = normalized_theta * (2^31 / π)
    const SCALE: f32 = 2147483648.0 / core::f32::consts::PI; // 2^31 / π
    let phase: i32 = (normalized_theta * SCALE) as i32;

    // cossin() returns (cos, sin) as (i32, i32) in range [-2^31, 2^31-1]
    let (cos_i32, sin_i32) = idsp::cossin(phase);

    // Convert i32 to f32 and normalize to [-1.0, 1.0]
    // Note: i32::MIN as f32 = -2147483648.0, but we want to normalize to 2^31
    const I32_TO_F32: f32 = 1.0 / 2147483648.0; // 1 / 2^31
    let cos_theta = cos_i32 as f32 * I32_TO_F32;
    let sin_theta = sin_i32 as f32 * I32_TO_F32;

    let v_alpha = vd * cos_theta - vq * sin_theta;
    let v_beta = vd * sin_theta + vq * cos_theta;

    (v_alpha, v_beta)
}

/// Inverse Park using libm (slower, ~100-200 cycles, but more familiar)
#[inline]
fn inverse_park_libm(vd: f32, vq: f32, theta: f32) -> (f32, f32) {
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
#[allow(dead_code)]
pub fn inverse_clarke(v_alpha: f32, v_beta: f32) -> (f32, f32, f32) {
    // Constants for Clarke transform
    const SQRT3_DIV_2: f32 = 0.866_025_4; // sqrt(3) / 2
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
#[allow(dead_code)]
pub fn normalize_angle(angle: f32) -> f32 {
    use core::f32::consts::TAU;

    let mut normalized = angle;
    while normalized >= TAU {
        normalized -= TAU;
    }
    while normalized < 0.0 {
        normalized += TAU;
    }
    normalized
}

/// Benchmark inverse_park implementations
///
/// Runs both idsp and libm implementations multiple times and returns
/// the relative performance difference
///
/// # Arguments
/// * `iterations` - Number of iterations to run
///
/// # Returns
/// Tuple of (idsp_result, libm_result, idsp_ticks, libm_ticks)
#[cfg(not(test))]
#[allow(dead_code)]
pub fn benchmark_inverse_park(iterations: u32) -> ((f32, f32), (f32, f32), u32, u32) {
    use cortex_m::peripheral::DWT;

    // Test parameters
    let vd = 12.0;
    let vq = 8.0;
    let theta = 1.57; // ~90 degrees

    // Enable cycle counter
    unsafe {
        let dwt = &*DWT::PTR;

        // Benchmark idsp implementation
        let start_idsp = dwt.cyccnt.read();
        let mut result_idsp = (0.0, 0.0);
        for _ in 0..iterations {
            result_idsp = inverse_park_idsp(vd, vq, theta);
            // Prevent optimization
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        }
        let end_idsp = dwt.cyccnt.read();

        // Benchmark libm implementation
        let start_libm = dwt.cyccnt.read();
        let mut result_libm = (0.0, 0.0);
        for _ in 0..iterations {
            result_libm = inverse_park_libm(vd, vq, theta);
            // Prevent optimization
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        }
        let end_libm = dwt.cyccnt.read();

        let ticks_idsp = end_idsp.wrapping_sub(start_idsp);
        let ticks_libm = end_libm.wrapping_sub(start_libm);

        (result_idsp, result_libm, ticks_idsp, ticks_libm)
    }
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
