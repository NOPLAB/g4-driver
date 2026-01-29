//! Clarke transformation for FOC (Field Oriented Control)
//!
//! The Clarke transform converts between three-phase (abc/uvw) quantities
//! and the two-phase stationary alpha-beta reference frame.

/// Inverse Clarke transformation (alpha-beta -> abc/uvw)
///
/// Transforms from the stationary alpha-beta frame to three-phase voltages.
///
/// # Arguments
/// * `v_alpha` - Alpha-axis voltage
/// * `v_beta` - Beta-axis voltage
///
/// # Returns
/// Tuple of (v_u, v_v, v_w) three-phase voltages
///
/// # Formula
/// ```text
/// v_u = v_alpha
/// v_v = -0.5 * v_alpha + (sqrt(3)/2) * v_beta
/// v_w = -0.5 * v_alpha - (sqrt(3)/2) * v_beta
/// ```
///
/// Note: This uses the amplitude-invariant form where v_u + v_v + v_w = 0
#[inline]
pub fn inverse_clarke(v_alpha: f32, v_beta: f32) -> (f32, f32, f32) {
    // Constants for Clarke transform
    const SQRT3_DIV_2: f32 = 0.866_025_4; // sqrt(3) / 2
    const ONE_DIV_2: f32 = 0.5;

    let v_u = v_alpha;
    let v_v = -ONE_DIV_2 * v_alpha + SQRT3_DIV_2 * v_beta;
    let v_w = -ONE_DIV_2 * v_alpha - SQRT3_DIV_2 * v_beta;

    (v_u, v_v, v_w)
}

/// Forward Clarke transformation (abc/uvw -> alpha-beta)
///
/// Transforms from three-phase quantities to the stationary alpha-beta frame.
///
/// # Arguments
/// * `v_u` - U phase voltage
/// * `v_v` - V phase voltage
/// * `v_w` - W phase voltage
///
/// # Returns
/// Tuple of (v_alpha, v_beta) in the stationary frame
///
/// # Formula (amplitude-invariant)
/// ```text
/// v_alpha = (2/3) * (v_u - 0.5*v_v - 0.5*v_w)
/// v_beta = (2/3) * (sqrt(3)/2 * v_v - sqrt(3)/2 * v_w)
/// ```
///
/// For balanced three-phase (v_u + v_v + v_w = 0):
/// ```text
/// v_alpha = v_u
/// v_beta = (v_v - v_w) / sqrt(3)
/// ```
#[inline]
#[allow(dead_code)]
pub fn forward_clarke(v_u: f32, v_v: f32, v_w: f32) -> (f32, f32) {
    // Constants for Clarke transform
    const TWO_DIV_3: f32 = 2.0 / 3.0;
    const SQRT3_DIV_2: f32 = 0.866_025_4; // sqrt(3) / 2
    const ONE_DIV_SQRT3: f32 = 0.577_350_3; // 1 / sqrt(3)

    // General form (works for unbalanced systems too)
    let v_alpha = TWO_DIV_3 * (v_u - 0.5 * v_v - 0.5 * v_w);
    let v_beta = TWO_DIV_3 * SQRT3_DIV_2 * (v_v - v_w);

    // Simplified form for balanced three-phase (faster):
    // let v_alpha = v_u;
    // let v_beta = (v_v - v_w) * ONE_DIV_SQRT3;

    let _ = ONE_DIV_SQRT3; // Suppress unused warning

    (v_alpha, v_beta)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 0.0001;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    #[test]
    fn test_inverse_clarke_alpha_only() {
        // v_alpha=1, v_beta=0 should give:
        // v_u = 1, v_v = -0.5, v_w = -0.5
        let (v_u, v_v, v_w) = inverse_clarke(1.0, 0.0);
        assert!(approx_eq(v_u, 1.0));
        assert!(approx_eq(v_v, -0.5));
        assert!(approx_eq(v_w, -0.5));
    }

    #[test]
    fn test_inverse_clarke_sum_zero() {
        // Sum of three-phase voltages should always be zero
        let (v_u, v_v, v_w) = inverse_clarke(1.0, 0.0);
        assert!(approx_eq(v_u + v_v + v_w, 0.0));

        let (v_u, v_v, v_w) = inverse_clarke(0.5, 0.866);
        assert!(approx_eq(v_u + v_v + v_w, 0.0));

        let (v_u, v_v, v_w) = inverse_clarke(-0.3, 0.4);
        assert!(approx_eq(v_u + v_v + v_w, 0.0));
    }

    #[test]
    fn test_forward_inverse_clarke_roundtrip() {
        // Forward then inverse should recover original values
        let v_alpha = 0.8;
        let v_beta = 0.6;

        let (v_u, v_v, v_w) = inverse_clarke(v_alpha, v_beta);
        let (v_alpha_out, v_beta_out) = forward_clarke(v_u, v_v, v_w);

        assert!(approx_eq(v_alpha, v_alpha_out));
        assert!(approx_eq(v_beta, v_beta_out));
    }

    #[test]
    fn test_forward_clarke_balanced() {
        // For balanced three-phase: v_u = 1, v_v = -0.5, v_w = -0.5
        // Should give v_alpha = 1, v_beta = 0
        let (v_alpha, v_beta) = forward_clarke(1.0, -0.5, -0.5);
        assert!(approx_eq(v_alpha, 1.0));
        assert!(approx_eq(v_beta, 0.0));
    }
}
