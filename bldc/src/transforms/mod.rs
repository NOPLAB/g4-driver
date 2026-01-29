//! Coordinate transformations for FOC (Field Oriented Control)
//!
//! This module provides the Park and Clarke transforms used in FOC control.

mod clarke;
mod park;
mod utils;

pub use clarke::inverse_clarke;
pub use park::inverse_park;
pub use utils::{limit_voltage, normalize_angle};

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 0.001;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < EPSILON
    }

    #[test]
    fn test_foc_pipeline() {
        // Test the typical FOC pipeline: inverse_park -> inverse_clarke
        let vd = 0.0;
        let vq = 12.0;
        let theta = 0.0;

        let (v_alpha, v_beta) = inverse_park(vd, vq, theta);
        let (v_u, v_v, v_w) = inverse_clarke(v_alpha, v_beta);

        // At theta=0: v_alpha=0, v_beta=vq=12
        // inverse_clarke: v_u=0, v_v=sqrt(3)/2*12=10.39, v_w=-10.39
        assert!(approx_eq(v_alpha, 0.0));
        assert!(approx_eq(v_beta, 12.0));
        assert!(approx_eq(v_u, 0.0));
        assert!(approx_eq(v_u + v_v + v_w, 0.0)); // Sum should be zero
    }
}
