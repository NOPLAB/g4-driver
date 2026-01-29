// Utility functions for Hall sensor processing

use core::f32::consts::TAU;
use libm::fmodf;

/// Normalize angle to [0, TAU) range using fast fmodf calculation
/// Avoids while loops for better performance
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

    #[test]
    fn test_normalize_positive() {
        let result = normalize_angle(PI);
        assert!((result - PI).abs() < 0.001);
    }

    #[test]
    fn test_normalize_negative() {
        let result = normalize_angle(-PI);
        assert!((result - PI).abs() < 0.001);
    }

    #[test]
    fn test_normalize_wrap() {
        let result = normalize_angle(TAU + 1.0);
        assert!((result - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_normalize_zero() {
        let result = normalize_angle(0.0);
        assert!(result.abs() < 0.001);
    }
}
