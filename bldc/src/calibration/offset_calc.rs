//! Electrical angle offset calculation
//!
//! Provides pure functions to calculate electrical angle offset from measured sector angles.

use crate::position::ShaftPosition;
use core::f32::consts::{PI, TAU};

/// Expected mechanical angles for each sector (rad)
/// Sector 1=0°, 2=60°, 3=120°, 4=180°, 5=240°, 6=300°
const EXPECTED_ANGLES: [f32; 7] = [
    0.0,            // Index 0 (unused)
    0.0,            // Sector 1: 0°
    PI / 3.0,       // Sector 2: 60°
    2.0 * PI / 3.0, // Sector 3: 120°
    PI,             // Sector 4: 180°
    4.0 * PI / 3.0, // Sector 5: 240°
    5.0 * PI / 3.0, // Sector 6: 300°
];

/// Calculate electrical angle offset from recorded angles for each sector
///
/// # Arguments
/// * `sector_angles` - Measured angles for each sector (index 0 unused, 1-6 are sectors 1-6)
/// * `pole_pairs` - Motor pole pairs
///
/// # Returns
/// Calculated electrical angle offset [rad] (0 to 2π)
pub fn calculate_electrical_offset(sector_angles: &[Option<f32>; 7], pole_pairs: u8) -> f32 {
    let mut offset_sum = 0.0;
    let mut count = 0;

    for (sector, angle_opt) in sector_angles.iter().enumerate().skip(1).take(6) {
        if let Some(measured_angle) = angle_opt {
            let offset = calculate_sector_offset(*measured_angle, sector, pole_pairs);
            offset_sum += offset;
            count += 1;
        }
    }

    if count > 0 {
        // Calculate average offset and normalize to 0 to 2π
        let average_offset = offset_sum / count as f32;
        ShaftPosition::clamp(average_offset)
    } else {
        0.0
    }
}

/// Calculate offset for a single sector
///
/// # Arguments
/// * `measured_angle` - Measured mechanical angle [rad]
/// * `sector` - Sector number (1-6)
/// * `pole_pairs` - Motor pole pairs
///
/// # Returns
/// Normalized offset [rad] (-π to +π)
fn calculate_sector_offset(measured_angle: f32, sector: usize, pole_pairs: u8) -> f32 {
    // Convert mechanical angle to electrical angle
    let measured_electrical = measured_angle * pole_pairs as f32;
    let expected_electrical = EXPECTED_ANGLES[sector] * pole_pairs as f32;

    // Offset = measured - expected
    let offset = measured_electrical - expected_electrical;

    // Normalize to -π to +π range
    normalize_to_signed_pi(offset)
}

/// Normalize angle to -π to +π range
#[inline]
fn normalize_to_signed_pi(mut angle: f32) -> f32 {
    while angle > PI {
        angle -= TAU;
    }
    while angle < -PI {
        angle += TAU;
    }
    angle
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_to_signed_pi() {
        assert!((normalize_to_signed_pi(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_to_signed_pi(TAU) - 0.0).abs() < 0.001);
        assert!((normalize_to_signed_pi(-TAU) - 0.0).abs() < 0.001);
        assert!((normalize_to_signed_pi(PI + 0.1) - (-PI + 0.1)).abs() < 0.001);
    }

    #[test]
    fn test_no_angles_returns_zero() {
        let angles: [Option<f32>; 7] = [None; 7];
        let offset = calculate_electrical_offset(&angles, 6);
        assert_eq!(offset, 0.0);
    }

    #[test]
    fn test_zero_offset_when_perfect_alignment() {
        // Perfect alignment case (offset 0)
        let mut angles: [Option<f32>; 7] = [None; 7];
        angles[1] = Some(0.0); // Sector 1: 0°
        angles[2] = Some(PI / 3.0); // Sector 2: 60°
        angles[3] = Some(2.0 * PI / 3.0); // Sector 3: 120°
        angles[4] = Some(PI); // Sector 4: 180°
        angles[5] = Some(4.0 * PI / 3.0); // Sector 5: 240°
        angles[6] = Some(5.0 * PI / 3.0); // Sector 6: 300°

        let offset = calculate_electrical_offset(&angles, 1);
        // pole_pairs=1 so mechanical angle = electrical angle
        assert!(offset.abs() < 0.01);
    }
}
