//! 電気角オフセット計算
//!
//! 各セクターの測定角度から電気角オフセットを計算する純粋関数を提供します。

use crate::fmt::*;
use crate::foc::shaft_position::ShaftPosition;
use core::f32::consts::{PI, TAU};

/// 各セクターの期待される機械角（rad）
/// セクター1=0°, 2=60°, 3=120°, 4=180°, 5=240°, 6=300°
const EXPECTED_ANGLES: [f32; 7] = [
    0.0,            // インデックス0（未使用）
    0.0,            // セクター1: 0°
    PI / 3.0,       // セクター2: 60°
    2.0 * PI / 3.0, // セクター3: 120°
    PI,             // セクター4: 180°
    4.0 * PI / 3.0, // セクター5: 240°
    5.0 * PI / 3.0, // セクター6: 300°
];

/// 各セクターで記録した角度から電気角オフセットを計算
///
/// # Arguments
/// * `sector_angles` - 各セクターの測定角度（インデックス0は未使用、1-6がセクター1-6）
/// * `pole_pairs` - モーターの極対数
///
/// # Returns
/// 計算された電気角オフセット [rad]（0～2π）
pub fn calculate_electrical_offset(sector_angles: &[Option<f32>; 7], pole_pairs: u8) -> f32 {
    info!("Calculating electrical offset from sector angles:");
    let mut offset_sum = 0.0;
    let mut count = 0;

    for sector in 1..=6 {
        if let Some(measured_angle) = sector_angles[sector] {
            let offset = calculate_sector_offset(measured_angle, sector, pole_pairs);

            info!(
                "  Sector {}: measured={}° ({} rad), expected={}° ({} rad), offset={}° ({} rad)",
                sector,
                measured_angle * 180.0 / PI,
                measured_angle,
                EXPECTED_ANGLES[sector] * 180.0 / PI,
                EXPECTED_ANGLES[sector],
                offset * 180.0 / PI,
                offset
            );

            offset_sum += offset;
            count += 1;
        }
    }

    if count > 0 {
        // 平均オフセットを計算し、0～2πに正規化
        let average_offset = offset_sum / count as f32;
        let normalized_offset = ShaftPosition::clamp(average_offset);

        info!(
            "Average electrical offset: {} rad ({} deg)",
            normalized_offset,
            normalized_offset * 180.0 / PI
        );

        normalized_offset
    } else {
        error!("No sector angles recorded, using offset=0");
        0.0
    }
}

/// 単一セクターのオフセットを計算
///
/// # Arguments
/// * `measured_angle` - 測定された機械角 [rad]
/// * `sector` - セクター番号（1-6）
/// * `pole_pairs` - モーターの極対数
///
/// # Returns
/// 正規化されたオフセット [rad]（-π～+π）
fn calculate_sector_offset(measured_angle: f32, sector: usize, pole_pairs: u8) -> f32 {
    // 機械角から電気角へ変換
    let measured_electrical = measured_angle * pole_pairs as f32;
    let expected_electrical = EXPECTED_ANGLES[sector] * pole_pairs as f32;

    // オフセット = 測定値 - 期待値
    let offset = measured_electrical - expected_electrical;

    // -π～+πの範囲に正規化
    normalize_to_signed_pi(offset)
}

/// 角度を-π～+πの範囲に正規化
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
        // 完璧にアラインされた場合（オフセット0）
        let mut angles: [Option<f32>; 7] = [None; 7];
        angles[1] = Some(0.0); // セクター1: 0°
        angles[2] = Some(PI / 3.0); // セクター2: 60°
        angles[3] = Some(2.0 * PI / 3.0); // セクター3: 120°
        angles[4] = Some(PI); // セクター4: 180°
        angles[5] = Some(4.0 * PI / 3.0); // セクター5: 240°
        angles[6] = Some(5.0 * PI / 3.0); // セクター6: 300°

        let offset = calculate_electrical_offset(&angles, 1);
        // pole_pairs=1なので機械角=電気角
        assert!(offset.abs() < 0.01);
    }
}
