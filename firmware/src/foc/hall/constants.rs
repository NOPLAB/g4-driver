// Hall sensor constants and lookup tables
// Contains state tables and angle mappings for Hall sensor processing

use core::f32::consts::{FRAC_PI_2, FRAC_PI_6, PI};

/// Hall state lookup table (foc-simple compatible)
/// Maps raw hall state (1-6) to normalized index (0-5)
/// Valid transition sequence: 1 -> 3 -> 2 -> 6 -> 4 -> 5 -> 1 (CW rotation)
/// Index mapping: [invalid, 0, 2, 1, 4, 5, 3, invalid]
/// Raw state:      [0,       1, 2, 3, 4, 5, 6, 7]
pub const HALL_STATE_TABLE: [u8; 8] = [
    255, // 0b000: Invalid state (use 255 as marker)
    0,   // 0b001: State 1 -> index 0
    2,   // 0b010: State 2 -> index 2
    1,   // 0b011: State 3 -> index 1
    4,   // 0b100: State 4 -> index 4
    5,   // 0b101: State 5 -> index 5
    3,   // 0b110: State 6 -> index 3
    255, // 0b111: Invalid state (use 255 as marker)
];

/// Hall状態から直接電気角を取得するテーブル（ラジアン）
/// 6ステップ駆動の各Hall状態の中心電気角 + 180°（逆回転補正）
///
/// Hall駆動パターン（中心電気角 + 180°）:
///   Hall 1: 30° + 180° = 210° = 3.665 rad
///   Hall 3: 90° + 180° = 270° = 4.712 rad
///   Hall 2: 150° + 180° = 330° = 5.760 rad
///   Hall 6: 210° + 180° = 390° = 30° = 0.524 rad
///   Hall 4: 270° + 180° = 450° = 90° = 1.571 rad
///   Hall 5: 330° + 180° = 510° = 150° = 2.618 rad
pub const HALL_TO_ELECTRICAL_ANGLE: [f32; 8] = [
    0.0,              // 0b000: Invalid
    7.0 * FRAC_PI_6,  // 0b001: Hall 1 → 210° = 7π/6
    11.0 * FRAC_PI_6, // 0b010: Hall 2 → 330° = 11π/6
    3.0 * FRAC_PI_2,  // 0b011: Hall 3 → 270° = 3π/2
    FRAC_PI_2,        // 0b100: Hall 4 → 90° = π/2
    5.0 * FRAC_PI_6,  // 0b101: Hall 5 → 150° = 5π/6
    FRAC_PI_6,        // 0b110: Hall 6 → 30° = π/6
    0.0,              // 0b111: Invalid
];

/// 機械角から電気角への変換時の初期オフセット
/// Hall状態1（normalized_state=0）の電気角が210°になるように調整
/// テーブル値: 210° - 機械角ベース値: 0° = 210° = 7π/6
pub const MECHANICAL_TO_ELECTRICAL_OFFSET: f32 = 7.0 * PI / 6.0; // 210° = 3.665 rad

/// Invalid hall state marker
pub const INVALID_STATE: u8 = 255;
