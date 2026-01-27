// Hall sensor processing for BLDC motor position and speed estimation
// Uses TIM4 hardware Hall interface for high-precision edge detection and speed calculation
// Implements foc-simple compatible mechanical angle based calculation

use crate::config::advance_angle::{
    BASE_ADVANCE_DEG, MAX_ADVANCE_DEG, MAX_SPEED_FOR_ADVANCE, MIN_SPEED_FOR_ADVANCE,
};
use crate::hall_tim;
use core::f32::consts::{FRAC_PI_2, FRAC_PI_6, PI, TAU};
use libm::fmodf;

/// 角度を [0, TAU) に高速正規化
/// whileループを使わず、fmodf で一発計算
#[inline(always)]
fn normalize_angle(angle: f32) -> f32 {
    let a = fmodf(angle, TAU);
    if a < 0.0 {
        a + TAU
    } else {
        a
    }
}

/// Hall state lookup table (foc-simple compatible)
/// Maps raw hall state (1-6) to normalized index (0-5)
/// Valid transition sequence: 1 -> 3 -> 2 -> 6 -> 4 -> 5 -> 1 (CW rotation)
/// Index mapping: [invalid, 0, 2, 1, 4, 5, 3, invalid]
/// Raw state:      [0,       1, 2, 3, 4, 5, 6, 7]
const HALL_STATE_TABLE: [u8; 8] = [
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
const HALL_TO_ELECTRICAL_ANGLE: [f32; 8] = [
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
const MECHANICAL_TO_ELECTRICAL_OFFSET: f32 = 7.0 * PI / 6.0; // 210° = 3.665 rad

/// Hall sensor state machine for position and speed estimation
/// Implements foc-simple compatible mechanical angle based calculation
/// Relies on hall_tim (TIM4 hardware) for edge detection and speed calculation
pub struct HallSensor {
    /// Previous normalized hall state (0-5)
    prev_state: u8,
    /// Current mechanical angle in radians (shaft angle)
    mechanical_angle: f32,
    /// Hall index base (increments by 6 each electrical revolution)
    hall_idx_base: u32,
    /// Maximum hall index (pole_pairs * 6)
    hall_idx_max: u32,
    /// Angle per hall state (mechanical angle) = TAU / hall_idx_max
    angle_per_state: f32,
    /// Current speed in RPM (from TIM4)
    speed_rpm: f32,
    /// Time since last edge (for interpolation)
    time_since_edge: f32,
    /// Low-pass filter coefficient for speed (0.0 - 1.0)
    /// Lower value = more filtering
    speed_filter_alpha: f32,
    /// Number of pole pairs
    pole_pairs: u8,
    /// Enable angle interpolation between Hall edges
    enable_interpolation: bool,
    /// Electrical offset in radians (calibration value)
    electrical_offset: f32,
    /// Enable advance angle for improved efficiency
    enable_advance_angle: bool,
}

impl HallSensor {
    /// Create a new Hall sensor instance
    ///
    /// # Arguments
    /// * `pole_pairs` - Number of pole pairs in the motor
    /// * `speed_filter_alpha` - Low-pass filter coefficient (0.0-1.0, foc-simple uses 0.05)
    pub fn new(pole_pairs: u8, speed_filter_alpha: f32) -> Self {
        let hall_idx_max = (pole_pairs as u32) * 6;
        let angle_per_state = TAU / (hall_idx_max as f32);

        Self {
            prev_state: 255, // Invalid initial state
            mechanical_angle: 0.0,
            hall_idx_base: 0,
            hall_idx_max,
            angle_per_state,
            speed_rpm: 0.0,
            time_since_edge: 0.0,
            speed_filter_alpha: speed_filter_alpha.clamp(0.0, 1.0),
            pole_pairs,
            enable_interpolation: true, // Enable angle interpolation by default
            electrical_offset: 0.0,
            enable_advance_angle: true, // Enable advance angle for improved efficiency
        }
    }

    /// Check if a hall state is valid
    ///
    /// # Arguments
    /// * `state` - Hall state (0-7)
    ///
    /// # Returns
    /// `true` if state is valid (1-6), `false` otherwise
    pub fn is_valid_state(state: u8) -> bool {
        (1..=6).contains(&state)
    }

    /// Update hall sensor state and estimate position/speed
    /// Uses foc-simple compatible mechanical angle based calculation
    /// Uses TIM4 hardware for both speed calculation and Hall state reading
    ///
    /// # Arguments
    /// * `dt` - Time step since last update (seconds) - used for angle interpolation
    ///
    /// # Returns
    /// Tuple of (electrical_angle in radians, speed in RPM)
    pub fn update(&mut self, dt: f32) -> (f32, f32) {
        // Get Hall state and period from TIM4 (read once for consistency)
        let raw_hall_state = hall_tim::get_hall_state();
        let period_cycles = hall_tim::get_period_cycles();
        let is_timeout = hall_tim::is_timeout();

        // Validate hall state
        if !Self::is_valid_state(raw_hall_state) {
            // Check timeout from TIM4
            // period_cycles == 0 も条件に含めることで、競合による誤判定を防止
            if is_timeout && period_cycles == 0 {
                self.speed_rpm = 0.0;
                self.time_since_edge = 0.0;
            } else {
                self.time_since_edge += dt;
            }

            // Calculate electrical angle from mechanical angle
            // 機械角ベース + 固定オフセット + キャリブレーションオフセット
            let electrical_angle = normalize_angle(
                self.mechanical_angle * (self.pole_pairs as f32)
                    + MECHANICAL_TO_ELECTRICAL_OFFSET
                    + self.electrical_offset,
            );

            return (electrical_angle, self.speed_rpm);
        }

        // Convert raw hall state to normalized index using lookup table (foc-simple compatible)
        let normalized_state = HALL_STATE_TABLE[raw_hall_state as usize];
        if normalized_state == 255 {
            // Invalid state (should not happen after is_valid_state check, but safety check)
            let electrical_angle = normalize_angle(
                self.mechanical_angle * (self.pole_pairs as f32)
                    + MECHANICAL_TO_ELECTRICAL_OFFSET
                    + self.electrical_offset,
            );

            return (electrical_angle, self.speed_rpm);
        }

        // Check for timeout (1秒以上Hallエッジがない場合のみ速度を0に)
        // period_cycles == 0 も条件に含めることで、TIMEOUT_FLAG と PERIOD_CYCLES の
        // 更新競合による誤判定を防止（Relaxed ordering 対策）
        if is_timeout && period_cycles == 0 {
            self.speed_rpm = 0.0;
            self.time_since_edge = 0.0;

            // Calculate mechanical angle from hall_idx (discrete, no interpolation)
            let hall_state_idx = self.hall_idx_base + (normalized_state as u32);
            self.mechanical_angle = normalize_angle((hall_state_idx as f32) * self.angle_per_state);

            // 電気角計算: 機械角ベース + 固定オフセット + キャリブレーションオフセット
            let electrical_angle = normalize_angle(
                self.mechanical_angle * (self.pole_pairs as f32)
                    + MECHANICAL_TO_ELECTRICAL_OFFSET
                    + self.electrical_offset,
            );

            return (electrical_angle, self.speed_rpm);
        }

        // period_cycles が 0 の場合は前回の速度を維持して継続
        // 補間も継続して適用
        if period_cycles == 0 {
            self.time_since_edge += dt;

            // 機械角の補間を適用
            let hall_state_idx = self.hall_idx_base + (normalized_state as u32);
            let base_mechanical_angle = (hall_state_idx as f32) * self.angle_per_state;

            if self.enable_interpolation && self.speed_rpm.abs() > 1.0 {
                let mechanical_omega = self.speed_rpm * (TAU / 60.0);
                let angle_increment = mechanical_omega * self.time_since_edge;
                self.mechanical_angle = normalize_angle(base_mechanical_angle + angle_increment);
            } else {
                self.mechanical_angle = base_mechanical_angle;
            }

            // 電気角計算（補間ベース）
            let base_electrical_angle = if self.enable_interpolation && self.speed_rpm.abs() > 1.0 {
                self.mechanical_angle * (self.pole_pairs as f32) + MECHANICAL_TO_ELECTRICAL_OFFSET
            } else {
                HALL_TO_ELECTRICAL_ANGLE[raw_hall_state as usize]
            };

            let mut electrical_angle = base_electrical_angle + self.electrical_offset;

            // 進角を適用
            if self.enable_advance_angle && self.speed_rpm > MIN_SPEED_FOR_ADVANCE {
                electrical_angle += self.calculate_advance_angle(self.speed_rpm);
            }

            return (normalize_angle(electrical_angle), self.speed_rpm);
        }

        // Calculate instant speed from TIM4 period
        let instant_rpm = hall_tim::calculate_speed_rpm(period_cycles, self.pole_pairs);

        // Detect state change (hall edge)
        let state_changed = normalized_state != self.prev_state && self.prev_state != 255;

        if state_changed {
            // Handle hall index wrapping (foc-simple compatible)
            // State 0 after state 5 means we completed an electrical revolution
            if normalized_state == 0 && self.prev_state == 5 {
                self.hall_idx_base += 6;
                if self.hall_idx_base >= self.hall_idx_max {
                    self.hall_idx_base = 0; // Wrap around after full mechanical revolution
                }
            }
            // State 5 after state 0 means we're going backwards
            else if normalized_state == 5 && self.prev_state == 0 {
                if self.hall_idx_base < 6 {
                    self.hall_idx_base = self.hall_idx_max - 6;
                } else {
                    self.hall_idx_base -= 6;
                }
            }

            // Apply low-pass filter to speed (foc-simple formula: new = (instant + 19*old)/20 for alpha=0.05)
            // Equivalent to: new = alpha*instant + (1-alpha)*old where alpha = 1/20 = 0.05
            // instant_rpm が 0 の場合はノイズ判定なので速度を更新しない
            if instant_rpm > 0.0 {
                self.speed_rpm = self.speed_filter_alpha * instant_rpm
                    + (1.0 - self.speed_filter_alpha) * self.speed_rpm;
            }

            // trace!(
            //     "Hall edge: {} -> {} (normalized: {} -> {}), period={} cycles, instant_rpm={}, filtered_rpm={}",
            //     self.prev_state,
            //     normalized_state,
            //     self.prev_state,
            //     normalized_state,
            //     period_cycles,
            //     instant_rpm,
            //     self.speed_rpm
            // );

            // Reset edge timer
            self.time_since_edge = 0.0;

            // Update previous state
            self.prev_state = normalized_state;
        } else {
            // Accumulate time since last edge
            self.time_since_edge += dt;

            // 初回（prev_state == 255）の場合は prev_state を初期化
            // これにより次回から状態変化が検出されるようになる
            if self.prev_state == 255 {
                self.prev_state = normalized_state;
                // 初回は速度フィルタも初期化
                if instant_rpm > 0.0 {
                    self.speed_rpm = instant_rpm;
                }
            }
            // 状態変化がない場合は速度を更新しない（ノイズ混入防止）
        }

        // Calculate mechanical angle from hall index (foc-simple method)
        let hall_state_idx = self.hall_idx_base + (normalized_state as u32);
        let base_mechanical_angle = (hall_state_idx as f32) * self.angle_per_state;

        // Apply angle interpolation if enabled and motor is moving
        self.mechanical_angle = if self.enable_interpolation && self.speed_rpm.abs() > 1.0 {
            // Calculate mechanical angular velocity (rad/s)
            let mechanical_omega = self.speed_rpm * (TAU / 60.0); // RPM to rad/s (2*PI/60)

            // Interpolate angle based on time since last edge
            let angle_increment = mechanical_omega * self.time_since_edge;
            normalize_angle(base_mechanical_angle + angle_increment)
        } else {
            // No interpolation or very low speed: use discrete Hall sensor angle
            base_mechanical_angle
        };

        // 電気角計算: 補間された機械角から計算（テーブル参照方式から変更）
        // 補間が有効でモーターが回転している場合は、機械角ベースの連続的な電気角を使用
        let base_electrical_angle = if self.enable_interpolation && self.speed_rpm.abs() > 1.0 {
            // 補間された機械角から電気角を計算 + 初期オフセット（テーブルとの整合性）
            self.mechanical_angle * (self.pole_pairs as f32) + MECHANICAL_TO_ELECTRICAL_OFFSET
        } else {
            // 低速時または補間無効時はHallテーブルから離散的な電気角を使用
            HALL_TO_ELECTRICAL_ANGLE[raw_hall_state as usize]
        };

        // オフセットを加算（キャリブレーション値）
        let mut electrical_angle = base_electrical_angle + self.electrical_offset;

        // 進角（Advance Angle）を適用
        // 高速回転時にトルク効率を最大化するため、電気角を進める
        if self.enable_advance_angle && self.speed_rpm > MIN_SPEED_FOR_ADVANCE {
            electrical_angle += self.calculate_advance_angle(self.speed_rpm);
        }

        (normalize_angle(electrical_angle), self.speed_rpm)
    }

    /// 進角を計算（速度に応じた線形補間）
    ///
    /// # Arguments
    /// * `speed_rpm` - 現在の速度 [RPM]
    ///
    /// # Returns
    /// 進角 [rad]
    fn calculate_advance_angle(&self, speed_rpm: f32) -> f32 {
        // 度からラジアンへの変換係数
        const DEG_TO_RAD: f32 = PI / 180.0;

        // 基本進角（常に適用）
        let base_advance_rad = BASE_ADVANCE_DEG * DEG_TO_RAD;

        // 速度が閾値以下なら基本進角のみ
        if speed_rpm <= MIN_SPEED_FOR_ADVANCE {
            return base_advance_rad;
        }

        // 速度比例の追加進角を計算
        let speed_ratio = ((speed_rpm - MIN_SPEED_FOR_ADVANCE)
            / (MAX_SPEED_FOR_ADVANCE - MIN_SPEED_FOR_ADVANCE))
            .clamp(0.0, 1.0);

        let additional_advance_rad =
            (MAX_ADVANCE_DEG - BASE_ADVANCE_DEG) * DEG_TO_RAD * speed_ratio;

        base_advance_rad + additional_advance_rad
    }

    /// Get current electrical angle in radians
    /// 機械角ベース + 固定オフセット + キャリブレーションオフセット
    #[allow(dead_code)]
    pub fn get_electrical_angle(&self) -> f32 {
        normalize_angle(
            self.mechanical_angle * (self.pole_pairs as f32)
                + MECHANICAL_TO_ELECTRICAL_OFFSET
                + self.electrical_offset,
        )
    }

    /// Get current mechanical angle in radians
    pub fn get_mechanical_angle(&self) -> f32 {
        self.mechanical_angle
    }

    /// Get current speed in RPM
    #[allow(dead_code)]
    pub fn get_speed_rpm(&self) -> f32 {
        self.speed_rpm
    }

    /// Reset the hall sensor state
    pub fn reset(&mut self) {
        self.prev_state = 255; // Invalid state
        self.mechanical_angle = 0.0;
        self.hall_idx_base = 0;
        self.speed_rpm = 0.0;
        self.time_since_edge = 0.0;
    }

    /// Reset speed filter and interpolation timer to a specific speed value
    /// This is useful when transitioning from OpenLoop to FOC mode to avoid
    /// transient effects from the low-pass filter
    ///
    /// # Arguments
    /// * `new_speed` - Speed value to set in RPM
    pub fn reset_speed_filter(&mut self, new_speed: f32) {
        self.speed_rpm = new_speed;
        self.time_since_edge = 0.0;
    }

    /// Enable or disable angle interpolation
    ///
    /// # Arguments
    /// * `enable` - True to enable interpolation, false for discrete Hall angles only
    #[allow(dead_code)]
    pub fn set_interpolation(&mut self, enable: bool) {
        self.enable_interpolation = enable;
    }

    /// Check if interpolation is enabled
    #[allow(dead_code)]
    pub fn is_interpolation_enabled(&self) -> bool {
        self.enable_interpolation
    }

    /// Set the speed filter coefficient
    ///
    /// # Arguments
    /// * `alpha` - Filter coefficient (0.0-1.0)
    ///   - Lower values = more filtering (smoother but slower response)
    ///   - Higher values = less filtering (faster but noisier)
    #[allow(dead_code)]
    pub fn set_filter_alpha(&mut self, alpha: f32) {
        self.speed_filter_alpha = alpha.clamp(0.0, 1.0);
    }

    /// Set the electrical offset (calibration value)
    ///
    /// # Arguments
    /// * `offset_rad` - Electrical offset in radians
    ///
    /// This is used to calibrate the motor. The electrical offset is the difference
    /// between the Hall sensor zero position and the motor's magnetic zero position.
    #[allow(dead_code)]
    pub fn set_electrical_offset(&mut self, offset_rad: f32) {
        self.electrical_offset = offset_rad;
    }

    /// Get the electrical offset
    #[allow(dead_code)]
    pub fn get_electrical_offset(&self) -> f32 {
        self.electrical_offset
    }

    /// Enable or disable advance angle
    ///
    /// # Arguments
    /// * `enable` - True to enable advance angle, false to disable
    #[allow(dead_code)]
    pub fn set_advance_angle(&mut self, enable: bool) {
        self.enable_advance_angle = enable;
    }

    /// Check if advance angle is enabled
    #[allow(dead_code)]
    pub fn is_advance_angle_enabled(&self) -> bool {
        self.enable_advance_angle
    }

    /// Get current advance angle in degrees for the current speed
    #[allow(dead_code)]
    pub fn get_current_advance_deg(&self) -> f32 {
        if !self.enable_advance_angle || self.speed_rpm <= MIN_SPEED_FOR_ADVANCE {
            return BASE_ADVANCE_DEG;
        }
        let advance_rad = self.calculate_advance_angle(self.speed_rpm);
        advance_rad * 180.0 / PI
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_states() {
        assert!(!HallSensor::is_valid_state(0));
        assert!(HallSensor::is_valid_state(1));
        assert!(HallSensor::is_valid_state(6));
        assert!(!HallSensor::is_valid_state(7));
    }

    #[test]
    fn test_hall_state_table() {
        // Test state mapping (foc-simple compatible)
        assert_eq!(HALL_STATE_TABLE[0], 255); // Invalid
        assert_eq!(HALL_STATE_TABLE[1], 0); // State 1 -> index 0
        assert_eq!(HALL_STATE_TABLE[2], 2); // State 2 -> index 2
        assert_eq!(HALL_STATE_TABLE[3], 1); // State 3 -> index 1
        assert_eq!(HALL_STATE_TABLE[4], 4); // State 4 -> index 4
        assert_eq!(HALL_STATE_TABLE[5], 5); // State 5 -> index 5
        assert_eq!(HALL_STATE_TABLE[6], 3); // State 6 -> index 3
        assert_eq!(HALL_STATE_TABLE[7], 255); // Invalid
    }

    #[test]
    fn test_angle_calculation() {
        // For pole_pairs = 6, hall_idx_max = 36
        // angle_per_state = TAU / 36 = 0.174533 rad (10 degrees)
        let pole_pairs = 6;
        let hall_idx_max = (pole_pairs as u32) * 6; // 36
        let angle_per_state = TAU / (hall_idx_max as f32);

        // Expected: ~0.174533 rad per state (10 degrees mechanical)
        let expected_deg = 360.0 / 36.0; // 10 degrees
        let expected_rad = expected_deg * core::f32::consts::PI / 180.0;

        assert!((angle_per_state - expected_rad).abs() < 0.001);
    }

    #[test]
    fn test_electrical_angle_calculation() {
        // Test electrical angle = mechanical_angle * pole_pairs - offset
        let sensor = HallSensor::new(6, 0.05);

        // With zero mechanical angle and zero offset
        assert_eq!(sensor.get_electrical_angle(), 0.0);

        // mechanical_angle = 0.174533 rad (10 deg), pole_pairs = 6
        // electrical_angle should be 1.047198 rad (60 deg)
        // This is because: 10 deg mechanical * 6 pole_pairs = 60 deg electrical
    }
}
