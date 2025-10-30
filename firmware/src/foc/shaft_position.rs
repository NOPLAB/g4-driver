//! シャフト位置管理モジュール
//!
//! このモジュールは、モーターのシャフト位置を管理します。
//! 複数回転を追跡し、角度と回転数の両方を保持します。

use core::f32::consts::TAU;

/// シャフトの位置を表す構造体
/// 角度（0～2π rad）と回転数（正または負の整数）を保持
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShaftPosition {
    /// 現在の角度 [rad] (0 ≤ angle < TAU)
    pub angle: f32,
    /// 回転数（正: 正転、負: 逆転）
    pub rotations: i32,
    /// 方向反転フラグ（true: 反転、false: 通常）
    inversed: bool,
    /// 前回の角度（速度計算用）
    prev_angle: f32,
}

impl ShaftPosition {
    /// 新しいShaftPositionを作成（ゼロ位置）
    pub fn new() -> Self {
        Self {
            angle: 0.0,
            rotations: 0,
            inversed: false,
            prev_angle: 0.0,
        }
    }

    /// 角度を0～TAU（0～2π）の範囲に正規化
    #[inline]
    pub fn clamp(angle: f32) -> f32 {
        let mut normalized = angle % TAU;
        if normalized < 0.0 {
            normalized += TAU;
        }
        normalized
    }

    /// 位置をリセット（ゼロ位置に戻す）
    pub fn reset(&mut self) {
        self.angle = 0.0;
        self.rotations = 0;
        self.prev_angle = 0.0;
    }

    /// 方向反転フラグを設定
    pub fn set_inversed(&mut self, inversed: bool) {
        self.inversed = inversed;
    }

    /// 方向反転フラグを取得
    #[allow(dead_code)]
    pub fn is_inversed(&self) -> bool {
        self.inversed
    }

    /// センサーから取得した角度でシャフト位置を更新
    ///
    /// # 引数
    /// * `sensor_angle` - センサーから取得した角度 [rad]
    pub fn update_shaft_angle(&mut self, mut sensor_angle: f32) {
        // 方向反転処理
        if self.inversed {
            sensor_angle = TAU - sensor_angle;
        }

        // 角度を0～TAUに正規化
        sensor_angle = Self::clamp(sensor_angle);

        // 回転数の更新（角度のジャンプを検出）
        let delta = sensor_angle - self.prev_angle;

        // 2π付近での境界を跨いだかチェック
        // 順方向: 前回が大きい値（例: 6.0）から小さい値（例: 0.2）へ
        if delta < -TAU / 2.0 {
            // 正転（0→2πの境界を越えた）
            self.rotations += 1;
        }
        // 逆方向: 前回が小さい値から大きい値へ
        else if delta > TAU / 2.0 {
            // 逆転（2π→0の境界を越えた）
            self.rotations -= 1;
        }

        self.prev_angle = sensor_angle;
        self.angle = sensor_angle;
    }

    /// 指定された角度増分だけ位置を進める
    ///
    /// # 引数
    /// * `delta_angle` - 角度増分 [rad]（正: 正転、負: 逆転）
    pub fn increment(&mut self, delta_angle: f32) {
        let mut new_angle = self.angle + delta_angle;

        // 回転数の更新
        while new_angle >= TAU {
            new_angle -= TAU;
            self.rotations += 1;
        }
        while new_angle < 0.0 {
            new_angle += TAU;
            self.rotations -= 1;
        }

        self.angle = new_angle;
        self.prev_angle = new_angle;
    }

    /// 現在の角度を取得（0～TAU）
    #[inline]
    pub fn get_angle(&self) -> f32 {
        self.angle
    }

    /// 総位置を取得（回転数 × TAU + 角度）
    ///
    /// # 戻り値
    /// 累積位置 [rad]（複数回転を含む）
    pub fn get_position(&self) -> f32 {
        self.rotations as f32 * TAU + self.angle
    }

    /// 前回の更新からの角度変化量を取得
    ///
    /// # 戻り値
    /// 角度変化量 [rad]
    #[allow(dead_code)]
    pub fn delta(&self) -> f32 {
        // 前回の位置を計算
        let prev_position = self.rotations as f32 * TAU + self.prev_angle;
        let current_position = self.get_position();
        current_position - prev_position
    }

    /// 他のShaftPositionとの位置差を計算
    ///
    /// # 引数
    /// * `other` - 比較対象のShaftPosition
    ///
    /// # 戻り値
    /// 位置差 [rad]（self - other）
    #[allow(dead_code)]
    pub fn compare(&self, other: &ShaftPosition) -> f32 {
        self.get_position() - other.get_position()
    }

    /// 2つの位置の差を-π～+πの範囲で計算（最短経路）
    ///
    /// # 引数
    /// * `other` - 比較対象のShaftPosition
    ///
    /// # 戻り値
    /// 角度差 [rad]（-π ≤ diff ≤ +π）
    #[allow(dead_code)]
    pub fn angular_distance(&self, other: &ShaftPosition) -> f32 {
        let diff = self.angle - other.angle;

        // -π～+πに正規化
        if diff > core::f32::consts::PI {
            diff - TAU
        } else if diff < -core::f32::consts::PI {
            diff + TAU
        } else {
            diff
        }
    }
}

impl Default for ShaftPosition {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clamp() {
        assert_eq!(ShaftPosition::clamp(0.0), 0.0);
        assert_eq!(ShaftPosition::clamp(TAU), 0.0);
        assert_eq!(ShaftPosition::clamp(TAU + 1.0), 1.0);
        assert!((ShaftPosition::clamp(-1.0) - (TAU - 1.0)).abs() < 1e-6);
    }

    #[test]
    fn test_update_forward() {
        let mut pos = ShaftPosition::new();

        // 0.0 → 1.0 → 2.0
        pos.update_shaft_angle(1.0);
        assert_eq!(pos.angle, 1.0);
        assert_eq!(pos.rotations, 0);

        pos.update_shaft_angle(2.0);
        assert_eq!(pos.angle, 2.0);
        assert_eq!(pos.rotations, 0);

        // 境界を越える: 6.0 → 0.5（1回転完了）
        pos.update_shaft_angle(6.0);
        pos.update_shaft_angle(0.5);
        assert_eq!(pos.rotations, 1);
    }

    #[test]
    fn test_update_backward() {
        let mut pos = ShaftPosition::new();
        pos.update_shaft_angle(1.0);

        // 逆方向: 1.0 → 0.5 → 6.0（逆転）
        pos.update_shaft_angle(0.5);
        assert_eq!(pos.rotations, 0);

        pos.update_shaft_angle(6.0);
        assert_eq!(pos.rotations, -1);
    }

    #[test]
    fn test_increment() {
        let mut pos = ShaftPosition::new();

        // 0.0 + 1.0 = 1.0
        pos.increment(1.0);
        assert_eq!(pos.angle, 1.0);
        assert_eq!(pos.rotations, 0);

        // 1.0 + 6.0 = 7.0 → 7.0 - TAU ≈ 0.717（1回転）
        pos.increment(6.0);
        assert!((pos.angle - (7.0 - TAU)).abs() < 1e-6);
        assert_eq!(pos.rotations, 1);
    }

    #[test]
    fn test_position() {
        let mut pos = ShaftPosition::new();
        pos.update_shaft_angle(1.0);
        assert!((pos.get_position() - 1.0).abs() < 1e-6);

        pos.update_shaft_angle(6.0);
        pos.update_shaft_angle(0.5);
        // 1回転 + 0.5rad
        assert!((pos.get_position() - (TAU + 0.5)).abs() < 1e-6);
    }

    #[test]
    fn test_inversed() {
        let mut pos = ShaftPosition::new();
        pos.set_inversed(true);

        // 反転モード: 1.0 → TAU - 1.0
        pos.update_shaft_angle(1.0);
        assert!((pos.angle - (TAU - 1.0)).abs() < 1e-6);
    }
}
