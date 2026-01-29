//! フラックス弱め制御
//!
//! 高速域でd軸に負の電圧を印加することで、逆起電力を抑制し
//! モーターの最高回転数を向上させます。
//!
//! 電流センシングがないため、速度ベースのフィードフォワード方式で実装。

use libm::sqrtf;

/// フラックス弱め制御器
pub struct FluxWeakeningController {
    /// 制御有効/無効
    enabled: bool,
    /// 弱め制御開始速度 [RPM]
    min_speed: f32,
    /// 最大弱め速度 [RPM]
    max_speed: f32,
    /// 最大弱め率 (0.0-1.0)
    max_weakening_ratio: f32,
    /// Vdレート制限 [V/s]
    vd_rate_limit: f32,
    /// 現在のVd値 [V]（レート制限用）
    current_vd: f32,
}

impl FluxWeakeningController {
    /// 新規作成
    ///
    /// # 引数
    /// * `min_speed` - 弱め制御開始速度 [RPM]
    /// * `max_speed` - 最大弱め速度 [RPM]
    /// * `max_weakening_ratio` - 最大弱め率 (0.0-1.0)
    /// * `vd_rate_limit` - Vdレート制限 [V/s]
    pub fn new(
        min_speed: f32,
        max_speed: f32,
        max_weakening_ratio: f32,
        vd_rate_limit: f32,
    ) -> Self {
        Self {
            enabled: false,
            min_speed,
            max_speed,
            max_weakening_ratio: max_weakening_ratio.clamp(0.0, 1.0),
            vd_rate_limit,
            current_vd: 0.0,
        }
    }

    /// 制御の有効/無効を設定
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// 制御が有効かどうかを返す
    #[allow(dead_code)]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// 内部状態をリセット
    pub fn reset(&mut self) {
        self.current_vd = 0.0;
    }

    /// 速度に応じたd軸電圧を計算
    ///
    /// # 引数
    /// * `speed_rpm` - 現在の速度 [RPM]
    /// * `vq` - q軸電圧指令 [V]
    /// * `v_dc` - DCバス電圧 [V]
    /// * `dt` - 制御周期 [s]
    ///
    /// # 戻り値
    /// d軸電圧指令 [V]（負の値）
    pub fn calculate_vd(&mut self, speed_rpm: f32, vq: f32, v_dc: f32, dt: f32) -> f32 {
        if !self.enabled {
            self.current_vd = 0.0;
            return 0.0;
        }

        let speed_abs = speed_rpm.abs();

        // 弱め制御開始速度以下では弱めなし
        if speed_abs < self.min_speed {
            // レート制限を適用して0に戻す
            return self.apply_rate_limit(0.0, dt);
        }

        // 速度に応じた弱め率を計算（線形補間）
        let speed_range = self.max_speed - self.min_speed;
        let weakening_ratio = if speed_range > 0.0 {
            let ratio = (speed_abs - self.min_speed) / speed_range;
            (ratio * self.max_weakening_ratio).clamp(0.0, self.max_weakening_ratio)
        } else {
            0.0
        };

        // 利用可能なd軸電圧の計算
        // |Vd|^2 + |Vq|^2 <= Vdc^2 より、Vd_max = sqrt(Vdc^2 - Vq^2)
        let vq_abs = vq.abs();
        let vd_available = if vq_abs < v_dc {
            sqrtf(v_dc * v_dc - vq_abs * vq_abs)
        } else {
            0.0
        };

        // 目標Vd（負の値、弱め率に応じて）
        let target_vd = -vd_available * weakening_ratio;

        // レート制限を適用
        self.apply_rate_limit(target_vd, dt)
    }

    /// レート制限を適用してVdを更新
    fn apply_rate_limit(&mut self, target_vd: f32, dt: f32) -> f32 {
        let max_delta = self.vd_rate_limit * dt;
        let delta = target_vd - self.current_vd;

        if delta.abs() > max_delta {
            if delta > 0.0 {
                self.current_vd += max_delta;
            } else {
                self.current_vd -= max_delta;
            }
        } else {
            self.current_vd = target_vd;
        }

        self.current_vd
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_disabled() {
        let mut fw = FluxWeakeningController::new(2000.0, 4000.0, 0.5, 100.0);
        // 無効時は常に0
        let vd = fw.calculate_vd(3000.0, 10.0, 24.0, 0.0001);
        assert_eq!(vd, 0.0);
    }

    #[test]
    fn test_below_min_speed() {
        let mut fw = FluxWeakeningController::new(2000.0, 4000.0, 0.5, 100.0);
        fw.set_enabled(true);

        // 弱め開始速度以下ではVd = 0
        let vd = fw.calculate_vd(1000.0, 10.0, 24.0, 0.001);
        assert_eq!(vd, 0.0);
    }

    #[test]
    fn test_weakening_at_mid_speed() {
        let mut fw = FluxWeakeningController::new(2000.0, 4000.0, 0.5, 1000.0);
        fw.set_enabled(true);

        // 中間速度（3000 RPM）では弱め率 = 0.5 * (3000-2000)/(4000-2000) = 0.25
        // 十分な時間をかけて目標に到達させる
        for _ in 0..100 {
            fw.calculate_vd(3000.0, 10.0, 24.0, 0.01);
        }
        let vd = fw.calculate_vd(3000.0, 10.0, 24.0, 0.01);

        // Vdは負の値
        assert!(vd < 0.0);
    }

    #[test]
    fn test_rate_limit() {
        let mut fw = FluxWeakeningController::new(2000.0, 4000.0, 0.5, 10.0);
        fw.set_enabled(true);

        // レート制限10V/s、dt=0.1sで最大1Vしか変化しない
        let vd1 = fw.calculate_vd(4000.0, 10.0, 24.0, 0.1);
        assert!(vd1 >= -1.1 && vd1 <= 0.0);

        // 2回目も同様にレート制限
        let vd2 = fw.calculate_vd(4000.0, 10.0, 24.0, 0.1);
        assert!(vd2 >= -2.1 && vd2 <= 0.0);
        assert!(vd2 < vd1); // より負方向へ
    }
}
