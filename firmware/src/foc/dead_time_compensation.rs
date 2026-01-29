//! デッドタイム補償
//!
//! PWMスイッチングのデッドタイムによる電圧歪みを補償します。
//! 電流センシングがないため、電気角とVq指令の符号から各相の電流方向を推定し、
//! Duty比を補正します。

use libm::sinf;

/// デッドタイム補償器
pub struct DeadTimeCompensation {
    /// 補償有効/無効
    enabled: bool,
    /// 補償量（Duty換算、事前計算）
    compensation_duty: u16,
}

impl DeadTimeCompensation {
    /// 新規作成
    ///
    /// # 引数
    /// * `dead_time_ns` - デッドタイム [ns]
    /// * `pwm_freq_hz` - PWM周波数 [Hz]
    /// * `v_dc` - DCバス電圧 [V]
    /// * `max_duty` - 最大Duty値
    pub fn new(dead_time_ns: f32, pwm_freq_hz: u32, _v_dc: f32, max_duty: u16) -> Self {
        // デッドタイムによる電圧損失をDuty比に換算
        // Vdrop = Vdc * Td * Fpwm * 2 (上下アーム両方のデッドタイム)
        // duty_compensation = Vdrop / Vdc * max_duty = Td * Fpwm * 2 * max_duty
        let dead_time_s = dead_time_ns * 1e-9;
        let compensation_ratio = dead_time_s * pwm_freq_hz as f32 * 2.0;
        let compensation_duty = (compensation_ratio * max_duty as f32) as u16;

        Self {
            enabled: false,
            compensation_duty,
        }
    }

    /// 補償の有効/無効を設定
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// 補償が有効かどうかを返す
    #[allow(dead_code)]
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// デッドタイム補償を適用
    ///
    /// # 引数
    /// * `duty_u`, `duty_v`, `duty_w` - 補償前のDuty値
    /// * `vq` - q軸電圧指令（電流方向推定用）
    /// * `theta` - 電気角 [rad]
    /// * `max_duty` - 最大Duty値
    ///
    /// # 戻り値
    /// 補償後のDuty値 (duty_u, duty_v, duty_w)
    pub fn compensate(
        &self,
        duty_u: u16,
        duty_v: u16,
        duty_w: u16,
        vq: f32,
        theta: f32,
        max_duty: u16,
    ) -> (u16, u16, u16) {
        if !self.enabled || self.compensation_duty == 0 {
            return (duty_u, duty_v, duty_w);
        }

        // 電流方向推定（Vqの符号と電気角から各相の電流方向を推定）
        // Vq > 0 の場合、q軸電流は正方向 → 各相電流は sin(theta), sin(theta - 2π/3), sin(theta + 2π/3)
        // Vq < 0 の場合、符号反転
        let vq_sign = if vq >= 0.0 { 1.0 } else { -1.0 };

        // 各相の電流方向を推定（sin値の符号で判定）
        let phase_offset = core::f32::consts::FRAC_PI_3 * 2.0; // 2π/3
        let i_u_sign = sinf(theta) * vq_sign;
        let i_v_sign = sinf(theta - phase_offset) * vq_sign;
        let i_w_sign = sinf(theta + phase_offset) * vq_sign;

        // 電流方向に基づいて補償を適用
        // 電流が正（ハイサイドからローサイドへ）: デッドタイムで電圧低下 → Duty増加
        // 電流が負（ローサイドからハイサイドへ）: デッドタイムで電圧上昇 → Duty減少
        let comp = self.compensation_duty as i32;

        let new_duty_u = if i_u_sign > 0.0 {
            (duty_u as i32 + comp).min(max_duty as i32) as u16
        } else if i_u_sign < 0.0 {
            (duty_u as i32 - comp).max(0) as u16
        } else {
            duty_u
        };

        let new_duty_v = if i_v_sign > 0.0 {
            (duty_v as i32 + comp).min(max_duty as i32) as u16
        } else if i_v_sign < 0.0 {
            (duty_v as i32 - comp).max(0) as u16
        } else {
            duty_v
        };

        let new_duty_w = if i_w_sign > 0.0 {
            (duty_w as i32 + comp).min(max_duty as i32) as u16
        } else if i_w_sign < 0.0 {
            (duty_w as i32 - comp).max(0) as u16
        } else {
            duty_w
        };

        (new_duty_u, new_duty_v, new_duty_w)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compensation_disabled() {
        let comp = DeadTimeCompensation::new(100.0, 50000, 24.0, 1000);
        // 無効時は入力をそのまま返す
        let (u, v, w) = comp.compensate(500, 500, 500, 10.0, 0.0, 1000);
        assert_eq!((u, v, w), (500, 500, 500));
    }

    #[test]
    fn test_compensation_enabled() {
        let mut comp = DeadTimeCompensation::new(100.0, 50000, 24.0, 1000);
        comp.set_enabled(true);

        // theta = 0, Vq > 0 の場合
        // i_u: sin(0) = 0 → 補償なし
        // i_v: sin(-2π/3) < 0 → Duty減少
        // i_w: sin(+2π/3) > 0 → Duty増加
        let (u, v, w) = comp.compensate(500, 500, 500, 10.0, 0.0, 1000);

        // U相はsin(0)≈0なので変化なし
        assert_eq!(u, 500);
        // V相は減少
        assert!(v < 500);
        // W相は増加
        assert!(w > 500);
    }

    #[test]
    fn test_compensation_clamp() {
        let mut comp = DeadTimeCompensation::new(1000.0, 50000, 24.0, 100);
        comp.set_enabled(true);

        // 極端な補償値でもクランプされる
        let (u, v, w) = comp.compensate(95, 5, 50, 10.0, 0.5, 100);
        assert!(u <= 100);
        assert!(v <= 100);
        assert!(w <= 100);
    }
}
