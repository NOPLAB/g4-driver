//! TIM1ベースの3相PWMドライバー実装
//!
//! STM32G431VBTのTIM1を使用して3相補完PWMを出力します。
//!
//! ## ピン配置
//! - PE9/PE8: U相 (CH1/CH1N)
//! - PE11/PE10: V相 (CH2/CH2N)
//! - PE13/PE12: W相 (CH3/CH3N)

use embassy_stm32::{
    peripherals,
    timer::{complementary_pwm::ComplementaryPwm, Channel},
};

use crate::motor_driver::traits::{BootstrapChargeable, PwmDriver};

/// 3相モータードライバー
///
/// STM32のComplementaryPwmを使用して3相ブラシレスモーターを駆動します。
pub struct MotorDriver {
    pwm: ComplementaryPwm<'static, peripherals::TIM1>,
    max_duty: u16,
}

impl MotorDriver {
    /// 新しいモータードライバーを作成
    ///
    /// # 引数
    /// * `pwm` - PWMペリフェラル（TIM1）
    pub fn new(pwm: ComplementaryPwm<'static, peripherals::TIM1>) -> Self {
        let max_duty = pwm.get_max_duty();
        Self { pwm, max_duty }
    }

    /// PWMの最大Duty値を取得
    #[inline(always)]
    pub fn max_duty(&self) -> u16 {
        self.max_duty
    }

    /// 3相全てのDuty比を設定
    #[inline(always)]
    pub fn set_duty_uvw(&mut self, duty_u: u16, duty_v: u16, duty_w: u16) {
        <Self as PwmDriver>::set_duty_uvw(self, duty_u, duty_v, duty_w)
    }

    /// 全チャネルを有効化
    #[inline(always)]
    pub fn enable_all_channels(&mut self) {
        <Self as PwmDriver>::enable_all_channels(self)
    }

    /// 全チャネルを無効化
    #[inline(always)]
    pub fn disable_all_channels(&mut self) {
        <Self as PwmDriver>::disable_all_channels(self)
    }

    /// 全チャネルのDuty比を0にして停止
    #[inline(always)]
    pub fn stop(&mut self) {
        <Self as PwmDriver>::stop(self)
    }

    /// 各チャネルを個別に有効/無効化
    #[inline(always)]
    pub fn set_channels(&mut self, enable_u: bool, enable_v: bool, enable_w: bool) {
        <Self as PwmDriver>::set_channels(self, enable_u, enable_v, enable_w)
    }
}

impl PwmDriver for MotorDriver {
    #[inline(always)]
    fn max_duty(&self) -> u16 {
        self.max_duty
    }

    #[inline(always)]
    fn set_duty_uvw(&mut self, duty_u: u16, duty_v: u16, duty_w: u16) {
        self.pwm.set_duty(Channel::Ch1, duty_u);
        self.pwm.set_duty(Channel::Ch2, duty_v);
        self.pwm.set_duty(Channel::Ch3, duty_w);
    }

    #[inline(always)]
    fn enable_all_channels(&mut self) {
        self.pwm.enable(Channel::Ch1);
        self.pwm.enable(Channel::Ch2);
        self.pwm.enable(Channel::Ch3);
    }

    #[inline(always)]
    fn disable_all_channels(&mut self) {
        self.pwm.disable(Channel::Ch1);
        self.pwm.disable(Channel::Ch2);
        self.pwm.disable(Channel::Ch3);
    }

    #[inline(always)]
    fn stop(&mut self) {
        self.set_duty_uvw(0, 0, 0);
        self.disable_all_channels();
    }

    #[inline(always)]
    fn set_channels(&mut self, enable_u: bool, enable_v: bool, enable_w: bool) {
        if enable_u {
            self.pwm.enable(Channel::Ch1);
        } else {
            self.pwm.disable(Channel::Ch1);
        }

        if enable_v {
            self.pwm.enable(Channel::Ch2);
        } else {
            self.pwm.disable(Channel::Ch2);
        }

        if enable_w {
            self.pwm.enable(Channel::Ch3);
        } else {
            self.pwm.disable(Channel::Ch3);
        }
    }
}

impl BootstrapChargeable for MotorDriver {
    /// 全ローサイドをON、ハイサイドをOFF
    ///
    /// 補完PWMでDuty=max_dutyに設定すると：
    /// - ハイサイド(INHx): OFF
    /// - ローサイド(INLx): ON
    ///
    /// これによりブートストラップコンデンサが充電される
    fn set_all_low_side_on(&mut self) {
        self.set_duty_uvw(self.max_duty, self.max_duty, self.max_duty);
        self.enable_all_channels();
    }

    /// 全チャネルをOFF
    fn set_all_off(&mut self) {
        self.set_duty_uvw(0, 0, 0);
        self.disable_all_channels();
    }
}
