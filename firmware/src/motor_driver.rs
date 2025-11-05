//! モータードライバー抽象化レイヤー
//!
//! PWMハードウェアへの直接アクセスを隠蔽し、
//! モーター制御に必要な高レベルインターフェースを提供します。

use embassy_stm32::{
    peripherals,
    timer::{complementary_pwm::ComplementaryPwm, Channel},
};

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
    pub fn max_duty(&self) -> u16 {
        self.max_duty
    }

    /// 3相全てのDuty比を設定
    ///
    /// # 引数
    /// * `duty_u` - U相のDuty比
    /// * `duty_v` - V相のDuty比
    /// * `duty_w` - W相のDuty比
    pub fn set_duty_uvw(&mut self, duty_u: u16, duty_v: u16, duty_w: u16) {
        self.pwm.set_duty(Channel::Ch1, duty_u);
        self.pwm.set_duty(Channel::Ch2, duty_v);
        self.pwm.set_duty(Channel::Ch3, duty_w);
    }

    /// 全チャネルを有効化
    pub fn enable_all_channels(&mut self) {
        self.pwm.enable(Channel::Ch1);
        self.pwm.enable(Channel::Ch2);
        self.pwm.enable(Channel::Ch3);
    }

    /// 全チャネルを無効化
    pub fn disable_all_channels(&mut self) {
        self.pwm.disable(Channel::Ch1);
        self.pwm.disable(Channel::Ch2);
        self.pwm.disable(Channel::Ch3);
    }

    /// 全チャネルのDuty比を0にして停止
    pub fn stop(&mut self) {
        self.set_duty_uvw(0, 0, 0);
        self.disable_all_channels();
    }

    /// 各チャネルを個別に有効/無効化
    ///
    /// # 引数
    /// * `enable_u` - U相を有効にするか
    /// * `enable_v` - V相を有効にするか
    /// * `enable_w` - W相を有効にするか
    pub fn set_channels(&mut self, enable_u: bool, enable_v: bool, enable_w: bool) {
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
