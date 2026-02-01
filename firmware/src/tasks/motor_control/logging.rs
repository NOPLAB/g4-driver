//! 周期的ログ出力ユーティリティ

/// 周期的なログ出力を管理
pub struct PeriodicLogger {
    counter: u32,
    interval: u32,
}

impl PeriodicLogger {
    /// 新しいロガーを作成
    ///
    /// # Arguments
    /// * `interval` - ログ出力間隔（サイクル数）
    pub const fn new(interval: u32) -> Self {
        Self {
            counter: 0,
            interval,
        }
    }

    /// 1Hz用のロガーを作成（10kHz制御ループ想定）
    pub const fn one_hz() -> Self {
        Self::new(10_000)
    }

    /// 2.5kHz制御ループで1秒ごと（キャリブレーション用）
    pub const fn every_2500_cycles() -> Self {
        Self::new(2_500)
    }

    /// カウンタを進め、ログを出すべきかを返す
    pub fn tick(&mut self) -> bool {
        self.counter += 1;
        if self.counter >= self.interval {
            self.counter = 0;
            true
        } else {
            false
        }
    }

    /// カウンタをリセット
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.counter = 0;
    }
}
