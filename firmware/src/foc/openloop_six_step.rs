//! オープンループ6ステップ駆動制御
//!
//! 始動時に6ステップ駆動（台形波）でモーターを回転させるための制御モジュールです。

/// 6ステップ駆動の各ステップ情報
#[derive(Debug, Clone, Copy)]
pub struct SixStepState {
    /// 現在のステップ (0-5)
    #[allow(dead_code)]
    pub step: u8,
    /// U相のデューティ比 (0-100)
    pub duty_u: u16,
    /// V相のデューティ比 (0-100)
    pub duty_v: u16,
    /// W相のデューティ比 (0-100)
    pub duty_w: u16,
    /// U相を有効化するか
    pub enable_u: bool,
    /// V相を有効化するか
    pub enable_v: bool,
    /// W相を有効化するか
    pub enable_w: bool,
}

/// オープンループ6ステップ駆動制御
/// 始動時に6ステップ駆動（台形波）でモーターを回転させる
pub struct OpenLoopSixStep {
    /// 現在のステップ (0-5)
    current_step: u8,
    /// ステップ切替周期 [s]
    step_period: f32,
    /// 初期ステップ周期 [s]
    initial_step_period: f32,
    /// ステップ周期の減少率（加速率）
    acceleration_rate: f32,
    /// 最小ステップ周期 [s]
    min_step_period: f32,
    /// 前回のステップ更新からの経過時間 [s]
    elapsed_time: f32,
    /// PWMデューティ比 (0-100)
    duty_ratio: u16,
    /// 極対数
    pole_pairs: u8,
}

impl OpenLoopSixStep {
    /// 新しいオープンループ6ステップ制御を作成
    ///
    /// # 引数
    /// * `initial_rpm` - 初期回転数 [RPM]
    /// * `target_rpm` - 目標回転数 [RPM]（この速度に達したらFOCに切り替え）
    /// * `acceleration_rpm_per_s` - 加速度 [RPM/s]
    /// * `duty_ratio` - PWMデューティ比 (0-100)
    /// * `pole_pairs` - モーターの極対数
    pub fn new(
        initial_rpm: f32,
        target_rpm: f32,
        acceleration_rpm_per_s: f32,
        duty_ratio: u16,
        pole_pairs: u8,
    ) -> Self {
        // RPMからステップ周期を計算
        // 1回転 = 6ステップ × 極対数
        let steps_per_rotation = 6.0 * pole_pairs as f32;
        let initial_step_period = 60.0 / (initial_rpm * steps_per_rotation);
        let min_step_period = 60.0 / (target_rpm * steps_per_rotation);

        // 加速率を計算
        // 各ステップでどれだけ周期を短くするか
        let acceleration_rate = if acceleration_rpm_per_s > 0.0 {
            1.0 - (acceleration_rpm_per_s * initial_step_period / initial_rpm)
        } else {
            0.98 // デフォルト
        };

        Self {
            current_step: 0,
            step_period: initial_step_period,
            initial_step_period,
            acceleration_rate,
            min_step_period,
            elapsed_time: 0.0,
            duty_ratio,
            pole_pairs,
        }
    }

    /// 6ステップ駆動のステップ状態を取得
    fn get_step_state(step: u8, duty: u16) -> SixStepState {
        match step % 6 {
            // Step 1: U-High (PWM), V-Low (0), W-Open (Off)
            0 => SixStepState {
                step,
                duty_u: duty,
                duty_v: 0,
                duty_w: 0,
                enable_u: true,
                enable_v: true,
                enable_w: false,
            },
            // Step 2: U-High (PWM), W-Low (0), V-Open (Off)
            1 => SixStepState {
                step,
                duty_u: duty,
                duty_v: 0,
                duty_w: 0,
                enable_u: true,
                enable_v: false,
                enable_w: true,
            },
            // Step 3: V-High (PWM), W-Low (0), U-Open (Off)
            2 => SixStepState {
                step,
                duty_u: 0,
                duty_v: duty,
                duty_w: 0,
                enable_u: false,
                enable_v: true,
                enable_w: true,
            },
            // Step 4: V-High (PWM), U-Low (0), W-Open (Off)
            3 => SixStepState {
                step,
                duty_u: 0,
                duty_v: duty,
                duty_w: 0,
                enable_u: true,
                enable_v: true,
                enable_w: false,
            },
            // Step 5: W-High (PWM), U-Low (0), V-Open (Off)
            4 => SixStepState {
                step,
                duty_u: 0,
                duty_v: 0,
                duty_w: duty,
                enable_u: true,
                enable_v: false,
                enable_w: true,
            },
            // Step 6: W-High (PWM), V-Low (0), U-Open (Off)
            5 => SixStepState {
                step,
                duty_u: 0,
                duty_v: 0,
                duty_w: duty,
                enable_u: false,
                enable_v: true,
                enable_w: true,
            },
            _ => unreachable!(),
        }
    }

    /// 6ステップ駆動を更新
    ///
    /// # 引数
    /// * `dt` - 制御周期 [s]
    ///
    /// # 戻り値
    /// * `SixStepState` - 現在のステップ状態
    pub fn update(&mut self, dt: f32) -> SixStepState {
        self.elapsed_time += dt;

        // ステップ切替時間に達したか
        if self.elapsed_time >= self.step_period {
            self.elapsed_time = 0.0;
            self.current_step = (self.current_step + 1) % 6;

            // 加速（ステップ周期を短縮）
            if self.step_period > self.min_step_period {
                self.step_period *= self.acceleration_rate;
                if self.step_period < self.min_step_period {
                    self.step_period = self.min_step_period;
                }
            }
        }

        Self::get_step_state(self.current_step, self.duty_ratio)
    }

    /// 目標速度に達したかチェック
    pub fn is_target_reached(&self) -> bool {
        self.step_period <= self.min_step_period
    }

    /// リセット
    pub fn reset(&mut self) {
        self.current_step = 0;
        self.step_period = self.initial_step_period;
        self.elapsed_time = 0.0;
    }

    /// 現在の速度を取得 [RPM]
    pub fn get_current_rpm(&self) -> f32 {
        let steps_per_rotation = 6.0 * self.pole_pairs as f32;
        60.0 / (self.step_period * steps_per_rotation)
    }

    /// 現在のステップを取得
    #[allow(dead_code)]
    pub fn get_current_step(&self) -> u8 {
        self.current_step
    }
}
