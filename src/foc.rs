// FOC (Field Oriented Control) module
// Hall sensor-based FOC implementation for BLDC motor control

pub mod hall_sensor;
pub mod pi_controller;
pub mod svpwm;
pub mod transforms;

// Re-export main types for easier access
pub use hall_sensor::HallSensor;
pub use pi_controller::PiController;
pub use svpwm::calculate_svpwm;
pub use transforms::{inverse_park, limit_voltage};

/// モーター制御モード
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlMode {
    /// オープンループ強制転流（始動時）
    OpenLoop,
    /// クローズドループFOC制御（通常運転）
    ClosedLoopFoc,
}

/// オープンループランプアップ制御
/// 始動時に強制転流でモーターを回転させ、しきい値速度に達したらFOC制御に移行
pub struct OpenLoopRampUp {
    /// 現在の電気角 [rad]
    electrical_angle: f32,
    /// 角速度 [rad/s]
    angular_velocity: f32,
    /// 初期角速度 [rad/s]
    initial_velocity: f32,
    /// 角加速度 [rad/s²]
    acceleration: f32,
    /// 目標角速度 [rad/s]
    target_velocity: f32,
    /// 出力電圧 [V]
    output_voltage: f32,
}

impl OpenLoopRampUp {
    /// 新しいオープンループランプアップ制御を作成
    ///
    /// # 引数
    /// * `initial_rpm` - 初期回転数 [RPM]
    /// * `target_rpm` - 目標回転数 [RPM]（この速度に達したらFOCに切り替え）
    /// * `acceleration_rpm_per_s` - 加速度 [RPM/s]
    /// * `output_voltage` - 出力電圧 [V]
    /// * `pole_pairs` - モーターの極対数
    pub fn new(
        initial_rpm: f32,
        target_rpm: f32,
        acceleration_rpm_per_s: f32,
        output_voltage: f32,
        pole_pairs: u8,
    ) -> Self {
        let pp = pole_pairs as f32;
        Self {
            electrical_angle: 0.0,
            angular_velocity: Self::rpm_to_rad_per_s(initial_rpm, pp),
            initial_velocity: Self::rpm_to_rad_per_s(initial_rpm, pp),
            acceleration: Self::rpm_to_rad_per_s(acceleration_rpm_per_s, pp),
            target_velocity: Self::rpm_to_rad_per_s(target_rpm, pp),
            output_voltage,
        }
    }

    /// RPMを電気角速度 [rad/s] に変換
    fn rpm_to_rad_per_s(rpm: f32, pole_pairs: f32) -> f32 {
        rpm * 2.0 * core::f32::consts::PI * pole_pairs / 60.0
    }

    /// 電気角速度 [rad/s] をRPMに変換
    fn rad_per_s_to_rpm(rad_per_s: f32, pole_pairs: f32) -> f32 {
        rad_per_s * 60.0 / (2.0 * core::f32::consts::PI * pole_pairs)
    }

    /// オープンループ制御を1ステップ更新
    ///
    /// # 引数
    /// * `dt` - 制御周期 [s]
    ///
    /// # 戻り値
    /// * `(electrical_angle, vq)` - 電気角 [rad] とq軸電圧 [V]
    pub fn update(&mut self, dt: f32) -> (f32, f32) {
        // 角速度を加速
        if self.angular_velocity < self.target_velocity {
            self.angular_velocity += self.acceleration * dt;
            if self.angular_velocity > self.target_velocity {
                self.angular_velocity = self.target_velocity;
            }
        }

        // 電気角を更新
        self.electrical_angle += self.angular_velocity * dt;

        // 電気角を0～2πの範囲に正規化
        while self.electrical_angle >= 2.0 * core::f32::consts::PI {
            self.electrical_angle -= 2.0 * core::f32::consts::PI;
        }

        (self.electrical_angle, self.output_voltage)
    }

    /// 目標速度に達したかチェック
    pub fn is_target_reached(&self) -> bool {
        self.angular_velocity >= self.target_velocity
    }

    /// リセット
    pub fn reset(&mut self) {
        self.electrical_angle = 0.0;
        self.angular_velocity = self.initial_velocity;
    }

    /// 現在の速度を取得 [RPM]
    pub fn get_current_rpm(&self, pole_pairs: u8) -> f32 {
        Self::rad_per_s_to_rpm(self.angular_velocity, pole_pairs as f32)
    }
}
