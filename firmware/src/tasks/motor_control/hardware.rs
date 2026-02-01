//! ハードウェアリソース管理
//!
//! モード間で共有されるハードウェアリソースを管理します。

use core::f32::consts::PI;

use crate::adapters::HallSensorAdapter;
use crate::board::{self, MotorDriver};
use crate::config::{hall, motor, speed};

/// ハードウェアリソース（モード間で共有）
pub struct Hardware {
    /// Hallセンサー
    pub hall_sensor: HallSensorAdapter,
    /// モータードライバー
    pub motor_driver: MotorDriver,
    /// 最大duty値（キャッシュ）
    pub max_duty: u16,
}

impl Hardware {
    /// 新しいHardwareを作成
    pub fn new(motor_driver: MotorDriver) -> Self {
        let max_duty = motor_driver.max_duty();

        // Hallセンサー初期化
        let mut hall_sensor =
            HallSensorAdapter::new(motor::DEFAULT_POLE_PAIRS, speed::DEFAULT_FILTER_ALPHA);
        hall_sensor.set_interpolation(false); // 角度補間を無効化（ノイズ対策）

        // デフォルトの電気オフセット
        let offset_rad = hall::DEFAULT_ANGLE_OFFSET_DEG * PI / 180.0;
        hall_sensor.set_electrical_offset(offset_rad);

        Self {
            hall_sensor,
            motor_driver,
            max_duty,
        }
    }

    /// モーター停止
    pub fn stop(&mut self) {
        self.motor_driver.stop();
        self.hall_sensor.reset();
        board::reset_state();
    }
}
