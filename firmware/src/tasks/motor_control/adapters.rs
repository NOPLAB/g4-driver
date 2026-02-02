//! bldcクレートのtrait実装アダプター
//!
//! firmwareのハードウェアをbldc状態機械に接続するためのアダプター。

use core::f32::consts::PI;

use bldc::traits::{
    ControlInput, ControlMode, HallStateReader, PositionSensor, PwmOutput, SpeedSensor,
    StatusOutput,
};

use crate::adapters::HallSensorAdapter;
use crate::board::{self, MotorDriver};
use crate::config::{hall, motor, speed};
use crate::state::{self, RUNTIME};

/// ファームウェアハードウェアアダプター
///
/// HallSensorAdapterとMotorDriverをbldc traitで提供
pub struct FirmwareHardware {
    /// Hallセンサーアダプター
    pub hall_sensor: HallSensorAdapter,
    /// モータードライバー
    pub motor_driver: MotorDriver,
    /// 最大duty値
    pub max_duty: u16,
}

impl FirmwareHardware {
    /// 新しいFirmwareHardwareを作成
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

    /// Hallセンサーを更新して電気角と速度を取得
    pub fn update_hall(&mut self, dt: f32) -> (f32, f32, u8) {
        self.hall_sensor.update(dt)
    }

    /// モーター停止
    pub fn stop(&mut self) {
        self.motor_driver.stop();
        self.hall_sensor.reset();
        board::reset_state();
    }

    /// 速度フィルターをリセット
    pub fn reset_speed_filter(&mut self, initial_rpm: f32) {
        self.hall_sensor.reset_speed_filter(initial_rpm);
    }
}

impl HallStateReader for FirmwareHardware {
    fn get_hall_state(&self) -> u8 {
        board::get_hall_state()
    }
}

impl PositionSensor for FirmwareHardware {
    fn electrical_angle(&self) -> f32 {
        self.hall_sensor.get_electrical_angle()
    }

    fn mechanical_angle(&self) -> f32 {
        self.hall_sensor.get_mechanical_angle()
    }
}

impl SpeedSensor for FirmwareHardware {
    fn speed_rad_s(&self) -> f32 {
        self.hall_sensor.get_speed_rpm() * core::f32::consts::TAU / 60.0
    }

    fn speed_rpm(&self) -> f32 {
        self.hall_sensor.get_speed_rpm()
    }
}

impl PwmOutput for FirmwareHardware {
    fn set_duty(&mut self, u: f32, v: f32, w: f32) {
        let max = self.max_duty as f32;
        let du = (u * max).clamp(0.0, max) as u16;
        let dv = (v * max).clamp(0.0, max) as u16;
        let dw = (w * max).clamp(0.0, max) as u16;
        self.motor_driver.set_duty_uvw(du, dv, dw);
    }

    fn enable(&mut self) {
        self.motor_driver.enable_all_channels();
    }

    fn disable(&mut self) {
        self.motor_driver.stop();
    }
}

/// 制御入力アダプター
///
/// 非同期状態アクセスの結果をキャッシュして同期的に提供
#[derive(Debug, Clone, Default)]
pub struct FirmwareControlInput {
    /// 目標速度 [RPM]
    pub target_speed: f32,
    /// PIゲイン (Kp, Ki)
    pub pi_gains: (f32, f32),
    /// キャリブレーションリクエスト
    pub calibration_requested: bool,
    /// キャリブレーショントルク
    pub calibration_torque: f32,
    /// モーター有効フラグ
    pub motor_enabled: bool,
}

impl FirmwareControlInput {
    /// 非同期でグローバル状態から入力パラメータを取得
    pub async fn fetch_from_state(&mut self) {
        // 制御パラメータを取得
        let foc_params = state::get_foc_input_params().await;
        self.target_speed = foc_params.target_speed;
        self.pi_gains = foc_params.pi_gains;

        // モーター有効フラグを取得
        self.motor_enabled = state::motor_context().await.enabled;

        // キャリブレーションリクエストを確認（消費）
        let mut calib_ctx = state::calibration_context().await;
        if calib_ctx.request {
            self.calibration_requested = true;
            self.calibration_torque = calib_ctx.torque as f32 / 100.0;
            calib_ctx.request = false;
        } else {
            self.calibration_requested = false;
        }
    }

    /// キャリブレーションリクエストをクリア
    pub fn clear_calibration_request(&mut self) {
        self.calibration_requested = false;
    }
}

impl ControlInput for FirmwareControlInput {
    fn target_speed(&self) -> f32 {
        self.target_speed
    }

    fn pi_gains(&self) -> (f32, f32) {
        self.pi_gains
    }

    fn calibration_requested(&self) -> bool {
        self.calibration_requested
    }

    fn calibration_torque(&self) -> f32 {
        self.calibration_torque
    }

    fn motor_enabled(&self) -> bool {
        self.motor_enabled
    }
}

/// ステータス出力アダプター
///
/// RUNTIME atomic変数を更新
#[derive(Default)]
pub struct FirmwareStatusOutput {
    /// 最後のモード（変更検出用）
    last_mode: Option<ControlMode>,
}

impl StatusOutput for FirmwareStatusOutput {
    fn update_status(&mut self, speed_rpm: f32, electrical_angle: f32) {
        RUNTIME.status.update(speed_rpm, electrical_angle);
    }

    fn on_mode_change(&mut self, mode: ControlMode) {
        self.last_mode = Some(mode);
    }

    fn on_stall_detected(&mut self) {
        // ログはmotor_control_taskで出力
    }

    fn on_calibration_complete(&mut self, success: bool, offset: f32, inversed: bool) {
        let _ = (success, offset, inversed);
        // キャリブレーション結果の保存はmotor_control_taskで行う
    }
}

impl FirmwareStatusOutput {
    /// 最後のモードを取得
    #[allow(dead_code)]
    pub fn last_mode(&self) -> Option<ControlMode> {
        self.last_mode
    }
}

/// bldcのControlModeからfirmwareのControlModeへ変換
pub fn to_firmware_control_mode(mode: ControlMode) -> state::ControlMode {
    match mode {
        ControlMode::OpenLoop => state::ControlMode::OpenLoop,
        ControlMode::Foc => state::ControlMode::ClosedLoopFoc,
        ControlMode::Calibration => state::ControlMode::Calibration,
    }
}
