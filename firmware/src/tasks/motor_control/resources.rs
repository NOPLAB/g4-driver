//! モーター制御リソース管理
//!
//! 全ての制御リソースを一元管理し、モード切り替え時のリセット処理を統一します。

use bldc::calibration::MotorCalibration;
use bldc::compensation::{DeadTimeCompensation, FluxWeakeningController};
use bldc::control::six_step::SixStepController;
use bldc::control::PiController;
use core::f32::consts::PI;

use crate::adapters::HallSensorAdapter;
use crate::config::*;
use crate::hall_tim;

use super::foc_mode;
use super::openloop_mode;

/// モーター制御に必要な全リソース
pub struct ControllerResources {
    /// Hallセンサー
    pub hall_sensor: HallSensorAdapter,
    /// 速度PIコントローラ
    pub speed_pi: PiController,
    /// オープンループ始動コントローラ
    pub openloop: SixStepController,
    /// キャリブレーションコントローラ
    pub calibration: MotorCalibration,
    /// デッドタイム補償器
    pub dead_time_comp: DeadTimeCompensation,
    /// フラックス弱め制御器
    pub flux_weakening: FluxWeakeningController,
    /// 速度ランプ（加速度制限）用の現在指令速度
    pub ramped_target_speed: f32,
}

impl ControllerResources {
    /// 新しいリソースセットを作成
    pub fn new(max_duty: u16) -> Self {
        // ホールセンサ初期化（foc-simple互換の機械角ベース計算）
        let mut hall_sensor =
            HallSensorAdapter::new(DEFAULT_POLE_PAIRS, DEFAULT_SPEED_FILTER_ALPHA);
        hall_sensor.set_interpolation(false); // 角度補間を無効化（ノイズ対策）

        // 電気オフセットを設定（キャリブレーション値）
        let offset_rad = DEFAULT_HALL_ANGLE_OFFSET_DEG * PI / 180.0;
        hall_sensor.set_electrical_offset(offset_rad);

        // 速度PIコントローラ初期化（アンチワインドアップ有効）
        let mut speed_pi =
            PiController::new_symmetric(DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, DEFAULT_MAX_VOLTAGE);
        speed_pi.set_anti_windup(true);

        // オープンループ始動コントローラ初期化
        let openloop = SixStepController::new(
            openloop::DEFAULT_INITIAL_RPM,
            openloop::DEFAULT_TARGET_RPM,
            openloop::DEFAULT_ACCELERATION_RPM_PER_S,
            openloop::DEFAULT_DUTY_RATIO,
            DEFAULT_POLE_PAIRS,
        );

        // キャリブレーション初期化（トルク0.1 = 10%、電力消費を抑える）
        let calibration = MotorCalibration::new(DEFAULT_POLE_PAIRS, 0.1);

        // デッドタイム補償器初期化
        let dead_time_comp = foc_mode::create_dead_time_compensation(max_duty);

        // フラックス弱め制御器初期化
        let flux_weakening = foc_mode::create_flux_weakening_controller();

        Self {
            hall_sensor,
            speed_pi,
            openloop,
            calibration,
            dead_time_comp,
            flux_weakening,
            ramped_target_speed: 0.0,
        }
    }

    /// 全リソースをリセット（モーター停止時に呼び出し）
    pub fn reset_all(&mut self) {
        self.speed_pi.reset();
        self.hall_sensor.reset();
        self.openloop.reset();
        openloop_mode::reset_execution_counter();
        hall_tim::reset_state();
        self.flux_weakening.reset();
        self.ramped_target_speed = 0.0;
    }

    /// OpenLoopモード用の準備
    pub fn prepare_for_openloop(&mut self) {
        self.speed_pi.reset();
        self.hall_sensor.reset();
        self.openloop.reset();
        openloop_mode::reset_execution_counter();
        foc_mode::reset_stall_counter();
        self.flux_weakening.reset();
        self.ramped_target_speed = 0.0;
    }

    /// FOCモード移行時の準備
    ///
    /// # Arguments
    /// * `current_rpm` - OpenLoopからの引き継ぎ速度
    pub fn prepare_for_foc(&mut self, current_rpm: f32) {
        // PI制御をリセット（クリーンな状態からスタート）
        self.speed_pi.reset();

        // FOCの脱落カウンタをリセット
        foc_mode::reset_stall_counter();

        // Hall センサーの速度フィルタを現在の速度で初期化
        self.hall_sensor.reset_speed_filter(current_rpm);

        // ランプも現在の速度からスタート（急激な変化を防ぐ）
        self.ramped_target_speed = current_rpm;
    }

    /// キャリブレーションモード用の準備
    pub fn prepare_for_calibration(&mut self, torque: f32) {
        self.calibration.set_torque(torque);
        self.calibration.start();
    }
}
