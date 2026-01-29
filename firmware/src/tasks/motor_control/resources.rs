//! モーター制御リソース管理
//!
//! 全ての制御リソースを一元管理し、モード切り替え時のリセット処理を統一します。

use bldc::calibration::MotorCalibration;
use bldc::compensation::{DeadTimeCompensation, FluxWeakeningController};
use bldc::control::six_step::SixStepController;
use bldc::control::PiController;
use core::f32::consts::PI;

use crate::adapters::HallSensorAdapter;
use crate::config::{hall, motor, openloop, speed, voltage};
use crate::hall_tim;
use crate::state::RUNTIME;

use super::foc_mode;

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
            HallSensorAdapter::new(motor::DEFAULT_POLE_PAIRS, speed::DEFAULT_FILTER_ALPHA);
        hall_sensor.set_interpolation(false); // 角度補間を無効化（ノイズ対策）

        // 電気オフセットを設定（キャリブレーション値）
        let offset_rad = hall::DEFAULT_ANGLE_OFFSET_DEG * PI / 180.0;
        hall_sensor.set_electrical_offset(offset_rad);

        // 速度PIコントローラ初期化（アンチワインドアップ有効）
        let mut speed_pi =
            PiController::new_symmetric(speed::DEFAULT_KP, speed::DEFAULT_KI, voltage::DEFAULT_MAX);
        speed_pi.set_anti_windup(true);

        // オープンループ始動コントローラ初期化
        let openloop_ctrl = SixStepController::new(
            openloop::DEFAULT_INITIAL_RPM,
            openloop::DEFAULT_TARGET_RPM,
            openloop::DEFAULT_ACCELERATION,
            openloop::DEFAULT_DUTY_RATIO,
            motor::DEFAULT_POLE_PAIRS,
        );

        // キャリブレーション初期化（トルク0.1 = 10%、電力消費を抑える）
        let calibration = MotorCalibration::new(motor::DEFAULT_POLE_PAIRS, 0.1);

        // デッドタイム補償器初期化
        let dead_time_comp = foc_mode::create_dead_time_compensation(max_duty);

        // フラックス弱め制御器初期化
        let flux_weakening = foc_mode::create_flux_weakening_controller();

        Self {
            hall_sensor,
            speed_pi,
            openloop: openloop_ctrl,
            calibration,
            dead_time_comp,
            flux_weakening,
            ramped_target_speed: 0.0,
        }
    }

    /// 共通のリセット処理
    fn reset_common(&mut self) {
        self.speed_pi.reset();
        self.hall_sensor.reset();
        self.openloop.reset();
        hall_tim::reset_state();
        self.flux_weakening.reset();
        self.ramped_target_speed = 0.0;
    }

    /// 全リソースをリセット（モーター停止時に呼び出し）
    pub fn reset_all(&mut self) {
        self.reset_common();
        RUNTIME.openloop.reset_for_normal();
        RUNTIME.foc.reset_all();
    }

    /// OpenLoopモード用の準備（通常起動・脱調回復共通）
    ///
    /// # Arguments
    /// * `is_recovery` - 脱調回復モードかどうか
    pub fn prepare_for_openloop_or_recovery(&mut self, is_recovery: bool) {
        self.reset_common();
        RUNTIME.foc.reset_all();

        if is_recovery {
            RUNTIME.openloop.reset_for_recovery();
        } else {
            RUNTIME.openloop.reset_for_normal();
        }
    }

    /// FOCモード移行時の準備
    ///
    /// # Arguments
    /// * `current_rpm` - OpenLoopからの引き継ぎ速度
    pub fn prepare_for_foc(&mut self, current_rpm: f32) {
        // PI制御をリセット（クリーンな状態からスタート）
        self.speed_pi.reset();

        // FOCの脱落カウンタをリセット
        RUNTIME.foc.reset_all();

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
