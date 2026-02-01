//! モーター制御リソース管理
//!
//! 全ての制御リソースを一元管理し、モード切り替え時のリセット処理を統一します。

use bldc::calibration::MotorCalibration;
use bldc::compensation::{DeadTimeCompensation, FluxWeakeningController};
use bldc::control::stall_detector::StallDetectorConfig;
use bldc::control::{FocConfig, FocController, OpenLoopConfig, OpenLoopController};
use core::f32::consts::PI;

use crate::adapters::HallSensorAdapter;
use crate::config::{
    dead_time_compensation, flux_weakening, foc_stall, hall, motor, openloop, pwm, speed, voltage,
};
use crate::hall_tim;
use crate::state::RUNTIME;

/// モーター制御に必要な全リソース
pub struct ControllerResources {
    /// Hallセンサー
    pub hall_sensor: HallSensorAdapter,
    /// FOCコントローラー（PIコントローラ、速度ランプ、脱調検出を統合）
    pub foc_controller: FocController,
    /// OpenLoopコントローラー（SVPWMベース）
    pub openloop_controller: OpenLoopController,
    /// キャリブレーションコントローラ
    pub calibration: MotorCalibration,
}

impl ControllerResources {
    /// 新しいリソースセットを作成
    pub fn new(max_duty: u16) -> Self {
        // ホールセンサ初期化
        let mut hall_sensor =
            HallSensorAdapter::new(motor::DEFAULT_POLE_PAIRS, speed::DEFAULT_FILTER_ALPHA);
        hall_sensor.set_interpolation(false); // 角度補間を無効化（ノイズ対策）

        // 電気オフセットを設定（キャリブレーション値）
        let offset_rad = hall::DEFAULT_ANGLE_OFFSET_DEG * PI / 180.0;
        hall_sensor.set_electrical_offset(offset_rad);

        // FOCコントローラー初期化（ビルダーパターンで全機能を統合）
        let foc_controller = FocController::builder(FocConfig {
            speed_kp: speed::DEFAULT_KP,
            speed_ki: speed::DEFAULT_KI,
            max_voltage: voltage::DEFAULT_MAX,
            v_dc: voltage::DEFAULT_DC_BUS,
            max_duty,
            vd: 0.0,
            max_acceleration: speed::MAX_ACCELERATION,
            min_voltage: voltage::MIN,
            min_voltage_error_threshold: voltage::MIN_ERROR_THRESHOLD,
            pi_integral_limit: speed::PI_INTEGRAL_LIMIT,
            anti_windup_enabled: true,
        })
        .with_dead_time_compensation(create_dead_time_compensation(max_duty))
        .with_flux_weakening(create_flux_weakening_controller())
        .with_stall_detection(StallDetectorConfig {
            speed_threshold: foc_stall::SPEED_THRESHOLD,
            count_threshold: foc_stall::COUNT_THRESHOLD,
        })
        .build();

        // OpenLoopコントローラー初期化（SVPWMベース）
        let openloop_controller = OpenLoopController::new(OpenLoopConfig {
            initial_rpm: openloop::DEFAULT_INITIAL_RPM,
            target_rpm: openloop::DEFAULT_TARGET_RPM,
            acceleration: openloop::DEFAULT_ACCELERATION,
            voltage_ratio: openloop::DEFAULT_DUTY_RATIO as f32 / 100.0,
            v_dc: voltage::DEFAULT_DC_BUS,
            max_duty,
            pole_pairs: motor::DEFAULT_POLE_PAIRS,
            forced_commutation_cycles: openloop::FORCED_COMMUTATION_CYCLES,
            min_cycles_for_foc: openloop::MIN_CYCLES_BEFORE_FOC,
            min_speed_for_foc: openloop::MIN_SPEED_FOR_FOC,
        });

        // キャリブレーション初期化（トルク0.1 = 10%、電力消費を抑える）
        let calibration = MotorCalibration::new(motor::DEFAULT_POLE_PAIRS, 0.1);

        Self {
            hall_sensor,
            foc_controller,
            openloop_controller,
            calibration,
        }
    }

    /// 共通のリセット処理
    fn reset_common(&mut self) {
        self.foc_controller.reset();
        self.hall_sensor.reset();
        hall_tim::reset_state();
    }

    /// 全リソースをリセット（モーター停止時に呼び出し）
    pub fn reset_all(&mut self) {
        self.reset_common();
        self.openloop_controller.reset_for_normal();
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
            self.openloop_controller.reset_for_recovery();
            RUNTIME.openloop.reset_for_recovery();
        } else {
            self.openloop_controller.reset_for_normal();
            RUNTIME.openloop.reset_for_normal();
        }
    }

    /// FOCモード移行時の準備
    ///
    /// OpenLoopのSVPWM駆動からFOCへスムーズに移行するため、
    /// PI制御器と速度フィルタを適切な初期値で初期化します。
    ///
    /// # Arguments
    /// * `current_rpm` - OpenLoopからの引き継ぎ速度
    pub fn prepare_for_foc(&mut self, current_rpm: f32) {
        // OpenLoopのDuty相当のVq初期値を計算（出力の連続性を確保）
        let initial_vq = (openloop::DEFAULT_DUTY_RATIO as f32 / 100.0) * voltage::DEFAULT_DC_BUS;
        self.foc_controller
            .initialize_for_foc(current_rpm, initial_vq);

        // FOCの脱落カウンタをリセット
        RUNTIME.foc.reset_all();

        // Hall センサーの速度フィルタを現在の速度で初期化
        self.hall_sensor.reset_speed_filter(current_rpm);
    }

    /// キャリブレーションモード用の準備
    pub fn prepare_for_calibration(&mut self, torque: f32) {
        self.calibration.set_torque(torque);
        self.calibration.start();
    }
}

/// デッドタイム補償器を初期化
fn create_dead_time_compensation(max_duty: u16) -> DeadTimeCompensation {
    let mut comp = DeadTimeCompensation::new(
        dead_time_compensation::DEAD_TIME_NS,
        pwm::DEFAULT_FREQUENCY.0,
        voltage::DEFAULT_DC_BUS,
        max_duty,
    );
    comp.set_enabled(dead_time_compensation::ENABLED);
    comp
}

/// フラックス弱め制御器を初期化
fn create_flux_weakening_controller() -> FluxWeakeningController {
    let mut fw = FluxWeakeningController::new(
        flux_weakening::MIN_SPEED,
        flux_weakening::MAX_SPEED,
        flux_weakening::MAX_WEAKENING_RATIO,
        flux_weakening::VD_RATE_LIMIT,
    );
    fw.set_enabled(flux_weakening::ENABLED);
    fw
}
