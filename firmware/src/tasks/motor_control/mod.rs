//! モーター制御タスク
//!
//! bldcクレートの状態機械を使用したFOC制御を実行します。
//! 各制御モード（OpenLoop、FOC、Calibration）は状態機械で管理されます。

mod adapters;

use core::f32::consts::PI;

use embassy_time::{Duration, Timer};

use bldc::control::stall_detector::StallDetectorConfig;
use bldc::state_machine::modes::FocModeBuilder;
use bldc::state_machine::{
    CalibrationMode, ControlState, ModeOutput, OpenLoopMode, StateTransition,
};
use bldc::traits::{ControlMode, StatusOutput};
use bldc::{FocConfig, OpenLoopConfig};

use crate::board::MotorDriver;
use crate::config::{
    control, dead_time_compensation, flux_weakening, foc_stall, motor, openloop, pwm, speed,
    voltage,
};
use crate::fmt::*;
use crate::state::{self, RUNTIME};

use adapters::{
    to_firmware_control_mode, FirmwareControlInput, FirmwareHardware, FirmwareStatusOutput,
};

/// モーター制御タスク（10kHz FOC制御ループ）
#[embassy_executor::task]
pub async fn motor_control_task(motor_driver: MotorDriver) {
    // 電源投入後、ハードウェア安定待ち
    Timer::after(Duration::from_millis(500)).await;

    info!("Motor control task started");
    info!(
        "  Control freq={}Hz, Pole pairs={}",
        1_000_000 / control::DEFAULT_PERIOD_US,
        motor::DEFAULT_POLE_PAIRS
    );
    info!(
        "  PWM freq={}Hz, Max duty={}",
        pwm::DEFAULT_FREQUENCY.0,
        motor_driver.max_duty()
    );

    let dt = control::DEFAULT_PERIOD_US as f32 / 1_000_000.0;
    let max_duty = motor_driver.max_duty();

    // ハードウェアアダプター初期化
    let mut hw = FirmwareHardware::new(motor_driver);

    // 制御入力アダプター初期化
    let mut input = FirmwareControlInput::default();

    // ステータス出力アダプター初期化
    let mut output = FirmwareStatusOutput::default();

    // 状態機械の初期状態（OpenLoop）
    let mut control_state = create_openloop_state(max_duty, false);

    let mut was_enabled = false;
    let mut last_mode = ControlMode::OpenLoop;

    loop {
        // 1. 入力パラメータを非同期で取得
        input.fetch_from_state().await;

        // 2. モーター有効チェック
        if !input.motor_enabled {
            if was_enabled {
                info!("Motor disabled");
                was_enabled = false;
            }
            hw.stop();
            control_state = create_openloop_state(max_duty, false);
            Timer::after(Duration::from_micros(control::DEFAULT_PERIOD_US)).await;
            continue;
        }

        if !was_enabled {
            info!("Motor enabled, starting with OpenLoop mode");
            hw.motor_driver.enable_all_channels();
            was_enabled = true;
        }

        // 3. キャリブレーションリクエストチェック
        if input.calibration_requested {
            info!("Calibration requested, torque={}", input.calibration_torque);
            control_state = create_calibration_state(max_duty, input.calibration_torque);
            input.clear_calibration_request();
        }

        // 4. Hallセンサー更新
        hw.update_hall(dt);

        // 5. 制御状態更新
        let mode_output = update_control_state(&mut control_state, &mut hw, &input, dt);

        // 6. PWM出力（PwmDutyを直接適用）
        hw.motor_driver
            .set_duty_uvw(mode_output.duty.u, mode_output.duty.v, mode_output.duty.w);
        hw.motor_driver.enable_all_channels();

        // 7. ステータス更新
        output.update_status(mode_output.speed_rpm, mode_output.electrical_angle);

        // 8. 状態遷移処理
        if let Some(transition) = mode_output.transition {
            apply_transition(
                &mut control_state,
                &mut hw,
                &mut output,
                transition,
                max_duty,
            )
            .await;
        }

        // 9. モード変更通知
        let current_mode = get_control_mode(&control_state);
        if current_mode != last_mode {
            output.on_mode_change(current_mode);
            last_mode = current_mode;

            // グローバル状態を更新
            state::motor_context().await.control_mode = to_firmware_control_mode(current_mode);
        }

        Timer::after(Duration::from_micros(control::DEFAULT_PERIOD_US)).await;
    }
}

/// OpenLoop状態を作成
fn create_openloop_state(max_duty: u16, is_recovery: bool) -> ControlState {
    let config = OpenLoopConfig {
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
    };

    let mode = OpenLoopMode::new(config, is_recovery);

    if is_recovery {
        RUNTIME.openloop.reset_for_recovery();
    } else {
        RUNTIME.openloop.reset_for_normal();
    }

    ControlState::OpenLoop(mode)
}

/// FOC状態を作成
fn create_foc_state(max_duty: u16, initial_vq: f32, current_rpm: f32) -> ControlState {
    let config = FocConfig {
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
    };

    let dead_time_comp = bldc::DeadTimeCompensation::new(
        dead_time_compensation::DEAD_TIME_NS,
        pwm::DEFAULT_FREQUENCY.0,
        voltage::DEFAULT_DC_BUS,
        max_duty,
    );

    let flux_weakening = bldc::FluxWeakeningController::new(
        flux_weakening::MIN_SPEED,
        flux_weakening::MAX_SPEED,
        flux_weakening::MAX_WEAKENING_RATIO,
        flux_weakening::VD_RATE_LIMIT,
    );

    let stall_config = StallDetectorConfig {
        speed_threshold: foc_stall::SPEED_THRESHOLD,
        count_threshold: foc_stall::COUNT_THRESHOLD,
    };

    let mode = FocModeBuilder::new(config)
        .with_initial_vq(initial_vq)
        .with_current_rpm(current_rpm)
        .with_invalid_hall_threshold(100)
        .with_dead_time_compensation(dead_time_comp)
        .with_flux_weakening(flux_weakening)
        .with_stall_detection(stall_config)
        .build();

    RUNTIME.foc.reset_all();

    ControlState::Foc(mode)
}

/// Calibration状態を作成
fn create_calibration_state(max_duty: u16, torque: f32) -> ControlState {
    let mode = CalibrationMode::new(
        motor::DEFAULT_POLE_PAIRS,
        max_duty,
        voltage::DEFAULT_DC_BUS,
        voltage::DEFAULT_MAX,
        torque,
    );

    ControlState::Calibration(mode)
}

/// 制御状態を更新
fn update_control_state(
    state: &mut ControlState,
    hw: &mut FirmwareHardware,
    input: &FirmwareControlInput,
    dt: f32,
) -> ModeOutput {
    match state {
        ControlState::OpenLoop(mode) => mode.update(hw, input, dt),
        ControlState::Foc(mode) => mode.update(hw, input, dt),
        ControlState::Calibration(mode) => mode.update(hw, input, dt),
    }
}

/// 現在の制御モードを取得
fn get_control_mode(state: &ControlState) -> ControlMode {
    match state {
        ControlState::OpenLoop(_) => ControlMode::OpenLoop,
        ControlState::Foc(_) => ControlMode::Foc,
        ControlState::Calibration(_) => ControlMode::Calibration,
    }
}

/// 状態遷移を適用
async fn apply_transition(
    state: &mut ControlState,
    hw: &mut FirmwareHardware,
    output: &mut FirmwareStatusOutput,
    transition: StateTransition,
    max_duty: u16,
) {
    match transition {
        StateTransition::ToFoc {
            initial_vq,
            current_rpm,
            is_reverse: _,
        } => {
            info!("Transition to FOC: rpm={}, vq={}", current_rpm, initial_vq);
            hw.reset_speed_filter(current_rpm);
            *state = create_foc_state(max_duty, initial_vq, current_rpm);
        }
        StateTransition::ToOpenLoop { is_recovery } => {
            if is_recovery {
                info!("Transition to OpenLoop (recovery mode)");
                output.on_stall_detected();
            } else {
                info!("Transition to OpenLoop");
            }
            hw.hall_sensor.reset();
            *state = create_openloop_state(max_duty, is_recovery);
        }
        StateTransition::ToCalibration { torque } => {
            info!("Transition to Calibration: torque={}", torque);
            *state = create_calibration_state(max_duty, torque);
        }
    }

    // キャリブレーション完了時の結果処理
    if let ControlState::Calibration(mode) = state {
        if mode.is_completed() {
            let result = mode.get_result();
            if result.success {
                info!("Calibration completed successfully!");
                info!(
                    "  Electrical offset: {} rad ({} deg)",
                    result.electrical_offset,
                    result.electrical_offset * 180.0 / PI
                );
                info!("  Direction inversed: {}", result.direction_inversed);

                // 結果をグローバル状態に保存
                state::calibration_context().await.result = result;

                // Hallセンサーに結果を適用
                hw.hall_sensor
                    .set_electrical_offset(result.electrical_offset);
                hw.hall_sensor
                    .set_direction_inversed(result.direction_inversed);

                output.on_calibration_complete(
                    true,
                    result.electrical_offset,
                    result.direction_inversed,
                );
            } else {
                error!("Calibration failed!");
                output.on_calibration_complete(false, 0.0, false);
            }
        }
    }
}
