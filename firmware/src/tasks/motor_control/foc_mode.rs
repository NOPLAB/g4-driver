//! FOC（Field Oriented Control）制御モード
//!
//! Hallセンサーベースのクローズドループ速度制御を実行します。

use core::sync::atomic::{AtomicU32, Ordering};

use crate::config::*;
use crate::fmt::*;
use crate::foc::{
    calculate_svpwm, inverse_park, limit_voltage, DeadTimeCompensation, FluxWeakeningController,
    HallSensor, PiController,
};
use crate::hall_tim;
use crate::motor_driver::MotorDriver;
use crate::state;

/// FOC詳細ログカウンタ（10Hz = 1000サイクルごと @ 10kHz）
static FOC_LOG_COUNTER: AtomicU32 = AtomicU32::new(0);

/// FOCモードログカウンタ（1Hz = 10000サイクルごと @ 10kHz）
static FOC_MODE_LOG_COUNTER: AtomicU32 = AtomicU32::new(0);

/// 無効Hall状態の連続カウンタ（一時的なノイズでPIリセットしないため）
static INVALID_HALL_COUNTER: AtomicU32 = AtomicU32::new(0);

/// 速度低下（脱落）の連続カウンタ
static STALL_COUNTER: AtomicU32 = AtomicU32::new(0);

/// PIリセットまでの無効Hall状態の連続回数閾値
/// 10kHz制御で20回 = 2ms以上連続して無効な場合のみリセット
const INVALID_HALL_THRESHOLD: u32 = 20;

/// デッドタイム補償器を初期化
pub fn create_dead_time_compensation(max_duty: u16) -> DeadTimeCompensation {
    let mut comp = DeadTimeCompensation::new(
        dead_time_compensation::DEAD_TIME_NS,
        pwm::DEFAULT_FREQUENCY.0,
        DEFAULT_V_DC_BUS,
        max_duty,
    );
    comp.set_enabled(dead_time_compensation::ENABLED);
    comp
}

/// フラックス弱め制御器を初期化
pub fn create_flux_weakening_controller() -> FluxWeakeningController {
    let mut fw = FluxWeakeningController::new(
        flux_weakening::MIN_SPEED,
        flux_weakening::MAX_SPEED,
        flux_weakening::MAX_WEAKENING_RATIO,
        flux_weakening::VD_RATE_LIMIT,
    );
    fw.set_enabled(flux_weakening::ENABLED);
    fw
}

/// FOC制御の実行結果
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FocResult {
    /// 正常に継続
    Continue,
    /// Hall状態が無効（一時的なエラー、継続可能）
    InvalidHall,
    /// 速度が低下してOpenLoopに戻るべき（脱落検出）
    Stalled,
}

/// 脱落カウンタをリセット（モード切り替え時に呼び出す）
pub fn reset_stall_counter() {
    STALL_COUNTER.store(0, Ordering::Relaxed);
    INVALID_HALL_COUNTER.store(0, Ordering::Relaxed);
}

/// FOC制御の実行
///
/// # 引数
/// * `hall_sensor` - Hallセンサー
/// * `speed_pi` - 速度PIコントローラー
/// * `motor_driver` - モータードライバー
/// * `dead_time_comp` - デッドタイム補償器
/// * `flux_weakening` - フラックス弱め制御器
/// * `ramped_target_speed` - ランプ処理後の目標速度
/// * `dt` - 制御周期 [s]
///
/// # 戻り値
/// * `FocResult` - 実行結果（Continue, InvalidHall, Stalled）
pub async fn execute(
    hall_sensor: &mut HallSensor,
    speed_pi: &mut PiController,
    motor_driver: &mut MotorDriver,
    dead_time_comp: &DeadTimeCompensation,
    flux_weakening: &mut FluxWeakeningController,
    ramped_target_speed: &mut f32,
    dt: f32,
) -> FocResult {
    // 電気角と速度とHall状態を取得（TIM4ハードウェアベース、foc-simple互換計算）
    // Hall状態は1回だけ読み取り、再取得による競合を防止
    let (hall_electrical_angle, speed_rpm, hall_state) = hall_sensor.update(dt);
    let is_valid_hall = (1..=6).contains(&hall_state);

    // Hallセンサが無効な場合の処理
    // 注意: 一時的なノイズ（1-2サイクル）ではPIリセットしない
    // 注意: ramped_target_speed はリセットしない（一時的なノイズで完全停止しないように）
    if !is_valid_hall {
        // 無効状態カウンタをインクリメント
        let invalid_count = INVALID_HALL_COUNTER.fetch_add(1, Ordering::Relaxed) + 1;

        // 連続して無効な場合のみPWMを中立に設定
        if invalid_count >= INVALID_HALL_THRESHOLD {
            // 長時間無効: PWMを中立に設定し、PI積分項をリセット
            motor_driver.set_duty_uvw(
                motor_driver.max_duty() / 2,
                motor_driver.max_duty() / 2,
                motor_driver.max_duty() / 2,
            );
            speed_pi.reset();

            // 警告ログ（初回のみ）
            if invalid_count == INVALID_HALL_THRESHOLD {
                warn!(
                    "[FOC] Invalid Hall state persisted for {}+ cycles, PWM neutralized",
                    INVALID_HALL_THRESHOLD
                );
            }
        }
        // 一時的な無効状態: PWMは前回値を維持（何もしない）
        // これにより、Hall遷移中の一時的な無効状態でモーターが停止しない

        return FocResult::InvalidHall;
    }

    // 有効なHall状態: 無効カウンタをリセット
    INVALID_HALL_COUNTER.store(0, Ordering::Relaxed);

    // FOC入力パラメータを一括取得（1回のMutexロックで統合）
    let foc_params = state::get_foc_input_params().await;
    let target_speed = foc_params.target_speed;
    let (kp, ki) = foc_params.pi_gains;

    // PIゲイン更新チェック（非同期で更新された場合）
    if kp != speed_pi.get_kp() || ki != speed_pi.get_ki() {
        speed_pi.set_gains(kp, ki);
        info!("PI gains updated: Kp={}, Ki={}", kp, ki);
    }

    // 速度ランプ（加速度制限）を適用
    let speed_error = target_speed - *ramped_target_speed;
    let max_delta_speed = MAX_SPEED_ACCELERATION * dt; // 1制御周期で変化可能な最大速度

    if speed_error.abs() > max_delta_speed {
        // 加速度制限を適用
        if speed_error > 0.0 {
            *ramped_target_speed += max_delta_speed;
        } else {
            *ramped_target_speed -= max_delta_speed;
        }
    } else {
        // 目標速度に到達
        *ramped_target_speed = target_speed;
    }

    // 速度PI制御（q軸電圧指令生成）- ランプ処理後の速度を使用
    let mut vq_cmd = speed_pi.update(*ramped_target_speed, speed_rpm, dt);

    // フラックス弱め制御（高速域でd軸負電圧を印加）
    let vd_cmd = flux_weakening.calculate_vd(speed_rpm, vq_cmd, DEFAULT_V_DC_BUS, dt);

    // 停止時の処理：目標速度が0で実際に停止している場合、PI積分項をリセット
    if ramped_target_speed.abs() < 1.0 && speed_rpm.abs() < 1.0 {
        speed_pi.reset();
        vq_cmd = 0.0;
    }

    // 最小電圧適用（静止摩擦克服用）
    let speed_error_abs = (*ramped_target_speed - speed_rpm).abs();
    if speed_error_abs > MIN_VOLTAGE_ERROR_THRESHOLD && vq_cmd.abs() > 0.0 {
        // 速度誤差が大きい場合、最小電圧を適用
        if vq_cmd > 0.0 {
            vq_cmd = vq_cmd.max(MIN_VOLTAGE);
        } else {
            vq_cmd = vq_cmd.min(-MIN_VOLTAGE);
        }
    }

    // 電圧ベクトル制限（100%）
    let max_voltage = DEFAULT_V_DC_BUS * 1.0;

    // 逆回転防止：Vqを正の値のみに制限（一方向回転）
    let vq_cmd_positive = vq_cmd.max(0.0);
    let (vd_limited, vq_limited) = limit_voltage(vd_cmd, vq_cmd_positive, max_voltage);

    // Park逆変換（dq → αβ）
    let (v_alpha, v_beta) = inverse_park(vd_limited, vq_limited, hall_electrical_angle);

    // SVPWM計算（実際のPWM最大値を使用）
    let pwm_max_duty = motor_driver.max_duty();
    let (duty_u, duty_v, duty_w) = calculate_svpwm(v_alpha, v_beta, DEFAULT_V_DC_BUS, pwm_max_duty);

    // デッドタイム補償（SVPWM計算後、PWM出力前）
    let (duty_u, duty_v, duty_w) = dead_time_comp.compensate(
        duty_u,
        duty_v,
        duty_w,
        vq_limited,
        hall_electrical_angle,
        pwm_max_duty,
    );

    // デバッグ用：FOC制御の詳細ログ（10Hz = 1000回に1回 @ 10kHz）
    let count = FOC_LOG_COUNTER.fetch_add(1, Ordering::Relaxed);
    if count >= 1000 {
        FOC_LOG_COUNTER.store(0, Ordering::Relaxed);
        let angle_deg = hall_electrical_angle * 180.0 / core::f32::consts::PI;
        let advance_deg = hall_sensor.get_current_advance_deg();
        trace!(
            "[FOC Detail] Hall={}, Angle={}° (adv={}°), Vq={}V, Valpha={}V, Vbeta={}V, DutyU={}, DutyV={}, DutyW={}",
            hall_state, angle_deg, advance_deg, vq_limited, v_alpha, v_beta, duty_u, duty_v, duty_w
        );
    }

    // PWM出力
    motor_driver.set_duty_uvw(duty_u, duty_v, duty_w);

    // FOCモードではすべてのチャネルを有効化
    motor_driver.enable_all_channels();

    // ステータス更新（Atomic変数でロックフリー）
    state::update_motor_status_atomic(speed_rpm, hall_electrical_angle);

    // 速度低下（脱落）検出
    // 目標速度が設定されているのに実測速度が閾値以下の場合をカウント
    if *ramped_target_speed > foc_stall::STALL_SPEED_THRESHOLD
        && speed_rpm < foc_stall::STALL_SPEED_THRESHOLD
    {
        let stall_count = STALL_COUNTER.fetch_add(1, Ordering::Relaxed) + 1;

        if stall_count >= foc_stall::STALL_COUNT_THRESHOLD {
            warn!(
                "[FOC] Stall detected: speed={} RPM < {} RPM for {}+ cycles, switching to OpenLoop",
                speed_rpm,
                foc_stall::STALL_SPEED_THRESHOLD,
                foc_stall::STALL_COUNT_THRESHOLD
            );
            // カウンタをリセット（次回のFOC移行に備える）
            STALL_COUNTER.store(0, Ordering::Relaxed);
            return FocResult::Stalled;
        }
    } else {
        // 速度が回復したらカウンタをリセット
        STALL_COUNTER.store(0, Ordering::Relaxed);
    }

    // デバッグログ（低頻度）- ローカル変数を再利用してMutexロックを回避
    let mode_count = FOC_MODE_LOG_COUNTER.fetch_add(1, Ordering::Relaxed);
    if mode_count >= 10000 {
        // 1秒ごと（10kHz / 10000 = 1Hz）
        FOC_MODE_LOG_COUNTER.store(0, Ordering::Relaxed);

        // TIM4ベースのHallセンサ値を取得（ログ用）
        let period_cycles = hall_tim::get_period_cycles();

        // ローカル変数を使用（追加のMutexロック不要）
        debug!(
            "[FOC] Speed: {}/{} RPM (ramped: {}), Angle: {}rad, Hall: {}, Period: {} cycles",
            speed_rpm,
            target_speed,
            *ramped_target_speed,
            hall_electrical_angle,
            hall_state,
            period_cycles
        );
    }

    FocResult::Continue
}
