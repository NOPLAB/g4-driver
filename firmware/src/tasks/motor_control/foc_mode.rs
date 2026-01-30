//! FOC（Field Oriented Control）制御モード
//!
//! Hallセンサーベースのクローズドループ速度制御を実行します。

use core::sync::atomic::{AtomicU32, Ordering};

use crate::config::{dead_time_compensation, flux_weakening, foc_stall, pwm, speed, voltage};
use crate::fmt::*;
use crate::state::{ControlMode, RUNTIME};
// Use bldc crate for portable algorithms
use bldc::modulation::calculate_svpwm;
use bldc::transforms::{inverse_park, limit_voltage};

use bldc::compensation::{DeadTimeCompensation, FluxWeakeningController};

use crate::hall_tim;
use crate::state;

use super::mode::{ModeContext, ModeResult};

/// FOC詳細ログカウンタ（10Hz = 1000サイクルごと @ 10kHz）
static FOC_LOG_COUNTER: AtomicU32 = AtomicU32::new(0);

/// FOCモードログカウンタ（1Hz = 10000サイクルごと @ 10kHz）
static FOC_MODE_LOG_COUNTER: AtomicU32 = AtomicU32::new(0);

/// PIリセットまでの無効Hall状態の連続回数閾値
/// 10kHz制御で100回 = 10ms以上連続して無効な場合のみリセット
/// 短すぎると一時的なノイズで「カクッ」と止まる問題が発生
const INVALID_HALL_THRESHOLD: u32 = 100;

/// デッドタイム補償器を初期化
pub fn create_dead_time_compensation(max_duty: u16) -> DeadTimeCompensation {
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

/// FOCモードのハンドラ
pub struct FocMode;

impl FocMode {
    /// モード固有の名前
    #[allow(dead_code)]
    pub fn name(&self) -> &'static str {
        "ClosedLoopFoc"
    }

    /// モード開始時の初期化
    pub fn on_enter(&self, ctx: &mut ModeContext<'_>, _prev_mode: ControlMode) {
        // 実際のHall速度を取得（OpenLoopの理論値ではなく実測値を使用）
        let period = hall_tim::get_period_cycles();
        let actual_rpm =
            hall_tim::calculate_speed_rpm(period, crate::config::motor::DEFAULT_POLE_PAIRS);

        // 実測値が有効な場合はそれを使用、そうでなければOpenLoopの理論値を使用
        // OpenLoopの理論値は常に正なので、逆回転時は符号を付ける
        let is_reverse = RUNTIME.openloop.is_reverse();
        let theoretical_rpm = RUNTIME.openloop.get_theoretical_rpm(is_reverse);

        let current_rpm = if actual_rpm > 50.0 {
            // 実測値に方向の符号を付ける
            if is_reverse {
                -actual_rpm
            } else {
                actual_rpm
            }
        } else {
            theoretical_rpm
        };

        ctx.resources.prepare_for_foc(current_rpm);
        info!("Switching to FOC mode: speed={} RPM", current_rpm);
    }

    /// モード終了時のクリーンアップ
    pub fn on_exit(&self, _ctx: &mut ModeContext<'_>) {
        // FOC終了時：特別な処理なし
    }

    /// 1制御サイクルの実行
    pub async fn execute(&self, ctx: &mut ModeContext<'_>) -> ModeResult {
        execute_foc_internal(ctx).await
    }
}

/// シングルトンインスタンス
pub static FOC_MODE: FocMode = FocMode;

/// FOC制御の内部実装
async fn execute_foc_internal(ctx: &mut ModeContext<'_>) -> ModeResult {
    let hall_sensor = &mut ctx.resources.hall_sensor;
    let speed_pi = &mut ctx.resources.speed_pi;
    let motor_driver = &mut ctx.motor_driver;
    let dead_time_comp = &ctx.resources.dead_time_comp;
    let flux_weakening = &mut ctx.resources.flux_weakening;
    let ramped_target_speed = &mut ctx.resources.ramped_target_speed;
    let dt = ctx.dt;

    // 電気角と速度とHall状態を取得（TIM4ハードウェアベース）
    // Hall状態は1回だけ読み取り、再取得による競合を防止
    let (hall_electrical_angle, speed_rpm_abs, hall_state) = hall_sensor.update(dt);
    let is_valid_hall = (1..=6).contains(&hall_state);

    // Hallセンサが無効な場合の処理
    // 注意: 一時的なノイズ（1-2サイクル）ではPIリセットしない
    // 注意: ramped_target_speed はリセットしない（一時的なノイズで完全停止しないように）
    if !is_valid_hall {
        // 無効状態カウンタをインクリメント
        let invalid_count = RUNTIME.foc.increment_invalid_hall();

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

        return ModeResult::Continue;
    }

    // 有効なHall状態: 無効カウンタをリセット
    RUNTIME.foc.reset_invalid_hall();

    // FOC入力パラメータを一括取得（1回のMutexロックで統合）
    let foc_params = state::get_foc_input_params().await;
    let target_speed = foc_params.target_speed;
    let (kp, ki) = foc_params.pi_gains;

    // PIゲイン更新チェック（非同期で更新された場合）
    if kp != speed_pi.get_kp() || ki != speed_pi.get_ki() {
        speed_pi.set_gains(kp, ki);
        info!("PI gains updated: Kp={}, Ki={}", kp, ki);
    }

    // 逆転対応：目標速度の符号に基づいてフィードバック速度の符号を推定
    // Hallセンサーは回転方向を検出できないため、目標速度の符号を使用
    let speed_rpm = if *ramped_target_speed < 0.0 {
        -speed_rpm_abs
    } else {
        speed_rpm_abs
    };

    // 速度ランプ（加速度制限）を適用
    let speed_error = target_speed - *ramped_target_speed;
    let max_delta_speed = speed::MAX_ACCELERATION * dt; // 1制御周期で変化可能な最大速度

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
    let vd_cmd = flux_weakening.calculate_vd(speed_rpm, vq_cmd, voltage::DEFAULT_DC_BUS, dt);

    // 停止時の処理：目標速度が0で実際に停止している場合、PI積分項をリセット
    if ramped_target_speed.abs() < 1.0 && speed_rpm.abs() < 1.0 {
        speed_pi.reset();
        vq_cmd = 0.0;
    }

    // 最小電圧適用（走行中の脱調防止）
    // 目標速度と実測速度の差が大きい場合のみ最小電圧を適用
    // 実測速度が目標に近い場合は適用しない（発振防止）
    let speed_error = *ramped_target_speed - speed_rpm;
    if speed_error > 10.0 {
        // 正方向加速時：正の最小電圧
        vq_cmd = vq_cmd.max(voltage::MIN);
    } else if speed_error < -10.0 {
        // 逆方向加速時：負の最小電圧
        vq_cmd = vq_cmd.min(-voltage::MIN);
    }

    // 電圧ベクトル制限（100%）
    let max_voltage = voltage::DEFAULT_DC_BUS * 1.0;

    // 逆回転対応：Vqの符号を保持しながら電圧制限
    let (vd_limited, vq_limited) = limit_voltage(vd_cmd, vq_cmd, max_voltage);

    // 電圧飽和検出：飽和状態が続くと脱調の原因になるためカウント
    // 50%以上の飽和（vq_limitedがvq_cmdの半分以下）は危険な状態
    let vq_cmd_abs = vq_cmd.abs();
    let is_severe_saturation = vq_cmd_abs > 0.1 && (vq_limited.abs() / vq_cmd_abs) < 0.5;
    if is_severe_saturation {
        RUNTIME.foc.increment_saturation();
    } else {
        RUNTIME.foc.reset_saturation();
    }

    // Park逆変換（dq → αβ）
    let (v_alpha, v_beta) = inverse_park(vd_limited, vq_limited, hall_electrical_angle);

    // SVPWM計算（実際のPWM最大値を使用）
    let pwm_max_duty = motor_driver.max_duty();
    let (duty_u, duty_v, duty_w) =
        calculate_svpwm(v_alpha, v_beta, voltage::DEFAULT_DC_BUS, pwm_max_duty);

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
    RUNTIME.status.update(speed_rpm, hall_electrical_angle);

    // 速度低下（脱落）検出
    // 目標速度が設定されているのに実測速度が閾値以下の場合をカウント
    // 逆回転対応：絶対値で判定
    let target_speed_magnitude = ramped_target_speed.abs();
    let speed_rpm_magnitude = speed_rpm.abs();
    if target_speed_magnitude > foc_stall::SPEED_THRESHOLD
        && speed_rpm_magnitude < foc_stall::SPEED_THRESHOLD
    {
        let stall_count = RUNTIME.foc.increment_stall();

        if stall_count >= foc_stall::COUNT_THRESHOLD {
            warn!(
                "[FOC] Stall detected: speed={} RPM < {} RPM for {}+ cycles, switching to OpenLoop",
                speed_rpm_magnitude,
                foc_stall::SPEED_THRESHOLD,
                foc_stall::COUNT_THRESHOLD
            );
            // カウンタをリセット（次回のFOC移行に備える）
            RUNTIME.foc.reset_stall();
            return ModeResult::TransitionTo(ControlMode::OpenLoop);
        }
    } else {
        // 速度が回復したらカウンタをリセット
        RUNTIME.foc.reset_stall();
    }

    // デバッグログ（低頻度）- ローカル変数を再利用してMutexロックを回避
    let mode_count = FOC_MODE_LOG_COUNTER.fetch_add(1, Ordering::Relaxed);
    if mode_count >= 10000 {
        // 1秒ごと（10kHz / 10000 = 1Hz）
        FOC_MODE_LOG_COUNTER.store(0, Ordering::Relaxed);

        // TIM4ベースのHallセンサ値を取得（ログ用）
        let period_cycles = hall_tim::get_period_cycles();

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

    ModeResult::Continue
}
