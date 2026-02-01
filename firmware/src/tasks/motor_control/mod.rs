//! モーター制御タスク
//!
//! 10kHz FOCループ + オープンループ始動制御を実行します。
//! 各制御モードは独立したモジュールに分離されています。

mod calibration_mode;
mod foc_mode;
mod mode;
mod openloop_mode;
mod resources;

use embassy_time::{Duration, Timer};

use crate::board::MotorDriver;
use crate::config::{control, motor, pwm};
use crate::fmt::*;
use crate::state;
use crate::state::ControlMode;

use calibration_mode::CALIBRATION_MODE;
use foc_mode::FOC_MODE;
use mode::ModeContext;
use openloop_mode::OPENLOOP_MODE;
use resources::ControllerResources;

/// モーターコントローラー
///
/// 各モード状態を保持し、統一インターフェースで制御を実行
struct MotorController {
    /// モータードライバー
    motor_driver: MotorDriver,
    /// 制御リソース
    resources: ControllerResources,
    /// 現在の制御モード
    current_mode: ControlMode,
    /// モーター有効状態の追跡（PWMチャネル制御用）
    was_enabled: bool,
    /// 制御周期 [s]
    dt: f32,
}

impl MotorController {
    /// 新しいMotorControllerを作成
    fn new(motor_driver: MotorDriver) -> Self {
        let resources = ControllerResources::new(motor_driver.max_duty());

        Self {
            motor_driver,
            resources,
            current_mode: ControlMode::OpenLoop,
            was_enabled: false,
            dt: control::DEFAULT_PERIOD_US as f32 / 1_000_000.0,
        }
    }

    /// モーター有効状態をチェック
    async fn is_enabled(&self) -> bool {
        state::motor_context().await.enabled
    }

    /// モーター無効時の処理
    async fn handle_disabled(&mut self) {
        if self.was_enabled {
            info!("Motor control loop: Disabling PWM channels");
            self.was_enabled = false;
        }

        // モーター停止：PWMチャネルを完全無効化
        self.motor_driver.stop();

        // 全リソースをリセット
        self.resources.reset_all();
        self.current_mode = ControlMode::OpenLoop;
    }

    /// モーター有効化時の処理
    fn handle_enabled(&mut self) {
        if !self.was_enabled {
            info!("Motor control loop: Starting with OpenLoop mode");
            self.motor_driver.enable_all_channels();
            self.was_enabled = true;
        }
    }

    /// キャリブレーションリクエストをチェック
    async fn handle_calibration_request(&mut self) {
        let mut calib_ctx = state::calibration_context().await;
        if calib_ctx.request {
            info!("Calibration requested, switching to Calibration mode");
            calib_ctx.request = false;
            let torque_f32 = calib_ctx.torque as f32 / 100.0;
            drop(calib_ctx);

            info!("Starting motor calibration...");
            info!("  Pole pairs: {}", motor::DEFAULT_POLE_PAIRS);
            info!("  Torque: {}", torque_f32);

            self.resources.prepare_for_calibration(torque_f32);
            self.current_mode = ControlMode::Calibration;
            state::motor_context().await.control_mode = ControlMode::Calibration;
        }
    }

    /// 現在のモードを実行
    async fn execute_current_mode(&mut self) {
        let mut ctx = ModeContext::new(&mut self.resources, &mut self.motor_driver, self.dt);

        let result = match self.current_mode {
            ControlMode::OpenLoop => OPENLOOP_MODE.execute(&mut ctx).await,
            ControlMode::ClosedLoopFoc => FOC_MODE.execute(&mut ctx).await,
            ControlMode::Calibration => CALIBRATION_MODE.execute(&mut ctx).await,
        };

        // モード遷移処理
        if let Some(next_mode) = result.next_mode() {
            self.transition_to(next_mode);
        }
    }

    /// モード遷移を実行
    fn transition_to(&mut self, next_mode: ControlMode) {
        let prev_mode = self.current_mode;

        // 旧モードの終了処理
        {
            let mut ctx = ModeContext::new(&mut self.resources, &mut self.motor_driver, self.dt);
            match prev_mode {
                ControlMode::OpenLoop => OPENLOOP_MODE.on_exit(&mut ctx),
                ControlMode::ClosedLoopFoc => FOC_MODE.on_exit(&mut ctx),
                ControlMode::Calibration => CALIBRATION_MODE.on_exit(&mut ctx),
            }
        }

        // モードを更新
        self.current_mode = next_mode;

        // 新モードの開始処理
        {
            let mut ctx = ModeContext::new(&mut self.resources, &mut self.motor_driver, self.dt);
            match next_mode {
                ControlMode::OpenLoop => OPENLOOP_MODE.on_enter(&mut ctx, prev_mode),
                ControlMode::ClosedLoopFoc => FOC_MODE.on_enter(&mut ctx, prev_mode),
                ControlMode::Calibration => CALIBRATION_MODE.on_enter(&mut ctx, prev_mode),
            }
        }
    }
}

/// モーター制御タスク（10kHz FOC制御ループ）
#[embassy_executor::task]
pub async fn motor_control_task(motor_driver: MotorDriver) {
    // 電源投入後、ハードウェア安定待ち
    Timer::after(Duration::from_millis(500)).await;

    info!("Motor control task started (OpenLoop + FOC mode)");

    // 制御周期
    let dt = control::DEFAULT_PERIOD_US as f32 / 1_000_000.0;

    info!(
        "FOC parameters: Pole pairs={}, Control freq={}Hz, dt={}s",
        motor::DEFAULT_POLE_PAIRS,
        1_000_000 / control::DEFAULT_PERIOD_US,
        dt
    );
    info!(
        "PWM configuration: Frequency={}Hz, Max duty={}",
        pwm::DEFAULT_FREQUENCY.0,
        motor_driver.max_duty()
    );

    // モーターコントローラー初期化
    let mut controller = MotorController::new(motor_driver);

    loop {
        // 1. モーター使能チェック
        if !controller.is_enabled().await {
            controller.handle_disabled().await;
            Timer::after(Duration::from_micros(control::DEFAULT_PERIOD_US)).await;
            continue;
        }

        // モーター有効化時の処理
        controller.handle_enabled();

        // 2. キャリブレーションリクエストをチェック
        controller.handle_calibration_request().await;

        // 3. 制御モード別処理
        controller.execute_current_mode().await;

        Timer::after(Duration::from_micros(control::DEFAULT_PERIOD_US)).await;
    }
}
