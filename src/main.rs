#![no_std]
#![no_main]

mod can_protocol;
mod fmt;
mod foc;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use fmt::*;

use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, SampleTime},
    bind_interrupts, can,
    gpio::{Input, Level, Output, Pull, Speed},
    opamp::{OpAmp, OpAmpSpeed},
    peripherals,
    time::Hertz,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        low_level::CountingMode,
        simple_pwm::PwmPin,
        Channel,
    },
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, Timer};

use can_protocol::{
    can_ids, encode_status, parse_enable_command, parse_pi_gains, parse_speed_command, MotorStatus,
};
use embedded_can::{Id, StandardId};
use foc::{
    calculate_svpwm, inverse_park, limit_voltage, ControlMode, HallSensor, OpenLoopSixStep,
    PiController,
};

// CANの割り込みをバインド
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

// モーター制御パラメータ（デフォルト値）
const DEFAULT_SPEED_KP: f32 = 0.1;
const DEFAULT_SPEED_KI: f32 = 0.01;
const MAX_VOLTAGE: f32 = 24.0; // V
const V_DC_BUS: f32 = 24.0; // V (DC bus voltage)
const POLE_PAIRS: u8 = 6; // 極対数（ポール数12 / 2 = 6）
const CONTROL_PERIOD_US: u64 = 1000; // 1kHz = 1000μs (1ms)
const MAX_DUTY: u16 = 100;
const SPEED_FILTER_ALPHA: f32 = 0.2; // ホールセンサ速度フィルタ係数

// オープンループ始動パラメータ（6ステップ駆動）
const OPENLOOP_INITIAL_RPM: f32 = 30.0; // 初期回転数 [RPM]
const OPENLOOP_TARGET_RPM: f32 = 200.0; // FOC切替回転数 [RPM]
const OPENLOOP_ACCELERATION_RPM_PER_S: f32 = 50.0; // 加速度 [RPM/s]
const OPENLOOP_DUTY_RATIO: u16 = 50; // デューティ比 (0-100)

// 共有状態（Mutexで保護）
static TARGET_SPEED: Mutex<ThreadModeRawMutex, f32> = Mutex::new(0.0);
static SPEED_PI_GAINS: Mutex<ThreadModeRawMutex, (f32, f32)> =
    Mutex::new((DEFAULT_SPEED_KP, DEFAULT_SPEED_KI));
static MOTOR_ENABLE: Mutex<ThreadModeRawMutex, bool> = Mutex::new(false);
static MOTOR_STATUS: Mutex<ThreadModeRawMutex, MotorStatus> = Mutex::new(MotorStatus::new());

// CAN通信タスク - モーター制御コマンド処理とステータス送信
#[embassy_executor::task]
pub async fn can_task(can: can::Can<'static>) {
    let (mut tx, mut rx, _properties) = can.split();

    info!("CAN motor control task started");

    // ステータス送信用タイマー（100ms周期）
    let mut status_ticker = Ticker::every(Duration::from_millis(100));

    loop {
        // CANフレーム受信とステータス送信を並行処理
        embassy_futures::select::select(
            async {
                // CANフレーム受信処理
                match rx.read().await {
                    Ok(envelope) => {
                        let frame = envelope.frame;
                        let data = frame.data();
                        let header = frame.header();

                        // IDを数値として取得
                        let id_raw = match header.id() {
                            Id::Standard(std_id) => std_id.as_raw() as u32,
                            Id::Extended(ext_id) => ext_id.as_raw(),
                        };

                        match id_raw {
                            can_ids::SPEED_CMD => {
                                if let Some(speed) = parse_speed_command(data) {
                                    *TARGET_SPEED.lock().await = speed;
                                }
                            }
                            can_ids::PI_GAINS => {
                                if let Some((kp, ki)) = parse_pi_gains(data) {
                                    *SPEED_PI_GAINS.lock().await = (kp, ki);
                                }
                            }
                            can_ids::ENABLE_CMD => {
                                if let Some(enable) = parse_enable_command(data) {
                                    *MOTOR_ENABLE.lock().await = enable;
                                    if enable {
                                        info!("Motor ENABLED via CAN");
                                    } else {
                                        info!("Motor DISABLED via CAN");
                                    }
                                }
                            }
                            can_ids::EMERGENCY_STOP => {
                                info!("Emergency stop received!");
                                *MOTOR_ENABLE.lock().await = false;
                                *TARGET_SPEED.lock().await = 0.0;
                            }
                            _ => {
                                debug!("Unknown CAN ID: 0x{:03X}", id_raw);
                            }
                        }
                    }
                    Err(e) => {
                        error!("CAN RX Error: {:?}", e);
                    }
                }
            },
            async {
                // ステータス送信（100ms周期）
                status_ticker.next().await;

                let status = *MOTOR_STATUS.lock().await;
                let data = encode_status(status.speed_rpm, status.electrical_angle);

                // Standard ID 0x200でフレームを作成
                if let Some(std_id) = StandardId::new(can_ids::STATUS as u16) {
                    let id = Id::Standard(std_id);
                    if let Ok(frame) = can::frame::Frame::new_data(id, &data) {
                        let _ = tx.write(&frame).await;
                    }
                }
            },
        )
        .await;
    }
}

#[embassy_executor::task]
pub async fn led_task(
    mut led1: Output<'static>,
    mut led2: Output<'static>,
    mut led3: Output<'static>,
) {
    loop {
        led1.set_high();
        led2.set_low();
        led3.set_low();
        Timer::after(Duration::from_millis(500)).await;

        led1.set_low();
        led2.set_high();
        led3.set_low();
        Timer::after(Duration::from_millis(500)).await;

        led1.set_low();
        led2.set_low();
        led3.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}

// モーター制御タスク（1kHz FOCループ + オープンループ始動）
#[embassy_executor::task]
pub async fn motor_control_task(
    hall_h1: Input<'static>,
    hall_h2: Input<'static>,
    hall_h3: Input<'static>,
    mut uvw_pwm: ComplementaryPwm<'static, peripherals::TIM1>,
) {
    info!("Motor control task started");

    // ホールセンサ初期化
    let mut hall_sensor = HallSensor::new(POLE_PAIRS, SPEED_FILTER_ALPHA);

    // 速度PIコントローラ初期化
    let mut speed_pi = PiController::new_symmetric(DEFAULT_SPEED_KP, DEFAULT_SPEED_KI, MAX_VOLTAGE);

    // オープンループ6ステップ駆動初期化
    let mut open_loop = OpenLoopSixStep::new(
        OPENLOOP_INITIAL_RPM,
        OPENLOOP_TARGET_RPM,
        OPENLOOP_ACCELERATION_RPM_PER_S,
        OPENLOOP_DUTY_RATIO,
        POLE_PAIRS,
    );

    // 制御モード（初期はオープンループ）
    let mut control_mode = ControlMode::OpenLoop;

    // 制御周期
    let dt = CONTROL_PERIOD_US as f32 / 1_000_000.0; // 秒に変換

    info!(
        "FOC parameters: Pole pairs={}, Control freq={}Hz, dt={}s",
        POLE_PAIRS,
        1_000_000 / CONTROL_PERIOD_US,
        dt
    );
    info!(
        "OpenLoop 6-Step startup: Initial={}RPM, Target={}RPM, Accel={}RPM/s, Duty={}%",
        OPENLOOP_INITIAL_RPM,
        OPENLOOP_TARGET_RPM,
        OPENLOOP_ACCELERATION_RPM_PER_S,
        OPENLOOP_DUTY_RATIO
    );

    // モーター有効状態の追跡（PWMチャネル制御用）
    let mut was_enabled = false;

    loop {
        // 1. モーター使能チェック
        let motor_enabled = *MOTOR_ENABLE.lock().await;
        if !motor_enabled {
            // 状態が変化した場合のみログとPWM停止処理
            if was_enabled {
                info!("Motor control loop: Disabling PWM channels");
                was_enabled = false;
                // 制御モードをオープンループに戻す
                control_mode = ControlMode::OpenLoop;
            }

            // モーター停止：PWMチャネルを完全無効化
            uvw_pwm.disable(Channel::Ch1);
            uvw_pwm.disable(Channel::Ch2);
            uvw_pwm.disable(Channel::Ch3);

            // Duty比も0にセット
            uvw_pwm.set_duty(Channel::Ch1, 0);
            uvw_pwm.set_duty(Channel::Ch2, 0);
            uvw_pwm.set_duty(Channel::Ch3, 0);

            // 各コントローラをリセット
            speed_pi.reset();
            hall_sensor.reset();
            open_loop.reset();

            Timer::after(Duration::from_micros(CONTROL_PERIOD_US)).await;
            continue;
        }

        // モーター有効化時の処理
        if !was_enabled {
            info!("Motor control loop: Starting with OpenLoop mode");
            uvw_pwm.enable(Channel::Ch1);
            uvw_pwm.enable(Channel::Ch2);
            uvw_pwm.enable(Channel::Ch3);
            was_enabled = true;
        }

        // 2. ホールセンサ読み取り（PB6=H1, PB7=H2, PB8=H3）
        let h1 = hall_h1.is_high();
        let h2 = hall_h2.is_high();
        let h3 = hall_h3.is_high();
        let hall_state = (h3 as u8) << 2 | (h2 as u8) << 1 | (h1 as u8);

        // 3. 電気角・速度推定（FOCモードで使用）
        let (hall_electrical_angle, speed_rpm) = hall_sensor.update(hall_state, dt);

        // 4. 制御モードに応じた処理
        match control_mode {
            ControlMode::OpenLoop => {
                // オープンループ6ステップ駆動
                let step_state = open_loop.update(dt);
                let openloop_rpm = open_loop.get_current_rpm();

                // しきい値速度に達したらFOCモードに切り替え
                if open_loop.is_target_reached() {
                    info!(
                        "Switching to FOC mode: OpenLoop reached {}RPM",
                        openloop_rpm
                    );
                    control_mode = ControlMode::ClosedLoopFoc;
                }

                // 6ステップ駆動のPWM設定
                uvw_pwm.set_duty(Channel::Ch1, step_state.duty_u);
                uvw_pwm.set_duty(Channel::Ch2, step_state.duty_v);
                uvw_pwm.set_duty(Channel::Ch3, step_state.duty_w);

                if step_state.enable_u {
                    uvw_pwm.enable(Channel::Ch1);
                } else {
                    uvw_pwm.disable(Channel::Ch1);
                }

                if step_state.enable_v {
                    uvw_pwm.enable(Channel::Ch2);
                } else {
                    uvw_pwm.disable(Channel::Ch2);
                }

                if step_state.enable_w {
                    uvw_pwm.enable(Channel::Ch3);
                } else {
                    uvw_pwm.disable(Channel::Ch3);
                }

                // ステータス更新
                {
                    let mut status = MOTOR_STATUS.lock().await;
                    status.speed_rpm = openloop_rpm;
                    status.electrical_angle = 0.0; // 6ステップでは電気角は使わない
                }
            }
            ControlMode::ClosedLoopFoc => {
                // クローズドループFOC制御

                // PIゲイン更新チェック（非同期で更新された場合）
                {
                    let (kp, ki) = *SPEED_PI_GAINS.lock().await;
                    if kp != speed_pi.get_kp() || ki != speed_pi.get_ki() {
                        speed_pi.set_gains(kp, ki);
                        info!("PI gains updated: Kp={}, Ki={}", kp, ki);
                    }
                }

                // 目標速度取得
                let target_speed = *TARGET_SPEED.lock().await;

                // 速度PI制御（q軸電圧指令生成）
                let vq_cmd = speed_pi.update(target_speed, speed_rpm, dt);
                let vd_cmd = 0.0; // SPMSM: d軸電流/電圧は0

                // 電圧ベクトル制限
                let (vd_limited, vq_limited) = limit_voltage(vd_cmd, vq_cmd, MAX_VOLTAGE);

                // Park逆変換（dq → αβ）
                let (v_alpha, v_beta) = inverse_park(vd_limited, vq_limited, hall_electrical_angle);

                // SVPWM計算
                let (duty_u, duty_v, duty_w) = calculate_svpwm(v_alpha, v_beta, V_DC_BUS, MAX_DUTY);

                // PWM出力
                uvw_pwm.set_duty(Channel::Ch1, duty_u);
                uvw_pwm.set_duty(Channel::Ch2, duty_v);
                uvw_pwm.set_duty(Channel::Ch3, duty_w);

                // FOCモードではすべてのチャネルを有効化
                uvw_pwm.enable(Channel::Ch1);
                uvw_pwm.enable(Channel::Ch2);
                uvw_pwm.enable(Channel::Ch3);

                // ステータス更新
                {
                    let mut status = MOTOR_STATUS.lock().await;
                    status.speed_rpm = speed_rpm;
                    status.electrical_angle = hall_electrical_angle;
                }
            }
        }

        // 5. デバッグログ（低頻度）
        static mut LOG_COUNTER: u32 = 0;
        unsafe {
            LOG_COUNTER += 1;
            if LOG_COUNTER >= 1000 {
                // 1秒ごと（1kHz / 1000 = 1Hz）
                LOG_COUNTER = 0;
                match control_mode {
                    ControlMode::OpenLoop => {
                        let openloop_rpm = open_loop.get_current_rpm();
                        debug!(
                            "[OpenLoop 6-Step] RPM: {}, Step: {}",
                            openloop_rpm, open_loop.get_current_step()
                        );
                    }
                    ControlMode::ClosedLoopFoc => {
                        let target_speed = *TARGET_SPEED.lock().await;
                        debug!(
                            "[FOC] Speed: {}/{} RPM, Angle: {}rad, Hall: {}",
                            speed_rpm, target_speed, hall_electrical_angle, hall_state
                        );
                    }
                }
            }
        }

        Timer::after(Duration::from_micros(CONTROL_PERIOD_US)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::Pll;
        use embassy_stm32::rcc::PllMul;
        use embassy_stm32::rcc::PllPreDiv;
        use embassy_stm32::rcc::PllRDiv;
        use embassy_stm32::rcc::PllSource;

        use embassy_stm32::rcc::Sysclk;

        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV2), // FDCANクロック用に追加
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R; // システムクロックをPLLに設定

        use embassy_stm32::rcc::mux::Adcsel;
        use embassy_stm32::rcc::mux::ClockMux;
        use embassy_stm32::rcc::mux::Fdcansel;

        let mut clock_mux = ClockMux::default();
        clock_mux.adc12sel = Adcsel::SYS;
        clock_mux.fdcansel = Fdcansel::PLL1_Q; // FDCANクロックをPLL1_Qに設定
        config.rcc.mux = clock_mux;
    }

    let p = embassy_stm32::init(config);

    let led1 = Output::new(p.PC13, Level::High, Speed::Low);
    let led2 = Output::new(p.PC14, Level::High, Speed::Low);
    let led3 = Output::new(p.PC15, Level::High, Speed::Low);

    spawner.spawn(led_task(led1, led2, led3)).unwrap();

    // CAN初期化（RX=PA11, TX=PA12）
    // ビットレートは250kbpsに設定
    let mut can_configurator = can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs);

    // すべての拡張IDフレームをFIFO1に受信
    can_configurator.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::accept_all_into_fifo1(),
    );

    // すべての標準IDフレームをFIFO0に受信
    can_configurator.properties().set_standard_filter(
        can::filter::StandardFilterSlot::_0,
        can::filter::StandardFilter::accept_all_into_fifo0(),
    );

    // ビットレート設定: 250kbps
    // 他の一般的な速度: 125kbps, 500kbps, 1000kbps
    can_configurator.set_bitrate(250_000);

    // CAN通信を開始（通常動作モード）
    let can = can_configurator.start(can::OperatingMode::NormalOperationMode);

    spawner.spawn(can_task(can)).unwrap();

    let mut adc1 = Adc::new(p.ADC1);
    adc1.set_sample_time(SampleTime::CYCLES640_5);
    let mut adc2 = Adc::new(p.ADC2);
    adc2.set_sample_time(SampleTime::CYCLES640_5);

    let mut op1 = OpAmp::new(p.OPAMP1, OpAmpSpeed::HighSpeed);
    let op1_sa = op1.pga_ext(p.PA1, p.PA2, embassy_stm32::opamp::OpAmpGain::Mul4);
    let mut op1_adc_ch = op1_sa.degrade_adc();

    let mut op2 = OpAmp::new(p.OPAMP2, OpAmpSpeed::Normal);
    let op2_sa = op2.standalone_ext(p.PA7, p.PC5, p.PA6);
    let mut op2_adc_ch = op2_sa.degrade_adc();

    let mut op3 = OpAmp::new(p.OPAMP3, OpAmpSpeed::Normal);
    let op3_sa = op3.standalone_ext(p.PB0, p.PB2, p.PB1);
    let mut op3_adc_ch = op3_sa.degrade_adc();

    let mut uvw_pwm = ComplementaryPwm::new(
        p.TIM1,
        Some(PwmPin::new(
            p.PE9,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(ComplementaryPwmPin::new(
            p.PE8,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(PwmPin::new(
            p.PE11,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(ComplementaryPwmPin::new(
            p.PE10,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(PwmPin::new(
            p.PE13,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(ComplementaryPwmPin::new(
            p.PE12,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        None,
        None,
        Hertz(50_000),
        CountingMode::EdgeAlignedUp,
    );

    uvw_pwm.disable(Channel::Ch1);
    uvw_pwm.disable(Channel::Ch2);
    uvw_pwm.disable(Channel::Ch3);
    uvw_pwm.set_dead_time(1);

    uvw_pwm.enable(Channel::Ch1);
    uvw_pwm.enable(Channel::Ch2);
    uvw_pwm.enable(Channel::Ch3);

    // ホールセンサ初期化（PB6=H1, PB7=H2, PB8=H3）
    let hall_h1 = Input::new(p.PB6, Pull::None);
    let hall_h2 = Input::new(p.PB7, Pull::None);
    let hall_h3 = Input::new(p.PB8, Pull::None);

    info!("Starting FOC motor control...");

    // モーター制御タスクを起動
    spawner
        .spawn(motor_control_task(hall_h1, hall_h2, hall_h3, uvw_pwm))
        .unwrap();

    info!("System initialized successfully");
    info!("Send CAN commands to control motor:");
    info!("  - 0x100: Speed command (f32 RPM)");
    info!("  - 0x101: PI gains (Kp, Ki as f32)");
    info!("  - 0x102: Enable motor (u8: 0=off, 1=on)");
    info!("  - 0x000: Emergency stop");

    // メインループ（オプション：ADC値の監視など）
    loop {
        // 電流センサ値の読み取り（現在は使用していないが、将来のために残す）
        let _op1_val = adc1.blocking_read(&mut op1_adc_ch);
        let _op2_val = adc2.blocking_read(&mut op2_adc_ch);
        let _op3_val = adc1.blocking_read(&mut op3_adc_ch);

        // 将来の拡張用（電流リミット監視など）
        Timer::after(Duration::from_millis(100)).await;
    }
}
