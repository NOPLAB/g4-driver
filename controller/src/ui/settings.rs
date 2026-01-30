use dioxus::prelude::*;
use tracing::{error, info};

use super::components::{
    Banner, BannerType, Button, ButtonVariant, Card, ErrorBanner, F32Input, HeaderColor,
    SectionHeader, StatusCard, StatusCardColor, U16Input, U32Input, U64Input, U8Input,
    WarningBanner,
};
use crate::can::commands::MotorCommand;
use crate::state::{AppState, ConnectionState};

// Default values (from firmware config) - grouped by category
mod defaults {
    // PI Control
    pub const KP: f32 = 0.5;
    pub const KI: f32 = 0.05;

    // Motor Control
    pub const MAX_VOLTAGE: f32 = 24.0;
    pub const V_DC_BUS: f32 = 24.0;
    pub const POLE_PAIRS: u8 = 6;
    pub const MAX_DUTY: u16 = 100;

    // Hall Sensor
    pub const SPEED_FILTER_ALPHA: f32 = 0.1;
    pub const HALL_ANGLE_OFFSET: f32 = 0.0;
    pub const ENABLE_ANGLE_INTERPOLATION: bool = true;

    // OpenLoop
    pub const OPENLOOP_INITIAL_RPM: f32 = 100.0;
    pub const OPENLOOP_TARGET_RPM: f32 = 500.0;
    pub const OPENLOOP_ACCELERATION: f32 = 100.0;
    pub const OPENLOOP_DUTY_RATIO: u16 = 50;
    pub const FORCED_COMMUTATION_CYCLES: u32 = 10000;
    pub const MIN_CYCLES_BEFORE_FOC: u32 = 10000;

    // Advance Angle
    pub const ADVANCE_BASE_DEG: f32 = 10.0;
    pub const ADVANCE_MAX_DEG: f32 = 30.0;
    pub const ADVANCE_MIN_SPEED: f32 = 100.0;
    pub const ADVANCE_MAX_SPEED: f32 = 3000.0;

    // Min Voltage
    pub const MIN_VOLTAGE: f32 = 2.0;
    pub const MIN_VOLTAGE_ERROR_THRESHOLD: f32 = 2.0;
    pub const MAX_SPEED_ACCELERATION: f32 = 100.0;

    // FOC Stall
    pub const FOC_STALL_SPEED_THRESHOLD: f32 = 50.0;
    pub const FOC_STALL_COUNT_THRESHOLD: u32 = 1000;

    // Dead Time Compensation
    pub const DEAD_TIME_COMP_ENABLED: bool = false;
    pub const DEAD_TIME_NS: f32 = 100.0;

    // Flux Weakening
    pub const FLUX_WEAKENING_ENABLED: bool = false;
    pub const FLUX_WEAKENING_MIN_SPEED: f32 = 2000.0;
    pub const FLUX_WEAKENING_MAX_SPEED: f32 = 4000.0;
    pub const FLUX_WEAKENING_MAX_RATIO: f32 = 0.5;
    pub const FLUX_WEAKENING_VD_RATE_LIMIT: f32 = 100.0;

    // Voltage Monitor
    pub const VOLTAGE_OVERVOLTAGE_THRESHOLD: f32 = 30.0;
    pub const VOLTAGE_UNDERVOLTAGE_THRESHOLD: f32 = 10.0;
    pub const VOLTAGE_FILTER_ALPHA: f32 = 0.1;

    // Hardware
    pub const PWM_FREQUENCY: u32 = 50000;
    pub const PWM_DEAD_TIME: u16 = 100;
    pub const CAN_BITRATE: u32 = 250000;
    pub const CONTROL_PERIOD_US: u64 = 400;
}

/// Helper to send a command asynchronously
fn send_command(app_state: Signal<AppState>, cmd: MotorCommand) {
    spawn(async move {
        let mgr = app_state.read().can_manager.clone();
        let result = mgr.lock().await.send(cmd).await;
        if let Err(e) = result {
            error!("Failed to send command: {}", e);
        }
    });
}

#[component]
pub fn SettingsPanel() -> Element {
    let app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    let is_connected = matches!(state.connection_state, ConnectionState::Connected);
    let selected_tab = use_signal(|| 0);

    rsx! {
        div {
            style: "display: flex; flex-direction: column; gap: 20px; max-width: 800px;",

            if !is_connected {
                WarningBanner {
                    message: "Not connected to CAN. Please connect first.".to_string()
                }
            }

            div {
                style: "display: flex; gap: 5px; border-bottom: 2px solid #ddd; padding-bottom: 0;",
                TabButton { selected_tab, index: 0, label: "PI Control" }
                TabButton { selected_tab, index: 1, label: "Motor Control" }
                TabButton { selected_tab, index: 2, label: "Hall Sensor" }
                TabButton { selected_tab, index: 3, label: "OpenLoop" }
                TabButton { selected_tab, index: 4, label: "FOC Advanced" }
                TabButton { selected_tab, index: 5, label: "Compensation" }
                TabButton { selected_tab, index: 6, label: "Voltage" }
                TabButton { selected_tab, index: 7, label: "Advanced" }
                TabButton { selected_tab, index: 8, label: "Calibration" }
            }

            match selected_tab() {
                0 => rsx! { PIControlTab { is_connected } },
                1 => rsx! { MotorControlTab { is_connected } },
                2 => rsx! { HallSensorTab { is_connected } },
                3 => rsx! { OpenLoopTab { is_connected } },
                4 => rsx! { FocAdvancedTab { is_connected } },
                5 => rsx! { CompensationTab { is_connected } },
                6 => rsx! { VoltageMonitorTab { is_connected } },
                7 => rsx! { AdvancedTab { is_connected } },
                8 => rsx! { CalibrationTab { is_connected } },
                _ => rsx! { div { "Invalid tab" } },
            }

            ConfigManagementSection { is_connected }
        }
    }
}

#[component]
fn TabButton(selected_tab: Signal<i32>, index: i32, label: &'static str) -> Element {
    let is_selected = selected_tab() == index;
    let style = if is_selected {
        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
    } else {
        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
    };

    rsx! {
        button {
            style: style,
            onclick: move |_| selected_tab.set(index),
            "{label}"
        }
    }
}

// ============================================================================
// PI Control Tab
// ============================================================================

#[component]
fn PIControlTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    let on_apply = move |_| {
        let kp = app_state.read().settings.pi.kp;
        let ki = app_state.read().settings.pi.ki;
        info!("Applying PI gains: Kp={}, Ki={}", kp, ki);
        send_command(app_state, MotorCommand::PiGains { kp, ki });
    };

    let on_reset = move |_| {
        app_state.write().settings.pi.kp = defaults::KP;
        app_state.write().settings.pi.ki = defaults::KI;
    };

    rsx! {
        Card {
            SectionHeader { title: "PI Controller Settings".to_string() }

            Banner {
                banner_type: BannerType::Info,
                message: "Configure the PI controller gains for speed control. These values affect the motor's response to speed commands.".to_string()
            }

            div { style: "display: grid; gap: 20px;",
                F32Input {
                    label: "Proportional Gain (Kp)".to_string(),
                    value: state.settings.pi.kp,
                    step: "0.01".to_string(),
                    on_change: move |v| { app_state.write().settings.pi.kp = v; },
                    is_connected,
                    description: format!("Higher values provide faster response but may cause oscillations. Default: {}", defaults::KP)
                }

                F32Input {
                    label: "Integral Gain (Ki)".to_string(),
                    value: state.settings.pi.ki,
                    step: "0.001".to_string(),
                    on_change: move |v| { app_state.write().settings.pi.ki = v; },
                    is_connected,
                    description: format!("Helps eliminate steady-state error. Too high may cause instability. Default: {}", defaults::KI)
                }

                div { style: "display: flex; gap: 10px; margin-top: 10px;",
                    Button {
                        variant: ButtonVariant::Success,
                        disabled: !is_connected,
                        custom_style: "flex: 1;".to_string(),
                        onclick: on_apply,
                        "Apply Settings"
                    }
                    Button {
                        variant: ButtonVariant::Secondary,
                        custom_style: "flex: 1;".to_string(),
                        onclick: on_reset,
                        "Reset to Defaults"
                    }
                }
            }
        }
    }
}

// ============================================================================
// Motor Control Tab
// ============================================================================

#[component]
fn MotorControlTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            SectionHeader { title: "Motor Control Parameters".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Configure motor voltage, pole pairs, and duty cycle parameters." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                F32Input {
                    label: "Max Voltage (V)".to_string(),
                    value: app_state.read().settings.motor.max_voltage,
                    step: "0.1".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.motor.max_voltage = v;
                        let vdc = app_state.read().settings.motor.v_dc_bus;
                        send_command(app_state, MotorCommand::MotorVoltage { max_voltage: v, v_dc_bus: vdc });
                    },
                    is_connected,
                    description: format!("Maximum voltage limit for motor control. Default: {}", defaults::MAX_VOLTAGE)
                }

                F32Input {
                    label: "DC Bus Voltage (V)".to_string(),
                    value: app_state.read().settings.motor.v_dc_bus,
                    step: "0.1".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.motor.v_dc_bus = v;
                        let max_v = app_state.read().settings.motor.max_voltage;
                        send_command(app_state, MotorCommand::MotorVoltage { max_voltage: max_v, v_dc_bus: v });
                    },
                    is_connected,
                    description: format!("DC bus voltage for calculations. Default: {}", defaults::V_DC_BUS)
                }

                U8Input {
                    label: "Pole Pairs".to_string(),
                    value: app_state.read().settings.motor.pole_pairs,
                    on_change: move |v| {
                        app_state.write().settings.motor.pole_pairs = v;
                        let duty = app_state.read().settings.motor.max_duty;
                        send_command(app_state, MotorCommand::MotorBasic { pole_pairs: v, max_duty: duty });
                    },
                    is_connected,
                    description: format!("Number of motor pole pairs (poles/2). Default: {}", defaults::POLE_PAIRS)
                }

                U16Input {
                    label: "Max Duty Cycle".to_string(),
                    value: app_state.read().settings.motor.max_duty,
                    on_change: move |v| {
                        app_state.write().settings.motor.max_duty = v;
                        let poles = app_state.read().settings.motor.pole_pairs;
                        send_command(app_state, MotorCommand::MotorBasic { pole_pairs: poles, max_duty: v });
                    },
                    is_connected,
                    description: format!("Maximum PWM duty cycle (0-100). Default: {}", defaults::MAX_DUTY)
                }
            }
        }
    }
}

// ============================================================================
// Hall Sensor Tab
// ============================================================================

#[component]
fn HallSensorTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            SectionHeader { title: "Hall Sensor Parameters".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Configure Hall sensor filter and angle offset." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                F32Input {
                    label: "Speed Filter Alpha".to_string(),
                    value: app_state.read().settings.hall_sensor.speed_filter_alpha,
                    step: "0.01".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.hall_sensor.speed_filter_alpha = v;
                        let offset = app_state.read().settings.hall_sensor.angle_offset;
                        send_command(app_state, MotorCommand::HallSensor { speed_filter_alpha: v, angle_offset: offset });
                    },
                    is_connected,
                    description: format!("Low-pass filter coefficient for speed (0-1). Default: {}", defaults::SPEED_FILTER_ALPHA)
                }

                F32Input {
                    label: "Hall Angle Offset (rad)".to_string(),
                    value: app_state.read().settings.hall_sensor.angle_offset,
                    step: "0.01".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.hall_sensor.angle_offset = v;
                        let alpha = app_state.read().settings.hall_sensor.speed_filter_alpha;
                        send_command(app_state, MotorCommand::HallSensor { speed_filter_alpha: alpha, angle_offset: v });
                    },
                    is_connected,
                    description: format!("Angle offset for Hall sensor alignment. Default: {}", defaults::HALL_ANGLE_OFFSET)
                }

                div {
                    label { style: "font-size: 14px; font-weight: 500; color: #555; display: flex; align-items: center; gap: 10px;",
                        input {
                            r#type: "checkbox",
                            checked: app_state.read().settings.hall_sensor.enable_interpolation,
                            disabled: !is_connected,
                            onchange: move |evt| {
                                let enabled = evt.value().parse::<bool>().unwrap_or(false);
                                app_state.write().settings.hall_sensor.enable_interpolation = enabled;
                                send_command(app_state, MotorCommand::AngleInterpolation(enabled));
                            },
                        }
                        "Enable Angle Interpolation"
                    }
                    p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                        "Interpolate angle between Hall sensor transitions for smoother control. Default: {defaults::ENABLE_ANGLE_INTERPOLATION}"
                    }
                }
            }
        }
    }
}

// ============================================================================
// OpenLoop Tab
// ============================================================================

#[component]
fn OpenLoopTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            SectionHeader { title: "OpenLoop Startup Parameters".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Configure openloop ramp-up for motor startup." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                F32Input {
                    label: "Initial RPM".to_string(),
                    value: app_state.read().settings.openloop.initial_rpm,
                    step: "10.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.openloop.initial_rpm = v;
                        let target = app_state.read().settings.openloop.target_rpm;
                        send_command(app_state, MotorCommand::OpenLoopRpm { initial_rpm: v, target_rpm: target });
                    },
                    is_connected,
                    description: format!("Starting RPM for openloop ramp-up. Default: {}", defaults::OPENLOOP_INITIAL_RPM)
                }

                F32Input {
                    label: "Target RPM".to_string(),
                    value: app_state.read().settings.openloop.target_rpm,
                    step: "10.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.openloop.target_rpm = v;
                        let initial = app_state.read().settings.openloop.initial_rpm;
                        send_command(app_state, MotorCommand::OpenLoopRpm { initial_rpm: initial, target_rpm: v });
                    },
                    is_connected,
                    description: format!("Target RPM to switch to FOC control. Default: {}", defaults::OPENLOOP_TARGET_RPM)
                }

                F32Input {
                    label: "Acceleration (RPM/s)".to_string(),
                    value: app_state.read().settings.openloop.acceleration,
                    step: "10.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.openloop.acceleration = v;
                        let duty = app_state.read().settings.openloop.duty_ratio;
                        send_command(app_state, MotorCommand::OpenLoopAccelDuty { acceleration: v, duty_ratio: duty });
                    },
                    is_connected,
                    description: format!("Ramp-up acceleration rate. Default: {}", defaults::OPENLOOP_ACCELERATION)
                }

                U16Input {
                    label: "Duty Ratio (0-100)".to_string(),
                    value: app_state.read().settings.openloop.duty_ratio,
                    on_change: move |v| {
                        app_state.write().settings.openloop.duty_ratio = v;
                        let accel = app_state.read().settings.openloop.acceleration;
                        send_command(app_state, MotorCommand::OpenLoopAccelDuty { acceleration: accel, duty_ratio: v });
                    },
                    is_connected,
                    description: format!("PWM duty ratio during openloop. Default: {}", defaults::OPENLOOP_DUTY_RATIO)
                }

                U32Input {
                    label: "Forced Commutation Cycles".to_string(),
                    value: app_state.read().settings.openloop.forced_commutation_cycles,
                    on_change: move |v| {
                        app_state.write().settings.openloop.forced_commutation_cycles = v;
                        let min = app_state.read().settings.openloop.min_cycles_before_foc;
                        send_command(app_state, MotorCommand::OpenLoopCycles { forced_cycles: v, min_cycles: min });
                    },
                    is_connected,
                    description: format!("Number of forced commutation cycles at startup. Default: {}", defaults::FORCED_COMMUTATION_CYCLES)
                }

                U32Input {
                    label: "Min Cycles Before FOC".to_string(),
                    value: app_state.read().settings.openloop.min_cycles_before_foc,
                    on_change: move |v| {
                        app_state.write().settings.openloop.min_cycles_before_foc = v;
                        let forced = app_state.read().settings.openloop.forced_commutation_cycles;
                        send_command(app_state, MotorCommand::OpenLoopCycles { forced_cycles: forced, min_cycles: v });
                    },
                    is_connected,
                    description: format!("Minimum cycles before switching to FOC. Default: {}", defaults::MIN_CYCLES_BEFORE_FOC)
                }
            }
        }
    }
}

// ============================================================================
// FOC Advanced Tab
// ============================================================================

#[component]
fn FocAdvancedTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            SectionHeader { title: "FOC Advanced Parameters".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Configure advance angle, minimum voltage, and stall detection." }

            // Advance Angle Section
            div { style: "margin-bottom: 30px;",
                h4 { style: "color: #444; margin-bottom: 15px; border-bottom: 1px solid #eee; padding-bottom: 5px;", "Advance Angle" }
                div { style: "display: grid; gap: 15px;",
                    F32Input {
                        label: "Base Advance (deg)".to_string(),
                        value: app_state.read().settings.advance_angle.base_deg,
                        step: "1.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.advance_angle.base_deg = v;
                            let max = app_state.read().settings.advance_angle.max_deg;
                            send_command(app_state, MotorCommand::AdvanceAngle { base_deg: v, max_deg: max });
                        },
                        is_connected,
                        description: format!("Base advance angle at low speed. Default: {} deg", defaults::ADVANCE_BASE_DEG)
                    }

                    F32Input {
                        label: "Max Advance (deg)".to_string(),
                        value: app_state.read().settings.advance_angle.max_deg,
                        step: "1.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.advance_angle.max_deg = v;
                            let base = app_state.read().settings.advance_angle.base_deg;
                            send_command(app_state, MotorCommand::AdvanceAngle { base_deg: base, max_deg: v });
                        },
                        is_connected,
                        description: format!("Maximum advance angle at high speed. Default: {} deg", defaults::ADVANCE_MAX_DEG)
                    }

                    F32Input {
                        label: "Min Speed for Advance (RPM)".to_string(),
                        value: app_state.read().settings.advance_angle.min_speed,
                        step: "10.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.advance_angle.min_speed = v;
                            let max = app_state.read().settings.advance_angle.max_speed;
                            send_command(app_state, MotorCommand::AdvanceAngleSpeed { min_speed: v, max_speed: max });
                        },
                        is_connected,
                        description: format!("Speed below which base advance is used. Default: {} RPM", defaults::ADVANCE_MIN_SPEED)
                    }

                    F32Input {
                        label: "Max Speed for Advance (RPM)".to_string(),
                        value: app_state.read().settings.advance_angle.max_speed,
                        step: "100.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.advance_angle.max_speed = v;
                            let min = app_state.read().settings.advance_angle.min_speed;
                            send_command(app_state, MotorCommand::AdvanceAngleSpeed { min_speed: min, max_speed: v });
                        },
                        is_connected,
                        description: format!("Speed above which max advance is used. Default: {} RPM", defaults::ADVANCE_MAX_SPEED)
                    }
                }
            }

            // Minimum Voltage Section
            div { style: "margin-bottom: 30px;",
                h4 { style: "color: #444; margin-bottom: 15px; border-bottom: 1px solid #eee; padding-bottom: 5px;", "Minimum Voltage" }
                div { style: "display: grid; gap: 15px;",
                    F32Input {
                        label: "Min Voltage (V)".to_string(),
                        value: app_state.read().settings.min_voltage.voltage,
                        step: "0.1".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.min_voltage.voltage = v;
                            let threshold = app_state.read().settings.min_voltage.error_threshold;
                            send_command(app_state, MotorCommand::MinVoltage { min_voltage: v, error_threshold: threshold });
                        },
                        is_connected,
                        description: format!("Minimum output voltage. Default: {}V", defaults::MIN_VOLTAGE)
                    }

                    F32Input {
                        label: "Error Threshold (RPM)".to_string(),
                        value: app_state.read().settings.min_voltage.error_threshold,
                        step: "0.1".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.min_voltage.error_threshold = v;
                            let min_v = app_state.read().settings.min_voltage.voltage;
                            send_command(app_state, MotorCommand::MinVoltage { min_voltage: min_v, error_threshold: v });
                        },
                        is_connected,
                        description: format!("Speed error threshold for min voltage. Default: {}", defaults::MIN_VOLTAGE_ERROR_THRESHOLD)
                    }

                    F32Input {
                        label: "Max Speed Acceleration (RPM/s)".to_string(),
                        value: app_state.read().settings.min_voltage.max_speed_acceleration,
                        step: "10.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.min_voltage.max_speed_acceleration = v;
                            send_command(app_state, MotorCommand::MaxSpeedAccel(v));
                        },
                        is_connected,
                        description: format!("Max speed command acceleration. Default: {} RPM/s", defaults::MAX_SPEED_ACCELERATION)
                    }
                }
            }

            // FOC Stall Detection Section
            div {
                h4 { style: "color: #444; margin-bottom: 15px; border-bottom: 1px solid #eee; padding-bottom: 5px;", "FOC Stall Detection" }
                div { style: "display: grid; gap: 15px;",
                    F32Input {
                        label: "Stall Speed Threshold (RPM)".to_string(),
                        value: app_state.read().settings.foc_stall.speed_threshold,
                        step: "5.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.foc_stall.speed_threshold = v;
                            let count = app_state.read().settings.foc_stall.count_threshold;
                            send_command(app_state, MotorCommand::FocStall { speed_threshold: v, count_threshold: count });
                        },
                        is_connected,
                        description: format!("Speed threshold for stall detection. Default: {} RPM", defaults::FOC_STALL_SPEED_THRESHOLD)
                    }

                    U32Input {
                        label: "Stall Count Threshold".to_string(),
                        value: app_state.read().settings.foc_stall.count_threshold,
                        on_change: move |v| {
                            app_state.write().settings.foc_stall.count_threshold = v;
                            let speed = app_state.read().settings.foc_stall.speed_threshold;
                            send_command(app_state, MotorCommand::FocStall { speed_threshold: speed, count_threshold: v });
                        },
                        is_connected,
                        description: format!("Consecutive low-speed cycles for stall. Default: {}", defaults::FOC_STALL_COUNT_THRESHOLD)
                    }
                }
            }
        }
    }
}

// ============================================================================
// Compensation Tab
// ============================================================================

#[component]
fn CompensationTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            SectionHeader { title: "Compensation Settings".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Configure dead time compensation and flux weakening control." }

            // Dead Time Compensation Section
            div { style: "margin-bottom: 30px;",
                h4 { style: "color: #444; margin-bottom: 15px; border-bottom: 1px solid #eee; padding-bottom: 5px;", "Dead Time Compensation" }
                div { style: "display: grid; gap: 15px;",
                    div {
                        label { style: "font-size: 14px; font-weight: 500; color: #555; display: flex; align-items: center; gap: 10px;",
                            input {
                                r#type: "checkbox",
                                checked: app_state.read().settings.dead_time_comp.enabled,
                                disabled: !is_connected,
                                onchange: move |evt| {
                                    let enabled = evt.value().parse::<bool>().unwrap_or(false);
                                    app_state.write().settings.dead_time_comp.enabled = enabled;
                                    let dt = app_state.read().settings.dead_time_comp.dead_time_ns;
                                    send_command(app_state, MotorCommand::DeadTimeComp { enabled, dead_time_ns: dt });
                                },
                            }
                            "Enable Dead Time Compensation"
                        }
                        p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                            "Compensate for PWM dead time effects. Default: {defaults::DEAD_TIME_COMP_ENABLED}"
                        }
                    }

                    F32Input {
                        label: "Dead Time (ns)".to_string(),
                        value: app_state.read().settings.dead_time_comp.dead_time_ns,
                        step: "10.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.dead_time_comp.dead_time_ns = v;
                            let enabled = app_state.read().settings.dead_time_comp.enabled;
                            send_command(app_state, MotorCommand::DeadTimeComp { enabled, dead_time_ns: v });
                        },
                        is_connected,
                        description: format!("Dead time to compensate in nanoseconds. Default: {} ns", defaults::DEAD_TIME_NS)
                    }
                }
            }

            // Flux Weakening Section
            div {
                h4 { style: "color: #444; margin-bottom: 15px; border-bottom: 1px solid #eee; padding-bottom: 5px;", "Flux Weakening Control" }
                div { style: "display: grid; gap: 15px;",
                    div {
                        label { style: "font-size: 14px; font-weight: 500; color: #555; display: flex; align-items: center; gap: 10px;",
                            input {
                                r#type: "checkbox",
                                checked: app_state.read().settings.flux_weakening.enabled,
                                disabled: !is_connected,
                                onchange: move |evt| {
                                    let enabled = evt.value().parse::<bool>().unwrap_or(false);
                                    app_state.write().settings.flux_weakening.enabled = enabled;
                                    let min_speed = app_state.read().settings.flux_weakening.min_speed;
                                    send_command(app_state, MotorCommand::FluxWeakeningEnable { enabled, min_speed });
                                },
                            }
                            "Enable Flux Weakening"
                        }
                        p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                            "Enable field weakening for higher speeds. Default: {defaults::FLUX_WEAKENING_ENABLED}"
                        }
                    }

                    F32Input {
                        label: "Min Speed (RPM)".to_string(),
                        value: app_state.read().settings.flux_weakening.min_speed,
                        step: "100.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.flux_weakening.min_speed = v;
                            let enabled = app_state.read().settings.flux_weakening.enabled;
                            send_command(app_state, MotorCommand::FluxWeakeningEnable { enabled, min_speed: v });
                        },
                        is_connected,
                        description: format!("Speed at which flux weakening starts. Default: {} RPM", defaults::FLUX_WEAKENING_MIN_SPEED)
                    }

                    F32Input {
                        label: "Max Speed (RPM)".to_string(),
                        value: app_state.read().settings.flux_weakening.max_speed,
                        step: "100.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.flux_weakening.max_speed = v;
                            let ratio = app_state.read().settings.flux_weakening.max_ratio;
                            send_command(app_state, MotorCommand::FluxWeakeningParams { max_speed: v, max_ratio: ratio });
                        },
                        is_connected,
                        description: format!("Speed at which max weakening is reached. Default: {} RPM", defaults::FLUX_WEAKENING_MAX_SPEED)
                    }

                    F32Input {
                        label: "Max Weakening Ratio".to_string(),
                        value: app_state.read().settings.flux_weakening.max_ratio,
                        step: "0.05".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.flux_weakening.max_ratio = v;
                            let max_speed = app_state.read().settings.flux_weakening.max_speed;
                            send_command(app_state, MotorCommand::FluxWeakeningParams { max_speed, max_ratio: v });
                        },
                        is_connected,
                        description: format!("Maximum flux weakening ratio (0-1). Default: {}", defaults::FLUX_WEAKENING_MAX_RATIO)
                    }

                    F32Input {
                        label: "Vd Rate Limit (V/s)".to_string(),
                        value: app_state.read().settings.flux_weakening.vd_rate_limit,
                        step: "10.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.flux_weakening.vd_rate_limit = v;
                            send_command(app_state, MotorCommand::FluxWeakeningVd(v));
                        },
                        is_connected,
                        description: format!("Rate limit for Vd changes. Default: {} V/s", defaults::FLUX_WEAKENING_VD_RATE_LIMIT)
                    }
                }
            }
        }
    }
}

// ============================================================================
// Voltage Monitor Tab
// ============================================================================

#[component]
fn VoltageMonitorTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            SectionHeader { title: "Voltage Monitor Settings".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Configure voltage monitoring thresholds and filter settings." }

            div { style: "display: grid; gap: 15px;",
                F32Input {
                    label: "Overvoltage Threshold (V)".to_string(),
                    value: app_state.read().settings.voltage_monitor.overvoltage_threshold,
                    step: "1.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.voltage_monitor.overvoltage_threshold = v;
                        let under = app_state.read().settings.voltage_monitor.undervoltage_threshold;
                        send_command(app_state, MotorCommand::VoltageMonitorThresholds { overvoltage: v, undervoltage: under });
                    },
                    is_connected,
                    description: format!("Voltage above which overvoltage is triggered. Default: {}V", defaults::VOLTAGE_OVERVOLTAGE_THRESHOLD)
                }

                F32Input {
                    label: "Undervoltage Threshold (V)".to_string(),
                    value: app_state.read().settings.voltage_monitor.undervoltage_threshold,
                    step: "1.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.voltage_monitor.undervoltage_threshold = v;
                        let over = app_state.read().settings.voltage_monitor.overvoltage_threshold;
                        send_command(app_state, MotorCommand::VoltageMonitorThresholds { overvoltage: over, undervoltage: v });
                    },
                    is_connected,
                    description: format!("Voltage below which undervoltage is triggered. Default: {}V", defaults::VOLTAGE_UNDERVOLTAGE_THRESHOLD)
                }

                F32Input {
                    label: "Filter Alpha".to_string(),
                    value: app_state.read().settings.voltage_monitor.filter_alpha,
                    step: "0.01".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.voltage_monitor.filter_alpha = v;
                        send_command(app_state, MotorCommand::VoltageMonitorFilter(v));
                    },
                    is_connected,
                    description: format!("Low-pass filter coefficient (0-1). Higher = faster response. Default: {}", defaults::VOLTAGE_FILTER_ALPHA)
                }
            }
        }
    }
}

// ============================================================================
// Advanced Tab
// ============================================================================

#[component]
fn AdvancedTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            ErrorBanner {
                message: "Warning: These settings require device reboot to take effect. Incorrect values may prevent the device from operating.".to_string()
            }

            SectionHeader { title: "Advanced Configuration".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Low-level hardware configuration. Change with caution." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                U32Input {
                    label: "PWM Frequency (Hz)".to_string(),
                    value: app_state.read().settings.hardware.pwm_frequency,
                    on_change: move |v| {
                        app_state.write().settings.hardware.pwm_frequency = v;
                        let dead_time = app_state.read().settings.hardware.pwm_dead_time;
                        send_command(app_state, MotorCommand::PwmConfig { frequency: v, dead_time });
                    },
                    is_connected,
                    description: format!("PWM switching frequency. Default: {} Hz. Requires reboot", defaults::PWM_FREQUENCY)
                }

                U16Input {
                    label: "PWM Dead Time".to_string(),
                    value: app_state.read().settings.hardware.pwm_dead_time,
                    on_change: move |v| {
                        app_state.write().settings.hardware.pwm_dead_time = v;
                        let freq = app_state.read().settings.hardware.pwm_frequency;
                        send_command(app_state, MotorCommand::PwmConfig { frequency: freq, dead_time: v });
                    },
                    is_connected,
                    description: format!("Dead time for complementary PWM. Default: {}. Requires reboot", defaults::PWM_DEAD_TIME)
                }

                U32Input {
                    label: "CAN Bitrate (bps)".to_string(),
                    value: app_state.read().settings.hardware.can_bitrate,
                    on_change: move |v| {
                        app_state.write().settings.hardware.can_bitrate = v;
                        send_command(app_state, MotorCommand::CanConfig(v));
                    },
                    is_connected,
                    description: format!("CAN bus bitrate. Default: {} bps. Requires reboot", defaults::CAN_BITRATE)
                }

                U64Input {
                    label: "Control Period (us)".to_string(),
                    value: app_state.read().settings.hardware.control_period_us,
                    on_change: move |v| {
                        app_state.write().settings.hardware.control_period_us = v;
                        send_command(app_state, MotorCommand::ControlTiming(v));
                    },
                    is_connected,
                    description: format!("FOC control loop period. Default: {} us. Requires reboot", defaults::CONTROL_PERIOD_US)
                }
            }
        }
    }
}

// ============================================================================
// Config Management Section
// ============================================================================

#[component]
fn ConfigManagementSection(is_connected: bool) -> Element {
    let app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    let on_save_config = move |_| {
        info!("Saving config to flash");
        send_command(app_state, MotorCommand::SaveConfig);
    };

    let on_reload_config = move |_| {
        info!("Reloading config from flash");
        send_command(app_state, MotorCommand::ReloadConfig);
    };

    let on_reset_config = move |_| {
        info!("Resetting config to defaults");
        send_command(app_state, MotorCommand::ResetConfig);
    };

    rsx! {
        Card {
            SectionHeader {
                title: "Configuration Management".to_string(),
                color: HeaderColor::Green
            }

            div { style: "display: flex; flex-direction: column; gap: 15px;",
                Banner {
                    banner_type: BannerType::Success,
                    message: "Save current settings to flash memory for persistence across power cycles.".to_string()
                }

                div { style: "display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px;",
                    StatusCard {
                        label: "Config Version".to_string(),
                        value: format!("{}", state.config_version),
                        color: StatusCardColor::Green
                    }

                    StatusCard {
                        label: "CRC Status".to_string(),
                        value: if state.config_crc_valid { "Valid".to_string() } else { "Invalid".to_string() },
                        color: if state.config_crc_valid { StatusCardColor::Green } else { StatusCardColor::Red }
                    }
                }

                div { style: "display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px;",
                    Button {
                        variant: ButtonVariant::Success,
                        disabled: !is_connected,
                        onclick: on_save_config,
                        "Save to Flash"
                    }

                    Button {
                        variant: ButtonVariant::Outline,
                        disabled: !is_connected,
                        onclick: on_reload_config,
                        "Reload from Flash"
                    }

                    Button {
                        variant: ButtonVariant::Danger,
                        disabled: !is_connected,
                        custom_style: "border: 1px solid #dc3545; background: white; color: #dc3545;".to_string(),
                        onclick: on_reset_config,
                        "Reset to Defaults"
                    }
                }
            }
        }
    }
}

// ============================================================================
// Calibration Tab
// ============================================================================

#[component]
fn CalibrationTab(is_connected: bool) -> Element {
    let app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    let mut calibration_torque = use_signal(|| 30u8);
    let mut is_calibrating = use_signal(|| false);

    let on_start_calibration = move |_| {
        let torque = calibration_torque();
        info!("Starting calibration with torque: {}", torque);
        is_calibrating.set(true);

        send_command(
            app_state,
            MotorCommand::StartCalibration {
                torque: Some(torque),
            },
        );

        spawn(async move {
            tokio::time::sleep(tokio::time::Duration::from_secs(10)).await;
            is_calibrating.set(false);
        });
    };

    rsx! {
        Card {
            SectionHeader { title: "Motor Calibration".to_string(), color: HeaderColor::Orange }

            Banner {
                banner_type: BannerType::Warning,
                message: "WARNING: Ensure the motor can spin freely during calibration. Keep clear of moving parts!".to_string()
            }

            div { style: "display: flex; flex-direction: column; gap: 20px; margin-top: 20px;",
                p { style: "color: #666; line-height: 1.6;",
                    "Motor calibration determines the electrical angle offset and rotation direction. "
                    "The motor will spin slowly during calibration to detect Hall sensor alignment."
                }

                div { style: "display: flex; flex-direction: column; gap: 15px; padding: 20px; background: #f8f9fa; border-radius: 8px;",
                    U8Input {
                        label: "Calibration Torque (0-100)".to_string(),
                        value: calibration_torque(),
                        on_change: move |v: u8| calibration_torque.set(v.min(100)),
                        is_connected,
                        description: "Torque level during calibration. Lower values for light motors, higher for heavy loads. Default: 30".to_string()
                    }

                    Button {
                        variant: if is_calibrating() { ButtonVariant::Secondary } else { ButtonVariant::Warning },
                        disabled: !is_connected || is_calibrating(),
                        custom_style: "width: 100%; padding: 12px;".to_string(),
                        onclick: on_start_calibration,
                        if is_calibrating() {
                            "Calibrating... (please wait)"
                        } else {
                            "Start Calibration"
                        }
                    }
                }

                if let Some(cal_status) = state.calibration_status {
                    div { style: "display: flex; flex-direction: column; gap: 15px;",
                        SectionHeader {
                            title: "Calibration Results".to_string(),
                            color: if cal_status.success { HeaderColor::Green } else { HeaderColor::Red }
                        }

                        if cal_status.success {
                            Banner {
                                banner_type: BannerType::Success,
                                message: "Calibration completed successfully!".to_string()
                            }
                        } else {
                            ErrorBanner {
                                message: "Calibration failed. Please try again.".to_string()
                            }
                        }

                        div { style: "display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px;",
                            StatusCard {
                                label: "Electrical Offset".to_string(),
                                value: format!("{:.4} rad ({:.1} deg)", cal_status.electrical_offset, cal_status.electrical_offset * 180.0 / std::f32::consts::PI),
                                color: StatusCardColor::Blue
                            }

                            StatusCard {
                                label: "Direction".to_string(),
                                value: if cal_status.direction_inversed { "Inversed".to_string() } else { "Normal".to_string() },
                                color: if cal_status.direction_inversed { StatusCardColor::Orange } else { StatusCardColor::Green }
                            }

                            StatusCard {
                                label: "Status".to_string(),
                                value: if cal_status.success { "Success".to_string() } else { "Failed".to_string() },
                                color: if cal_status.success { StatusCardColor::Green } else { StatusCardColor::Red }
                            }
                        }

                        if cal_status.success {
                            Banner {
                                banner_type: BannerType::Info,
                                message: "Remember to save the configuration to flash memory to persist these calibration values!".to_string()
                            }
                        }
                    }
                }
            }
        }
    }
}
