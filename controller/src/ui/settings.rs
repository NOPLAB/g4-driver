use dioxus::prelude::*;
use tracing::{error, info};

use super::components::{
    Banner, BannerType, Button, ButtonVariant, Card, ErrorBanner, F32Input, HeaderColor,
    SectionHeader, StatusCard, StatusCardColor, U16Input, U32Input, U64Input, U8Input,
    WarningBanner,
};
use crate::state::{AppState, ConnectionState};

// Default values (from firmware config)
const DEFAULT_KP: f32 = 0.5;
const DEFAULT_KI: f32 = 0.05;
const DEFAULT_MAX_VOLTAGE: f32 = 24.0;
const DEFAULT_V_DC_BUS: f32 = 24.0;
const DEFAULT_POLE_PAIRS: u8 = 6;
const DEFAULT_MAX_DUTY: u16 = 100;
const DEFAULT_SPEED_FILTER_ALPHA: f32 = 0.1;
const DEFAULT_HALL_ANGLE_OFFSET: f32 = 0.0;
const DEFAULT_ENABLE_ANGLE_INTERPOLATION: bool = true;
const DEFAULT_OPENLOOP_INITIAL_RPM: f32 = 100.0;
const DEFAULT_OPENLOOP_TARGET_RPM: f32 = 500.0;
const DEFAULT_OPENLOOP_ACCELERATION: f32 = 100.0;
const DEFAULT_OPENLOOP_DUTY_RATIO: u16 = 50;
const DEFAULT_PWM_FREQUENCY: u32 = 50000;
const DEFAULT_PWM_DEAD_TIME: u16 = 100;
const DEFAULT_CAN_BITRATE: u32 = 250000;
const DEFAULT_CONTROL_PERIOD_US: u64 = 400;

// Extended defaults
const DEFAULT_ADVANCE_BASE_DEG: f32 = 10.0;
const DEFAULT_ADVANCE_MAX_DEG: f32 = 30.0;
const DEFAULT_ADVANCE_MIN_SPEED: f32 = 100.0;
const DEFAULT_ADVANCE_MAX_SPEED: f32 = 3000.0;
const DEFAULT_MIN_VOLTAGE: f32 = 2.0;
const DEFAULT_MIN_VOLTAGE_ERROR_THRESHOLD: f32 = 2.0;
const DEFAULT_MAX_SPEED_ACCELERATION: f32 = 100.0;
const DEFAULT_FOC_STALL_SPEED_THRESHOLD: f32 = 50.0;
const DEFAULT_FOC_STALL_COUNT_THRESHOLD: u32 = 1000;
const DEFAULT_FORCED_COMMUTATION_CYCLES: u32 = 10000;
const DEFAULT_MIN_CYCLES_BEFORE_FOC: u32 = 10000;
const DEFAULT_DEAD_TIME_COMP_ENABLED: bool = false;
const DEFAULT_DEAD_TIME_NS: f32 = 100.0;
const DEFAULT_FLUX_WEAKENING_ENABLED: bool = false;
const DEFAULT_FLUX_WEAKENING_MIN_SPEED: f32 = 2000.0;
const DEFAULT_FLUX_WEAKENING_MAX_SPEED: f32 = 4000.0;
const DEFAULT_FLUX_WEAKENING_MAX_RATIO: f32 = 0.5;
const DEFAULT_FLUX_WEAKENING_VD_RATE_LIMIT: f32 = 100.0;
const DEFAULT_VOLTAGE_OVERVOLTAGE_THRESHOLD: f32 = 30.0;
const DEFAULT_VOLTAGE_UNDERVOLTAGE_THRESHOLD: f32 = 10.0;
const DEFAULT_VOLTAGE_FILTER_ALPHA: f32 = 0.1;

#[component]
pub fn SettingsPanel() -> Element {
    let app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    let is_connected = matches!(state.connection_state, ConnectionState::Connected);

    // Tab selection state
    let mut selected_tab = use_signal(|| 0);

    rsx! {
        div {
            style: "display: flex; flex-direction: column; gap: 20px; max-width: 800px;",

            // Connection warning
            if !is_connected {
                WarningBanner {
                    message: "Not connected to CAN. Please connect first.".to_string()
                }
            }

            // Tab navigation
            div {
                style: "display: flex; gap: 5px; border-bottom: 2px solid #ddd; padding-bottom: 0;",

                button {
                    style: if selected_tab() == 0 {
                        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
                    } else {
                        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
                    },
                    onclick: move |_| selected_tab.set(0),
                    "PI Control"
                }

                button {
                    style: if selected_tab() == 1 {
                        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
                    } else {
                        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
                    },
                    onclick: move |_| selected_tab.set(1),
                    "Motor Control"
                }

                button {
                    style: if selected_tab() == 2 {
                        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
                    } else {
                        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
                    },
                    onclick: move |_| selected_tab.set(2),
                    "Hall Sensor"
                }

                button {
                    style: if selected_tab() == 3 {
                        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
                    } else {
                        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
                    },
                    onclick: move |_| selected_tab.set(3),
                    "OpenLoop"
                }

                button {
                    style: if selected_tab() == 4 {
                        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
                    } else {
                        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
                    },
                    onclick: move |_| selected_tab.set(4),
                    "FOC Advanced"
                }

                button {
                    style: if selected_tab() == 5 {
                        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
                    } else {
                        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
                    },
                    onclick: move |_| selected_tab.set(5),
                    "Compensation"
                }

                button {
                    style: if selected_tab() == 6 {
                        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
                    } else {
                        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
                    },
                    onclick: move |_| selected_tab.set(6),
                    "Voltage"
                }

                button {
                    style: if selected_tab() == 7 {
                        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
                    } else {
                        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
                    },
                    onclick: move |_| selected_tab.set(7),
                    "Advanced"
                }

                button {
                    style: if selected_tab() == 8 {
                        "padding: 12px 24px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500; border-bottom: 3px solid #007bff;"
                    } else {
                        "padding: 12px 24px; border: none; background: #f8f9fa; color: #333; cursor: pointer; border-radius: 8px 8px 0 0; font-size: 14px; font-weight: 500;"
                    },
                    onclick: move |_| selected_tab.set(8),
                    "Calibration"
                }
            }

            // Tab content
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

            // Config Management Section (always visible)
            ConfigManagementSection { is_connected }
        }
    }
}

#[component]
fn PIControlTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    let on_kp_change = move |value: f32| {
        app_state.write().settings.kp = value;
    };

    let on_ki_change = move |value: f32| {
        app_state.write().settings.ki = value;
    };

    let on_apply = move |_| {
        let kp = app_state.read().settings.kp;
        let ki = app_state.read().settings.ki;
        info!("Applying PI gains: Kp={}, Ki={}", kp, ki);

        spawn(async move {
            let manager = app_state.read().can_manager.clone();
            match manager.lock().await.send_pi_gains(kp, ki).await {
                Ok(_) => info!("PI gains sent successfully"),
                Err(e) => error!("Failed to send PI gains: {}", e),
            };
        });
    };

    let on_reset = move |_| {
        app_state.write().settings.kp = DEFAULT_KP;
        app_state.write().settings.ki = DEFAULT_KI;
    };

    let state = app_state.read();

    rsx! {
        Card {
            SectionHeader { title: "PI Controller Settings".to_string() }

            Banner {
                banner_type: BannerType::Info,
                message: "Configure the PI controller gains for speed control. These values affect the motor's response to speed commands.".to_string()
            }

            div { style: "display: grid; gap: 20px;",
                // Kp input
                F32Input {
                    label: "Proportional Gain (Kp)".to_string(),
                    value: state.settings.kp,
                    step: "0.01".to_string(),
                    on_change: on_kp_change,
                    is_connected,
                    description: format!("Higher values provide faster response but may cause oscillations. Default: {}", DEFAULT_KP)
                }

                // Ki input
                F32Input {
                    label: "Integral Gain (Ki)".to_string(),
                    value: state.settings.ki,
                    step: "0.001".to_string(),
                    on_change: on_ki_change,
                    is_connected,
                    description: format!("Helps eliminate steady-state error. Too high may cause instability. Default: {}", DEFAULT_KI)
                }

                // Action buttons
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

#[component]
fn MotorControlTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            SectionHeader { title: "Motor Control Parameters".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Configure motor voltage, pole pairs, and duty cycle parameters." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                // Max Voltage
                F32Input {
                    label: "Max Voltage (V)".to_string(),
                    value: app_state.read().settings.max_voltage,
                    step: "0.1".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.max_voltage = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let vdc = app_state.read().settings.v_dc_bus;
                            let _ = mgr.lock().await.send_motor_voltage_params(val, vdc).await;
                        });
                    },
                    is_connected,
                    description: format!("Maximum voltage limit for motor control. Default: {}", DEFAULT_MAX_VOLTAGE)
                }

                // DC Bus Voltage
                F32Input {
                    label: "DC Bus Voltage (V)".to_string(),
                    value: app_state.read().settings.v_dc_bus,
                    step: "0.1".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.v_dc_bus = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let max_v = app_state.read().settings.max_voltage;
                            let _ = mgr.lock().await.send_motor_voltage_params(max_v, val).await;
                        });
                    },
                    is_connected,
                    description: format!("DC bus voltage for calculations. Default: {}", DEFAULT_V_DC_BUS)
                }

                // Pole Pairs
                U8Input {
                    label: "Pole Pairs".to_string(),
                    value: app_state.read().settings.pole_pairs,
                    on_change: move |v| {
                        app_state.write().settings.pole_pairs = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let duty = app_state.read().settings.max_duty;
                            let _ = mgr.lock().await.send_motor_basic_params(val, duty).await;
                        });
                    },
                    is_connected,
                    description: format!("Number of motor pole pairs (poles/2). Default: {}", DEFAULT_POLE_PAIRS)
                }

                // Max Duty
                U16Input {
                    label: "Max Duty Cycle".to_string(),
                    value: app_state.read().settings.max_duty,
                    on_change: move |v| {
                        app_state.write().settings.max_duty = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let poles = app_state.read().settings.pole_pairs;
                            let _ = mgr.lock().await.send_motor_basic_params(poles, val).await;
                        });
                    },
                    is_connected,
                    description: format!("Maximum PWM duty cycle (0-100). Default: {}", DEFAULT_MAX_DUTY)
                }
            }
        }
    }
}

#[component]
fn HallSensorTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            SectionHeader { title: "Hall Sensor Parameters".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Configure Hall sensor filter and angle offset." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                // Speed Filter Alpha
                F32Input {
                    label: "Speed Filter Alpha".to_string(),
                    value: app_state.read().settings.speed_filter_alpha,
                    step: "0.01".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.speed_filter_alpha = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let offset = app_state.read().settings.hall_angle_offset;
                            let _ = mgr.lock().await.send_hall_sensor_params(val, offset).await;
                        });
                    },
                    is_connected,
                    description: format!("Low-pass filter coefficient for speed (0-1). Default: {}", DEFAULT_SPEED_FILTER_ALPHA)
                }

                // Hall Angle Offset
                F32Input {
                    label: "Hall Angle Offset (rad)".to_string(),
                    value: app_state.read().settings.hall_angle_offset,
                    step: "0.01".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.hall_angle_offset = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let alpha = app_state.read().settings.speed_filter_alpha;
                            let _ = mgr.lock().await.send_hall_sensor_params(alpha, val).await;
                        });
                    },
                    is_connected,
                    description: format!("Angle offset for Hall sensor alignment. Default: {}", DEFAULT_HALL_ANGLE_OFFSET)
                }

                // Angle Interpolation
                div {
                    label { style: "font-size: 14px; font-weight: 500; color: #555; display: flex; align-items: center; gap: 10px;",
                        input {
                            r#type: "checkbox",
                            checked: app_state.read().settings.enable_angle_interpolation,
                            disabled: !is_connected,
                            onchange: move |evt| {
                                let enabled = evt.value().parse::<bool>().unwrap_or(false);
                                app_state.write().settings.enable_angle_interpolation = enabled;
                                spawn(async move {
                                    let mgr = app_state.read().can_manager.clone();
                                    let _ = mgr.lock().await.send_angle_interpolation(enabled).await;
                                });
                            },
                        }
                        "Enable Angle Interpolation"
                    }
                    p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                        "Interpolate angle between Hall sensor transitions for smoother control. Default: {DEFAULT_ENABLE_ANGLE_INTERPOLATION}"
                    }
                }
            }
        }
    }
}

#[component]
fn OpenLoopTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        Card {
            SectionHeader { title: "OpenLoop Startup Parameters".to_string() }
            p { style: "color: #666; margin: 10px 0 20px 0;", "Configure openloop ramp-up for motor startup." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                // Initial RPM
                F32Input {
                    label: "Initial RPM".to_string(),
                    value: app_state.read().settings.openloop_initial_rpm,
                    step: "10.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.openloop_initial_rpm = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let target = app_state.read().settings.openloop_target_rpm;
                            let _ = mgr.lock().await.send_openloop_rpm_params(val, target).await;
                        });
                    },
                    is_connected,
                    description: format!("Starting RPM for openloop ramp-up. Default: {}", DEFAULT_OPENLOOP_INITIAL_RPM)
                }

                // Target RPM
                F32Input {
                    label: "Target RPM".to_string(),
                    value: app_state.read().settings.openloop_target_rpm,
                    step: "10.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.openloop_target_rpm = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let initial = app_state.read().settings.openloop_initial_rpm;
                            let _ = mgr.lock().await.send_openloop_rpm_params(initial, val).await;
                        });
                    },
                    is_connected,
                    description: format!("Target RPM to switch to FOC control. Default: {}", DEFAULT_OPENLOOP_TARGET_RPM)
                }

                // Acceleration
                F32Input {
                    label: "Acceleration (RPM/s)".to_string(),
                    value: app_state.read().settings.openloop_acceleration,
                    step: "10.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.openloop_acceleration = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let duty = app_state.read().settings.openloop_duty_ratio;
                            let _ = mgr.lock().await.send_openloop_accel_duty_params(val, duty).await;
                        });
                    },
                    is_connected,
                    description: format!("Ramp-up acceleration rate. Default: {}", DEFAULT_OPENLOOP_ACCELERATION)
                }

                // Duty Ratio
                U16Input {
                    label: "Duty Ratio (0-100)".to_string(),
                    value: app_state.read().settings.openloop_duty_ratio,
                    on_change: move |v| {
                        app_state.write().settings.openloop_duty_ratio = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let accel = app_state.read().settings.openloop_acceleration;
                            let _ = mgr.lock().await.send_openloop_accel_duty_params(accel, val).await;
                        });
                    },
                    is_connected,
                    description: format!("PWM duty ratio during openloop. Default: {}", DEFAULT_OPENLOOP_DUTY_RATIO)
                }

                // Forced Commutation Cycles
                U32Input {
                    label: "Forced Commutation Cycles".to_string(),
                    value: app_state.read().settings.forced_commutation_cycles,
                    on_change: move |v| {
                        app_state.write().settings.forced_commutation_cycles = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let min = app_state.read().settings.min_cycles_before_foc;
                            let _ = mgr.lock().await.send_openloop_cycles_params(val, min).await;
                        });
                    },
                    is_connected,
                    description: format!("Number of forced commutation cycles at startup. Default: {}", DEFAULT_FORCED_COMMUTATION_CYCLES)
                }

                // Min Cycles Before FOC
                U32Input {
                    label: "Min Cycles Before FOC".to_string(),
                    value: app_state.read().settings.min_cycles_before_foc,
                    on_change: move |v| {
                        app_state.write().settings.min_cycles_before_foc = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let forced = app_state.read().settings.forced_commutation_cycles;
                            let _ = mgr.lock().await.send_openloop_cycles_params(forced, val).await;
                        });
                    },
                    is_connected,
                    description: format!("Minimum cycles before switching to FOC. Default: {}", DEFAULT_MIN_CYCLES_BEFORE_FOC)
                }
            }
        }
    }
}

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
                        value: app_state.read().settings.advance_base_deg,
                        step: "1.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.advance_base_deg = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let max = app_state.read().settings.advance_max_deg;
                                let _ = mgr.lock().await.send_advance_angle_params(val, max).await;
                            });
                        },
                        is_connected,
                        description: format!("Base advance angle at low speed. Default: {}°", DEFAULT_ADVANCE_BASE_DEG)
                    }

                    F32Input {
                        label: "Max Advance (deg)".to_string(),
                        value: app_state.read().settings.advance_max_deg,
                        step: "1.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.advance_max_deg = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let base = app_state.read().settings.advance_base_deg;
                                let _ = mgr.lock().await.send_advance_angle_params(base, val).await;
                            });
                        },
                        is_connected,
                        description: format!("Maximum advance angle at high speed. Default: {}°", DEFAULT_ADVANCE_MAX_DEG)
                    }

                    F32Input {
                        label: "Min Speed for Advance (RPM)".to_string(),
                        value: app_state.read().settings.advance_min_speed,
                        step: "10.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.advance_min_speed = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let max = app_state.read().settings.advance_max_speed;
                                let _ = mgr.lock().await.send_advance_angle_speed(val, max).await;
                            });
                        },
                        is_connected,
                        description: format!("Speed below which base advance is used. Default: {} RPM", DEFAULT_ADVANCE_MIN_SPEED)
                    }

                    F32Input {
                        label: "Max Speed for Advance (RPM)".to_string(),
                        value: app_state.read().settings.advance_max_speed,
                        step: "100.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.advance_max_speed = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let min = app_state.read().settings.advance_min_speed;
                                let _ = mgr.lock().await.send_advance_angle_speed(min, val).await;
                            });
                        },
                        is_connected,
                        description: format!("Speed above which max advance is used. Default: {} RPM", DEFAULT_ADVANCE_MAX_SPEED)
                    }
                }
            }

            // Minimum Voltage Section
            div { style: "margin-bottom: 30px;",
                h4 { style: "color: #444; margin-bottom: 15px; border-bottom: 1px solid #eee; padding-bottom: 5px;", "Minimum Voltage" }
                div { style: "display: grid; gap: 15px;",
                    F32Input {
                        label: "Min Voltage (V)".to_string(),
                        value: app_state.read().settings.min_voltage,
                        step: "0.1".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.min_voltage = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let threshold = app_state.read().settings.min_voltage_error_threshold;
                                let _ = mgr.lock().await.send_min_voltage_params(val, threshold).await;
                            });
                        },
                        is_connected,
                        description: format!("Minimum output voltage. Default: {}V", DEFAULT_MIN_VOLTAGE)
                    }

                    F32Input {
                        label: "Error Threshold (RPM)".to_string(),
                        value: app_state.read().settings.min_voltage_error_threshold,
                        step: "0.1".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.min_voltage_error_threshold = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let min_v = app_state.read().settings.min_voltage;
                                let _ = mgr.lock().await.send_min_voltage_params(min_v, val).await;
                            });
                        },
                        is_connected,
                        description: format!("Speed error threshold for min voltage. Default: {}", DEFAULT_MIN_VOLTAGE_ERROR_THRESHOLD)
                    }

                    F32Input {
                        label: "Max Speed Acceleration (RPM/s)".to_string(),
                        value: app_state.read().settings.max_speed_acceleration,
                        step: "10.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.max_speed_acceleration = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let _ = mgr.lock().await.send_max_speed_accel(val).await;
                            });
                        },
                        is_connected,
                        description: format!("Max speed command acceleration. Default: {} RPM/s", DEFAULT_MAX_SPEED_ACCELERATION)
                    }
                }
            }

            // FOC Stall Detection Section
            div {
                h4 { style: "color: #444; margin-bottom: 15px; border-bottom: 1px solid #eee; padding-bottom: 5px;", "FOC Stall Detection" }
                div { style: "display: grid; gap: 15px;",
                    F32Input {
                        label: "Stall Speed Threshold (RPM)".to_string(),
                        value: app_state.read().settings.foc_stall_speed_threshold,
                        step: "5.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.foc_stall_speed_threshold = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let count = app_state.read().settings.foc_stall_count_threshold;
                                let _ = mgr.lock().await.send_foc_stall_params(val, count).await;
                            });
                        },
                        is_connected,
                        description: format!("Speed threshold for stall detection. Default: {} RPM", DEFAULT_FOC_STALL_SPEED_THRESHOLD)
                    }

                    U32Input {
                        label: "Stall Count Threshold".to_string(),
                        value: app_state.read().settings.foc_stall_count_threshold,
                        on_change: move |v| {
                            app_state.write().settings.foc_stall_count_threshold = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let speed = app_state.read().settings.foc_stall_speed_threshold;
                                let _ = mgr.lock().await.send_foc_stall_params(speed, val).await;
                            });
                        },
                        is_connected,
                        description: format!("Consecutive low-speed cycles for stall. Default: {}", DEFAULT_FOC_STALL_COUNT_THRESHOLD)
                    }
                }
            }
        }
    }
}

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
                                checked: app_state.read().settings.dead_time_comp_enabled,
                                disabled: !is_connected,
                                onchange: move |evt| {
                                    let enabled = evt.value().parse::<bool>().unwrap_or(false);
                                    app_state.write().settings.dead_time_comp_enabled = enabled;
                                    spawn(async move {
                                        let mgr = app_state.read().can_manager.clone();
                                        let dt = app_state.read().settings.dead_time_ns;
                                        let _ = mgr.lock().await.send_dead_time_comp_params(enabled, dt).await;
                                    });
                                },
                            }
                            "Enable Dead Time Compensation"
                        }
                        p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                            "Compensate for PWM dead time effects. Default: {DEFAULT_DEAD_TIME_COMP_ENABLED}"
                        }
                    }

                    F32Input {
                        label: "Dead Time (ns)".to_string(),
                        value: app_state.read().settings.dead_time_ns,
                        step: "10.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.dead_time_ns = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let enabled = app_state.read().settings.dead_time_comp_enabled;
                                let _ = mgr.lock().await.send_dead_time_comp_params(enabled, val).await;
                            });
                        },
                        is_connected,
                        description: format!("Dead time to compensate in nanoseconds. Default: {} ns", DEFAULT_DEAD_TIME_NS)
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
                                checked: app_state.read().settings.flux_weakening_enabled,
                                disabled: !is_connected,
                                onchange: move |evt| {
                                    let enabled = evt.value().parse::<bool>().unwrap_or(false);
                                    app_state.write().settings.flux_weakening_enabled = enabled;
                                    spawn(async move {
                                        let mgr = app_state.read().can_manager.clone();
                                        let min_speed = app_state.read().settings.flux_weakening_min_speed;
                                        let _ = mgr.lock().await.send_flux_weakening_enable(enabled, min_speed).await;
                                    });
                                },
                            }
                            "Enable Flux Weakening"
                        }
                        p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                            "Enable field weakening for higher speeds. Default: {DEFAULT_FLUX_WEAKENING_ENABLED}"
                        }
                    }

                    F32Input {
                        label: "Min Speed (RPM)".to_string(),
                        value: app_state.read().settings.flux_weakening_min_speed,
                        step: "100.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.flux_weakening_min_speed = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let enabled = app_state.read().settings.flux_weakening_enabled;
                                let _ = mgr.lock().await.send_flux_weakening_enable(enabled, val).await;
                            });
                        },
                        is_connected,
                        description: format!("Speed at which flux weakening starts. Default: {} RPM", DEFAULT_FLUX_WEAKENING_MIN_SPEED)
                    }

                    F32Input {
                        label: "Max Speed (RPM)".to_string(),
                        value: app_state.read().settings.flux_weakening_max_speed,
                        step: "100.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.flux_weakening_max_speed = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let ratio = app_state.read().settings.flux_weakening_max_ratio;
                                let _ = mgr.lock().await.send_flux_weakening_params(val, ratio).await;
                            });
                        },
                        is_connected,
                        description: format!("Speed at which max weakening is reached. Default: {} RPM", DEFAULT_FLUX_WEAKENING_MAX_SPEED)
                    }

                    F32Input {
                        label: "Max Weakening Ratio".to_string(),
                        value: app_state.read().settings.flux_weakening_max_ratio,
                        step: "0.05".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.flux_weakening_max_ratio = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let max_speed = app_state.read().settings.flux_weakening_max_speed;
                                let _ = mgr.lock().await.send_flux_weakening_params(max_speed, val).await;
                            });
                        },
                        is_connected,
                        description: format!("Maximum flux weakening ratio (0-1). Default: {}", DEFAULT_FLUX_WEAKENING_MAX_RATIO)
                    }

                    F32Input {
                        label: "Vd Rate Limit (V/s)".to_string(),
                        value: app_state.read().settings.flux_weakening_vd_rate_limit,
                        step: "10.0".to_string(),
                        on_change: move |v| {
                            app_state.write().settings.flux_weakening_vd_rate_limit = v;
                            let val = v;
                            spawn(async move {
                                let mgr = app_state.read().can_manager.clone();
                                let _ = mgr.lock().await.send_flux_weakening_vd(val).await;
                            });
                        },
                        is_connected,
                        description: format!("Rate limit for Vd changes. Default: {} V/s", DEFAULT_FLUX_WEAKENING_VD_RATE_LIMIT)
                    }
                }
            }
        }
    }
}

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
                    value: app_state.read().settings.voltage_overvoltage_threshold,
                    step: "1.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.voltage_overvoltage_threshold = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let under = app_state.read().settings.voltage_undervoltage_threshold;
                            let _ = mgr.lock().await.send_voltage_monitor_thresholds(val, under).await;
                        });
                    },
                    is_connected,
                    description: format!("Voltage above which overvoltage is triggered. Default: {}V", DEFAULT_VOLTAGE_OVERVOLTAGE_THRESHOLD)
                }

                F32Input {
                    label: "Undervoltage Threshold (V)".to_string(),
                    value: app_state.read().settings.voltage_undervoltage_threshold,
                    step: "1.0".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.voltage_undervoltage_threshold = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let over = app_state.read().settings.voltage_overvoltage_threshold;
                            let _ = mgr.lock().await.send_voltage_monitor_thresholds(over, val).await;
                        });
                    },
                    is_connected,
                    description: format!("Voltage below which undervoltage is triggered. Default: {}V", DEFAULT_VOLTAGE_UNDERVOLTAGE_THRESHOLD)
                }

                F32Input {
                    label: "Filter Alpha".to_string(),
                    value: app_state.read().settings.voltage_filter_alpha,
                    step: "0.01".to_string(),
                    on_change: move |v| {
                        app_state.write().settings.voltage_filter_alpha = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let _ = mgr.lock().await.send_voltage_monitor_filter(val).await;
                        });
                    },
                    is_connected,
                    description: format!("Low-pass filter coefficient (0-1). Higher = faster response. Default: {}", DEFAULT_VOLTAGE_FILTER_ALPHA)
                }
            }
        }
    }
}

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
                // PWM Frequency
                U32Input {
                    label: "PWM Frequency (Hz)".to_string(),
                    value: app_state.read().settings.pwm_frequency,
                    on_change: move |v| {
                        app_state.write().settings.pwm_frequency = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let dead_time = app_state.read().settings.pwm_dead_time;
                            let _ = mgr.lock().await.send_pwm_config(val, dead_time).await;
                        });
                    },
                    is_connected,
                    description: format!("PWM switching frequency. Default: {} Hz. ⚠ Requires reboot", DEFAULT_PWM_FREQUENCY)
                }

                // PWM Dead Time
                U16Input {
                    label: "PWM Dead Time".to_string(),
                    value: app_state.read().settings.pwm_dead_time,
                    on_change: move |v| {
                        app_state.write().settings.pwm_dead_time = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let freq = app_state.read().settings.pwm_frequency;
                            let _ = mgr.lock().await.send_pwm_config(freq, val).await;
                        });
                    },
                    is_connected,
                    description: format!("Dead time for complementary PWM. Default: {}. ⚠ Requires reboot", DEFAULT_PWM_DEAD_TIME)
                }

                // CAN Bitrate
                U32Input {
                    label: "CAN Bitrate (bps)".to_string(),
                    value: app_state.read().settings.can_bitrate,
                    on_change: move |v| {
                        app_state.write().settings.can_bitrate = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let _ = mgr.lock().await.send_can_config(val).await;
                        });
                    },
                    is_connected,
                    description: format!("CAN bus bitrate. Default: {} bps. ⚠ Requires reboot", DEFAULT_CAN_BITRATE)
                }

                // Control Period
                U64Input {
                    label: "Control Period (μs)".to_string(),
                    value: app_state.read().settings.control_period_us,
                    on_change: move |v| {
                        app_state.write().settings.control_period_us = v;
                        let val = v;
                        spawn(async move {
                            let mgr = app_state.read().can_manager.clone();
                            let _ = mgr.lock().await.send_control_timing(val).await;
                        });
                    },
                    is_connected,
                    description: format!("FOC control loop period. Default: {} μs. ⚠ Requires reboot", DEFAULT_CONTROL_PERIOD_US)
                }
            }
        }
    }
}

#[component]
fn ConfigManagementSection(is_connected: bool) -> Element {
    let app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    let on_save_config = move |_| {
        info!("Saving config to flash");
        spawn(async move {
            let manager = app_state.read().can_manager.clone();
            match manager.lock().await.send_save_config().await {
                Ok(_) => info!("Save config command sent successfully"),
                Err(e) => error!("Failed to send save config command: {}", e),
            };
        });
    };

    let on_reload_config = move |_| {
        info!("Reloading config from flash");
        spawn(async move {
            let manager = app_state.read().can_manager.clone();
            match manager.lock().await.send_reload_config().await {
                Ok(_) => info!("Reload config command sent successfully"),
                Err(e) => error!("Failed to send reload config command: {}", e),
            };
        });
    };

    let on_reset_config = move |_| {
        info!("Resetting config to defaults");
        spawn(async move {
            let manager = app_state.read().can_manager.clone();
            match manager.lock().await.send_reset_config().await {
                Ok(_) => info!("Reset config command sent successfully"),
                Err(e) => error!("Failed to send reset config command: {}", e),
            };
        });
    };

    rsx! {
        Card {
            SectionHeader {
                title: "Configuration Management".to_string(),
                color: HeaderColor::Green
            }

            div { style: "display: flex; flex-direction: column; gap: 15px;",
                // Description
                Banner {
                    banner_type: BannerType::Success,
                    message: "Save current settings to flash memory for persistence across power cycles.".to_string()
                }

                // Config status display
                div { style: "display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px;",
                    StatusCard {
                        label: "Config Version".to_string(),
                        value: format!("{}", state.config_version),
                        color: StatusCardColor::Green
                    }

                    StatusCard {
                        label: "CRC Status".to_string(),
                        value: if state.config_crc_valid { "✓ Valid".to_string() } else { "✗ Invalid".to_string() },
                        color: if state.config_crc_valid { StatusCardColor::Green } else { StatusCardColor::Red }
                    }
                }

                // Action buttons
                div { style: "display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px;",
                    Button {
                        variant: ButtonVariant::Success,
                        disabled: !is_connected,
                        onclick: on_save_config,
                        "💾 Save to Flash"
                    }

                    Button {
                        variant: ButtonVariant::Outline,
                        disabled: !is_connected,
                        onclick: on_reload_config,
                        "🔄 Reload from Flash"
                    }

                    Button {
                        variant: ButtonVariant::Danger,
                        disabled: !is_connected,
                        custom_style: "border: 1px solid #dc3545; background: white; color: #dc3545;".to_string(),
                        onclick: on_reset_config,
                        "⚠ Reset to Defaults"
                    }
                }
            }
        }
    }
}

#[component]
fn CalibrationTab(is_connected: bool) -> Element {
    let app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    // Calibration torque value (0-100)
    let mut calibration_torque = use_signal(|| 30u8);
    let mut is_calibrating = use_signal(|| false);

    let on_start_calibration = move |_| {
        let torque = calibration_torque();
        info!("Starting calibration with torque: {}", torque);
        is_calibrating.set(true);

        spawn(async move {
            let manager = app_state.read().can_manager.clone();
            match manager
                .lock()
                .await
                .send_start_calibration(Some(torque))
                .await
            {
                Ok(_) => info!("Calibration command sent successfully"),
                Err(e) => error!("Failed to send calibration command: {}", e),
            };

            // Reset calibrating flag after a delay
            tokio::time::sleep(tokio::time::Duration::from_secs(10)).await;
            is_calibrating.set(false);
        });
    };

    rsx! {
        Card {
            SectionHeader { title: "Motor Calibration".to_string(), color: HeaderColor::Orange }

            Banner {
                banner_type: BannerType::Warning,
                message: "⚠ WARNING: Ensure the motor can spin freely during calibration. Keep clear of moving parts!".to_string()
            }

            div { style: "display: flex; flex-direction: column; gap: 20px; margin-top: 20px;",
                // Description
                p { style: "color: #666; line-height: 1.6;",
                    "Motor calibration determines the electrical angle offset and rotation direction. "
                    "The motor will spin slowly during calibration to detect Hall sensor alignment."
                }

                // Calibration control
                div { style: "display: flex; flex-direction: column; gap: 15px; padding: 20px; background: #f8f9fa; border-radius: 8px;",
                    // Torque input
                    U8Input {
                        label: "Calibration Torque (0-100)".to_string(),
                        value: calibration_torque(),
                        on_change: move |v: u8| calibration_torque.set(v.min(100)),
                        is_connected,
                        description: "Torque level during calibration. Lower values for light motors, higher for heavy loads. Default: 30".to_string()
                    }

                    // Start calibration button
                    Button {
                        variant: if is_calibrating() { ButtonVariant::Secondary } else { ButtonVariant::Warning },
                        disabled: !is_connected || is_calibrating(),
                        custom_style: "width: 100%; padding: 12px;".to_string(),
                        onclick: on_start_calibration,
                        if is_calibrating() {
                            "🔄 Calibrating... (please wait)"
                        } else {
                            "⚡ Start Calibration"
                        }
                    }
                }

                // Calibration status display
                if let Some(cal_status) = state.calibration_status {
                    div { style: "display: flex; flex-direction: column; gap: 15px;",
                        SectionHeader {
                            title: "Calibration Results".to_string(),
                            color: if cal_status.success { HeaderColor::Green } else { HeaderColor::Red }
                        }

                        if cal_status.success {
                            Banner {
                                banner_type: BannerType::Success,
                                message: "✓ Calibration completed successfully!".to_string()
                            }
                        } else {
                            ErrorBanner {
                                message: "✗ Calibration failed. Please try again.".to_string()
                            }
                        }

                        div { style: "display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px;",
                            StatusCard {
                                label: "Electrical Offset".to_string(),
                                value: format!("{:.4} rad ({:.1}°)", cal_status.electrical_offset, cal_status.electrical_offset * 180.0 / std::f32::consts::PI),
                                color: StatusCardColor::Blue
                            }

                            StatusCard {
                                label: "Direction".to_string(),
                                value: if cal_status.direction_inversed { "Inversed".to_string() } else { "Normal".to_string() },
                                color: if cal_status.direction_inversed { StatusCardColor::Orange } else { StatusCardColor::Green }
                            }

                            StatusCard {
                                label: "Status".to_string(),
                                value: if cal_status.success { "✓ Success".to_string() } else { "✗ Failed".to_string() },
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
