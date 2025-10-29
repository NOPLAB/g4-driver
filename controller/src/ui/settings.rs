use dioxus::prelude::*;
use tracing::{error, info};

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
                div {
                    style: "padding: 15px; background: #fff3cd; border: 1px solid #ffc107; border-radius: 4px; color: #856404;",
                    "⚠ Not connected to CAN. Please connect first."
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
                    "Advanced"
                }
            }

            // Tab content
            match selected_tab() {
                0 => rsx! { PIControlTab { is_connected } },
                1 => rsx! { MotorControlTab { is_connected } },
                2 => rsx! { HallSensorTab { is_connected } },
                3 => rsx! { OpenLoopTab { is_connected } },
                4 => rsx! { AdvancedTab { is_connected } },
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

    let on_kp_change = move |evt: Event<FormData>| {
        if let Some(value) = evt.value().parse::<f32>().ok() {
            app_state.write().settings.kp = value;
        }
    };

    let on_ki_change = move |evt: Event<FormData>| {
        if let Some(value) = evt.value().parse::<f32>().ok() {
            app_state.write().settings.ki = value;
        }
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
        div {
            style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);",

            h2 { style: "margin: 0 0 15px 0; font-size: 20px; color: #333;", "PI Controller Settings" }

            div {
                style: "padding: 12px; background: #e7f3ff; border-left: 4px solid #007bff; border-radius: 4px; margin-bottom: 20px;",
                p { style: "margin: 0; font-size: 14px; color: #555;",
                    "Configure the PI controller gains for speed control. These values affect the motor's response to speed commands."
                }
            }

            div { style: "display: grid; gap: 20px;",
                // Kp input
                div {
                    label { style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;",
                        "Proportional Gain (Kp)"
                    }
                    input {
                        r#type: "number",
                        step: "0.01",
                        value: "{state.settings.kp}",
                        oninput: on_kp_change,
                        disabled: !is_connected,
                        style: "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                    }
                    p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                        "Higher values provide faster response but may cause oscillations. Default: {DEFAULT_KP}"
                    }
                }

                // Ki input
                div {
                    label { style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;",
                        "Integral Gain (Ki)"
                    }
                    input {
                        r#type: "number",
                        step: "0.001",
                        value: "{state.settings.ki}",
                        oninput: on_ki_change,
                        disabled: !is_connected,
                        style: "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                    }
                    p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                        "Helps eliminate steady-state error. Too high may cause instability. Default: {DEFAULT_KI}"
                    }
                }

                // Action buttons
                div { style: "display: flex; gap: 10px; margin-top: 10px;",
                    button {
                        style: if is_connected {
                            "flex: 1; padding: 10px 20px; border: none; background: #28a745; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
                        } else {
                            "flex: 1; padding: 10px 20px; border: none; background: #ccc; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 14px; font-weight: 500;"
                        },
                        onclick: on_apply,
                        disabled: !is_connected,
                        "Apply Settings"
                    }
                    button {
                        style: "flex: 1; padding: 10px 20px; border: 1px solid #6c757d; background: white; color: #6c757d; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;",
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
        div {
            style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px;",
            h2 { "Motor Control Parameters" }
            p { style: "color: #666;", "Configure motor voltage, pole pairs, and duty cycle parameters." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                // Max Voltage
                SettingInput {
                    label: "Max Voltage (V)",
                    value: app_state.read().settings.max_voltage,
                    step: "0.1",
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
                    default_value: DEFAULT_MAX_VOLTAGE,
                    description: "Maximum voltage limit for motor control"
                }

                // DC Bus Voltage
                SettingInput {
                    label: "DC Bus Voltage (V)",
                    value: app_state.read().settings.v_dc_bus,
                    step: "0.1",
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
                    default_value: DEFAULT_V_DC_BUS,
                    description: "DC bus voltage for calculations"
                }

                // Pole Pairs
                SettingInputU8 {
                    label: "Pole Pairs",
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
                    default_value: DEFAULT_POLE_PAIRS,
                    description: "Number of motor pole pairs (poles/2)"
                }

                // Max Duty
                SettingInputU16 {
                    label: "Max Duty Cycle",
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
                    default_value: DEFAULT_MAX_DUTY,
                    description: "Maximum PWM duty cycle (0-100)"
                }
            }
        }
    }
}

#[component]
fn HallSensorTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        div {
            style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px;",
            h2 { "Hall Sensor Parameters" }
            p { style: "color: #666;", "Configure Hall sensor filter and angle offset." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                // Speed Filter Alpha
                SettingInput {
                    label: "Speed Filter Alpha",
                    value: app_state.read().settings.speed_filter_alpha,
                    step: "0.01",
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
                    default_value: DEFAULT_SPEED_FILTER_ALPHA,
                    description: "Low-pass filter coefficient for speed (0-1)"
                }

                // Hall Angle Offset
                SettingInput {
                    label: "Hall Angle Offset (rad)",
                    value: app_state.read().settings.hall_angle_offset,
                    step: "0.01",
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
                    default_value: DEFAULT_HALL_ANGLE_OFFSET,
                    description: "Angle offset for Hall sensor alignment"
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
        div {
            style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px;",
            h2 { "OpenLoop Startup Parameters" }
            p { style: "color: #666;", "Configure openloop ramp-up for motor startup." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                // Initial RPM
                SettingInput {
                    label: "Initial RPM",
                    value: app_state.read().settings.openloop_initial_rpm,
                    step: "10.0",
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
                    default_value: DEFAULT_OPENLOOP_INITIAL_RPM,
                    description: "Starting RPM for openloop ramp-up"
                }

                // Target RPM
                SettingInput {
                    label: "Target RPM",
                    value: app_state.read().settings.openloop_target_rpm,
                    step: "10.0",
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
                    default_value: DEFAULT_OPENLOOP_TARGET_RPM,
                    description: "Target RPM to switch to FOC control"
                }

                // Acceleration
                SettingInput {
                    label: "Acceleration (RPM/s)",
                    value: app_state.read().settings.openloop_acceleration,
                    step: "10.0",
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
                    default_value: DEFAULT_OPENLOOP_ACCELERATION,
                    description: "Ramp-up acceleration rate"
                }

                // Duty Ratio
                SettingInputU16 {
                    label: "Duty Ratio (0-100)",
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
                    default_value: DEFAULT_OPENLOOP_DUTY_RATIO,
                    description: "PWM duty ratio during openloop"
                }
            }
        }
    }
}

#[component]
fn AdvancedTab(is_connected: bool) -> Element {
    let mut app_state = use_context::<Signal<AppState>>();

    rsx! {
        div {
            style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px;",

            // Warning banner
            div {
                style: "padding: 15px; background: #f8d7da; border: 1px solid #f5c6cb; border-radius: 4px; color: #721c24; margin-bottom: 20px;",
                "⚠ Warning: These settings require device reboot to take effect. Incorrect values may prevent the device from operating."
            }

            h2 { "Advanced Configuration" }
            p { style: "color: #666;", "Low-level hardware configuration. Change with caution." }

            div { style: "display: grid; gap: 15px; margin-top: 20px;",
                // PWM Frequency
                SettingInputU32 {
                    label: "PWM Frequency (Hz)",
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
                    default_value: DEFAULT_PWM_FREQUENCY,
                    description: "PWM switching frequency. Default: {DEFAULT_PWM_FREQUENCY} Hz. ⚠ Requires reboot"
                }

                // PWM Dead Time
                SettingInputU16 {
                    label: "PWM Dead Time",
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
                    default_value: DEFAULT_PWM_DEAD_TIME,
                    description: "Dead time for complementary PWM. ⚠ Requires reboot"
                }

                // CAN Bitrate
                SettingInputU32 {
                    label: "CAN Bitrate (bps)",
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
                    default_value: DEFAULT_CAN_BITRATE,
                    description: "CAN bus bitrate. Default: {DEFAULT_CAN_BITRATE} bps. ⚠ Requires reboot"
                }

                // Control Period
                SettingInputU64 {
                    label: "Control Period (μs)",
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
                    default_value: DEFAULT_CONTROL_PERIOD_US,
                    description: "FOC control loop period. Default: {DEFAULT_CONTROL_PERIOD_US} μs. ⚠ Requires reboot"
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
        div {
            style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);",

            h2 {
                style: "margin: 0 0 15px 0; font-size: 20px; color: #333; border-bottom: 2px solid #28a745; padding-bottom: 10px;",
                "Configuration Management"
            }

            div { style: "display: flex; flex-direction: column; gap: 15px;",
                // Description
                div {
                    style: "padding: 12px; background: #e7f9ed; border-left: 4px solid #28a745; border-radius: 4px;",
                    p { style: "margin: 0; font-size: 14px; color: #555;",
                        "Save current settings to flash memory for persistence across power cycles."
                    }
                }

                // Config status display
                div { style: "display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px;",
                    // Version
                    div {
                        style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #28a745;",
                        div { style: "font-size: 12px; color: #666; margin-bottom: 5px;", "Config Version" }
                        div { style: "font-size: 24px; font-weight: bold; color: #333;", "{state.config_version}" }
                    }

                    // CRC status
                    div {
                        style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #28a745;",
                        div { style: "font-size: 12px; color: #666; margin-bottom: 5px;", "CRC Status" }
                        div {
                            style: if state.config_crc_valid {
                                "font-size: 18px; font-weight: bold; color: #28a745;"
                            } else {
                                "font-size: 18px; font-weight: bold; color: #dc3545;"
                            },
                            if state.config_crc_valid { "✓ Valid" } else { "✗ Invalid" }
                        }
                    }
                }

                // Action buttons
                div { style: "display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px;",
                    button {
                        style: if is_connected {
                            "padding: 10px 20px; border: none; background: #28a745; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
                        } else {
                            "padding: 10px 20px; border: none; background: #ccc; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 14px; font-weight: 500;"
                        },
                        onclick: on_save_config,
                        disabled: !is_connected,
                        "💾 Save to Flash"
                    }

                    button {
                        style: if is_connected {
                            "padding: 10px 20px; border: 1px solid #007bff; background: white; color: #007bff; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
                        } else {
                            "padding: 10px 20px; border: 1px solid #ccc; background: white; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 14px; font-weight: 500;"
                        },
                        onclick: on_reload_config,
                        disabled: !is_connected,
                        "🔄 Reload from Flash"
                    }

                    button {
                        style: if is_connected {
                            "padding: 10px 20px; border: 1px solid #dc3545; background: white; color: #dc3545; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
                        } else {
                            "padding: 10px 20px; border: 1px solid #ccc; background: white; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 14px; font-weight: 500;"
                        },
                        onclick: on_reset_config,
                        disabled: !is_connected,
                        "⚠ Reset to Defaults"
                    }
                }
            }
        }
    }
}

// Helper components for consistent input styling
#[component]
fn SettingInput(
    label: String,
    value: f32,
    step: String,
    on_change: EventHandler<f32>,
    is_connected: bool,
    default_value: f32,
    description: String,
) -> Element {
    rsx! {
        div {
            label { style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;", "{label}" }
            input {
                r#type: "number",
                step: "{step}",
                value: "{value}",
                disabled: !is_connected,
                style: "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                oninput: move |evt| {
                    if let Some(v) = evt.value().parse::<f32>().ok() {
                        on_change.call(v);
                    }
                },
            }
            p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;", "{description}" }
        }
    }
}

#[component]
fn SettingInputU8(
    label: String,
    value: u8,
    on_change: EventHandler<u8>,
    is_connected: bool,
    default_value: u8,
    description: String,
) -> Element {
    rsx! {
        div {
            label { style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;", "{label}" }
            input {
                r#type: "number",
                value: "{value}",
                disabled: !is_connected,
                style: "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                oninput: move |evt| {
                    if let Some(v) = evt.value().parse::<u8>().ok() {
                        on_change.call(v);
                    }
                },
            }
            p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;", "{description}" }
        }
    }
}

#[component]
fn SettingInputU16(
    label: String,
    value: u16,
    on_change: EventHandler<u16>,
    is_connected: bool,
    default_value: u16,
    description: String,
) -> Element {
    rsx! {
        div {
            label { style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;", "{label}" }
            input {
                r#type: "number",
                value: "{value}",
                disabled: !is_connected,
                style: "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                oninput: move |evt| {
                    if let Some(v) = evt.value().parse::<u16>().ok() {
                        on_change.call(v);
                    }
                },
            }
            p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;", "{description}" }
        }
    }
}

#[component]
fn SettingInputU32(
    label: String,
    value: u32,
    on_change: EventHandler<u32>,
    is_connected: bool,
    default_value: u32,
    description: String,
) -> Element {
    rsx! {
        div {
            label { style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;", "{label}" }
            input {
                r#type: "number",
                value: "{value}",
                disabled: !is_connected,
                style: "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                oninput: move |evt| {
                    if let Some(v) = evt.value().parse::<u32>().ok() {
                        on_change.call(v);
                    }
                },
            }
            p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;", "{description}" }
        }
    }
}

#[component]
fn SettingInputU64(
    label: String,
    value: u64,
    on_change: EventHandler<u64>,
    is_connected: bool,
    default_value: u64,
    description: String,
) -> Element {
    rsx! {
        div {
            label { style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;", "{label}" }
            input {
                r#type: "number",
                value: "{value}",
                disabled: !is_connected,
                style: "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                oninput: move |evt| {
                    if let Some(v) = evt.value().parse::<u64>().ok() {
                        on_change.call(v);
                    }
                },
            }
            p { style: "margin: 4px 0 0 0; font-size: 12px; color: #666;", "{description}" }
        }
    }
}
