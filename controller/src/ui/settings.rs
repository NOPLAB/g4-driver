use dioxus::prelude::*;
use tracing::{error, info};

use crate::state::{AppState, ConnectionState};
use super::components::{
    Banner, BannerType, Button, ButtonVariant, Card, ErrorBanner, F32Input, HeaderColor,
    SectionHeader, StatusCard, StatusCardColor, U16Input, U32Input, U64Input, U8Input,
    WarningBanner,
};

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

