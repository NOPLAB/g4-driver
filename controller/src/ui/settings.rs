use dioxus::prelude::*;
use tracing::{error, info};

use crate::state::{AppState, ConnectionState};

// Default PI gains (from firmware config)
const DEFAULT_KP: f32 = 0.5;
const DEFAULT_KI: f32 = 0.05;

#[component]
pub fn SettingsPanel() -> Element {
    let mut app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    let is_connected = matches!(state.connection_state, ConnectionState::Connected);

    // Kp input change handler
    let on_kp_change = move |evt: Event<FormData>| {
        if let Some(value) = evt.value().parse::<f32>().ok() {
            app_state.write().settings.kp = value;
        }
    };

    // Ki input change handler
    let on_ki_change = move |evt: Event<FormData>| {
        if let Some(value) = evt.value().parse::<f32>().ok() {
            app_state.write().settings.ki = value;
        }
    };

    // Apply PI gains handler
    let on_apply_gains = move |_| {
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

    // Reset to defaults handler
    let on_reset_defaults = move |_| {
        info!("Resetting PI gains to defaults");
        app_state.write().settings.kp = DEFAULT_KP;
        app_state.write().settings.ki = DEFAULT_KI;
    };

    // Save config handler
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

    // Reload config handler
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

    // Reset config handler
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
            style: "display: flex; flex-direction: column; gap: 20px; max-width: 600px;",

            // Connection warning
            if !is_connected {
                div {
                    style: "padding: 15px; background: #fff3cd; border: 1px solid #ffc107; border-radius: 4px; color: #856404;",
                    "⚠ Not connected to CAN. Please connect first."
                }
            }

            // PI Gains Settings Section
            div {
                style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);",

                h2 {
                    style: "margin: 0 0 15px 0; font-size: 20px; color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px;",
                    "PI Controller Settings"
                }

                div {
                    style: "display: flex; flex-direction: column; gap: 20px;",

                    // Description
                    div {
                        style: "padding: 12px; background: #e7f3ff; border-left: 4px solid #007bff; border-radius: 4px;",
                        p {
                            style: "margin: 0; font-size: 14px; color: #555;",
                            "Configure the PI controller gains for speed control. These values affect the motor's response to speed commands."
                        }
                    }

                    // Kp input
                    div {
                        style: "display: flex; flex-direction: column; gap: 8px;",
                        label {
                            style: "font-size: 14px; font-weight: 500; color: #555;",
                            "Proportional Gain (Kp)"
                        }
                        div {
                            style: "display: flex; gap: 10px; align-items: center;",
                            input {
                                r#type: "number",
                                step: "0.01",
                                min: 0,
                                value: "{state.settings.kp}",
                                oninput: on_kp_change,
                                disabled: !is_connected,
                                style: "flex: 1; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                            }
                            span {
                                style: "color: #666; font-size: 12px; min-width: 100px;",
                                "Default: {DEFAULT_KP}"
                            }
                        }
                        p {
                            style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                            "Higher values provide faster response but may cause oscillations."
                        }
                    }

                    // Ki input
                    div {
                        style: "display: flex; flex-direction: column; gap: 8px;",
                        label {
                            style: "font-size: 14px; font-weight: 500; color: #555;",
                            "Integral Gain (Ki)"
                        }
                        div {
                            style: "display: flex; gap: 10px; align-items: center;",
                            input {
                                r#type: "number",
                                step: "0.001",
                                min: 0,
                                value: "{state.settings.ki}",
                                oninput: on_ki_change,
                                disabled: !is_connected,
                                style: "flex: 1; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                            }
                            span {
                                style: "color: #666; font-size: 12px; min-width: 100px;",
                                "Default: {DEFAULT_KI}"
                            }
                        }
                        p {
                            style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                            "Helps eliminate steady-state error. Too high may cause instability."
                        }
                    }

                    // Action buttons
                    div {
                        style: "display: flex; gap: 10px; margin-top: 10px;",
                        button {
                            style: if is_connected {
                                "flex: 1; padding: 10px 20px; border: none; background: #28a745; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
                            } else {
                                "flex: 1; padding: 10px 20px; border: none; background: #ccc; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 14px; font-weight: 500;"
                            },
                            onclick: on_apply_gains,
                            disabled: !is_connected,
                            "Apply Settings"
                        }
                        button {
                            style: "flex: 1; padding: 10px 20px; border: 1px solid #6c757d; background: white; color: #6c757d; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;",
                            onclick: on_reset_defaults,
                            "Reset to Defaults"
                        }
                    }
                }
            }

            // Current Values Display
            div {
                style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);",

                h2 {
                    style: "margin: 0 0 15px 0; font-size: 20px; color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px;",
                    "Current Settings"
                }

                div {
                    style: "display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px;",

                    // Kp value
                    div {
                        style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #007bff;",
                        div {
                            style: "font-size: 12px; color: #666; margin-bottom: 5px;",
                            "Kp"
                        }
                        div {
                            style: "font-size: 24px; font-weight: bold; color: #333;",
                            "{state.settings.kp:.3}"
                        }
                    }

                    // Ki value
                    div {
                        style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #007bff;",
                        div {
                            style: "font-size: 12px; color: #666; margin-bottom: 5px;",
                            "Ki"
                        }
                        div {
                            style: "font-size: 24px; font-weight: bold; color: #333;",
                            "{state.settings.ki:.4}"
                        }
                    }
                }
            }

            // Config Management Section
            div {
                style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);",

                h2 {
                    style: "margin: 0 0 15px 0; font-size: 20px; color: #333; border-bottom: 2px solid #28a745; padding-bottom: 10px;",
                    "Configuration Management"
                }

                div {
                    style: "display: flex; flex-direction: column; gap: 15px;",

                    // Description
                    div {
                        style: "padding: 12px; background: #e7f9ed; border-left: 4px solid #28a745; border-radius: 4px;",
                        p {
                            style: "margin: 0; font-size: 14px; color: #555;",
                            "Save current settings to flash memory for persistence across power cycles."
                        }
                    }

                    // Config status display
                    div {
                        style: "display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px;",

                        // Version
                        div {
                            style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #28a745;",
                            div {
                                style: "font-size: 12px; color: #666; margin-bottom: 5px;",
                                "Config Version"
                            }
                            div {
                                style: "font-size: 24px; font-weight: bold; color: #333;",
                                "{state.config_version}"
                            }
                        }

                        // CRC status
                        div {
                            style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #28a745;",
                            div {
                                style: "font-size: 12px; color: #666; margin-bottom: 5px;",
                                "CRC Status"
                            }
                            div {
                                style: if state.config_crc_valid {
                                    "font-size: 18px; font-weight: bold; color: #28a745;"
                                } else {
                                    "font-size: 18px; font-weight: bold; color: #dc3545;"
                                },
                                if state.config_crc_valid {
                                    "✓ Valid"
                                } else {
                                    "✗ Invalid"
                                }
                            }
                        }
                    }

                    // Action buttons
                    div {
                        style: "display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px;",

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

            // Information Section
            div {
                style: "padding: 20px; background: #f8f9fa; border: 1px solid #dee2e6; border-radius: 8px;",

                h3 {
                    style: "margin: 0 0 10px 0; font-size: 16px; color: #333;",
                    "ℹ Information"
                }

                ul {
                    style: "margin: 0; padding-left: 20px; font-size: 14px; color: #555; line-height: 1.6;",
                    li { "Changes are applied to the motor controller in real-time" }
                    li { "The motor must be connected via CAN to apply settings" }
                    li { "Use caution when adjusting gains while the motor is running" }
                    li { "Default values are optimized for stable operation" }
                    li { "Save to Flash: Persist current settings across power cycles" }
                    li { "Reload from Flash: Restore settings from flash memory" }
                    li { "Reset to Defaults: Restore factory default settings and save to flash" }
                }
            }
        }
    }
}
