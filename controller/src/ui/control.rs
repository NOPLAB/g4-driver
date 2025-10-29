use dioxus::prelude::*;
use tracing::{error, info};

use crate::state::{AppState, ConnectionState};

#[component]
pub fn ControlPanel() -> Element {
    let mut app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    let is_connected = matches!(state.connection_state, ConnectionState::Connected);

    // Speed slider change handler
    let on_speed_slider_change = move |evt: Event<FormData>| {
        if let Some(value) = evt.value().parse::<f32>().ok() {
            app_state.write().settings.target_speed = value;
        }
    };

    // Speed input change handler
    let on_speed_input_change = move |evt: Event<FormData>| {
        if let Some(value) = evt.value().parse::<f32>().ok() {
            app_state.write().settings.target_speed = value.clamp(0.0, 3000.0);
        }
    };

    // Speed apply button handler
    let on_speed_apply = move |_| {
        let speed = app_state.read().settings.target_speed;
        info!("Applying speed: {} RPM", speed);

        spawn(async move {
            let manager = app_state.read().can_manager.clone();
            match manager.lock().await.send_speed_command(speed).await {
                Ok(_) => info!("Speed command sent successfully"),
                Err(e) => error!("Failed to send speed command: {}", e),
            };
        });
    };

    // Motor enable toggle handler
    let on_motor_enable_toggle = move |_| {
        let current_enabled = app_state.read().settings.motor_enabled;
        let new_enabled = !current_enabled;
        app_state.write().settings.motor_enabled = new_enabled;

        info!("Motor enable: {}", new_enabled);

        spawn(async move {
            let manager = app_state.read().can_manager.clone();
            match manager.lock().await.send_enable_command(new_enabled).await {
                Ok(_) => info!("Enable command sent successfully"),
                Err(e) => error!("Failed to send enable command: {}", e),
            };
        });
    };

    // Emergency stop handler
    let on_emergency_stop = move |_| {
        info!("EMERGENCY STOP");
        app_state.write().settings.target_speed = 0.0;
        app_state.write().settings.motor_enabled = false;

        spawn(async move {
            let manager = app_state.read().can_manager.clone();
            match manager.lock().await.send_emergency_stop().await {
                Ok(_) => info!("Emergency stop sent successfully"),
                Err(e) => error!("Failed to send emergency stop: {}", e),
            };
        });
    };

    rsx! {
        div {
            style: "display: flex; flex-direction: column; gap: 20px; max-width: 900px;",

            // Connection warning
            if !is_connected {
                div {
                    style: "padding: 15px; background: #fff3cd; border: 1px solid #ffc107; border-radius: 4px; color: #856404;",
                    "⚠ Not connected to CAN. Please connect first."
                }
            }

            // Speed Control Section
            div {
                style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);",

                h2 {
                    style: "margin: 0 0 15px 0; font-size: 20px; color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px;",
                    "Speed Control"
                }

                div {
                    style: "display: flex; flex-direction: column; gap: 15px;",

                    // Speed slider
                    div {
                        style: "display: flex; flex-direction: column; gap: 8px;",
                        label {
                            style: "font-size: 14px; font-weight: 500; color: #555;",
                            "Target Speed: {state.settings.target_speed:.1} RPM"
                        }
                        input {
                            r#type: "range",
                            min: 0,
                            max: 3000,
                            step: 10,
                            value: "{state.settings.target_speed}",
                            oninput: on_speed_slider_change,
                            disabled: !is_connected,
                            style: "width: 100%;",
                        }
                    }

                    // Speed input and apply button
                    div {
                        style: "display: flex; gap: 10px; align-items: center;",
                        input {
                            r#type: "number",
                            min: 0,
                            max: 3000,
                            step: 10,
                            value: "{state.settings.target_speed}",
                            oninput: on_speed_input_change,
                            disabled: !is_connected,
                            style: "flex: 1; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                        }
                        span {
                            style: "color: #666; font-size: 14px;",
                            "RPM"
                        }
                        button {
                            style: if is_connected {
                                "padding: 8px 20px; border: none; background: #28a745; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
                            } else {
                                "padding: 8px 20px; border: none; background: #ccc; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 14px; font-weight: 500;"
                            },
                            onclick: on_speed_apply,
                            disabled: !is_connected,
                            "Apply"
                        }
                    }
                }
            }

            // Motor Control Section
            div {
                style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);",

                h2 {
                    style: "margin: 0 0 15px 0; font-size: 20px; color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px;",
                    "Motor Control"
                }

                div {
                    style: "display: flex; gap: 20px; align-items: center;",

                    // Motor enable toggle
                    div {
                        style: "display: flex; align-items: center; gap: 10px;",
                        label {
                            style: "font-size: 14px; font-weight: 500; color: #555;",
                            "Motor Enable:"
                        }
                        button {
                            style: if is_connected {
                                if state.settings.motor_enabled {
                                    "padding: 8px 20px; border: none; background: #28a745; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
                                } else {
                                    "padding: 8px 20px; border: none; background: #6c757d; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
                                }
                            } else {
                                "padding: 8px 20px; border: none; background: #ccc; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 14px; font-weight: 500;"
                            },
                            onclick: on_motor_enable_toggle,
                            disabled: !is_connected,
                            if state.settings.motor_enabled { "Enabled" } else { "Disabled" }
                        }
                    }

                    // Emergency stop button
                    div {
                        style: "margin-left: auto;",
                        button {
                            style: if is_connected {
                                "padding: 12px 30px; border: none; background: #dc3545; color: white; cursor: pointer; border-radius: 4px; font-size: 16px; font-weight: bold; box-shadow: 0 2px 4px rgba(220,53,69,0.3);"
                            } else {
                                "padding: 12px 30px; border: none; background: #ccc; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 16px; font-weight: bold;"
                            },
                            onclick: on_emergency_stop,
                            disabled: !is_connected,
                            "🛑 EMERGENCY STOP"
                        }
                    }
                }
            }

            // Status Display Section
            div {
                style: "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);",

                h2 {
                    style: "margin: 0 0 15px 0; font-size: 20px; color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px;",
                    "Status"
                }

                div {
                    style: "display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px;",

                    // Current speed
                    div {
                        style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #007bff;",
                        div {
                            style: "font-size: 12px; color: #666; margin-bottom: 5px;",
                            "Current Speed"
                        }
                        div {
                            style: "font-size: 24px; font-weight: bold; color: #333;",
                            "{state.motor_status.speed_rpm:.1} RPM"
                        }
                    }

                    // Electrical angle
                    div {
                        style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #007bff;",
                        div {
                            style: "font-size: 12px; color: #666; margin-bottom: 5px;",
                            "Electrical Angle"
                        }
                        div {
                            style: "font-size: 24px; font-weight: bold; color: #333;",
                            "{state.motor_status.electrical_angle * 180.0 / std::f32::consts::PI:.1}°"
                        }
                    }

                    // DC Bus voltage
                    div {
                        style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #28a745;",
                        div {
                            style: "font-size: 12px; color: #666; margin-bottom: 5px;",
                            "DC Bus Voltage"
                        }
                        div {
                            style: "font-size: 24px; font-weight: bold; color: #333;",
                            "{state.voltage_status.voltage:.1} V"
                        }
                    }

                    // Voltage warnings
                    div {
                        style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid #ffc107;",
                        div {
                            style: "font-size: 12px; color: #666; margin-bottom: 5px;",
                            "Voltage Status"
                        }
                        div {
                            style: "font-size: 14px; font-weight: 500; color: #333;",
                            if state.voltage_status.overvoltage {
                                "⚠ OVERVOLTAGE"
                            } else if state.voltage_status.undervoltage {
                                "⚠ UNDERVOLTAGE"
                            } else {
                                "✓ Normal"
                            }
                        }
                    }
                }
            }
        }
    }
}
