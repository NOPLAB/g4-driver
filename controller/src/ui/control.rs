use dioxus::prelude::*;
use tracing::{error, info};

use super::components::{
    Button, ButtonVariant, Card, EmergencyStopButton, F32InputInline, SectionHeader, StatusCard,
    StatusCardColor, ToggleSwitch, WarningBanner,
};
use crate::state::{AppState, ConnectionState};

#[component]
pub fn ControlPanel() -> Element {
    let mut app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    let is_connected = matches!(state.connection_state, ConnectionState::Connected);

    // Speed slider change handler
    let on_speed_slider_change = move |evt: Event<FormData>| {
        if let Ok(value) = evt.value().parse::<f32>() {
            app_state.write().settings.target_speed = value;
        }
    };

    // Speed input change handler
    let on_speed_input_change = move |value: f32| {
        app_state.write().settings.target_speed = value.clamp(0.0, 3000.0);
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
                WarningBanner {
                    message: "Not connected to CAN. Please connect first.".to_string()
                }
            }

            // Speed Control Section
            Card {
                SectionHeader {
                    title: "Speed Control".to_string()
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
                        F32InputInline {
                            value: state.settings.target_speed,
                            on_change: on_speed_input_change,
                            disabled: !is_connected,
                            style: "flex: 1; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;".to_string(),
                            placeholder: Some("Enter speed...".to_string()),
                        }
                        span {
                            style: "color: #666; font-size: 14px;",
                            "RPM"
                        }
                        Button {
                            variant: ButtonVariant::Success,
                            disabled: !is_connected,
                            onclick: on_speed_apply,
                            "Apply"
                        }
                    }
                }
            }

            // Motor Control Section
            Card {
                SectionHeader {
                    title: "Motor Control".to_string()
                }

                div {
                    style: "display: flex; gap: 20px; align-items: center;",

                    // Motor enable toggle switch
                    ToggleSwitch {
                        label: "Motor Enable:".to_string(),
                        checked: state.settings.motor_enabled,
                        is_disabled: !is_connected,
                        onchange: on_motor_enable_toggle
                    }

                    // Emergency stop button
                    div {
                        style: "margin-left: auto;",
                        EmergencyStopButton {
                            disabled: !is_connected,
                            onclick: on_emergency_stop
                        }
                    }
                }
            }

            // Status Display Section
            Card {
                SectionHeader {
                    title: "Status".to_string()
                }

                div {
                    style: "display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px;",

                    // Current speed
                    StatusCard {
                        label: "Current Speed".to_string(),
                        value: format!("{:.1} RPM", state.motor_status.speed_rpm),
                        color: StatusCardColor::Blue
                    }

                    // Electrical angle
                    StatusCard {
                        label: "Electrical Angle".to_string(),
                        value: format!("{:.1}°", state.motor_status.electrical_angle * 180.0 / std::f32::consts::PI),
                        color: StatusCardColor::Blue
                    }

                    // DC Bus voltage
                    StatusCard {
                        label: "DC Bus Voltage".to_string(),
                        value: format!("{:.1} V", state.voltage_status.voltage),
                        color: StatusCardColor::Green
                    }

                    // Voltage warnings
                    StatusCard {
                        label: "Voltage Status".to_string(),
                        value: if state.voltage_status.overvoltage {
                            "⚠ OVERVOLTAGE".to_string()
                        } else if state.voltage_status.undervoltage {
                            "⚠ UNDERVOLTAGE".to_string()
                        } else {
                            "✓ Normal".to_string()
                        },
                        color: StatusCardColor::Yellow
                    }
                }
            }
        }
    }
}
