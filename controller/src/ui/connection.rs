use dioxus::prelude::*;
use tracing::{error, info};

use crate::can::CanManager;
use crate::state::{AppState, ConnectionState};

#[component]
pub fn ConnectionBar() -> Element {
    let mut app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    // List of available CAN interfaces
    let interfaces = vec!["can0", "can1", "vcan0", "vcan1"];

    // Connection button handler
    let on_connect = move |_| {
        let is_connected = matches!(
            app_state.read().connection_state,
            ConnectionState::Connected
        );

        if is_connected {
            // Disconnect
            info!("Disconnecting from CAN");
            let can_manager = app_state.read().can_manager.clone();
            spawn(async move {
                let mut manager = can_manager.lock().await;
                manager.disconnect().await;
                app_state.write().connection_state = ConnectionState::Disconnected;
            });
        } else {
            // Connect
            let interface = app_state.read().interface.clone();
            info!("Connecting to CAN interface: {}", interface);
            app_state.write().connection_state = ConnectionState::Connecting;

            let can_manager = app_state.read().can_manager.clone();
            spawn(async move {
                let mut manager = can_manager.lock().await;
                match manager.connect(&interface).await {
                    Ok(_) => {
                        app_state.write().connection_state = ConnectionState::Connected;
                        info!("Connected successfully");

                        // Start CAN receive task
                        spawn(can_receive_task(app_state));
                    }
                    Err(e) => {
                        error!("Connection failed: {}", e);
                        app_state.write().connection_state =
                            ConnectionState::Error(format!("Connection failed: {}", e));
                    }
                }
            });
        }
    };

    // Interface selection handler
    let on_interface_change = move |evt: Event<FormData>| {
        app_state.write().interface = evt.value();
    };

    // Determine connection status display
    let (status_color, status_text) = match &state.connection_state {
        ConnectionState::Disconnected => ("#999", "Disconnected"),
        ConnectionState::Connecting => ("#ff9800", "Connecting..."),
        ConnectionState::Connected => ("#4caf50", "Connected"),
        ConnectionState::Error(_) => ("#f44336", "Error"),
    };

    let button_text = if matches!(state.connection_state, ConnectionState::Connected) {
        "Disconnect"
    } else {
        "Connect"
    };

    let button_enabled = !matches!(state.connection_state, ConnectionState::Connecting);

    rsx! {
        div {
            style: "display: flex; align-items: center; gap: 15px; padding: 15px 20px; background: #f5f5f5; border-bottom: 2px solid #ddd;",

            // Title
            div {
                style: "font-size: 18px; font-weight: bold; color: #333;",
                "G4 Driver Controller"
            }

            // Spacer
            div { style: "flex: 1;" }

            // Interface selection
            div {
                style: "display: flex; align-items: center; gap: 8px;",
                label {
                    style: "font-size: 14px; color: #555;",
                    "Interface:"
                }
                select {
                    style: "padding: 6px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;",
                    value: "{state.interface}",
                    onchange: on_interface_change,
                    disabled: matches!(state.connection_state, ConnectionState::Connected | ConnectionState::Connecting),

                    for interface in interfaces {
                        option {
                            value: "{interface}",
                            "{interface}"
                        }
                    }
                }
            }

            // Connect/Disconnect button
            button {
                style: if button_enabled {
                    "padding: 8px 20px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
                } else {
                    "padding: 8px 20px; border: none; background: #ccc; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 14px; font-weight: 500;"
                },
                onclick: on_connect,
                disabled: !button_enabled,
                "{button_text}"
            }

            // Status indicator
            div {
                style: "display: flex; align-items: center; gap: 8px; padding: 6px 12px; background: white; border-radius: 4px; border: 1px solid #ddd;",
                div {
                    style: "width: 12px; height: 12px; border-radius: 50%; background: {status_color};",
                }
                span {
                    style: "font-size: 14px; color: #333;",
                    "{status_text}"
                }
            }

            // Error message
            if let ConnectionState::Error(msg) = &state.connection_state {
                div {
                    style: "color: #f44336; font-size: 12px; max-width: 200px;",
                    "{msg}"
                }
            }
        }
    }
}

/// Background task to receive CAN messages
async fn can_receive_task(mut app_state: Signal<AppState>) {
    info!("CAN receive task started");

    loop {
        let manager = app_state.read().can_manager.clone();

        // Check if still connected
        if !matches!(app_state.read().connection_state, ConnectionState::Connected) {
            info!("CAN receive task stopped: not connected");
            break;
        }

        // Receive frame with timeout
        match manager.lock().await.receive_frame(100).await {
            Ok(Some(frame)) => {
                // Parse motor status
                if let Some(motor_status) = CanManager::parse_motor_status(&frame) {
                    let mut state = app_state.write();
                    state.motor_status = motor_status;
                    state.last_status_update = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_millis() as u64;
                }

                // Parse voltage status
                if let Some(voltage_status) = CanManager::parse_voltage_status(&frame) {
                    app_state.write().voltage_status = voltage_status;
                }
            }
            Ok(None) => {
                // Timeout - check connection health
                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;

                let last_update = app_state.read().last_status_update;

                // If no status update for 500ms, consider connection lost
                if last_update > 0 && now - last_update > 500 {
                    error!("CAN status timeout");
                    app_state.write().connection_state =
                        ConnectionState::Error("Status timeout".to_string());
                    break;
                }
            }
            Err(e) => {
                error!("CAN receive error: {}", e);
                app_state.write().connection_state =
                    ConnectionState::Error(format!("Receive error: {}", e));
                break;
            }
        }

        // Small delay to prevent busy loop
        tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
    }

    info!("CAN receive task ended");
}
