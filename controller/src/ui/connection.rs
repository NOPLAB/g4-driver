use dioxus::prelude::*;
use tracing::{error, info};

use super::components::{Button, ButtonVariant, ErrorBanner, StatusColor, StatusIndicator};
use crate::can::{self, CanManager};
use crate::state::{AppState, ConnectionState};

#[component]
pub fn ConnectionBar() -> Element {
    let mut app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    // Refresh serial ports on first render
    use_effect(move || {
        spawn(async move {
            refresh_serial_ports(app_state).await;
        });
    });

    // Connection button handler
    let on_connect = move |_| {
        let is_connected = matches!(
            app_state.read().connection_state,
            ConnectionState::Connected
        );

        if is_connected {
            // Disconnect
            info!("Disconnecting from SLCAN adapter");
            let can_manager = app_state.read().can_manager.clone();
            spawn(async move {
                let mut manager = can_manager.lock().await;
                manager.disconnect().await;
                app_state.write().connection_state = ConnectionState::Disconnected;
            });
        } else {
            // Connect
            let port = app_state.read().selected_port.clone();
            if port.is_empty() {
                app_state.write().connection_state =
                    ConnectionState::Error("No serial port selected".to_string());
                return;
            }

            info!("Connecting to SLCAN adapter: {}", port);
            app_state.write().connection_state = ConnectionState::Connecting;

            spawn(async move {
                let can_manager = app_state.read().can_manager.clone();
                let mut manager = can_manager.lock().await;

                match manager.connect(&port).await {
                    Ok(_) => {
                        app_state.write().connection_state = ConnectionState::Connected;
                        info!("Connected successfully to {}", port);

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

    // Port selection handler
    let on_port_change = move |evt: Event<FormData>| {
        app_state.write().selected_port = evt.value();
    };

    // Refresh ports button
    let on_refresh_ports = move |_| {
        spawn(async move {
            refresh_serial_ports(app_state).await;
        });
    };

    // Determine connection status display
    let (status_color, status_text) = match &state.connection_state {
        ConnectionState::Disconnected => (StatusColor::Gray, "Disconnected"),
        ConnectionState::Connecting => (StatusColor::Orange, "Connecting..."),
        ConnectionState::Connected => (StatusColor::Green, "Connected"),
        ConnectionState::Error(_) => (StatusColor::Red, "Error"),
    };

    let button_text = if matches!(state.connection_state, ConnectionState::Connected) {
        "Disconnect"
    } else {
        "Connect"
    };

    let button_enabled = !matches!(state.connection_state, ConnectionState::Connecting);

    rsx! {
        div {
            style: "display: flex; flex-direction: column; background: #f5f5f5; border-bottom: 2px solid #ddd;",

            // Main connection bar
            div {
                style: "display: flex; align-items: center; gap: 15px; padding: 15px 20px;",

                // Title
                div {
                    style: "font-size: 18px; font-weight: bold; color: #333;",
                    "G4 Driver Controller"
                }

                // Spacer
                div { style: "flex: 1;" }

                // Serial port selection
                div {
                    style: "display: flex; align-items: center; gap: 8px;",
                    label {
                        style: "font-size: 14px; color: #555;",
                        "Serial Port:"
                    }
                    select {
                        style: "padding: 6px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px; min-width: 250px;",
                        value: "{state.selected_port}",
                        onchange: on_port_change,
                        disabled: matches!(state.connection_state, ConnectionState::Connected | ConnectionState::Connecting),

                        // Empty option
                        if state.available_ports.is_empty() {
                            option { value: "", "No ports detected" }
                        } else {
                            option { value: "", "Select a port..." }
                        }

                        // Available ports
                        for port in &state.available_ports {
                            option {
                                value: "{port.port_name}",
                                "{port.port_name} - {port.description}"
                            }
                        }
                    }
                }

                // Refresh button
                Button {
                    variant: ButtonVariant::Outline,
                    disabled: matches!(state.connection_state, ConnectionState::Connecting),
                    custom_style: "padding: 6px 12px; font-size: 13px;".to_string(),
                    onclick: on_refresh_ports,
                    "Refresh"
                }

                // Connect/Disconnect button
                Button {
                    variant: ButtonVariant::Primary,
                    disabled: !button_enabled,
                    custom_style: "padding: 8px 20px;".to_string(),
                    onclick: on_connect,
                    "{button_text}"
                }

                // Status indicator
                StatusIndicator {
                    text: status_text.to_string(),
                    color: status_color
                }
            }

            // Error message
            if let ConnectionState::Error(msg) = &state.connection_state {
                div {
                    style: "padding: 8px 20px; border-top: 1px solid #ef5350;",
                    ErrorBanner {
                        message: format!("Error: {}", msg)
                    }
                }
            }
        }
    }
}

/// Refresh available serial ports
async fn refresh_serial_ports(mut app_state: Signal<AppState>) {
    info!("Refreshing serial ports");

    // Detect serial ports
    let ports = can::detect_serial_ports();
    info!("Found {} serial ports", ports.len());

    let mut state = app_state.write();
    state.available_ports = ports;

    // Auto-select first port if none selected
    if state.selected_port.is_empty() && !state.available_ports.is_empty() {
        state.selected_port = state.available_ports[0].port_name.clone();
    }
}

/// Background task to receive CAN messages
async fn can_receive_task(mut app_state: Signal<AppState>) {
    info!("CAN receive task started");

    loop {
        let manager = app_state.read().can_manager.clone();

        // Check if still connected
        if !matches!(
            app_state.read().connection_state,
            ConnectionState::Connected
        ) {
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

                // Parse config status
                if let Some((version, crc_valid)) = CanManager::parse_config_status(&frame) {
                    let mut state = app_state.write();
                    state.config_version = version;
                    state.config_crc_valid = crc_valid;
                }

                // Parse calibration status
                if let Some(calibration_status) = CanManager::parse_calibration_status(&frame) {
                    info!(
                        "Calibration status: offset={:.4}, inversed={}, success={}",
                        calibration_status.electrical_offset,
                        calibration_status.direction_inversed,
                        calibration_status.success
                    );
                    app_state.write().calibration_status = Some(calibration_status);
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
