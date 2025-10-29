use dioxus::prelude::*;
use tracing::{error, info};

use crate::can::{self, CanManager};
use crate::state::{AppState, ConnectionState};
use super::components::{Button, ButtonVariant, ErrorBanner, StatusColor, StatusIndicator};

#[component]
pub fn ConnectionBar() -> Element {
    let mut app_state = use_context::<Signal<AppState>>();
    let state = app_state.read();

    // Refresh interfaces on first render
    use_effect(move || {
        spawn(async move {
            refresh_interfaces(app_state).await;
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
            info!("Connecting to device: {}", interface);
            app_state.write().connection_state = ConnectionState::Connecting;

            spawn(async move {
                // Check if the selected interface is a USB device (starts with /dev/)
                let is_usb_device = interface.starts_with("/dev/");

                let actual_interface = if is_usb_device {
                    // USB device - setup SLCAN first
                    info!("USB device detected: {}", interface);
                    info!("Setting up SLCAN interface automatically...");

                    match can::setup_slcan_interface(&interface, "slcan0", 250000) {
                        Ok(_) => {
                            info!("SLCAN interface setup successful");

                            // Wait a bit for interface to be ready
                            tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;

                            // Refresh interfaces to show the new slcan0
                            refresh_interfaces(app_state).await;

                            "slcan0".to_string()
                        }
                        Err(e) => {
                            error!("SLCAN setup failed: {}", e);
                            app_state.write().connection_state = ConnectionState::Error(format!(
                                "SLCAN setup failed: {}. Try running with sudo privileges.",
                                e
                            ));
                            return;
                        }
                    }
                } else {
                    // Regular CAN interface
                    interface.clone()
                };

                // Connect to the CAN interface
                info!("Connecting to CAN interface: {}", actual_interface);
                let can_manager = app_state.read().can_manager.clone();
                let mut manager = can_manager.lock().await;
                match manager.connect(&actual_interface).await {
                    Ok(_) => {
                        app_state.write().connection_state = ConnectionState::Connected;
                        info!("Connected successfully to {}", actual_interface);

                        // Update interface name to the actual interface (slcan0 if USB was used)
                        if is_usb_device {
                            app_state.write().interface = actual_interface.clone();
                        }

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

    // Refresh interfaces button
    let on_refresh_interfaces = move |_| {
        spawn(async move {
            refresh_interfaces(app_state).await;
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

                // Interface selection (CAN interfaces + USB devices)
                div {
                    style: "display: flex; align-items: center; gap: 8px;",
                    label {
                        style: "font-size: 14px; color: #555;",
                        "Device:"
                    }
                    select {
                        style: "padding: 6px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px; min-width: 200px;",
                        value: "{state.interface}",
                        onchange: on_interface_change,
                        disabled: matches!(state.connection_state, ConnectionState::Connected | ConnectionState::Connecting),

                        // CAN Interfaces group
                        if !state.available_interfaces.is_empty() {
                            optgroup {
                                label: "CAN Interfaces",
                                for interface in &state.available_interfaces {
                                    {
                                        let status = if interface.is_up { "UP" } else { "DOWN" };
                                        let display_text = format!("{} ({})", interface.name, status);
                                        rsx! {
                                            option {
                                                value: "{interface.name}",
                                                "{display_text}"
                                            }
                                        }
                                    }
                                }
                            }
                        } else {
                            optgroup {
                                label: "CAN Interfaces",
                                option { value: "can0", "can0 (default)" }
                                option { value: "vcan0", "vcan0 (default)" }
                            }
                        }

                        // USB Devices group
                        if !state.available_usb_devices.is_empty() {
                            optgroup {
                                label: "USB-CAN Adapters (auto-setup SLCAN)",
                                for device in &state.available_usb_devices {
                                    option {
                                        value: "{device.device_path}",
                                        "{device.device_path} - {device.description}"
                                    }
                                }
                            }
                        }
                    }
                }

                // Refresh button
                Button {
                    variant: ButtonVariant::Outline,
                    disabled: matches!(state.connection_state, ConnectionState::Connecting),
                    custom_style: "padding: 6px 12px; font-size: 13px;".to_string(),
                    onclick: on_refresh_interfaces,
                    "🔄 Refresh"
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

/// Refresh available CAN interfaces and USB devices
async fn refresh_interfaces(mut app_state: Signal<AppState>) {
    info!("Refreshing CAN interfaces and USB devices");

    // Detect CAN interfaces
    match can::detect_can_interfaces() {
        Ok(interfaces) => {
            info!("Found {} CAN interfaces", interfaces.len());
            app_state.write().available_interfaces = interfaces;
        }
        Err(e) => {
            error!("Failed to detect CAN interfaces: {}", e);
        }
    }

    // Detect USB-CAN devices
    let usb_devices = can::detect_usb_can_devices();
    info!("Found {} USB-CAN devices", usb_devices.len());
    app_state.write().available_usb_devices = usb_devices;
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
