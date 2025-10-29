use dioxus::prelude::*;
use tracing::{error, info};

use crate::can::{self, CanManager};
use crate::state::{AppState, ConnectionState};

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

    // Refresh interfaces button
    let on_refresh_interfaces = move |_| {
        spawn(async move {
            refresh_interfaces(app_state).await;
        });
    };

    // Setup SLCAN button
    let on_setup_slcan = move |_| {
        spawn(async move {
            setup_slcan_dialog(app_state).await;
        });
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

                        if state.available_interfaces.is_empty() {
                            option { value: "can0", "can0" }
                            option { value: "vcan0", "vcan0" }
                            option { value: "slcan0", "slcan0" }
                        } else {
                            for interface in &state.available_interfaces {
                                {
                                    let status = if interface.is_up { "(UP)" } else { "(DOWN)" };
                                    let display_text = format!("{} {}", interface.name, status);
                                    rsx! {
                                        option {
                                            value: "{interface.name}",
                                            "{display_text}"
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // Refresh button
                button {
                    style: "padding: 6px 12px; border: 1px solid #007bff; background: white; color: #007bff; cursor: pointer; border-radius: 4px; font-size: 13px;",
                    onclick: on_refresh_interfaces,
                    disabled: matches!(state.connection_state, ConnectionState::Connecting),
                    "🔄 Refresh"
                }

                // Setup SLCAN button
                button {
                    style: "padding: 6px 12px; border: 1px solid #28a745; background: white; color: #28a745; cursor: pointer; border-radius: 4px; font-size: 13px;",
                    onclick: on_setup_slcan,
                    disabled: matches!(state.connection_state, ConnectionState::Connected | ConnectionState::Connecting),
                    "⚙️ Setup SLCAN"
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
            }

            // USB devices info bar (only show if devices detected)
            if !state.available_usb_devices.is_empty() {
                div {
                    style: "padding: 8px 20px; background: #e3f2fd; border-top: 1px solid #90caf9; font-size: 13px;",
                    span {
                        style: "color: #1976d2; font-weight: 500;",
                        "📡 USB-CAN Devices: "
                    }
                    for (idx, device) in state.available_usb_devices.iter().enumerate() {
                        if idx > 0 {
                            span { ", " }
                        }
                        span {
                            style: "color: #555;",
                            "{device.device_path}"
                        }
                    }
                }
            }

            // Error message
            if let ConnectionState::Error(msg) = &state.connection_state {
                div {
                    style: "padding: 8px 20px; background: #ffebee; border-top: 1px solid #ef5350; color: #c62828; font-size: 13px;",
                    "❌ Error: {msg}"
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

/// Setup SLCAN interface from USB device
async fn setup_slcan_dialog(mut app_state: Signal<AppState>) {
    info!("Setting up SLCAN interface");

    let usb_devices = app_state.read().available_usb_devices.clone();

    if usb_devices.is_empty() {
        error!("No USB-CAN devices found");
        app_state.write().connection_state =
            ConnectionState::Error("No USB-CAN devices found. Please connect a CANUSB adapter.".to_string());
        return;
    }

    // Use first available device
    let device = &usb_devices[0];
    info!("Using device: {}", device.device_path);

    // Setup slcan0 interface at 250kbps (matching firmware config)
    let device_path = device.device_path.clone();
    match can::setup_slcan_interface(&device_path, "slcan0", 250000) {
        Ok(_) => {
            info!("SLCAN interface setup successful");

            // Refresh interfaces to show the new slcan0
            refresh_interfaces(app_state).await;

            // Automatically select slcan0
            app_state.write().interface = "slcan0".to_string();
        }
        Err(e) => {
            error!("SLCAN setup failed: {}", e);
            app_state.write().connection_state =
                ConnectionState::Error(format!("SLCAN setup failed: {}. Try running with sudo privileges.", e));
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
