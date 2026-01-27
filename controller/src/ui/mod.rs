mod components;
mod connection;
mod control;
mod settings;

pub use connection::ConnectionBar;
use control::ControlPanel;
use settings::SettingsPanel;

use dioxus::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Tab {
    Control,
    Settings,
}

#[component]
pub fn MainContent() -> Element {
    let mut selected_tab = use_signal(|| Tab::Control);

    rsx! {
        div {
            style: "display: flex; flex-direction: column; height: 100%;",

            // Tab buttons
            div {
                style: "display: flex; gap: 10px; margin-bottom: 20px; border-bottom: 2px solid #ddd; padding-bottom: 10px;",
                button {
                    style: if selected_tab() == Tab::Control {
                        "padding: 10px 20px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 4px 4px 0 0; font-size: 14px;"
                    } else {
                        "padding: 10px 20px; border: none; background: #f0f0f0; color: #333; cursor: pointer; border-radius: 4px 4px 0 0; font-size: 14px;"
                    },
                    onclick: move |_| selected_tab.set(Tab::Control),
                    "Control"
                }
                button {
                    style: if selected_tab() == Tab::Settings {
                        "padding: 10px 20px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 4px 4px 0 0; font-size: 14px;"
                    } else {
                        "padding: 10px 20px; border: none; background: #f0f0f0; color: #333; cursor: pointer; border-radius: 4px 4px 0 0; font-size: 14px;"
                    },
                    onclick: move |_| selected_tab.set(Tab::Settings),
                    "Settings"
                }
            }

            // Tab content
            div {
                style: "flex: 1; overflow: auto;",
                match selected_tab() {
                    Tab::Control => rsx! { ControlPanel {} },
                    Tab::Settings => rsx! { SettingsPanel {} },
                }
            }
        }
    }
}
