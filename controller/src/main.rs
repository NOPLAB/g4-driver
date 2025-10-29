mod can;
mod state;
mod ui;

use dioxus::prelude::*;
use dioxus::desktop::{Config, WindowBuilder};
use state::AppState;
use tracing_subscriber;

fn main() {
    // Initialize tracing
    tracing_subscriber::fmt()
        .with_env_filter("g4_driver_controller=debug,info")
        .init();

    // Launch the Dioxus app with custom window title
    dioxus::LaunchBuilder::desktop()
        .with_cfg(Config::new().with_window(
            WindowBuilder::new()
                .with_title("G4 Driver")
        ))
        .launch(App);
}

#[component]
fn App() -> Element {
    // Initialize application state
    use_context_provider(|| Signal::new(AppState::new()));

    rsx! {
        div {
            style: "display: flex; flex-direction: column; height: 100vh; font-family: sans-serif; margin: 0; padding: 0;",
            ui::ConnectionBar {}
            div {
                style: "flex: 1; padding: 20px; overflow: auto;",
                ui::MainContent {}
            }
        }
    }
}
