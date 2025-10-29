use dioxus::prelude::*;

#[component]
pub fn ToggleSwitch(
    label: String,
    checked: bool,
    #[props(default = false)] is_disabled: bool,
    onchange: EventHandler<FormEvent>,
) -> Element {
    let is_connected = !is_disabled;

    rsx! {
        div {
            style: "display: flex; align-items: center; gap: 15px;",
            label {
                style: "font-size: 14px; font-weight: 500; color: #555;",
                "{label}"
            }
            // Toggle switch container
            label {
                style: "position: relative; display: inline-block; width: 60px; height: 34px;",
                input {
                    r#type: "checkbox",
                    checked: checked,
                    onchange: move |evt| onchange.call(evt),
                    disabled: is_disabled,
                    style: "opacity: 0; width: 0; height: 0;",
                }
                // Slider
                span {
                    style: if is_connected {
                        if checked {
                            "position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #28a745; border-radius: 34px; transition: .4s;"
                        } else {
                            "position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 34px; transition: .4s;"
                        }
                    } else {
                        "position: absolute; cursor: not-allowed; top: 0; left: 0; right: 0; bottom: 0; background-color: #e0e0e0; border-radius: 34px; transition: .4s;"
                    },
                    // Slider button
                    span {
                        style: if checked {
                            "position: absolute; content: ''; height: 26px; width: 26px; left: 30px; bottom: 4px; background-color: white; border-radius: 50%; transition: .4s; box-shadow: 0 2px 4px rgba(0,0,0,0.2);"
                        } else {
                            "position: absolute; content: ''; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; border-radius: 50%; transition: .4s; box-shadow: 0 2px 4px rgba(0,0,0,0.2);"
                        }
                    }
                }
            }
            // Status text
            span {
                style: if checked {
                    "font-size: 14px; font-weight: 600; color: #28a745;"
                } else {
                    "font-size: 14px; font-weight: 600; color: #6c757d;"
                },
                if checked { "ON" } else { "OFF" }
            }
        }
    }
}
