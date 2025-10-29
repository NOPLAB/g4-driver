use dioxus::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ButtonVariant {
    Primary,
    Secondary,
    Success,
    Danger,
    Warning,
    Outline,
}

impl ButtonVariant {
    fn get_style(&self, disabled: bool) -> &'static str {
        if disabled {
            return "padding: 10px 20px; border: none; background: #ccc; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 14px; font-weight: 500;";
        }

        match self {
            ButtonVariant::Primary => {
                "padding: 10px 20px; border: none; background: #007bff; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
            }
            ButtonVariant::Secondary => {
                "padding: 10px 20px; border: 1px solid #6c757d; background: white; color: #6c757d; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
            }
            ButtonVariant::Success => {
                "padding: 10px 20px; border: none; background: #28a745; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
            }
            ButtonVariant::Danger => {
                "padding: 10px 20px; border: none; background: #dc3545; color: white; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
            }
            ButtonVariant::Warning => {
                "padding: 10px 20px; border: none; background: #ffc107; color: #212529; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
            }
            ButtonVariant::Outline => {
                "padding: 10px 20px; border: 1px solid #007bff; background: white; color: #007bff; cursor: pointer; border-radius: 4px; font-size: 14px; font-weight: 500;"
            }
        }
    }
}

#[component]
pub fn Button(
    #[props(default = ButtonVariant::Primary)] variant: ButtonVariant,
    #[props(default = false)] disabled: bool,
    #[props(default = "".to_string())] custom_style: String,
    onclick: EventHandler<Event<MouseData>>,
    children: Element,
) -> Element {
    let base_style = variant.get_style(disabled);
    let final_style = if custom_style.is_empty() {
        base_style.to_string()
    } else {
        format!("{} {}", base_style, custom_style)
    };

    rsx! {
        button {
            style: "{final_style}",
            onclick: move |evt| {
                if !disabled {
                    onclick.call(evt);
                }
            },
            disabled,
            {children}
        }
    }
}

#[component]
pub fn EmergencyStopButton(
    #[props(default = false)] disabled: bool,
    onclick: EventHandler<Event<MouseData>>,
) -> Element {
    let style = if disabled {
        "padding: 12px 30px; border: none; background: #ccc; color: #666; cursor: not-allowed; border-radius: 4px; font-size: 16px; font-weight: bold;"
    } else {
        "padding: 12px 30px; border: none; background: #dc3545; color: white; cursor: pointer; border-radius: 4px; font-size: 16px; font-weight: bold; box-shadow: 0 2px 4px rgba(220,53,69,0.3);"
    };

    rsx! {
        button {
            style: "{style}",
            onclick: move |evt| {
                if !disabled {
                    onclick.call(evt);
                }
            },
            disabled,
            "🛑 EMERGENCY STOP"
        }
    }
}
