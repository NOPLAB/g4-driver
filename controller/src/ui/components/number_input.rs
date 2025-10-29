use dioxus::prelude::*;

/// Inline F32 input (no label/description, for inline layouts)
#[component]
pub fn F32InputInline(
    value: f32,
    on_change: EventHandler<f32>,
    disabled: bool,
    style: String,
    placeholder: Option<String>,
) -> Element {
    let mut text_value = use_signal(|| value.to_string());
    let mut is_valid = use_signal(|| true);

    use_effect(move || {
        text_value.set(value.to_string());
    });

    let on_input = move |evt: Event<FormData>| {
        let input = evt.value();
        text_value.set(input.clone());

        if input.is_empty() {
            is_valid.set(false);
            return;
        }

        if let Ok(parsed) = input.parse::<f32>() {
            is_valid.set(true);
            on_change.call(parsed);
        } else {
            is_valid.set(false);
        }
    };

    let on_blur = move |_| {
        if !is_valid() || text_value().is_empty() {
            text_value.set(value.to_string());
            is_valid.set(true);
        }
    };

    let input_style = if !is_valid() {
        format!("{} border: 2px solid #dc3545; background: #fff5f5;", style)
    } else {
        style
    };

    rsx! {
        input {
            r#type: "text",
            value: "{text_value}",
            disabled: disabled,
            style: "{input_style}",
            oninput: on_input,
            onblur: on_blur,
            placeholder: placeholder.unwrap_or_default(),
        }
    }
}


/// F32 number input component with step support
#[component]
pub fn F32Input(
    label: String,
    value: f32,
    step: String,
    on_change: EventHandler<f32>,
    is_connected: bool,
    description: String,
) -> Element {
    let mut text_value = use_signal(|| value.to_string());
    let mut is_valid = use_signal(|| true);

    use_effect(move || {
        text_value.set(value.to_string());
    });

    let on_input = move |evt: Event<FormData>| {
        let input = evt.value();
        text_value.set(input.clone());

        if input.is_empty() {
            is_valid.set(false);
            return;
        }

        if let Ok(parsed) = input.parse::<f32>() {
            is_valid.set(true);
            on_change.call(parsed);
        } else {
            is_valid.set(false);
        }
    };

    let on_blur = move |_| {
        if !is_valid() || text_value().is_empty() {
            text_value.set(value.to_string());
            is_valid.set(true);
        }
    };

    let input_style = if !is_valid() {
        "width: 100%; padding: 8px 12px; border: 2px solid #dc3545; border-radius: 4px; font-size: 14px; background: #fff5f5;"
    } else {
        "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;"
    };

    rsx! {
        div {
            label {
                style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;",
                "{label}"
            }
            input {
                r#type: "text",
                value: "{text_value}",
                disabled: !is_connected,
                style: "{input_style}",
                oninput: on_input,
                onblur: on_blur,
                placeholder: "Enter value...",
            }
            p {
                style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                "{description}"
            }
        }
    }
}

/// U8 number input component
#[component]
pub fn U8Input(
    label: String,
    value: u8,
    on_change: EventHandler<u8>,
    is_connected: bool,
    description: String,
) -> Element {
    let mut text_value = use_signal(|| value.to_string());
    let mut is_valid = use_signal(|| true);

    use_effect(move || {
        text_value.set(value.to_string());
    });

    let on_input = move |evt: Event<FormData>| {
        let input = evt.value();
        text_value.set(input.clone());

        if input.is_empty() {
            is_valid.set(false);
            return;
        }

        if let Ok(parsed) = input.parse::<u8>() {
            is_valid.set(true);
            on_change.call(parsed);
        } else {
            is_valid.set(false);
        }
    };

    let on_blur = move |_| {
        if !is_valid() || text_value().is_empty() {
            text_value.set(value.to_string());
            is_valid.set(true);
        }
    };

    let input_style = if !is_valid() {
        "width: 100%; padding: 8px 12px; border: 2px solid #dc3545; border-radius: 4px; font-size: 14px; background: #fff5f5;"
    } else {
        "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;"
    };

    rsx! {
        div {
            label {
                style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;",
                "{label}"
            }
            input {
                r#type: "text",
                value: "{text_value}",
                disabled: !is_connected,
                style: "{input_style}",
                oninput: on_input,
                onblur: on_blur,
                placeholder: "Enter value...",
            }
            p {
                style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                "{description}"
            }
        }
    }
}

/// U16 number input component
#[component]
pub fn U16Input(
    label: String,
    value: u16,
    on_change: EventHandler<u16>,
    is_connected: bool,
    description: String,
) -> Element {
    let mut text_value = use_signal(|| value.to_string());
    let mut is_valid = use_signal(|| true);

    use_effect(move || {
        text_value.set(value.to_string());
    });

    let on_input = move |evt: Event<FormData>| {
        let input = evt.value();
        text_value.set(input.clone());

        if input.is_empty() {
            is_valid.set(false);
            return;
        }

        if let Ok(parsed) = input.parse::<u16>() {
            is_valid.set(true);
            on_change.call(parsed);
        } else {
            is_valid.set(false);
        }
    };

    let on_blur = move |_| {
        if !is_valid() || text_value().is_empty() {
            text_value.set(value.to_string());
            is_valid.set(true);
        }
    };

    let input_style = if !is_valid() {
        "width: 100%; padding: 8px 12px; border: 2px solid #dc3545; border-radius: 4px; font-size: 14px; background: #fff5f5;"
    } else {
        "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;"
    };

    rsx! {
        div {
            label {
                style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;",
                "{label}"
            }
            input {
                r#type: "text",
                value: "{text_value}",
                disabled: !is_connected,
                style: "{input_style}",
                oninput: on_input,
                onblur: on_blur,
                placeholder: "Enter value...",
            }
            p {
                style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                "{description}"
            }
        }
    }
}

/// U32 number input component
#[component]
pub fn U32Input(
    label: String,
    value: u32,
    on_change: EventHandler<u32>,
    is_connected: bool,
    description: String,
) -> Element {
    let mut text_value = use_signal(|| value.to_string());
    let mut is_valid = use_signal(|| true);

    use_effect(move || {
        text_value.set(value.to_string());
    });

    let on_input = move |evt: Event<FormData>| {
        let input = evt.value();
        text_value.set(input.clone());

        if input.is_empty() {
            is_valid.set(false);
            return;
        }

        if let Ok(parsed) = input.parse::<u32>() {
            is_valid.set(true);
            on_change.call(parsed);
        } else {
            is_valid.set(false);
        }
    };

    let on_blur = move |_| {
        if !is_valid() || text_value().is_empty() {
            text_value.set(value.to_string());
            is_valid.set(true);
        }
    };

    let input_style = if !is_valid() {
        "width: 100%; padding: 8px 12px; border: 2px solid #dc3545; border-radius: 4px; font-size: 14px; background: #fff5f5;"
    } else {
        "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;"
    };

    rsx! {
        div {
            label {
                style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;",
                "{label}"
            }
            input {
                r#type: "text",
                value: "{text_value}",
                disabled: !is_connected,
                style: "{input_style}",
                oninput: on_input,
                onblur: on_blur,
                placeholder: "Enter value...",
            }
            p {
                style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                "{description}"
            }
        }
    }
}

/// U64 number input component
#[component]
pub fn U64Input(
    label: String,
    value: u64,
    on_change: EventHandler<u64>,
    is_connected: bool,
    description: String,
) -> Element {
    let mut text_value = use_signal(|| value.to_string());
    let mut is_valid = use_signal(|| true);

    use_effect(move || {
        text_value.set(value.to_string());
    });

    let on_input = move |evt: Event<FormData>| {
        let input = evt.value();
        text_value.set(input.clone());

        if input.is_empty() {
            is_valid.set(false);
            return;
        }

        if let Ok(parsed) = input.parse::<u64>() {
            is_valid.set(true);
            on_change.call(parsed);
        } else {
            is_valid.set(false);
        }
    };

    let on_blur = move |_| {
        if !is_valid() || text_value().is_empty() {
            text_value.set(value.to_string());
            is_valid.set(true);
        }
    };

    let input_style = if !is_valid() {
        "width: 100%; padding: 8px 12px; border: 2px solid #dc3545; border-radius: 4px; font-size: 14px; background: #fff5f5;"
    } else {
        "width: 100%; padding: 8px 12px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px;"
    };

    rsx! {
        div {
            label {
                style: "font-size: 14px; font-weight: 500; color: #555; display: block; margin-bottom: 8px;",
                "{label}"
            }
            input {
                r#type: "text",
                value: "{text_value}",
                disabled: !is_connected,
                style: "{input_style}",
                oninput: on_input,
                onblur: on_blur,
                placeholder: "Enter value...",
            }
            p {
                style: "margin: 4px 0 0 0; font-size: 12px; color: #666;",
                "{description}"
            }
        }
    }
}
