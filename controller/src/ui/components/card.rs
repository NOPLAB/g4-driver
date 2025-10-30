use dioxus::prelude::*;

#[component]
pub fn Card(#[props(default = "".to_string())] custom_style: String, children: Element) -> Element {
    let base_style = "padding: 20px; background: white; border: 1px solid #ddd; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);";
    let final_style = if custom_style.is_empty() {
        base_style.to_string()
    } else {
        format!("{} {}", base_style, custom_style)
    };

    rsx! {
        div {
            style: "{final_style}",
            {children}
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StatusCardColor {
    Blue,
    Green,
    Yellow,
    Orange,
    Red,
}

impl StatusCardColor {
    fn get_border_color(&self) -> &'static str {
        match self {
            StatusCardColor::Blue => "#007bff",
            StatusCardColor::Green => "#28a745",
            StatusCardColor::Yellow => "#ffc107",
            StatusCardColor::Orange => "#fd7e14",
            StatusCardColor::Red => "#dc3545",
        }
    }
}

#[component]
pub fn StatusCard(
    label: String,
    value: String,
    #[props(default = StatusCardColor::Blue)] color: StatusCardColor,
) -> Element {
    let border_color = color.get_border_color();

    rsx! {
        div {
            style: "padding: 15px; background: #f8f9fa; border-radius: 4px; border-left: 4px solid {border_color};",
            div {
                style: "font-size: 12px; color: #666; margin-bottom: 5px;",
                "{label}"
            }
            div {
                style: "font-size: 24px; font-weight: bold; color: #333;",
                "{value}"
            }
        }
    }
}
