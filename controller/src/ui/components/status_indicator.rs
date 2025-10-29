use dioxus::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StatusColor {
    Gray,
    Orange,
    Green,
    Red,
}

impl StatusColor {
    fn get_hex(&self) -> &'static str {
        match self {
            StatusColor::Gray => "#999",
            StatusColor::Orange => "#ff9800",
            StatusColor::Green => "#4caf50",
            StatusColor::Red => "#f44336",
        }
    }
}

#[component]
pub fn StatusIndicator(
    text: String,
    color: StatusColor,
) -> Element {
    let color_hex = color.get_hex();

    rsx! {
        div {
            style: "display: flex; align-items: center; gap: 8px; padding: 6px 12px; background: white; border-radius: 4px; border: 1px solid #ddd;",
            div {
                style: "width: 12px; height: 12px; border-radius: 50%; background: {color_hex};",
            }
            span {
                style: "font-size: 14px; color: #333;",
                "{text}"
            }
        }
    }
}
