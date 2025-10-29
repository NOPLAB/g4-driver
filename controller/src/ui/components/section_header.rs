use dioxus::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HeaderColor {
    Blue,
    Green,
}

impl HeaderColor {
    fn get_color(&self) -> &'static str {
        match self {
            HeaderColor::Blue => "#007bff",
            HeaderColor::Green => "#28a745",
        }
    }
}

#[component]
pub fn SectionHeader(
    title: String,
    #[props(default = HeaderColor::Blue)] color: HeaderColor,
) -> Element {
    let border_color = color.get_color();

    rsx! {
        h2 {
            style: "margin: 0 0 15px 0; font-size: 20px; color: #333; border-bottom: 2px solid {border_color}; padding-bottom: 10px;",
            "{title}"
        }
    }
}
