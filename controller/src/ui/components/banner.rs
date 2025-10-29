use dioxus::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BannerType {
    Info,
    Warning,
    Error,
    Success,
}

impl BannerType {
    fn get_style(&self) -> &'static str {
        match self {
            BannerType::Info => "padding: 12px; background: #e7f3ff; border-left: 4px solid #007bff; border-radius: 4px;",
            BannerType::Warning => "padding: 15px; background: #fff3cd; border: 1px solid #ffc107; border-radius: 4px; color: #856404;",
            BannerType::Error => "padding: 15px; background: #f8d7da; border: 1px solid #f5c6cb; border-radius: 4px; color: #721c24;",
            BannerType::Success => "padding: 12px; background: #e7f9ed; border-left: 4px solid #28a745; border-radius: 4px;",
        }
    }

    fn get_icon(&self) -> &'static str {
        match self {
            BannerType::Info => "ℹ",
            BannerType::Warning => "⚠",
            BannerType::Error => "❌",
            BannerType::Success => "✓",
        }
    }
}

#[component]
pub fn Banner(
    #[props(default = BannerType::Info)] banner_type: BannerType,
    message: String,
) -> Element {
    let style = banner_type.get_style();
    let icon = banner_type.get_icon();

    rsx! {
        div {
            style: "{style}",
            p {
                style: "margin: 0; font-size: 14px; color: #555;",
                "{icon} {message}"
            }
        }
    }
}

#[component]
pub fn WarningBanner(message: String) -> Element {
    rsx! {
        Banner {
            banner_type: BannerType::Warning,
            message
        }
    }
}

#[component]
pub fn ErrorBanner(message: String) -> Element {
    rsx! {
        Banner {
            banner_type: BannerType::Error,
            message
        }
    }
}
