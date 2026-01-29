//! Output module
//!
//! Data export and visualization:
//! - CSV data export (feature-gated)
//! - Plot generation (feature-gated)

#[cfg(feature = "csv-output")]
mod csv;
#[cfg(feature = "visualization")]
mod plotting;

#[cfg(feature = "csv-output")]
pub use csv::{write_to_csv, CsvWriter};

#[cfg(feature = "visualization")]
pub use plotting::{plot_multi_panel, plot_speed, PlotConfig};
