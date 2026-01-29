//! Validation module
//!
//! Performance criteria and metrics for control validation:
//! - Overshoot
//! - Settling time
//! - Steady-state error
//! - Rise time

mod criteria;
mod metrics;

pub use criteria::PerformanceCriteria;
pub use metrics::MetricsCalculator;
