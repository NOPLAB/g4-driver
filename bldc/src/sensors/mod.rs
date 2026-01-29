//! Sensor processing algorithms
//!
//! This module provides algorithms for processing sensor data
//! for motor position and speed estimation.

pub mod hall;

pub use hall::{HallConfig, HallProcessor, HallResult};
