// FOC (Field Oriented Control) module
// Hall sensor-based FOC implementation for BLDC motor control

pub mod hall_sensor;
pub mod pi_controller;
pub mod svpwm;
pub mod transforms;

// Re-export main types for easier access
pub use hall_sensor::HallSensor;
pub use pi_controller::PiController;
pub use svpwm::calculate_svpwm;
pub use transforms::{inverse_park, limit_voltage};
