//! SLCAN (Serial Line CAN) implementation
//!
//! This module provides cross-platform CAN communication over serial ports
//! using the SLCAN protocol. It replaces the Linux-specific tokio-socketcan
//! dependency.
//!
//! ## Usage
//!
//! ```ignore
//! use crate::can::slcan::{SlcanStream, SlcanBitrate, CanFrame};
//!
//! // Connect to adapter
//! let mut stream = SlcanStream::connect("/dev/ttyACM0", SlcanBitrate::B250K).await?;
//!
//! // Send a frame
//! let frame = CanFrame::new(0x100, &[0x01, 0x02, 0x03, 0x04]).unwrap();
//! stream.send_frame(&frame).await?;
//!
//! // Receive a frame
//! if let Some(frame) = stream.receive_frame(100).await? {
//!     println!("Received: ID=0x{:X}", frame.id());
//! }
//!
//! // Disconnect
//! stream.disconnect().await?;
//! ```

pub mod frame;
pub mod protocol;
pub mod stream;

pub use frame::CanFrame;
pub use protocol::SlcanBitrate;
pub use stream::SlcanStream;
