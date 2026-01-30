//! SLCAN serial stream wrapper
//!
//! Provides async CAN communication over a serial port using the SLCAN protocol.

use anyhow::{Context, Result};
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader, BufWriter};
use tokio::time::timeout;
use tokio_serial::SerialPortBuilderExt;
use tracing::{debug, error, info, warn};

use super::frame::CanFrame;
use super::protocol::{self, SlcanBitrate};

/// Default serial port baud rate for SLCAN adapters
const DEFAULT_BAUD_RATE: u32 = 115200;

/// SLCAN stream for async CAN communication
pub struct SlcanStream {
    /// Serial port path (e.g., "/dev/ttyACM0" or "COM3")
    #[allow(dead_code)]
    port_path: String,
    /// Buffered reader/writer for the serial port
    reader: BufReader<tokio::io::ReadHalf<tokio_serial::SerialStream>>,
    writer: BufWriter<tokio::io::WriteHalf<tokio_serial::SerialStream>>,
    /// CAN bitrate
    bitrate: SlcanBitrate,
    /// Line buffer for reading responses
    line_buffer: String,
}

impl SlcanStream {
    /// Connect to an SLCAN adapter
    ///
    /// # Arguments
    /// * `port_path` - Serial port path (e.g., "/dev/ttyACM0", "COM3")
    /// * `bitrate` - CAN bus bitrate
    ///
    /// # Returns
    /// Connected SlcanStream or error
    pub async fn connect(port_path: &str, bitrate: SlcanBitrate) -> Result<Self> {
        info!(
            "Connecting to SLCAN adapter at {} with bitrate {} bps",
            port_path,
            bitrate.to_bps()
        );

        // Open serial port
        let port = tokio_serial::new(port_path, DEFAULT_BAUD_RATE)
            .timeout(Duration::from_millis(100))
            .open_native_async()
            .with_context(|| format!("Failed to open serial port: {}", port_path))?;

        // Split into read/write halves
        let (read_half, write_half) = tokio::io::split(port);

        let mut stream = Self {
            port_path: port_path.to_string(),
            reader: BufReader::new(read_half),
            writer: BufWriter::new(write_half),
            bitrate,
            line_buffer: String::with_capacity(64),
        };

        // Send initialization sequence
        stream.initialize().await?;

        info!("SLCAN adapter connected successfully");
        Ok(stream)
    }

    /// Initialize the SLCAN adapter
    async fn initialize(&mut self) -> Result<()> {
        // Clear any pending data
        self.flush_input().await;

        // Send init sequence
        let init_cmds = protocol::init_sequence(self.bitrate);

        for cmd in init_cmds {
            debug!("Sending SLCAN command: {:?}", cmd.trim());

            self.writer
                .write_all(cmd.as_bytes())
                .await
                .context("Failed to write init command")?;

            self.writer.flush().await.context("Failed to flush")?;

            // Wait for response
            tokio::time::sleep(Duration::from_millis(50)).await;

            // Read response (optional, some adapters don't respond)
            if let Ok(response) = self.read_response(100).await {
                debug!("SLCAN response: {:?}", response);
                if protocol::is_error_response(&response) {
                    warn!("SLCAN init command got error response");
                }
            }
        }

        Ok(())
    }

    /// Flush input buffer
    async fn flush_input(&mut self) {
        // Read and discard any pending data
        loop {
            self.line_buffer.clear();
            match timeout(
                Duration::from_millis(10),
                self.reader.read_line(&mut self.line_buffer),
            )
            .await
            {
                Ok(Ok(0)) | Err(_) => break,
                Ok(Ok(_)) => continue,
                Ok(Err(_)) => break,
            }
        }
    }

    /// Read a response line with timeout
    async fn read_response(&mut self, timeout_ms: u64) -> Result<String> {
        self.line_buffer.clear();

        match timeout(
            Duration::from_millis(timeout_ms),
            self.reader.read_line(&mut self.line_buffer),
        )
        .await
        {
            Ok(Ok(0)) => Err(anyhow::anyhow!("Serial port closed")),
            Ok(Ok(_)) => Ok(self.line_buffer.clone()),
            Ok(Err(e)) => Err(e.into()),
            Err(_) => Err(anyhow::anyhow!("Read timeout")),
        }
    }

    /// Disconnect from the SLCAN adapter
    pub async fn disconnect(&mut self) -> Result<()> {
        info!("Disconnecting from SLCAN adapter");

        // Send close sequence
        for cmd in protocol::close_sequence() {
            let _ = self.writer.write_all(cmd.as_bytes()).await;
        }
        let _ = self.writer.flush().await;

        info!("SLCAN adapter disconnected");
        Ok(())
    }

    /// Send a CAN frame
    ///
    /// # Arguments
    /// * `frame` - CAN frame to send
    pub async fn send_frame(&mut self, frame: &CanFrame) -> Result<()> {
        let encoded = protocol::encode_frame(frame);
        debug!(
            "Sending CAN frame: ID=0x{:X}, len={}, encoded={:?}",
            frame.id(),
            frame.len(),
            encoded.trim()
        );

        self.writer
            .write_all(encoded.as_bytes())
            .await
            .context("Failed to write frame")?;

        self.writer.flush().await.context("Failed to flush")?;

        Ok(())
    }

    /// Receive a CAN frame with timeout
    ///
    /// # Arguments
    /// * `timeout_ms` - Timeout in milliseconds
    ///
    /// # Returns
    /// * `Ok(Some(frame))` if frame received
    /// * `Ok(None)` if timeout
    /// * `Err` on error
    pub async fn receive_frame(&mut self, timeout_ms: u64) -> Result<Option<CanFrame>> {
        self.line_buffer.clear();

        match timeout(
            Duration::from_millis(timeout_ms),
            self.reader.read_line(&mut self.line_buffer),
        )
        .await
        {
            Ok(Ok(0)) => {
                // EOF - connection closed
                Err(anyhow::anyhow!("Serial port closed"))
            }
            Ok(Ok(_)) => {
                let line = self.line_buffer.trim();

                // Skip empty lines and error responses
                if line.is_empty() || protocol::is_error_response(line) {
                    return Ok(None);
                }

                // Try to decode as CAN frame
                match protocol::decode_frame(line) {
                    Some(frame) => {
                        debug!(
                            "Received CAN frame: ID=0x{:X}, len={}",
                            frame.id(),
                            frame.len()
                        );
                        Ok(Some(frame))
                    }
                    None => {
                        // Not a frame, might be a response to a command
                        debug!("Received non-frame data: {:?}", line);
                        Ok(None)
                    }
                }
            }
            Ok(Err(e)) => {
                error!("Serial read error: {}", e);
                Err(e.into())
            }
            Err(_) => {
                // Timeout
                Ok(None)
            }
        }
    }

    /// Get the port path
    #[allow(dead_code)]
    pub fn port_path(&self) -> &str {
        &self.port_path
    }

    /// Get the CAN bitrate
    #[allow(dead_code)]
    pub fn bitrate(&self) -> SlcanBitrate {
        self.bitrate
    }
}
