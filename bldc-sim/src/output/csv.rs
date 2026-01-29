//! CSV output for simulation data

use crate::motor_model::StateSnapshot;
use std::fs::File;
use std::io::{self, Write};
use std::path::Path;

/// CSV writer for simulation data
pub struct CsvWriter {
    file: File,
    header_written: bool,
}

impl CsvWriter {
    /// Create a new CSV writer
    pub fn new<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let file = File::create(path)?;
        Ok(Self {
            file,
            header_written: false,
        })
    }

    /// Write header row
    fn write_header(&mut self) -> io::Result<()> {
        if !self.header_written {
            writeln!(
                self.file,
                "time_s,speed_rpm,omega_m_rad_s,theta_m_rad,theta_e_rad,i_d_A,i_q_A,torque_Nm,rotations"
            )?;
            self.header_written = true;
        }
        Ok(())
    }

    /// Write a single snapshot
    pub fn write_snapshot(&mut self, snapshot: &StateSnapshot) -> io::Result<()> {
        self.write_header()?;
        writeln!(
            self.file,
            "{:.6},{:.3},{:.3},{:.6},{:.6},{:.4},{:.4},{:.6},{}",
            snapshot.time,
            snapshot.speed_rpm,
            snapshot.omega_m,
            snapshot.theta_m,
            snapshot.theta_e,
            snapshot.i_d,
            snapshot.i_q,
            snapshot.torque,
            snapshot.rotations,
        )
    }

    /// Write all snapshots from history
    pub fn write_history(&mut self, history: &[StateSnapshot]) -> io::Result<()> {
        for snapshot in history {
            self.write_snapshot(snapshot)?;
        }
        Ok(())
    }

    /// Flush and close the file
    pub fn finish(mut self) -> io::Result<()> {
        self.file.flush()
    }
}

/// Write simulation history to CSV file
pub fn write_to_csv<P: AsRef<Path>>(
    path: P,
    history: &[StateSnapshot],
) -> io::Result<()> {
    let mut writer = CsvWriter::new(path)?;
    writer.write_history(history)?;
    writer.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn test_csv_write() {
        let history = vec![
            StateSnapshot {
                time: 0.0,
                i_d: 0.0,
                i_q: 1.0,
                theta_m: 0.0,
                omega_m: 0.0,
                theta_e: 0.0,
                speed_rpm: 0.0,
                torque: 0.0,
                rotations: 0,
            },
            StateSnapshot {
                time: 0.001,
                i_d: 0.1,
                i_q: 1.0,
                theta_m: 0.1,
                omega_m: 10.0,
                theta_e: 0.6,
                speed_rpm: 95.5,
                torque: 0.05,
                rotations: 0,
            },
        ];

        let path = "/tmp/bldc_sim_test.csv";
        write_to_csv(path, &history).unwrap();

        // Verify file exists and has content
        let content = fs::read_to_string(path).unwrap();
        assert!(content.contains("time_s,speed_rpm"));
        assert!(content.contains("0.000000")); // First time
        assert!(content.contains("0.001000")); // Second time

        // Cleanup
        fs::remove_file(path).ok();
    }
}
