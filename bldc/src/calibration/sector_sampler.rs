//! Sector sampling
//!
//! Manages angle measurements for each Hall sector.

/// Number of samples per sector
const SAMPLES_PER_SECTOR: u8 = 10;

/// Wait cycles for angle stabilization (50ms @ 10kHz)
const STABILIZATION_CYCLES: u32 = 500;

/// Hall sector angle sampler
///
/// Samples and averages angles at each Hall sector (1-6).
pub struct SectorSampler {
    /// Recorded angles for each Hall sector [rad] (index 0 unused, 1-6 are sectors 1-6)
    sector_angles: [Option<f32>; 7],
    /// Sample sum for each sector [rad]
    sector_sample_sum: [f32; 7],
    /// Sample count for each sector
    sector_sample_count: [u8; 7],
    /// Previous Hall sector (for sector transition detection)
    prev_hall_sector: u8,
    /// Wait counter in current sector (for angle stabilization)
    sector_wait_counter: u32,
}

impl SectorSampler {
    /// Create new sampler
    pub fn new() -> Self {
        Self {
            sector_angles: [None; 7],
            sector_sample_sum: [0.0; 7],
            sector_sample_count: [0; 7],
            prev_hall_sector: 0,
            sector_wait_counter: 0,
        }
    }

    /// Reset sampler
    pub fn reset(&mut self) {
        self.sector_angles = [None; 7];
        self.sector_sample_sum = [0.0; 7];
        self.sector_sample_count = [0; 7];
        self.prev_hall_sector = 0;
        self.sector_wait_counter = 0;
    }

    /// Record angle sample
    ///
    /// # Arguments
    /// * `hall_sector` - Current Hall sector (1-6)
    /// * `angle` - Current angle [rad]
    ///
    /// # Returns
    /// `true` if sector recording is complete
    pub fn record_sample(&mut self, hall_sector: u8, angle: f32) -> bool {
        // Check if valid Hall sector
        if !(1..=6).contains(&hall_sector) {
            return false;
        }

        // Check if sector changed
        if hall_sector != self.prev_hall_sector {
            self.prev_hall_sector = hall_sector;
            self.sector_wait_counter = 0;
            return false;
        }

        let sector_idx = hall_sector as usize;

        // Skip if already have enough samples
        if self.sector_sample_count[sector_idx] >= SAMPLES_PER_SECTOR {
            return false;
        }

        // Wait for angle stabilization
        if self.sector_wait_counter < STABILIZATION_CYCLES {
            self.sector_wait_counter += 1;
            return false;
        }

        // Sampling: accumulate angle
        self.sector_sample_sum[sector_idx] += angle;
        self.sector_sample_count[sector_idx] += 1;

        // Calculate average when target sample count reached
        if self.sector_sample_count[sector_idx] >= SAMPLES_PER_SECTOR {
            let avg_angle =
                self.sector_sample_sum[sector_idx] / self.sector_sample_count[sector_idx] as f32;
            self.sector_angles[sector_idx] = Some(avg_angle);
            return true;
        }

        false
    }

    /// Check if all sectors are recorded
    pub fn is_complete(&self) -> bool {
        (1..=6).all(|i| self.sector_angles[i].is_some())
    }

    /// Get recorded angles array
    pub fn get_angles(&self) -> &[Option<f32>; 7] {
        &self.sector_angles
    }
}

impl Default for SectorSampler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_sampler() {
        let sampler = SectorSampler::new();
        assert!(!sampler.is_complete());
    }

    #[test]
    fn test_invalid_sector() {
        let mut sampler = SectorSampler::new();
        assert!(!sampler.record_sample(0, 1.0));
        assert!(!sampler.record_sample(7, 1.0));
    }

    #[test]
    fn test_sector_change_resets_counter() {
        let mut sampler = SectorSampler::new();
        // Enter sector 1
        sampler.record_sample(1, 1.0);
        assert_eq!(sampler.sector_wait_counter, 0);

        // Advance wait counter
        for _ in 0..100 {
            sampler.record_sample(1, 1.0);
        }
        let counter_before = sampler.sector_wait_counter;

        // Change to sector 2
        sampler.record_sample(2, 1.0);
        assert_eq!(sampler.sector_wait_counter, 0);
        assert!(counter_before > 0);
    }
}
