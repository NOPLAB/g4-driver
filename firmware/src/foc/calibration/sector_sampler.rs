//! セクターサンプリング
//!
//! 各Hallセクターでの角度測定を管理します。

use crate::fmt::*;

/// 各セクターでのサンプル数
const SAMPLES_PER_SECTOR: u8 = 10;

/// 角度安定化のための待機サイクル数（50ms @ 10kHz）
const STABILIZATION_CYCLES: u32 = 500;

/// Hallセクター角度サンプラー
///
/// 各Hallセクター（1-6）での角度をサンプリングして平均化します。
pub struct SectorSampler {
    /// 各Hallセクターでの角度記録 [rad]（インデックス0は未使用、1-6がセクター1-6）
    sector_angles: [Option<f32>; 7],
    /// 各セクターのサンプル合計 [rad]
    sector_sample_sum: [f32; 7],
    /// 各セクターのサンプル数
    sector_sample_count: [u8; 7],
    /// 前回のHallセクター（セクター遷移検出用）
    prev_hall_sector: u8,
    /// 現在のセクターでの待機カウンター（角度安定化のため）
    sector_wait_counter: u32,
}

impl SectorSampler {
    /// 新しいサンプラーを作成
    pub fn new() -> Self {
        Self {
            sector_angles: [None; 7],
            sector_sample_sum: [0.0; 7],
            sector_sample_count: [0; 7],
            prev_hall_sector: 0,
            sector_wait_counter: 0,
        }
    }

    /// サンプラーをリセット
    pub fn reset(&mut self) {
        self.sector_angles = [None; 7];
        self.sector_sample_sum = [0.0; 7];
        self.sector_sample_count = [0; 7];
        self.prev_hall_sector = 0;
        self.sector_wait_counter = 0;
    }

    /// 角度サンプルを記録
    ///
    /// # Arguments
    /// * `hall_sector` - 現在のHallセクター（1-6）
    /// * `angle` - 現在の角度 [rad]
    ///
    /// # Returns
    /// セクターの記録が完了した場合は`true`
    pub fn record_sample(&mut self, hall_sector: u8, angle: f32) -> bool {
        // 有効なHallセクターかチェック
        if !(1..=6).contains(&hall_sector) {
            return false;
        }

        // セクターが変わったかチェック
        if hall_sector != self.prev_hall_sector {
            info!(
                "Calibration: Entered Hall sector {}, waiting for stabilization...",
                hall_sector
            );
            self.prev_hall_sector = hall_sector;
            self.sector_wait_counter = 0;
            return false;
        }

        let sector_idx = hall_sector as usize;

        // 既に十分なサンプルがある場合はスキップ
        if self.sector_sample_count[sector_idx] >= SAMPLES_PER_SECTOR {
            return false;
        }

        // 角度安定化のため待機
        if self.sector_wait_counter < STABILIZATION_CYCLES {
            self.sector_wait_counter += 1;
            return false;
        }

        // サンプリング：角度を蓄積
        self.sector_sample_sum[sector_idx] += angle;
        self.sector_sample_count[sector_idx] += 1;

        // 目標サンプル数に達したら平均を計算
        if self.sector_sample_count[sector_idx] >= SAMPLES_PER_SECTOR {
            let avg_angle =
                self.sector_sample_sum[sector_idx] / self.sector_sample_count[sector_idx] as f32;
            self.sector_angles[sector_idx] = Some(avg_angle);
            info!(
                "Calibration: Recorded angle for sector {} ({} samples): {} rad ({} deg)",
                hall_sector,
                SAMPLES_PER_SECTOR,
                avg_angle,
                avg_angle * 180.0 / core::f32::consts::PI
            );
            return true;
        }

        false
    }

    /// 全セクターの記録が完了したかチェック
    pub fn is_complete(&self) -> bool {
        (1..=6).all(|i| self.sector_angles[i].is_some())
    }

    /// 記録された角度の配列を取得
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
        // セクター1に入る
        sampler.record_sample(1, 1.0);
        assert_eq!(sampler.sector_wait_counter, 0);

        // 待機カウンターを進める
        for _ in 0..100 {
            sampler.record_sample(1, 1.0);
        }
        let counter_before = sampler.sector_wait_counter;

        // セクター2に変更
        sampler.record_sample(2, 1.0);
        assert_eq!(sampler.sector_wait_counter, 0);
        assert!(counter_before > 0);
    }
}
