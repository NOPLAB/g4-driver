//! モーター自動キャリブレーションモジュール
//!
//! このモジュールは、モーターの電気角オフセットと回転方向を
//! 自動的に検出するキャリブレーション機能を提供します。

use super::shaft_position::ShaftPosition;
use crate::fmt::*;
use crate::hall_tim;
use core::f32::consts::TAU;

/// キャリブレーション状態
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationState {
    /// 初期化状態
    Init,
    /// 回転方向検出中（モーター方向とセンサー方向の関係を確認）
    FindDirection,
    /// 各Hallセクターでの角度測定中
    MeasureSectors,
    /// 開始位置に戻る
    ReturnToStart,
    /// キャリブレーション完了
    Completed,
}

/// キャリブレーション結果
#[derive(Debug, Clone, Copy)]
pub struct CalibrationResult {
    /// 電気角オフセット [rad]（0～2π）
    pub electrical_offset: f32,
    /// センサー方向反転フラグ（true: モーターと逆方向、false: 同方向）
    pub direction_inversed: bool,
    /// キャリブレーション成功フラグ
    pub success: bool,
}

impl CalibrationResult {
    /// 新しいキャリブレーション結果を作成（失敗状態）
    pub fn new() -> Self {
        Self {
            electrical_offset: 0.0,
            direction_inversed: false,
            success: false,
        }
    }
}

impl Default for CalibrationResult {
    fn default() -> Self {
        Self::new()
    }
}

/// モーター自動キャリブレーション
pub struct MotorCalibration {
    /// 現在の状態
    state: CalibrationState,
    /// 極対数
    pole_pairs: u8,
    /// トルク（0.0～1.0）
    torque: f32,
    /// 要求シャフト位置
    shaft_position_req: ShaftPosition,
    /// 実際のシャフト位置
    shaft_position_act: ShaftPosition,
    /// キャリブレーション結果
    result: CalibrationResult,
    /// 各Hallセクターでの角度記録 [rad]（インデックス0は未使用、1-6がセクター1-6）
    sector_angles: [Option<f32>; 7],
    /// 前回のHallセクター（セクター遷移検出用）
    prev_hall_sector: u8,
    /// 現在のセクターでの待機カウンター（角度安定化のため）
    sector_wait_counter: u32,
}

impl MotorCalibration {
    /// 新しいモーターキャリブレーションを作成
    ///
    /// # 引数
    /// * `pole_pairs` - モーターの極対数
    /// * `torque` - キャリブレーション用トルク（0.0～1.0、推奨: 0.15～0.25）
    pub fn new(pole_pairs: u8, torque: f32) -> Self {
        Self {
            state: CalibrationState::Init,
            pole_pairs,
            torque: torque.clamp(0.1, 0.5), // 安全のため0.1～0.5に制限
            shaft_position_req: ShaftPosition::new(),
            shaft_position_act: ShaftPosition::new(),
            result: CalibrationResult::new(),
            sector_angles: [None; 7],
            prev_hall_sector: 0,
            sector_wait_counter: 0,
        }
    }

    /// キャリブレーションを開始
    pub fn start(&mut self) {
        info!("Starting motor calibration...");
        info!("  Pole pairs: {}", self.pole_pairs);
        info!("  Torque: {}", self.torque);

        self.state = CalibrationState::Init;
        self.shaft_position_req.reset();
        self.shaft_position_act.reset();
        self.result = CalibrationResult::new();
        self.sector_angles = [None; 7];
        self.prev_hall_sector = 0;
        self.sector_wait_counter = 0;
    }

    /// 現在の状態を取得
    #[allow(dead_code)]
    pub fn get_state(&self) -> CalibrationState {
        self.state
    }

    /// キャリブレーション結果を取得
    pub fn get_result(&self) -> CalibrationResult {
        self.result
    }

    /// キャリブレーションが完了したかチェック
    pub fn is_completed(&self) -> bool {
        self.state == CalibrationState::Completed
    }

    /// キャリブレーションステートマシンを更新
    ///
    /// # 引数
    /// * `sensor_angle` - センサーから取得した角度 [rad]
    ///
    /// # 戻り値
    /// * `Ok((electrical_angle, torque))` - 電気角[rad]とトルク（0.0～1.0）
    /// * `Err(())` - エラー（モーターが動かなかった等）
    pub fn update(&mut self, sensor_angle: f32) -> Result<(f32, f32), ()> {
        // 実際のシャフト位置を更新
        self.shaft_position_act.update_shaft_angle(sensor_angle);

        match self.state {
            CalibrationState::Init => {
                info!("Calibration: Init");
                self.shaft_position_req.reset();
                self.shaft_position_act.reset();
                self.result.electrical_offset = 0.0;
                self.sector_angles = [None; 7];
                self.prev_hall_sector = 0;
                self.sector_wait_counter = 0;
                self.state = CalibrationState::FindDirection;
                info!("Calibration: Init -> FindDirection");
                Ok((0.0, 0.0))
            }

            CalibrationState::FindDirection => {
                // デバッグ：定期的に状態をログ出力（2500サイクルごと = 1秒）
                static mut DEBUG_COUNTER_FD: u32 = 0;
                unsafe {
                    DEBUG_COUNTER_FD += 1;
                    if DEBUG_COUNTER_FD >= 2500 {
                        DEBUG_COUNTER_FD = 0;
                        info!(
                            "[Calibration FindDirection] Req: {} rot + {} rad, Act: {} rot + {} rad",
                            self.shaft_position_req.rotations,
                            self.shaft_position_req.angle,
                            self.shaft_position_act.rotations,
                            self.shaft_position_act.angle
                        );
                    }
                }

                // 目標: 1回転以上（1電気角回転）
                if self.shaft_position_req.rotations >= 1 {
                    // モーターが動いたかチェック
                    if self.shaft_position_act.rotations == 0 && self.shaft_position_act.angle < 0.1
                    {
                        error!("Calibration failed: Motor did not move");
                        self.state = CalibrationState::Completed;
                        self.result.success = false;
                        return Err(());
                    }

                    // 回転方向をチェック
                    let actual_position = self.shaft_position_act.get_position();
                    if actual_position < 0.0 {
                        // センサーが逆方向
                        info!("Direction: INVERSED (sensor is reversed)");
                        self.shaft_position_act.set_inversed(true);
                        self.result.direction_inversed = true;
                    } else {
                        info!("Direction: NORMAL");
                        self.shaft_position_act.set_inversed(false);
                        self.result.direction_inversed = false;
                    }

                    self.state = CalibrationState::MeasureSectors;
                    info!("Calibration: FindDirection -> MeasureSectors");
                } else {
                    // ゆっくり回転（5 rad/s ≈ 48 RPM）- より遅く
                    // 2.5kHz更新なので、1ステップあたり: 5 / 2500 = 0.002 rad
                    self.shaft_position_req.increment(0.002);
                }

                // 要求位置の電気角を返す（オフセット未適用）
                let electrical_angle = self.shaft_position_req.get_angle() * self.pole_pairs as f32;
                Ok((electrical_angle, self.torque))
            }

            CalibrationState::MeasureSectors => {
                // 現在のHallセクターを取得（1-6）
                let current_hall = hall_tim::get_hall_state();

                // デバッグ：定期的に状態をログ出力（2500サイクルごと = 1秒）
                static mut DEBUG_COUNTER: u32 = 0;
                unsafe {
                    DEBUG_COUNTER += 1;
                    if DEBUG_COUNTER >= 2500 {
                        DEBUG_COUNTER = 0;
                        let recorded_count =
                            (1..=6).filter(|&i| self.sector_angles[i].is_some()).count();
                        info!(
                            "[Calibration Debug] Hall={}, Req pos={} rad, Act pos={} rad, Recorded: {}/6 sectors",
                            current_hall,
                            self.shaft_position_req.get_position(),
                            self.shaft_position_act.get_position(),
                            recorded_count
                        );
                    }
                }

                // 有効なHallセクターかチェック
                if (1..=6).contains(&current_hall) {
                    // セクターが変わったかチェック
                    if current_hall != self.prev_hall_sector {
                        info!(
                            "Calibration: Entered Hall sector {}, waiting for stabilization...",
                            current_hall
                        );
                        self.prev_hall_sector = current_hall;
                        self.sector_wait_counter = 0;
                    }

                    // 角度安定化のため待機（25サイクル = 10ms @ 2.5kHz）
                    if self.sector_wait_counter < 25 {
                        self.sector_wait_counter += 1;
                    } else if self.sector_angles[current_hall as usize].is_none() {
                        // このセクターの角度をまだ記録していない場合
                        let angle = self.shaft_position_act.get_angle();
                        self.sector_angles[current_hall as usize] = Some(angle);
                        info!(
                            "Calibration: Recorded angle for sector {}: {} rad ({} deg)",
                            current_hall,
                            angle,
                            angle * 180.0 / core::f32::consts::PI
                        );

                        // 全セクターの角度が記録されたかチェック
                        let all_recorded = (1..=6).all(|i| self.sector_angles[i].is_some());
                        if all_recorded {
                            // オフセットを計算
                            self.calculate_offset();
                            self.state = CalibrationState::ReturnToStart;
                            info!("Calibration: MeasureSectors -> ReturnToStart");
                        }
                    }
                }

                // 引き続きゆっくり回転（5 rad/s ≈ 48 RPM）
                self.shaft_position_req.increment(0.002);

                let electrical_angle = self.shaft_position_req.get_angle() * self.pole_pairs as f32;
                Ok((electrical_angle, self.torque))
            }

            CalibrationState::ReturnToStart => {
                // 目標: 0回転、角度 < π/2
                if self.shaft_position_req.rotations == 0
                    && self.shaft_position_req.angle < TAU / 4.0
                {
                    info!("Calibration completed successfully!");
                    info!(
                        "  Electrical offset: {} rad ({} deg)",
                        self.result.electrical_offset,
                        self.result.electrical_offset * 180.0 / core::f32::consts::PI
                    );
                    info!("  Direction inversed: {}", self.result.direction_inversed);

                    self.state = CalibrationState::Completed;
                    self.result.success = true;
                    Ok((0.0, 0.0)) // トルク0で停止
                } else {
                    // 逆方向に回転（開始位置に戻る）
                    self.shaft_position_req.increment(-0.004);

                    let electrical_angle =
                        self.shaft_position_req.get_angle() * self.pole_pairs as f32;
                    Ok((electrical_angle, self.torque))
                }
            }

            CalibrationState::Completed => {
                // キャリブレーション完了、トルク0
                Ok((0.0, 0.0))
            }
        }
    }

    /// トルクを設定
    #[allow(dead_code)]
    pub fn set_torque(&mut self, torque: f32) {
        self.torque = torque.clamp(0.1, 0.5);
    }

    /// 各セクターで記録した角度から電気角オフセットを計算
    fn calculate_offset(&mut self) {
        // 各セクターの期待される機械角（rad）
        // セクター1=0°, 2=60°, 3=120°, 4=180°, 5=240°, 6=300°
        const EXPECTED_ANGLES: [f32; 7] = [
            0.0,                               // インデックス0（未使用）
            0.0,                               // セクター1: 0°
            core::f32::consts::PI / 3.0,       // セクター2: 60°
            2.0 * core::f32::consts::PI / 3.0, // セクター3: 120°
            core::f32::consts::PI,             // セクター4: 180°
            4.0 * core::f32::consts::PI / 3.0, // セクター5: 240°
            5.0 * core::f32::consts::PI / 3.0, // セクター6: 300°
        ];

        info!("Calculating electrical offset from sector angles:");
        let mut offset_sum = 0.0;
        let mut count = 0;

        #[allow(clippy::needless_range_loop)]
        for sector in 1..=6 {
            if let Some(measured_angle) = self.sector_angles[sector] {
                // 機械角から電気角へ変換
                let measured_electrical = measured_angle * self.pole_pairs as f32;
                let expected_electrical = EXPECTED_ANGLES[sector] * self.pole_pairs as f32;

                // オフセット = 測定値 - 期待値
                let mut offset = measured_electrical - expected_electrical;

                // -π～+πの範囲に正規化
                while offset > core::f32::consts::PI {
                    offset -= TAU;
                }
                while offset < -core::f32::consts::PI {
                    offset += TAU;
                }

                info!(
                    "  Sector {}: measured={}° ({} rad), expected={}° ({} rad), offset={}° ({} rad)",
                    sector,
                    measured_angle * 180.0 / core::f32::consts::PI,
                    measured_angle,
                    EXPECTED_ANGLES[sector] * 180.0 / core::f32::consts::PI,
                    EXPECTED_ANGLES[sector],
                    offset * 180.0 / core::f32::consts::PI,
                    offset
                );

                offset_sum += offset;
                count += 1;
            }
        }

        if count > 0 {
            // 平均オフセットを計算
            let average_offset = offset_sum / count as f32;

            // 0～2πに正規化
            self.result.electrical_offset = ShaftPosition::clamp(average_offset);

            info!(
                "Average electrical offset: {} rad ({} deg)",
                self.result.electrical_offset,
                self.result.electrical_offset * 180.0 / core::f32::consts::PI
            );
        } else {
            error!("No sector angles recorded, using offset=0");
            self.result.electrical_offset = 0.0;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calibration_new() {
        let cal = MotorCalibration::new(6, 0.2);
        assert_eq!(cal.get_state(), CalibrationState::Init);
        assert!(!cal.is_completed());
    }

    #[test]
    fn test_calibration_start() {
        let mut cal = MotorCalibration::new(6, 0.2);
        cal.start();
        assert_eq!(cal.get_state(), CalibrationState::Init);
        assert!(!cal.get_result().success);
    }

    #[test]
    fn test_torque_clamping() {
        let mut cal = MotorCalibration::new(6, 0.8); // 0.8は高すぎる
        assert!(cal.torque <= 0.5);

        cal.set_torque(0.05); // 0.05は低すぎる
        assert!(cal.torque >= 0.1);
    }
}
