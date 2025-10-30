//! モーター自動キャリブレーションモジュール
//!
//! このモジュールは、モーターの電気角オフセットと回転方向を
//! 自動的に検出するキャリブレーション機能を提供します。

use super::shaft_position::ShaftPosition;
use crate::fmt::*;
use core::f32::consts::TAU;

/// キャリブレーション状態
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationState {
    /// 初期化状態
    Init,
    /// 回転方向検出中（モーター方向とセンサー方向の関係を確認）
    FindDirection,
    /// 電気角オフセット検出中
    FindOffset,
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
                self.state = CalibrationState::FindDirection;
                Ok((0.0, 0.0))
            }

            CalibrationState::FindDirection => {
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

                    self.state = CalibrationState::FindOffset;
                    info!("Calibration: FindDirection -> FindOffset");
                } else {
                    // ゆっくり回転（10 rad/s ≈ 95 RPM）
                    // 2.5kHz更新なので、1ステップあたり: 10 / 2500 = 0.004 rad
                    self.shaft_position_req.increment(0.004);
                }

                // 要求位置の電気角を返す（オフセット未適用）
                let electrical_angle = self.shaft_position_req.get_angle() * self.pole_pairs as f32;
                Ok((electrical_angle, self.torque))
            }

            CalibrationState::FindOffset => {
                // 目標: さらに2回転以上 + 3/4回転
                // （合計3回転以上で安定性を確保）
                if self.shaft_position_req.rotations >= 3
                    && self.shaft_position_req.angle > 3.0 * TAU / 4.0
                {
                    // 電気角オフセットを計算
                    // 実際のシャフト角度 × 極対数 = 電気角
                    let offset = self.shaft_position_act.get_angle() * self.pole_pairs as f32;
                    // 0～2πに正規化
                    self.result.electrical_offset = ShaftPosition::clamp(offset);

                    info!(
                        "Electrical offset detected: {} rad ({} deg)",
                        self.result.electrical_offset,
                        self.result.electrical_offset * 180.0 / core::f32::consts::PI
                    );

                    self.state = CalibrationState::ReturnToStart;
                    info!("Calibration: FindOffset -> ReturnToStart");
                } else {
                    // 引き続き回転
                    self.shaft_position_req.increment(0.004);
                }

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
