//! ハードウェア抽象化トレイト定義
//!
//! モータードライバーのハードウェア依存部分を抽象化するためのトレイト群。
//! 異なるハードウェア（STSPIN32G4、DRV8353等）で共通のインターフェースを提供。

#![allow(dead_code)]

/// 3相PWMドライバーインターフェース
///
/// BLDCモーター駆動用のPWM出力を制御するためのトレイト。
pub trait PwmDriver {
    /// PWMの最大Duty値を取得
    fn max_duty(&self) -> u16;

    /// 3相全てのDuty比を設定
    ///
    /// # 引数
    /// * `u` - U相のDuty比
    /// * `v` - V相のDuty比
    /// * `w` - W相のDuty比
    fn set_duty_uvw(&mut self, u: u16, v: u16, w: u16);

    /// 全チャネルを有効化
    fn enable_all_channels(&mut self);

    /// 全チャネルを無効化
    fn disable_all_channels(&mut self);

    /// 全チャネルのDuty比を0にして停止
    fn stop(&mut self);

    /// 各チャネルを個別に有効/無効化
    ///
    /// # 引数
    /// * `enable_u` - U相を有効にするか
    /// * `enable_v` - V相を有効にするか
    /// * `enable_w` - W相を有効にするか
    fn set_channels(&mut self, enable_u: bool, enable_v: bool, enable_w: bool);
}

/// ブートストラップ充電可能なPWMドライバーのトレイト
///
/// ハイサイドFETのゲート駆動用ブートストラップコンデンサを
/// 充電するためのインターフェース。
pub trait BootstrapChargeable {
    /// 全ローサイドをON、ハイサイドをOFF
    ///
    /// 補完PWMでDuty=max_dutyに設定すると：
    /// - ハイサイド(INHx): OFF
    /// - ローサイド(INLx): ON
    ///
    /// これによりブートストラップコンデンサが充電される
    fn set_all_low_side_on(&mut self);

    /// 全チャネルをOFF
    fn set_all_off(&mut self);
}

/// Hallセンサーインターフェース
///
/// BLDCモーターの回転子位置を検出するためのトレイト。
/// TIM4ハードウェアXORモードなどの実装で使用。
pub trait HallSensorInterface {
    /// 現在のHall状態を取得（3ビット: H3<<2 | H2<<1 | H1）
    fn get_hall_state(&self) -> u8;

    /// 周期（サイクル数）を取得
    fn get_period_cycles(&self) -> u32;

    /// タイムアウトフラグを取得
    fn is_timeout(&self) -> bool;

    /// Hallセンサーの一貫したスナップショットを取得
    ///
    /// シーケンスロックを使用して、ISR更新中のデータを読まないことを保証。
    ///
    /// # Returns
    /// Tuple of (hall_state, period_cycles, is_timeout)
    fn get_snapshot(&self) -> (u8, u32, bool);

    /// 周期から速度（RPM）を計算
    ///
    /// # Arguments
    /// * `period_cycles` - Hall edgeエッジ間のサイクル数
    /// * `pole_pairs` - モーターの極対数
    ///
    /// # Returns
    /// 機械角速度 [RPM]
    fn calculate_speed_rpm(&self, period_cycles: u32, pole_pairs: u8) -> f32;

    /// 状態をリセット（モーター停止時に使用）
    fn reset_state(&mut self);
}

/// ゲートドライバー制御インターフェース
///
/// STSPIN32G4などのゲートドライバーICを制御するためのトレイト。
pub trait GateDriverControl {
    /// エラー型
    type Error;

    /// ゲートドライバーを初期化
    ///
    /// 1. READYピンがHighになるまで待機
    /// 2. ステータス読み取り（デバッグ用）
    /// 3. フォルトクリア
    /// 4. NFAULTピンを確認
    async fn initialize(&mut self) -> Result<(), Self::Error>;

    /// NFAULTピンの状態を確認（High = 正常）
    fn is_nfault_ok(&self) -> bool;

    /// READYピンの状態を確認
    fn is_ready(&self) -> bool;
}
