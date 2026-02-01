//! STSPIN32G4 ゲートドライバーIC制御モジュール
//!
//! I2C3経由でゲートドライバーICを初期化し、フォルトクリアを行います。
//! データシート DS13630 Rev 2 に基づく実装。

use embassy_stm32::{
    gpio::{Input, Pull},
    i2c::{self, I2c},
    mode::Blocking,
    peripherals, Peri,
};
use embassy_time::{Duration, Instant, Timer};

use crate::fmt::*;
use crate::motor_driver::traits::{BootstrapChargeable, GateDriverControl};

use super::registers::{reg, status, CLEAR_ALL_FAULTS, I2C_ADDRESS};

/// READY待機タイムアウト [ms]
const READY_TIMEOUT_MS: u64 = 100;

/// ブートストラップ充電時間 [ms]
const BOOTSTRAP_CHARGE_MS: u64 = 10;

/// ゲートドライバー初期化エラー
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GateDriverError {
    /// I2C通信エラー
    I2cError,
    /// READYピンタイムアウト
    ReadyTimeout,
    /// NFAULTピンがLow（フォルト状態）
    FaultDetected,
    /// フォルトクリア失敗
    ClearFaultFailed,
}

/// ゲートドライバー制御構造体
pub struct GateDriver<'d> {
    i2c: I2c<'d, Blocking, i2c::Master>,
    ready_pin: Input<'d>,
    nfault_pin: Input<'d>,
}

impl<'d> GateDriver<'d> {
    /// 新しいGateDriverを作成
    ///
    /// # Arguments
    /// * `i2c_peri` - I2C3ペリフェラル
    /// * `scl` - SCLピン (PC8)
    /// * `sda` - SDAピン (PC9)
    /// * `ready` - READYピン (PE14)
    /// * `nfault` - NFAULTピン (PE15)
    pub fn new(
        i2c_peri: Peri<'d, peripherals::I2C3>,
        scl: Peri<'d, peripherals::PC8>,
        sda: Peri<'d, peripherals::PC9>,
        ready: Peri<'d, peripherals::PE14>,
        nfault: Peri<'d, peripherals::PE15>,
    ) -> Self {
        // I2C設定（100kHz標準モード）
        let i2c_config = i2c::Config::default();
        let i2c = I2c::new_blocking(i2c_peri, scl, sda, i2c_config);

        // GPIO入力設定
        // READYはオープンドレイン出力、内部プルアップあり
        let ready_pin = Input::new(ready, Pull::Up);
        // NFAULTはオープンドレイン出力、内部プルアップあり
        let nfault_pin = Input::new(nfault, Pull::Up);

        Self {
            i2c,
            ready_pin,
            nfault_pin,
        }
    }

    /// READYピンがHighになるまで待機
    async fn wait_for_ready(&self) -> Result<(), GateDriverError> {
        let timeout = Duration::from_millis(READY_TIMEOUT_MS);
        let start = Instant::now();

        while !self.ready_pin.is_high() {
            if start.elapsed() > timeout {
                error!("READY pin timeout after {}ms", READY_TIMEOUT_MS);
                return Err(GateDriverError::ReadyTimeout);
            }
            Timer::after(Duration::from_millis(1)).await;
        }
        Ok(())
    }

    /// フォルトクリア（CLEARレジスタに0xFFを書き込む）
    fn clear_faults(&mut self) -> Result<(), GateDriverError> {
        self.i2c
            .blocking_write(I2C_ADDRESS, &[reg::CLEAR, CLEAR_ALL_FAULTS])
            .map_err(|_| GateDriverError::ClearFaultFailed)
    }

    /// ステータスレジスタを読み取り
    fn read_status(&mut self) -> Result<u8, GateDriverError> {
        let mut buf = [0u8; 1];
        self.i2c
            .blocking_write_read(I2C_ADDRESS, &[reg::STATUS], &mut buf)
            .map_err(|_| GateDriverError::I2cError)?;
        Ok(buf[0])
    }

    /// レジスタを読み取り
    #[allow(dead_code)]
    pub fn read_register(&mut self, register: u8) -> Result<u8, GateDriverError> {
        let mut buf = [0u8; 1];
        self.i2c
            .blocking_write_read(I2C_ADDRESS, &[register], &mut buf)
            .map_err(|_| GateDriverError::I2cError)?;
        Ok(buf[0])
    }

    /// レジスタに書き込み
    #[allow(dead_code)]
    pub fn write_register(&mut self, register: u8, value: u8) -> Result<(), GateDriverError> {
        self.i2c
            .blocking_write(I2C_ADDRESS, &[register, value])
            .map_err(|_| GateDriverError::I2cError)
    }
}

impl<'d> GateDriverControl for GateDriver<'d> {
    type Error = GateDriverError;

    /// ゲートドライバーを初期化
    ///
    /// 1. READYピンがHighになるまで待機
    /// 2. ステータス読み取り（デバッグ用）
    /// 3. フォルトクリア
    /// 4. NFAULTピンを確認
    async fn initialize(&mut self) -> Result<(), Self::Error> {
        info!("Initializing gate driver IC via I2C3...");

        // 1. READYピン待機
        self.wait_for_ready().await?;
        info!("Gate driver READY");

        // 2. ステータス読み取り（デバッグ用）
        match self.read_status() {
            Ok(status_val) => {
                info!("Gate driver STATUS before clear: 0x{:02X}", status_val);
                if status_val & status::RESET != 0 {
                    info!("  - RESET flag is set (expected after power-up)");
                }
                if status_val & status::VCC_UVLO != 0 {
                    warn!("  - VCC_UVLO flag is set");
                }
                if status_val & status::THSD != 0 {
                    warn!("  - Thermal shutdown flag is set");
                }
                if status_val & status::VDS_P != 0 {
                    warn!("  - VDS protection flag is set");
                }
            }
            Err(_) => {
                warn!("Failed to read STATUS register (I2C error)");
            }
        }

        // 3. フォルトクリア（パワーアップ時のRESETフラグをクリア）
        self.clear_faults()?;
        info!("Gate driver faults cleared");

        // 4. NFAULTピン確認
        if !self.is_nfault_ok() {
            error!("NFAULT pin is LOW - fault condition detected");
            return Err(GateDriverError::FaultDetected);
        }
        info!("Gate driver NFAULT OK");

        // 5. ステータス再読み取り（確認用）
        if let Ok(status_val) = self.read_status() {
            info!("Gate driver STATUS after clear: 0x{:02X}", status_val);
        }

        info!("Gate driver initialization complete");
        Ok(())
    }

    /// NFAULTピンの状態を確認（High = 正常）
    fn is_nfault_ok(&self) -> bool {
        self.nfault_pin.is_high()
    }

    /// READYピンの状態を確認
    fn is_ready(&self) -> bool {
        self.ready_pin.is_high()
    }
}

/// ブートストラップ充電シーケンス
///
/// PWM開始前に全ローサイドをONにしてブートストラップコンデンサを充電。
/// データシート Section 5.2.4.1 参照。
pub async fn bootstrap_charge<PWM>(pwm: &mut PWM)
where
    PWM: BootstrapChargeable,
{
    info!(
        "Starting bootstrap charge sequence ({}ms)...",
        BOOTSTRAP_CHARGE_MS
    );

    // 全ローサイドをON（ハイサイドはOFF）
    pwm.set_all_low_side_on();

    // 充電時間待機
    Timer::after(Duration::from_millis(BOOTSTRAP_CHARGE_MS)).await;

    // 通常状態に戻す
    pwm.set_all_off();

    info!("Bootstrap charge complete");
}
