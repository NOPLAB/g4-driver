//! STSPIN32G4 ゲートドライバーIC I2Cレジスタ定義
//!
//! データシート DS13630 Rev 2 Section 5.5.5 に基づく

#![allow(dead_code)]

/// I2Cスレーブアドレス (7ビット)
/// データシート Table 16: 0b1000111
pub const I2C_ADDRESS: u8 = 0x47;

/// レジスタアドレス定義
pub mod reg {
    /// 電源管理設定 (Protected)
    /// - bit[6] REG3V3_DIS: 3.3Vレギュレータ無効化
    /// - bit[5] VCC_DIS: VCCバックレギュレータ無効化
    /// - bit[4] STBY_REG_EN: スタンバイレギュレータ有効化
    /// - bit[1:0] VCC_VAL: VCC出力電圧設定 (00=8V, 01=10V, 10=12V, 11=15V)
    pub const POWMNG: u8 = 0x01;

    /// ドライブロジック設定 (Protected)
    /// - bit[3:2] VDS_P_DEG: VDS保護デグリッチ時間
    /// - bit[1] DTMIN: 最小デッドタイム挿入 (1=有効)
    /// - bit[0] ILOCK: インターロック機能 (1=有効)
    pub const LOGIC: u8 = 0x02;

    /// READY出力設定
    /// - bit[3] STBY_RDY: スタンバイ状態をREADYで報告
    /// - bit[1] THSD_RDY: サーマルシャットダウンをREADYで報告
    /// - bit[0] VCC_UVLO_RDY: VCC UVLO状態をREADYで報告
    pub const READY: u8 = 0x07;

    /// NFAULT出力設定 (Protected)
    /// - bit[2] VDS_P_FLT: VDS保護トリガーをNFAULTで報告
    /// - bit[1] THSD_FLT: サーマルシャットダウンをNFAULTで報告
    /// - bit[0] VCC_UVLO_FLT: VCC UVLO状態をNFAULTで報告
    pub const NFAULT: u8 = 0x08;

    /// フォルトクリアコマンド
    /// 0xFFを書き込むとラッチされたフォルトがクリアされる
    pub const CLEAR: u8 = 0x09;

    /// スタンバイレジスタ (Protected)
    /// - bit[0] STBY: スタンバイモード要求
    pub const STBY: u8 = 0x0A;

    /// ロックレジスタ
    /// - bit[7:4] NLOCK: LOCKのビット反転と一致する場合にアンロック
    /// - bit[3:0] LOCK: 保護レジスタのロック状態
    pub const LOCK: u8 = 0x0B;

    /// リセットコマンド (Protected)
    /// 0xFFを書き込むとレジスタがデフォルト値にリセットされる
    pub const RESET: u8 = 0x0C;

    /// デバイスステータス (読み取り専用)
    /// - bit[7] LOCK: 保護レジスタのロック状態
    /// - bit[3] RESET: リセットフラグ（パワーアップ時にセット）
    /// - bit[2] VDS_P: VDS保護トリガー状態
    /// - bit[1] THSD: サーマルシャットダウン状態
    /// - bit[0] VCC_UVLO: VCC UVLO状態
    pub const STATUS: u8 = 0x80;
}

/// フォルトクリア値
pub const CLEAR_ALL_FAULTS: u8 = 0xFF;

/// STATUSレジスタのビットマスク
pub mod status {
    /// 保護レジスタがロックされている
    pub const LOCK: u8 = 1 << 7;
    /// リセットフラグ（パワーアップ時にセット、クリアが必要）
    pub const RESET: u8 = 1 << 3;
    /// VDS保護がトリガーされた
    pub const VDS_P: u8 = 1 << 2;
    /// サーマルシャットダウン状態
    pub const THSD: u8 = 1 << 1;
    /// VCC低電圧状態
    pub const VCC_UVLO: u8 = 1 << 0;
}

/// LOGICレジスタのデフォルト値
/// デッドタイム6μs、インターロック有効、最小デッドタイム有効
pub const LOGIC_DEFAULT: u8 = 0b0111_0011;

/// POWMNGレジスタのデフォルト値
/// VCC = 8V
pub const POWMNG_DEFAULT: u8 = 0x00;
