//! Hallセンサー状態管理
//!
//! シーケンスロック機構を使用したISR-タスク間同期

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

/// シーケンス番号（偶数=安定、奇数=更新中）
pub(super) static SEQUENCE: AtomicU32 = AtomicU32::new(0);

/// Hallセンサー状態（グローバル共有）
pub static HALL_STATE: AtomicU8 = AtomicU8::new(0);

/// 最後のキャプチャ値（タイマーカウント）- デバッグ用
pub static LAST_CAPTURE: AtomicU32 = AtomicU32::new(0);

/// 最後のオーバーフローカウント - デバッグ用
pub static LAST_OVERFLOW: AtomicU32 = AtomicU32::new(0);

/// オーバーフローカウンタ（65536カウントごとにインクリメント、キャプチャ時にリセット）
pub static OVERFLOW_COUNTER: AtomicU32 = AtomicU32::new(0);

/// 速度計算用：前回キャプチャ（リセット）からの経過サイクル数
/// period = (overflow << 16) | capture として計算
pub static PERIOD_CYCLES: AtomicU32 = AtomicU32::new(0);

/// タイムアウトフラグ（モーター停止検出）
pub static TIMEOUT_FLAG: AtomicU8 = AtomicU8::new(0);

/// Hall状態を取得（TIM4割り込みでキャプチャされた値）
///
/// 注: 一貫性が必要な場合は `get_snapshot()` を使用してください
#[inline(always)]
pub fn get_hall_state() -> u8 {
    HALL_STATE.load(Ordering::Acquire)
}

/// 周期（サイクル数）を取得
///
/// 注: 一貫性が必要な場合は `get_snapshot()` を使用してください
#[inline(always)]
pub fn get_period_cycles() -> u32 {
    PERIOD_CYCLES.load(Ordering::Acquire)
}

/// タイムアウトフラグを取得
///
/// 注: 一貫性が必要な場合は `get_snapshot()` を使用してください
#[inline(always)]
#[allow(dead_code)]
pub fn is_timeout() -> bool {
    TIMEOUT_FLAG.load(Ordering::Acquire) != 0
}

/// Hallセンサーの一貫したスナップショットを取得
///
/// シーケンスロックを使用して、ISR更新中のデータを読まないことを保証します。
/// 3つの値（hall_state, period_cycles, is_timeout）が同じISRサイクルの
/// データであることが保証されます。
///
/// # Returns
/// Tuple of (hall_state, period_cycles, is_timeout)
#[inline(always)]
pub fn get_snapshot() -> (u8, u32, bool) {
    loop {
        // シーケンス番号を読む（偶数なら安定状態）
        let seq1 = SEQUENCE.load(Ordering::Acquire);
        if seq1 & 1 != 0 {
            // 奇数 = ISR更新中、リトライ
            core::hint::spin_loop();
            continue;
        }

        // データ読み取り
        let hall_state = HALL_STATE.load(Ordering::Acquire);
        let period_cycles = PERIOD_CYCLES.load(Ordering::Acquire);
        let is_timeout = TIMEOUT_FLAG.load(Ordering::Acquire) != 0;

        // シーケンス番号再確認（変わっていなければ一貫性あり）
        let seq2 = SEQUENCE.load(Ordering::Acquire);
        if seq1 == seq2 {
            return (hall_state, period_cycles, is_timeout);
        }

        // シーケンスが変わった = 読み取り中にISRが発火した、リトライ
        core::hint::spin_loop();
    }
}

/// TIM4の状態をリセット（モーター停止時に使用）
///
/// モーター停止時に古い周期データをクリアします。
/// 注: OpenLoop→FOC切り替え時は呼ばない（リアルタイムデータを保持）
pub fn reset_state() {
    LAST_CAPTURE.store(0, Ordering::Relaxed);
    LAST_OVERFLOW.store(0, Ordering::Relaxed);
    OVERFLOW_COUNTER.store(0, Ordering::Relaxed);
    PERIOD_CYCLES.store(0, Ordering::Relaxed);
    TIMEOUT_FLAG.store(0, Ordering::Relaxed); // タイムアウトフラグもクリア
}

/// 最大許容速度 [RPM]（これを超える速度はノイズとして無視）
const MAX_VALID_RPM: f32 = 10000.0;

/// 最小周期サイクル数（これ未満はノイズとして無視）
/// 10000 RPM = 170MHz / (cycles * 6 * 6 / 60) → cycles = 170M * 10 / (10000 * 36) ≈ 4722 cycles
/// 安全マージンを持って1000 cyclesを最小値とする
const MIN_VALID_PERIOD_CYCLES: u32 = 1000;

/// 周期から速度（RPM）を計算
///
/// # Arguments
/// * `period_cycles` - Hall edgeエッジ間のサイクル数（170MHz）
/// * `pole_pairs` - モーターの極対数
///
/// # Returns
/// 機械角速度 [RPM]（異常値の場合は0.0を返す）
#[inline(always)]
pub fn calculate_speed_rpm(period_cycles: u32, pole_pairs: u8) -> f32 {
    // 周期が0または最小値未満の場合はノイズとして0を返す
    if period_cycles < MIN_VALID_PERIOD_CYCLES {
        return 0.0;
    }

    // 170MHz、6ステップ/1電気回転、pole_pairs電気回転/1機械回転
    // RPM = (170_000_000 / period_cycles) * (60 / 6) / pole_pairs
    //     = (170_000_000 * 10) / (period_cycles * pole_pairs)

    const SYSTEM_CLOCK_HZ: f32 = 170_000_000.0;
    const STEPS_PER_ELEC_REV: f32 = 6.0; // Hallセンサー6ステップで1電気回転

    let freq_hz = SYSTEM_CLOCK_HZ / period_cycles as f32; // エッジ周波数 [Hz]
    let elec_rpm = freq_hz * 60.0 / STEPS_PER_ELEC_REV; // 電気角RPM

    let rpm = elec_rpm / pole_pairs as f32; // 機械角RPM

    // 最大速度を超える場合はノイズとして0を返す
    if rpm > MAX_VALID_RPM {
        return 0.0;
    }

    rpm
}
