//! TIM4ベースのHallセンサーインターフェース実装
//!
//! STM32のハードウェアHall Sensor Interface Mode（XORモード）を使用して、
//! 3つのHallセンサー入力から自動的にエッジ検出とタイムスタンプキャプチャを行います。
//!
//! ## ハードウェア構成
//! - TIM4_CH1 (PB6): Hall H1
//! - TIM4_CH2 (PB7): Hall H2
//! - TIM4_CH3 (PB8): Hall H3
//! - クロック: 170MHz (APB1)
//!
//! ## 動作原理（参照: HAL_TIMEx_HallSensor_Init）
//! 1. 3つのHall入力がXORされてTI1に接続される（CR2.TI1S=1）
//! 2. TI1のエッジ検出がトリガーとして選択される（SMCR.TS=TI1F_ED）
//! 3. トリガーエッジでカウンターがリセットされる（SMCR.SMS=RESET）
//! 4. いずれかのHallセンサーがエッジを検出すると、TIM4_CCR1にカウンタ値がキャプチャされる
//! 5. CC1割り込みが発生し、エッジ間の時間から速度を計算
//! 6. UPDATE割り込みでタイムアウト（低速/停止）を検出

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use embassy_stm32::pac;

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

/// TIM4 Hall Sensor Interface の初期化
///
/// # Safety
/// PACを使用した直接的なレジスタ操作を含むため、unsafe
pub unsafe fn init_hall_timer() {
    let rcc = pac::RCC;
    let tim4 = pac::TIM4;
    let gpiob = pac::GPIOB;

    // 1. クロック有効化
    rcc.ahb2enr().modify(|w| w.set_gpioben(true)); // GPIOB
    rcc.apb1enr1().modify(|w| w.set_tim4en(true)); // TIM4

    // 2. GPIO設定（PB6/PB7/PB8をAlternate Function AF2に設定）
    // 注: 参照実装（HAL）ではプルアップ無し（NOPULL）
    //     ハードウェアに外部プルアップ抵抗がある場合は内部プルアップ不要

    // PB6: TIM4_CH1 (Hall H1)
    gpiob
        .moder()
        .modify(|w| w.set_moder(6, pac::gpio::vals::Moder::ALTERNATE));
    gpiob.afr(0).modify(|w| w.set_afr(6, 2)); // AF2 (AFR[0] = AFRL)
    gpiob
        .pupdr()
        .modify(|w| w.set_pupdr(6, pac::gpio::vals::Pupdr::FLOATING)); // プルアップ無し（参照実装に準拠）
    gpiob
        .ospeedr()
        .modify(|w| w.set_ospeedr(6, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED));

    // PB7: TIM4_CH2 (Hall H2)
    gpiob
        .moder()
        .modify(|w| w.set_moder(7, pac::gpio::vals::Moder::ALTERNATE));
    gpiob.afr(0).modify(|w| w.set_afr(7, 2)); // AF2 (AFR[0] = AFRL)
    gpiob
        .pupdr()
        .modify(|w| w.set_pupdr(7, pac::gpio::vals::Pupdr::FLOATING)); // プルアップ無し（参照実装に準拠）
    gpiob
        .ospeedr()
        .modify(|w| w.set_ospeedr(7, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED));

    // PB8: TIM4_CH3 (Hall H3)
    gpiob
        .moder()
        .modify(|w| w.set_moder(8, pac::gpio::vals::Moder::ALTERNATE));
    gpiob.afr(1).modify(|w| w.set_afr(0, 2)); // AF2 (AFR[1] = AFRH, PB8は8番目なのでAFRH[0])
    gpiob
        .pupdr()
        .modify(|w| w.set_pupdr(8, pac::gpio::vals::Pupdr::FLOATING)); // プルアップ無し（参照実装に準拠）
    gpiob
        .ospeedr()
        .modify(|w| w.set_ospeedr(8, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED));

    // 3. TIM4設定
    // タイマーを停止
    tim4.cr1().modify(|w| w.set_cen(false));

    // プリスケーラー設定（初期値: 0、フルスピード170MHz）
    tim4.psc().write_value(0); // PSC = 0
    tim4.arr().write_value(pac::timer::regs::ArrCore(0xFFFF)); // ARR = 65535

    // 4. Hall Sensor Interface Mode設定
    // CR2.TI1S = 1: CH1/CH2/CH3をXOR -> TI1 (Hall Sensor Mode)
    // 参照: HAL_TIMEx_HallSensor_Init() in stm32g4xx_hal_tim_ex.c
    tim4.cr2().modify(|w| {
        w.set_ti1s(pac::timer::vals::Ti1s::XOR); // TI1S = XOR: 3つのHall入力をXORしてTI1へ
    });

    // SMCR設定: トリガー選択とスレーブモード
    tim4.smcr().modify(|w| {
        // TS = TI1F_ED (0b100): TI1のエッジ検出をトリガーに選択
        w.set_ts(pac::timer::vals::Ts::TI1F_ED);
        // SMS = RESET (0b100): トリガーエッジでカウンターをリセット
        w.set_sms(pac::timer::vals::Sms::RESET_MODE);
    });

    // 5. Input Capture設定（CH1でキャプチャ）
    // CCMR1_Input: CC1S=TRC (IC1はTRCにマップ = TI1にマップ)、IC1F (フィルタ設定)
    // 参照: TIM_TI1_SetConfig(..., TIM_ICSELECTION_TRC, ...)
    tim4.ccmr_input(0).modify(|w| {
        w.set_ccs(0, pac::timer::vals::CcmrInputCcs::TRC); // CC1S = TRC (IC1 -> TI1/TRC)
        w.set_icf(0, pac::timer::vals::FilterValue::FCK_INT_N8); // IC1F = 0011 (8サイクルフィルタ)
    });

    // 6. CCER: CC1E=1（キャプチャ有効）、立ち上がりエッジのみ
    // 参照実装では IC1Polarity = RISING のみ
    // 注: TI1F_EDトリガーにより両エッジが自動的に検出される
    tim4.ccer().modify(|w| {
        w.set_cce(0, true); // CC1E: Capture enabled
        w.set_ccp(0, false); // CC1P: 0 = RISING (立ち上がりエッジのみ)
                             // CC1NP はデフォルト false のまま
    });

    // 7. 割り込み設定
    // DIER: CC1IE（キャプチャ割り込み）、UIE（更新割り込み）を有効化
    tim4.dier().modify(|w| {
        w.set_ccie(0, true); // CC1IE: Capture/Compare 1 interrupt enable
        w.set_uie(true); // UIE: Update interrupt enable
    });

    // 8. 割り込み有効化（NVIC）
    // 優先度設定（参照実装: Priority 2）
    // Embassy環境では優先度に注意（Embassyタスクより高優先度にする）
    // STM32 uses 4 bits for priority (16 levels), shifted left by 4
    // Priority 2 = 0x20
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM4);
        let mut cp = cortex_m::Peripherals::steal();
        cp.NVIC.set_priority(pac::Interrupt::TIM4, 0x20);
    }

    // 9. カウンタをリセットしてタイマー開始
    tim4.cnt().write_value(pac::timer::regs::CntCore(0)); // CNT = 0
    tim4.sr().write(|w| w.0 = 0); // ステータスフラグクリア
    tim4.egr().write(|w| w.set_ug(true)); // Update生成（プリスケーラ反映）

    tim4.cr1().modify(|w| {
        w.set_cen(true); // カウンタ有効
        w.set_urs(pac::timer::vals::Urs::COUNTER_ONLY); // Update Request Source: カウンタオーバーフローのみ
    });
}

/// TIM4割り込みハンドラー（Capture/Compare 1 + Update）
///
/// # Safety
/// 割り込みコンテキストで実行されるため、処理は最小限にする
#[inline(always)]
pub unsafe fn tim4_irq_handler() {
    let tim4 = pac::TIM4;
    let gpiob = pac::GPIOB;

    let sr = tim4.sr().read();

    // UPDATE割り込み（オーバーフロー）
    if sr.uif() {
        tim4.sr().modify(|w| w.set_uif(false)); // フラグクリア

        // オーバーフローカウンタをインクリメント
        OVERFLOW_COUNTER.fetch_add(1, Ordering::Relaxed);

        // タイムアウト検出（オーバーフローが一定回数以上ならモーター停止）
        // 170MHz、PSC=0、ARR=0xFFFF → 約385μs/overflow
        // 1秒 = 2597回オーバーフロー → タイムアウト閾値: 2600回
        let overflow_count = OVERFLOW_COUNTER.load(Ordering::Relaxed);
        if overflow_count > 2600 {
            TIMEOUT_FLAG.store(1, Ordering::Relaxed);
            PERIOD_CYCLES.store(0, Ordering::Relaxed); // 速度0
        }
    }

    // CAPTURE/COMPARE 1割り込み（Hallエッジ検出）
    if sr.ccif(0) {
        tim4.sr().modify(|w| w.set_ccif(0, false)); // フラグクリア

        // 1. キャプチャ値読み取り
        let capture = tim4.ccr(0).read().ccr() as u32;
        let overflow = OVERFLOW_COUNTER.load(Ordering::Relaxed);

        // 2. Hall状態読み取り（GPIO直接読み取り）
        let idr = gpiob.idr().read();
        let h1 = idr.idr(6) as u8; // PB6
        let h2 = idr.idr(7) as u8; // PB7
        let h3 = idr.idr(8) as u8; // PB8
        let hall_state = (h3 << 2) | (h2 << 1) | h1;

        // 3. 周期計算（前回キャプチャからの経過サイクル）
        // オーバーフローカウンタはキャプチャごとにリセットされるため、
        // 周期 = overflow * 65536 + capture が前回リセットからの絶対経過サイクル数になる
        let period = (overflow << 16) | capture;

        // 4. グローバル変数更新
        HALL_STATE.store(hall_state, Ordering::Relaxed);
        LAST_CAPTURE.store(capture, Ordering::Relaxed);
        LAST_OVERFLOW.store(overflow, Ordering::Relaxed); // デバッグ用に保持
        PERIOD_CYCLES.store(period, Ordering::Relaxed);
        OVERFLOW_COUNTER.store(0, Ordering::Relaxed); // リセット（次のエッジまでの計測開始）
        TIMEOUT_FLAG.store(0, Ordering::Relaxed); // タイムアウト解除
    }
}

/// TIM4割り込みのRust側エントリーポイント
/// memory.xまたはリンカースクリプトでTIM4割り込みベクタに登録する
#[allow(non_snake_case)]
#[no_mangle]
pub unsafe extern "C" fn TIM4() {
    tim4_irq_handler();
}

/// Hall状態を取得（TIM4割り込みでキャプチャされた値）
#[inline(always)]
pub fn get_hall_state() -> u8 {
    HALL_STATE.load(Ordering::Relaxed)
}

/// 周期（サイクル数）を取得
#[inline(always)]
pub fn get_period_cycles() -> u32 {
    PERIOD_CYCLES.load(Ordering::Relaxed)
}

/// タイムアウトフラグを取得
#[inline(always)]
pub fn is_timeout() -> bool {
    TIMEOUT_FLAG.load(Ordering::Relaxed) != 0
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

/// 周期から速度（RPM）を計算
///
/// # Arguments
/// * `period_cycles` - Hall edgeエッジ間のサイクル数（170MHz）
/// * `pole_pairs` - モーターの極対数
///
/// # Returns
/// 機械角速度 [RPM]
#[inline(always)]
pub fn calculate_speed_rpm(period_cycles: u32, pole_pairs: u8) -> f32 {
    if period_cycles == 0 {
        return 0.0;
    }

    // 170MHz、6ステップ/1電気回転、pole_pairs電気回転/1機械回転
    // RPM = (170_000_000 / period_cycles) * (60 / 6) / pole_pairs
    //     = (170_000_000 * 10) / (period_cycles * pole_pairs)

    const SYSTEM_CLOCK_HZ: f32 = 170_000_000.0;
    const STEPS_PER_ELEC_REV: f32 = 6.0; // Hallセンサー6ステップで1電気回転

    let freq_hz = SYSTEM_CLOCK_HZ / period_cycles as f32; // エッジ周波数 [Hz]
    let elec_rpm = freq_hz * 60.0 / STEPS_PER_ELEC_REV; // 電気角RPM

    elec_rpm / pole_pairs as f32 // 機械角RPM
}
