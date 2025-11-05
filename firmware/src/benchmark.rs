//! ベンチマークモジュール
//!
//! FOC関数のパフォーマンス測定を提供します。

use crate::fmt::*;
use crate::foc;

/// DWTサイクルカウンタを有効化
///
/// # Safety
/// Cortex-Mペリフェラルへの直接アクセスを含む
#[allow(dead_code)]
pub unsafe fn enable_cycle_counter() {
    use core::ptr::{read_volatile, write_volatile};

    // DWTとDCBペリフェラルのレジスタアドレス
    const DWT_CTRL: *mut u32 = 0xE000_1000 as *mut u32;
    const DWT_CYCCNT: *mut u32 = 0xE000_1004 as *mut u32;
    const DCB_DEMCR: *mut u32 = 0xE000_EDFC as *mut u32;

    // DEMCRレジスタのTRCENAビット(bit 24)を立ててDWTを有効化
    let demcr = read_volatile(DCB_DEMCR);
    write_volatile(DCB_DEMCR, demcr | 0x0100_0000);

    // サイクルカウンタをリセット
    write_volatile(DWT_CYCCNT, 0);

    // DWT_CTRLのCYCCNTENAビット(bit 0)を立ててサイクルカウンタを有効化
    let ctrl = read_volatile(DWT_CTRL);
    write_volatile(DWT_CTRL, ctrl | 0x0000_0001);

    info!("DWT cycle counter enabled");
}

/// inverse_park()のベンチマークを実行して結果を表示
///
/// # 引数
/// * `iterations` - ベンチマーク実行回数
#[allow(dead_code)]
pub fn run_inverse_park_benchmark(iterations: u32) {
    info!("Running inverse_park() benchmark...");

    // ベンチマーク実行
    let (result_idsp, result_libm, ticks_idsp, ticks_libm) =
        foc::benchmark_inverse_park(iterations);

    // サイクル/呼び出し を計算（整数に変換してdefmtで表示）
    let cycles_per_call_idsp = ticks_idsp / iterations;
    let cycles_per_call_libm = ticks_libm / iterations;

    info!("Benchmark results ({} iterations):", iterations);
    info!(
        "  idsp::cossin():  {} cycles total, {} cycles/call",
        ticks_idsp, cycles_per_call_idsp
    );
    info!(
        "  libm::cosf/sinf: {} cycles total, {} cycles/call",
        ticks_libm, cycles_per_call_libm
    );

    // スピードアップを計算（ゼロ除算を回避）
    if cycles_per_call_idsp > 0 {
        let speedup_x10 = (cycles_per_call_libm * 10) / cycles_per_call_idsp; // 10倍してdefmtで表示
        info!(
            "  Speedup: {}.{}x faster with idsp",
            speedup_x10 / 10,
            speedup_x10 % 10
        );
    } else {
        error!("  Cannot calculate speedup: idsp measurement returned 0 cycles");
    }
    info!(
        "  Result idsp:  alpha={}, beta={}",
        result_idsp.0, result_idsp.1
    );
    info!(
        "  Result libm:  alpha={}, beta={}",
        result_libm.0, result_libm.1
    );
    info!(
        "  Error: alpha={}, beta={}",
        result_idsp.0 - result_libm.0,
        result_idsp.1 - result_libm.1
    );
}
