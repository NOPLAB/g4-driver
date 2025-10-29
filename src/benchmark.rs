//! ベンチマークモジュール
//!
//! FOC関数のパフォーマンス測定を提供します。

use crate::fmt::*;
use crate::foc;

/// DWTサイクルカウンタを有効化
///
/// # Safety
/// Cortex-Mペリフェラルへの直接アクセスを含む
pub unsafe fn enable_cycle_counter() {
    let mut cp = cortex_m::Peripherals::steal();
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
}

/// inverse_park()のベンチマークを実行して結果を表示
///
/// # 引数
/// * `iterations` - ベンチマーク実行回数
pub fn run_inverse_park_benchmark(iterations: u32) {
    info!("Running inverse_park() benchmark...");

    // ベンチマーク実行
    let (result_idsp, result_libm, ticks_idsp, ticks_libm) =
        foc::benchmark_inverse_park(iterations);

    // サイクル/呼び出し を計算（整数に変換してdefmtで表示）
    let cycles_per_call_idsp = ticks_idsp / iterations;
    let cycles_per_call_libm = ticks_libm / iterations;
    let speedup_x10 = (cycles_per_call_libm * 10) / cycles_per_call_idsp; // 10倍してdefmtで表示

    info!("Benchmark results ({} iterations):", iterations);
    info!(
        "  idsp::cossin():  {} cycles total, {} cycles/call",
        ticks_idsp, cycles_per_call_idsp
    );
    info!(
        "  libm::cosf/sinf: {} cycles total, {} cycles/call",
        ticks_libm, cycles_per_call_libm
    );
    info!(
        "  Speedup: {}.{}x faster with idsp",
        speedup_x10 / 10,
        speedup_x10 % 10
    );
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
