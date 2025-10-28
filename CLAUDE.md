# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

STM32G431C6Txマイコンを使用したBLDCモータードライバー。Embassy非同期フレームワークベースの組み込みRustプロジェクトです。

## 開発コマンド

### ビルドとフラッシュ
```bash
# ビルドしてデバイスにフラッシュ・実行（デバッグ機能有効）
cargo run

# リリースビルド
cargo run --release

# ビルドのみ（フラッシュしない）
cargo build
cargo build --release
```

### デバッグ
- デフォルトで `debug` フィーチャーが有効（`defmt`、`defmt-rtt`、`panic-probe` を含む）
- ログレベルは `.cargo/config.toml` の `DEFMT_LOG = "trace"` で設定
- `probe-rs` がデバッガとして使用される（STM32G431C6Tx チップ指定）

### フォーマット
```bash
cargo fmt
```

## アーキテクチャ

### ハードウェア構成
- **マイコン**: STM32G431C6Tx（Cortex-M4、170MHz動作設定）
  - HSI → PLL（÷4 × 85 ÷ 2）で170MHz生成
- **ターゲット**: thumbv7em-none-eabi
- **デバッグ**: probe-rs経由のSWD/JTAG

### 主要ペリフェラル
- **TIM1**: 3相補完PWM出力（50kHz、デッドタイム設定済み）
  - U相: PE9/PE8（High/Low側）
  - V相: PE11/PE10（High/Low側）
  - W相: PE13/PE12（High/Low側）
- **OPAMP1/2/3**: 電流センシング用アンプ
  - OPAMP1: PGA×4ゲイン（PA1入力、PA2フィードバック）
  - OPAMP2: スタンドアロン（PA7入力、PC5出力、PA6フィードバック）
  - OPAMP3: スタンドアロン（PB0入力、PB2出力、PB1フィードバック）
- **ADC1/ADC2**: OPAMP出力の電流値読み取り（サンプリング時間640.5サイクル）
- **GPIO**: PC13/PC14/PC15にLED接続

### ソフトウェア構造

#### Embassy非同期ランタイム
- `#[embassy_executor::main]` でメインループ
- `#[embassy_executor::task]` で非同期タスク生成
- Embassy Time で遅延制御

#### 主要タスク
1. **led_task**: 3つのLEDを500msごとに順次点灯
2. **motor_task**: 現在空（今後のモーター制御用）

#### メインループ（[main.rs:158-191](src/main.rs#L158-L191)）
- 3相正弦波PWM生成（120度位相差）
- 角速度の段階的加速（初期0.01 rad/loop → 最大1.0 rad/loop、加速率1.02）
- OPAMP経由のADC電流値読み取り
- 1msごとにループ実行

### defmt ログマクロ（fmt.rs）
- `defmt` フィーチャーの有無で `core` と `defmt` のマクロを切り替え
- `trace!`、`debug!`、`info!`、`error!` などのログマクロ
- `unwrap!`、`assert!` などのユーティリティマクロ
- `defmt` 無効時もコンパイルエラーにならないよう互換性確保

## ビルド最適化

- 開発・リリース両方で `opt-level = "z"`（サイズ最適化）
- LTO有効
- `build-std` でコア標準ライブラリを再ビルド
- `panic_immediate_abort` でパニックサイズ削減

## 重要な制約

- `#![no_std]`、`#![no_main]`: 標準ライブラリ不使用の bare metal 環境
- 浮動小数点演算に `libm` クレートを使用（`sin` 関数など）
- メモリ制約の厳しい組み込み環境のため、ヒープアロケーション非推奨
