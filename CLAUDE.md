# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

STM32G431VBTxマイコンを使用したBLDCモータードライバー。Hall センサベースの FOC（Field Oriented Control）実装で、CAN 通信によるモーター制御を行う Embassy 非同期フレームワークベースの組み込み Rust プロジェクトです。

## 開発コマンド

### ビルドとフラッシュ
```bash
# ビルドしてデバイスにフラッシュ・実行（デバッグ機能有効）
cargo run

# リリースビルド（最適化レベル O2）
cargo run --release

# ビルドのみ（フラッシュしない）
cargo build
cargo build --release
```

### デバッグ
- デフォルトで `debug` フィーチャーが有効（`defmt`、`defmt-rtt`、`panic-probe` を含む）
- ログレベルは `.cargo/config.toml` の `DEFMT_LOG = "trace"` で設定
- `probe-rs` がデバッガとして使用される（STM32G431VBTx チップ指定）

### フォーマット
```bash
cargo fmt
```

## アーキテクチャ

### ハードウェア構成
- **マイコン**: STM32G431VBTx（Cortex-M4、170MHz動作設定）
  - HSI → PLL（÷4 × 85 ÷ 2）で170MHz生成
  - FDCAN1: PLL1_Q クロック使用
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
  - 現在は将来の電流リミット監視用に予約（FOCループでは未使用）
- **Hall センサ**: PB6=H1、PB7=H2、PB8=H3（位置・速度検出）
- **FDCAN1**: CAN通信（PA11=RX、PA12=TX、250kbps）
- **GPIO**: PC13/PC14/PC15にLED接続

### モーター制御パラメータ
- **極対数**: 6（ポール数12）
- **制御周期**: 1kHz（1000μs）
- **最大電圧**: 24V
- **DCバス電圧**: 24V
- **最大Duty比**: 100（PWM範囲：0=0%, 100=100%）
- **速度フィルタ係数**: 0.2（Hallセンサ用ローパスフィルタ）
- **デフォルトPI ゲイン**: Kp=0.5、Ki=0.05（応答性向上のため増加）
- **Hall角度オフセット**: 0度（ハードウェアに応じて調整可能）
- **最小q軸電圧**: 0.5V（静止摩擦克服用）

## ソフトウェア構造

### Embassy非同期ランタイム
- `#[embassy_executor::main]` でメインループ
- `#[embassy_executor::task]` で非同期タスク生成
- Embassy Time で遅延制御

### 主要タスク

#### 1. led_task ([main.rs:146-168](src/main.rs#L146-L168))
3つのLEDを500msごとに順次点灯（動作確認用）

#### 2. motor_control_task ([main.rs:180-410](src/main.rs#L180-L410))
**1kHz FOC制御ループ** - BLDCモーターのField Oriented Control（オープンループ始動付き）
1. モーター使能チェック（無効時はPWM停止、PIリセット）
2. Hallセンサ読み取り（PB6/7/8 → 3ビット状態）
3. 電気角・速度推定（HallSensor::update + オフセット適用）
4. **制御モード分岐**:
   - **OpenLoop**: 6ステップ駆動（台形波）で始動、目標RPM到達でFOCに切替
   - **ClosedLoopFoc**: 以下のFOC制御ループ
5. PIゲイン更新チェック（CAN経由で変更された場合）
6. 目標速度取得（CAN経由で設定）
7. 速度PI制御（q軸電圧指令生成、d軸は0）
8. 最小電圧適用（静止摩擦克服用、速度誤差>10RPMの場合）
9. 電圧ベクトル制限（円形リミッタ）
10. Park逆変換（dq → αβ座標）
11. SVPWM計算（αβ → UVW Duty比）
12. PWM出力（TIM1への設定）
13. ステータス更新（CAN送信用）
14. デバッグログ（1秒ごと）

#### 3. can_task ([main.rs:63-144](src/main.rs#L63-L144))
**CAN通信タスク** - モーター制御コマンド受信とステータス送信
- 100ms周期でステータス送信（ID 0x200: 速度RPM + 電気角）
- コマンド受信処理：
  - `0x100`: 速度指令（f32 RPM、4バイト）
  - `0x101`: PIゲイン設定（Kp、Kiそれぞれf32、8バイト）
  - `0x102`: モーター使能（u8: 0=無効、1=有効）
  - `0x000`: 緊急停止（即座にモーター停止、速度0）

### FOCモジュール（[src/foc.rs](src/foc.rs)）

#### HallSensor ([src/foc/hall_sensor.rs](src/foc/hall_sensor.rs))
- Hall状態（1-6）から電気角推定（セクター中心：30, 90, 150, 210, 270, 330度）
  - **重要**: セクターの中心角を使用することでFOC制御の精度向上
- Hall エッジ検出による速度計算（RPM）
- ローパスフィルタによる速度平滑化
- タイムアウト検出（1秒間エッジなし → 速度0）

#### PiController ([src/foc/pi_controller.rs](src/foc/pi_controller.rs))
- 比例・積分制御（anti-windup付き）
- 出力リミッタ（飽和時は積分停止）
- ゲイン・リミット動的変更可能
- `new_symmetric()` で±リミット設定

#### SVPWM ([src/foc/svpwm.rs](src/foc/svpwm.rs))
- Space Vector PWM生成（正弦波PWMより15%電圧利用率向上）
- セクター判定（1-6）とデューティ比計算
- ゼロベクトル時間の均等配分
- **ゼロ電圧時の特別処理**: magnitude=0の場合、中心値(50%)を返す
  - 3相が同じDuty比(50%)の時、相間電圧=0V（モーター停止状態）
- `calculate_sinusoidal_pwm()` も実装（シンプル版）

#### Transforms ([src/foc/transforms.rs](src/foc/transforms.rs))
- `inverse_park()`: dq → αβ座標変換（回転座標系 → 静止座標系）
- `inverse_clarke()`: αβ → UVW 3相変換
- `limit_voltage()`: dq電圧ベクトルの円形制限
- `normalize_angle()`: 角度正規化（0～2π）

#### OpenLoopRampUp ([src/foc.rs:15-124](src/foc.rs#L15-L124))
- 始動時のオープンループ制御（強制転流）
- 角速度のランプアップ（加速度指定）
- 目標速度到達後にFOC制御へ移行
- **注**: 現在のmain.rsでは未使用（将来の始動制御用）

### CANプロトコル（[src/can_protocol.rs](src/can_protocol.rs)）
- CAN ID定義: `can_ids` モジュール
- パース関数: `parse_speed_command()`, `parse_pi_gains()`, `parse_enable_command()`
- エンコード関数: `encode_status()`, `decode_status()`
- 全てリトルエンディアンf32形式
- テストコード付き（`#[cfg(test)]`）

### defmt ログマクロ（[src/fmt.rs](src/fmt.rs)）
- `defmt` フィーチャーの有無で `core` と `defmt` のマクロを切り替え
- `trace!`、`debug!`、`info!`、`error!` などのログマクロ
- `unwrap!`、`assert!` などのユーティリティマクロ
- `defmt` 無効時もコンパイルエラーにならないよう互換性確保

## ビルド最適化

- 開発ビルド: `opt-level = "z"`（サイズ最適化）、`debug = true`、`incremental = true`
- リリースビルド: `opt-level = 2`（速度最適化）、`debug = false`、`incremental = true`
- LTO有効（両ビルド）
- `build-std = ["core"]` でコア標準ライブラリを再ビルド
- `panic_immediate_abort` でパニックサイズ削減

## 重要な制約

- `#![no_std]`、`#![no_main]`: 標準ライブラリ不使用の bare metal 環境
- 浮動小数点演算に `libm` クレートを使用（`sin`、`cos`、`atan2`、`sqrt` など）
- メモリ制約の厳しい組み込み環境のため、ヒープアロケーション非推奨
- FOC制御は1kHzループで実行されるため、処理時間に注意（1ms以内に完了必要）
