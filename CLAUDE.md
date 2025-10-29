# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

STM32G431VBTxマイコンを使用したBLDCモータードライバー。Hall センサベースの FOC（Field Oriented Control）実装で、CAN 通信によるモーター制御を行う Embassy 非同期フレームワークベースの組み込み Rust プロジェクトです。

**プロジェクト構造**: このリポジトリには複数の独立したコンポーネントが含まれています：

- `firmware/` - STM32組み込みファームウェア（`no_std`、Embassy ベース）
- `controller/` - CAN通信用デスクトップGUIコントローラー（Dioxus、`std`）
- `scripts/` - CANデバッグ用Bashスクリプト

## 開発コマンド

**重要**: このリポジトリは複数の独立したプロジェクトを含むため、作業ディレクトリに注意してください。

### ファームウェア（firmware/）

#### ビルドとフラッシュ

```bash
# ファームウェアディレクトリに移動
cd firmware

# ビルドしてデバイスにフラッシュ・実行（デバッグ機能有効）
cargo run

# リリースビルド（最適化レベル O2）
cargo run --release

# ビルドのみ（フラッシュしない）
cargo build
cargo build --release
```

#### デバッグ

- デフォルトで `debug` フィーチャーが有効（`defmt`、`defmt-rtt`、`panic-probe` を含む）
- ログレベルは `.cargo/config.toml` の `DEFMT_LOG = "trace"` で設定
- `probe-rs` がデバッガとして使用される（STM32G431VBTx チップ指定）

#### Lint とフォーマット

```bash
cd firmware
cargo fmt
cargo clippy
```

### コントローラー（controller/）

デスクトップGUIアプリケーション - モータードライバーをCAN経由で制御・監視

#### ビルドと実行

```bash
# コントローラーディレクトリに移動
cd controller

# ビルドと実行
cargo run

# リリースビルド
cargo run --release
```

#### 機能

- **CAN接続管理**: socketcanインターフェース（`can0`、`vcan0`、`slcan0`など）への接続
- **モーター制御**: 速度指令、PIゲイン設定、モーター有効/無効切替
- **ステータス監視**: リアルタイムでモーター速度、電気角、電圧状態を表示
- **設定UI**: PIゲイン、角度オフセット、最小電圧などの動的調整

#### 技術スタック

- **Dioxus**: Reactライクなデスクトップアプリケーションフレームワーク
- **tokio-socketcan**: 非同期CAN通信
- **tracing**: 構造化ログ

### CANデバッグスクリプト（scripts/can.sh）

コマンドライン経由でモーターを制御・デバッグするためのBashスクリプト

#### 使い方

```bash
# スクリプトに実行権限を付与（初回のみ）
chmod +x scripts/can.sh

# 使用方法を表示
./scripts/can.sh

# 速度指令を送信（1000 RPM）
./scripts/can.sh speed 1000

# PIゲインを設定
./scripts/can.sh pi 0.5 0.05

# モーター有効化
./scripts/can.sh enable

# ステータスを監視
./scripts/can.sh monitor

# テストシーケンスを実行（自動ランプアップ/ダウン）
./scripts/can.sh test

# 異なるCANインターフェースを使用
CAN_INTERFACE=can0 ./scripts/can.sh speed 500
```

#### サポートされるコマンド

- `speed <RPM>` - 速度指令送信
- `pi <Kp> <Ki>` - PIゲイン設定
- `enable` / `disable` - モーター有効/無効
- `estop` - 緊急停止
- `monitor` - ステータスメッセージ監視（0x200、0x201）
- `dump` - 全CANトラフィックをダンプ
- `sniffer` - インタラクティブCANスニファー
- `test` - 自動テストシーケンス実行

### CANインターフェース設定

#### 仮想CAN（開発/テスト用）

```bash
# vcan0の作成
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

#### ハードウェアCAN

```bash
# 250kbpsで設定（ファームウェアと一致）
sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0
```

#### slcan（USB-CANアダプター）

```bash
# slcanインターフェースのセットアップ（250kbps）
sudo slcand -o -c -s6 /dev/ttyUSB0 slcan0
sudo ip link set up slcan0

# または scripts/can.sh が自動的にセットアップを試みます
CAN_INTERFACE=slcan0 ./scripts/can.sh monitor
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
- **制御周期**: 2.5kHz（400μs）
- **最大電圧**: 24V
- **DCバス電圧**: 24V
- **最大Duty比**: 100（PWM範囲：0=0%, 100=100%）
- **速度フィルタ係数**: 0.1（Hallセンサ用ローパスフィルタ、滑らかな速度推定のため低減）
- **デフォルトPI ゲイン**: Kp=0.5、Ki=0.05（角度補間により制御精度向上、安定性重視）
- **Hall角度オフセット**: 0度（ハードウェアに応じて調整可能）
- **Hall角度補間**: 有効（連続的な角度推定により制御安定性向上）

## ソフトウェア構造

### ファームウェアモジュール構成

ファームウェアは以下のモジュールで構成されています：

- **[firmware/src/main.rs](firmware/src/main.rs)** - メインエントリーポイント、ハードウェア初期化、タスク起動
- **[firmware/src/config.rs](firmware/src/config.rs)** - 全設定パラメータの集約（モーター、PWM、CAN、オープンループ等）
- **[firmware/src/state.rs](firmware/src/state.rs)** - グローバル共有状態（Mutex保護）
- **[firmware/src/hardware.rs](firmware/src/hardware.rs)** - ハードウェア初期化ロジック（クロック設定、割り込み設定等）
- **[firmware/src/hall_tim.rs](firmware/src/hall_tim.rs)** - TIM4ハードウェアHallセンサーインターフェース（XORモード）
- **[firmware/src/benchmark.rs](firmware/src/benchmark.rs)** - FOC関数のパフォーマンス測定
- **[firmware/src/voltage_monitor.rs](firmware/src/voltage_monitor.rs)** - DCバス電圧監視モジュール
- **[firmware/src/tasks/](firmware/src/tasks/)** - 非同期タスク実装
  - [led.rs](firmware/src/tasks/led.rs) - LED点滅タスク
  - [can.rs](firmware/src/tasks/can.rs) - CAN通信タスク
  - [motor_control.rs](firmware/src/tasks/motor_control.rs) - モーター制御タスク
  - [voltage_monitor.rs](firmware/src/tasks/voltage_monitor.rs) - 電圧監視タスク
- **[firmware/src/foc/](firmware/src/foc/)** - FOC制御アルゴリズム実装
- **[firmware/src/can_protocol.rs](firmware/src/can_protocol.rs)** - CANプロトコル定義とパーサー
- **[firmware/src/fmt.rs](firmware/src/fmt.rs)** - ログマクロ（defmt/core切り替え）

### コントローラーモジュール構成

コントローラーは以下のモジュールで構成されています：

- **[controller/src/main.rs](controller/src/main.rs)** - Dioxusアプリケーションのエントリーポイント
- **[controller/src/state.rs](controller/src/state.rs)** - アプリケーション状態管理（CAN接続、モーターステータス等）
- **[controller/src/can/](controller/src/can/)** - CAN通信モジュール
  - [mod.rs](controller/src/can/mod.rs) - CAN モジュール公開インターフェース
  - [protocol.rs](controller/src/can/protocol.rs) - CANプロトコル実装（firmwareと共通のロジック）
  - [manager.rs](controller/src/can/manager.rs) - CAN接続管理、送受信ロジック
- **[controller/src/ui/](controller/src/ui/)** - Dioxus UIコンポーネント
  - [mod.rs](controller/src/ui/mod.rs) - UI モジュール公開インターフェース
  - [connection.rs](controller/src/ui/connection.rs) - CAN接続UI（接続バー、インターフェース選択）
  - [control.rs](controller/src/ui/control.rs) - モーター制御UI（速度指令、有効/無効ボタン）
  - [settings.rs](controller/src/ui/settings.rs) - 設定UI（PIゲイン、オフセット、最小電圧等）

### Embassy非同期ランタイム
- `#[embassy_executor::main]` でメインループ
- `#[embassy_executor::task]` で非同期タスク生成
- Embassy Time で遅延制御

### 主要タスク

#### 1. led_task ([firmware/src/tasks/led.rs](firmware/src/tasks/led.rs))
3つのLEDを500msごとに順次点灯（動作確認用）

#### 2. motor_control_task ([firmware/src/tasks/motor_control.rs](firmware/src/tasks/motor_control.rs))
**2.5kHz FOC制御ループ** - BLDCモーターのField Oriented Control（オープンループ始動付き）
1. モーター使能チェック（無効時はPWM停止、PIリセット）
2. TIM4ハードウェアからHallセンサ状態取得（hall_tim::get_hall_state()）
3. 周期・速度計算（hall_tim::get_period_cycles() → calculate_speed_rpm()）
4. 電気角推定（HallSensor::update + オフセット適用）
5. **制御モード分岐**:
   - **OpenLoop**: 6ステップ駆動（台形波）で始動、目標RPM到達でFOCに切替
   - **ClosedLoopFoc**: 以下のFOC制御ループ
6. PIゲイン更新チェック（CAN経由で変更された場合）
7. 目標速度取得（CAN経由で設定）
8. 速度PI制御（q軸電圧指令生成、d軸は0）
9. 最小電圧適用（静止摩擦克服用、速度誤差>10RPMの場合）
10. 電圧ベクトル制限（円形リミッタ）
11. Park逆変換（dq → αβ座標）
12. SVPWM計算（αβ → UVW Duty比）
13. PWM出力（TIM1への設定）
14. ステータス更新（CAN送信用）
15. デバッグログ（1秒ごと）

#### 3. can_task ([firmware/src/tasks/can.rs](firmware/src/tasks/can.rs))
**CAN通信タスク** - モーター制御コマンド受信とステータス送信
- 100ms周期でステータス送信（ID 0x200: 速度RPM + 電気角）
- コマンド受信処理：
  - `0x100`: 速度指令（f32 RPM、4バイト）
  - `0x101`: PIゲイン設定（Kp、Kiそれぞれf32、8バイト）
  - `0x102`: モーター使能（u8: 0=無効、1=有効）
  - `0x000`: 緊急停止（即座にモーター停止、速度0）

#### 4. voltage_monitor_task ([firmware/src/tasks/voltage_monitor.rs](firmware/src/tasks/voltage_monitor.rs))
**電圧監視タスク** - DCバス電圧の監視と保護
- PC1ピン（ADC2_IN7）でDCバス電圧を監視
- 分圧回路（100kΩ + 10kΩ）で最大36.3V測定可能
- ローパスフィルタで電圧平滑化（α=0.1）
- 過電圧（>30V）/低電圧（<10V）検出
- ステータスをstate::VOLTAGE_STATEに更新（CAN送信用）

### FOCモジュール（[firmware/src/foc.rs](firmware/src/foc.rs)）

#### HallSensor ([firmware/src/foc/hall_sensor.rs](firmware/src/foc/hall_sensor.rs))
- Hall状態（1-6）から電気角推定（セクター中心：30, 90, 150, 210, 270, 330度）
  - **重要**: セクターの中心角を使用することでFOC制御の精度向上
- **角度補間機能**（デフォルト有効）：
  - Hall エッジ間で速度ベースの角度補間を実施
  - 離散的な60度ステップから連続的な角度推定へ改善
  - FOC制御の安定性と滑らかさが大幅に向上
- Hall エッジ検出による速度計算（RPM）
- ローパスフィルタによる速度平滑化
- タイムアウト検出（1秒間エッジなし → 速度0）

#### PiController ([firmware/src/foc/pi_controller.rs](firmware/src/foc/pi_controller.rs))
- 比例・積分制御（anti-windup機能付き）
- **積分項計算**: `integral += ki * error * dt` 形式（[calebfletcher/foc](https://github.com/calebfletcher/foc)実装準拠）
  - 数値安定性向上
  - ゲイン変更時の挙動改善
- **アンチワインドアップ**: デフォルトで無効（参照実装準拠）
  - 出力飽和時も積分項が蓄積される
  - モーター制御の安定性向上
- 出力リミッタ（±max_voltage）
- ゲイン・リミット動的変更可能
- `new_symmetric()` で±リミット設定

#### SVPWM ([firmware/src/foc/svpwm.rs](firmware/src/foc/svpwm.rs))
- **高速x/y/z座標変換方式**（[calebfletcher/foc](https://github.com/calebfletcher/foc)実装準拠）
  - 三角関数（`atan2f`, `sinf`）を使わず、符号判定でセクター決定
  - 計算負荷を大幅削減（組み込み最適化）
  - 精度と安定性が向上
- Space Vector PWM生成（正弦波PWMより15%電圧利用率向上）
- セクター判定（1-6）とデューティ比計算
- `calculate_sinusoidal_pwm()` も実装（シンプル版、後方互換性用）

#### Transforms ([firmware/src/foc/transforms.rs](firmware/src/foc/transforms.rs))
- `inverse_park()`: dq → αβ座標変換（回転座標系 → 静止座標系）
- `inverse_clarke()`: αβ → UVW 3相変換
- `limit_voltage()`: dq電圧ベクトルの円形制限
- `normalize_angle()`: 角度正規化（0～2π）

#### OpenLoopRampUp ([firmware/src/foc.rs:15-124](firmware/src/foc.rs#L15-L124))
- 始動時のオープンループ制御（強制転流）
- 角速度のランプアップ（加速度指定）
- 目標速度到達後にFOC制御へ移行
- **注**: 現在のmain.rsでは未使用（将来の始動制御用）

### TIM4 Hallセンサーインターフェース（[firmware/src/hall_tim.rs](firmware/src/hall_tim.rs)）
**STM32ハードウェアHall Sensor Interface Mode（XORモード）実装**
- PB6/PB7/PB8（TIM4_CH1/CH2/CH3）でHallセンサー入力
- 3つのHall信号をXORしてTI1に接続（自動エッジ検出）
- Input Captureで各エッジのタイムスタンプをキャプチャ
- CC1割り込みでエッジ間周期から速度計算
- UPDATE割り込みでタイムアウト検出（モーター停止判定）
- Atomic変数でロックフリー実装（割り込みハンドラ↔制御ループ間）
- 170MHzクロック、フルスピード動作（PSC=0）
- `get_hall_state()`: Hall状態取得（1-6）
- `get_period_cycles()`: エッジ間サイクル数取得
- `calculate_speed_rpm()`: 周期からRPM計算
- `is_timeout()`: タイムアウトフラグ確認（1秒間エッジなし）

**メリット**: ソフトウェアポーリング不要、マイクロ秒精度のタイムスタンプ、CPUオーバーヘッド最小化

### CANプロトコル

**共通プロトコル**: ファームウェア、コントローラー、スクリプトは全て同じCANプロトコルを使用

#### CAN ID定義

- `0x100`: 速度指令（Host → Driver）- f32 RPM、4バイト
- `0x101`: PIゲイン設定（Host → Driver）- Kp: f32、Ki: f32、8バイト
- `0x102`: モーター有効/無効（Host → Driver）- u8: 0=無効、1=有効
- `0x200`: モーターステータス（Driver → Host）- 速度: f32 RPM、電気角: f32 rad、8バイト
- `0x201`: 電圧ステータス（Driver → Host）- 電圧: f32 V、フラグ: u8、5バイト
- `0x000`: 緊急停止（Host → Driver）- 任意のデータ

#### 実装

- **ファームウェア**: [firmware/src/can_protocol.rs](firmware/src/can_protocol.rs)
  - パース関数: `parse_speed_command()`, `parse_pi_gains()`, `parse_enable_command()`
  - エンコード関数: `encode_status()`, `decode_status()`
  - 全てリトルエンディアンf32形式
  - テストコード付き（`#[cfg(test)]`）
- **コントローラー**: [controller/src/can/protocol.rs](controller/src/can/protocol.rs)
  - ファームウェアと同じプロトコルロジックを実装
  - エンコード/デコード関数
- **スクリプト**: [scripts/can.sh](scripts/can.sh)
  - Python3 `struct.pack('<f')` でf32をリトルエンディアンに変換
  - `candump`/`cansend` でCAN通信

### Config/State/Hardware モジュール
- **[firmware/src/config.rs](firmware/src/config.rs)**: 全設定パラメータを集約（マジックナンバー排除）
  - モーターパラメータ（極対数、電圧、PIゲイン等）
  - PWM設定（周波数、デッドタイム）
  - CAN設定（ビットレート）
  - オープンループ始動パラメータ
- **[firmware/src/state.rs](firmware/src/state.rs)**: タスク間共有状態（Mutex保護）
  - `TARGET_SPEED`: 目標速度 [RPM]
  - `SPEED_PI_GAINS`: PIゲイン (Kp, Ki)
  - `MOTOR_ENABLE`: モーター有効/無効フラグ
  - `MOTOR_STATUS`: モーターステータス（CAN送信用）
  - `VOLTAGE_STATE`: 電圧監視ステータス（CAN送信用）
- **[firmware/src/hardware.rs](firmware/src/hardware.rs)**: ハードウェア初期化ロジック
  - `create_clock_config()`: RCC/PLL設定（170MHz生成）
  - `init_hall_sensor()`: TIM4 Hallインターフェース初期化
  - CAN割り込みバインディング（`Irqs`）

### defmt ログマクロ（[firmware/src/fmt.rs](firmware/src/fmt.rs)）
- `defmt` フィーチャーの有無で `core` と `defmt` のマクロを切り替え
- `trace!`、`debug!`、`info!`、`error!` などのログマクロ
- `unwrap!`、`assert!` などのユーティリティマクロ
- `defmt` 無効時もコンパイルエラーにならないよう互換性確保

### Benchmark（[firmware/src/benchmark.rs](firmware/src/benchmark.rs)）
- DWTサイクルカウンタを使用したパフォーマンス測定
- `run_inverse_park_benchmark()`: inverse_park()のベンチマーク実行
- idsp vs libm の三角関数実装比較（起動時に自動実行）

## ビルド最適化

- 開発ビルド: `opt-level = "z"`（サイズ最適化）、`debug = true`、`incremental = true`
- リリースビルド: `opt-level = 2`（速度最適化）、`debug = false`、`incremental = true`
- LTO有効（両ビルド）
- `build-std = ["core"]` でコア標準ライブラリを再ビルド
- `panic_immediate_abort` でパニックサイズ削減

## 重要な制約

### ファームウェアの制約

- `#![no_std]`、`#![no_main]`: 標準ライブラリ不使用の bare metal 環境
- 浮動小数点演算に `libm` クレートを使用（`roundf`、`sin`、`cos` など）
  - **最適化**: SVPWMでは三角関数を使わない高速方式を採用
- メモリ制約の厳しい組み込み環境のため、ヒープアロケーション非推奨
- FOC制御は2.5kHz（400μs周期）ループで実行されるため、処理時間に注意（400μs以内に完了必要）
  - SVPWM最適化により計算負荷を削減

### コントローラーの制約

- 標準ライブラリ（`std`）を使用可能
- Dioxusデスクトップアプリケーション（非同期ランタイム: tokio）
- socketcanを使用するため、Linux環境が必要（Windows/macOSはWSL/VM経由）

## 最適化履歴と設計判断

### TIM4ハードウェアHallセンサーインターフェース
**STM32組み込みペリフェラルを活用した最適化**
- ソフトウェアポーリング不要：ハードウェアが自動的に3つのHall信号をXORしてエッジ検出
- マイクロ秒精度のタイムスタンプ：170MHzタイマーで高精度周期測定
- CPUオーバーヘッド最小化：割り込みハンドラは数十サイクルで完了
- Atomic変数でロックフリー実装：割り込みハンドラとFOCループ間の競合を排除
- リアルタイム性向上：エッジ検出～速度計算のレイテンシを大幅削減

### FOC制御アルゴリズム最適化
参照実装 [calebfletcher/foc](https://github.com/calebfletcher/foc) との比較により最適化：

1. **SVPWM実装の改善**（[firmware/src/foc/svpwm.rs](firmware/src/foc/svpwm.rs)）
   - 三角関数ベース → x/y/z座標変換+符号判定方式に変更
   - `atan2f`と`sinf`を削除、計算負荷を大幅削減
   - セクター判定がシンプルかつ高速に
   - 制御安定性が向上

2. **PI制御の改善**（[firmware/src/foc/pi_controller.rs](firmware/src/foc/pi_controller.rs)）
   - 積分項計算: `integral += error * dt; ki * integral` → `integral += ki * error * dt`
   - 数値安定性向上
   - ゲイン動的変更時の挙動改善

3. **アンチワインドアップの修正**（[firmware/src/foc/pi_controller.rs](firmware/src/foc/pi_controller.rs)）
   - デフォルト設定: 有効 → **無効** に変更
   - 参照実装では出力飽和時も積分項が蓄積され続ける
   - これによりモーター始動時や負荷変動時の応答性が向上
   - 制御の安定性が大幅に改善

### コードモジュール化（Rust 2018+スタイル）
- タスクを`src/tasks/`に分離（led, can, motor_control, voltage_monitor）
- 設定パラメータを`config.rs`に集約（マジックナンバー排除）
- グローバル状態を`state.rs`に集約（Mutex保護）
- ハードウェア初期化を`hardware.rs`に分離
- 可読性・保守性・テスタビリティの向上
