# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

STM32G431VBTxマイコンを使用したBLDCモータードライバー。Hall センサベースの FOC（Field Oriented Control）実装で、CAN 通信によるモーター制御を行う Embassy 非同期フレームワークベースの組み込み Rust プロジェクト。

**プロジェクト構造**（ワークスペース）:
- `firmware/` - STM32組み込みファームウェア（`no_std`、Embassy ベース）
- `controller/` - CAN通信用デスクトップGUIコントローラー（Dioxus、`std`）
- `bldc/` - ハードウェア非依存BLDCモーター制御ライブラリ（`no_std`対応）
- `bldc-sim/` - FOC制御検証用物理シミュレーション
- `protocol/` - CAN通信プロトコル定義（ファームウェア/コントローラー共有）
- `scripts/` - CANデバッグ用Bashスクリプト

**参考資料**:
- [STSPIN32G4 Datasheet (PDF)](https://www.st.com/resource/en/datasheet/stspin32g4.pdf)
- [STSPIN32G4 Product Page](https://www.st.com/en/motor-drivers/stspin32g4.html)

## 開発コマンド

**重要**: 各プロジェクトは独立しているため、作業ディレクトリに注意。

### ファームウェア（firmware/）

```bash
cd firmware

# ビルドしてデバイスにフラッシュ・実行
cargo run

# リリースビルド
cargo run --release

# ビルドのみ
cargo build

# Lint とフォーマット
cargo fmt
cargo clippy
```

### コントローラー（controller/）

```bash
cd controller

# ビルドと実行
cargo run

# リリースビルド
cargo run --release
```

### テスト（ルートディレクトリ）

```bash
# ワークスペース全体のテスト（bldc、bldc-sim、protocol）
cargo test

# 特定クレートのテスト
cargo test -p bldc
cargo test -p bldc-sim
cargo test -p g4-driver-protocol
```

**注意**: ファームウェアは`no_std`のためホストでのテスト不可。`bldc`クレートでアルゴリズムをテスト。

### Pre-commit hooks

```bash
# セットアップ（初回のみ）
pip install pre-commit
pre-commit install

# 手動実行
pre-commit run --all-files
```

### CANデバッグスクリプト（scripts/can.sh）

```bash
./scripts/can.sh speed 1000    # 速度指令（1000 RPM）
./scripts/can.sh pi 0.5 0.05   # PIゲイン設定
./scripts/can.sh enable        # モーター有効化
./scripts/can.sh monitor       # ステータス監視
./scripts/can.sh test          # テストシーケンス

# 別のCANインターフェース使用
CAN_INTERFACE=can0 ./scripts/can.sh speed 500
```

### CANインターフェース設定

```bash
# 仮想CAN（開発/テスト用）
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# ハードウェアCAN（250kbps）
sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0
```

## アーキテクチャ

### ハードウェア構成
- **マイコン**: STM32G431VBTx（Cortex-M4、170MHz）
- **ターゲット**: thumbv7em-none-eabi
- **デバッグ**: probe-rs経由のSWD/JTAG

### 主要ペリフェラル
- **TIM1**: 3相補完PWM出力（50kHz）- PE9/PE8、PE11/PE10、PE13/PE12
- **TIM4**: Hallセンサーインターフェース（XORモード）- PB6/PB7/PB8
- **OPAMP1/2/3**: 電流センシング用アンプ
- **ADC1/ADC2**: 電流値読み取り
- **FDCAN1**: CAN通信（PA11=RX、PA12=TX、250kbps）
- **GPIO**: PC13/PC14/PC15にLED接続

### モーター制御パラメータ
- **極対数**: 6
- **制御周期**: 2.5kHz（400μs）
- **DCバス電圧**: 24V
- **デフォルトPIゲイン**: Kp=0.5、Ki=0.05

### ファームウェアアーキテクチャ

**Embassy非同期ランタイム**で以下のタスクを並行実行:

1. **motor_control_task** (2.5kHz) - FOC制御ループ
   - OpenLoop: 6ステップ駆動で始動
   - ClosedLoopFoc: 速度PI制御 → Park逆変換 → SVPWM → PWM出力
   - Calibration: 電気角オフセット自動検出

2. **can_task** - CAN通信（100ms周期でステータス送信）

3. **voltage_monitor_task** - DCバス電圧監視（過電圧/低電圧保護）

4. **led_task** - 動作確認用LED点滅

**主要モジュール**:
- `config/` - パラメータ定義、EEPROM永続化
- `state.rs` - タスク間共有状態（Mutex保護、コンテキストベース管理）
- `hall_tim.rs` - TIM4ハードウェアHallインターフェース
- `motor_driver.rs` - PWM制御抽象化

### bldcクレート（ハードウェア非依存）

ファームウェアのFOCロジックを分離した`no_std`対応ライブラリ:
- `pi_controller.rs` - PI制御器（アンチワインドアップ対応）
- `svpwm.rs` - 空間ベクトルPWM
- `transforms.rs` - Clarke/Park変換
- `hall_sensor.rs` - Hallセンサー状態管理

**フィーチャフラグ**: `hall`（デフォルト）、`calibration`、`encoder`、`sensorless`、`std`

### コントローラーアーキテクチャ

**Dioxus**デスクトップアプリケーション + **tokio-socketcan**非同期CAN通信

- `state.rs` - アプリケーション状態管理
- `can/` - CAN通信（protocol、manager、setup）
- `ui/` - Dioxus UIコンポーネント（connection、control、settings）

### protocolクレート

ファームウェアとコントローラーで共有するCANプロトコル定義。`defmt`フィーチャでファームウェア向けデバッグ出力対応。

### CANプロトコル

| CAN ID | 方向 | 内容 |
|--------|------|------|
| 0x100 | Host→Driver | 速度指令（f32 RPM、4B） |
| 0x101 | Host→Driver | PIゲイン（Kp: f32、Ki: f32、8B） |
| 0x102 | Host→Driver | モーター有効/無効（u8、1B） |
| 0x200 | Driver→Host | ステータス（速度: f32、電気角: f32、8B） |
| 0x201 | Driver→Host | 電圧（電圧: f32、フラグ: u8、5B） |
| 0x000 | Host→Driver | 緊急停止 |

全てリトルエンディアンf32形式。

## 重要な制約

### ファームウェア
- `#![no_std]`、`#![no_main]`: bare metal環境
- 浮動小数点演算に`libm`、高速三角関数に`idsp`使用
- FOC制御は400μs以内に完了必要
- ヒープアロケーション非推奨

### コントローラー
- Linux環境必須（socketcan使用）
- Windows/macOSはWSL/VM経由

## 設計判断と最適化

### SVPWM最適化
三角関数を使わないx/y/z座標変換方式を採用（[calebfletcher/foc](https://github.com/calebfletcher/foc)準拠）。`atan2f`/`sinf`を削除し計算負荷を大幅削減。

### PI制御
- 積分項計算: `integral += ki * error * dt`形式で数値安定性向上
- アンチワインドアップ: デフォルト無効（応答性優先）

### TIM4 Hallインターフェース
STM32ハードウェアXORモードでソフトウェアポーリング不要。170MHzタイマーでマイクロ秒精度のタイムスタンプ。Atomic変数でロックフリー実装。

### 状態管理
12個の独立したMutex → 3つのコンテキスト（MotorContext、CalibrationContext、SystemContext）にグループ化。責任範囲を明確化。
