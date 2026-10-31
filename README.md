# DualSenseで手動操縦

DualSenseとCCTLを、それぞれLinux PCへUSB接続する。
PC側はRust。`gilrs`で入力を読み、`serialport`で既存CCTLへ直接送る。
設定は `machine.toml`。機体のr/θ/zとEEだけを扱う。

## 起動

Rust/Cargoとlibudev開発パッケージを使用する。
Ubuntuでは `sudo apt install build-essential pkg-config libudev-dev`。
ゲームパッドとUSBシリアルにアクセスできるユーザーで実行する。

```sh
cd /home/kotek/dev/cctl2026-sandbox
cargo run --release --locked -- /dev/serial/by-id/対象CCTL
```

DualSenseは起動前に1台接続する。通常版とEdgeに対応する入力IDを使用。
切断や異常時は終了するため、原因を直してから起動し直す。

## 操作

1. 出力解除中に、rを前端接点、zを下端接点、θを正面へ手動で合わせる。
2. 全入力を離してからCreateを1秒長押しし、原点を確定する。
3. 再び全入力を離し、Optionsで運転を開始する。

| 入力 | 動作 |
|---|---|
| 左スティック上下・左右 | r・θ |
| R2・L2 | zの正・負方向 |
| 右スティック左右 | EE回転の微調整。θ回転を補償 |
| 十字キー上下 | 押している間、畳みを操作 |
| 十字キー左・右 | 把持を開く・閉じる |
| ○ | 把持を受け渡し幅にする |
| L1 | 低速 |
| PS | 3軸を保持停止、EEは出力解除 |
| Ctrl+C / SIGTERM | 全出力解除を送信して終了 |

保持停止からの再開も、全入力を離してからOptions。
自動原点走行、端末コマンド、GUI、操作API、再接続処理はない。
全出力解除時は重力による動きを保持しない。

PWMは実測位置を読めないため、起動時に出力無効のまま `initial_us` を設定し、
Optionsで有効化するとその姿勢へ動く。現在値は4本とも500 µs。
EE回転の `zero0_count=1024` は元設定のままで、実機校正は未実施。

## コードを読む

本体は `src/` の3モジュール。

| ファイル | 内容 |
|---|---|
| `main.rs` | パッド入力、起動・終了、単一の操縦ループ |
| `robot.rs` | 設定の読込み、座標・制動・サーボの計算 |
| `board.rs` | USB/CANの形式、応答読取り、起動時の設定照合 |

`main.rs` の `drive()` を読むと操縦の流れを追える。
設定は起動時に順番に書込み、操縦中のループへ設定同期処理を持ち込まない。
制御器クラス、指令型、要求キュー、独立した状態更新層は使わない。
受信値と入力値は小さな構造体に保持する。

`cctl/`、`serial_svmd/`、`svmd/` は既存基板FWと必要なライブラリ。
元のモータ保護・通信ドライバを保持している。書込みは自動実行しない。
`led-music/` は別プロジェクトとして変更・追跡しない。

## 検証

```sh
make test
make build-cctl
make build-serial-svmd
make build-svmd
```

模擬基板は `tests/rust/` 内だけにある。
実機の駆動・停止距離・入力方向・サーボ初期姿勢は未確認。
検証内容は `VERIFICATION.md` に記録する。
