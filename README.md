# DualSenseで手動操縦する最小構成

DualSenseと既存CCTLを、それぞれLinux PCへUSB接続する。
CCTLから先のCAN配線・基板・モータは元の機体を使用する。
対象はr/θ/z、EE回転・畳み・把持と手動原点設定。

## 起動

Rust/Cargo、C/C++ビルド環境、pkg-config、libudev開発パッケージが必要。
Ubuntuでは `sudo apt install build-essential pkg-config libudev-dev`。
USBシリアルとゲームパッドのアクセス権を持つユーザーで起動する。

```sh
./operate build
./operate run /dev/serial/by-id/対象CCTL
```

接続先は必ず指定する。DualSense（通常版/Edge）は1台だけ接続する。
設定は直下の `machine.toml`。元機体の換算・方向・制限値を継承している。
基板のパラメータ応答と照合が完了すると `configured=true` になる。
照合不一致・通信異常時は停止して終了する。原因を直して再起動する。

## 原点設定と操作

1. 起動時は全出力解除。rを前端接点、zを下端接点、θを正面へ手動で合わせる。
2. ボタンを離し、スティック・トリガーを中立にする。
3. Createを1秒長押しする。r/z両接点を確認して3軸の原点を採用する。
4. 一度すべて離してからOptionsを押すと運転開始。

自動で原点へ走行する処理はない。θ正面は目視で合わせる。
PWMサーボには実測位置の読取りがないため、アプリ起動時に `initial_us` を
出力無効のまま設定し、Optionsで有効化する際にその姿勢へ動く。
現在値は畳み・把持とも500 µs。実機の機構と設定の対応を確認すること。
EE回転の `zero0_count` は元設定の1024を引き継いでおり、このプロジェクトでは実機校正していない。

| 操作 | 動作 |
|---|---|
| 左スティック上下 / 左右 | r / θ |
| R2 / L2 | zの正 / 負方向 |
| 右スティック左右 | EE回転の向きを微調整（θ回転を補償） |
| 十字キー上下 | 押している間、EE畳みを操作 |
| 十字キー左 / 右 | 把持を開く / 閉じる |
| ○ | 把持を受け渡し用の幅にする |
| L1 | 低速操作 |
| PS / 端末の `stop` | 3軸を保持して停止、EEは出力解除 |
| 端末の `estop` | 全出力解除、原点を破棄 |
| 端末の `reset` | 緊停フラグのみ解除。原点再設定が必要 |
| `quit` / Ctrl+C / 端末入力終了 | 全出力解除を送って終了 |

パッド切断・通信途絶・制御ループ遅延・基板異常で出力解除する。
保持停止から再開する際も、一度すべての入力を離してOptionsを押す。
全出力解除後は重力による動きを保持しない。

## コードの構成

このGitプロジェクトのルートは `/home/kotek/dev/cctl2026-sandbox`。
元の `/home/kotek/dev/catchrobo2026` からcloneした履歴を保持している。
`led-music/` は以前の別プロジェクトで、このGit管理から除外している。

PC側は `host/src/` にある。最初に `main.rs`、次に `control.rs` を読む。

| ファイル | 内容 |
|---|---|
| `main.rs` | gilrs・serialport・端末を直接使う20 msのループ |
| `input.rs` | DualSenseの固定ボタン割当 |
| `control.rs` | 原点設定、運転、保持停止、異常時の出力解除 |
| `motion.rs` | r/θ/zの換算・制動・可動域制限、EE回転補償 |
| `config.rs` | この機体専用の設定と入力検証 |
| `settings.rs` | 起動時の基板パラメータ書込み・応答照合 |
| `protocol/` | 既存基板の通信形式 |
| `simulator.rs` | 使用する指令だけの模擬基板 |

設定は `machine.toml` の1つ。旧worker、API、汎用プロファイル、GUI、
自動シーケンス、XY制御、ボーナス/DC、旧構成の回帰テストは削除した。
旧構成を有効化する機能フラグもない。

`cctl/`・`serial_svmd/`・`svmd/` は既存基板のFW。
HAL/CMSIS/USBと実モータのドライバ・保護処理は必要な依存として保持する。
`tests/` はこの3基板のロジックと通信ドライバの検証。

## 検証とファームウェア

```sh
./operate sim               # 模擬通信 + 接続したDualSense
host/target/debug/simple --simulate --no-gamepad  # パッドなしで起動・停止を確認
./operate test              # PC側・基板ロジックの非実機テスト
make build-cctl
make build-serial-svmd
make build-svmd
```

STM32ビルドにはCMake/Ninja/Arm GNU Toolchain、SVMDにはPlatformIOが必要。
`make build-svmd PIO=/path/to/pio` で実行ファイルを指定できる。
書込みは自動実行しない。模擬機体は通信・状態遷移の検証用で、衝突・重力・制動距離を再現しない。
本変更では実機への書込み・モータ駆動は行っていない。
