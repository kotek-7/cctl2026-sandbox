# DualSenseで手動操縦

この1台専用の最小構成。DualSenseとCCTLをそれぞれLinux PCへUSB接続する。
PCはRustで入力を送るだけ。r/θ/z・EEの制御はCCTLで行う。

## ビルドと起動

PCにはRust、pkg-config、libudev-devが必要。
STM32にはarm-none-eabi-gcc/g++、CMake、Ninja、SVMDにはPlatformIOを使う。

```sh
make build
make build-cctl
make build-serial-svmd
make build-svmd
cargo run --release --locked -- /dev/serial/by-id/対象CCTL
```

**PCと3基板をこの版で揃える。旧FWとの混在には対応しない。**
ビルドでは書き込まない。実機動作は未確認。
DualSenseは起動前に接続する。アクセス権のあるユーザーで実行する。
STS3215はID=1、1Mbps、位置モードに設定済みであること。

## 操作

1. 出力OFFの状態でrを前端、zを下端、θを正面に手動で合わせる。
2. Createを押して現在位置を原点にする。
3. Optionsで運転を開始する。入力中でもそのまま開始する。

| 入力 | 動作 |
|---|---|
| 左スティック上下・左右 | r・θ |
| R2・L2 | zの正・負方向 |
| 右スティック左右 | EE回転。θ回転を補償 |
| 十字キー上下 | 畳みを動かす |
| 十字キー左・右・○ | 把持を開く・閉じる・受け渡し幅 |
| L1 | 軸・回転入力を30%にする |
| PS | 3軸を現在位置で保持、EE出力OFF |
| Options | 保持から再開 |
| Ctrl+C / パッド切断 | PC終了。指令途絶から250ms後に基板側で出力OFF |

初回開始時のPWM目標は全て500µs、EE回転は1024カウント。
現在のサーボ位置を読まないため、開始すると指定姿勢へ動く。
1024は元設定のままで実機校正していない。
通信断の出力OFFは、PSの保持停止とは異なり重力を保持しない。

## 読む順番

- `src/main.rs` → `src/pad.rs`：入力・送信。
- `cctl/src/app.cpp` → `motors.cpp` / `ee.cpp`：基板のループ・機体制御。
- `cctl/src/config.hpp`：機体定数。変更後は再ビルド・書込み。
- `serial_svmd/src/app.cpp`、`svmd/src/main.cpp`：CAN目標値の出力。

設定ファイル、ACK待ち、応答照合、自動復旧はない。
通信形式はDESIGN.md、検証結果はVERIFICATION.md。`make test`で通信符号化・PIDの小さなテストを実行する。

## フォーマット

Rustはrustfmt、C/C++はclang-format 18.1.8を使用する。

```sh
rustup component add rustfmt
uv tool install clang-format==18.1.8
make format
make format-check
```

設定はルートの `rustfmt.toml` と `.clang-format`。どちらも4スペース・100桁。
対象はRust本体と、各基板の `src/`・`tests/` のGit管理された自作C/C++コード。
新しいC/C++ファイルは `git add` 後に実行する。
生成コード、外部ライブラリ、`led-music/` は対象外。CIでも `make format-check` を実行する。

VSCodeのSTM32Cube clangd拡張はフォーマット機能が既定でOFFのため、
`stm32cube-ide-clangd.enableFormatting: true` と、C/C++の
`editor.defaultFormatter: "STMicroelectronics.stm32cube-ide-clangd"` を指定する。
`cctl/` と `serial_svmd/` のVSCode設定には指定済み。
リポジトリ全体を開く場合はルートの `.vscode/settings.json` に同じ設定を置く
（ルートのエディタ設定は個人設定としてGit管理対象外）。
設定後に「Developer: Reload Window」を実行し、C/C++ファイルで
「Format Document」（LinuxではShift+Alt+F）を使う。
整形ルールは共通の `.clang-format` を読む。拡張内のclangdとCLIのclang-formatは
バージョンが異なる場合があるため、コミット前は `make format-check` で確認する。
