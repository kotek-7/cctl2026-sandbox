# DualSenseで手動操縦

DualSenseとCCTLをそれぞれLinux PCへUSB接続して、r・θ・zとEEを操作する。
PCは入力を20msごとに送り、CCTLが機体を制御する。

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

hostとCCTLは同じ版を使用する。上記のコマンドはビルドとPCプログラムの起動用で、
基板への書込みは別途必要。実機動作は未確認。
DualSenseは起動前に接続する。アクセス権のあるユーザーで実行する。
STS3215はID=1、1Mbps、位置モードに設定済みであること。

## 操作

DualSenseとCCTLを接続してPCプログラムを起動する。
有効なPC入力と正常なモータ応答が揃うと、起動位置から手動で操作できる。

| 入力 | 動作 |
|---|---|
| 左スティック上下・左右 | r・θ |
| R2・L2 | zの正・負方向 |
| 右スティック左右 | EEの相対角度。θとは独立 |
| 十字キー上下 | 畳みを動かす |
| 十字キー左・右 | 把持を開く・閉じる |
| L1 | r・θ・z・EE回転の入力を30%にする |
| Ctrl+C / パッド切断 | PC終了。指令途絶から250msを超えると基板側で出力OFF |

スティックを離すと減速して位置を保持する。

| 軸 | 接触前 | 最初の接触後 |
|---|---|---|
| r | 座標による位置制限なし | SW1の位置を570mmとして、0〜570mmに制限 |
| z | 座標による位置制限なし | SW2の位置を0mmとして、0〜315mmに制限 |
| θ | 角度制限なし | 角度制限なし |

押されているスイッチへさらに押し込む方向は止め、離れる方向には動かせる。
起動時から押されている場合も、位置情報が揃ったらその軸を登録する。
原点は各軸の最初の接触で一度だけ登録し、再接触では更新しない。
速度・加減速・電流制限は、原点未登録でも有効。

初回出力時のPWM目標は全て500µs、EE回転は1024カウント。
現在のサーボ位置を読まないため、入力とモータ応答が揃うと指定姿勢へ動く。
回転の初期値1024は実機未校正。

通信断・モータ異常では全出力を解除するが、登録済みの原点は消さない。
条件が回復すると自動で操作可能になる。PCプログラムは切断時に終了するため起動し直す。
出力解除中は、重力による機体の落下を防げない。
モータの電源を入れ直した場合はCCTLも再起動する。r・zは再び接点で原点を取得する。

## ソース構成

- `src/main.rs` → `src/pad.rs`：入力・送信。
- `cctl/src/app.cpp` → `motors.cpp` / `ee.cpp`：基板のループ・機体制御。
- `cctl/src/config.hpp`：機体定数。変更後は再ビルド・書込み。
- `cctl/src/motion.hpp` / `input.hpp`：軸の座標・制限計算とUSB入力判定。HALに依存せず直接テストする。
- `serial_svmd/src/app.cpp`、`svmd/src/main.cpp`：CAN目標値の出力。

通信形式と制御の詳細は[DESIGN.md](DESIGN.md)、検証結果は[VERIFICATION.md](VERIFICATION.md)を参照。
`make test`で通信符号化・軸計算・PIDのテストを実行する。

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

VSCodeではSTM32Cube clangd拡張を有効にし、次の設定を使う。
`cctl/`・`serial_svmd/` には設定済み。ルートを開く場合は、
個人用の `.vscode/settings.json` に追加する。

```json
{
    "stm32cube-ide-clangd.enableFormatting": true,
    "[c]": { "editor.defaultFormatter": "STMicroelectronics.stm32cube-ide-clangd" },
    "[cpp]": { "editor.defaultFormatter": "STMicroelectronics.stm32cube-ide-clangd" }
}
```

設定後に「Developer: Reload Window」を実行し、「Format Document」で整形する。
CIと同じ整形結果を確認するには `make format-check` を使う。
