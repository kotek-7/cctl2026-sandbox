# DualSenseで手動操縦

DualSenseとCCTLをそれぞれLinux PCへUSB接続して、r・θ・zとEEを操作する。
PCは入力を20msごとに送り、CCTLが機体を制御する。CCTLの状態はLCDとPC端末へ表示する。

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

hostとCCTLは同じ版を使用する。基板への書込みは別途行う。実機動作は未確認。

DualSenseを接続してから、シリアルポートへのアクセス権があるユーザーでPCプログラムを起動する。
STS3215はあらかじめID=1、1Mbps、位置モードに設定しておく。

## 操作

有効なPC入力が届いたら操作できる。モータ応答は待たず、届いたときに実測値を更新する。

| 入力 | 動作 |
|---|---|
| 左スティック上下・左右 | r・θ |
| R2・L2 | zの正・負方向 |
| 右スティック左右 | EEを回す。θを回してもEEの回転目標は変わらない |
| 十字キー上下 | 畳みを動かす |
| 十字キー左・右 | 把持を開く・閉じる |
| L1 | r・θ・z・EE回転の入力を30%にする |
| Ctrl+C / パッド切断 | PC終了。250msの入力途絶で速度入力を0にし、EEの姿勢指令を続ける |

r・θ・zは速度制御で動かす。入力を離すと速度0を目指して減速する。位置保持は行わない。

### 原点と位置制限

| 軸 | 接触前 | 最初の接触後 |
|---|---|---|
| r | 座標による位置制限なし | SW1の位置を570mmとして、0〜570mmに制限 |
| z | 座標による位置制限なし | SW2の位置を0mmとして、0〜315mmに制限 |
| θ | 角度制限なし | 角度制限なし |

押されているスイッチへさらに押し込む方向は止め、離れる方向には動かせる。
r・zの原点は最初の接触で一度だけ登録する。起動時からスイッチが押されている場合は、
その軸の位置情報が届いた時点で登録する。再接触しても原点は変わらない。
速度・加減速・電流の上限は、スイッチに当たる前から有効。rの加減速はモータ内部、θ・zの加減速はCCTLで行う。

### EEの初期姿勢

最初に出力が有効になると、4本のPWMサーボに500µs、EE回転サーボに1024カウントを指令する。
サーボの現在位置は読まないので、起動時の姿勢をそのまま保つとは限らない。
回転の初期値1024が実機でどの向きになるかは未確認。

### 通信断・モータ異常からの再開

PC入力が250msを超えて途絶えたら操作入力を0にする。軸の速度制御とEEの姿勢指令は続ける。
登録済みの原点は保持し、入力が戻れば操作を再開する。切断で終了したPCプログラムは起動し直す。

モータ返信の途絶は検知するが、出力を止めない。温度による停止は行わない。
r自身の異常報告には、rへ停止＋異常リセット要求を送る。異常が消えれば操作を再開する。
他軸とEEの出力は継続する。
モータの電源を入れ直した場合はCCTLも再起動する。r・zは再び接点で原点を取得する。

### 状態表示

LCDにPC入力の状態、rの異常コード、各軸の返信状態を表示する。
PC端末にも同じ情報を返し、変化した行だけ表示する。
`NO`は返信が未受信または200msを超えて途絶えた状態、`R_FAULT=0x00`はrの異常報告なし。
表示対象と形式は[DESIGN.md](DESIGN.md#lcdとpcへの状態表示)を参照。

## ソース構成

- `src/main.rs`：20ms周期の入力送信とCCTLの状態表示。`src/pad.rs`：パッド入力の読取り。
- `cctl/src/app.cpp`：基板のメインループ。`motors.cpp` / `ee.cpp`：軸とEEの制御。
- `cctl/src/status.cpp`：CCTLのLCD表示とUSBへの状態返信。
- `cctl/src/config.hpp`：機体定数。変更後は再ビルド・書込み。
- `cctl/src/motion.hpp` / `input.hpp`：モータ角度と機体位置の換算・移動範囲の制限とUSB入力の検査。基板なしでテストできる。
- `serial_svmd/src/app.cpp`、`svmd/src/main.cpp`：CAN目標値の出力。

通信形式と制御の詳細は[DESIGN.md](DESIGN.md)、検証結果は[VERIFICATION.md](VERIFICATION.md)を参照。
`make test`で通信符号化・軸計算・P制御のテストを実行する。

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
