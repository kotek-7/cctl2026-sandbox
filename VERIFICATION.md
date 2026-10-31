# 検証記録

2026-10-31。PC入力送信と基板直接制御へ置換した版。

以下が成功:

- `make build`：PC RustのReleaseビルド。
- `make build-cctl` / `make build-serial-svmd` / `make build-svmd`：3基板のビルド。
- `make test`：通信符号化、多回転角、PID、時刻差の55テスト・172アサーション。
- `cargo clippy --locked --all-targets -- -D warnings`、`cargo fmt --check`。
- Release実行ファイルの`--help`、`git diff --check`。
- 本番のUSBパーサを一時的なホストプログラムで実行し、正常な7項目、符号、CR終端、項目不足/過剰、値の範囲外、区切り欠落、不正文字を確認。

CCTLはFlash 29KB程度、SerialSVMDは8KB程度、SVMDは42KB程度。
PC本体は2ファイル。基板の生成コード・低水準ドライバは別に保持する。
旧仮想シリアル試験・模擬基板・HALモックは削除し、今回の検証には使用していない。

実機への書込み、DualSenseの実入力、3軸の方向・端点・停止距離、EE初期姿勢は未確認。
基板を含め通信仕様を変更したため、旧版と混在させない。
実機確認では、原点を手動取得し、各軸を低速で短く動かして方向・端点を確認する。
続いてCreateでのEE有効化、入力を離した際の位置保持、通信を止めた際の出力解除を確認する。
この実機確認はまだ実施していない。

## 2026-11-01：原点登録後の常時操作

Options/PSによる開始・保持・再開を削除した。
Createで原点登録すると3軸とEEを有効にし、以後は常時入力を適用する。
通信断・モータ異常では原点登録も無効にし、復帰にはCreateを押し直す。

`make build build-cctl`、`make test`（既存の55テスト）、
`cargo clippy --locked --all-targets -- -D warnings`、`make format-check`、
`git diff --check`が成功。サーボ基板FWは今回変更していない。
既存テストは通信符号化・PID等を対象とし、新しい操作手順の実機検証ではない。
今回も実機への書込み・操作は未実施。
