# 検証記録

2026-10-31、`/home/kotek/dev/cctl2026-sandbox` へ移設・簡素化後のLinux環境。

- `cargo test --locked --manifest-path host/Cargo.toml`: 27ユニットテストと3端末テストが成功。
- 原点、接点、符号付き座標換算、制動距離、加速、停止目標の保持、EE回転補償を検証。
- 運転開始、保持・再開、中立条件、切断、緊停、基板異常、応答待ち、出力無効化を検証。
- `ctest --test-dir tests/build --output-on-failure`: 必要な3基板とモータ/STS通信の5テストが成功。
- `cargo clippy --manifest-path host/Cargo.toml --locked --all-targets -- -D warnings`: 成功。
- `cargo fmt --manifest-path host/Cargo.toml --check`: 成功。
- `make build-cctl`: Release RAM 15,528 B / FLASH 59,924 B。
- `make build-serial-svmd`: Release RAM 3,800 B / FLASH 21,744 B。
- `make build-svmd`: RAM 4,056 B / FLASH 43,316 B。
- `git diff --check`: 成功。

旧host構成のテストは削除済み。以前の287件という数字は現在の構成には使わない。
現在のPC側は14ソースファイルで、旧構成をコンパイルする機能フラグはない。

実DualSense・USBシリアル実接続・モータ方向・サーボ初期姿勢・接点極性・停止距離は実機未確認。
ファームウェア書込みも未実施。模擬基板は衝突・重力・基板内のタイミングを再現しない。
原点は手動位置合わせ後の確定方式。
