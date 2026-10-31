# 検証記録

2026-10-31。PC側をRustの3モジュール（main / robot / board）へ置き換えた後の記録。
Python版とその依存設定は削除済み。

`make test` 成功:

- Rust 14テスト。起動時の出力無効化、原点/中立条件、接点、PS保持停止、RUN応答待ちとその中断、通信断、モータ異常、入力処理遅延、終了時の停止送信。
- r/zの符号と制動、EE回転範囲、PWM端点への到達、不正設定の拒否。
- serialportの仮想端末ペアに模擬基板を接続し、実際の設定照合・操縦ループ・終了処理を検証。模擬基板はtests/rust内のみ。
- CTestの5テスト: CCTL・SerialSVMD・SVMDのロジック、モータ/STS通信ドライバ。

`cargo run --locked -- --help`、`cargo clippy --locked --all-targets -- -D warnings`、
`cargo fmt --check`、`git diff --check` も成功。
Rust依存はCargo.lockに固定。

基板FWのソースは今回変更していない。
実DualSense、USB実接続、実機での移動・停止距離・サーボ初期姿勢は未確認。
基板への書込みも未実施。仮想端末テストは実機の確認を代替しない。
