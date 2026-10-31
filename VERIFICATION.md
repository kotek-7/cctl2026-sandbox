# 検証記録

2026-10-31、Linux開発環境。実機は操作していない。

- `./operate test`: Rustユニット273件、最小実行ファイルの端末テスト3件、旧経路の回帰テスト11件が成功。
- 上記のうち新しい制御のテストは9件。原点前の開始拒否、PWM設定後の有効化、パッド切断、緊停、中立復帰、モータ異常、制御遅延、継続運転、周辺出力無効、接点条件、RUN応答待ちを確認。
- CTest: CCTL・SerialSVMD・SVMD・DCMD・モータ通信・STS通信の6実行ファイルが成功。
- CCTL Release: RAM 15,528 B / FLASH 59,924 B。
- SerialSVMD Release: RAM 3,800 B / FLASH 21,744 B。
- SVMD: RAM 4,056 B / FLASH 43,316 B。
- `cargo check --locked --manifest-path host/Cargo.toml --bin simple` 成功。
- `git diff --check` 成功。

模擬通信は物理機構を再現しない。実DualSenseの入力、USBシリアル実接続、モータ方向、
サーボ初期姿勢、接点極性、停止距離は実機未確認。ファームウェア書込みも未実施。
原点は自動走行ではなく手動位置合わせ後の確定方式。
