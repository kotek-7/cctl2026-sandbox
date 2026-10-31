# 非実機テスト

ルートで `make test` を実行する。

- `rust/control.rs`: Rustの実行経路を直接検証。仮想シリアル接続による設定・原点・運転・保持・異常停止と、機体計算を確認する。
- `rust/simulator.rs`: テスト時だけ使う模擬基板。実行プログラムには含めない。
- CMake/CTest: CCTL・SerialSVMD・SVMDのロジックとモータ/STS通信ドライバ。HALの代替を使用。

実機にはアクセスしない。
