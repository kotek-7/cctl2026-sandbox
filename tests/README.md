# 非実機テスト

ルートで `make test` を実行する。

- `python/test_operate.py`: 実行コードoperate.pyを直接検証。パッド入力、原点、運転/停止、制限、終了処理、pyserialの仮想端末通信。
- CMake/CTest: CCTL・SerialSVMD・SVMDのロジックとモータ/STS通信ドライバ。HALの代替を使用。

実機にはアクセスしない。
