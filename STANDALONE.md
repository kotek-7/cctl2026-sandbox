# 現在の開発方針

元: `/home/kotek/dev/catchrobo2026`、基点 `7eefba9620cebc112ca7aadb2825fcf1e2b407ce`。
このディレクトリは独立したGitクローン。名称にstandaloneが残るが、現在の方式はPC経由。

`DualSense → USB → PC → USB CDC → 既存CCTL → CAN → 既存機体`

CCTLのUSBホスト化・新基板への変更は行わない。
ユーザー確認済みの範囲はr/θ/z手動操作、EE回転/畳み/把持、原点出し。
GUI、AI/API、自動取得、ボーナス操作は外す。
アーキテクチャの単純さを最優先し、ライブラリに直接乗る単一ループへ整理する。
現時点では移行作業中。実機操作は行っていない。
