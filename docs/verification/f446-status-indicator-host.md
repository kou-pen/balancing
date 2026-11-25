# LED/Buzzer論理表示と非ブロッキング道具

仕様：BOARD-F446-001、BEHAV-001。Ledは確認済み出力関数・極性だけでon/off/toggleを行う。RgbLedは3色すべての設定を要求する。Buzzerは単調時刻差で単発鳴動、期限・逆行時停止。StatusIndicatorは状態とFAULT記録を受け、FAULT優先、状態変化でだけ単発鳴動し、点滅は時刻差から計算する。時計逆行で論理表示をラッチ無効化。Applicationには論理表示コピーのみ接続し、既定は未検証。物理GPIO、EMS_OUT_N、CAN送信、実機の色・点滅・鳴動時間は変更しない。出力関数の実行時間・実基板極性は未検証。

4機能統合：CTest 29/29 PASS、通常ホスト/UBSan全回帰終了0、CubeIDE設定試験22/22、CAN契約14/14、仕様監査、Doxygen生成PASS（既知の翻訳警告）。ARMクロスビルド・実機未検証。
