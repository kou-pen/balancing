# 復号済み操作入力のC++17境界

仕様：BEHAV-001、PROTO-REMOTE-001、PROTO-PS5-001。RemoteControlServiceが既存C11の操作入力を単一所有する。ARMINGは目標ゼロだが操作元変更は認めない。C11との同一入力比較、通信断からの減速、ニュートラル復帰、切替・不正入力を検証。Applicationでは状態確定後に周期コピーを作るだけで、未検証の既定値は無効。実HID/CANの復号、操作イベント、位置基準、駆動は未接続。

CTest 27/27 PASS。実HID/CANとARMクロスビルドは未検証。
