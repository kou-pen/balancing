# Raspberry Pi操作通信コーデックのホスト検証

対象：PROTO-REMOTE-001、T-PROTO-06/11。2026-11-25。

## 実装範囲

| ファイル | 役割 |
|---|---|
| RemoteControlCodec.hpp | 運動指令、操作要求、操作結果のデータ型と公開API |
| RemoteControlCodec.cpp | MOTION_COMMAND、ARM、DISARM、CLEAR_FAULT、SELECT_SOURCE、REMOTE_RESULTの符号化・復号 |

既存ProtocolId/ProtocolPayloadを使用し、固定ID・IDE・DLC・方向、有限float32、列挙値、予約領域を検査する。SELECT_SOURCEだけ操作元指定を必須とし、それ以外の要求では指定を拒否する。要求番号は応答照合用であり、コーデックで重複や鮮度を判断しない。Acceptedと実際のロボット状態を区別する。

## 検証

- CTest 32/32 PASS。
- 通常・UBSan全件PASS（job 5c199f9e3cd64825、exit 0）。CAN静的監査19/19、CubeIDE設定22/22。
- validate_specs、CubeIDEメタデータ監査、Doxygen生成、diff check PASS。Doxygenは既知の日本語翻訳警告のみ。
- 6個の固定IDと独立した既知バイト列、全正常列挙、要求番号の端点、操作元指定の有無を確認。
- NaN/Inf、予約バイト、IDE、DLC、未登録ID、誤方向、別サービス、DJI予約領域を拒否。
- 運動指令の数値は人工のテスト値であり、実機の速度上限を定義しない。

## 非変更・保留

CAN2配送、RemoteControlService・RobotSequenceとの結合、操作権限、実際の駆動、タイムアウトの設定、RS06/C620通信は変更しない。ARMクロスビルドと実機CANは未検証。T-PROTO-06/11は未完了のままとする。
