# CAN2自作通信の受信分類ホスト検証

対象：PROTO-PWR-001、PROTO-REMOTE-001、PROTO-C620-001、BOARD-F446-001、T-PROTO-11。2026-11-25。

## 実装

| ファイル | 役割 |
|---|---|
| CanRouter.hpp | 自作CAN2の6分類をCanRxTargetへ追加 |
| CanRouter.cpp | 固定許可リスト・DLC・IDE・受信方向の照合 |
| MotorCanLink.cpp | 既存RS06および標準0x201/202だけを機器サービスへ配送 |

| 受信分類 | ID |
|---|---|
| PowerStatus | 0x0840BC03 |
| PowerResult | 0x08408404 |
| EmergencyStop | 0x0004BC01 / 0x0004FC01 |
| RemoteMotion | 0x0880C401 |
| RemoteRequest | 0x0880C402 / 0x0480C403 / 0x0880C404 / 0x0880C405 |
| DiagnosticRequest | 0x0CC0C480 / 0x0CC0C481 / 0x0D00C401 |

受信フレームと採取時刻を既存CanEventSinkに渡すだけで、ペイロードの意味・操作権限・状態遷移は扱わない。F446自身の発信や別機器間通信はUnmatched。既知車輪の短いDLCは従来どおりMotorServiceに通知し、連続性の欠落を検出する。新しい分類は車輪に配送しない。

## 検証

CTest 33/33 PASS。通常・UBSan全件PASS（job c85cb2bc7d6f4110、exit 0）。CAN静的監査19/19、CubeIDE設定22/22、validate_specs、sync_cubeide_cpp、Doxygen生成、diff check PASS。Doxygenは既知の日本語翻訳警告のみ。

35個の割当IDの全方向、12個の受信対象、DLC0、全DLC変異、IDE、予約領域、範囲外ID、別バス、時刻保持を確認。ポート模擬FIFOから自作12 IDを流して車輪観測の連番・時刻・連続性が変化しないことを確認。既存RS06/C620の回帰試験を維持。

## 保留

実機HAL受信フィルタは従来の標準0x201/202のみ。実行時の上位コーデックへの配送、受信バッファ、状態機械、配信・送信管理、Pi/F303との実通信は別工程。ARMクロスビルド、実機駆動未検証。T-PROTO-11は未完了。
