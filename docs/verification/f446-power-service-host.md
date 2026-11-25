# F303電源の論理観測

仕様：PROTO-PWR-001、BOARD-F446-001、BEHAV-001。PowerBoardServiceは外部で正しく復号・受信元確認済みの状態と時刻を受理し、500ms未満のONのみreadyとする。FAULTは次の周期消費まで保持し、既存ExternalFaultへ通知する。OFF/PRECHARGE/途絶はready=false。未取得では駆動許可なし。Applicationに論理観測を追加したが、実CAN ID・DLC・データ配置は未確定のため受信配送は未接続。電源操作なし。試験は100ms報告、期限境界、異常ラッチ、未来・重複・逆行・未知状態を検証する。

CTest 26/26 PASS。実CAN・ARMクロスビルド未検証。
