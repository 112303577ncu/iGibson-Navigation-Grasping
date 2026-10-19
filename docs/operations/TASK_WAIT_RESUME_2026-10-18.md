# 等待任務與續行前定位檢查

狀態：腳本及離線測試完成，**未部署Jetson、未接入現行任務流程**。機器保持關機；下一次開機先部署／驗證，再由任務層呼叫。

入口：integration/v23_task_resume_guard.py，Python3.8以上、僅標準函式庫；不載入模型、不匯入ROS、不開Serial、不發底盤／手臂命令。

## 三個階段

| 模式 | 用途 | 不代表什麼 |
|---|---|---|
| begin | 在停穩、travel、控制者釋放、定位健康時建立任務checkpoint，綁定boot／owner／raw-map hash及起始map／odom | 不會開始夾取，也不會暫停AMCL |
| wait | 只觀察等待期間定位／手臂狀態，任務等待時間獨立於定位故障20秒期限 | 觀察時間結束不代表夾取／投放完成，task_completed永遠false |
| resume | 任務完成後，有界等待新鮮且穩定的位置，回報READY_TO_REPLAN | 不開輪子、不重放舊速度、不重做夾取／投放 |

等待時維持既有AMCL服務及0.5秒nomotion更新。健康位置持續更新，正常閒置／夾取超過120秒不會單純因時間經過而失效。故障、被搬運或感測遮擋仍需依實際資料處理。

## 續行條件

- 停車至少0.5秒、四輪零、無底盤client、arm_busy=false、arm_pose=travel；仍在忙時回報WAIT_TASK_READY，不計成夾取故障或自動叫手臂重做。
- grasp／release必須收到與checkpoint相同的operation ID及success回報；並核對resident holding=true／false。unknown、failure、錯誤ID或持物狀態不符，回報WAIT_ACTION_RESULT。operation ID由任務層建立並與組員結果綁定，不能因斷線自行假設成功。
- 即時resident回傳、scan／odom／TF、AMCL covariance、boot／owner／map均須通過；下一階段開始要求可用RAM1024MB。
- 至少5個不同且遞增的odom時間戳，整段採樣的map位置漂移≤3cm、角度≤3°。重複讀同一幀不能累計。
- idle／grasp／release期間預期底盤不移動；超過3cm／3°或AMCL與odom傳播差超過15cm／10°，要求新絕對參考。visual-approach允許該階段實際移動，會依odom傳播位置再核對AMCL。
- boot、owner或map改變必須先完成新定位並建立新任務context，不套用舊checkpoint。

正常等待可以很久；resume開始後預設最多20秒（可設1～60秒），不能恢復就回報RESUME_TIMEOUT。READY_TO_REPLAN只代表本次即時檢查通過，附valid_until=當時+0.4秒；後續導航仍必須使用自己的live guard並從現在位置重算路徑，不能拿舊檔案作長期通行許可。

## 下一次開機先做的基本測試

Jetson專案根目錄，使用 /home/jetson/grasp_venv/bin/python3。首次用idle模式，不需要手臂動作：

```bash
python3 integration/v23_task_resume_guard.py begin --phase idle --checkpoint /tmp/idle_001.json --output /tmp/idle_001.begin.json
python3 integration/v23_task_resume_guard.py wait --checkpoint /tmp/idle_001.json --seconds 120 --output /tmp/idle_001.wait.json
python3 integration/v23_task_resume_guard.py resume --checkpoint /tmp/idle_001.json --seconds 20 --output /tmp/idle_001.resume.json
```

每個任務用新的checkpoint路徑；begin不覆寫既有任務。wait的ok只表示觀察命令執行正常，是否能移動必須再看resume結果。

接入組員夾取流程時，在開始夾取前建立checkpoint，task層保留操作ID：

```bash
python3 integration/v23_task_resume_guard.py begin --phase grasp --operation-id grasp-001 --next-task dropoff --checkpoint /tmp/grasp_001.json --output /tmp/grasp_001.begin.json
# 組員執行原本的夾取／stow，回報實際結果；本腳本不執行或重試它。
python3 integration/v23_task_resume_guard.py resume --checkpoint /tmp/grasp_001.json --completed-operation-id grasp-001 --action-result success --output /tmp/grasp_001.resume.json
```

上例success必須來自實際結果，不能預先寫死。返回碼0為該檢查通過／觀察完成，2為預期等待或需要修復／新參考，1為腳本或輸入錯誤。尚未將此呼叫自動插入v23_delivery_navigation.py或組員手臂流程，待實車通過後再接。

## 新ArUco絕對參考的橋接

可選配置 deploy/ros/aruco_resume_reference.example.json，預設calibration_verified=false，禁止拿模板直接啟用。需完成相機內／外參、marker map登錄及實際校正ID後，才由PC producer寫入最新一份metric map位姿：

```json
{
  "valid": true,
  "source": "aruco",
  "frame": "map",
  "stamp": 0,
  "seq": 0,
  "boot_id": "CURRENT_BOOT",
  "owner_pid": 0,
  "map_hash": "RAW_MAP_HASH",
  "calibration_id": "MEASURED_CAMERA_TO_BASE_ID",
  "registration_id": "REGISTERED_MARKER_MAP_ID",
  "marker_ids": [2],
  "pose_map": [0, 0, 0],
  "covariance_diag": [0, 0, 0]
}
```

此JSON只是欄位示意，不是有效量測。stamp必須是影像擷取時間並換到Jetson時基，不能在傳送時重打時間；covariance必須由有效估測提供，不能為了通過檢查填零。僅有ID而沒有metric位姿會被拒絕。

橋接接受3個不同seq／stamp、來源時間≤1秒、位置／角度一致的新參考；校正ID、登錄ID、marker清單、boot／owner／map也須相符。位置方差上限0.05²m²、角度方差上限5°²。底盤必須停穩、travel且無其他控制者。

resume加上 `--reference-config <已驗證設定>` 可讀取producer；預設只回報REFERENCE_READY_DRY_RUN，不寫seed。加 `--real` 才允許把驗證後的參考寫給既有AMCL supervisor。**此旗標仍不發任何底盤／手臂命令。** 同一次resume最多提交一次seed，之後等監督器accepted_reference_stamp回覆、確認新的AMCL位置與來源相符，再累計5個穩定新幀；逾時不反覆重送。

已知車子被搬動時可加 `--relocated`，強制新參考。未被回報、且輪式odom沒有改變的搬運，在對稱場地可能無法只靠AMCL／odom發現；這需要實際的ArUco絕對位置producer，本輪沒有完成相機校正或冒充已接上producer。

## 監督器配套變更

- 明確外部新參考在底盤停穩、手臂idle時才接受；忙碌期間延後，不打斷健康定位。
- 接受後真正重建自己的AMCL程序，不只把新參考拿來與舊位置比較；仍受既有3次／600秒預算限制。
- 健康JSON增加accepted_reference_stamp。新AMCL的接收時間及訊息來源時間都須晚於實際新程序啟動，排除延遲送達的舊publication。

下次同步需包含新task guard、g2_localization_service.py、g2_localization_policy.py及先前AMCL watchdog unit。先依AMCL_WATCHDOG_2026-10-18.md部署看門狗，再驗證idle120秒→resume；接著做夾取等待、故障後新參考、隨機點重新規劃。ArUco配置未校正前，只驗證正常等待／健康續行和正確拒絕無效參考。

## 離線證據

新task guard39項測試通過；含既有監督器的相關261項通過。另controller138、servo37、FloorGuard641、UI50（skip1）、FSM通過。實際supervisor迴圈的虛擬測試包含手臂忙碌超過120秒仍維持健康位置、新參考等待手臂idle後才套用、以及舊publication不得假裝新AMCL已就緒。

測試輸出在 docs/calibration/task_resume_2026-10-18/。本版沒有Jetson執行、實車故障注入、ArUco校正／producer或任務自動串接的驗收結果。
