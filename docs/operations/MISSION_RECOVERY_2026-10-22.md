# 任務故障恢復與卡住原因

本版僅離線整合與故障注入，未連線、部署或啟動實車。
同日最新修正見 [Claude審查優化](CLAUDE_REVIEW_OPTIMIZATION_2026-10-22.md)：批准35cm mask、恢復雷射煞停、一般脫困只前進，定位跳位只跨中斷比較。
PC 流程固定使用 koala `loop-opt-2026-10-21` 的
[`e8766f03d0de3e48419a488f330b62e3412afab3`](https://github.com/koala915/iGibson-Navigation-Grasping/commit/e8766f03d0de3e48419a488f330b62e3412afab3)。
定位監督與原生巡航使用本 repo 的配套版本。不是只更新 PC 入口就能取得 Jetson 看門狗修正。

## 已修正的失效路徑

| 原因 | 現在的處理 | 限制 |
|---|---|---|
| 一筆 AMCL／健康回報過期就讓放置後退出失敗 | 先停車；等待同一 owner／boot／map 的新鮮定位，五筆不同時間戳且位置穩定後繼續 | 任務恢復最多30秒；監督器已等待的時間算在同一預算內，錯誤種類改變不歸零 |
| 後退已完成一段，但定位延遲後重算整段命令 | 以實際 odom 保留後退進度；AMCL只判斷目前地圖位置 | 不重送上次相對位移、不重播夾取／放置 |
| PC 的8秒定位等待先於監督器恢復期限退出 | 原巡航停車並結束後，任務層經恢復閘重新派發巡航 | 每次巡航呼叫最多重派一次；不是重跑整個任務 |
| ROS no-motion ServiceProxy 卡住，但監督器主迴圈仍送 heartbeat | 獨立檢查請求執行緒，超過2秒使定位監督器非零退出，由 systemd 重建該執行緒 | 只管理自己的 AMCL roslaunch 群組，不重啟 grasp-service／Serial owner |
| 監督器主迴圈本身卡死 | systemd WatchdogSec=5、TimeoutStopSec=5、RestartSec=1 | 名義最壞11秒，低於15秒last-good有效期；啟動耗時仍可能導致過期，不能保證免重定位 |
| 視覺許可橋連線結束 | 確認原巡航停止及新定位後，關閉舊橋、建立新橋，重新進原本影像放行流程 | 一次重試；不使用舊permit或直接放行底盤 |
| 無GUI的OpenCV在正常收尾時拋錯 | 只包住 destroyAllWindows 的 cv2.error | 偵測／控制的其他異常仍向上回報 |
| 分散覆蓋組員檔案造成版本混用 | 新目錄匯出固定完整commit、套用兩項可重現修正，入口核對來源SHA256 | 來源不符合即拒絕；不覆寫原工作樹手臂草稿 |

定位恢復還要通過：車輪零回授、手臂travel且閒置、空爪、沒有外部TCP client，
odom／定位新鮮，AMCL位置相對odom的跳變≤15cm／10°。
新的健康位置必須連續五筆停穩在3cm／3°內；恢復後再由原巡航重規劃。
重複舊時間戳不能湊足五筆。no-motion逾時與任務等待使用monotonic時間。

放置後每次直退最多10cm，完成條件是中心及6／7cm取樣圈進入巡航mask，
再取得三筆新的停穩位置；不是「退10cm且看得到定位就下一圈」。
詳見 [放置後退出](POST_PLACE_PATROL_REENTRY_2026-10-22.md)。

## 仍可能中止任務的項目

| 項目 | 判斷與處理 |
|---|---|
| 無可信定位：seed過期、搬車、boot／owner／map改變、定位跳變 | 停車並要求新獨立參考；目前沒有在任務中自動掃ArUco並重建參考的producer。八ID登錄不等於任何姿態都可自動重定位 |
| LiDAR、控制板／Serial、相機或模型程序持續失效 | 有逾時與停車保護，沒有第二套實體感測／控制硬體可替代。未知結果的手臂動作不能自動重試 |
| AMCL平面協方差不符35cm幾何預算 | 看門狗可能ready但任務仍等待；退出／空場幾何模式要求每軸≤(0.04/3)^2。這是品質不足而非腳本死鎖，30秒後明確中止，不能把ready強制設true |
| PPO接近途中越過巡航禁航區 | 本版有巡航邊界恢復與放置後退出；PPO每段局部速度尚未全面接同一全域mask／可退路徑閘，仍須整合接近段的全域幾何檢查 |
| 投放姿態直退60cm內無可轉向區，或實體退路不足 | 明確停車；需改投放接近角度或保存接近路徑並驗證倒退，不盲轉、不反覆固定倒車 |
| 影像過期、模型推論慢、遠距／校正外目標、支架遮擋 | 保留影像許可、停穩確認、homography範圍及誤認防護；須用 timings.jsonl／影像記錄量測模型耗時與覆蓋。重啟AMCL不能修好這類問題 |
| 自回波／牆面使前向閘過早煞停 | 巡航保留雷射煞停；僅經量測批准的車體回波遮罩可排除自回波，不用地圖檢查取代偵測人員的雷射閘 |

25cm車體半徑是操作者提供的上界；10cm綜合誤差預算、地圖牆線與投放盒包絡仍需實測。
正式repo場景採批准35cm地圖；糖果擺放圖仍是離線提案，不能視為任意夾取點已完成路徑預約。
夾取／投放全循環、Jetson Python2執行、systemd實際SIGSTOP／故障注入尚未驗收。
離線通過不能宣稱任一模組實體受損仍必定完成任務。

## PC準備與日後部署

只下載並準備PC來源，不會SSH或控制機器：

```powershell
python integration/g2_prepare_koala.py --output .runtime/koala-e8766f0
```

需要Git與網路；也可提供含固定commit的 `--git-cache 路徑`，該Git目錄只讀。
輸出目錄必須全新；來源不符或patch上下文不同即失敗。`.runtime/`不進Git。

入口 `integration/v23_mission_entry.py` 預設指向此目錄，亦接受 `--koala-root`。
預設預檢會唯讀連線Jetson，實車另須既有real參數；本次皆未執行。

Jetson日後須配套同步 `deploy/ros/g2_localization_service.py`、`deploy/ros/g2_map_patrol.py`、
`integration/g2_localization_policy.py`、`integration/g2_boundary_recovery.py`、場景JSON及systemd單元，
其餘既有依賴亦需在位。先編譯、確認單一Serial owner與停車，才依
[看門狗部署文件](AMCL_WATCHDOG_2026-10-18.md)安裝；本版沒有執行這些操作。

診斷輸出：`mission/patrol_recovery.json`、`mission/post_place_reentry.json`、
`entry_report.json`、Jetson巡航report與 `/dev/shm/g2_localization_health.json`。
錯誤應依reason_code、next_action、協方差及資料年齡區分，不用「AMCL沒更新」概括所有停車。

## 離線驗證

本repo定位、導航、8-ID、任務恢復及幾何17套測試316 passed。
固定koala runtime的 flow_contracts／mission_loop／mission_runtime：Python3.11，
138 passed、56 subtests passed；來源測試使用ast.unparse，不能在Python3.8執行該測項。
本repo執行時的Python3.8相關套件測試則通過。
AGENTS指定回歸：controller138、servo37、FloorGuard641 checks通過；UI50 tests、skip1；FSM SELFTEST PASSED。
四份Jetson來源通過Python2語法及ASCII檢查，實際Python2／systemd行為仍待Jetson驗證。
