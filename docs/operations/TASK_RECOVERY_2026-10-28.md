# 定位暫停與原任務續行：離線版

本輪沒有連線車輛、部署、馬達或手臂動作。保留 f5447af 的 CI 修復；不改地圖、禁航距離、權重或實車校正。

## 入口與已確認缺口

實際入口是 `grasp/v23/demo.py mission` → `integration/v23_mission_entry.py`，
再載入固定 koala `e8766f03d0de3e48419a488f330b62e3412afab3` 的獨立 runtime。
本 repo 的歷史 `integration/mission_loop.py` 不在這條路徑，沒有修改它來冒充完成整合。

- 原生 `back_straight` 再讀一次定位時，一筆過期資料便會結束；外層等待無法攔住這次讀取。
- 接近端不認識定位暫停，無進展與父程序 approach timeout 會繼續計時，舊目標與延遲動作仍在記憶體。
- 重派 patrol-cycle 會從新位置選起點；直接重規劃至最終目標也可能跳過未完成的中間子路段。
- 原生 scan／TF 中斷可能早於 AMCL 健康旗標；只看 AMCL ready 會漏掉觀察補驗。
- 一筆 native pulse 故障若在停穩前已恢復，原本不一定建立恢復事件；現在先保存補看要求，再處理停穩。
- 背景串流停住時，舊 READY 不能繼續作有效證據；推論錯誤／無效投影也不能當作「已看過、沒有目標」。
- H264 接收端只有 PC 解碼到達時間，沒有相機來源擷取時間／序號。操作者確認 0.30 秒上界尚未量測，不能據此批准實車自動續行。

## 狀態與預算

`RUNNING → WAIT_LOCALIZATION → WAIT_FRESH_OBSERVATION → REVALIDATE_PATH → 原階段`

缺少來源時效量測時改進入 `WAIT_OBSERVATION_PROVENANCE`，在同一預算耗盡後明確回報
`OBSERVATION_PROVENANCE_REQUIRED`。視野不足則回報 `OBSERVATION_GAP_UNRESOLVED`，不盲退／盲轉。

既有總等待上限仍是30秒，包含監督器已等時間；重入、scan換成TF故障、恢復一下又失效都沿用已用額度。
等待採 monotonic；ROS Python2 用 CLOCK_MONOTONIC。一次事件必須等原目標路段及觀察均完成才能結清，
不是收到一筆 ready 就重新給30秒。父程序有效接近時間最多扣除這30秒，安全 watchdog 不暫停。
恢復讀取的 SSH／健康 RPC 各有較短期限，受剩餘預算限制；正常控制週期只讀本地快取，不逐幀建 SSH。

暫時 scan／odom／TF／AMCL 過期可等待；boot／owner／map改變、定位對odom跳變超過原15cm／10度、
手臂未知結果、物理路徑不成立仍停止。不偽造 ready、不延長舊定位或影像有效期，不重啟 Serial owner、模型或任務。

## 各階段接回方式

| 階段 | 保存與續行 |
|---|---|
| 巡航 | 原進程保留目標與未完成子路段；修復到該子目標的路徑後才續往後面的點。導航與觀察完成旗標分開。 |
| 因目標而中斷巡航 | 保存 remaining_path／remaining_route；若恢復後目標不再出現，以專用 reobserve 結果結束接近，不記成夾取失敗／永久忽略，再送回原未完成目標。最多沿用原12個 goals 限制，超出即明確拒絕。 |
| 接近 | 同一模型程序停車等待；清空目標、動作延遲佇列、控制器暫存及進展視窗。五筆新穩定停車定位後，重新觀察，再由原生相機／scan／路徑閘決定動作。 |
| 持物轉運 | holding 維持已確認狀態；原生導航重驗剩餘路徑。新增只讀觀察執行緒，用原有 vision permit 暫停／確認，不持有 TCP 或 Serial。 |
| 放下後退出 | 已確認 release 不再發送；內層定位讀取也經恢復閘。起始odom固定，用實際進度扣剩餘退距；滑行已到位即驗證，不補退整段。 |
| 手臂已發動 | 原生安全收尾及結果確認保留；結果 unknown 直接停止，不能用稍後的 holding 猜測來重播。 |

後退每次仍≤10cm、總額度≤60cm；續退前查整條剩餘退路，側移4cm／偏航5度或原有超距條件仍拒絕。
放置後仍需轉向圈合法及三筆新停穩資料，不是退10cm就下一圈。

## 觀察進度與來源證據

`task_checkpoint.json` 保存階段、目標、未完成巡航路段、起始odom／移動目標／實際進度、holding、
最後確認的手臂結果、影像序號／時間／對應位姿、boot／owner／map及等待預算。
原生巡航報告保存原路線及尚未走完的子目標；worker 的 checkpoint 會在清理暫存目錄前歸檔到任務檢查點。
階段交接保留 worker 最後實測基準；缺少中斷前基準不能略過定位跳變檢查。
成功 exit code 必須與確認持物狀態一致；程序例外／放下回覆遺失記成未知，不因稍後仍讀得到 holding 就重試。

補看根據已校正地面視野、既有手臂相機遮蔽區、影像延遲、停止後位姿與誤差計算未覆蓋多邊形。
走過不會自動將它清除；重複序號、舊時間戳、沒有位姿對應的畫面不能作新恢復證據。
目前只自動接受停止位置的新視野能補足的情況；無法補足就保留原因與檢查點，不產生固定回退距離。
原生 TF／scan 故障另有 recovery generation／ack，確認本次新觀察後才重新放行 permit。

`[RECOVERY]` JSON 狀態會出現在程序／UI操作台的主控台日誌；獨立看門狗及 Ctrl-C／原有停止路徑保持有效。
本輪沒有新增 GUI 狀態面板，也沒有在失效時自動開啟影像時效放行旗標。

### 尚未批准的影像上界

未設定 `G2_RECOVERY_FRAME_BOUND_RECORD` 時，來源時效閘預設關閉；這是操作者要求保留的驗收項。
日後量測紀錄須包含 verified、≤0.30秒上界、sender／receiver SHA256與原始量測檔的SHA256，
由 `verified_frame_bound()` 驗證。範例在 `docs/planning/frame_bound_record_2026-10-28.example.json`，預設 verified=false。
量測若超過既有上界，不能把範例數字調大來放行，須改善排隊或提供真正來源時間戳協定後再審查。

## 準備與配套

```powershell
python integration/g2_prepare_koala.py --git-cache . --output .runtime/koala-e8766f0-recovery
```

schema3 綁定固定commit、每個來源SHA256、補丁配方SHA256，以及與PC相同的幾何／恢復／協調器。
schema2 或舊配方拒絕，不覆寫舊目錄。驗證工具不先載入本 repo 的歷史 integration package。
真正任務開始前會只讀核對 Jetson `g2_map_patrol.py` 與 `g2_task_recovery.py` 的配套雜湊；
舊版不懂 pause／ack 時拒絕開始。這些部署動作本輪未執行。

## 最小實車驗收順序（未執行）

1. 保持車輪／手臂停止，核對兩端版本與唯一硬體 owner，量測來源擷取到解碼的延遲及最壞排隊，保留原始紀錄。
2. 先在原地注入一筆定位過期：確認五筆新定位、三筆新觀察與來源證據均通過才恢復；人工停止始終可用。
3. 在已驗證的單一直線退路測單筆／半途失效及滑行到位，尺量實際退距與剩餘量，確認沒有重退。
4. 僅測 A→B：於中途暫停，將 sugarbox 移入／移出視野，確認重新觀察且先完成原未完子路段／B，才往C。
5. 分別測持物轉運、已放下後退出；核對 release 次數不增加。最後測反覆抖動及持續失效在同一預算內停止。

單準備點／小擺放區／固定方向提案見 `docs/planning/recovery_minimal_cell_2026-10-28.json`。
提案 enabled=false、hardware_validated=false，沒有任何 runtime 讀它或改變擺放限制；
需量測持物包絡、鏡頭遮蔽、投放箱位姿／尺寸／箱緣、停止誤差及退出漂移後，才能驗收再擴大。

## 2026-10-28 離線驗證

修改檔案分工：

| 檔案 | 用途 |
|---|---|
| `integration/g2_task_recovery.py`（新增） | 共用異常、預算、檢查點、觀察與移動進度契約 |
| `integration/g2_task_coordinator.py`（新增） | 真正任務各階段交接、相機補看、持物轉運及未完路線保存 |
| `integration/g2_recovery_patches.py`（新增） | 對固定 runtime 的接近器、任務、permit及計時精確補丁 |
| `integration/g2_mission_recovery.py` | 共用預算與不可重派原則 |
| `integration/g2_patrol_reentry.py` | 原生內層後退攔截及退出階段基準保存 |
| `integration/g2_placement_supervisor.py` | 定位背景資料與持物轉運觀察接入 |
| `deploy/ros/g2_map_patrol.py` | 同進程暫停、停止樣本、觀察ack、原子路段重驗 |
| `integration/g2_prepare_koala.py`、`integration/v23_mission_entry.py` | schema3、配方／來源雜湊與真正入口配套 |
| `tests/test_g2_task_recovery.py`（新增） | 故障注入 |
| `tests/test_g2_prepare_koala.py`、`tests/test_v23_mission_entry.py`、`tests/test_g2_patrol_policy.py`、`.github/workflows/tests.yml` | 準備、部署契約及回歸接入 |
| `CLAUDE.md`、本文件、`docs/planning/frame_bound_record_2026-10-28.example.json`、`docs/planning/recovery_minimal_cell_2026-10-28.json` | 規格、未批准量測範例及未啟用小區域提案 |

- 恢復／AMCL／邊界／準備工具／入口／巡航政策聯合回歸：243 passed，包含單筆內層讀取失效、半途後退、滑行到位、原子路段保留、新影像／舊影像、無進展暫停、同預算抖動、上下文變更、未知手臂結果等注入。
- 固定 runtime 原生相容流程：330 passed、320 subtests passed，保留原有檢查並核對新的 permit 欄位／reobserve 結果。
- 完整 V23 finalist resident stack 全部指令成功：170／37／641／62／89／59（Windows chassis）／26（odom）／28／13，加96 passed、1 skipped；Linux health socket 項另以 WSL 執行原測試函式通過。
- 專案要求的 v21 controller／servo／floor guard、FSM，以及 workflow 其餘 Python 套件／導航／視覺回歸全部成功；systemd helper 於 WSL 的臨時 LF 副本通過。
- UI：50 tests、1 skipped（外部路線資料不在此 checkout）。沒有移除該斷言或偽造路線。
- 最終預設 runtime `.runtime/koala-e8766f0-recovery`：schema3／所有雜湊／真實入口與 adapter 安裝／compileall 通過；smoke 封鎖 subprocess.run／Popen，沒有車輛連線。

上述結果來自本機離線驗證；GitHub CI 結果以對應 commit 的 Actions 紀錄為準。Jetson Python2／實車 runtime、相機延遲／視野／停止距離仍未驗收，沒有部署。
影像來源驗收閘保持關閉。離線合成故障通過不代表已核准實車自動續行，也不保證未知曲折滑行或未實測視野可補足；覆蓋證據不足應停止並保留檢查點。
