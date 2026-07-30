# 完整任務流程設計 — 巡航撿垃圾（Phase 8）

> 狀態：設計文件 v1.3（2026-07-16：再次對照 repo、實機 set_motor/odom 紀錄與
> 最新導航權重後修訂），尚未實作。v1.3 統一 odom topic／TF ownership、釐清獨立
> motor server 與最終單程序 runtime 的邊界，並新增細部落地規格
> `integration/SETMOTOR_ODOM_INTEGRATION.md`。
> 前提：Phase 3–6 全部過關後才開工。本文件不影響眼前的 Phase 3＋Phase 5 上機計畫。

## 1. 任務描述

機器人在**一個已建圖的空間**中自主巡邏撿垃圾：

1. 沿預先畫好的路線巡航；
2. 看到垃圾 → 切換為視覺導航前往（避障）；
3. 到達 → 視覺精對位 → PPO 夾取；
4. 夾到後送到地圖上的**固定垃圾桶區域**放置；
5. 從垃圾桶續接路線的**下一個 waypoint** 繼續巡航；
6. 走完整條路線任務結束。

## 2. 核心架構決策（含討論結論）

### 2.1 一個導航策略＋三種目標來源（不需要「巡航權重檔」）

nav RL 策略（`integration/nav_rl.py`）的 obs 前三維是 `[dist, sin(bearing), cos(bearing)]`
——**相對量**，策略從頭到尾不知道地圖存在。相對量是外部餵的，來源可以替換：

| 狀態 | 目標來源 | 策略 |
|------|----------|------|
| 巡航 PATROL | `route.yaml` 下一個 waypoint（AMCL 位姿換算） | 同一個 nav RL |
| 接近垃圾 APPROACH | YOLO 視覺（現有 mode C） | 同一個 nav RL |
| 送垃圾桶 DELIVER | 地圖固定區域（AMCL 位姿換算） | 同一個 nav RL |

絕對→相對換算只有兩行（與訓練時 iGibson env 在 sim 內做的轉換**同構**，
`navigation_env_room.py` 的 `atan2(rel_y, rel_x) − yaw`）：

```text
dist    = hypot(x_w − x_r, y_w − y_r)
bearing = atan2(y_w − y_r, x_w − x_r) − θ_r     # 正規化 ±π，左正（同 GoalTracker）
```

**結論：獨立巡航權重檔取消**（尚未開訓、零沉沒成本）。它只在「無定位、要 RL 學遊走
覆蓋」時有意義；已選 AMCL 定位，該場景不存在。訓練機不用開這條線。

導航權重版本要和「目標來源」分開看：目前預設仍是 `ppo_nav_281440_steps`；
`doorway_ft_final` 是 2026-07-16 新增、同為 55D obs／2D action 的候選權重。候選只能用
同場地 A/B 實車 gate 決定是否升預設，不能因模擬成功率較高就直接切換。

沿用注意：
- waypoint 間距 0.5–1m 且 **間距 > 2× 到達半徑**（否則到達判定會互相干擾），
  讓每段 dist 落在訓練分佈內；**策略是局部避障，不是規劃器**，繞不出大型障礙
  ——中繼點**不可跨牆或跨大型障礙**，每段只讓策略解局部避障。
- bearing 號向照抄 `GoalTracker` 慣例（左正），不要自創。
- **去垃圾桶的路徑選擇**（三輪回饋補）：waypoint 密度本身不回答「從任意夾取位置
  怎麼走到垃圾桶」。單一房間／簡單場地 → 把垃圾桶放在巡邏迴圈上，DELIVER 沿
  迴圈走最近方向即可；有房間或岔路 → `route.yaml` 升級為 **waypoint graph**，
  用圖搜尋（最近節點→BFS/Dijkstra）選中繼點鏈。先假設前者，場地定案後再決定。

### 2.2 定位 = ROS 只當「定位服務」，保住串列直控與夾取迴圈不動

**鐵律：`/dev/myserial` 只能有一個擁有者。** 全面上 ROS 意味著已驗證的手臂 PPO
緊迴圈（逐步 3° 限幅）改走 topic，大手術、不做。採混合架構：

```text
主 pipeline（py3.8，唯一 myserial owner）    ROS 端（ROS1，只跑定位）
├─ 手臂+底盤共用一個 Rosmaster handle       ├─ LiDAR driver（依實際型號）
├─ feedback → /odom_setmotor + TF ────────→ ├─ map_server（一次性建好的圖）
├─ 訂閱 /scan（取代直讀 LiDAR）  ←──────── ├─ AMCL（唯一 map→odom owner）
└─ 訂閱 /amcl_pose → 換算 dist/bearing      └─ rosbridge_server
```

- py2.7（Melodic）↔ py3.8 橋：**rosbridge（websocket）+ roslibpy**，6Hz 控制下延遲無感。
- **實作範圍（二輪審查修正：不是「零改動」）**——保住不動的是「`/dev/myserial` 直控＋
  手臂 PPO 夾取迴圈」；導航層需要以下**新增與重構**：
  1. ROS bridge 模組：roslibpy `/scan` 訂閱已完成（2026-07-16）；`/amcl_pose` 訂閱與
     canonical `/odom_setmotor`＋TF 發佈仍待 Phase 8。需要 `/odom` 的 consumer 用 remap，
     不得另算第二份 odom。
  2. feedback odom：直控程序內讀 `get_motion_data()`、按時間戳積分、經 bridge 發佈；
     encoder 在規格確認前先作交叉驗證。
  3. 地圖目標提供者（MapGoalProvider）：waypoint／垃圾桶區域 → dist/bearing。
  4. **導航迴圈重構**：現有 `_rl_navigate()`（`nav_rl_grasp_pipeline.py`）把
     「YOLO 偵測→tracker 更新」寫死在迴圈裡；LiDAR 已改為可插拔 ROS/RPLidar backend，
     目標來源仍要抽出
     「目標來源無關」的通用導航迴圈，視覺／地圖目標源皆可插拔。
  5. `/scan` 轉接層已完成（REP-103 角度、latest-only queue=1、資料 age／stale 停車）；
     實機前／左／右與零點校正仍須通過，資料契約見 §5。
- **串接順序（三輪回饋更正：odom/TF 必須最先，不能先 bridge `/amcl_pose`**——
  沒有 scan、`odom→base_footprint→base_link→laser` 時 AMCL 根本不會產生可用定位）：
  1. 確認 `/dev/myserial` 單一擁有者；
  2. 產生並驗證 `odom → base_footprint`（motion feedback 積分；encoder 先作交叉驗證）；
  3. 由 robot_state_publisher 保持 `base_footprint → base_link → laser_link`，只在確有需要時
     由唯一 static owner 補 `laser_link → laser`；
  4. ROS 端自成一體確認 `/map`＋`/scan`＋TF → AMCL 可運作（RViz 在外部 PC 看）；
  5. py3.8 才訂閱 `/amcl_pose`；
  6. 對已完成的 `/scan` 轉接層做 §5 鏡像／零點實機 gate；
  7. 最後 MapGoalProvider＋底盤控制。
- rosbridge 傳輸注意：`/scan` 整條陣列走 websocket JSON 有量，訂閱一律
  **latest-only、queue=1**；實測仍超載 → 在 ROS 端先降為 48 rays＋front-min 再傳，
  **不要先犧牲 AMCL 品質**（如亂降粒子數）。
- 建圖一次性離線做（出廠 ROS stack + gmapping 掃一遍），與 runtime 架構無關。
- 垃圾桶=**區域**：到位判定「AMCL 位姿進入區域 / 離中心 < r」，對 AMCL ±10–20cm
  誤差寬容；放置點取區域邊緣朝內固定 pose。

### 2.3 set_motor 與最終 runtime 的邊界

下載紀錄中的 `ai_motor_server_B.py` 可用來**獨立恢復** feedback odom，但最終導航＋夾爪
不能讓它與主 pipeline 各自開一次 `/dev/myserial`。正式架構採以下順序：

1. 先在不跑手臂／導航時找回並驗證正確 server，重現 `/odom_setmotor` 約 20 Hz 與
   `odom → base_footprint`；
2. 再把已驗證的四輪映射、watchdog、feedback odom 抽成接收既有 Rosmaster handle 的模組；
3. 導航器由直接 `set_car_motion()` 改成 `ChassisActuator.command_velocity(vx, vy, wz)`；
4. `SetMotorActuator` 必須支援 RL 的連續速度，只有 forward／turn／curve 的離散命令不夠；
5. 手臂、底盤、feedback 最終共用 `GraspController.servo.device`。

完整模組契約、odom 公式、stationary gate、啟動順序與驗收 gate 見
`integration/SETMOTOR_ODOM_INTEGRATION.md`。

### 2.4 Blacklist（防死鎖，必做）

夾取失敗 ×3 直接回巡航會**無限迴圈**（物體還在原地→再偵測→再夾→再失敗）。
有 AMCL 後成本極低：失敗時記物體的地圖座標，之後偵測換算位置落在該點
**忽略半徑內即忽略**（本輪任務內永久，或冷卻制）。

二輪審查補充：
- 記錄座標取 **latch 時刻**的目標 base 座標＋**同時刻（時間對齊）的 AMCL 位姿**換算，
  不用偵測瞬間的粗略距離（>1.5m 的距離值本來就不可信，見 §3）。
- 忽略半徑預設 0.3m，列為**實測後可調參數**（受 AMCL 誤差與物體密度影響）。

## 3. 狀態機（v1.3）

```text
BOOT → SELF_CHECK → IDLE（serial/feedback/ROS/TF 任一不健康都不得 start）
PATROL: 沿 route.yaml waypoint 走（nav RL + AMCL 目標）
  ├─ 同一 track 連續 N 幀（見下方觸發定義）且 不在 blacklist：
  │    ├─ 距離量測有效（≤1.5m）→ APPROACH（記當前 waypoint index）
  │    └─ 僅 bearing 可信（>1.5m）→ INVESTIGATE
  └─ 走完最後一個 waypoint → 任務結束
INVESTIGATE: 朝 bearing 轉向＋慢速前進，直到後鏡頭距離量測進入有效區（≤1.5m）
  ├─ 取得有效距離 → APPROACH
  └─ 超時／目標丟失 → PATROL（續接原 waypoint）
APPROACH: nav RL + 視覺目標（現有 mode C）
  ├─ lost timeout → PATROL（續接原 waypoint）
  └─ 到 stop-dist → ALIGN → STATIONARY_GATE → LATCH → GRASP
GRASP（現有流程，重試 ≤3）
  ├─ 失敗×3 → 物體地圖座標進 blacklist（§2.4 對時規則）→ PATROL
  └─ 成功 → 手臂收攜帶 pose → DELIVER
DELIVER: nav RL + 地圖目標（垃圾桶區域，沿路線給中繼 waypoint 鏈）
  ├─ 卡住 超時 → 停車回報（print+log），任務暫停
  └─ 進入區域 → STATIONARY_GATE → 原地轉向至放置 yaw → 再次停穩 → PLACE
       → PATROL（續接下一個 waypoint）
全程：LiDAR 煞停常駐；攜帶中忽略新偵測；AMCL 健康監測常駐（見下）；
      每次狀態切換／目標源切換／停車重啟時，**清空 ActionDelay 緩衝並重置
      GoalTracker**（延遲佇列裡是上一個目標的動作，殘留會讓新目標的頭兩步亂走）
全域錯誤出口：sensor stale／serial error／watchdog／exception → 先送底盤零命令，再進
PAUSED 或 FAULT；ESTOP 不得自動移動手臂，必須人工 reset。
```

`STATIONARY_GATE` 不能只靠「已送 stop」判斷；必須等 motion feedback 連續多筆落在靜止
noise 門檻內，底盤未停穩時禁止啟動 PPO 手臂。詳細 entry／exit／timeout 行為見
`integration/SETMOTOR_ODOM_INTEGRATION.md` §6–8。

**切換觸發（二輪審查修正）**：「連續 N 幀**同類別**」不足以保證是同一個物體——
觸發條件是「**同一 track** 連續 N 幀」：同類別＋幀間 IoU 或幾何位置一致
（與 `integration/KALMAN_FILTER_DESIGN.md` §9 的資料關聯原則同一套，屆時共用實作）。

**遠距偵測缺口（INVESTIGATE 的由來）**：後鏡頭距離模型僅 0.7–1.5m 校正有效；
nav policy 的 obs 需要 dist，現有程式也只在取得有效距離時才更新 tracker
（`nav_rl_grasp_pipeline.py` 的 `dist_f > 0` 檢查）——所以 >1.5m 的偵測**不能直接進
APPROACH**，先進 INVESTIGATE 用 bearing 轉向慢速靠近，距離進入有效區才切換。

**AMCL 健康監測（二輪審查新增）**：`/amcl_pose` 新鮮度、位姿協方差門檻、TF 可用性
三者任一失敗 → 依賴地圖目標的狀態（PATROL／DELIVER）**立即停車回報**，不等
行為逾時才處理；視覺目標的 APPROACH／ALIGN 不依賴定位，可繼續完成當前目標後再停。

## 4. 問題與討論記錄（2026-07-10）

| # | 質疑 | 討論結論 |
|---|------|----------|
| 1 | 「送固定位置＋回原位」需要定位，現有系統沒有 | 採 AMCL（ROS 定位服務架構，§2.2）；垃圾桶=地圖區域 |
| 2 | 巡航權重的 obs/plant 規格未定義；RL 遊走 vs 固定路線矛盾 | 路線=手畫 waypoint → **巡航權重取消**（§2.1），巡航=同一 nav RL 走點 |
| 3 | 「回到切換前位置」成本高收益低 | 弱化為「從垃圾桶續接下一個 waypoint」 |
| 4 | 失敗分支全缺 | lost→回巡航；夾取失敗×3→**blacklist**（§2.4）→巡航；攜帶中不切換目標；去垃圾桶卡住→停車回報；放置不驗證（接受偶爾掉桶外） |
| 5 | 物理問題 | 垃圾桶=淺盤/低開口（手臂可達）；放置腳本化（細節另議）；攜帶限速另議；已確認手臂/物體不擋 LiDAR 與後鏡頭 |
| 6 | 二輪審查（同日）：低估 ROS 資料接線與狀態機邊界 | §2.2 改為誠實的實作範圍（bridge/odom/MapGoalProvider/導航迴圈重構/scan 轉接）；§5 補 TF 鏈與 LiDAR 資料契約（左右鏡像風險）；§3 新增 INVESTIGATE 態、觸發改「同一 track N 幀」、AMCL 健康監測即時停車；現 §2.4 blacklist 改 latch 時刻對時、半徑列為可調 |
| 7 | 三輪外部回饋（同日）：地圖對齊、串接順序、Nano 資源 | 六點大多採納但修法有更正：①地圖/scan 對不上→**先靜態驗外參與鏡像、再看飄移模式**，不可盲調 map resolution/origin（§5 gate 3）；②串接順序改為 odom/TF 最先（§2.2）；③`--probe` 目前只測直讀 RPLidar，ROS 轉接層要有自己的 probe 路徑（§5 gate 5）；④Nano 資源先量測再談降級，不可先砍 AMCL 粒子（§5 gate 6）；⑤回饋中「/odom_setmotor 已恢復」**查證後 repo 內不存在**，僅參考驅動有 API（gate 1）；另補 waypoint graph（§2.1）、放置前 yaw 對位與 ActionDelay 清空（§3） |

相關：YOLO 目標追蹤的 Kalman filter 設計（`integration/KALMAN_FILTER_DESIGN.md`）
結論為「Phase 5 先只加記錄、依 log 決定輕量版→完整版」；其中「連續 N 幀穩定」
判定與本文件的切換觸發共用。

## 5. 前置驗證（go/no-go gates，Phase 8 第一步；v1.3 擴充）

1. **編碼器/輪速 odom（最大 go/no-go）**。已查證：參考驅動
   `grasp/Rosmaster_Lib_reference.py` 有 `set_auto_report_state()`（:332）、
   `get_motion_data()`（:942）、`get_motor_encoder()`（:948）——但後兩者只回傳
   **由接收執行緒更新的快取欄位**，而主程式目前只呼叫 `create_receive_threading()`、
   從未開 auto-report。上機必驗：開 `set_auto_report_state(True)` 後數值**持續更新
   而非重複舊值**；直走 5–10m 距離比例、原地旋轉 yaw 正負與比例都對。
   ⚠️ 外部回饋稱「/odom_setmotor 已恢復」——**repo 內不存在**此節點/topic，
   使用前先釐清它是什麼；若它會開 `/dev/myserial` 即違反單一擁有者，不可與主
   pipeline 同時跑。
2. **serial 單一擁有者稽核**：runtime 全程 `/dev/myserial` 只有一個程序開啟
   （`fuser /dev/myserial` 查證），LiDAR 序列埠同理（歸 ROS 節點）。
3. **地圖↔/scan 對齊（RViz 疊圖，診斷順序不可倒）**：
   a. 機器人**靜止**：先驗 `base_link→laser` 外參與 scan 左右方向（gate 5）；
   b. 手動 2D Pose Estimate 後，靜止時 `/scan` 應貼牆；
   c. 慢速直走＋原地旋轉＋繞一圈：**一開始就對不上** → 地圖比例/origin/雷射外參
      問題；**開始對得上、移動後逐漸飄** → odom 比例/yaw/時間戳問題。
   不可看到不吻合就盲調 map YAML 的 resolution/origin（那會把 TF/外參/odom 問題
   蓋掉）：gmapping 用同一顆 LiDAR 建的圖 → 優先重建或修 TF；外部平面圖轉的圖
   → 才依實際丈量校正比例與 origin。
4. **TF 鏈**：`map → odom → base_footprint → base_link → laser_link → laser` 完整，且**每筆 scan 的時間戳都能
   解析**（不是只在當下時刻能查）；`rosversion -d` 確認版本（預期 Melodic）＋
   rosbridge 裝得起來。
5. **LiDAR 資料契約與鏡像**：ROS backend 已把 `angle_min + i×angle_increment` 轉成
   REP-103 逆時針／左正角度，`scan_to_rays` 的 ROS 預設為 `--lidar-dir 1`；只有舊
   RPLidar direct backend 預設 `-1`。`--probe --lidar-backend ros` 走的就是正式轉接層，
   並已用單元測試確認左右不鏡像。仍須前／左／右放手測＋raw 煞停實機重驗，
   通過前不准跑巡航。分工維持清楚：`--lidar-dir` 修左右手性、
   `--lidar-yaw-offset-deg` 修零度方向——**不可用 yaw offset 修鏡像**，也不可在
   轉接層與 `scan_to_rays` 各翻一次。
6. **Nano 資源（先量測、後降級）**：記錄 CPU/RAM/swap/溫度與降頻、YOLO 推論時間、
   6Hz 控制迴圈實際週期、`/scan`／`/amcl_pose` 的接收 age（p95/p99）、rosbridge
   是否累積舊訊息。建圖/定位測試時**不開 YOLO**；RViz 移到外部 PC；資源不足時
   先用 §2.2 的 ROS 端預縮減方案，**不要先砍 AMCL 粒子數**。
7. **stale→停車總則**：`/scan`、odom、AMCL 位姿任一 stale 都必須停車
   （現有 LiDAR stale 規則的推廣）；rosbridge 延遲與資源量測通過後才允許開 YOLO
   進入完整任務。
8. **actuator 介面 gate**：現行 pipeline 仍直接呼叫 `set_car_motion()`；切到自製
   `set_motor()` 前必須完成連續 `(vx,vy,wz)`→四輪 mapping、同比例飽和、deadband、
   acceleration limit、命令 watchdog、停止後 feedback 確認，且不得改變 nav policy 的 6 Hz plant。
9. **導航權重 A/B gate**：先用目前預設 `ppo_nav_281440_steps` 建立實車 baseline，再以完全
   相同起點／目標／障礙測 `doorway_ft_final`；兩者 zip/pkl 必須成對。`--bench` 只證明載入、
   維度與速度，不代表實車導航較好。

## 6. 落地順序

1. **8A 原檔回收**：取回 `ai_motor_server_B.py`、launch、driver 版本與校正資料，先做只讀稽核。
2. **8B 獨立 feedback odom 恢復**：不跑手臂／導航，通過細部規劃 A1–A6。
3. **8C 單程序 actuator 合併**：抽出 SetMotorActuator＋MotionFeedbackOdom，共用既有
   Rosmaster handle，通過 B1–B5；舊外部 motor server 不再參與正式 runtime。
4. **8D ROS 定位資料鏈**：`/odom_setmotor`＋TF → `/scan` adapter → map_server＋AMCL；
   依 §5 gate 3–7 驗 RViz 貼圖、timestamp、stale 與資源。
5. **8E 一次性建圖／地圖驗收**：建圖程序和 runtime 分離；保存 map、量測比例與版本。
6. **8F 無視覺純巡航**：`route.yaml` + nav RL waypoint 鏈；先預設權重，再候選 A/B。
7. **8G 偵測與操作**：同一 track N 幀→INVESTIGATE／APPROACH→stationary gate→夾取；
   分別驗 lost、retry、blacklist。
8. **8H DELIVER／PLACE／RESUME**：先空爪、再帶假負載、最後真物體；驗放置 yaw、攜帶限速、
   中斷後續接下一 waypoint。
9. **8I 全任務 soak test**：多輪巡航撿取，記成功／碰撞／timeout／stale／重定位／CPU 溫度；
   所有 gate 過後才把候選權重或參數升成預設。

## 7. 待決事項

- 放置腳本細節（pose、可達性 dry-run）——垃圾桶實物到位後定
- 攜帶 pose（nav home 是張爪姿勢，不適合攜帶）與攜帶限速
- N 幀觸發的 N 值、blacklist 半徑/冷卻時間、waypoint 到達門檻——上機調
- INVESTIGATE 的轉向/慢速速度、超時秒數；AMCL 健康監測的三個門檻
  （pose 新鮮度、協方差、TF timeout）——上機調
- `route.yaml` 格式（map frame 座標列表＋垃圾桶區域定義）；場地定案後決定
  單一迴圈 vs waypoint graph（§2.1）
- 放置前 yaw 對位的實作（AMCL yaw 原地轉向即可，或需視覺輔助——垃圾桶實物到位後定）
- 停車回報的通知方式（先 print+log，之後可加蜂鳴/LED）

## 8. 與現行計畫的關係

- **不影響** Phase 3（座標對齊）＋ Phase 5（避障測試）→ Phase 4 → 6 的既定順序。
- 訓練機：**不用訓巡航權重**；時間可轉投評測或（若之後想加視覺兜底）垃圾桶偵測資料。
- 本文件的 Phase 8 在 Phase 6 全流程過關後啟動，第一個動作是 §5 的 go/no-go。
- `set_motor`／feedback odom 的細部實作與每次上機啟停順序以
  `integration/SETMOTOR_ODOM_INTEGRATION.md` 為準；舊的 22D／Arm_Lib ROS 部署指南不適用。
