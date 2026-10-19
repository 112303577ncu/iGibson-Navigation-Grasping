> 最新接手狀態：請先讀 [koala小工具交接](KOALA_CORNER_TOOL_HANDOFF_2026-10-19.md)。v2已加入本輪讀到的C牆ID1／5，巡航碰撞驗收失敗，現在暫停；下方保留較早階段紀錄。

# ArUco 分辨角落 → LiDAR 起點 → AMCL 確認

## 10/19上車更新（優先於下方原離線說明）

已配套部署監督器、角落工具、source-timestamp相機取樣及task_resume_guard；實車以目前位置的ID0／2／7完成
`LOCALIZED`，沒有發底盤非零命令。AMCL child故障注入6.03秒恢復，後續124.54秒／246筆健康樣本全ready，
odom增加2473筆、Serial owner維持4443、四輪零／travel，該等待段可用RAM最低1569.27MB。
尚未push；目前是單一起點實測，不是所有未知位置／完整巡航驗收。

- 現車唯一Serial owner從8033換成4443，是修正開機停穩時間回報後的明確服務重啟；沒有覆寫手臂controller及深度校正。
- 從未移動的底盤原本stopped_for_s=null，新增第一個已確認零命令的時間，只修正telemetry，不改原先手臂／底盤閘。現車60項chassis回歸通過。
- `deploy/ros/g2_corner_camera_capture.py`可在巡航前直接取後相機三張實際影像，使用V4L2 driver的monotonic來源時間轉到Jetson時基；本輪來源影像延遲約25～39ms。它不發速度／手臂命令。
- 相機JSON匯出須**逐位元組保留native capture**，不能重新排版後拿另一份hash上車；第一次真實提交因這點被正確拒絕，修正後重新取樣成功。
- 目前正式粗略表為`maps/demo_box_20261018_patrol/aruco_corner_landmarks.json`，只登錄本輪影像確認的0／2／7；8張example仍停用。位置沿用安裝配置的20cm誤差界，不冒稱精密survey。其他5張及不同起點仍待驗證／擴充。
- 相機未鏡像：原始影像三個ID可解碼，水平翻轉後無ID；標記名義方位／畫面位置給出約61°水平視野的一致性檢查，正式表仍保守用100°上界與10°方向誤差，並非完整相機內參標定。
- 候選至少85%牆點須在地圖牆面5cm內，與scan_match的85%截尾目標一致；不能讓低截尾平均掩蓋17%不吻合的牆。候選最低分不是正確角落，本輪正確為index1。
- 起點定位成功不等於可直接巡航。本輪preflight回報current centre outside navigation mask、0脈衝；需先向場內調整，不能對這個原因一律後退。

影像、原始sensor、完整候選／結果、部署備份位置與恢復／等待記錄在`docs/calibration/aruco_live_2026-10-19/`。

### 簡化的Jetson相機取樣入口

若後相機尚未被串流占用，可在開機／巡航停止後執行：

```bash
/home/jetson/grasp_venv/bin/python3 deploy/ros/g2_corner_camera_capture.py \
  --output-dir /tmp/corner_batch --registration-id coarse_wall_layout_20261019_v1
```

它輸出capture.json、observations.json及三張JPEG，capture與`/dev/shm/g2_corner_capture.json`位元組相同。
若koala的rear stream已持有相機，須先協調停止**自己的後相機sender**，取樣結束後再恢復；不停止grasp-vision／手臂相機，不開第二個reader。
目前只登錄0／2／7，遇到其他未登錄ID不能假裝已確認；應先擴充正式表或選擇只含已登錄ID的觀察方向。

### Windows與Jetson地圖YAML差異

現車AMCL YAML的image是Linux絕對路徑，因此byte hash為`2274a1c2...`，與repo相對路徑版`063cd6bc...`不同。
保留現車地圖不改；PC須下載實際使用的YAML，使用同一raster及新增參數：

```powershell
python integration/g2_scan_match.py --snapshot corner_capture.json --scene maps/demo_box_20261018_patrol --reference-map-yaml active_map.yaml --output corner_candidates.json
```

工具核對解析後geometry／thresholds，再以原YAML位元組及本地同一raster產生hash；不把不相同的地圖或時間偽裝成相同。
下方較早不帶reference參數的指令只適用兩端YAML byte一致的情況。

## 工具及範圍

- `deploy/ros/g2_corner_snapshot.py`：Jetson native ROS Python2，5秒內取得新鮮 `/scan`、odom及capture ID；確認唯一Serial owner、停穩／travel、1024MB RAM、LiDAR TF前移10cm／yaw π。**不需要先有AMCL**。
- `integration/g2_scan_match.py`：筆電計算候選，保留 `accepted=false`；新增snapshot SHA256及raw-map hash，不能把別次轉車前的候選拿來用。
- `integration/g2_aruco_corner_seed.py`：以三個新影格中的ID／畫面左右方向篩選候選。預設只做離線判斷；Jetson加`--real`才核對現場資料、寫一次seed並等AMCL確認。
- `maps/demo_box_20261018_patrol/aruco_corner_landmarks.example.json`：8張標記的**配置圖推算值**，預設未驗證。不是現場已量測登錄。
- `deploy/ros/aruco_corner_observations.example.json`：koala輸入格式；示範時間與ID綁定不可直接作真實觀測。

這條路線由LiDAR計算公尺位姿，ArUco只排除相似角落，無須完整相機內參或PnP。
仍需確認相機朝向、是否鏡像及大致水平視野上界；預測使用標記位置／相機偏移／方向的不確定區間。
仍可能有多個候選時回報AMBIGUOUS，不憑最低LiDAR分數選角落。
與 `v23_task_resume_guard.ReferenceCollector` 的「已校正metric ArUco位姿」是不同輸入介面，不混用範本。

## 下次開機需要確認的資料

1. ID實際貼位是否與配置一致：A牆0／2、B牆3／4、C牆5／1、D牆6／7；各點約距參考角50／150cm。A為紙盒後牆，B為紙盒左牆。若不同，修改各marker的point_map及面向場內的normal_yaw_rad。
2. 後相機是否向車頭、影像是否鏡像、約略水平可視角度；不需要先完成完整標定。填妥camera確認旗標及不確定度。
3. 確認PC拿到的是最新來源影格，並把來源時間轉到Jetson時基；影格與capture相差不得超過1秒，三張跨度不得超過1.5秒。不可用「傳送時間」替代影像時間。只有接收時間且延遲未知時，先處理串流時效，不能啟用真實提交。

確認後給登錄表一個新的registration_id，再將registration_verified設true。標記移位、鏡頭改位或換地圖後重新確認；不要僅因有配置PDF就設true。
範本目前位置誤差界20cm、相機偏移誤差界20cm、方向誤差界10°，是保留歧義的起始值。實際超出這些界限就需重新量測／登錄。

## koala 串接順序

總指揮只在開機／巡航重定位階段呼叫。結果不明的手臂動作須先處理，工具本身不重試手臂。

1. 分段轉向由koala既有resident控制器執行，停穩並釋放TCP控制權。
2. Jetson取一份capture；PC保留附近三個**不同來源影格**的ArUco結果。相機可持續取樣，不必每次重開。
3. PC抓取capture後跑LiDAR比對，組成observations JSON；分析期間車保持不動。
4. PC可先離線預覽；將候選、observations及已確認登錄表傳到Jetson不同檔案路徑，再呼叫`--real`。
5. 只有 `ok=true AND localized=true AND status=LOCALIZED` 才能進巡航。巡航入口仍會重新核對即時健康；valid_until只是當下檢查時效，不是永久移動許可。

輸入綁定須照capture複製：capture_id、boot_id、owner_pid、map_hash；另帶camera_id及registration_id。
frames需3～10張，seq／stamp嚴格遞增，每張markers為`[{"id":2,"side":"left"}]`等。
side是**影像中的**left／center／right，不能先自行轉成車體左右後又讓工具翻轉。
可把畫面寬度中間40%～60%定為center，其餘left／right；不需像素→角度內參換算。
至少一個ID須出現在全部影格，多張穩定標記一起約束；未知／重複ID不接受。

Jetson取樣：

```bash
cd /home/jetson/Documents/deploy_jetson2
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
/usr/bin/python deploy/ros/g2_corner_snapshot.py --map maps/demo_box_20261017/demo_box.yaml
```

PC先將`/dev/shm/g2_corner_capture.json`下載為`corner_capture.json`，並準備`corner_observations.json`。
候選比對放PC，避免把SciPy搜尋負擔放在Jetson；總指揮應給比對子程序60秒上限，逾時取消並重新取樣。
PC的raw map須與Jetson檔案hash相同；不能改換成膨脹後的中心通行mask。

```powershell
python integration/g2_scan_match.py --snapshot corner_capture.json --scene maps/demo_box_20261018_patrol --output corner_candidates.json
python integration/g2_aruco_corner_seed.py --snapshot corner_capture.json --candidates corner_candidates.json --observations corner_observations.json --landmarks maps/demo_box_20261018_patrol/aruco_corner_landmarks.example.json --output corner_selection.json
```

將`corner_candidates.json`、`corner_observations.json`及已確認登錄表傳到Jetson，例如`/tmp/`。
**不要覆寫Jetson自己的`/dev/shm/g2_corner_capture.json`。**最後在Jetson執行：

```bash
/home/jetson/grasp_venv/bin/python3 integration/g2_aruco_corner_seed.py \
  --snapshot /dev/shm/g2_corner_capture.json \
  --candidates /tmp/corner_candidates.json \
  --observations /tmp/corner_observations.json \
  --landmarks /tmp/confirmed_corner_landmarks.json \
  --output /tmp/corner_localization_result.json --real
```

純離線成功回報SELECTED_DRY_RUN，localized=false，不寫seed；exit code 0不等於已定位。
真實提交要求：capture在120秒內、boot／唯一owner／raw-map hash相符、1024MB可用RAM、板回傳新鮮、無速度控制者、travel且一直停穩，odom偏移≤1cm／1°。
seed保留capture的原時間與odom，不更新舊資料時間。AMCL監督器須有accepted_reference_stamp，且五個不同odom新幀健康穩定、結果距候選≤8cm／8°才回報LOCALIZED；等待最多20秒。
等待期間RAM低於768MB、owner改變或車被移動時不宣告成功。

## 結果處理

| status | 總指揮的下一步 |
|---|---|
| LOCALIZED | 從當前位置規劃接入巡航 |
| SELECTED_DRY_RUN | 僅預覽，不可啟動巡航 |
| AMBIGUOUS／NO_MATCH／NEED_MORE_FRAMES | 再分段觀察；轉動後重新capture、重新比對，不混用舊候選 |
| LANDMARKS_UNVERIFIED／CAMERA_DIRECTION_UNVERIFIED | 補齊貼位與相機方向確認 |
| CONTEXT_MISMATCH／CONTEXT_CHANGED／CAPTURE_MOVED／CAPTURE_EXPIRED | 棄用本次資料，取得新capture與影像 |
| AMCL_TIMEOUT | 查定位健康reason；不要重打seed時間、不要開始巡航 |
| SUBMISSION_UNCERTAIN | 先查seed／健康狀態，不能自動重送；必要時取得新capture |
| WAIT_MEMORY／STALE_FEEDBACK／WAIT_STOPPED_TRAVEL | 等待對應狀態恢復後重新評估 |
| SIMULATION_ONLY | 這是測試登錄表，不能上車；改用現場確認的正式表 |

同capture重複呼叫且seed內容完全一致時，只重新查AMCL結果，不改寫seed或時間。
receipt先寫意圖；若中間斷線／寫入失敗而無法確認seed，回SUBMISSION_UNCERTAIN，避免無條件重播。
工具不發速度、手臂命令或開Serial，不自行環視、重啟服務或放寬LiDAR遮罩。

## 部署與驗證

尚未部署或push。需配套部署本輪兩支新工具、更新後的g2_scan_match.py、v23_task_resume_guard.py，
以及AMCL監督新版的service／policy／patrol／systemd單元；詳見AMCL_WATCHDOG_2026-10-18.md及TASK_WAIT_RESUME_2026-10-18.md。
舊監督器若沒有accepted_reference_stamp，工具會逾時，不把原本ready誤當新seed已接受。
這些檔案可精選交接給koala，不需合併其接近／手臂控制器。

下次先做靜止選候選及AMCL確認，再搬到另一位置重做；確認角落正確後才跑最近路段。
離線測試與歷史scan重播在`docs/calibration/aruco_corner_tool_2026-10-19/`。
重播的登錄確認／三影格時間綁定是模擬資料，不能當成實體相機／貼位驗收，不能用其seed上車。
