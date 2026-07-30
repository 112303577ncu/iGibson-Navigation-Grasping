# 校正 → 整合完整路線圖（相機內參 → 座標對齊 → RL 導航 → 全流程）

> 依序執行，**每個 Phase 有明確過關標準，沒過不要進下一步**。
> 每完成一個 Phase，把量到的數字填進「常數更新對照表」並更新 progress.md。
> 預估總時數：約 2–3 個工作天（Phase 1–3 一天、Phase 4 半天、Phase 5–6 一天）。

## 總覽

| Phase | 內容 | 過關標準（一句話） | 預估 |
|---|---|---|---|
| 0 | 環境同步與基礎檢查 | 相機串流可看、夾取 dry-run 照舊能跑 | 0.5h |
| 1 | 相機內參（fx/fy/cx/cy+畸變）×2 台 | 重投影誤差 < 0.5px | 1.5h |
| 2 | 地面距離模型（θ、H）×2 台 | 手臂相機 ±3cm、桅相機 ±10cm | 1.5h |
| 3 | YOLO 座標 ↔ 手臂 base 座標 | latch 輸出與實擺位置差 < 2cm | 1.5h |
| 4 | 定點視覺夾取端到端（含 smooth-approach） | 真物體夾取 ≥3/5 成功 | 3h |
| 5 | RL 導航單獨驗證（--nav-only） | 2m 直線到點 + 繞障不碰 + 煞停距離量測合格 | 3h |
| 6 | 全流程整合 | 偵測→導航→夾取 ≥2/3 成功 | 2h |
| 7 | 收尾 | PR merge、紀錄更新 | 0.5h |

> **執行順序調整（2026-07-10 決定）**：下次上機先做 **Phase 3 → Phase 5（避障測試）**，
> 這兩個通過後才回頭做 Phase 4 → 6。兩者互相獨立（Phase 5 只用後鏡頭+LiDAR+底盤，
> 不碰夾取；Phase 3 只用手臂相機+手臂），順序對調不影響過關條件。

---

## Phase 0 — 環境同步與基礎檢查

**目標**：Jetson 上有最新程式碼 + 權重，相機/序列埠可用，舊功能沒壞。

**步驟**
1. 電腦端先同步 GitHub `main` 並確認沒有拿到舊分支；再把 `x3plus/` 內容送上 Jetson：
   ```powershell
   git pull --ff-only origin main
   git log --oneline -3
   # 在 x3plus/ 目錄下傳「內容」，避免多一層 x3plus/x3plus
   .\set_jetson_host.ps1 172.31.28.252
   scp -r .\* "jetson@${env:X3PLUS_JETSON_HOST}:~/Documents/deploy_jetson2/"
   ```
2. Jetson：先跑 `python3 startup_device_check.py`。相機**一律依 serial/標籤辨識**，記下它輸出的
   `<ARM_N>` 與 `<REAR_N>`；不得假設 `/dev/video0` 或 `/dev/video1` 對應固定相機。
3. 若相機被佔用，對上一步辨識出的裝置執行 `sudo fuser /dev/video<ARM_N> /dev/video<REAR_N>`；確認是出廠程序後才停止它。
4. 起相機串流：`python3 stream_cam.py --device <ARM_N>`（手臂相機）；桅相機另開 `--device <REAR_N>`。
5. 確認：瀏覽器看得到兩路正確影像；`ls -la /dev/ttyUSB*` 記下各埠晶片（ch341=Rosmaster）。
6. 回歸測試：`cd grasp && python3 x3plus_real_grasp.py`（dry-run）跑完無錯。

**過關**：兩路串流正常、dry-run 照舊、序列埠清單記錄下來。
**沒過**：查 TROUBLESHOOTING #4–#6（串流/相機被佔用/裝置對應）。

---

## Phase 1 — 相機內參校正（你指定的起點）

**目標**：量出兩台相機真正的 fx/fy/cx/cy 和畸變係數。
目前程式裡有**兩套互相矛盾的常數**（pipeline 用 fx≈957、arm_cam.py 用 650），
本 Phase 一次定案，之後所有距離估計都建立在這上面。

**準備**：A4 列印棋盤格，貼平板上；尺量實際方格邊長（印表機常縮放）。
**本專案的板 = 7×10 方格（標示 20mm）→ 內角點 6×9**，即工具**預設** `--cols 9 --rows 6`
（校的是內角點＝方格數各減 1，不是方格數），只需 `--square-mm 20`。

**步驟**（每台相機各做一次；在 Jetson 上直接讀本地裝置 = 免串流、畫質最好）
1. 手臂相機（Phase 0 辨識出的 `/dev/video<ARM_N>`）：先把手臂移到**穩定姿勢**（建議直接用 nav home，反正之後就是這個姿勢）：
   ```bash
   cd ~/Documents/deploy_jetson2/grasp && python3 move_arm.py    # -> nav home，送完 servo 保持不動
   ```
   ⚠️ **內參與姿勢無關**——固定姿勢只是讓相機在校正過程中不飄，servo 上電別讓手臂垂下；
   校出來的 fx/fy/cx/cy 對所有姿勢通用。
   ```bash
   cd ~/Documents/deploy_jetson2/detection/calibration
   # 預設 --cols 9 --rows 6 已對應 6×9 內角點；只需給方格邊長
   python3 calibrate_intrinsics.py --source <ARM_N> \
     --square-mm 20 --out arm_cam_intrinsics.json --show
   ```
   （SSH 無螢幕時拿掉 `--show`，靠終端印的 `shot N/M` 計數；或改在**電腦端**對 Jetson 串流 URL 跑，
   這樣 `--show` 有畫面能確認棋盤有被偵測到、覆蓋是否夠。）
2. 拿棋盤在鏡頭前**慢慢移動並在每個位置停一下**（動態模糊會毀了角點精度）：中央、四角、近、遠、
   左右各傾 30°，自動擷取 18 張。
3. 桅相機（Phase 0 辨識出的 `/dev/video<REAR_N>`）同法：`--source <REAR_N> --out rear_cam_intrinsics.json`。
4. 有螢幕的話跑 `--check <json>` 看去畸變後直線是否變直。
5. **畸變決策**（工具會直接印出）：看 `distortion displacement` 的
   bbox-bottom-center 位移——距離模型讀的是**原始** y2/cx_box，這個數字就是誤差來源。
   - **< 2px** → 在本文件記「畸變：忽略」，程式不動。
   - **≥ 2px** → 偵測前對 frame 做 `cv2.undistort`（或至少對 bbox 底邊中點做
     `cv2.undistortPoints` 再進距離公式），列入 Phase 2 前的修改項。

**過關標準**
- RMS 重投影誤差 **< 0.5px**（理想 < 0.3）
- fx ≈ fy（差 < 2%）
- (cx, cy) 落在 (320, 240)±40 內（640×480）
- **畸變決策已記錄**（忽略 or 加 undistort）
- 兩台的 fx 出爐後就知道 957 和 650 哪個對（或都不對）

**沒過**：板不平/方格尺寸量錯/傾角覆蓋不足，重拍。單張誤差特別大的 view 重做。
**產出**：兩份 json → 填入對照表①，更新程式常數（見表）。

---

## Phase 2 — 地面距離模型校正（θ 俯角、H 高度）

**目標**：`estimate_ground_distance(y2) = H / tan(θ + atan((y2−cy)/fy))` 的 θ、H 定案。
fy/cy 用 Phase 1 的新值——**Phase 1 沒過關前做這步沒有意義**。

**步驟**（手臂相機在 nav home；桅相機直接做）
1. H 用尺量：鏡頭中心離地高度（手臂相機參考值：URDF 實算 ≈0.263m；量到差很多要查姿勢）。
2. 地上貼膠帶做距離記號：手臂相機 0.3/0.4/0.5/0.6/0.8m，桅相機 1.0/1.5/2.0/2.5m
   （從**鏡頭正下方地面點**起量）。
3. 每個距離放物體，跑 YOLO 記 bbox 底邊 y2（可用 `debug_tools/yolo_test.py` 或
   `calibrate_arm_camera_theta.py`，記得先把裡面 fy/cy 換成新值）。
4. 每點解 θᵢ = atan(H/Dᵢ) − atan((y2ᵢ−cy)/fy)，取中位數為 θ；帶回去算殘差。
5. 左右偏移驗證：物體放正前方偏左/右 10cm。`dist` 是地面前向距離，水平像素換算要先求
   `optical_depth = dist×cos(theta) + H×sin(theta)`，再用
   `offset = optical_depth×(cx_box−cx)/fx`，結果應 ≈ ±0.10m；不可直接以 `dist` 代替 optical depth。

**過關標準**
- 手臂相機：全部量測點距離誤差 **< 3cm**
- 桅相機：**< 10cm**（遠距容忍大些）
- 偏移量誤差 < 2cm、正負號正確（右偏 = 正）

**沒過**：H 重量、確認 y2 取的是「物體接地點」而非陰影；θ 殘差單邊偏 → cy 可疑，回 Phase 1。
**產出**：θ_ARM/H_ARM、θ_REAR/H_REAR → 對照表②。

---

## Phase 3 — YOLO 座標 ↔ 手臂 base 座標校正

**目標**：視覺輸出的 (x, y) 對齊 FK/policy 的 base 座標系（TROUBLESHOOTING #12 的方法）。

**狀態（2026-07-16）：⚠️ nav-home 已完成，grasp-home 待重標。** 舊四點結果
X=0.39cm、Y=0.33cm 只證明 **nav-home** 的遠距模型正確；它看得到約 25cm 之外的物體，
但實測 PPO grasp-home 的可用前伸半徑只有約 15cm。手臂切到 grasp-home 後，相機高度、俯角與
前後方向都改變，因此舊 `H/theta/CAM_TO_BASE_X/Y/SIGN_Y` 不可用於最後 latch。

**grasp-home 重標步驟**
1. 只把手臂移到相機標定姿態，不載入 PPO、不夾取：
   ```bash
   python3 grasp/x3plus_real_grasp.py --real --pose-only grasp-home
   ```
   程式退出後伺服機會保持姿態；標定期間車體與手臂都不可移動。
2. 在 grasp-home 畫面可見且物理可達的區域選至少 **6 個非共線點**（建議 9 點，覆蓋畫面
   上下左右，且外框要能包住物體 bbox 底邊）。以抓取中心地面記號為量尺原點，換算每點的
   `base_link` 絕對 `(X,Y)`；不要假設整個 15cm 圓都在相機 FOV。
3. 每個實擺點各跑一次，記錄輸出的去畸變中位數 `u,v`：
   ```bash
   python3 integration/vision_grasp_bridge.py \
     --stream 0 --calibration-only --calibration-samples 10 --once
   ```
   此模式不連 5555、不會送夾取座標。
4. 將至少 6 組 `U,V,X,Y` 解成平面 homography：
   ```bash
   python3 integration/grasp_home_homography.py \
     --point U1,V1,X1,Y1 --point U2,V2,X2,Y2 \
     --point U3,V3,X3,Y3 --point U4,V4,X4,Y4 \
     --point U5,V5,X5,Y5 --point U6,V6,X6,Y6 \
     --output integration/grasp_home_homography.json --max-rmse-cm 1.0
   ```
5. 另用未參與求解的點驗證；X、Y 各自誤差都須 <2cm。runtime 會要求至少 6 點、
   最大擬合誤差 <2cm，並拒絕 calibration hull 外的偵測（不允許外插）。

**過關標準**：grasp-home 驗證點 X/Y 誤差各 **<2cm**、完整物體底邊落在校正 hull 內，
且輸出目標離 grasp-home TCP ≤15cm。
**產出**：`integration/grasp_home_homography.json`；物體 Z 留到 Phase 4 依類別實抓微調。

---

## Phase 4 — 定點視覺夾取端到端（不含導航）

**目標**：只在 grasp-home 可見且 ≤15cm 的工作區，驗證平順化與校正後視覺夾取。

> **前置 gate**：必須先完成上面的 grasp-home homography。舊 nav-home 四點常數不能通過此 gate。

**步驟（依序）**
1. **近距固定座標 dry-run**：用實際基準點前方約 5／10／14cm 分別 dry-run；三點純軟體
   rollout 已通過，但 Jetson 仍需核對 `[Reach]`、lock pose 與關節限制。
   ```bash
   python3 grasp/x3plus_real_grasp.py --smooth-approach \
     --obj-x <base絕對X> --obj-y <base絕對Y> --obj-z 0.02 --max-steps 180
   ```
   超過 15cm 必須在任何 policy/glide 前被拒絕。
2. **跨指間隙量測**（TROUBLESHOOTING #10）：S6=30° 開爪內側間隙 vs 物體寬 →
   單側裕度 ≥2cm 才安全。
3. **固定座標真物體夾取**：物體必須在 grasp-home TCP 15cm 內；先選約 10cm 的中央點。
   `--obj-x/--obj-y` 填 base-frame 絕對座標。瓶蓋先用 `--obj-z 0.02`，不可套用已撤銷的
   H_real−TCP z 算法。
4. **grasp-home 視覺 latch**（不要跑完整 pipeline；它在新 final-align 接妥前會拒絕 `--real`）：
   ```bash
   # 終端機 1：先啟動；手臂到 grasp-home 後會丟棄移動途中舊偵測
   python3 grasp/x3plus_real_grasp.py --real --socket --width-grip --latch-obj \
     --latch-wait-sec 30 --smooth-approach --i-confirm-external-frame --max-steps 180

   # 終端機 2：看到 [Latch] Waiting... 後才執行，只送一次
   python3 integration/vision_grasp_bridge.py --stream 0 \
     --homography integration/grasp_home_homography.json \
     --host 127.0.0.1 --port 5555 --obj-z 0.02 --once
   ```
   跑 5 次。

**過關標準**：步驟 3 ≥2/3、步驟 4 **≥3/5** 夾起且 verify 通過；過程無撞地/撞物。
**沒過**：夾空 → 座標差（回 Phase 3）；碰倒 → 看 #10 的對策順序（obj-z 對上半部→y 對位→墊高）。
**產出**：可用的定點夾取；TROUBLESHOOTING 補實測心得。

---

## Phase 5 — RL 導航單獨驗證（--nav-only，不夾取）

**目標**：導航策略上實車，先不碰夾取。細節見 `integration/NAV_RL.md`。

**步驟（依序）**
1. **LiDAR 資料鏈已確認（2026-07-16）**：實機是 YDLIDAR TG30，不是 Slamtec；
   `TG.launch` 已辨識 firmware 2.1／health good，`/scan` 約 10.17Hz。程式已加入
   rosbridge `/scan` adapter，Phase 5 不再卡在 Python `rplidar` 套件；不要安裝
   `rplidar-roboticia`。先開兩個終端機：
   ```bash
   # 終端機 1
   source /opt/ros/melodic/setup.bash
   source /home/jetson/software/library_ws/devel/setup.bash
   unset ROS_IP
   export ROS_HOSTNAME=127.0.0.1
   export ROS_MASTER_URI=http://127.0.0.1:11311
   roslaunch ydlidar_ros_driver TG.launch

   # 終端機 2
   source /opt/ros/melodic/setup.bash
   source /home/jetson/software/library_ws/devel/setup.bash
   unset ROS_IP
   export ROS_HOSTNAME=127.0.0.1
   export ROS_MASTER_URI=http://127.0.0.1:11311
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```
2. **方向校正**（終端機 3，`grasp_venv`）：
   ```bash
   cd ~/Documents/deploy_jetson2
   source ~/grasp_venv/bin/activate
   python3 -m pip install roslibpy  # 首次一次
   python3 integration/nav_rl.py --probe --lidar-backend ros --ros-host 127.0.0.1
   ```
   手放車**正前** 0.4m → 中間扇區數字應變小；再放**左**、**右**各測。
   ROS 預設已是左正（`--lidar-dir 1`）；若實測左右相反才改 `--lidar-dir -1`。
   整體偏 → `--lidar-yaw-offset-deg`。
   **順便用尺量 LiDAR 中心到車體中心的前後距離** → `nav_rl.NavRLConfig.
   lidar_forward_offset_m`（LiDAR 在中心前方為正；不填的話 48 束距離全部
   以 LiDAR 位置當車中心，近距離判斷會偏）。
3. **推論測試**：先跑目前預設 baseline，再測 doorway 候選；兩組 zip/pkl 不可交叉：
   ```bash
   python3 integration/nav_rl.py --bench
   python3 integration/nav_rl.py --bench \
     --model integration/nav_best_model/doorway_ft_final.zip \
     --vecnorm integration/nav_best_model/doorway_ft_final_vecnormalize.pkl
   ```
   兩組都要能以 SB3 2.3.2 載入且速率 ≥30Hz；bench 不代表實車導航已通過。
4. **轉向正負**：dry-run `python3 integration/nav_rl_grasp_pipeline.py --nav-only`，
   物體放偏左，log 的 `wz` 應為正；`--real` 低速首測若車右轉 → `--wz-sign -1`。
5. **直線到點 baseline**：空地、物體正前 2m，先用目前預設權重：
   ```bash
   python3 integration/nav_rl_grasp_pipeline.py --real --nav-only --nav-stop-dist 1.0 \
     --lidar-backend ros --ros-host 127.0.0.1
   ```
   通過後降 `--nav-stop-dist 0.75` 再測。
6. **doorway 候選 A/B**：在完全相同起點、物體與障礙配置，顯式加：
   ```bash
   python3 integration/nav_rl_grasp_pipeline.py --real --nav-only --nav-stop-dist 1.0 \
     --model integration/nav_best_model/doorway_ft_final.zip \
     --vecnorm integration/nav_best_model/doorway_ft_final_vecnormalize.pkl
   ```
   候選需成功率不降、碰撞／BRAKE 介入不增且無固定單側偏轉，才可考慮升預設。
7. **繞障**：機器人與物體之間放一個紙箱（偏一側），看策略是否繞過去、
   log 有無 `STALL`/`BRAKE`、`minray` 數字合理。
8. **煞停測試**：行進中把紙板伸到車前 <0.25m → log 出現 `BRAKE`、車停；以膠帶記錄
   觸發點與停止點，量出實際停止距離。首測全速 0.5 m/s 時，停止距離必須小於預先保留的
   空地安全裕度，否則降低速度／加大煞停門檻後重測。

**過關標準**
- probe 三方向反應正確；bench ≥30Hz
- 直線 2m 到點停下（停點離物體 ≈ nav-stop-dist ±0.15m）
- 繞障不碰箱子（跑 3 次至少 2 次乾淨繞過——策略本身碰撞率 0.2 左右，煞停要兜底）
- 煞停 100% 有效，且每次量得的停止距離均在預先標出的安全裕度內

**沒過**：完全不動/亂轉 → 檢查 obs（開 log 比對 dist/bearing 合理性）、control period；
行為抖 → `--control-period` 微調（**不要動 plant 參數**）。
**產出**：lidar-dir/yaw-offset/forward-offset/wz-sign 定案 → 對照表④。

> 目前此 Phase 的底盤 backend 仍是 `set_car_motion`。若要改用自製 `set_motor`，必須先通過
> `integration/SETMOTOR_ODOM_INTEGRATION.md` 的 A/B gates；不可在 Phase 5 現場臨時把
> 連續速度改成離散 forward/turn 指令。

---

## Phase 6 — 全流程整合

**目標**：偵測 → RL 導航避障 → 精對位 → 夾取 → 驗證，一條龍。

**步驟**
1. 物體放 2–3m 外、視野內，無障礙跑一次：
   ```bash
   python3 integration/nav_rl_grasp_pipeline.py --real --show \
     --cam-x <Phase3_X> --cam-y <Phase3_Y> --sign-y <1或-1> \
     --i-confirm-camera-frame
   ```
2. 加一個障礙物再跑。
3. 故意讓第一次夾失敗（物體放歪一點）驗 retry：退避→重新接近→再夾。

**過關標準**：≥2/3 端到端成功（夾起 + verify 過）；retry 邏輯運作正常。
**沒過**：分段定位——導航段掛 → 回 Phase 5；精對位/latch 偏 → 回 Phase 3；
夾取段掛 → 回 Phase 4。每段 log 都有自己的前綴（[nav-rl]/[pipeline]/[Stage]）。

---

## Phase 7 — 收尾

- [ ] PR #2 轉正式 + merge；校正後的常數 commit（訊息附量測數據）
- [ ] progress.md 補各 Phase 結果；TROUBLESHOOTING.md 補新踩的坑
- [ ] 錄一段全流程 demo 影片（之後報告/口試用）

---

## 常數更新對照表（校到哪填到哪）

| # | 校正值 | 量到的值 | 要更新的位置 |
|---|--------|---------|-------------|
| ① | 手臂相機 fx/fy/cx/cy | ✅ 919.08 / 919.41 / 212.23 / 168.42（RMS 0.495px）| **已寫入 2026-07-10**：`vision_grasp_pipeline.py` FX_ARM…、`vision_grasp_bridge.py` FX…、`arm_cam.py` FX… |
| ① | 桅相機 fx/fy/cx/cy | ✅ 544.16 / 544.82 / 316.98 / 244.79（RMS 0.374px）| **已寫入 2026-07-10**：`vision_grasp_pipeline.py` FX_REAR… |
| ② | θ_ARM / H_ARM | ✅ 36.40° / 0.332 m（回推誤差 ≤0.43cm）| **已寫入 2026-07-10**：三檔同步 |
| ② | θ_REAR / H_REAR | ✅ 16.35° / 0.503 m（回推誤差 ≤0.98cm，有效 0.7–1.5m）| **已寫入 2026-07-10**：`vision_grasp_pipeline.py` |
| ③ | CAM_TO_BASE_X/Y、SIGN_Y | ✅ +0.1639 m / +0.0331 m / −1（套用後 4 點最大誤差 X 0.39cm、Y 0.33cm） | **已寫入並實機複驗 2026-07-16**：`vision_grasp_pipeline.py` 同名常數；bridge `--cam-x/--cam-y/--sign-y` 預設值 |
| ③ | object Z | `z_offset=0.0488m` 已撤銷；瓶蓋首測 `OBJ_Z_FIXED=0.02m` | TCP 是 `arm_link5` 慣性中心而非指間中心；Phase 4 用 `--obj-z`／`--class-z` 實抓微調 |
| ④ | lidar dir / yaw offset / port | ＿＿ | `nav_rl.NavRLConfig` 預設值或 CLI |
| ④ | lidar forward offset（LiDAR 中心↔車中心，前正） | ＿＿ | `nav_rl.NavRLConfig.lidar_forward_offset_m` |
| ④ | wz_sign | ＿＿ | CLI `--wz-sign`（定案後可改 pipeline 預設） |
| ① | 畸變決策（忽略 / undistort） | ✅ 手臂 9px 不可忽略；後鏡頭 1.79px 忽略 | **已寫入 2026-07-10**：三檔的手臂鏈路都在距離模型前對 bbox 底邊中點做 `undistort_pixel()`（純數學版，與 cv2.undistortPoints 同法）；後鏡頭維持 raw |

> θ_ARM 是以「去畸變後」像素解出的（Phase 2 流程如此），所以 runtime 一定要先
> undistort 再進距離模型——三個檔案已內建，selftest 用 Phase 2 實測點驗證誤差 ≤0.33cm。

**改完常數一律重跑該 Phase 的驗證一次**，確認填對地方。
