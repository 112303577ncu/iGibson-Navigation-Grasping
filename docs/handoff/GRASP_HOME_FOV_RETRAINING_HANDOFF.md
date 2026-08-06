# X3Plus grasp-home 視野擴充與 PPO 重訓／架構評估需求書

> 文件版本：1.0  
> 日期：2026-07-16  
> 對象：持有 X3Plus 模擬環境與訓練程式碼的 AI／開發者  
> 現況：實機部署端已確認目前 grasp-home 的相機地面視野只有約 `0–7 cm`；目標是能辨識 `0–18 cm`，但尚未選定新姿態、尚未重訓、尚未完成新姿態專用的實機 homography。

## 0. 接手 AI 必須完成的任務

請不要只在部署端任意改一組伺服機角度，也不要直接拿現有 v17 權重從新姿態開始執行。需要先完成以下工程判斷與交付：

1. 在模擬中建立與實機一致的手臂相機模型，搜尋一個安全姿態，使相機能看到以夾取中心為基準、地面前向 `0–18 cm` 的區域。
2. 比較兩種架構：
   - **方案 A：新增較高的 `vision_home`，辨識並鎖定 base-frame 物體座標後，再移到舊的 `policy_grasp_home` 使用 v17。**
   - **方案 B：讓新的較高姿態同時成為 `policy_grasp_home`，並以它作為 episode 初始狀態重新訓練新 policy。**
3. 先輸出候選姿態、模擬視野、碰撞／關節裕量與可達性報告；完整轉場路徑離線驗證並建立受控 waypoints 後，候選姿態才可在實機做 FOV 驗證，確認後才進行大規模重訓。
4. 若方案 A 無法安全完成轉場，或使用者確認要讓高姿態直接成為 policy 起點，才執行方案 B 的重訓。
5. 若物理幾何無法同時滿足 `0–18 cm` 視野與可抓取需求，必須提供不可行證據、最小盲區與可達上限，不能默默把需求縮小。

使用者目前偏好的方向是 **方案 B：提高 grasp-home 並重訓**。方案 A 是必須先評估的低風險替代方案，不得在未說明差異的情況下取代使用者需求。

---

## 1. 問題摘要

### 1.1 實際觀察到的問題

手臂相機固定在機械臂上。當機械臂移到現有 PPO `grasp_home` 時：

- 相機只能看到距離「夾取 home 中心」約 `0–7 cm` 的地面範圍。
- 使用者希望相機能看到約 `0–18 cm`。
- 物體放在約 `10 cm` 時，bridge 可以打開相機，但 YOLO 無法累積 10 張有效瓶蓋偵測；主要原因是物體不在完整可視區，而不是 TCP 5555 或模型載入失敗。
- 本次對話附圖是目前 grasp-home 的 640×480 相機視角；尺規顯示視野集中在夾取中心附近，較遠區域沒有進入畫面。圖片不在 deployment repo 內，但本文件已保留量測結論；若交接系統允許，請另外附上原圖作佐證。

### 1.2 為何這是架構問題，不是單純校正問題

目前無法用 homography 或調整 `cam_x/cam_y` 修好，因為：

1. **物體沒有進入影像，就沒有像素可以校正。**
2. 相機透過 `mono_joint` 固定在 `arm_link4`；J1–J4 改變時，相機位置、俯角、yaw 與自遮擋都一起改變。
3. 現有 grasp-home 同時是 v17 PPO 的訓練初始姿態。只在實機提高姿態，第一筆 28D observation 就會落在訓練分布外，舊 policy 可能發散或撞地。
4. 舊 v17 從接近地面的姿態學到近乎水平的跨指式前伸；它沒有學過從較高位置安全下降、對位再夾取。
5. 改姿態後，舊 nav-home 外參與尚未完成的 grasp-home homography 都不能沿用。

### 1.3 先前架構為何失敗

- nav-home 能看到約 `25 cm` 前方的目標。
- 但該目標離 grasp-home runtime TCP 約 `25 cm`，超過目前實測約 `15 cm` 的可用範圍。
- 當底盤讓物體進入可抓區後，nav-home 又只能看到局部或完全看不到物體。
- 因此已把最後一次 detection/latch 改成「機械臂到 grasp-home 並 settle 後，丟棄舊封包，再接受新偵測」。
- 新發現是：現有 grasp-home 本身的 FOV 只有 `0–7 cm`，仍不足以完成 `0–18 cm` 的辨識需求。

---

## 2. 座標與名詞必須先統一

### 2.1 建議的需求座標定義

本文件中的地面位置應定義為：

- `grasp_center`：兩指預期接觸中心的固定虛擬點，而不是 `arm_link5` inertial COM。
- `D=0`：**實際執行 PPO 的 policy grasp-home** 下，`grasp_center` 投影到地面的點。方案 B 使用新的高 grasp-home；方案 A 則仍以舊 v17 policy grasp-home 為基準，不能改成 vision-home 的投影點。
- `D>0`：機器人物理前方，即 canonical root frame 的 `+X`。
- `L>0`：機器人物理左側，即 `+Y`。
- `L<0`：機器人物理右側，即 `-Y`。使用者站在機器人後方時，他的右方就是機器人物理右方，也就是 `-Y`。

「看得到 `0–18 cm`」是指地面物體中心相對 `grasp_center` 的前向距離 `D`，不是相機到物體的斜距，也不是目前程式的 `arm_link5` 3D 距離。

### 2.2 現有程式的座標陷阱

- PyBullet 載入的 URDF root 是 `base_footprint`，其 `z=0` 是地面。
- `base_link` 透過固定關節位於 `base_footprint` 上方 `0.0815 m`。
- 文件與 CLI 常把 PPO frame 簡稱為 `base_link`，但目前 PyBullet world 數值實際以 `base_footprint` root 為準。
- runtime log 的 `TCP` 是 `arm_link5` 的 `getLinkState()[0]`／inertial COM，不是真正兩指夾取中心。
- 現有名義 grasp-home 的 `arm_link5` TCP 約為：

  ```text
  [0.17190, 0.01817, 0.01122] m
  ```

- 實機 servo 同步後曾得到：

  ```text
  [0.1668, 0.0182, 0.0101] m
  ```

- 使用者實測「地面到真實夾取中心」為 `H_real=0.06 m`。這與 `arm_link5` COM 的 URDF z 不同，不能直接互相相減當作物體 z offset。

### 2.3 建議處理方式

重訓前必須做出明確決定：

1. **相容模式**：policy observation 繼續使用 `arm_link5` COM；另新增固定的 `grasp_center_link` 只供 FOV、量測與驗收使用。
2. **修正模式**：28D observation 的 TCP 改用真正 `grasp_center_link`。這會改變 observation 分布，必須建立新 model／VecNormalize 版本，部署 FK 也必須同步。

建議在**先量測／驗證完整 6D transform** `T_arm_link5_grasp_center` 後，再於 URDF 新增一個不破壞舊 link 的固定 `grasp_center_link`。目前只有 `H_real=0.06 m` 的高度資訊，不足以決定 x/y/orientation；不得靠猜測新增 link，否則 FOV、reach、reward 與 observation 都會一起偏移。

---

## 3. 目前已確認的硬體與軟體狀態

### 3.0 系統背景

| 項目 | 現況 |
|---|---|
| 機器人 | Yahboom X3Plus，麥克納姆底盤＋5-DOF 機械臂＋S6 夾爪 |
| 實機控制 | Jetson Nano 直接使用 `Rosmaster_Lib`，serial stable path `/dev/myserial`，不依賴 ROS 控手臂 |
| 模擬／策略 | PyBullet kinematics＋Stable-Baselines3 PPO |
| 相機 | ARM camera 固定在 `arm_link4`；REAR camera 固定在車體，與本問題無關 |
| policy 輸入 | 28D structured state；相機影像先由 YOLO／幾何轉成物體座標 |
| policy 輸出 | 6D normalized arm/gripper action |

### 3.1 相機與資料鏈

- ARM camera：Sonix、非 `SN0001`，640×480。
- ARM camera 的 `/dev/videoN` 曾因 USB 重連而改變；正式使用只能依 stable by-id 與當次 startup check 解析，不可把 `video0/video2` 任一編號寫死。
- startup device check 已確認兩支相機都可產生 640×480 影格。
- YOLO 模型可載入，類別為：

  ```text
  0: bottle-cap
  1: paper-ball
  ```

- 目前 blocker 是 grasp-home 的物理視野，不是相機身分、YOLO 權重載入或 TCP 5555。

### 3.2 ARM camera 內參

在相同相機、640×480、相同 crop／縮放設定下，內參可跨姿態沿用：

| 參數 | 值 |
|---|---:|
| `fx` | `919.08 px` |
| `fy` | `919.41 px` |
| `cx` | `212.23 px` |
| `cy` | `168.42 px` |
| distortion `(k1,k2,p1,p2,k3)` | `(-0.3764,-0.0748,-0.0015,0.0035,0.4793)` |
| calibration RMS | `0.495 px` |
| 影像尺寸 | `640×480` |

主點明顯不在影像中心，bbox bottom-center 的畸變可達約 `9 px`，所以新流程仍必須先 undistort，不能用理想置中相機近似。

若先忽略 distortion，直接由 raw K 的 `fy/cy` 計算，名義垂直上下邊界角約為：

```text
top    = atan((0   - cy) / fy) = -10.38°
bottom = atan((479 - cy) / fy) = +18.67°
vertical FOV ≈ 29.05°
```

這不是最終 rectified FOV。以目前 D 對中央列邊界做 inverse distortion，約為 `-10.51° / +19.56°`、合計約 `30.07°`。正式 evaluator 必須對完整影像邊界逐點 undistort／ray-cast，不能只用上下中心點或對稱 FOV。

### 3.3 現有兩組 home pose

| 用途 | API servo degrees `(S1..S6)` | 說明 |
|---|---|---|
| nav/cruise home | `(90,140,0,0,90,30)` | 導航與回收姿態，不是 PPO 起點 |
| v17 policy grasp-home | `(90,32.704,9.786,32.704,90,30)` | 目前 PPO episode 起點，S6=30 為開爪 |

現有 grasp-home 對應的 sim arm pose 約為：

```text
(0.0, -1.0, -1.4, -1.0, 0.0) rad
```

現行映射契約：

```text
arm_hw_invert = (False, False, False, False, False)
API_deg = 90 + degrees(sim_rad)
S6 open=30°, closed=180°
```

Rosmaster／App 對 S2–S4 有鏡像顯示，因此現有 grasp-home 的物理/App 角約為：

```text
(90,147.296,170.214,147.296,90,30)
```

### 3.4 現有 grasp-home 的相機幾何

URDF 中：

```text
mono_joint parent = arm_link4
origin xyz = (-0.0481, -0.05145, -0.0022)
origin rpy = (1.5708, 0, 0)
```

以現有 deployment mapping 對名義 grasp-home 做 FK，並假設 `mono_link +Z` 是光軸：

| 量 | 值 | frame／注意事項 |
|---|---:|---|
| mono origin x | `+0.2574 m` | base_link 與 base_footprint 的 x 相同 |
| mono origin y | `-0.0022 m` | 同上 |
| mono origin z | `+0.0641 m` | 相對 `base_link` |
| mono modeled ground height | `+0.1456 m` | URDF 中相對 `base_footprint`，已加 `0.0815 m`；不是實體光心量測 |
| optical yaw | 約 `180°` | 投影方向與 nav-home 相反 |
| optical pitch | 約向下 `75.2°` | 非常接近俯視 |

在「`mono_link +Z` 等於 optical axis、base_footprint `z=0` 等於模擬地面、使用 raw-K central vertical plane、忽略 distortion 與自遮擋」的假設下，把 `H≈0.1456 m`、俯角 `75.2°` 與上述名義邊界角代入平面射線模型，地面縱向 **span** 約 `7.8 cm`。若改用中央列去畸變邊界，一階值約為 `8.1 cm`。它們與實機尺規觀察的 `0–7 cm` 同量級，是合理的幾何 sanity check，但不是實體光心量測，也沒有證明這段 span 相對 `grasp_center` 的起點／終點正好為 0 與 7 cm。

若保持相同俯角與相同相機，只靠提高相機來把縱向 footprint 從約 `7.8 cm` 放大到 `18 cm`，一階粗估相機離地高度約需：

```text
raw-K central estimate:       0.1456 × 18 / 7.8 ≈ 0.335 m
undistorted-center estimate:  0.1456 × 18 / 8.1 ≈ 0.324 m
```

也就是約 `33.5 cm` 高。這只是未驗證的 rectified central-plane 一階比例估計；完整去畸變射線、左右像素位置、實體光心、ground plane 與自遮擋都會改變結果。它不是新 home 的指定高度，也不能單靠 span 證明覆蓋 `D=0–18 cm`，只用來說明「稍微提高」可能遠遠不夠。實際設計必須同時計算高度、俯角、yaw、五關節姿態、FOV polygon 相對 `grasp_center` 的位置與自遮擋。

另外，nav-home 實測外參與 URDF 仍有數公分／數度差異，所以 URDF 射線結果只能用於候選搜尋與 sanity check；最終 FOV 必須由實機尺規驗證。

### 3.5 現有可達性與安全 guard

部署端目前有：

```text
grasp_home_max_target_distance_m = 0.15
max_delta_deg = 3.0 per control step
smooth approach joint-limit margin = 1.0°
```

重要：目前 `0.15 m` guard 是「物體座標到 runtime 同步後 `arm_link5` COM 的完整 3D Euclidean 距離」，不是水平地面半徑，也不是真正指尖／夾取中心半徑。更改 home 或 end-effector reference 後，必須重做 workspace scan，不能直接把舊 `15 cm` 當成新姿態的真值。

`0–18 cm` 是**視覺需求**。實際 PPO spawn／夾取區只能使用：

```text
相機有效 FOV
∩ 新姿態實測可達區
∩ homography calibration hull
∩ 關節裕量與碰撞安全區
∩ 夾爪可接受物寬
```

相機看得到 `15–18 cm` 可作為底盤微調與「超出抓取區」判斷的緩衝，不代表手臂一定要抓得到 18 cm。

### 3.6 現有 PPO 契約

現有 v17：

- PPO model：`trained_6d_models_v17/ppo_6d_final_ready_for_real_robot.zip`
- VecNormalize：`trained_6d_models_v17/vecnormalize_6d_final.pkl`
- model 與 VecNormalize 必須成對。
- 訓練註記中的物體 spawn 為 base-frame `x=0.17–0.33 m`、`y=±0.15 m`。
- full-workspace 2000 episodes 的既有紀錄約為 `89.2%`。
- Stage 2 arm action 沒有真正訓練；實機以 script 帶著閉合夾爪回 nav-home。

Observation 是 28D structured state，不是相機影像：

```text
[0:5]   arm joints
[5]     gripper joint
[6:9]   TCP position
[9:13]  TCP quaternion
[13:16] object position
[16:19] object - TCP
[19:22] stage one-hot
[22:28] previous 6D action
```

Action 是 6D：五個 arm joints 加一個 gripper joint。

因此，相機 FOV 必須在模擬中另外驗證；PPO 本身不會從影像學會「看見」物體。視覺 bridge 先把像素轉成 canonical XY／width，Z 由獨立、版本化的 object/class reference 規則提供，policy 才使用完整座標。

目前 deployment 時序／stage 契約還包括：

| 項目 | 現值／行為 |
|---|---|
| control rate | `10 Hz` |
| Stage 0 servo target duration | `250 ms` blending |
| per-step joint delta | 最多 `3°` |
| Stage 0→1 | `distance<0.05 m`，或 grip command `>0.90`，或先明顯接近後從最小距離反彈 `>0.05 m` |
| Stage 1 | 鎖定當下 S1–S5；先以約 `600 ms` settle command 到 lock pose，再等待 `0.7 s`，之後只控制 S6 閉合 |
| width-aware close | 可依偵測寬度決定 S6；無寬度時使用既定閉合規則 |
| Stage 2 | 不信任未訓練 policy arm action；scripted 回 nav-home 並保持夾爪閉合 |

新訓練的 dynamics／state machine 必須複製這些行為，或明確升版並同步 deployment，否則模擬成功率不代表實機流程。

### 3.7 物體 Z 的現況

- 實體瓶蓋總高：`1.3 cm`。
- 幾何中心約 `0.65 cm`，但現有 policy 使用 `obj_z=0.02 m` 時才有已驗證的可行 rollout。
- 曾直接使用物理中心高度時把 S2 推近極限。

新訓練必須明確定義 object reference（底面／幾何中心／sim body COM）並同步 deployment；不可直接把 `1.3 cm / 2` 填進舊模型。

---

## 4. 現有視覺校正與 pipeline 狀態

### 4.1 nav-home 校正已完成，但不能給新姿態使用

nav-home ARM camera 目前實測常數：

```text
H = 0.332 m
theta = 36.40°
CAM_TO_BASE_X = +0.1639 m
CAM_TO_BASE_Y = +0.0331 m
SIGN_X = +1
SIGN_Y = -1
```

四點複驗誤差約在 1 cm 以內，足以證明 nav-home 模型有效。但相機移到 grasp-home 後，位置、高度、俯角與前向符號都改變，這些值不得沿用。

### 4.2 grasp-home homography 軟體已存在，但實體資料尚未完成

現有程式已具備：

- `integration/grasp_home_homography.py`
- grasp-home `u,v -> canonical base XY` normalized DLT
- bridge runtime 至少 6 點、fitted max error `<2 cm`
- pixel hull 與 base hull 雙重拒絕外插
- bbox bottom-center 與左右底角先 undistort
- 物寬由左右底角映射後距離取得
- calibration-only 目前會取 10 次成功 detection read 的中位數且不連 TCP；尚需補 unique sequence/timestamp gate，才能保證是 10 張不同的新 frame

但目前 repo **沒有**：

```text
integration/grasp_home_homography.json
```

也沒有完整 grasp-home 實測點集，因此 Phase 3B 尚未完成。現在又已確認舊 grasp-home 的 FOV 不足，所以不應繼續為舊姿態收集 homography；應先定案新 detection／policy pose。

目前 homography schema 也**沒有綁定精確 servo pose、影像尺寸或 K/D identity**，controller 會忽略 payload 的泛稱 `camera_pose`。因此在改姿態前還必須升版 calibration/runtime contract：

- JSON 記錄 `pose_id`、精確 API degrees、sim radians、解析度、camera stable identity、K/D hash、canonical root frame 與 calibration ID。
- bridge 啟動時檢查指定 pose/config 與 calibration metadata 完全相符。
- detection payload 帶 `pose_id`、`calibration_id`、capture timestamp 與 frame sequence。
- controller 解析並拒絕 pose/calibration 不符、過期或重複 sequence 的 payload。
- 若無法從實機讀回／驗證目前 servo pose，至少要有明確的狀態機證明相機已到該 pose；不能只相信檔名叫 `grasp-home`。

另外，現有 calibration-only 只是累積 10 次成功 detection read；它本身尚未驗證 sequence/timestamp 是否唯一。sequence-aware MJPEG 已避免伺服器主動重送同一 JPEG，但正式校正仍應在 bridge 加入 unique-frame gate，才能宣稱是 10 張真正不同的新 frame。

### 4.3 真實整合 pipeline 現在刻意 fail-closed

- `vision_grasp_pipeline.py --real` 目前被拒絕，因 final-align/latch 還沒有接上合格的 grasp-home homography。
- `nav_rl_grasp_pipeline.py` 的真實整合夾取也被拒絕，只允許 `--nav-only`。
- 現階段唯一設計中的 real vision grasp 路徑，是 standalone controller 在正確 pose settle 後等待 fresh socket detection，再由合格 homography bridge 單次送座標。

在新姿態、新模型與新 homography 通過前，應繼續保持 fail-closed。

---

## 5. 希望得到的最終成果

### 5.1 視覺成果

新固定辨識姿態必須達到：

1. 在 640×480 live preview 中，地面前向 `D=0–18 cm` 均落在有效視野內。
2. 不是只看到物體中心；瓶蓋完整 bbox 必須在影像內，而平面 homography 的 calibration hull 必須包含 undistorted bottom-center、bottom-left 與 bottom-right 三個地面代表點。bbox 上緣不是地面點，不應拿去做平面 hull 判定。
3. 對 `D={0,3,7,10,12,15,18} cm` 的中心線測試點，物體應完整可見。
4. 對實際可抓區，還要覆蓋左右方向。若使用者尚未另定，先以 `L=0, ±4, ±8 cm` 作候選設計網格，超出新可達區的組合可標為 detection-only 或 N/A。
5. 在每個視覺驗收點，以 `conf=0.3` 收 10 張真正不同的新 frame，建議至少 9/10 張能穩定辨識正確類別。
6. preview 必須顯示持續遞增的 frame sequence；不得用重複的 stale JPEG 假裝多幀。

若 `D=0` 因夾爪／手臂自遮擋在物理上無法完整看見，必須回報：

- 最小盲區半徑；
- 遮擋來源；
- 調整 pitch／height／wrist 後的最佳結果；
- 是否需要獨立 `vision_home` 才能滿足需求。

### 5.2 新姿態成果

候選 pose 必須同時具備：

- nav-home 到候選 pose 的無碰撞轉場；
- 相機覆蓋上述 FOV；
- 夾爪與底盤、地面、相機線材無碰撞；
- nominal home 不貼硬體關節極限，建議至少保留 `5°` 裕量並報告每關節裕量；
- 能從該 pose 到達實際抓取區，且不靠穿地或超限；
- S6 保持 `30°` 開爪；
- 重複回到 pose 時，相機外參可重現。

不只 nominal pose 要有 margin；必須報告完整 nav→candidate、candidate→pregrasp、grasp→return 軌跡的最小 joint margin、幾何 clearance、速度、預估 effort／load 與接觸 impulse。相機與線材應使用保守 proxy，而不是只檢查 arm link mesh。

候選結果必須同時提供：

```text
sim radians S1–S5
API degrees S1–S6
Rosmaster/App physical degrees S1–S6
grasp_center pose
arm_link5 pose
mono/camera pose
每關節距上下限裕量
ground-FOV polygon
可達 workspace polygon／3D set
nav-home→candidate 的轉場路徑
```

### 5.3 Policy 成果（若採方案 B）

新模型不得只是把 v17 放到新起點繼續跑。需要：

- 新版本 model，例如 `trained_6d_models_v18/...zip`；
- 與其成對的新 VecNormalize `.pkl`；
- training reset pose 與 deployment `grasp_home_deg` 完全一致；
- policy 學會從較高位置安全下降／對位，而不是只水平前伸；
- 維持 6D action 與 28D observation，除非明確版本化並同步部署；
- Stage 2 要明確選擇「繼續 scripted return」或「正式納入訓練」，不可讓未訓練 action 控制實機回收。

建議模擬驗收：

- 使用至少 3 個獨立 training seeds，分別報告平均與變異，不可只交最佳 seed；
- 在最終 deployable workspace 至少 1000 個固定 evaluation seed episode；
- v17 的 `89.2%` 來自舊 workspace，不能直接與新 workspace 平均值相比。必須額外建立兩模型都能跑的 common reference distribution 做 A/B，並在新 deployable workspace 另報新模型目標 `≥95%`；
- 空間分箱後不得只靠容易區域拉高平均。`80%` 只能當診斷下限；未達最終部署門檻的 bin 必須從 operational workspace mask 排除或重新訓練；
- ground/base/self forbidden collision 為 0；
- 另列 joint-limit、推動物體、掉落與 timeout 比率；
- 提供 D/L 距離分箱 heatmap，而不只一個平均成功率。

每個 episode 的「成功」必須用物理結果定義，不能只以 TCP 距離、Stage 轉換或 gripper action 判定。建議至少同時滿足：

1. 抓取前 `grasp_center` 的 XY、姿態與高度進入容許範圍。
2. 左右兩指都對物體形成有效接觸／約束，而不是單指把物體推走。
3. 夾爪閉合後物體沒有穿透地面、彈飛或倒出指間。
4. 物體被抬離地面至少 `3–5 cm`，維持至少 `0.5 s`。
5. scripted Stage 2 或正式訓練的 Stage 2 返回期間不掉落。
6. 全軌跡沒有 ground/base/self forbidden collision、沒有 joint-limit contact，也沒有超出設定的 effort／velocity／clearance gate。

目前部署的 Stage 0→1 邏輯包含 `dist<5 cm`、gripper command、或先接近後反彈的 trigger；Stage 1 會鎖住 S1–S5、settle 後依寬度閉合 S6；Stage 2 是 scripted return。新訓練若改動其中任何契約，必須同步修改部署狀態機與測試，而不能只改 reward。

### 5.4 實機最終成果

完成新姿態與模型後：

1. 對該固定 pose 重做至少 9 點 homography。
2. 另留至少 3 個不參與 fit 的 holdout。
3. 現有 runtime 的 fitted max-error 硬上限是 `<2 cm`，但每軸 2 cm 最壞會形成約 2.8 cm 平面誤差，對瓶蓋可能太大。最終 holdout 容許值必須由瓶蓋直徑、夾爪開口與容許偏心實測反推；第一版目標採 radial error `≤1 cm`，同時保留現有每軸 `<2 cm` 的 fail-closed ceiling，建議 fit RMSE `<1 cm`。
4. 瓶蓋物寬估計建議誤差 `≤5 mm`，最低要求 `≤1 cm`。paper-ball 的 bbox 底角不一定代表真實接觸寬度，必須依 class 建立獨立幾何／驗收規則，不能直接沿用瓶蓋公式。
5. nav→new pose 重返至少 3 次，同一 holdout 仍需維持 `<2 cm`。
6. 先做 fixed-coordinate dry rollout，確認 reach、virtual lock 與 joint margin。
7. 再做低風險中央點單次實抓，最後才擴充到網格。
8. 第一輪可用每點 3 次、至少 27 次作 smoke test，目標 `≥24/27` 且零安全碰撞；這個樣本數不足以宣稱穩定成功率。正式可靠度報告應提高每個 operational bin 的次數、提供 binomial confidence interval，且未達門檻的 bin 必須從 workspace mask 排除或重新訓練。

### 5.5 必須完成的端到端成果

最終目標不是只有 standalone 單點夾取。整合狀態機必須實作並驗證：

```text
nav-home 粗偵測／導航
→ 底盤停止且確認靜止
→ 進入 vision_home 或 new policy_grasp_home
→ settle
→ 丟棄姿態切換前的 frame／payload
→ 只接受 pose/calibration/sequence 匹配的 fresh detection
→ 檢查 FOV hull、reach、joint margin、width 與 target freshness
→ latch canonical XY/width，並依明確 z convention 取得 Z
→ 執行 policy／scripted grasp
```

若目標在 `15–18 cm` 的 detection-only margin、但在新實測可抓區外，狀態機必須：

1. 不啟動手臂 policy；
2. 若 detection pose 未被驗證可隨底盤移動，先回 nav-home；
3. 讓底盤做受控微調後完全停止；
4. 再次進入固定 detection pose、settle 並取得新的 fresh latch；
5. 直到目標進入可抓區才允許 policy。

完成此流程並通過實機測試後，才能解除 `vision_grasp_pipeline.py`／`nav_rl_grasp_pipeline.py` 目前的 real-grasp fail-closed。舊座標不得跨底盤移動沿用。

---

## 6. 必須先比較的兩種架構

### 6.1 方案 A：`vision_home` 與 `policy_grasp_home` 分離

流程：

```text
nav-home
→ 較高 vision_home settle
→ 以 vision_home 專屬 homography 取得並凍結 canonical object XY/width
→ 依版本化的 class/object reference 規則另行取得 Z
→ 安全移到舊 v17 policy_grasp_home
→ 使用凍結座標執行 v17
```

優點：

- 可能不需重訓；
- 保留已驗證的 v17 policy 起點與接近方式；
- 只要物體與底盤不動，base-frame 座標在相機移動後仍有效。

必要條件：

- vision_home 能看 `0–18 cm`；
- vision_home→old policy home 轉場不會碰到物體、地面或底盤；
- 舊 policy home 到物體仍在可達區；
- v17 必須對 vision-home 可視且舊 policy-home 可達的完整 D/L 網格做 virtual rollout；不能只用 15 cm 物理 guard 推定 policy 一定在分布內或會收斂；
- detection 必須是 canonical absolute frame，不得是相機相對距離；
- 移動後不得再使用 vision_home 的 live detection 更新 policy target。

目前 controller 會在到達 grasp-home 後丟棄移動途中／之前的 detection，所以現行程式**尚未直接支援方案 A**。若選方案 A，必須新增明確的 `vision_home → latch canonical target → policy_grasp_home` 狀態，並讓已驗證的 base-frame latch 在轉場後保留；不得沿用現在的 `discard_pending()` 時序。

目前 bridge CLI 也只有泛稱 `grasp-home/nav-home`，沒有獨立 `vision-home` pose identity；方案 A 必須同步擴充 pose-bound calibration 與 payload contract，不能只多存一份未綁角度的 JSON。

### 6.2 方案 B：新的高姿態同時是 policy grasp-home

流程：

```text
nav-home
→ new high grasp-home settle
→ 同一 pose 做 detection/homography latch
→ 新 policy 從此 pose 開始下降、對位、夾取
```

優點：

- detection pose 與 policy 初始 pose 完全一致；
- runtime 狀態機較直接；
- 不需要在 latch 後再切換相機 pose。

代價：

- 必須重新訓練；
- 新 policy 要學垂直下降與防撞；
- 新 home 可能降低水平可達距離或造成關節奇異；
- model、VecNormalize、FK reference、homography 與部署常數都需重新版本化。

### 6.3 架構決策 Gate 0

訓練 AI 第一份回覆必須是比較報告，而不是直接開始長時間訓練：

| 判斷項 | 方案 A | 方案 B |
|---|---|---|
| `0–18 cm` FOV | 量化結果 | 量化結果 |
| 可達區 | 舊 policy home 的交集 | 新 home 的交集 |
| 轉場碰撞 | vision→old policy home | nav→new home |
| 是否需要重訓 | 預期否 | 是 |
| sim-to-real 風險 | 較低／列原因 | 較高／列原因 |
| 所需程式變更 | 列出 | 列出 |

若方案 A 完整滿足功能，請把它作為推薦方案，同時保留方案 B 的候選與重訓估算，交由使用者選擇。

---

## 7. 模擬端必須執行的工作

### 7.1 核對 authoritative training repo

部署端最後記錄的訓練檔案名稱是：

```text
training/x3plus_ground_grasp_env.py
training/robot_grasp_env.py
```

其中舊 `home_arm_pose` 為：

```python
(0.0, -1.0, -1.4, -1.0, 0.0)
```

接手 AI 必須先在訓練電腦確認實際 branch／commit、reset 邏輯、reward、spawn、Stage 2 與 export script；不要只依這份部署文件猜測。最後記錄過的 training main commit 是 `391b07952431009dbb758f8aafb2d878c847b950`，但應以訓練電腦目前工作樹為準。

### 7.2 建立相機 FOV evaluator

至少要能對任意 arm pose：

1. 算出 `mono_link`／真正 optical frame 在 `base_footprint` 的 6D pose。
2. 使用 640×480 K/D 產生影像四角與 bbox sampling rays。
3. 與地面平面求交，得到 ground-FOV polygon。
4. 對手臂、夾爪、底盤做 ray／render self-occlusion 測試。
5. 在 `D/L` 網格渲染瓶蓋與 paper-ball，確認完整 bbox 與像素大小。
6. 輸出 pose、相機高度／pitch／yaw、FOV polygon、盲區與截圖。

在使用 `base_footprint z=0` 作地面前，必須確認 training loader 的 robot base pose 與實際 ground plane equation；正式相機高度應算 `optical_center_world_z - ground_plane_z_at_xy`，不可硬編碼 `0.1456 m`。

不得未驗證就假設 `mono_link +Z` 一定等於 renderer optical axis；要以已知標記或 rendered axes 做一次 convention test。

### 7.3 候選 pose 搜尋

搜尋 S1–S5，而不是只手動改一個角度。每個候選至少評分：

- `0–18 cm` longitudinal coverage；
- lateral coverage；
- full-object visibility；
- camera pixel resolution；
- ground/base/self collision；
- home joint-limit margin；
- 從 nav-home 的可行轉場；
- 到物體網格的 IK／policy reachable ratio；
- 接近奇異點程度；
- 相機線材與實體結構的額外保守空間。

請保留至少 3 個 Pareto candidates，不要只交一組角度。

### 7.4 實機 pose-only Gate：先驗視野、後訓練

候選角度先在部署端 dry-run：

```bash
python3 grasp/x3plus_real_grasp.py \
  --pose-only grasp-home \
  --grasp-home-deg S1,S2,S3,S4,S5,30
```

此 dry-run **只會印出終點命令**，不會模擬 2 秒 direct servo move 的整條路徑，也不會做碰撞檢查；`pose-only` 的 direct move 還會繞過一般每步 `3°` rate limiter。因此在任何 `--real` 之前，必須先在模擬中對 nav-home→candidate trajectory 密集取樣做 self/base/ground collision check，並在需要時產生已驗證的分段 waypoints。不能把「dry-run 印出的終點角度合理」當成路徑安全證據。

現有 `pose-only` 不會先把手臂帶回已知 nav-home；上一個測試還會讓伺服機停在上一候選。因此每次實機驗證都必須記錄真實起點，離線檢查「該起點→候選」的完整軌跡。優先做法是先實作可限制速度、按驗證 waypoints 執行且可中止的專用 pose validator。

只有在「起點已知，而且現有單段 2000 ms joint interpolation 本身已被逐段證明安全」時，才可由人在旁、清空工作區並準備中止，使用現有命令：

```bash
python3 grasp/x3plus_real_grasp.py \
  --real \
  --pose-only grasp-home \
  --grasp-home-deg S1,S2,S3,S4,S5,30
```

用 stable ARM-camera by-id 開 preview，以尺規測 `0–18 cm`，記錄：

```text
API/App joint angles
grasp-center height
camera height
最小／最大可見 D
左右可見 L
遮擋區
frame screenshots
```

只有實機 FOV 過關的 pose 才能成為 training reset pose，避免先訓完才發現 URDF 相機外參與實體不一致。

### 7.5 若採方案 B：訓練環境修改

必須同步修改：

1. reset 的 `home_arm_pose`；
2. observation 初始 joint/TCP/quaternion；
3. object spawn distribution；
4. reward 與 curriculum，使策略學會從較高位置下降；
5. collision／joint-limit／ground contact；
6. evaluation maps；
7. export 的 model／VecNormalize；
8. deployment `grasp_home_deg` 與模型路徑；
9. 若 end-effector reference 改成 `grasp_center_link`，同步所有相對位置與 stage thresholds。

新的 spawn 不應再只是舊的 absolute x/y 矩形。應由下列交集產生：

```text
new pose FOV hull
∩ new pose reachable workspace
∩ joint-margin-safe region
∩ collision-free region
∩ gripper width/class constraints
```

建議以 `D/L` 分層 curriculum：

1. 中央、近距、較容易的下降對位；
2. 擴充 D；
3. 擴充 lateral；
4. 加入 home joint noise、object pose/size/friction、servo lag 與座標誤差；
5. 全 deployable workspace mixed sampling；
6. 固定 seed 全網格 evaluation。

reward 至少應處理：

- 到 pre-grasp waypoint 的距離；
- 夾爪姿態／朝向；
- 安全下降與對位；
- 撞地、自碰撞、撞底盤；
- 關節極限與過大 action；
- 過早碰撞／推走物體；
- 進入指間、閉爪與實際 grasp；
- 成功後的 lift 或 scripted Stage 2 contract。

### 7.6 sim-to-real randomization

至少評估：

- reset joint 誤差與回差；
- API command 與實際 physical joint 的偏差；
- 100–250 ms servo lag；
- 每 step `3°` rate limit；
- arm link／camera mount 小幅外參誤差；
- grasp-center 與 arm_link5 COM offset；
- object XY/Z、直徑、高度、摩擦與質量；
- detection XY 誤差與 hold latency；
- ground height 與 base tilt。

policy deployment 現在主要根據 rate-limited commanded state 建 observation，而不是每 step 的完整實測 feedback。模擬評估要包含這種 open-loop lag，不可只測理想 joint state。

---

## 8. 不可破壞的部署契約與安全限制

除非新版本明確重新設計、重訓並同步部署，以下不得任意改掉：

- nav-home 與 policy/detection pose 的角色必須分開命名。
- `arm_hw_invert=(False,False,False,False,False)`。
- `API_deg=90+sim_deg` 的 deployment mapping。
- S6 `30°=open`、`180°=closed`。
- `max_delta_deg=3.0`；不可為了看起來收斂快而移除。
- model 與 VecNormalize 必須成對、版本化。
- real mode 無 fresh detection 時必須拒動，不得使用 default `[0.25,0,0.02]`。
- 新 pose 必須使用自己的 homography；不得沿用 nav-home H/theta/offset。
- bbox 任一底角在 calibration hull 外時必須拒絕。
- 新 workspace scan 完成前，保留或縮緊現有 reach guard，不得擴大。
- smooth virtual rollout 未鎖定或 lock pose 進入 joint-limit margin 時，實機必須拒動。
- 現有 v17 與其 VecNormalize 必須保留可回滾，不能覆寫。
- 現有 integrated real pipelines 在新 final-align/latch 接妥前繼續 fail-closed。

尤其禁止：只改 deployment `grasp_home_deg`，仍載入 v17，然後直接做 `--real` PPO 測試。

---

## 9. 建議的執行 Gate

| Gate | 工作 | 通過條件 |
|---|---|---|
| 0 | 方案 A/B 架構比較 | 有量化 FOV、reach、collision、改動量與推薦 |
| 1 | 模擬候選 pose | 至少 3 個候選；ground FOV 與 workspace 報告完整 |
| 2 | 離線軌跡＋受控實機 pose 驗證 | 起點明確、全路徑 clearance/joint margin 過關；尺規確認 0–18 cm；frame 持續更新 |
| 3 | 鎖定 canonical frames | `base_footprint/base_link/grasp_center/object-z` 定義寫入程式與文件 |
| 4 | 重訓（只在方案 B） | 新 model + 新 VecNormalize；固定 seed grid 過關 |
| 5 | deployment dry-run | 新 pose、mapping、28D/6D、reach、joint margins 全部一致 |
| 6 | 新 pose homography | metadata 綁定 pose/K/D/解析度；9+ fit 點、3+ holdout；誤差符合瓶蓋容許偏心；bbox 底邊三點在 hull |
| 7 | 單點 real grasp | 中央約 10 cm，先 dry rollout 再一次低速真夾 |
| 8 | 網格 real grasp | 覆蓋近／中／遠與左右；零安全碰撞；結果報告完成 |
| 9 | integrated end-to-end | nav→stop→pose→settle→fresh latch→reach gate→grasp 完成；超距時回 nav 微調並重新 latch |

任何 Gate 未過，不得跳到後續 real grasp。

---

## 10. 訓練 AI 最後必須交付的檔案／資訊

### 10.1 架構與姿態

- 方案 A/B 比較報告。
- 至少 3 組候選 pose 表格。
- 最終選定 pose 的 sim rad、API deg、App physical deg。
- `grasp_center`、`arm_link5`、camera 的 FK pose。
- camera FOV polygon、可達 workspace 與兩者交集圖。
- nav-home→候選 pose 的碰撞檢查與轉場軌跡。

### 10.2 若重新訓練

- 新 training source diff／commit。
- 完整 training config 與 seed。
- 新 PPO `.zip`。
- 成對的新 VecNormalize `.pkl`。
- train/eval curves。
- 固定 seed 結果 CSV。
- D/L success heatmap。
- collision、joint-limit、push、drop、timeout 統計。
- 與 v17 的同分布比較。

### 10.3 部署交接

- 新 `grasp_home_deg` 或新增的 `vision_home_deg`。
- model／VecNormalize 路徑修改。
- observation／action contract；若仍為 28D/6D，要明確確認。
- object reference 與 z convention。
- reach guard 的新量測與定義。
- 新姿態的 calibration instructions。
- pose-bound calibration schema、payload identity／sequence 檢查與 controller 驗證規則。
- nav→stop→detection pose→fresh latch→grasp 的 integrated state machine 與測試。
- rollback 到 v17 的方法。
- 不可直接執行 real PPO 的剩餘 blocker 清單。

---

## 11. 尚待確認但不可忽略的項目

以下沒有足夠實機證據，接手 AI 不得自行當成已確認：

1. `0–18 cm` 是否要求每個距離都可抓，或 `15–18 cm` 只需可辨識。本文暫定後者可作 detection margin，真正抓取以新 workspace scan 為準。
2. lateral 硬需求尚未由使用者指定；本文以 `±8 cm` 作第一版設計網格，不代表最終值。
3. exact `D=0` 可能被夾爪自遮擋；若不可行要量化最小盲區。
4. `mono_link +Z` 與實體／renderer optical axis 是否完全一致。
5. URDF camera mount 與實體 mount 的位置／角度誤差。
6. 真正抓取中心相對 `arm_link5` COM 的固定 transform。
7. 新 home 的實體可達半徑；舊 `15 cm` 不可直接沿用。
8. 新 policy 的 object z convention。
9. 訓練電腦目前 branch 是否仍與最後記錄 commit 一致。

---

## 12. 接手 AI 可直接採用的工作指令

```text
請先閱讀本文件與實際 training repo。不要先訓練，也不要直接改部署角度。

第一階段請：
1. 核對目前訓練環境的 home_arm_pose、28D observation、6D action、spawn、reward、Stage 2、URDF 與 camera frame。
2. 建立可對任意 S1–S5 pose 計算／渲染 640×480 ARM-camera ground FOV、自遮擋與物體 bbox 的 evaluator。
3. 以 grasp_center 的地面前向 D=0–18cm、左右候選 L=0/±4/±8cm 為視覺目標，聯合搜尋高度、俯角與五關節姿態。
4. 對每個候選計算 joint margin、nav-home 轉場碰撞、reachable workspace，以及 FOV∩reach 的 deployable region。
5. 至少交付三個 Pareto candidates，並比較：
   A) high vision_home latch → old v17 policy home；
   B) high pose 直接成為新 policy home 並重訓。
6. 先提供完整離線轉場驗證、受控 waypoints／專用 validator 與實機 FOV 驗證步驟。實機尺規確認 FOV 後才開始方案 B 重訓；不要把現有 pose-only dry-run 當成路徑安全檢查。

若採方案 B，請輸出全新的 model + VecNormalize 配對，保留 v17，並提供固定 seed 網格、分箱成功率、碰撞與關節極限報告。不要用 v17 從新高姿態直接執行。
```

---

## 13. 相關部署端來源

| 來源 | 目前用途 |
|---|---|
| `grasp/x3plus_real_grasp.py` | home poses、PPO/FK、28D/6D contract、reach/safety、pose-only |
| `grasp/x3plus/yahboomcar.urdf` | arm、camera mount、base_footprint/base_link、joint limits |
| `integration/verify_camera_grasp_frame.py` | nav/grasp camera FK sanity check |
| `integration/vision_grasp_bridge.py` | intrinsics、undistort、grasp-home homography bridge |
| `integration/grasp_home_homography.py` | pixel→canonical XY calibration solver |
| `../calibration/CALIBRATION_PLAN.md` | Phase 3B/4 gate 與 holdout 需求 |
| `TROUBLESHOOTING.md` | v17 OOD、視覺／reach 架構問題與 Z 陷阱 |
| `../calibration/arm_pose.md` | 兩種 home 的歷史設計；部分舊 latch 敘述應以目前程式為準 |
| 本次對話的尺規照片 | 不在 repo；本文件已記錄 `0–7 cm` 結論，若交接介面允許請另附原始圖片 |

本文件的核心完成標準不是「找到一組看起來比較高的角度」，而是：**相機實機確實覆蓋 `0–18 cm`、姿態與轉場安全、policy 起點與訓練一致、可達區量化、校正重做、model/VecNormalize 成對，且在 dry-run 與固定網格驗收後才允許 real grasp。**
