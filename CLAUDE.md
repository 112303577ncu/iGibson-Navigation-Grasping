# X3Plus 專題 — Sim-to-Real 夾取部署

## 專題概述
將 PyBullet + PPO 訓練的 6D 夾取策略，部署到 Yahboom X3Plus 實體機器人。

- **硬體**：X3Plus（麥克納姆輪底盤 + 5-DOF 機械臂），Rosmaster 擴充板，Jetson Nano
- **框架**：Stable-Baselines3 PPO + PyBullet（FK only）
- **驅動**：Rosmaster_Lib（需本地化到 `grasp/` 目錄）
- **視覺**：YOLOv11（Ultralytics）手臂相機辨識 → 透過 TCP 5555 送物體座標+寬度給夾取端

---

## 檔案結構（三資料夾：夾取 / 辨識 / 整合）

專案根目錄分成三個資料夾，另保留 `CLAUDE.md`、`AGENTS.md`、`progress.md`、
`TROUBLESHOOTING.md`（問題與解決紀錄：症狀→原因→解法→教訓）、`INDEX.md`、
`arm_pose.md`、`JETSON_DRYRUN_CHECKLIST.md`。

### `grasp/`（夾取）
| 檔案 | 說明 |
|------|------|
| `x3plus_real_grasp.py` | 主部署腳本（無 ROS，直接在 Jetson 執行） |
| `x3plus_deploy_bridge.py` | 工具類，正規化 action → 伺服機角度 |
| `trained_6d_models_v17/*.zip` | 訓練好的 PPO 模型 |
| `trained_6d_models_v17/*.pkl` | VecNormalize 統計（觀測正規化） |
| `x3plus/yahboomcar.urdf` + `meshes/` | PyBullet FK 用的 URDF 與模型 |
| `Rosmaster_Lib/` | 硬體驅動（本地化到此，見下方指令） |
| `fk_test.py` / `servo_test.py` / `workspace_scan.py` / `joint_direction_calibration.py` | 測試/校正小工具 |

### `detection/`（辨識）
| 檔案 | 說明 |
|------|------|
| `arm_cam.py` | ★手臂相機 + YOLO，bbox → 前向距離+左右偏移（橋接幾何來源） |
| `models/best.pt` | ★正式 YOLOv11 模型（train5），類別 `bottle-cap`/`paper-ball` |
| `models/data.yaml` / `yolo11n.pt` | 類別定義 / 基底模型 |
| `calibration/` | 相機/底盤校正腳本與量測資料（含 `calibrate_arm_camera_theta.py`） |
| `debug_tools/` | `yolo_test.py`、`detect_video.py` 等測試/擷取工具 |
| `rear_nav/` | 後相機導航子專案（與夾取無關，僅歸檔） |
| `requirements_detection.txt` | 辨識端相依（ultralytics, opencv-python） |

### `integration/`（整合）
| 檔案 | 說明 |
|------|------|
| `vision_grasp_pipeline.py` | ★模式A 自走全流程：雙相機導航(set_car_motion)→handoff→PPO 夾取(obj_provider)→驗證/重試(≤3)。含 `--selftest` |
| `vision_grasp_bridge.py` | 模式B（除錯）：辨識→算 x/y/z/寬度→TCP 5555 送夾取端 |
| `nav_rl.py` + `nav_rl_grasp_pipeline.py` | 模式C：RL 導航避障（PPO+48束LiDAR，訓練 plant 復刻+幾何煞停）→精對位→夾取，見 `NAV_RL.md` |
| `nav_best_model/` | 導航 PPO 權重（best + checkpoint 281440，各配 vecnorm pkl，來源 igibson_x3_test） |
| `README.md` | 兩種模式開啟流程、各檔用途、校正清單 |

---

## 環境設定（在 Jetson Nano 執行一次）

```bash
# 步驟 1：本地化驅動到 grasp/（解決 Python 3.8 找不到 Python 3.6 驅動的問題）
cd ~/Documents/deploy_jetson2/grasp
cp -r /usr/local/lib/python3.6/dist-packages/Rosmaster_Lib .

# 步驟 2：確認 Python 3.8 虛擬環境已啟用
source ~/grasp_venv/bin/activate

# 步驟 3：安裝辨識端相依（首次）
pip install -r ~/Documents/deploy_jetson2/detection/requirements_detection.txt
```

---

## 部署指令

夾取端在 `grasp/` 執行：

```bash
cd grasp

# 空跑測試（不驅動伺服機，確認角度輸出合理）
python3 x3plus_real_grasp.py

# 實際執行
python3 x3plus_real_grasp.py --real

# 實際執行 + 外部視覺偵測（TCP port 5555）
# 注意：--real --socket 強制要求 --latch-obj 與 --i-confirm-external-frame
#（確認送來的 XYZ 已校正到 PPO/URDF base_link 座標系）
python3 x3plus_real_grasp.py --real --socket --latch-obj --i-confirm-external-frame

# 完整視覺夾取（建議）：socket + 寬度控夾爪 + home 鎖定物體
python3 x3plus_real_grasp.py --real --socket --width-grip --latch-obj --i-confirm-external-frame

# 自訂物體位置（公尺，無視覺時）
python3 x3plus_real_grasp.py --real --obj-x 0.30 --obj-y 0.05 --obj-z 0.02
```

### 模式 A：自走全流程（推薦，單一程式控車+手臂）

```bash
# 純邏輯自測（免相機/硬體/torch，可在開發機跑）
python3 integration/vision_grasp_pipeline.py --selftest
# 乾跑（不驅動硬體；需相機串流）
python3 integration/vision_grasp_pipeline.py
# 正式自走夾取（偵測→導航→夾取→失敗重試≤3）
# --real 需 Phase 3 校正值與確認旗標，否則會拒絕啟動
python3 integration/vision_grasp_pipeline.py --real --show   --cam-x <Phase3_X> --cam-y <Phase3_Y> --sign-y <1或-1> --i-confirm-camera-frame
```
⚠️ 執行前確認 Jetson **沒有跑 port 7000 motor server 或 ROS 底盤 driver**（會搶 Rosmaster 序列埠）。

### 模式 B：TCP 橋接（除錯用，底盤需另外處理）

夾取端用上面的 `--socket`；辨識+整合端（另一終端機）：

```bash
# 先 --once 核對座標/寬度合理，再連續送
python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --once --show
python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --show
```

**旗標說明**
- `--socket`：從 TCP 5555 接收 `{x,y,z,w}`（公尺）。
- `--width-grip`：依物體寬度決定 Stage 1 夾爪閉合角；不加則一律全閉 180°（向後相容）。
- `--latch-obj`：在 home 姿勢擷取一次 obj 座標+寬度並凍結整輪。**手臂相機裝在 arm_link4 會隨手臂移動**，
  固定高度/俯仰角只在 home 成立，故強烈建議開啟以免移動後失真偵測干擾 policy。

---

## Sim-to-Real 關節映射

**重要**：Rosmaster_Lib API 對 S2/S3/S4 有內部鏡像（API = 180 − 物理角度）。
下表的 hw 值指 **API 值**（程式碼送出的值），非 App 顯示的物理值。

| 伺服機 | 方向 | hw(API) 公式 | 狀態 |
|--------|------|-------------|------|
| S1 | 正向 | hw = 90 + sim_deg | 已確認（無鏡像） |
| S2 | 正向 | hw = 90 + sim_deg | 已確認（API 鏡像抵消原 invert） |
| S3 | 正向 | hw = 90 + sim_deg | 已確認（同上） |
| S4 | 正向 | hw = 90 + sim_deg | 已確認（同上） |
| S5 | 正向 | hw = 90 + sim_deg（0~270°）| 已確認（無鏡像） |
| S6 夾爪 | 特殊 | open=30°, closed=180° | 已確認（2026-05-22）；`--width-grip` 時閉合角依寬度落在 120~180° |

---

## 觀測空間（28D）

```
[0:5]   臂關節角度（sim rad）
[5]     夾爪角度（sim rad）
[6:9]   TCP 位置（m）
[9:13]  TCP 四元數（x, y, z, w）
[13:16] 物體位置（m）
[16:19] 相對位置 = obj − tcp
[19:22] Stage one-hot [s0, s1, s2]
[22:28] 上一步 action（6D）
```

---

## 三段式控制邏輯

| Stage | 觸發條件 | 行為 |
|-------|----------|------|
| 0 | 初始 | RL 輸出對位，夾爪張開 |
| 1 | dist < 5 cm（`stage0_dist_threshold`）或 grip_cmd > 0.90（`grip_close_threshold`）或 diverging | 鎖定手臂、S6 限速閉合至目標角（`--width-grip` 時依寬度，否則 180°），到位後轉 Stage 2 |
| 2 | Stage 1 完成後 | 維持夾爪閉合角，回 home |

---

## Rosmaster_Lib API（與舊版 Arm_Lib 對照）

| 動作 | 舊版 Arm_Lib | 新版 Rosmaster_Lib |
|------|-------------|-------------------|
| 初始化 | `Arm_Lib.Arm_Device()` | `Rosmaster(); create_receive_threading()` |
| 送角度 | `Arm_serial_servo_write6_array(s1..s6, t)` | `set_uart_servo_angle_array(angle_s=[...], run_time=t)` |
| 讀角度 | `Arm_serial_servo_read(i)` | `get_uart_servo_angle(i)` |

---

## 工作流程（放程式碼到此資料夾後）

當使用者放入 `.py` 檔案，Claude 應：
1. 確認使用 Rosmaster_Lib（不是 Arm_Lib）
2. 確認觀測空間 28D，動作空間 6D
3. 確認 VecNormalize 載入時 `training=False`、`norm_reward=False`
4. 直接修改並回報差異

---

## 注意事項
- 首次連接伺服機前，務必先跑一次 dry-run，目視確認角度輸出合理
- arm_hw_invert = (False, False, False, False, False) — API 鏡像（S2/S3/S4）已抵消原 invert，全部改為 False
- 安全限制：`max_delta_deg=3.0`，每步最多動 3°，防止暴衝
- `stage0_dist_threshold = 0.05m`；備用觸發：dist 從最小值回升 > 0.05m 也觸發 Stage 1
- 若 Stage 1 觸發但夾爪與物體相差甚遠，需校正 obj 位置或 FK 偏移量
- **物體座標必須在 base/URDF 座標系**（與 FK 算出的 TCP 同框）；橋接端的相機距離→base 映射需校正
  `--cam-x/--cam-y/--sign-y/--obj-z`，詳見 `integration/README.md` 校正清單
- **手臂相機（URDF `mono_link`）固定在 `arm_link4`，會隨手臂移動**，`arm_cam` 固定高度/俯仰角只在 home 成立 →
  視覺夾取務必加 `--latch-obj`（home 鎖定一次）
- `--width-grip` 寬度→夾爪角公式：`close = 180 − (w/grip_max_object_width_m)×(180 − grip_min_close_deg)`
  （預設 `grip_max_object_width_m=0.06`、`grip_min_close_deg=120`，需依夾爪實測校正）
