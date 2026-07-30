# X3Plus 視覺辨識 → 夾取 整合說明

把 YOLOv11 垃圾辨識接到 PPO 夾取部署。提供三種模式：

- **模式 A（推薦・自走全流程）** `integration/vision_grasp_pipeline.py`
  單一程式統一控制：辨識 → 底盤導航靠近 → 到範圍 → 手臂夾取 → 失敗重試（≤3）。
- **模式 B（分離式・除錯用）** `integration/vision_grasp_bridge.py`
  只做「辨識→TCP 5555 送座標+寬度」，底盤導航需另外處理。
- **模式 C（RL LiDAR 導航＋夾取）** `integration/nav_rl_grasp_pipeline.py`
  YDLIDAR TG30 由 ROS 發佈 `/scan`，Python 3.8 pipeline 經 rosbridge/roslibpy 取得
  48 束避障輸入；底盤與手臂仍共用同一 Rosmaster handle。

```
模式 A：後相機/手臂相機 ─YOLO─▶ 決策 ─set_car_motion─▶ 麥輪靠近
        ─到 handoff 距離─▶ 鎖定 {x,y,z,w} ─obj_provider─▶ PPO 手臂夾取 ─驗證/重試
模式 B：手臂相機 ─YOLO─▶ {x,y,z,w} ─TCP 5555─▶ x3plus_real_grasp.py(--socket)
模式 C：TG30 ─/scan─▶ RL 導航 ─▶ 相機精對位 ─▶ PPO 手臂夾取
```

專案分成三個資料夾：

| 資料夾 | 角色 | 說明 |
|--------|------|------|
| `grasp/` | 夾取 | PPO 夾取部署（寬度控夾爪 `--width-grip`、物體鎖定 `--latch-obj`、`obj_provider` 注入） |
| `detection/` | 辨識 | YOLOv11 模型與相機辨識/校正/除錯腳本 |
| `integration/` | 整合 | 自走 pipeline（模式 A）＋ TCP 橋接（模式 B） |

---

## 0. 模式 A：自走全流程（vision_grasp_pipeline.py）

> 單一程式持有 Rosmaster，同時驅動麥輪與手臂。**執行前確認 Jetson 沒有跑 port 7000
> motor server 或 ROS 底盤 driver**，否則會搶序列埠。

```bash
# 純邏輯自測（不需相機/硬體/torch，可在開發機跑）
python3 integration/vision_grasp_pipeline.py --selftest

# 乾跑（不驅動硬體，印出狀態機與會送出的 set_car_motion；需相機串流）
python3 integration/vision_grasp_pipeline.py

# 正式自走夾取
python3 integration/vision_grasp_pipeline.py --real --show \
  --i-confirm-camera-frame
```

流程對應使用者 5 步：1 辨識 → 2 底盤前進 → 3 到 handoff 距離(預設 0.24m) →
4 手臂依鎖定座標+寬度夾取 → 5 失敗(原位置仍辨識到)則退回重試，最多 `--max-retries`(預設 3) 次。

主要參數：`--real`、`--show`、`--max-retries`、`--max-steps`、`--port`、`--handoff-dist`、`--selftest`。
座標/相機校正常數在檔案頂部（`CAM_TO_BASE_X/Y`、`SIGN_Y`、`OBJ_Z_FIXED`、`THETA_ARM/H_ARM`、`KX/KZ` 等）。

> ⚠️ 導航決策/幾何由 `detection/rear_nav/rear_to_arm_blind_handoff.py` 移植，但 actuator 由
> port 7000 motor server 改成 `set_car_motion`，速度/轉向門檻可能需**重新微調**；
> 且 `set_car_motion` 的 v_z 正負(左右轉)需實機確認。

---

## 模式 B：TCP 橋接（vision_grasp_bridge.py）

## 1. 端到端開啟流程

> 三步都在 Jetson 上、`grasp_venv` 已啟用的前提下執行。

1. **啟動手臂相機串流**（Jetson，ROS web_video_server，預設 8080）
   - 確認 `http://127.0.0.1:8080/stream?topic=/arm_cam/image_raw` 可開。

2. **終端機 A — 夾取端**（監聽 TCP 5555）
   ```bash
   cd grasp
   python3 x3plus_real_grasp.py --real --socket --width-grip --latch-obj \
     --i-confirm-external-frame
   ```
   - `--socket`：從 5555 接收 `{x,y,z,w}`。
   - `--width-grip`：啟用「依寬度決定夾爪閉合角」；不加則一律全閉 180°（行為同改寫前）。
   - `--latch-obj`：**強烈建議開啟**。手臂相機（URDF `mono_link`）裝在 `arm_link4`，會隨手臂移動，
     `arm_cam` 的固定高度/俯仰角只在 home 姿勢成立。開啟後會在 home 姿勢**擷取一次** obj 座標+寬度並
     凍結整輪，避免手臂一動後的失真偵測干擾 policy。等候新偵測逾時（`latch_wait_sec`，預設 5 秒）會退回預設座標並印警告。
   - 第一次務必先不加 `--real` 做 dry-run，目視確認角度合理。

3. **終端機 B — 辨識+整合端**
   ```bash
   python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --show
   ```
   - `--show`：顯示標註視窗（按 `q` 關閉）。
   - 正式跑之前，先用 `--once` 送一筆、核對印出的 x/y/z/w 是否合理：
     ```bash
     python3 integration/vision_grasp_bridge.py --host 127.0.0.1 --once --show
     ```

---

## 2. 各檔案用途

### `integration/`（整合，本次新增）
| 檔案 | 用途 |
|------|------|
| `vision_grasp_pipeline.py` | ★模式 A 主程式。統一控制：雙相機導航(set_car_motion) → handoff → PPO 夾取(obj_provider) → 驗證/重試。含 `--selftest` 純邏輯自測。 |
| `vision_grasp_bridge.py` | 模式 B。讀 `detection/models/best.pt` + 手臂相機，將 bbox 換算成 `{x,y,z,w}` 並透過 TCP 5555 送給夾取端（每筆開新連線、送完即關，符合夾取端 `DetectionReceiver` 行為）。 |
| `nav_rl.py` | 模式 C 導航核心。ROS `/scan`／舊 RPLidar backend、55D obs、48 rays、stale 停車、probe/bench/selftest。 |
| `nav_rl_grasp_pipeline.py` | 模式 C 全流程：RL 避障導航→相機精對位→PPO 夾取；實機預設 `--lidar-backend ros`。 |
| `README.md` | 本說明文件。 |

`vision_grasp_bridge.py` 主要參數：

| 參數 | 預設 | 說明 |
|------|------|------|
| `--host` / `--port` | `127.0.0.1` / `5555` | 夾取端位址 |
| `--stream` | arm_cam HTTP 串流 | 相機來源；給數字（如 `0`）則當 USB webcam 索引 |
| `--conf` | `0.3` | YOLO 信心閾值 |
| `--rate` | `0.3` | 每筆送出最小間隔（秒） |
| `--once` | 關 | 送出第一筆有效偵測後結束（測試用） |
| `--show` | 關 | 顯示標註視窗 |
| `--cam-x` / `--cam-y` | `0.1639` / `0.0331` | 相機原點相對手臂基座的前向/左右偏移（Phase 3 實測） |
| `--obj-z` | `0.02` | 物體在夾取座標系的高度（公尺） |
| `--sign-y` | `-1.0` | 相機左右偏移 → 夾取 +Y 的方向（Phase 3 實測） |

### `detection/`（辨識）
| 檔案 | 主線? | 用途 |
|------|:----:|------|
| `arm_cam.py` | ★ | 手臂相機 + YOLO，把 bbox 換算成前向距離與左右偏移（無 ROS）。`vision_grasp_bridge.py` 的幾何即源自此檔。 |
| `arm_center_setmotor.py` |  | 手臂相機對中（馬達 socket 狀態機，使用 box 寬度）。 |
| `nav_arm_cam.py` |  | 手臂相機 + ROS `cmd_vel` 驅動底盤靠近。 |
| `models/best.pt` | ★ | **正式模型**（來源 `runs/detect/train5`）。類別：`bottle-cap`、`paper-ball`。 |
| `models/yolo11n.pt` |  | YOLOv11 基底模型（備用）。 |
| `models/data.yaml` | ★ | 類別定義。 |
| `models/README.roboflow.txt` |  | Roboflow 資料集來源說明。 |
| `calibration/` |  | 相機/底盤校正腳本與量測資料（見下）。 |
| `debug_tools/` |  | 測試/擷取/雜項腳本（見下）。 |
| `rear_nav/` |  | 後相機導航子專案（與夾取無關，僅歸檔保存）。 |

`detection/calibration/`：
`calibrate_arm_camera_theta.py`（★校手臂相機俯仰角 `FIXED_THETA`）、`calibrate_rearcam.py`、
`calibrate_mecanum_setmotor.py`、`calibrate_vy_only.py`、`calibrate_wx_only.py`，
以及量測資料 `*_calibration_points.csv`、`action_speed_calibration_*`、`wz_90deg_calibration_result.json`、`Vx.csv`、`Wz.csv`。

`detection/debug_tools/`：
`yolo_test.py`（純 YOLO 檢視器，驗證模型與類別）、`detect_video.py`（對影片跑 YOLO 存檔）、
`multiple_ball_detect.py`、`armcam_photoshot.py`、`rear_cam_autophoto.py`、`snapshot_sim.py`、
`check_rl_model.py`、`train_yolo.py`、`test.py`、`keyboard_*_test.py`。

### `grasp/`（夾取）
| 檔案 | 用途 |
|------|------|
| `x3plus_real_grasp.py` | ★主部署腳本。本次新增 `--width-grip`（寬度→夾爪閉合角）與 `--latch-obj`（home 姿勢擷取一次並凍結 obj 座標/寬度，解手臂相機移動問題）。 |
| `x3plus_deploy_bridge.py` | action→伺服角度工具類（獨立 util）。 |
| `trained_6d_models_v17/` | PPO 模型 `.zip` + VecNormalize `.pkl`。 |
| `x3plus/` | FK 用 URDF 與 meshes。 |
| `fk_test.py` / `servo_test.py` / `workspace_scan.py` / `joint_direction_calibration.py` | 測試/校正小工具。 |
| `Rosmaster_Lib_reference.py` | 硬體驅動 API 參考（實機需 `cp` 真正的 `Rosmaster_Lib`）。 |
| `requirements_jetson.txt` | 夾取端相依套件。 |

---

## 3. 模型說明
- 正式使用：`detection/models/best.pt`（train5）。其他複製進來的腳本原本指向 `train6`/`train7`
  或外部 `v8i(0517)` 模型（**未隨附**），若要跑那些腳本需自行修改其 `MODEL_PATH` 或補上對應權重。
- 已修正為相對路徑、可直接執行：`detection/arm_cam.py`、`detection/debug_tools/detect_video.py`。
- 驗證模型：`python -c "from ultralytics import YOLO; print(YOLO('detection/models/best.pt').names)"`
  → 應印出 `{0: 'bottle-cap', 1: 'paper-ball'}`。

---

## 4. 校正清單（接實機前務必做）

1. **相機俯仰角 `FIXED_THETA` / 內參**：用 `detection/calibration/calibrate_arm_camera_theta.py`
   以已知距離反推，更新 `arm_cam.py` 與 `integration/vision_grasp_bridge.py` 頂部常數。
2. **相機↔手臂基座偏移 `--cam-x` / `--cam-y`**：Phase 3 已於 2026-07-16
   以正前、左、右、遠方四點解得 `0.1639 / 0.0331 m`。
3. **左右方向 `--sign-y`**：Phase 3 已確認為 `-1`；物理右側對應 base 負 Y。
4. **物體高度 `--obj-z`**：依實際物體擺放高度設定（地面物約 0.02）。
5. **寬度→夾爪角**：`grasp/x3plus_real_grasp.py` 的 `DeployConfig` 內
   `grip_max_object_width_m`（最寬可夾物寬度）與 `grip_min_close_deg`（對應的閉合角）依夾爪實測調整。
   公式：`close_deg = 180 − (w / max_w) × (180 − min_close)`（窄物趨近全閉 180°，寬物提早停）。

---

## 5. 安全提醒
- 首次接伺服機前，夾取端先跑 dry-run（不加 `--real`），目視確認角度合理。
- 橋接端先用 `--once` 確認座標/寬度數值正確，再連續送。
- 夾取端 `max_delta_deg=3.0` 每步限動 3°；`--width-grip` 不影響此安全限制。
