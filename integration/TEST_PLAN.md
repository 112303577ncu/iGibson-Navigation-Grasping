# 上機測試計畫 — 巡航到丟垃圾

離線能驗的都綠了（11 套、約 900 個檢查）。這份是**只能在實車上驗**的部分，按依賴順序排。
每一關沒過就不要往下跑：後面的失敗會被前面的問題污染，浪費的是現場時間。

規則：**一次只改一個變因**。任何方向異常立刻斷電，不要讓 policy 自己「恢復」。

---

## T0 — 開機前（每次都做，2 分鐘）

```bash
sudo fuser -v /dev/myserial          # 必須剛好一個 PID，或空的
ps -ef | grep -E '[r]osmaster_main|[M]cnamu_driver|[a]i_motor_server_B|[r]oute_a_runtime'
```

有其他程序佔著就先殺掉。手臂和輪子共用這條 UART，兩個程序開它 = 命令交錯。

ROS 端起來（各自終端機）：`roscore` → TG30 driver → robot_state_publisher → map_server →
AMCL → rosbridge。RViz 用 **2D Pose Estimate 設緊初始化**。

**通過條件**：`rostopic hz /scan` 約 10 Hz，RViz 看得到地圖和粒子雲。

---

## T1 — odom 活著（風險最高，先驗）

這是整個專案唯一「壞掉但看起來正常」的環節。v21 的 `ServoController` 不會開自動回報，
沒開的話 `get_motion_data()` 永遠回傳 0 —— odom 會顯示成一台從不移動的機器人，**而 AMCL
會相信它**。`mission_pipeline` 的 self-check 補了這個並會拒絕啟動，這關就是驗它有生效。

```bash
source ~/grasp_venv/bin/activate
python3 integration/ros_io.py --probe --ros-host 127.0.0.1     # 先確認 AMCL 有在發
python3 integration/mission_pipeline.py --real --no-deliver --detection-streak 999 \
  --max-laps 0 --route <route.yaml> \
  --i-confirm-serial-owner --i-confirm-lidar-orientation --i-confirm-arm-cam-pose
```

自檢跑完會停在等你按 Enter。**這時先不要按**，另開終端機：

```bash
rostopic hz /odom_setmotor                  # 應該約 20 Hz
rostopic echo -n1 /odom_setmotor/child_frame_id   # 應該是 base_footprint
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map base_footprint        # AMCL 有接上才會有
```

然後**用手推車前進約 50 cm**，看 `/odom_setmotor` 的 x 有沒有跟著長。

**通過條件**
- self-check 印出 `auto-report enabled` + `board is reporting` + `wheel feedback valid`
- `/odom_setmotor` 約 20 Hz，`odom→base_footprint` 只有一個發布者
- 手推 50 cm，odom 走約 50 cm（±10%；Route A 實測方形閉合誤差 7.8 cm）

**失敗代表什麼**
- 沒有 `board is reporting` → 自動回報沒開或主機板沒回話，**不要往下跑**
- odom 完全不動 → 同上
- odom 動但方向反了 → 校正常數或輪序有問題，回頭查 `feedback_odom.py` 的 scale

---

## T2 — LiDAR 左右方向（policy 把左當右就完了）

Route B 的文件說 policy 端要再補 180°，主 repo 說不用（因為 ROS `/scan` 已經是 CCW）。
**兩邊只有一個對**，只能實物驗。

```bash
python3 integration/nav_rl.py --probe --lidar-backend ros --ros-host 127.0.0.1
```

在車子**正前方**放一個箱子 → 中間的 ray 應該變短。移到**左邊** → 高 index（接近 47）變短。
移到**右邊** → 低 index（接近 0）變短。

**通過條件**：左右和 index 對得上。
**失敗**：左右反了 → 加 `--lidar-dir -1`；整體偏轉 → `--lidar-yaw-offset-deg`。

---

## T3 — 巡航（不夾取、不送桶）

```bash
python3 integration/mission_pipeline.py --real --show \
  --no-deliver --detection-streak 999 --max-laps 1 \
  --route <route.yaml> \
  --i-confirm-serial-owner --i-confirm-lidar-orientation --i-confirm-arm-cam-pose
```

`--detection-streak 999` = 永遠不會離開路線。這關只驗「跟著 83 個 waypoint 走完一圈」。

**先在空曠處驗轉向正負**：起步後如果車子朝**遠離**目標的方向轉，立刻 Ctrl+C，加 `--wz-sign -1`。

**通過條件**
- 走完一圈（58.2 m），不撞牆
- 每個 waypoint 都有 `next waypoint: wp_xxx` 的 log，不會卡住或跳號
- 過程中不會反覆進 PAUSED

**要記的**：到點誤差、幾何煞停觸發次數、走完一圈的時間、AMCL covariance 有沒有發散。

**已知風險**
- 地圖是 GLB 渲染不是 SLAM 建圖，現場家具不在圖上 → AMCL 可能被拉偏
- 28 m 長走廊 + 麥輪旋轉系統性左偏 → 丟失定位風險
- 丟失後**沒有自動恢復**，會進 PAUSED 等人重設初始位姿

---

## T4 — 巡航中發現物體並走過去 ★ 從未實測

**這關是這次最需要盯的**，因為 2026-08-04 在這段修掉三個只會在實機上表現成
「它無視垃圾」的邏輯錯誤。離線測試涵蓋了，但沒有實機證據。

```bash
python3 integration/mission_pipeline.py --real --show \
  --no-deliver --max-laps 1 \
  --route <route.yaml> \
  --i-confirm-serial-owner --i-confirm-lidar-orientation --i-confirm-arm-cam-pose
```

在巡航路線旁邊放一個 sugarbox。

**要逐一確認的狀態轉換**（log 會印狀態名）

| 看到什麼 | 代表 |
|---|---|
| `PATROL ... streak=1,2,3` | 偵測連續幀在累積 |
| `INVESTIGATE  trash seen on 3 consecutive frames` | **有離開路線**（修好前這裡會立刻彈回 PATROL） |
| `APPROACH  target confirmed at X.XX m` | 距離進入 2.5 m |
| `ALIGN  within 0.75 m — fine align` | RL 導航交棒給視覺精對位 |
| `STATIONARY_GATE  object inside the trained envelope` | 過了交接門檻 |

**通過條件**
- 從 PATROL 進到 INVESTIGATE 後**沒有**在一兩個 tick 內彈回 PATROL
- 一路走到 ALIGN

**失敗的判讀**
- `INVESTIGATE → RESUME (target lost during investigate)` 幾乎立刻發生
  → tracker 又被清掉了，回頭看 `reset_nav(clear_tracker=...)`
- 一直停在 PATROL、streak 反覆歸零
  → 偵測不穩，或 `--detection-jump-m`（預設 0.35 m）太緊
- `aligned but outside the v21 envelope: x=...`
  → 車停的位置不對。log 會同時印三個座標系，看 base_footprint 那個值：
    **目標是 policy x 0.20~0.28 = base_footprint 前方 18~26 cm**

**要記的**：從看到到停穩花多久、停下來時物體在 base 座標的實際位置、失敗重試了幾次。

---

## T5 — 夾取（v21 已驗過，這裡驗的是整合沒有破壞它）

拿掉 `--no-deliver` 以外的限制，讓它跑到 VERIFY。

**通過條件**
- `[Init] Contract: obs_28_incremental`、模型 sha256 verified
- 夾爪在未完全閉合的角度停住（接觸判定），不是走到 180° 底
- `grasp VERIFIED — object is off the floor`

**要特別看的**：夾取全程底盤必須**完全不動**。FSM 保證輪子和手臂不同時被允許動，
如果看到底盤在手臂動作時抖動，那是別的程序在下命令 → 回 T0。

**失敗重試**：最多 3 次，之後會放棄、把該地點加入黑名單、回去巡航。
確認第 4 次不會再對同一個物體重試。

---

## T6 — 丟垃圾（最簡化）

```bash
python3 integration/mission_pipeline.py --real --show \
  --route <route.yaml> \
  --i-confirm-serial-owner --i-confirm-lidar-orientation --i-confirm-arm-cam-pose
```

夾到之後會導航到 `route.yaml` 的 `trash_bin.approach`（4.35, 13.00），然後直接跑
`run_release_only()`：往前伸 → 開爪 → 回 home。不看垃圾桶在哪。

**通過條件**
- log 印 `bin approach target: (4.35, 13.00, ...)`，而**不是**某個 patrol waypoint
  （這是修掉的 bug：原本會開去當時的巡航點）
- `[Stage 3] reach 1..6` 有跑
- `[Done] Release motion complete`
- 之後回到 `PATROL`，且是**中斷點的下一個** waypoint

**注意**：沒有放開的驗證。`released` 的意思是**動作跑完了**，不是「東西在桶子裡」。
要確認就自己看。

---

## 致命問題（會讓整套跑不起來，或壞掉但看起來正常）

| # | 問題 | 現況 | 怎麼確認 |
|---|---|---|---|
| 1 | **自動回報沒開 → odom 永遠是 0，AMCL 相信它** | 已在 self-check 補上並會拒絕啟動 | T1 |
| 2 | **LiDAR 180° 兩份文件說法相反** | 未解，只能實物驗 | T2 |
| 3 | **序列埠被別的程序佔住** | 只能人工檢查 | T0 |
| 4 | 轉進 INVESTIGATE 時丟掉觸發它的偵測 → 永遠走不到物體 | 已修 + 回歸測試 | T4 |
| 5 | DELIVER 沒有把目標指向垃圾桶 → 開去巡航點 | 已修 + 回歸測試 | T6 |
| 6 | 放棄物體後旗標沒清 → 下一次 LATCH 用**上一個物體的座標** | 已修 + 回歸測試 | T5 重試後再放一個物體 |
| 7 | 沒有黑名單 → 對夾不起來的東西無限重試 | 已修（給up 點 1 m 內不再觸發） | T5 |
| 8 | `route.yaml` 最小間距 0.049 m，到點判定會一次過好幾個點 | 已修（重取樣 117→83） | T3 看 waypoint 有沒有跳號 |
| 9 | AMCL 丟失定位沒有自動恢復 | **未解**，會進 PAUSED 等人 | T3 |
| 10 | 地圖是 GLB 渲染，現場障礙不在圖上 | **未解**，靠 48 束 policy 避障 | T3 |

---

## 仍未解、但這次不處理的

- **手臂相機 C3 外參是 URDF 預測值不是量測值**。依 2026-08-04 決定不處理：v21 用這組
  參數實機夾成功了。若夾取落點系統性偏移，這是第一個要回頭看的地方。
- **v21 是 `candidate` 不是 approved**。`manifest.json` 的 `protocol_valid: false`。
  成績是真的但沒有認證，報告/簡報要照這個講法。
- **靜止 feedback 雜訊沒量過**，stationary gate 的門檻（0.02 m/s、0.05 rad/s）是暫定值。
- **阻塞動作期間不檢查 `/scan` 新鮮度**。夾取時底盤是停的所以還好，但 FINE_ALIGN
  期間底盤會動而且不看 LiDAR（沿用既有模式 A 的行為）。
