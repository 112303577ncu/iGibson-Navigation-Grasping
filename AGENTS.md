# X3Plus 專題 — Sim-to-Real 夾取部署

## 專題概述
將 PyBullet + PPO 訓練的 6D 夾取策略，部署到 Yahboom X3Plus 實體機器人。

- **硬體**：X3Plus（麥克納姆輪底盤 + 5-DOF 機械臂），Rosmaster 擴充板，Jetson Nano
- **框架**：Stable-Baselines3 PPO + PyBullet（FK only）
- **驅動**：Rosmaster_Lib（需本地化到腳本同一目錄）

---

## 檔案結構（Jetson `deploy_jetson2` 資料夾）

| 檔案 | 說明 |
|------|------|
| `x3plus_real_grasp.py` | 主部署腳本（無 ROS，直接在 Jetson 執行） |
| `x3plus_deploy_bridge.py` | 工具類，正規化 action → 伺服機角度 |
| `trained_6d_models_v17/*.zip` | 訓練好的 PPO 模型 |
| `trained_6d_models_v17/*.pkl` | VecNormalize 統計（觀測正規化） |
| `x3plus/yahboomcar.urdf` | PyBullet FK 用的 URDF |
| `Rosmaster_Lib/` | 硬體驅動（本地化，見下方指令） |

---

## 環境設定（在 Jetson Nano 執行一次）

```bash
# 步驟 1：本地化驅動（解決 Python 3.8 找不到 Python 3.6 驅動的問題）
cd ~/Documents/deploy_jetson2
cp -r /usr/local/lib/python3.6/dist-packages/Rosmaster_Lib .

# 步驟 2：確認 Python 3.8 虛擬環境已啟用
source ~/grasp_venv/bin/activate
```

---

## 部署指令

```bash
# 空跑測試（不驅動伺服機，確認角度輸出合理）
python3 x3plus_real_grasp.py

# 實際執行
python3 x3plus_real_grasp.py --real

# 實際執行 + 外部視覺偵測（TCP port 5555）
python3 x3plus_real_grasp.py --real --socket

# 自訂物體位置（公尺）
python3 x3plus_real_grasp.py --real --obj-x 0.30 --obj-y 0.05 --obj-z 0.02
```

---

## Sim-to-Real 關節映射

| 伺服機 | 方向 | hw 公式 | 狀態 |
|--------|------|---------|------|
| S1 | 正向 | hw(API) = 90 + sim_deg | 已確認 |
| S2 | 正向 | hw(API) = 90 + sim_deg | API 鏡像已抵消舊 invert |
| S3 | 正向 | hw(API) = 90 + sim_deg | API 鏡像已抵消舊 invert |
| S4 | 正向 | hw(API) = 90 + sim_deg | API 鏡像已抵消舊 invert |
| S5 | 正向 | hw(API) = 90 + sim_deg（0~270°）| 已確認 |
| S6 夾爪 | 特殊 | open=30°, closed=180° | 已確認（2026-05-22） |

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
| 1 | dist < 5 cm、grip_cmd > 0.90、或 dist 從最小值回升 > 5 cm | 強制閉合夾爪 |
| 2 | Stage 1 完成後 | 維持夾爪閉合，回 home |

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
- `arm_hw_invert = (False, False, False, False, False)`。Rosmaster API 對 S2/S3/S4 的鏡像已在 API 層抵消舊的反向設定；所有部署與校正工具均以此為準。
- 安全限制：`max_delta_deg=3.0`，每步最多動 3°，防止暴衝
