# Jetson Pull 後 Dry-Run 驗證清單（v17 部署）

> 適用：commit `bb51a61`（v17 權重 + 新 `grasp_home_deg`）之後的版本。
> 目標：確認 v17 模型與新的準備夾取姿態在實機上正確，才進行第一次 `--real` 夾取。
> 姿態原理見 [arm_pose.md](arm_pose.md)；視覺 pipeline 的完整驗證另見
> [integration/DEPLOY_VERIFY.md](integration/DEPLOY_VERIFY.md)（本清單只聚焦夾取端 dry-run）。

---

## Phase 0 — Pull 與檔案確認

```bash
cd <repo 目錄>
git pull
git log --oneline -3        # 應看到 bb51a61（v17 部署）與 arm_pose/checklist 更新
ls -l grasp/trained_6d_models_v17/
```

- [ ] `ppo_6d_final_ready_for_real_robot.zip`（約 180 KB）存在
- [ ] `vecnormalize_6d_final.pkl`（約 2.3 KB）存在
- [ ] `grasp/` 頂層與 Jetson 家目錄**沒有殘留的舊模型 zip**（避免 `--model` 手動指路徑時拿錯檔）

## Phase 1 — 靜態驗證（不動任何馬達）

```bash
source ~/grasp_venv/bin/activate   # 依實際 venv 調整
python3 integration/verify_x3plus_deploy.py
```

- [ ] PPO model / VecNormalize / URDF 檢查通過（路徑應顯示 `trained_6d_models_v17`）
- [ ] `Rosmaster_Lib`、`ultralytics`、`cv2` 可載入；序列埠存在
- [ ] port 7000 motor server 沒在跑、沒有 ROS 底盤 driver 占用序列埠

## Phase 2 — 相機/座標靜態檢查

```bash
python3 integration/verify_camera_grasp_frame.py
```

（`--grasp-home-deg` 預設已更新為 `90,32.704,9.786,32.704,90,30`，不用手動帶參數）

- [ ] navigation home 與 PPO grasp home 兩組 `mono_link` 位姿都有印出
- [ ] grasp home 下相機方向大致朝 base +X 前下方（俯視地面）
- [ ] 記下兩個姿勢的相機位置差（之後校正視覺 offset 會用到）

## Phase 3 — 夾取 dry-run（核心步驟；只印角度，不動馬達）

```bash
python3 grasp/x3plus_real_grasp.py          # 不加 --real；預設物體 (0.25, 0, 0.02)
```

逐項核對 log：

- [ ] `[Init] Loading model: ...trained_6d_models_v17/ppo_6d_final_ready_for_real_robot.zip`
- [ ] `[Init] Loading VecNormalize: ...trained_6d_models_v17/vecnormalize_6d_final.pkl`
- [ ] 先出現導航 home：目標 `[90, 140, 0, 0, 90, 30]`
- [ ] 再出現 `move_to_grasp_home`：目標 **`[90, 32.704, 9.786, 32.704, 90, 30]`**
      （S2 140→32.7、S3 0→9.8、S4 0→32.7 —— 這是 API 角，實體會是前傾俯視姿態）
- [ ] `Home TCP (URDF)` 位置合理：x 約 0.2–0.3、y 約 0、z 在物體上方；初始 dist < 0.2 m
- [ ] Stage 0：dist 前 10–30 步**穩定下降**；S1–S5 每步變化 ≤ 3°（rate limit）；S6 全程維持 30（張開）
- [ ] Stage 1：印出 `Arm locked at [...]`；S1–S5 完全不變；S6 從 30 每步 +3° 升到 180（閉合）
- [ ] Stage 2：手臂逐步回 `[90, 140, 0, 0, 90]`，S6 **保持 180**
- [ ] 結尾：`[Done] Returned to home. Object held.` 與 `[End] Grasp succeeded — keeping gripper closed...`
- [ ] 全程角度都在範圍內（S1–S4: 0–180、S5: 0–270、S6: 30–180），無單步跳變

任一項不符 → 停在 dry-run，先查 `DeployConfig` 與 [arm_pose.md](arm_pose.md)，不要上 `--real`。

## Phase 4 — 首次 `--real`：只驗證姿態（低速，不夾）

準備：工作區淨空、隨手可斷電/Ctrl+C。

```bash
python3 grasp/x3plus_real_grasp.py --real
# 等手臂走完「導航 home → 夾取 home」後、policy 開始前的 4 秒 settle 期間 Ctrl+C
```

- [ ] 手臂平順移到夾取 home，無暴衝、無自碰、無撞地
- [ ] **Yahboom App 讀值 ≈ `[90, 147, 170, 147, 90]`（物理角，容差 ±5°）**
      —— API 送 32.7/9.8/32.7，App 顯示 147/170/147 才是正確的（API 鏡像）
- [ ] 目視：相機朝前下方看得到地面、夾爪張開懸在前方放物區（base 前方 x 0.17–0.33 m）上方
- [ ] 若 App 讀值或姿態不符：**先懷疑該關節的 API 鏡像假設**（見 arm_pose.md 校正原則第 4 點），
      不要直接改 `grasp_home_deg` 數字

## Phase 5 — 首次實夾

物體：木塊或小方盒（**高度 ≥ 5.5 cm**，寬 < 6 cm），放地面、正前方約 x=0.25、y=0。

```bash
python3 grasp/x3plus_real_grasp.py --real --obj-x 0.25 --obj-y 0 --obj-z 0.03
```

- [ ] Stage 0 dist 收斂（期望 < 0.06 m 觸發）
- [ ] Stage 1 閉爪夾住物體；Stage 2 帶著物體回導航 home
- [ ] 成功後再測左右偏移（`--obj-y ±0.05`）與遠近（`--obj-x 0.20 / 0.30`）

> 預期管理：sim 的夾取判定有 ~1.4 cm 磁吸容差，實機是真實摩擦夾取，
> 首輪成功率低於 sim 的 97–100% 屬正常。記錄失敗模式（沒對準 / 碰倒 / 夾空 / 抬升掉落）再迭代。
