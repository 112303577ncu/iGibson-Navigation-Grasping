# v21 部署審查與後續執行計畫（2026-07-30）

> **給執行端 AI（Opus）**：本文件是 v21 candidate 進 repo（PR #14）後的完整審查結果與
> 分階段執行計畫。工作分支 `worktree-grasp-v21-candidate`，worktree 路徑
> `C:\Users\user\Documents\repo\claude\x3plus\.claude\worktrees\grasp-v21-candidate`。
> 先讀完「禁止事項」再動手。所有發現都已實際驗證過，不是推測。

---

## 0. 背景一句話

v21 candidate 已以隔離部署棧進 `grasp/v21/`（PR #14 draft），筆電端 38/641 自測與
dry-run 全過、與交接包基準逐字一致。本審查在上機前又抓出 **1 個必修大洞（F1）**
與數個中小問題。

---

## 1. 審查發現總覽

| # | 嚴重度 | 問題 | 修復階段 |
|---|---|---|---|
| F1 | 🔴 上機前必修 | v21 硬體層用 `Arm_Lib`（DOFBOT 舊驅動），非 X3Plus 的 `Rosmaster_Lib`；Jetson 上 `--real` 會**靜默降級成只印不動** | A1 |
| F2 | 🟠 上機前必修 | `--real` 無 runtime gate：manifest `hardware_gates` 全 false 但 script 照跑不誤 | A2 |
| F3 | 🟠 上機前必修 | `--obj-x/y` 實體量測基準未定義（2cm frame offset 導致「從哪量」不明） | A5 + C3 |
| F4 | 🟡 順手修 | PR #14 base 落後 `origin/main` 2 commits（缺 PR#12 v18 staging） | A3 |
| F5 | 🟡 順手修 | HANDOFF 宣稱 evidence/ 含 `GRASP_TRAINING_REQUIREMENTS_2026-07-27.md`，實際封包沒有 | A4 |
| F6 | 🟢 已降級 | SB3 版本配對：訓練端實為 **2.2.1**（torch 2.4.1+cpu / gymnasium 0.29.1 / numpy 1.23.5），Jetson pin 2.3.2 = 舊存新載的安全方向；筆電警告純屬 Py3.12 pickle 問題 | B2 驗證即可 |
| F7 | 🟢 上機時測 | lateral sign（+y=左？）從 v18 manifest 起就是 open question，未實機確認 | C4 |
| F8 | 🟢 上機時處理 | Rosmaster_Lib 在 `grasp/v21/` 的 import 路徑（驅動在 `grasp/Rosmaster_Lib`，cwd 不同找不到） | A1 一併處理 |

**F1 細節**（教訓：dry-run 印的 `[WARNING] Arm_Lib not found` 就是證據，先前被當雜訊）：
- `grasp/v21/x3plus_real_grasp.py:64-71`：`import Arm_Lib; Arm_Lib.Arm_Device()`
- `:368`：`self.dry_run = dry_run or (not HAS_ARM_LIB)` → Jetson import 失敗 → `--real` 靜默變 dry-run
- 送角度用 `Arm_serial_servo_write6_array`、讀用 `Arm_serial_servo_read` —— DOFBOT API
- **數學不用動**：config 的 `90 + sim_deg`、invert 全 False 本來就是照 Rosmaster 慣例寫的
- **測試不用動**：test_deploy_controller 用 FakeServoPlant，不碰驅動（已驗證無 Arm_Lib 依賴）
- 此發現須回報訓練端（他們的部署腳本傳輸層是別台機器人的，v22 封包要修）

---

## 1.5 快速路徑（目標：最快夾到第一把）

使用者若指示走快速路徑，照本節；否則走完整 Stage A–E。兩者的差別只是**順序與延後**，
不是跳過安全檢查。

**關鍵路徑（依序，順利可在一個工作天內完成第一夾）：**

1. **A1 移植 + 筆電重驗**（38/641 + dry-run 基準比對）——唯一的程式工作，沒得繞
2. **scp → Jetson**：跑 controller 自測 + dry-run 就可上機；`test_deploy_floor_guard.py`
   （641 檢查，Nano 上慢）**丟背景跑，同時做實體準備**（量木塊、清場地）——平行化，不是跳過
3. **`--real` 短輪到 C3 → 尺規 gate（14.4±1cm）→ 標 P 點**——不可省
4. **第一夾**：木塊（卡尺實測 `--object-height`）放 P 前方 1.4cm、左 0.4cm =(0.24, 0.00)

**可延後項目與代價：**

| 延後項 | 何時必須補 | 延後的代價 |
|---|---|---|
| A2 runtime gate | 第一把成功後 | 誤配權重/誤用 `--real` 的防線暫缺（單人在場操作可接受） |
| A3 merge origin/main | PR 轉 ready 前 | 無（純 repo 衛生） |
| A4 evidence 補檔 / A5 README | 回報訓練端前 | 無 |
| C4 lateral sign | **打任何 y≠0 的格子之前** | 中心格 y≈0 不受影響 |
| C5 reach 掃描 | 碰 x=0.20 / 0.28 之前 | 先夾 0.24 不受影響 |
| 紙團、瓶蓋 | 木塊兩格都過之後 | 無（本來就照序） |

**快速路徑也不准省的四樣**：①尺規 gate（唯一能在第一夾前抓出 FK/frame 系統性偏移的
檢查）②`--object-height` 卡尺實測（28D contract 不會擋缺席）③物體由高到矮
④`guard=[...]` log 原文保存（零成本，8mm 線鬆緊的唯一證據）。

**期望管理**：實機 overshoot vs 8mm 線、實際 reach 皆從未量過，第一把不保證成。
木塊+中心格是全系統最寬容組合（模擬 100%、pad 閉合高度 4.4cm、離 guard 線最遠），
若它失敗，guard 行會直接指出原因——那是設計好的回饋迴路，不是白跑。

---

## 2. Stage A — 筆電端修復（Opus 執行，在上述 worktree 分支上）

### A1. 傳輸層移植 Arm_Lib → Rosmaster_Lib（F1、F8）

**只動 `grasp/v21/x3plus_real_grasp.py` 的硬體 I/O，其餘一律不碰。**
照抄主 script 的現成寫法（參考行號為 `grasp/x3plus_real_grasp.py`，主 checkout 或 origin/main）：

| 要改的 | v21 現況 | 照抄來源（v17 主 script） |
|---|---|---|
| import 塊 `:62-71` | `import Arm_Lib` | `:56-63` `from Rosmaster_Lib import Rosmaster as _RosmasterCls`；失敗時 flag=False |
| DeployConfig | 無 serial_port | `:87` `serial_port: str = "/dev/myserial"` |
| ServoController.__init__ | 靜默降級 | `:387-398` **--real 且驅動缺 → 直接 raise（fail-closed），不准降級**；成功則 `_RosmasterCls(com=cfg.serial_port)` + `create_receive_threading()` |
| 送角度 | `Arm_serial_servo_write6_array` | `:433/:534` `set_uart_servo_angle_array(angle_s=[...6個], run_time=ms)`；`:555` **ms clamp ≤2000**（板子限制） |
| 讀角度 | `Arm_serial_servo_read(i)` | `:566` `get_uart_servo_angle(i)`，i=1..6；保留 v21 既有的回讀驗證邏輯 |
| CLI | 無 | `:1542` `--port` 預設 `/dev/myserial` |

規則：
- 保持 `send_degrees` / `read_degrees` / `emergency_stop` 簽名與 `.dry_run` 屬性不變（測試靠它們）
- `HAS_ARM_LIB` 改名 `HAS_ROSMASTER`（已確認測試檔不引用此旗標）
- **JointMapper / deploy_contract / FloorGuard / GraspController 邏輯一行都不准動**
- F8：import 前把 `Path(__file__).parent.parent` 加進 `sys.path`（讓 `grasp/Rosmaster_Lib` 可見），
  或在 Jetson 步驟 `cp -r ../Rosmaster_Lib .` —— 擇一，寫進 README

**驗收**：
```bash
cd grasp/v21
PYTHONIOENCODING=utf-8 python test_deploy_controller.py    # 必須仍 all 38 checks passed
PYTHONIOENCODING=utf-8 python test_deploy_floor_guard.py   # 必須仍 all 641 checks passed
# dry-run 必須與基準一致：wrist_z_offset 0.0564、Stage 0→1 xy=13.5/22mm、9 steps、ABORT、exit 0
grep -in "arm_serial\|Arm_Device" x3plus_real_grasp.py     # 必須 0 hit（歷史註解除外）
```
完成後更新 `manifest.json`（laptop_verification 重跑紀錄 + transport 移植註記）與 README。

### A2. `--real` runtime gate（F2）

在 `main()` 的 `args.real` 分支、**任何硬體初始化之前**：
1. 讀同目錄 `manifest.json`，sha256 驗 model 與 vecnorm 兩檔，不符 → 印明細後 exit
2. `status != "hardware-approved"` → 要求新旗標 `--unlock-candidate-real`，未帶則印出
   全部 hardware_gates 狀態後 exit
3. dry-run（無 `--real`）完全不受影響

**驗收**：筆電上 `--real`（無旗標）→ 在碰 serial 前就拒絕；加旗標 → 走到
Rosmaster 缺驅動的 fail-closed 錯誤（順便證明 A1 的 fail-closed 生效）。38/641 不變。

### A3. 補上 PR base（F4）

```bash
git fetch origin && git merge origin/main   # 預期無衝突（PR#12 只加 trained_6d_models_v18/）
git push
```
**禁止 rebase / 禁止 force-push。**

### A4. evidence 補檔（F5）

從主 checkout 根目錄複製 `GRASP_TRAINING_REQUIREMENTS_2026-07-27.md` →
`grasp/v21/evidence/`，並在 manifest 註記 provenance（HANDOFF 宣稱封包內含此檔但實際缺）。

### A5. 量測協定寫進 README（F3，常數已算好）

FK 實算（本 repo `grasp/v21/` 環境，訓練座標系）：
- **C3 時 gripper_center = (x 0.2263, y −0.0035, z 0.1439)**
- **C3 時 pad 最低點離地 z = 0.111**

協定（Stage C 用）：手臂停在 C3 → 從兩指 pad 中點鉛垂投影到地面標記 **P 點** →
P 在訓練座標 = **(0.2263, −0.0035)** → 之後所有 `--obj-x/--obj-y` 一律用
「P + 尺量偏移」換算，**不要從機殼或 base_link 原點量**（2cm frame offset 會咬人）。

| 目標格 | 相對 P 的擺放 |
|---|---|
| (0.24, 0.00) 中心格 | P 前方 **1.4 cm**、左 **0.4 cm** |
| (0.20, −0.09) 最差格 | P 後方 **2.6 cm**、右 **8.7 cm** |

（「左=+y」是假設，C4 會實測確認。）

### A6. Commit + push

`git show --stat` 核對（根 `.gitignore:29` 的 `*.zip` 會靜默吃權重檔——本輪雖無新 zip，習慣要在）。

---

## 3. Stage B — Jetson 驗證（Opus 產指令，使用者操作）

前置：`.\set_jetson_host.ps1 <IP>`；ssh 後 **kill 出廠 `rosmaster_main.py` 與 port 7000
motor server**（會佔 serial）；`ls -l /dev/myserial` 確認存在。

```bash
# 1) scp（deploy_jetson2 非 git，直接覆蓋）
scp -r x3plus/grasp/v21 jetson@$JETSON:~/Documents/deploy_jetson2/grasp/

# 2) Jetson 上
cd ~/Documents/deploy_jetson2/grasp/v21 && source ~/grasp_venv/bin/activate
ls ../x3plus/yahboomcar.urdf && find ../x3plus/meshes -type f | wc -l   # 前提：≥38

# 2a) 驅動與序列埠（A1 移植後的新前置條件，最容易卡在這）
ls -d ../Rosmaster_Lib || cp -r /usr/local/lib/python3.6/dist-packages/Rosmaster_Lib ..
ls -l /dev/myserial                       # 不存在 → 用 --port 指定實際裝置
python3 -c "import sys; sys.path.append('..'); from Rosmaster_Lib import Rosmaster; print('driver OK')"
# 佔用序列埠的先殺掉（出廠自啟）
ps aux | grep -E "rosmaster_main|motor_server|roslaunch" | grep -v grep

# 3) 載入測試（F6 驗證，預期乾淨無警告——訓練端 SB3 2.2.1 < Jetson 2.3.2）
python3 -c "
from stable_baselines3 import PPO
m = PPO.load('models/candidate_v21_seed816_ckpt550000.zip', device='cpu')
print('OK', m.observation_space, m.action_space)"

# 4) 兩支自測 + dry-run（同 Stage A 驗收指令）
```

**Gate 判準**：38/641 必須一字不差；dry-run 的 `wrist_z_offset` 必須 **0.0564**（純幾何，
跨平台不該變）；Stage 序列必須相同（0→1 → close → ABORT → home）；策略步數容許 ±3 的
浮點漂移，超過 → **停，回報**。全過才把 manifest `jetson_*` gates 翻 true。

---

## 4. Stage C — 實機標定（使用者在場，`--real --unlock-candidate-real`，手放電源）

1. **姿態驗證**：先用 `--max-steps 5` 短輪跑 startup-home→C3（若要中斷，先確認
   Ctrl+C 會走 emergency_stop 再用）。目視 C3 側面輪廓（特徵：S3==S4）。
2. **尺規 gate（最關鍵）**：C3 時 pad 中點離地 ≈ **14.4 cm**、pad 最低點 ≈ **11.1 cm**。
   ±1 cm pass；1–2 cm 記錄再議；**>2 cm 立刻停** —— 代表 TCP/frame 修正在實機不成立，
   後面每一夾都會系統性偏掉。
3. **標 P 點**（A5 協定），之後座標全部相對 P。
4. **lateral sign（F7）**：物體放 P 左 5 cm、給 `--obj-y +0.0465`，看 S1 是否向左轉；
   反了 → 停，回報（y 符號要翻）。
5. **reach**：在 P 座標系標 x=0.20 / 0.24 / 0.28 三點，確認實機可達。
6. **物體高度卡尺表**：木塊 / 紙團 / 瓶蓋各量一次，記錄。
7. 過的 gates 翻 true，記入 progress.md。

---

## 5. Stage D — 首夾

| 順序 | 物體 | `--object-height` | `--obj-z` | 位置順序 |
|---|---|---|---|---|
| 1 | 木塊 | 0.066 | 0.033 | (0.24,0.00) → (0.20,−0.09) |
| 2 | 紙團 | 0.035 | 0.0175 | 同上 |
| 3 | 瓶蓋 | 0.013 | 0.0065 | 同上 |

- `--object-height` **每次必給實測值**（28D contract 不會擋缺席，會靜默用對稱假設）
- 每次保存完整 console log，**`guard=[...]` 行原文照抄**（8mm 線鬆緊的唯一證據）
- 瓶蓋觸發 guard 開爪退回 = 設計內安全失敗，不是 bug（模擬淨空 3.99mm < 8mm 線）

---

## 6. Stage E — 回報與收尾

1. 照 HANDOFF §8 回報訓練端四項，**外加 F1（Arm_Lib）發現**
2. 依 guard 實證決定 8mm 線調整
3. 決定 backport 清單（TCP=gripper_center / URDF_TO_TRAINING_FRAME / 預防式 FloorGuard → 主 script）
4. 最後才碰 `--socket`：bridge 送 `{x,y,z,w,…}` vs v21 讀 `{x,y,z,height}` —— 方案是
   bridge 加 class→實測高度查表、送 `height` 欄位
5. F1 教訓補進 TROUBLESHOOTING.md（症狀：dry-run 印 Arm_Lib 警告；教訓：交接包必查驅動庫）

---

## 7. 需要使用者決定的事（Opus 不可自行執行）

| # | 事項 | 建議 |
|---|---|---|
| D1 | PR #13（v18/C3 啟用） | 不 merge；萃取「pairing guard + `move_arm.py --pose`」成獨立小 PR，關閉動作由使用者確認 |
| D2 | 主 checkout 20 modified + 12 untracked（第 2+3 批） | 另開 session 在**使用者 checkout** 分批 commit；`.claude/worktrees/` 巢狀 `.git` 勿 add |
| D3 | PR #7 det-log（open 20 天） | rebase 後決定 merge 或關 |

---

## 8. 禁止事項（Opus 必讀）

1. **不 force-push、不 rebase 已推分支**（要同步用 merge）
2. **不把 v21 權重餵 `grasp/x3plus_real_grasp.py`**（absolute vs incremental，shape 檢查抓不到）
3. 本輪**不動** `grasp/x3plus_real_grasp.py`、`trained_6d_models_v17/`、`trained_6d_models_v18/`
4. 權重與 vecnorm 不混搭；以 manifest sha256 為準
5. A1 只換傳輸層；數學 / 契約 / guard / 狀態機邏輯不改
6. 所有 `--real` 需使用者在場；gates 未全 true 前必帶 `--unlock-candidate-real` 且手放電源
7. `git add` 目錄後必 `git show --stat` 核對（根 `.gitignore` 的 `*.zip` 陷阱）
8. 不得把本包寫成 `sim-approved`（正式評估 `protocol_valid=false`）

---

## 9. 驗收清單

- [x] **A1 移植完成**（2026-07-30，commit `7967e62`）：38/641 不變、dry-run 與基準逐字一致、
      無 Arm_Lib 呼叫殘留。額外驗證：`--real` 無驅動 exit 2（載模型前）、無驅動的
      ServoController 不 raise 也不降級（寫入拒絕/讀取 invalid/姿態不前進）、stub driver
      確認 API 呼叫形狀正確且 9999ms→2000ms、越界 S2=200 被擋且 0 筆送到板子
- [ ] A2 gate 生效（無旗標 `--real` 在碰硬體前被拒）
- [ ] A3 分支已 merge origin/main 並 push
- [ ] A4 evidence 補檔 + provenance
- [ ] A5 量測協定入 README
- [ ] B Jetson：load 乾淨、38/641、dry-run wrist_z_offset=0.0564
- [ ] C2 尺規 14.4cm ±1cm
- [ ] C4 lateral sign 確認
- [ ] D 首夾 log（含 guard 行）× 6 組合
- [ ] E 回報訓練端（含 F1）
