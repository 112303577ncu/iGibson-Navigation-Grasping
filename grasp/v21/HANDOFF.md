# X3Plus 夾取部署交接 — v21 candidate

打包日期:2026-07-29
打包來源:`D:\repo\igibson_x3_test`(branch `main`)
用途:**筆電端實機測試**

---

## 0. 一句話結論

這包裡的權重**在模擬端通過了 `GRASP_TRAINING_REQUIREMENTS_2026-07-27.md` §5 全部訓練規格與 §6 全部驗收門檻**,
部署端的 TCP bug 已修好並驗證,dry-run 走得通,**可以上機實測**。

但它**不是**通過正式協定的版本 — 見第 4 節,務必讀完再上機。

---

## 1. 這包裡有什麼

```
X3PLUS_GRASP_DEPLOY_v21candidate_20260729/
├── HANDOFF.md                      ← 本檔
├── deploy/                         ← 執行目錄(所有 import 都是同層,不要拆散)
│   ├── x3plus_real_grasp.py        ← 主部署程式
│   ├── deploy_contract.py          ← 契約定義 + fail-closed 驗證
│   ├── action_execution_v21.py     ← 動作執行狀態
│   ├── test_deploy_controller.py   ← 可在筆電自我驗證(38 checks)
│   ├── test_deploy_floor_guard.py  ← 可在筆電自我驗證(641 checks)
│   ├── models/
│   │   ├── candidate_v21_seed816_ckpt550000.zip       ← 權重
│   │   └── candidate_v21_seed816_ckpt550000_vec.pkl   ← VecNormalize(必須配對使用)
│   └── x3plus/
│       ├── yahboomcar.urdf         ← FK 用
│       └── meshes/                 ← URDF 實際引用的 38 個 mesh(151 MB)
└── evidence/                       ← 佐證,不參與執行
    ├── v21_formal_evaluation_seed90210.json   ← 235 cases 完整逐筆結果
    ├── v21_selection_lock.json
    ├── v21_formal_claim.json
    ├── v21_seed816_training_metadata.json     ← 完整訓練設定
    └── GRASP_TRAINING_REQUIREMENTS_2026-07-27.md
```

> **meshes 很重要**:主 repo 的 `.gitignore` 排除所有 `*.STL/*.obj/*.dae`,所以 git clone
> **拿不到 mesh**,URDF 會載入失敗、FK 直接掛掉。這包已經把 URDF 實際引用的 38 個
> 全部帶上,不需要另外抓。

---

## 2. 權重身份(請核對)

| | |
|---|---|
| 檔案 | `candidate_v21_seed816_ckpt550000.zip` |
| SHA256 | `3a7137712669725b2b97cbe68cfceb76c09a52b6f9cc1df73f682ae0277843ab` |
| VecNormalize | `candidate_v21_seed816_ckpt550000_vec.pkl` |
| SHA256 | `f986d8463577d20cec3c157a932665f2a1c61edd23a4d6aa23095cecb925bca0` |
| 來源 | 訓練 seed `816`,checkpoint `550000` |
| 契約 | `obs_28_incremental`(28D 觀測 / 6D 動作) |
| 動作語意 | **手臂增量**(`desired[i] = current[i] + action[i] × 0.08 rad`)、**夾爪絕對** |

這兩個 SHA 與 selection lock、formal report 的 `integrity_start` / `integrity_end` **三方一致**,已核對。

**權重與 VecNormalize 必須成對使用。** 混搭不同 checkpoint 的 vec 會靜默地產生錯誤行為。

---

## 3. 模擬端成績(seed 90210,235 cases,`deterministic=True`)

### §6 驗收標準

| 需求 | 門檻 | 實測 | |
|---|---|---|---|
| 瓶蓋 1.3 cm 確定性評估 | ≥ 90% | **100.0%** (25/25) | 通過 |
| 混合高度 1.3 / 3.5 / 6.6 cm | ≥ 90% | **100.0%** (75/75) | 通過 |
| 九宮格每格(x 0.20–0.28) | 每格 ≥ 80% | 最差格 **86.7%**,9/9 全過 | 通過 |
| 附上夾爪中心離地高度 | 需附 | grasp home 13.3–14.6 cm(均 14.0)<br>lock 3.9–8.7 cm(均 5.3) | 通過 |

九宮格逐格:

| cell (x_y) | 成功率 |
|---|---|
| 0.20_-0.09 | 86.7% ← 最差 |
| 0.20_+0.00 | 93.3% |
| 0.20_+0.08 | 100% |
| 0.24_-0.09 / 0.24_+0.00 / 0.24_+0.08 | 100% |
| 0.28_-0.09 / 0.28_+0.00 / 0.28_+0.08 | 100% |

### 地板安全

| | |
|---|---|
| floor violation | 0 |
| floor penetration | 0 |
| guard intervention | 0 |
| 最低淨空 | **3.99 mm** |
| 平均每回合最低淨空 | 18.16 mm |
| 撞倒物件 | 0 |

### §5 訓練規格合規(對照 `training_metadata.json`)

| 需求 | 要求 | 實際 |
|---|---|---|
| §5.1 高度連續分布 | 1.0–7.0 cm | `proc_object_height_range = [0.01, 0.07]`,矮物 40% 加權至 1.0–2.5 cm |
| §5.1 瓶蓋 1.3 cm | 圓扁 Ø≈3 cm | anchor `['cylinder', 0.013, 0.03]` |
| §5.1 紙團 3–4 cm | 球狀 | anchor `['sphere', 0.035, 0.035]` |
| §5.1 wood_block 6.6 cm | 保留 | anchor `['box', 0.066, 0.04]` |
| §5.2 spawn x | (0.20, 0.28) | `[0.2, 0.28]` |
| §5.2 spawn y | (−0.09, 0.08) | `[-0.09, 0.08]` |
| §5.3 grasp home | C3 不變 | `[0.0, -0.4, -1.4, -1.4, 0.0]`(= C3) |
| §5.4 成功判準 | 物理 + lift > 0.10 m | `success_height_threshold=0.1`, `success_use_max_lift=True` |
| §5.4 TCP | 維持 gripper_center | pad 連桿中點,未更動 |

---

## 4. 上機前必讀的三個限制

### 4.1 它沒有正式協定認證

v21 一次性正式評估的 verdict:

```json
{ "protocol_valid": false,
  "bottle_cap_ge_90": true, "mixed_ge_90": true, "grid_all_ge_80": true,
  "grid_min_rate": 0.8667, "floor_penetration_zero": true,
  "integrity_unchanged": true, "all_acceptance_gates": false }
```

唯一的 false 是 `protocol_valid`,原因是 `case_plan_valid.home_jitter_exact == false`
—— **home jitter 的執行方式不符 v21 協定**,不是抓取能力問題。

所以:**性能數字是真的、可信的;認證章沒有。** 檔名用 `candidate_`,
請不要在任何文件裡把它寫成 `simulation_accepted` 或 `ready_for_real_robot`。

### 4.2 地板餘裕低於部署 guard 的線

- 模擬 235 回合最低淨空:**3.99 mm**
- 部署 FloorGuard 的線:整段掃掠路徑 **≥ 8 mm**

代表**最低的那些姿態(主要是瓶蓋)實機上 guard 會介入**:它會 clamp 或安全放棄、
開爪退回 home。**不會撞地**,但那一把會失敗。

這是已知且刻意保留的訓練/部署落差(訓練 sim 用 5 mm 狀態檢查,部署用 8 mm 路徑檢查)。
瓶蓋成功率在實機上預期會低於模擬的 100%。

### 4.3 沒有任何實機證據

`deployment_ready` = **false**。Jetson 標定未做。
dry-run(不加 `--real`)的模擬讀數依設計只會回報 `unverified`,**不可當成抓取成功證據**。

---

## 5. 怎麼跑

### 5.1 環境需求

Python 3.8+,需要:

```
numpy
pybullet
stable-baselines3
gymnasium
torch
```

(訓練端用的是 Python 3.8.10。若筆電只做部署,不需要 iGibson。)

### 5.2 先自我驗證(不接硬體,建議先做)

```bash
cd deploy
python test_deploy_controller.py     # 期望:all 38 checks passed
python test_deploy_floor_guard.py    # 期望:all 641 checks passed,guard clamped 291x
```

這兩支**從本封包目錄實跑過**(2026-07-29),結果:

```
all 38 checks passed
all 641 checks passed — deployment guard is preventive
```

**若筆電端數字不同,先不要上機** —— 代表環境或檔案有差異。

> `test_deploy_parity.py` 需要 iGibson,不可攜,沒放進來。
> 它在訓練端的結果是 **77/77 passed**,平均關節目標誤差 0.0277 rad(< 一步 0.08 rad)。

### 5.3 Dry-run(不接硬體)

```bash
cd deploy
python x3plus_real_grasp.py \
  --model models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental \
  --object-height 0.066 --obj-x 0.24 --obj-y 0.00 --obj-z 0.033
```

**這包實際跑出來的結果**(2026-07-29,從本封包目錄執行,正確行為):

```
[Init] Contract: obs_28_incremental (obs 28D, arm action incremental)
[Init] All systems ready.
[Init] episode wrist_z_offset = 0.0564 m (object height 0.066, resting z 0.0330)
[Stage] 0→1  xy=13.5/22mm z=17.7/18mm r=22.3/26mm centred=True pads_ready=True
        (pad_after_close=44.4mm, object_top=66.0mm)
[Stage] 1: jaw close — reached=True (arrived, 9 steps) guard=['pass']
[Stage] 1 ABORT — jaw reached the fully-closed stop — nothing blocked it,
        so there is no object between the fingers
[Stage] Not lifting. Opening the jaw and retreating home.
[Retreat] home: reached=True (arrived)
exit code: 0
```

最後 ABORT 是**對的** — dry-run 沒有實體物件,爪子關到底,狀態機就拒絕宣告抓到。

### 5.4 實機

加 `--real`:

```bash
cd deploy
python x3plus_real_grasp.py --real \
  --model models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental \
  --obj-x 0.24 --obj-y 0.00 --obj-z 0.033 --object-height 0.066
```

**參數規則:**

- `--object-height`:物體**完整高度**(公尺),**必須實際量測,不可估、不可猜**。
  不給會直接 `MissingObjectHeightError` 擋下 —— 這是刻意的 fail-closed。
- `--obj-z`:物體**幾何中心**離地高度 = `object-height / 2`。
- `--obj-x` / `--obj-y`:base 座標。有效範圍 x 0.20–0.28、y −0.09–0.08(C3 實測可見∩可抓區)。

### 5.5 建議測試順序

**由高到矮**,因為越矮越貼近 8 mm guard 線:

| 順序 | 物體 | `--object-height` | `--obj-z` | 預期 |
|---|---|---|---|---|
| 1 | wood block 6.6 cm | `0.066` | `0.033` | 淨空充裕,最穩 |
| 2 | 紙團 3.5 cm | `0.035` | `0.0175` | 中等 |
| 3 | 瓶蓋 1.3 cm | `0.013` | `0.0065` | 最容易觸發 guard |

每一種先在 `0.24 / 0.00`(中心格,模擬 100%)測,再往 `0.20 / -0.09`(最差格 86.7%)測。

---

## 6. 已知未完成事項

| 項目 | 狀態 |
|---|---|
| YOLO detection 的 `height` 欄位 | **未接**。`--socket` 自動偵測路徑不能用;手動指定座標不受影響 |
| Jetson 標定 | 未做 |
| 實機 overshoot vs 8 mm margin | **從未量測過**。guard 的 8 mm 是否足夠吸收實機超調,目前無證據 |
| v22 正式流程 | 未啟動(registration / seed 818-819 / formal claim 都未建立) |

---

## 7. 部署端這輪修好了什麼(背景)

需求書 §1 指出的 TCP 定義不一致(訓練用指尖、部署用 `arm_link5` 質心,差 6.7–8.1 cm)
**已修正並驗證**,另外補了一整套安全層:

- TCP 統一到 `gripper_center`(pad 連桿中點),parity 誤差 **0.67 mm**
- 發現並修正 iGibson link merging 造成的 **2.2 cm** 座標系偏移(`URDF_TO_TRAINING_FRAME`)
- FloorGuard 改為**預防式**:送指令前先 FK 掃掠整段路徑 + 二分逼近,而非事後懲罰
- 統一 `move_guarded_and_verified` 動作原語:長距離切段、每段套 guard、讀回確認到位才更新狀態
  (靜態掃描確認 primitive 之外的 `send_degrees` 呼叫點 = **0**)
- send/read 失敗一律 fail-closed;`read_degrees` 不再用「上一次指令」當回讀值
- Stage 1 嚴格化:只有「所有 guard action = pass + feedback 有效 + 關爪因真實物體 stall」
  才可進 Stage 2。clamped / emergency_raise / emergency_hold / timeout / 讀寫失敗 / 關到底
  全部阻擋 lift
- Stage 2 改為腳本化 lift + 回 home(不再在 FK 最不可靠時繼續用 policy 動作)
- 無抓取感測器 → `_grasp_looks_real()` 三態 `confirmed` / `rejected` / `unverified`,
  dry-run 一律 `unverified`

測試:controller **38/38**、floor guard **641/641**(guard 實際 clamp 291 次,非空過)、
parity **77/77**、floor invariant 全策略成立。

---

## 8. 回報請附上

上機後不論成敗,請把以下貼回來:

1. 5.2 兩支自我驗證的**完整輸出**(38 / 641 的數字)
2. 每一次嘗試的**完整 console log**,特別是 `[Stage]` 與 `guard=[...]` 那幾行
3. 每次的 `--object-height` / `--obj-x` / `--obj-y` **實際量測值**
4. 目視結果:夾爪停在哪、有沒有碰到物體、有沒有抬起來

`guard=` 那幾行最關鍵 —— 它會直接告訴我們 8 mm 的線在實機上是太鬆還是太緊。
