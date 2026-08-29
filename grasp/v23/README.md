# grasp/v23 — E1 高姿態夾取（測試中，**尚未合併**）

這是 v23 的實機測試包。跟 v21 唯一的實質差別是**手臂起始姿勢**與**權重**：

| | v21 | v23 |
|---|---|---|
| grasp home | C3 = API (90, **67.08**, **9.79**, **9.79**, 90, 30) | E1 = API (90, **74.2**, **8.6**, **8.6**, 90, 30) |
| 權重 | `candidate_v21_seed816_ckpt550000` | `candidate_v23_seed23401_ckpt250000` |
| 相機高度（training frame） | 0.2183 m | 0.2340 m（+1.58 cm）|
| gripper_center（張爪） | 0.1439 m | 0.1558 m（+1.19 cm）|
| 光軸離鉛垂 | 3.34°（偏後）| 1.40°（**偏前**，過了鉛垂線）|
| 可用 FOV | ~70 cm² | ~101 cm²（+44%）|
| 訓練 spawn band | x 0.20–0.33、y ±0.10 | x 0.205–0.280、y −0.070–+0.065 |

契約沒變（`obs_28_incremental`、28D/6D、incremental arm + absolute gripper），
`deploy_contract.py`、`action_execution_v21.py`、URDF 都與 v21 逐位元組相同。

---

## ⚠ 這個包的腳本不是交付包裡那一支

`grasp/deploy_v23/x3plus_real_grasp.py` **沒有**被採用。它跟
`grasp/v21/reference/x3plus_real_grasp.release_branch.py` 除了三個 hunk
（model 路徑、`home_deg`、`home_deg` 的註解）以外**逐位元組相同**，而那支在
`grasp/v21/reference/README.md` 裡標的是「**參考副本 — 不要執行**」：它在
2026-07-31 之前分支出去，缺少讓 v21 真的夾得起來的東西。

| 缺什麼 | 後果 |
|---|---|
| `_park_jaw_hold` / `jaw_contact` | 夾爪一路推到 180° 全閉停點並維持近堵轉扭矩（2026-07-31 實機聽得到齒輪研磨）|
| `_check_target_envelope` | 相機看得到、但策略沒訓練過的目標不會被擋 |
| 認 ID 的半雙工匯流排讀取 | 一個遲到封包讓整條讀取管線錯位 |
| manifest 發佈閘 | `--real` 不再驗權重 sha256 |
| 12 個 CLI 旗標 | 其中 `--latch-obj`、`--i-confirm-external-frame`、`--unlock-candidate-real`、`--pose-tol-deg`、`--stale-timeout` 是 `jetson_one_command_grasp.py` 在傳的 —— 沒有它們，一鍵流程根本驅動不了 |

所以本目錄用的是 **v21 那支 3268 行的硬體版**，只改了 `home_deg`、model 路徑、
target envelope 三處。交付包留在 `grasp/deploy_v23/` 原封不動，當對照用。

如果之後要 v23 的 `--release-to-bin` / `--release-only`，請照 2026-08-04 移植
release 動作的做法把旗標搬過來，**不要**整支換回去。

---

## 上機順序

### 0. 離線前置檢查（不碰硬體）

```bash
cd grasp/v23 && ./jetson_verify.sh
```

要看到 **119 / 37 / 641 / 42** 一字不差，`wrist_z_offset = 0.0564`。

> `wrist_z_offset` 錨在**地板**不是手臂（`dc.hover_gripper_center_z`），所以
> C3→E1 這個值不變。它要是動了，代表變的不是姿勢。

### 1. 量 E1 的外參（**必做，沒有捷徑**）

C3 的校正檔在 E1 是錯的，這是幾何上的必然：相機升高 1.6 cm，光軸還跨過了鉛垂線。
兩個檔案都是合法 JSON、都過同一道 ≥6 點／<2 cm 閘、也都不記錄自己是在哪個姿勢量的，
**唯一**把它們分開的是檔名不同：

- v21 → `integration/grasp_home_homography.json`
- v23 → `integration/grasp_home_homography_e1.json`

```bash
source ~/grasp_venv/bin/activate
cd grasp/v23
python3 jetson_one_command_grasp.py --calibrate
```

手臂會走到 E1 並停在那裡不動，相機持續印
`[bridge][calibration] {"u": ..., "v": ..., ...}`。每擺一個位置就等它印一行，
把裡面的 `u,v` 跟你用尺量的 base `(x, y)` 記成一組。

**至少 6 組不共線**、覆蓋四周與中央，另外留 **2 組**不要拿去擬合當驗證點。
擺放位置要落在 v23 的 band 內：**x 0.205–0.280 m、y −0.070–+0.065 m**。

Ctrl+C 之後：

```bash
python3 ../../integration/grasp_home_homography.py \
  --points-json grasp_home_points_e1.json \
  --output ../../integration/grasp_home_homography_e1.json \
  --max-rmse-cm 1
```

擬合 RMSE 與兩個保留點的 x/y 誤差都要 ≤1 cm。驗收：

```bash
python3 jetson_one_command_grasp.py --check
```

> **為什麼 E1 只能走 homography。** `arm_cam_geometry` 的三角測距要除以光線與地面
> 夾角的正切；相機越接近鉛垂，回推的橫向偏移越是趨近 0、深度對 theta 誤差越敏感。
> C3 的 3.34° 已經在這個區間邊緣，E1 的 1.40° 更深。`vision_grasp_bridge` 在
> grasp-home 直接拒收非 homography 的映射，`--i-accept-predicted-extrinsics` 也繞不過。

### 2. 空跑

```bash
python3 x3plus_real_grasp.py \
  --model models/candidate_v23_seed23401_ckpt250000.zip \
  --vecnorm models/candidate_v23_seed23401_ckpt250000_vec.pkl \
  --contract obs_28_incremental \
  --object-height 0.065 --obj-x 0.24 --obj-y 0.00 --obj-z 0.0325
```

### 3. 定點實機（先固定座標，還沒接視覺）

同上加 `--real --unlock-candidate-real`。人在旁邊、手放電源開關 ——
**網頁上的停止鈕不是急停，電源開關才是。**

manifest `status` 是 `candidate`。目前只有 E1 homography gate 已完成；動作、可達範圍、
dry-run 與實抓 gate 都還沒過（v21 的實機結果是在 C3，不能繼承）。

### 4. 一鍵：辨識＋夾取

```bash
python3 jetson_one_command_grasp.py --check    # 不碰硬體
python3 jetson_one_command_grasp.py            # 正式
```

---

## 姿勢還沒定案 — `pose_explorer.py`

E1 的實測結果是：6.5 cm 的 sugarbox 只能擺在**前 2.0–5.5、左 2.0–右 5.5 cm**
（約 26 cm²），而策略的訓練 band 是 101 cm²。視覺最多只能餵給它三分之一。

這不是校正沒做好，是姿勢的幾何。盒子 6.5 cm 高、相機 23 cm 高幾乎垂直往下看，
盒子**頂面**的投影比底面遠 1.4 倍，所以 `bbox_touches_border` 會在接地點還離
地面視野邊界很遠時就先拒收。**物體高度是這裡最大的變數**，不是相機解析度。

```bash
python3 pose_explorer.py --list                    # 只印候選表，不碰硬體
python3 pose_explorer.py --check                   # 檢查相機/序列埠/借用契約
python3 pose_explorer.py --i-am-beside-the-robot   # 實際走一遍
```

相機**整場只開一次**，手臂在候選姿勢之間移動，每個姿勢存一張標註過的快照到
`~/pose_explorer/`（`--show` 可另開即時視窗，需要顯示器）。快照上疊的是
**預測**的地面格線（2 cm 一格）與光軸十字 —— 光軸在 u=212 而不是 320，這就是
為什麼往右的可用範圍是往左的兩倍。

`k` 記下你看上的姿勢，`q` 離開時會回 E1 並把清單寫進 `~/pose_explorer/session.json`。
把那份清單給訓練端當新的 grasp home。

⚠ **表格裡的面積是預測值，會高估。** E1 預測遠端到 x=0.305，實測前 6.0（x=0.2887）
就撞上緣了，差 2 cm。用眼睛看，不要看數字。

⚠ **換姿勢＝重訓。** 這支只幫你選，選完的姿勢不能直接部署 —— 策略是從特定
home 訓練出來的。

---

## 合併回 main 的條件

這個分支要合併，下面每一項都要成立（也就是 `manifest.json` 的
`hardware_gates` 必須逐項有實測證據）：

- [x] `e1_homography_measured` — 2026-08-29：7 點 RMSE 0.388 cm；2 個保留點最大軸誤差 0.448 cm
- [ ] `e1_fov_ruler_check` — E1 放尺量，確認 x 約 13 cm、y 約 19 cm
- [ ] `e1_gripper_center_height_ruler_check` — FK 說 15.58 cm（張爪），量出來對得上
- [ ] `jetson_dry_run_ok` — 119/37/641/42 + `wrist_z_offset = 0.0564`
- [ ] `first_real_grasp_logged` — E1 至少一次實機夾起來，留完整 log

沒過就留在分支上。v21 完全沒被動到，`grasp/v21/` 仍是唯一有實機夾取紀錄的那一套
（2026-07-31），模式 A / B / C 也都還指著它。

---

## 已知會踩到的東西

**(0.28, −0.070) 這一格訓練只有 73.3%**，低於 80% 的單格門檻，hotspot 加了但 RL 還沒重跑。
那是 band 的右後角，envelope guard **會**放行。在那裡失敗是預期內的，不是新 bug。

**URDF 手指比實體長 16.7 mm**（2026-07-31 在 C3 量的，沒修、也還沒在 E1 重量）。
紙球、瓶蓋這類矮物體照樣夾不到。

**姿勢重現性 ±2°** 會帶來約 1.6 cm 的不裁切深度變動。固定 homography 沒辦法自適應，
所以每筆偵測都會重新核對手臂姿勢（`--pose-tol-deg`）。

**沒有訓練 metadata、沒有 git_head、沒有正式評估**隨包附上，這次訓練無法從
記錄重現。要離開 `candidate` 之前先跟訓練端要 v21 那種
`evidence/*_training_metadata.json`。

**模式 A / C 沒有接 v23。** `integration/vision_grasp_pipeline.py` 與
`nav_rl_grasp_pipeline.py` 仍透過 `_load_grasp_module()` 載入 `grasp/v21/`，
跑的還是 C3 策略。導航停止距離要加的 +1.3 cm 也還沒套進任何導航程式。

細節、每個數字的來源、以及各項 gate 的定義都在 `manifest.json`。
