# grasp/v21 — v21 candidate 獨立部署棧

> **狀態：`candidate`。沒有任何實機證據。`--real` 前請先讀完 `manifest.json` 的 `hardware_gates`。**

## 這是什麼

這**不是**「一組新權重」，而是一整套自成一格的部署程式。它和 `grasp/x3plus_real_grasp.py`
是兩條不同的血脈，**不能互相混用**。

放在這裡的目的就是隔離：v21 出問題不會波及現有流程，要退回只要刪掉 `grasp/v21/`。

## 🔴 絕對不要做的事

**不要把 `models/` 裡的權重丟給 `grasp/x3plus_real_grasp.py` 跑。**

那支 script 把手臂動作解讀成 **absolute**（normalized action 直接映射到關節上下限），
但這組權重是 **incremental**（`desired[i] = current[i] + action[i] × 0.08 rad`）。

兩者的 observation 都是 28D、action 都是 6D，所以**程式裡每一道 shape 檢查都抓不到這個錯**
（`deploy_contract.py` 自己就寫明了這點）。結果是手臂會以全幅度被送到錯誤位置，而且不會報錯。

除此之外還有三個不相容：

| | `grasp/x3plus_real_grasp.py`（現行） | `grasp/v21/`（本目錄） |
|---|---|---|
| TCP | `arm_link5` 原點 | `gripper_center`（兩指 pad 中點），**高 6.7–8.1 cm** |
| FK base | `[0, 0, 0]` | `URDF_TO_TRAINING_FRAME`，**差約 2 cm** |
| grasp home | `(90, 32.704, 9.786, 32.704, 90, 30)` | **C3** `(90, 67.08, 9.79, 9.79, 90, 30)` |

FK base 差 2 cm 代表**現有的相機標定值不能直接沿用**——同一組 `--obj-x/--obj-y`
在兩邊代表的相對幾何是不一樣的。

## 檔案

| 檔案 | 說明 |
|---|---|
| `x3plus_real_grasp.py` | 主部署程式（**唯一一處為 repo 改過的地方**：`urdf_path` 指向 `../x3plus/yahboomcar.urdf`） |
| `deploy_contract.py` | 契約定義 + fail-closed 驗證 |
| `action_execution_v21.py` | 動作執行狀態 |
| `test_deploy_controller.py` / `test_servo_read.py` / `test_deploy_floor_guard.py` | 自我驗證（93 / 37 / 641），不需硬體 |
| `bus_probe.py` | 伺服機匯流排診斷，**唯讀、不送任何指令、手臂不會動** |
| `pose_check.py` | C3 夾爪幾何量測，只驅動 S6，手臂不動 |
| `models/` | 權重 + VecNormalize（**成對使用，不可拆開**） |
| `manifest.json` | SHA256、契約、姿態、驗收數字、hardware gates |
| `HANDOFF.md` | 訓練端原始交接文件 |
| `evidence/` | 235 cases 逐筆結果、selection lock、訓練 metadata（不參與執行） |

**URDF 與 mesh 沿用 `grasp/x3plus/`**，不另外複製。已逐檔比對過與訓練端封包 byte-identical
（git blob sha256 `0a43968783b1…`）。注意 Windows 上 `core.autocrlf=true` 會讓工作檔的
sha256 看起來不一樣，那是假警報——要比就比 `git cat-file blob`。

> **⚠️ 加新權重時的陷阱**：repo 根目錄的 `.gitignore` 第 29 行有 `*.zip`（大型二進位檔的
> 通則排除）。模型權重會被**靜默略過**，`git status` 也不會提醒你。repo 既有的
> `trained_6d_models_v17/` 與 `integration/nav_best_model/` 都是靠 `git add -f` 進來的，
> 這裡的 `.zip` 也是。加 v22 時記得 `git add -f`，並且 commit 後**核對 `git show --stat`
> 裡真的有那個 `.zip`**——否則 clone 出來會是一套沒有權重的程式，直到 Jetson 上跑才炸。
> （`.pkl` 不受影響，不在排除清單裡。）

## 硬體驅動（2026-07-30 修正）

交接包原本 import 的是 **`Arm_Lib`（DOFBOT 的舊驅動）**，X3Plus 根本沒有這個模組。
更糟的是 `ServoController.__init__` 寫成 `dry_run = dry_run or (not HAS_ARM_LIB)`——
Jetson 上 import 失敗就**靜默把 `--real` 變成只印不動**，跑完一輪看起來像成功但手臂沒動過。

已移植到 `Rosmaster_Lib`（只換傳輸層，數學/契約/guard/狀態機一行沒動，
因為角度慣例 `api = 90 + sim_deg` 本來就是照 Rosmaster 寫的）。

現在的 fail-closed 行為：

| 情況 | 行為 |
|---|---|
| `--real` 但驅動不可 import | `main()` **載入模型前**就拒絕，exit 2 |
| 驅動缺席但 `dry_run=False` | **不會**變成 dry-run；寫入被拒、讀取回傳 invalid、追蹤姿態不前進 |
| 板子會靜默丟棄的角度 | 送出前擋下（S1–S4 0–180、S5 0–270、S6 0–180） |
| 送出時間 >2000ms | clamp 到 2000（板子上限） |

Jetson 上驅動放在 **`grasp/Rosmaster_Lib`**（照 CLAUDE.md 的 `cp -r` 步驟）。
`grasp/v21/` 會自動把上層 `grasp/` 加進 `sys.path`，**不需要**在 v21 裡再複製一份。

序列埠預設 `/dev/myserial`（ch341 的 udev 穩定符號連結），用 `--port` 覆蓋。

⚠️ 執行前確認 Jetson **沒有跑出廠 `rosmaster_main.py`、ROS 底盤 driver 或 port 7000
motor server**——它們會佔住序列埠。

## 伺服機讀取：Rosmaster_Lib 的 ID 錯位缺陷（2026-07-31）

三次實跑分別死在 **S1（第 1 步）**、**S6（第 49 步）**、**S3+S4（啟動第一次讀取）**。
最後那次還沒送出任何指令，所以「跟自己的寫入撞在一起」被排除掉了。

原因在驅動原始碼裡。`Rosmaster_Lib.get_uart_servo_value`：

```python
self.__read_id = 0
self.__request_data(self.FUNC_UART_SERVO, int(servo_id) & 0xff)
timeout = 30
while timeout > 0:
    if self.__read_id > 0:
        return self.__read_id, self.__read_val   # ← 不檢查是不是你問的那顆
```

它回傳**第一個抵達的回應**，不比對請求的 `servo_id`。接著 `get_uart_servo_angle`
自己比對 ID、對不上就回 `-1`——**把一個完全正確的讀值丟掉，並且冤枉一顆有回答的伺服機**。

一個遲到的回應會讓整條管線錯位一格：S2 的答案被 S3 的請求接走 → S3 回 -1，
而 S3 的答案又被 S4 接走。**症狀就是相鄰數顆同時「讀不到」**，正好是 S3+S4。
這也解釋了為什麼重試沒用——每次重試都重新發問，管線一直保持錯位。

修法：`_read_one()` 改叫 `get_uart_servo_value`，**把讀值記到回應上標示的那顆伺服機頭上**，
而不是記到被問的那顆。錯位變成多一次來回就自動修正。原始值→角度的換算逐字複製自
`__arm_convert_angle`（含 `int(x + 0.5)` 與超出範圍的拒絕），所以數字與走驅動完全相同。

`[Bus]` 那行現在也會回報**已修復的錯位次數**——這個數字區分「匯流排沒事」與
「匯流排在錯位、只是讀取層吸收掉了」，只有後者會在負載下惡化。

### 出事時先跑 `bus_probe.py`

```bash
python3 bus_probe.py            # 50 輪，唯讀
python3 bus_probe.py --gap-ms 20   # 拉開間隔；錯位歸零就代表 --bus-quiet-ms 有效
```

它把驅動壓成同一個 `-1` 的三種故障拆開，而這三種的處置完全不同：

| 分類 | 意義 | 處置 |
|---|---|---|
| **misdirected** | 有回應，但貼著別顆的 ID | 上面的缺陷，讀取層已能修復 |
| **timeout** | 完全沒有回應 | 接線／接頭／供電，**重試救不回來** |
| **out of range** | 有回應但解碼成不可能的角度 | 封包損壞或伺服機亂回報 |

**失敗當下立刻跑，不要先關電源**——關機把證據清掉了。

## 夾爪接觸處理（2026-07-31）

實機觀察：物體被夾住後，S6 **仍持續朝 180° 全閉走**，齒輪發出研磨聲。原因是限速器
從「上次指令」推進、不看讀回值——夾爪被物體擋在 ~135° 時指令照走到 180，伺服機
扛著 ~45° 的位置誤差＝近堵轉扭力，直到整輪結束。**持續堵轉會剝齒。**

這顆伺服機的夾持力**就是**指令與實際角度的差：差 0 = 沒有夾持力（抬升時物體滑出），
差到底 = 研磨。兩個極端都是錯的，修法是把這個差管理起來：

- **接觸偵測**：指令在前進、編碼器沒跟上（連續 2 次）→ 判定夾到東西
- **停在接觸角 + `jaw_hold_bias_deg`（預設 8°）**：約 3mm 的虛擬擠壓量，穩固夾持
  但遠離堵轉。這是夾持力旋鈕——抬升會滑就調大
- **Stage 2 全程維持同一 hold**：抬升與回家的夾爪目標都是 hold，不是讀回值
  （讀回值＝接觸角＝零夾持力），到位判定只看手臂五軸
- 空夾照舊走到 180 → ABORT（「什麼都沒夾到」的判定不變）

**8° 這個 bias 還沒在實機驗證過會不會滑**——下一次真的夾住的那輪就是答案。

## 懸停後備與超時救援（2026-07-31，第 5 跑）

實機觀察：策略在 Stage 0 自己把物體夾住（S6=138），然後在 `target_dist=27mm` 懸停
270 步——**離 26mm 進場半徑只差 1mm**——超時後照規則開爪回家，把到手的物體放掉。

三個修正，主閘門一個數字都沒動：

1. **懸停後備**：xy、z、pads_ready 全部真的過、只有半徑超出 ≤5mm，且連續 30 步
   （3 秒）→ 進 Stage 1。還在接近中的策略一兩步就穿過這個帶，累積不起來；
   停在平衡點的策略才會累積——3 秒的平衡跟到位沒有實質差別。
2. **超時救援**：max-steps 到時若夾爪閉合超過 50%，先試一次 scripted close 再開爪。
   空夾照舊走到 180 → ABORT，多花幾秒；有東西就變成夾取而不是放生。
3. **遙測**：Stage 0 每 25 步印一行 `[Gate]`（xy/z/r/pads 各差多少）＋ `[Guard]`
   guard 動作統計印在每個出口——第 5 跑只有 `target_dist` 一個數字，
   卡在哪一項、guard 有沒有擋，事後完全查不到。

另修：Stage 2 結束後不再落入「回 home + 開爪」的共用結尾——**夾取成功後夾爪保持
hold（伺服機通電就會維持位置），把物體拿走再 Ctrl+C**；抬升失敗的凍結也不再被
自動回 home 蓋掉。

## Guard 死鎖與手指誤差（2026-07-31，第 6 跑）

同一條指令跑兩次得到兩種結局，這本身就是結論之一：**編碼器只有 1° 解析度，
incremental 策略會把起始位姿的 1–2° 差異放大成完全不同的軌跡。** 這不是 bug，
是這套硬體＋incremental 契約的固有性質；判讀實跑時不能只看單次。

第二次跑撞上真正的缺陷——**guard 死鎖**：

```
[floor guard] emergency_raise {'z_now': 0.007413693381111391, ...}   ← 42 次，數值完全一樣
```

夾爪提早閉合 → `close_drop` 把模型的 pad 拉低到 7.4mm → **8mm 安全線正好落在
S2 相鄰兩個編碼器刻度之間**。策略命令往下（S2=21）、guard 抬起一格（S2=22）、
策略再往下……**213 步、零進展，看起來很忙其實完全卡死**，直到你 Ctrl+C。

`emergency_raise_max_iters=12` 擋不住：policy step 用 `max_iters=1` 呼叫，
那個計數器每步都重置。現在加了 **episode 層級**的偵測——連續 25 步介入且
clearance 沒有增加 → 停下來報告死鎖，而不是把剩下的步數燒完。

### 你量到的 1.5cm

guard 說 7.4mm、你的尺說約 15mm。原因就是 **URDF 手指比實體長 16.7mm**——
guard 相信 pad 在比實際低 1.67cm 的地方，所以提早 1.67cm 就喊停。

`[floor guard]` 現在會多印 `low_link` / `low_is_pad`：**最低的是不是指墊**決定了
這 16.7mm 該不該套用（是指墊才算，是別的連桿件就不算）。死鎖訊息也會直接算給你看。

新旗標（**預設 0 = 現行保守行為，不動安全性**）：

```bash
--floor-finger-error-mm 16.7    # 只修正 guard 幾何裡的兩片指墊
```

它只抬高 guard 眼中的指墊，不動訓練用的 `min_gripper_link_z` 預設值、不動策略觀測。
範圍限制 [0, 30]，打錯成 167 會直接 exit 2。

> **⚠️ 先用尺確認再開。** 這個值直接決定手指離地面多近才踩煞車，給太大就是把手指
> 開進地板。建議：先跑一次看 `low_is_pad` 是不是 `True`，再量一次實際離地高度，
> 對得起來才用。

## `--real` 安全閘（2026-07-30 新增）

`--real` 在**開序列埠、載模型之前**先過三關，全部以 `manifest.json` 為準：

| 關卡 | 擋什麼 | 可否豁免 |
|---|---|---|
| sha256 | `--model` / `--vecnorm` 不是本包記載的那兩個檔（含成對規則、scp 傳壞） | ❌ 不可 |
| contract | `--contract` 與 manifest 不符（例如誤填 `obs_28_absolute`） | ❌ 不可 |
| status | `status != hardware-approved`（本包是 `candidate`） | ✅ `--unlock-candidate-real` |

前兩關不可豁免是刻意的：沒有任何正當理由用「不是本包記載」的權重去驅動手臂。
第三關可豁免，因為 gates 本來就要靠實機跑才翻得成 true——但豁免＝**人在旁邊、手放電源**。

被擋下時 exit **3**（驅動不可 import 是 exit 2）。dry-run 完全不受這三關影響，
驗證永遠免費。

```bash
# candidate 階段的實跑（人在場、手放電源）
python x3plus_real_grasp.py \
  --model models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental \
  --object-height <卡尺實測> --obj-x 0.24 --obj-y 0.00 --obj-z <height/2> \
  --real --unlock-candidate-real
```

gates 全部翻 true 之後，把 `manifest.json` 的 `status` 改成 `hardware-approved`，
`--unlock-candidate-real` 就不再需要。

## 怎麼跑

```bash
cd grasp/v21

# 1. 自我驗證（先做，數字不對就不要往下）
PYTHONIOENCODING=utf-8 python test_deploy_controller.py    # → all 93 checks passed
PYTHONIOENCODING=utf-8 python test_servo_read.py           # → all 37 checks passed
PYTHONIOENCODING=utf-8 python test_deploy_floor_guard.py   # → all 641 checks passed, clamped 291x

# 2. Dry-run（不驅動伺服機）
python x3plus_real_grasp.py \
  --model models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental \
  --object-height 0.066 --obj-x 0.24 --obj-y 0.00 --obj-z 0.033
```

⚠️ Windows 上兩支測試一定要加 `PYTHONIOENCODING=utf-8`，否則 cp950 主控台會在中途
`UnicodeEncodeError` 中斷。那是編碼問題，不是測試失敗。

dry-run 最後出現 `Stage 1 ABORT — jaw reached the fully-closed stop` 是**正確**的：
空跑沒有實體物件，爪子關到底，狀態機拒絕宣告抓到。

## ⚠️ `--object-height` 一定要給

`HANDOFF.md` §5.4 說不給會 `MissingObjectHeightError` 擋下——**那只在 v20 的 32D contract 成立**。
`obs_28_incremental` 的 `requires_object_height=False`，不給**不會被擋**，會安靜地用
「對稱物體」假設推 `height = 2 × (obj_z − ground_z)`。

瓶蓋這種扁物最容易踩雷。**每次都實際量測，不要估。**

## 測試順序

由高到矮，因為越矮越貼近 8 mm 的 floor guard 線：

| 順序 | 物體 | `--object-height` | `--obj-z` |
|---|---|---|---|
| 1 | 木塊 | `0.066` | `0.033` |
| 2 | 紙團 | `0.035` | `0.0175` |
| 3 | 瓶蓋 | `0.013` | `0.0065` |

每種先測中心格 `0.24 / 0.00`，再測最差格 `0.20 / -0.09`。

**預期瓶蓋成功率會低於模擬的 100%**：模擬最低淨空 3.99 mm 低於部署 guard 的 8 mm 線，
guard 會介入、開爪退回 home。那是設計上的安全失敗，不是 bug。

## 量測協定：`--obj-x/--obj-y` 到底從哪裡量

**不要從機殼、也不要從 base_link 原點量。** 這套 FK 把 URDF 載在
`URDF_TO_TRAINING_FRAME`，訓練座標系原點跟你眼睛看得到的任何特徵都差約 2 cm。
唯一可靠的做法是拿手臂自己的 C3 姿態當基準。

**FK 實算（本目錄環境，訓練座標系）：**

| 量 | 值 |
|---|---|
| C3 時 `gripper_center` | `(0.2263, −0.0035, 0.1439)` |
| C3 時兩指 pad 最低點離地 | `0.1110` |

**協定：**

1. 手臂停在 C3（`--max-steps 5` 短輪即可）。
2. **尺規 gate**：量 pad 中點離地 ≈ **14.4 cm**、pad 最低點離地 ≈ **11.1 cm**。
   - ±1 cm → 過
   - 1–2 cm → 記錄下來再議
   - **>2 cm → 立刻停**。代表 TCP / frame 修正在實機不成立，後面每一夾都會系統性偏掉，
     繼續只是重複同一個錯。
3. 從 pad 中點**鉛垂投影到地面**，貼膠帶標記 **P 點**。P 在訓練座標 = `(0.2263, −0.0035)`。
4. 之後所有座標一律「P + 尺量偏移」換算：

| 目標格 | 相對 P 的擺放 |
|---|---|
| `(0.24, 0.00)` 中心格 | P **前方 1.4 cm**、橫向 0.35 cm |
| `(0.20, −0.09)` 最差格 | P **後方 2.6 cm**、橫向 8.7 cm |

⚠️ **橫向的左右方向（`+y` 是左還是右）尚未實機確認**（manifest 從 v18 起就掛著這個
open question）。中心格的橫向只有 3.5 mm，猜錯也無所謂；**最差格的 8.7 cm 必須先確認符號**——
做法是物體放 P 左 5 cm、給 `--obj-y +0.0465`，看 S1 是否向左轉，反了就停下來回報。

## 目前缺什麼

`--socket` 自動偵測**還不能用**：`integration/vision_grasp_bridge.py` 送的是
`{x, y, z, w, class, camera_pose}`，這支讀的是 `{x, y, z, height}`，`w` 會被忽略、
`height` 缺席。第一輪一律手動給座標。

其餘（Jetson SB3 載入、C3 尺規量測、reach、實機 guard 餘裕）見 `manifest.json`
的 `hardware_gates`，目前**全部是 false**。
