# 模式 B 實測步驟：C3 起始姿勢 → 辨識 → socket 傳座標 → 夾取

目標流程（使用者定義）：

```
grasp_home_pose (v21 C3)  →  辨識到 sugarbox  →  夾取端從 socket 收到座標  →  真的夾起來
```

兩個行程：

```
電腦（GPU）                              Jetson
vision_grasp_bridge.py  ──TCP 5555──▶  x3plus_real_grasp.py --real --socket
```

---

## 為什麼一定要先做 Phase 2 校正

`integration/arm_cam_geometry.py` 裡 C3 的外參目前是 **URDF 推算的預測值**：

| | v17 nav home（實測） | v21 C3（**預測**） |
|---|---|---|
| θ | 36.40° | 89.740° |
| H | 0.332 m | 0.2148 m |
| cam_x | 0.1783 m | 0.2762 m |

C3 的地面距離只有 ±4 cm，物體 x 幾乎完全由 `cam_x` 決定。**沒校正就上機，手臂會很有
自信地夾在錯的地方。**

**但誤差是可以估的**，不是完全未知。已知證據：manifest 記載 C3 的 `gripper_center`
實測與 FK **相符到 1 mm**，而相機裝在 `arm_link4`，位於 16.7 mm 手指誤差的上游——
所以手臂幾何本身是對的，未知的只有「相機在 link 內的安裝偏移」。nav home 量到的
z 方向安裝偏移是 12.6 mm，可拿來當量級參考。模擬各項誤差對目標 x 的影響：

| 誤差來源 | 目標 x 偏移 |
|---|---|
| `cam_x` 偏 10 mm（安裝偏移量級） | **10 mm（全畫面一致的純偏移）** |
| θ 偏 2° | 7.6–8.3 mm |
| H 偏 5 mm | 0–1.5 mm（近垂直時 H 幾乎不影響）|

進場容差是 22 mm。所以**帶著預測值做一次有人監督的試夾，是有機會成功的**，
而且失敗模式是「整體固定偏移」——很好診斷。若你想先試：

```bash
# bridge 端加這個旗標即可（其餘照 §4）
python integration/vision_grasp_bridge.py --i-accept-predicted-extrinsics ...
```

**手放電源開關。** 若夾偏了，量一下物體實際的 base x，然後：
`cam_x 修正量 = 實際 x − log 印的 latch x`——**一次量測就能修掉主要誤差**，
因為 cam_x 的影響是全畫面一致的偏移。修完再跑一次。

> `cam_x`/`cam_y` 是 **policy 座標**（URDF 載入時帶 `URDF_TO_TRAINING_FRAME` 偏移）。
> `verify_camera_grasp_frame.py` 印的是原始 `base_link` 座標，x 少 19.9 mm——直接拿它
> 的數字當 `cam_x` 會讓目標落在物體後方 2 cm。`tests/test_safety_guards.py` 有測試釘住。

bridge 會拒絕送出，除非你：

- 跑完 `../calibration/CALIBRATION_PLAN.md` Phase 2（推薦），或
- 明示接受預測值：`--i-accept-predicted-extrinsics`（只建議在 §2 的空跑觀察用）

---

## 0. 桌上預檢（不碰硬體，每次上機前都跑）

```bash
cd x3plus
python integration/arm_cam_geometry.py          # 幾何自測 20 項
python grasp/v21/test_deploy_controller.py      # all 119 checks passed
python grasp/v21/test_servo_read.py             # all 37
python grasp/v21/test_deploy_floor_guard.py     # all 641
python tests/test_safety_guards.py              # Ran 53 / OK
python integration/solve_arm_cam_extrinsics.py --selftest
python integration/smoke_mode_b.py              # 模式 B 迴路 6 情境
```

`smoke_mode_b.py` 會在本機把整條 socket 路徑跑一遍（dry-run，不動硬體），涵蓋：
正常latch、偵測過期、完全沒偵測、姿勢戳記不符、超出可及範圍、缺 height。**這一步抓到的問題，
在機器上全部長得一樣（手臂就是不動或夾錯地方），到現場才查很貴。**

---

## 1. Jetson 端準備

```bash
# 找出 Jetson IP 並設好（IP 是 DHCP 動態）
.\set_jetson_host.ps1 <IP>

# Jetson 上：確認出廠程式沒佔住相機／序列埠
sudo fuser /dev/video0 /dev/video1 /dev/myserial
# 若是 rosmaster_main.py 佔住 → kill <pid>（非 systemd，不會自動重啟）

bash grasp/v21/jetson_verify.sh     # 需 119 / 37 / 641 一字不差
```

---

## 2. 校正（Phase 2，第一次上機必做，約 2 小時）

依 `../calibration/CALIBRATION_PLAN.md` Phase 3 步驟 1 建 base 基準點，再做 Phase 2。

手臂開到 C3 並停住：

```bash
python3 grasp/v21/x3plus_real_grasp.py --real --unlock-candidate-real \
  --model models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental --max-steps 0
```

**先在桌上演練一次**（不用機器、不用相機，把流程和按鍵走一遍）：

```bash
python integration/capture_arm_cam_obs.py --simulate --h 0.2170 --out rehearsal.json
```

尺量 H（鏡頭中心到地面），然後開採集工具。**你只要擺物體、輸入座標、按 Enter**，
它會取 20 幀的中位數像素（壓掉像素雜訊，直接改善擬合），每筆即時存檔可中斷續做，
並自動留 2 點不參與擬合：

```bash
python integration/capture_arm_cam_obs.py --h <尺量的H> --solve \
  --stream http://<JETSON_IP>:8080/stream?topic=/arm_cam/image_raw \
  --out c3_calib.json
```

擺 **≥5 點（建議 8 點）**。⚠ **可用範圍比你想的小很多**——工具一開始就會印出來：

| 物體高度 | 完整入鏡的 base x | base y（在 x 中段量）|
|---|---|---|
| **sugarbox 6.5 cm** | **0.239 – 0.292 m** | **−0.028 – +0.052 m** |
| 3 cm | 0.227 – 0.299 | −0.036 – +0.068 |
| 平面標記 | 0.218 – 0.304 | −0.042 – +0.081 |

近垂直視角下，物體的**頂面投影得比底面遠**，所以輪廓比它的落地面積大，越高的物體越
早出畫面。擺到範圍外會被「貼邊」檢查擋掉、根本不會記錄（工具會當場告訴你）。
左右也**不對稱**，因為主點在 640 寬影像的 x=212 而不是 320。

⚠ **不要只擺 3 點**：模擬顯示 3 點的保留點誤差中位數只有 1.4 mm，但 **p95 是 33 mm**——
偶爾會嚴重解錯，而且殘差看起來完全正常。工具會在少於 5 點時警告。

**過關**：擬合殘差 ≤ 10 mm，**保留點誤差 ≤ 10 mm**，`sign_y` 有被資料決定。

> 演練實測（模擬 1.5 px 雜訊、投影真實盒子）：sugarbox 只有 5.3 cm 的前後跨距可用，
> 因此擬合比理想情況鬆一些——保留點誤差中位 **3.3 mm**、p95 **5.7 mm**（5 點）。
> 仍遠在 10 mm 關卡與 22 mm 進場容差之內。

**不要拿解出來的 θ 去對 89.740。** 可視帶太窄，θ 與 cam_x 會互相補償——同一次演練
解出的 θ 差 2.26°、cam_x 差 8.5 mm，預測卻仍在 1 mm 內。**看殘差與保留點，不看參數。**

---

## 3. 靜態驗證（先不動手臂）

```bash
# Jetson：只開 socket，不裝模型也行——這步只看座標對不對
python3 integration/vision_grasp_bridge.py --dry-run --show \
  --cam-theta <θ> --cam-h <H> --cam-x <cam_x> --cam-y <cam_y> --sign-y <±1> \
  --class-height sugarbox=0.065
```

把 sugarbox 放在量好的位置，檢查印出的 `x`/`y`：

- [ ] x 落在 **0.239–0.292**（6.5 cm 物體完整入鏡的範圍），與實擺差 < 1 cm
- [ ] **物體往左移，y 要變大**（往右變小）。反了就把 `--sign-y` 改號重解
- [ ] `w` 約 **2.3 cm**（sugarbox 實量寬度；已扣掉頂面外擴，未修正前會讀成 2.8）
- [ ] `height` 是 `0.065`，`z` 是 `0.0325`（bridge 會自動由 height 推 z）
- [ ] `cam_pose_name` 是 `v21_c3_grasp_home`

---

## 4. 首次實機夾取

**兩個視窗，順序不能顛倒。**

**視窗 A（先開，Jetson）** — 夾取端。它會先走到 C3，站定後才 latch：

```bash
cd ~/Documents/deploy_jetson2/grasp/v21
python3 x3plus_real_grasp.py --real --socket --latch-obj \
  --i-confirm-external-frame --unlock-candidate-real \
  --model models/candidate_v21_seed816_ckpt550000.zip \
  --vecnorm models/candidate_v21_seed816_ckpt550000_vec.pkl \
  --contract obs_28_incremental \
  --latch-wait 10 --stale-timeout 1.0 --max-steps 300
```

**視窗 B（看到 `Listening on 0.0.0.0:5555` 後再開，電腦端）** — 辨識端，**持續送、不要用 `--once`**：

```bash
python integration/vision_grasp_bridge.py --host <JETSON_IP> \
  --cam-theta <θ> --cam-h <H> --cam-x <cam_x> --cam-y <cam_y> --sign-y <±1> \
  --class-height sugarbox=0.065 --show
```

> `--once` 送一筆就結束。夾取端要先走到 C3（實機要幾秒）才 latch，那時這筆早就
> 超過 `--stale-timeout` 被判過期了，結果是 latch 逾時中止。

**手放電源開關。** manifest 仍是 `candidate`，`--unlock-candidate-real` 是有人在旁監督的實驗。

### 預期 log 順序

```
[Start] Moving to grasp start pose [90.0, 67.08, 9.79, 9.79, 90.0, 30.0]...
[Start] home: reached=True (arrived, N steps)
[DetectionReceiver] New detection ... cam_pose=v21_c3_grasp_home
[Latch] cam_pose stamp v21_c3_grasp_home agrees with the encoders (worst joint off by 0.xx deg)
[Latch] Object latched at home pose: pos=[...] height=0.065m
[Init] episode wrist_z_offset = 0.05xx m
... Stage 0 對位 → Stage 1 收夾 → Stage 2 抬起返家
```

### 檢查點

- [ ] latch 的 pos 與實擺位置差 < 2 cm
- [ ] `wrist_z_offset ≈ 0.0565`（sugarbox 6.5 cm 的預期值）
- [ ] Stage 1 收夾時 S6 從 30° 往 180° 走，**停在約 164–165°**（2.3 cm 寬的預測接觸角）
- [ ] S6 停住 = 夾到了。若走到 180° 表示夾空
- [ ] Stage 2 抬起時夾爪保持閉合，物體沒掉
- [ ] 結束時 `[Guard] policy-step floor guard: {'pass': N}`，沒有 clamp

---

## 5. 出問題時看哪裡

| 症狀 | 原因 | 處置 |
|---|---|---|
| `no fresh socket detection arrived before latch timeout` | bridge 沒在送、或 IP/port 錯、或用了 `--once` | 確認視窗 B 有在印 `sent`；拉大 `--latch-wait` |
| `[Latch] REFUSED: detection cam_pose does not match the arm` | bridge 的幾何屬於別的姿勢 | 看 log 印的兩組角度；bridge 要用 C3 的外參 |
| `REFUSED: arm-camera extrinsics ... are not measured` | 還沒做 Phase 2 | 去做 §2，或暫時加 `--i-accept-predicted-extrinsics` |
| bridge 一直印 `dropped ... unusable geometry` | θ 錯得離譜，射線打不到地面 | 重看 §2 的殘差 |
| bridge 完全不送但畫面有框 | 物體超過 `--max-width`（預設 6 cm） | log 會寫 `ungraspable` 與實測寬度 |
| bridge 印 `bbox touches the ... frame edge` | 物體只有一部分在畫面裡，bbox 被截斷 | 把物體往畫面中央挪。6.5 cm 物體完整入鏡的範圍只有 x 0.239–0.292 |
| bridge 印 `[bottom-edge]` 而不是 `[centroid]` | 沒給 `--class-height` | 補上。近垂直視角下底邊法會偏 −11～−45 mm |
| bridge 印 `outside the policy's evaluated range` | 物體不在 policy 訓練過的 x 0.20–0.33 / y ±0.10 內 | 把物體或車子挪進範圍。相機看得到 x 0.200–0.318，比訓練區還往外探，所以畫面最底那排就已經在邊界上 |
| 夾取端印 `[SAFETY] REFUSED: target is outside the region` | 同上，但 bridge 沒擋到（例如外參偏移） | 先回 §3 確認座標對，再重擺 |
| latch 位置固定偏移 | base 基準點記錯 | 回 `../calibration/CALIBRATION_PLAN.md` Phase 3 步驟 1 |
| 左右反了 | `sign_y` 反號 | `--sign-y` 改號，重跑 §3 |
| S6 走到 180° 沒停 | 夾空，或 latch 位置偏了 | 先看 latch 位置；再看 Stage 0 的 `centred`/`pads_ready` |

---

## 6. 尚未驗證的項目（誠實清單）

- **辨識模型從未在 C3 視角驗證過**。`data.yaml` 記的 mAP@50-95 = 0.995 是在它自己的
  val split 上——410 張是由較少原圖增強來的，train/val 很可能有同一張的變體，那個數字
  接近上限、不代表實地準確率。更關鍵的是它完全不涵蓋 C3 視角：手臂相機從 21 cm 幾乎
  垂直俯視，盒子主要露出頂面且佔畫面很大一塊。**若實測偵測不穩，先從這裡查**，
  解法是補該姿勢的訓練影像，不是調信心閾值。

- C3 外參**從未實機量測**——§2 就是為了補上這件事
- 模式 B 在 v21 上**從未跑過實機**。2026-07-31 那次成功是用 `--obj-x/y/z` 手動給座標
- `manifest.json` 的 `status` 仍是 `candidate`，硬體 gate 未全 true：
  `guard_margin_8mm_validated_on_hardware`、`c3_real_reach_envelope`、`object_heights_measured`
- 20.5 mm 的模型嚙合深度扣掉 16.7 mm URDF 手指誤差後只剩 ~3.8 mm（頂緣夾），
  但 7/31 用 3 cm 物體成功時接觸記錄在 151°，與這個算式矛盾——**尚未解決**
