# Yahboom X3Plus Jetson 系統救援後上機測試與 AI 交接文件

> 建立日期：2026-07-18（Asia/Taipei）  
> 適用對象：接手協助首次開機、Jetson/ROS/硬體診斷或 X3Plus 實機部署的 AI／工程人員  
> 目前狀態：損壞磁碟已完成映像救援、離線 EXT4 修復、備份與雜湊；修復映像已用 balenaEtcher 寫回原系統碟，第二次寫入與驗證成功，但舊系統碟曾在第一次持續寫入時從 USB 匯流排消失，因此只能視為「暫時可測」，不能視為可靠的長期系統碟。

---

## 0. 給下一個 AI 的最高優先級指令

請先遵守以下規則，再要求使用者執行任何指令：

1. **一次只給一個可驗證步驟。** 使用者會手動輸入或貼上指令；曾多次因貼上出現 `^[[200~`、前置 `~`、漏掉 `/`、大小寫或把 `;` 打成 `:`。
2. **第一次上機禁止直接執行任何 `--real`、馬達伺服器或全自動 launch。** 先完成本文件的磁碟、USB、電源、SSH、裝置與 dry-run 檢查。
3. **禁止在已掛載的根檔案系統上執行 `e2fsck`。** 若根分割區再次發生 EXT4/JBD2/I/O 錯誤，應停機並離線處理，不能在 `/` 掛載中強修。
4. **禁止要求 Windows 格式化、初始化或修復 Jetson 磁碟。** Windows 不認得 EXT4 與 Jetson 小分割區，格式化會破壞系統。
5. **禁止修改或覆寫 `x3plus-original.img`。** 它是唯一的未修改原始救援證據。
6. **只要出現新的根碟 USB disconnect/reset、`I/O error`、EXT4/JBD2 錯誤、唯讀根檔案系統、Bus error 或系統執行檔讀取失敗，立刻停止實機測試。** 不要反覆重開機或反覆 fsck；應更換新系統碟、USB 線／轉接器或排查供電。
7. **輪子必須架空、機械臂周圍清空、有人可立即斷電，才能進入任何實際運動測試。**
8. **不要假設舊 IP 一定有效。** 舊 IP 是 `172.31.28.252`，但首次重啟後必須以 `hostname -I` 或路由器／網卡資訊重新確認。

---

## 1. 任務背景與目前結論

### 1.1 專案目標

本專案要將 PyBullet + Stable-Baselines3 PPO 訓練的 6D 夾取策略部署到 Yahboom X3Plus 實體機器人：

- 底盤：麥克納姆輪
- 機械臂：5-DOF + S6 夾爪
- 控制板：Rosmaster 擴充板
- 主機：Jetson Nano/T210 系列
- 驅動：`Rosmaster_Lib`
- ROS：Melodic
- 主要實機部署入口：`grasp/x3plus_real_grasp.py`

### 1.2 真正的阻塞問題

先前問題不是 PPO、ROS topic、LiDAR、TF 或馬達公式，而是 Jetson 根檔案系統與儲存路徑失效。

已確認的故障演進：

1. `/dev/sda1`（EXT4，掛載 `/`）累積大量 EXT4 錯誤，曾回報約 87,657 次錯誤。
2. `/tmp` 變成唯讀，`mktemp` 失敗。
3. 一般搜尋或執行程序出現 `Bus error`。
4. SSH 可通過驗證並建立 session，但 `/usr/bin/scp` 回報 `Input/output error`、exit 126。
5. Python 3.6 無法讀取標準函式庫，例如 `encodings/__init__.py` 回報 `OSError: [Errno 5] Input/output error`。
6. 重新開機後出現：
   - `JBD2: Invalid checksum recovering block ... in log`
   - `EXT4-fs (sda1): error loading journal`
   - `mount: /mnt: can't read superblock on /dev/sda1`
   - USB `Cannot enable`／`unable to enumerate USB device`
7. 最終無法正常進入原系統。

### 1.3 根因判斷（仍有未知）

確定有 EXT4 邏輯損壞；底層原因尚未完全證實，可能是：

- 突然斷電或不正常關機
- 舊 Generic USB 系統碟老化／控制器不穩
- USB 接點、線材、轉接器或連接埠不穩
- Jetson 供電瞬間掉壓造成 USB 根碟斷線

重要新證據：第一次用 Etcher 對舊碟做 62.9GB 持續寫入時，出現 `The writer process ended unexpectedly`，而且該 58.59GiB 磁碟一度從 Windows「磁碟管理」完全消失。拔除、冷卻、改接後才重新出現；第二次 Etcher 寫入與驗證成功。這使「舊系統碟或 USB 路徑硬體不可靠」的可能性明顯升高。

---

## 2. 已確認的原系統規格

| 項目 | 已確認內容 |
|---|---|
| Jetson 平台 | `t210ref`，Jetson Nano/T210 系列 |
| 架構 | `aarch64` |
| NVIDIA L4T | `R32.6.1` |
| 對應 JetPack | JetPack 4.6 |
| Ubuntu | Ubuntu 18.04.6 LTS（Bionic） |
| ROS | Melodic |
| 預設 `python` | Python 2.7 |
| 預設 `python3` | Python 3.6 |
| 額外 Python | Python 3.8；家目錄另有 Python 3.9 套件痕跡 |
| 實機虛擬環境 | 預期為 `~/grasp_venv`，Python 3.8 |
| 原根分割區 | `/dev/sda1`，EXT4，掛載 `/` |
| 舊系統碟 | Generic USB Disk，62.9GB decimal／58.59GiB，序號 `FC1743C7147A28FA` |
| 主要分割區 | 約 53.26GB EXT4，另有 13 個 Jetson 小分割區與約 5.32GB 未配置空間 |
| 舊 SSH 帳號 | `jetson` |
| 舊 IP | `172.31.28.252`（不可假設重開後仍相同） |

原始 NVIDIA 版本文字：

```text
# R32 (release), REVISION: 6.1, GCID: 27863751, BOARD: t210ref, EABI: aarch64, DATE: Mon Jul 26 19:20:30 UTC 2021
```

---

## 3. 已完成的救援與修復

### 3.1 ddrescue 原始映像

在驗證過的 Ubuntu Live 環境中，來源穩定路徑為：

```text
/dev/disk/by-id/usb-General_USB_Disk_FC1743C7147A28FA-0:0
```

ddrescue 第一階段結果：

```text
rescued:     62914 MB
pct rescued: 100.00%
bad-sector:  0 B
bad areas:   0
read errors: 0
run time:    33m32s
```

此結果代表當時能完整讀取所有區塊，但**不能排除寫入、過熱、控制器重置、線材或供電問題**。

### 3.2 EXT4 修復方式

1. 原始映像 `x3plus-original.img` 保持未修改。
2. 複製出 `x3plus-working.img`。
3. 僅對 working image 的第 1 分割區執行 `e2fsck -f -v -y`。
4. 修復過程發現並修正：
   - corrupted orphan inode list
   - 大量 orphan inode
   - deleted/unused inode directory entry
   - reference count 錯誤
   - block bitmap differences
   - inode bitmap checksum mismatch
   - free block/inode count 錯誤
5. 修復後再次執行 `e2fsck -f -n`，結果 `verify exit=0`。
6. 修復映像以 `ro,noload` 掛載，可正常讀取 `/home/jetson`。

### 3.3 寫回舊系統碟

- 寫入來源：`x3plus-working.img`
- 寫入目標：原 Generic USB Device 62.9GB
- 工具：balenaEtcher
- 第一次：writer process 異常結束，目標磁碟從 Windows 消失
- 第二次：更換／重新建立 USB 連線後，Etcher 顯示綠色「燒錄成功」，並完成內建驗證

**目前上機用的是修復後 working image，不是未修復 original image。**

---

## 4. Windows 救援檔案與雜湊（不可刪除）

備份位置：

```text
D:\x3plus-recovery
```

核心檔案：

| 檔案 | 約略大小 | 用途 |
|---|---:|---|
| `x3plus-original.img` | 59GB | 原始、未修改的完整磁碟映像；永不直接修改 |
| `x3plus-working.img` | 59GB | 已完成 e2fsck 修復、已寫回舊碟的工作映像 |
| `jetson-home.tar` | 12GB | 完整 `/home/jetson`，保留 Linux owner/ACL/xattr |
| `jetson-system-config.tar` | 2.4GB | `/etc`、Python 3.6 system packages、dpkg status |
| `x3plus-critical.tar` | 736MB | 方便快速解壓的專案、snapshot、Motor/ROS 關鍵資料 |
| `SHA256SUMS.txt` | 小型文字檔 | 五個核心檔案的統一雜湊清單 |
| `x3plus-rescue.map` | 小型文字檔 | ddrescue mapfile |
| `e2fsck-before.txt` | 小型文字檔 | 修復前檢查紀錄 |
| `e2fsck-repair.txt` | 小型文字檔 | 正式修復紀錄 |
| `camera-calibration-locations.txt` | 小型文字檔 | 相機與校正檔位置清單 |
| `rosmaster-location.txt` | 小型文字檔 | Rosmaster 相關檔案位置清單 |

Canonical SHA-256：

```text
21ab36d2d944579fe61be43928f9f0431a9cfd4d7c2663488223d08a139fb567  jetson-home.tar
0685f1220f488d4970bb11c1e0e1dff97d230d08d0af672b1bdfd82562e79ffa  jetson-system-config.tar
14eec424149899b1853ee9c2713c30b81f92e6f5597baceda27ee775aefc0aba  x3plus-critical.tar
a646e49998f10b6e7c1749b94862dc0819855c5f84060d58d6d4d9f09b4a87f4  x3plus-original.img
4dec837697b021f8f1906aa304463e13554068533adce484e5985034c3ea26e8  x3plus-working.img
```

Windows 已再次計算 `x3plus-working.img`，結果與上述 `4DEC...26E8` 完全一致。

備份使用注意：

- 不要用 Windows 直接修改 raw image。
- Windows/7-Zip 解壓 tar 可能不完整保留 owner、ACL、xattr；要做完整系統還原時，優先在 Linux 使用 `tar --xattrs --acls --numeric-owner`。
- 若要反覆實驗，先複製 working image，再對副本操作。
- 最少應將 `x3plus-critical.tar`、`jetson-home.tar`、`jetson-system-config.tar`、各自 `.sha256` 與 `SHA256SUMS.txt` 再備份到第二個實體磁碟或可信任的儲存位置。

---

## 5. 預測修復後 Jetson 內會有的內容

### 5.1 `/home/jetson` 根目錄

已目視確認存在：

```text
~/ai_motor_server.py
~/ai_motor_server_A.py
~/ai_motor_server_B.py
~/ai_motor_server_B_wrong_20260704_180857.py
~/arm_cam.launch
~/back_cam.launch
~/cam_only.launch
~/hector_x3plus_bag.launch
~/rplidar_usb0.launch
~/setmotor_start.launch
~/start_all.launch
~/start_robot.sh
~/make_cam_only_launch.sh
~/mecanum_motor_server.py
~/motor_direct_test.py
~/motor_id_test.py
~/rosmaster_direct_speed_test.py
~/maps/
~/my_map.pgm
~/my_map.yaml
~/ROS/X3/yahboomcar_ws/
~/grasp_venv/
~/x3plus_robot_snapshot_20260717_001753/
~/jetson_backup_20260707/
~/yahboom_backup/
~/ydlidar_ws/
~/ydlidar_ros_driver_backup/
```

以上「存在」不代表都應執行。尤其 `start_all.launch`、`start_robot.sh`、`ai_motor_server_B.py`、`setmotor_start.launch` 可能啟動底盤或馬達，首次健康檢查期間不得直接執行。

### 5.2 `deploy_jetson2` 與校正檔

預期主要路徑：

```text
~/Documents/deploy_jetson2
```

已在映像中找到：

```text
~/Documents/deploy_jetson2/arm_cam_intrinsics.json
~/Documents/deploy_jetson2/rear_cam_intrinsics.json
~/Documents/deploy_jetson2/arm_cam_intrinsics_bad_rms1154.json
~/Documents/deploy_jetson2/rear_cam_intrinsics_bad_rms3571.json
~/Documents/deploy_jetson2/arm_cam_intrinsics_candidate_rms0495_cx212.json
~/Documents/deploy_jetson2/arm_cam_intrinsics_bad_rms0452_cy115.json
~/Documents/deploy_jetson2/CALIBRATION_PLAN.md
~/Documents/deploy_jetson2/CAMERA_CALIBRATION_PARAMETERS.md
~/Documents/deploy_jetson2/detection/calibration/rear_camera_calibration_points.csv
~/Documents/deploy_jetson2/detection/calibration/arm_camera_calibration_points.csv
~/Documents/deploy_jetson2/detection/calibration/action_speed_calibration_result.json
~/Documents/deploy_jetson2/detection/calibration/wz_90deg_calibration_result.json
~/Documents/deploy_jetson2/detection/calibration/action_speed_calibration_raw.csv
```

使用相機內參時，優先使用**沒有** `bad` 或 `candidate` 後綴的：

```text
arm_cam_intrinsics.json
rear_cam_intrinsics.json
```

不要因檔名相似而誤用 `*_bad_*` 或候選檔。

### 5.3 Rosmaster 驅動

系統中不是單一普通資料夾，而是有多個 `.egg` 版本：

```text
/usr/local/lib/python3.6/dist-packages/Rosmaster_Lib-*.py3.6.egg
/usr/local/lib/python2.7/dist-packages/Rosmaster_Lib-*.py2.7.egg
```

家目錄也存在原始碼與本地化副本，例如：

```text
~/software/py_install/Rosmaster_Lib/
~/Documents/deploy_jetson/grasp/Rosmaster_Lib/
~/Documents/deploy_jetson2/grasp/Rosmaster_Lib/
~/Documents/x3plus_pipeline_deploy/grasp/Rosmaster_Lib/
```

首次測試必須先確認實際被 Python 載入的是哪一份，不能只假設 system package 或本地副本。

### 5.4 本機開發專案已確認的重點

目前 Windows 專案根目錄：

```text
C:\Users\user\Documents\repo\claude\x3plus
```

已確認主要檔案：

```text
grasp/x3plus_real_grasp.py
grasp/x3plus_deploy_bridge.py
grasp/joint_direction_calibration.py
grasp/servo_test.py
integration/vision_grasp_pipeline.py
integration/nav_rl_grasp_pipeline.py
integration/verify_x3plus_deploy.py
tests/test_safety_guards.py
```

重要程式行為：

- `x3plus_real_grasp.py` 預設為 dry-run；只有加 `--real` 才會初始化 Rosmaster 並驅動伺服機。
- `--real` 找不到 `Rosmaster_Lib` 時會拒絕執行。
- VecNormalize 推論應設為 `training=False`、`norm_reward=False`。
- 觀測空間為 28D、動作空間為 6D。
- 控制映射 S1-S5 為 `hw(API) = 90 + sim_deg`；S6 為 open=30°、closed=180°。
- `arm_hw_invert = (False, False, False, False, False)`。
- 手臂每步最大變化 `max_delta_deg=3.0`。
- emergency stop 的設計是讀取目前角度後保持當前姿態，不是突然回 home。
- `integration/verify_x3plus_deploy.py` 會把 TCP port 7000 已被 motor server 監聽視為 real-mode 阻塞條件。
- `nav_rl_grasp_pipeline.py` 的 real navigation 不允許無 LiDAR，且整合式 real grasp 有額外安全限制。

不要直接用 Windows 專案覆蓋 Jetson 復原內容。應先 diff、備份、確認模型與校正檔，再選擇性同步。

---

## 6. 首次上機前物理安全清單

### 6.1 必做

- [ ] 機器人完全斷電後才插拔系統碟。
- [ ] 系統碟插到底、接頭無鬆動，不使用容易晃動的延長線或 Hub。
- [ ] 使用原廠或已知穩定的 Yahboom/Jetson 供電方式；不要臨時改變供電電壓。
- [ ] 若有可獨立關閉的馬達／底盤電源，首次 OS 檢查時保持馬達電源關閉。
- [ ] 輪子架空或車體放在支架上，不能接觸地面。
- [ ] 機械臂周圍完全清空，避免碰撞人員、桌面、螢幕與線材。
- [ ] 準備可立即切斷整機電源的方法，並由一人專門觀察。
- [ ] 連接顯示器，最好同時準備 USB 鍵盤或可用的有線網路。
- [ ] 拍攝開機畫面或保留完整錯誤照片。
- [ ] 首次開機暫時不要接入會大量耗電的非必要 USB 周邊；若易於拆卸，可先移除額外攝影機/LiDAR，再逐一加回。

### 6.2 禁止事項

- [ ] 不要把 `x3plus-original.img` 當作已修復映像重新燒錄。
- [ ] 不要在 Windows 對 Jetson 碟按格式化／初始化／掃描修復。
- [ ] 不要在開機卡住時連續強制斷電重開。
- [ ] 不要在第一次登入後立刻執行 `start_robot.sh`、`start_all.launch`、`ai_motor_server_B.py` 或任何 `--real`。
- [ ] 不要在 mounted root 上執行 `fsck`／`e2fsck`。

---

## 7. 首次開機流程

### Step 1：開機觀察

1. 系統碟安裝完成、顯示器已連接後再供電。
2. 記錄開機開始時間。
3. 至少等待 5～10 分鐘；若系統自行做檢查，不要斷電。
4. 觀察是否出現：
   - Ubuntu 登入或桌面
   - emergency mode
   - `bash-4.4#`
   - `EXT4-fs error`
   - `JBD2`
   - `Input/output error`
   - `USB disconnect/reset/cannot enable/unable to enumerate`

### Step 2：依結果分流

#### A. 正常進入 Ubuntu

不要啟動 ROS/馬達。登入後先執行第 8 節的唯讀與低風險檢查。

#### B. 再次出現 EXT4/JBD2 或根分割區 mount 失敗

1. 拍下完整畫面。
2. 不要反覆重開。
3. 不要在該機器上對 mounted root 強制 fsck。
4. 優先判定舊系統碟／USB 路徑／供電仍不可靠。
5. 停止使用舊碟，改用新的 128GB 以上可靠儲存裝置寫入 `x3plus-working.img`。

#### C. 找不到開機裝置或黑畫面

檢查：

- 系統碟是否插牢
- 是否接到原先可開機的 USB 位置
- USB 轉接器／線材
- Jetson bootloader 是否仍從 USB 根碟啟動
- 顯示器線材與輸入源

不要因此格式化系統碟。

#### D. 系統進入桌面，但稍後出現 Bus error、唯讀或 I/O error

這仍屬儲存失敗，不是應用程式問題。停止所有測試並正常關機；不要繼續跑 ROS、Python、scp 或 apt。

---

## 8. 正常登入後的第一輪系統健康檢查

以下命令原則上不會驅動機器人。一次執行一組並保留完整輸出。

### 8.1 基本版本

```bash
date
uname -a
cat /etc/nv_tegra_release
cat /etc/os-release
```

預期：L4T R32.6.1、Ubuntu 18.04.6、aarch64。

### 8.2 根檔案系統與容量

```bash
findmnt -no SOURCE,TARGET,FSTYPE,OPTIONS /
lsblk -o NAME,SIZE,FSTYPE,LABEL,MOUNTPOINT,MODEL,SERIAL,TRAN
df -hT /
```

通過條件：

- `/` 是 EXT4
- 掛載選項包含 `rw`
- 根分割區有合理剩餘空間
- 沒有忽然消失或反覆重連的系統碟

### 8.3 最小寫入測試

只有在 `/` 顯示 `rw` 且前一步沒有 I/O error 時才執行：

```bash
touch /tmp/x3plus_write_test && rm /tmp/x3plus_write_test && echo TMP_WRITE_OK
touch ~/x3plus_home_write_test && rm ~/x3plus_home_write_test && echo HOME_WRITE_OK
```

若回報 `Read-only file system`、I/O error 或 Bus error，立刻停止。

### 8.4 當次開機的磁碟／USB 錯誤

```bash
sudo dmesg -T | grep -Ei 'EXT4-fs error|JBD2|Buffer I/O|Input/output error|I/O error|blk_update_request|USB disconnect|cannot enable|unable to enumerate|reset (high|SuperSpeed) USB device'
```

理想結果：沒有輸出。

注意：單純某個相機或 LiDAR USB 裝置失敗，與根系統碟失敗的嚴重度不同；但必須用 `lsusb -t` 和裝置路徑確認是哪個 USB 裝置。只要根碟本身 reset/disconnect，就應停止。

### 8.5 systemd 與本次開機錯誤

```bash
sudo systemctl --failed
sudo journalctl -b -p err --no-pager
```

先保存輸出，不要看到 failed service 就立刻刪除或 disable。要先判斷是否為攝影機、LiDAR、ROS、自動啟動或真正的系統錯誤。

### 8.6 USB 拓樸

```bash
lsusb
lsusb -t
```

記錄系統碟、攝影機、LiDAR、Rosmaster serial 各自在哪條 bus/port。後續若出現 `usb X-Y` 錯誤，才能定位是哪個裝置。

### 8.7 禁止在此階段執行

```bash
sudo e2fsck -f /dev/sda1
sudo fsck /dev/sda1
```

根分割區正在掛載時不可這樣做。

---

## 9. 網路與 SSH 恢復

### 9.1 Jetson 上確認 IP

```bash
hostname -I
ip -br address
```

舊 IP `172.31.28.252` 僅是歷史資訊。以本次輸出為準。

### 9.2 確認 SSH 服務

```bash
systemctl is-active ssh
sudo systemctl status ssh --no-pager
```

### 9.3 Windows 端

```powershell
Test-NetConnection <JETSON_IP> -Port 22
ssh jetson@<JETSON_IP>
```

若發生 SSH host key mismatch：

1. 先確認 IP 沒有被其他設備占用。
2. 確認 Jetson 的網卡/MAC 與目標正確。
3. 因為本次是同一映像還原，SSH host key 理論上應保留；不要未確認就盲目刪除 known_hosts。

### 9.4 SSH 後的安全原則

- 先執行唯讀檢查。
- 不要先做 apt upgrade。
- 不要先 scp 大量資料回根碟。
- 不要先啟動 motor server。
- 若再次出現 `/usr/bin/scp: Input/output error`，立刻視為磁碟故障。

---

## 10. 檢查是否有自動啟動馬達／ROS 程序

在接通馬達電源前執行：

```bash
ps aux | grep -Ei 'ai_motor|motor_server|rosmaster|roslaunch|roscore' | grep -v grep
sudo systemctl --type=service --state=running | grep -Ei 'motor|ros|yahboom|x3plus'
sudo ss -ltnp | grep ':7000'
```

判讀：

- 若 TCP 7000 已有人監聽，先找出 PID 與啟動來源；本機專案的 real preflight 將其視為衝突。
- 若有 motor/ROS 自動程序，先不要粗暴 `kill -9`；先記錄完整 command line、service 名稱與日誌。
- 只有確認程序用途後，才使用正常的 `systemctl stop <exact-service>` 或程式自己的停止方式。

---

## 11. 專案與模型完整性檢查

### 11.1 確認路徑

```bash
ls -la ~/Documents/deploy_jetson2
find ~/Documents/deploy_jetson2 -maxdepth 3 -name 'x3plus_real_grasp.py' -print
find ~/Documents/deploy_jetson2 -maxdepth 4 -name 'Rosmaster_Lib*' -print
```

Jetson 復原資料與 Windows 本機 repo 的目錄層級可能不同。執行命令前應先 `find` 真實位置，不要假設腳本一定在 deploy 根目錄或 `grasp/`。

### 11.2 關鍵檔案

```bash
find ~/Documents/deploy_jetson2 -maxdepth 5 \( -name '*.zip' -o -name '*.pkl' -o -name '*.urdf' \) -print
ls -l ~/Documents/deploy_jetson2/arm_cam_intrinsics.json
ls -l ~/Documents/deploy_jetson2/rear_cam_intrinsics.json
```

預期應有 PPO `.zip`、VecNormalize `.pkl`、URDF/mesh、相機內參與 Rosmaster 本地副本。

### 11.3 JSON 語法檢查

```bash
python3 -m json.tool ~/Documents/deploy_jetson2/arm_cam_intrinsics.json >/dev/null && echo ARM_JSON_OK
python3 -m json.tool ~/Documents/deploy_jetson2/rear_cam_intrinsics.json >/dev/null && echo REAR_JSON_OK
```

### 11.4 虛擬環境

```bash
source ~/grasp_venv/bin/activate
python -V
python -c 'import numpy; print("numpy", numpy.__version__)'
python -c 'import torch; print("torch", torch.__version__)'
python -c 'import stable_baselines3; print("sb3", stable_baselines3.__version__)'
python -c 'import pybullet; print("pybullet import OK")'
```

如果 import 失敗，不要立即重裝全部套件。先記錄錯誤與實際 Python 路徑：

```bash
which python
python -c 'import sys; print(sys.executable); print("\n".join(sys.path))'
```

---

## 12. Rosmaster 與 serial 的「不動作」檢查

### 12.1 裝置節點

```bash
ls -l /dev/myserial /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
readlink -f /dev/myserial
```

### 12.2 udev 規則

```bash
grep -Rni 'myserial\|ttyUSB\|Rosmaster' /etc/udev/rules.d 2>/dev/null
```

### 12.3 Python 實際載入位置

在 `~/grasp_venv` 啟用後：

```bash
python -c 'import Rosmaster_Lib; print(Rosmaster_Lib.__file__)'
```

這只是 import/path 檢查，不應送出馬達命令。若 import 本身會觸發裝置初始化或專案版本有副作用，AI 必須先閱讀該副本的 `__init__.py` 再執行。

若專案提示應先執行 `startup_device_check.py`，先確認檔案真的存在：

```bash
find ~/Documents/deploy_jetson2 -name 'startup_device_check.py' -print
```

本機 repo 只看見對此檔名的引用，沒有確認該檔案一定存在於 Windows repo；不要假設。

---

## 13. 分階段軟體與 dry-run 測試

### 階段 0：只看 CLI

找到真實腳本路徑後：

```bash
source ~/grasp_venv/bin/activate
python <PATH_TO_x3plus_real_grasp.py> --help
```

不得加 `--real`。

### 階段 1：純 dry-run

```bash
python <PATH_TO_x3plus_real_grasp.py> --max-steps 20
```

預期：

- 載入模型、VecNormalize 與 PyBullet FK
- 只輸出角度/Stage/距離等資訊
- 不初始化實體 Rosmaster 或送伺服角度
- 輸出角度應在合理範圍，沒有 NaN/Inf

若預設不帶 `--real` 仍造成任何實體運動，立即斷電並保留程式版本；這代表腳本與目前本機 repo 行為不一致。

### 階段 2：安全工具 dry-run

若檔案存在：

```bash
python grasp/servo_test.py
python grasp/joint_direction_calibration.py
```

兩者預設應為 dry-run。先確認畫面只列出預計命令，不實際移動。

### 階段 3：ROS 基礎，不啟動機器人

```bash
source /opt/ros/melodic/setup.bash
source ~/ROS/X3/yahboomcar_ws/devel/setup.bash
rosversion -d
rospack find yahboomcar_bringup
```

預期 `rosversion -d` 為 `melodic`。

可以單獨啟動 `roscore` 做基礎測試，但不要啟動：

```text
start_all.launch
setmotor_start.launch
start_robot.sh
ai_motor_server_B.py
```

除非已閱讀內容並確認不會立刻驅動底盤／伺服機。

### 階段 4：攝影機

```bash
ls -l /dev/video* 2>/dev/null
v4l2-ctl --list-devices 2>/dev/null
```

先逐台確認影像與 frame 方向，再使用 `--socket`。本機程式有 `--i-confirm-external-frame` 類型的安全確認；外部偵測座標必須是 PPO/URDF `base_link` XYZ，不能把像素、相機 frame 或錯誤軸向直接送入 real grasp。

### 階段 5：LiDAR 與 navigation

- 先確認 LiDAR USB 穩定且 topic 正確。
- real navigation 不允許無 LiDAR；無障礙物輸入會使 safety brake 失明。
- 先做 nav-only 或靜態 topic 檢查，再做整合抓取。

---

## 14. 第一次實際運動的 Go/No-Go 條件

只有以下全部通過才可考慮 real mode：

- [ ] 已正常開機至少 10～15 分鐘。
- [ ] `/` 是 `rw`，`/tmp` 與 `$HOME` 寫入測試成功。
- [ ] 本次 `dmesg` 沒有根碟 EXT4/JBD2/I/O/reset/disconnect。
- [ ] 已完成一次正常 `sudo reboot`，第二次開機仍乾淨。
- [ ] Rosmaster serial 路徑與 udev 規則正確。
- [ ] TCP port 7000 沒有衝突的 motor server。
- [ ] dry-run 成功，角度合理且無 NaN/Inf。
- [ ] 模型與 VecNormalize 配對正確，推論模式為 `training=False`、`norm_reward=False`。
- [ ] 相機 frame、物體 XYZ 與 base_link 定義已人工確認。
- [ ] 輪子架空、機械臂清空、電源可立即切斷。
- [ ] 至少一名人員全程監看，不讓機器人無人運轉。

只要有一項未通過，就是 **No-Go**。

### 建議的第一個實際動作

不要直接做完整 PPO grasp。優先順序：

1. 確認讀角度／裝置連線。
2. `servo_test.py` dry-run。
3. 在輪子架空、手臂清空的前提下，才考慮 `servo_test.py --real` 的小幅 S1 測試。
4. 確認急停／斷電可用。
5. 再做單關節方向驗證。
6. 最後才執行完整 `x3plus_real_grasp.py --real`。

任何 real 指令都必須由當下接手 AI 根據實際檔案內容重新確認，不能只因本文件列出就直接執行。

---

## 15. 可能遇到的問題與處置

| 症狀 | 最可能原因 | 立即處置 |
|---|---|---|
| `EXT4-fs error`、`JBD2` | 根碟再次損壞或掉線 | 停止測試、正常關機；換新碟／線／供電，不在 mounted root fsck |
| `/tmp: Read-only file system` | kernel 將根檔案系統 remount ro | 停止所有寫入與 ROS；保存畫面後關機 |
| `Bus error` | 執行檔/共享庫讀取失敗或記憶體問題；此歷史案例高度指向磁碟 I/O | 先查 dmesg；若伴隨 I/O/EXT4，立即停機 |
| `/usr/bin/scp: Input/output error` | 系統檔讀取失敗 | 不再嘗試大量傳輸，停機 |
| Python `encodings` I/O error | Python stdlib 無法從根碟讀取 | 視為儲存失敗，不要重裝 Python |
| USB system disk disconnect/reset | 磁碟、接頭、轉接器、USB port 或供電不穩 | 立即停止，換新硬體；不要反覆重試 |
| 僅 camera/LiDAR enumerate 失敗 | 周邊 USB、Hub、線材或供電 | 先拔除該周邊隔離，確認根碟仍穩定 |
| SSH 不通 | IP 改變、網路或 ssh service | 用本機螢幕查 `hostname -I`、`systemctl status ssh` |
| SSH 可登入但命令隨機失敗 | 儲存或記憶體不穩 | 立刻查 dmesg；本案例先假設儲存問題 |
| `Rosmaster_Lib` 找不到 | Python 版本／sys.path／本地化副本不同 | 查 `which python`、`sys.path`、`find`，不要盲目 pip install |
| `/dev/myserial` 不存在 | udev 規則、控制板、USB serial 或權限 | 查 `/etc/udev/rules.d`、`lsusb`、`dmesg` |
| port 7000 已使用 | 舊 motor server 自動啟動 | 找 PID/service，正常停止後再測 |
| dry-run 就移動 | 腳本版本與預期不符或 dry-run guard 失效 | 立即斷電，禁止 real；先審查程式 |
| 模型載入失敗 | `.zip`/`.pkl` 配對、Python/SB3 版本或檔案損壞 | 記錄完整 traceback，先驗證檔案與版本 |
| VecNormalize shape error | 觀測不是 28D或模型/統計不配對 | 不可繞過後直接 real；修正配對 |
| 相機有影像但抓取座標錯 | frame、內參、外參、軸向或候選 calibration 用錯 | 禁止 real；回到靜態標定與 dry-run |
| 機器人突然移動 | 自動啟動 service、motor server、舊 ROS launch | 立即斷電；之後檢查 systemd、cron、shell startup |

---

## 16. 建議的穩定性驗證

若第一輪檢查完全正常：

1. 保持 idle 10～15 分鐘。
2. 再查一次 dmesg 的 EXT4/I/O/USB 關鍵字。
3. 執行一次正常重開機：

```bash
sudo reboot
```

4. 第二次開機重新執行第 8 節。
5. 若第二次仍乾淨，才繼續相機、LiDAR、serial 與 dry-run。

不要用高強度磁碟壓力測試這顆舊碟來「證明可靠」。它已在第一次完整寫入中掉線；即使現在能用，也應規劃替換。

---

## 17. 長期修復建議

### 最佳方案

取得新的 128GB 以上可靠系統儲存裝置，將：

```text
x3plus-working.img
```

完整寫入並驗證。建議優先考慮可靠 SSD／高耐久儲存與已知穩定的 USB-SATA/USB 儲存連線，而非繼續依賴舊 Generic USB Disk。

### 為什麼不使用現有 64GB Ubuntu Live USB

當時 Live USB 實際約 57.7GiB，小於來源系統碟約 58.6GiB，無法容納完整 raw image。名義上同為 64GB，不代表可用 sector 數相同。

### 若改做全新安裝

不能把救援用的 Ubuntu 26.04 amd64 Live ISO 安裝到 Jetson。Jetson 是 aarch64，原系統基準是 JetPack 4.6 / L4T R32.6.1 / Ubuntu 18.04.6 / ROS Melodic。全新安裝必須使用相容的 NVIDIA/Yahboom Jetson 映像，再從 tar 與專案備份選擇性恢復。

---

## 18. 本次上機測試的建議回報格式

下一個 AI 應要求使用者每完成一步回報：

```text
[步驟名稱]
時間：
是否正常完成：
完整輸出／照片：
是否有 EXT4/JBD2/I/O/USB error：
是否有任何輪子或機械臂動作：
是否已正常關機：
```

不要只接受「好了」就直接進入 real mode；對磁碟、serial、dry-run 與安全前置條件必須看到具體輸出。

---

## 19. 目前仍未知、必須在現場確認

- [ ] 第二次 Etcher 成功後，系統碟裝回 Jetson 是否能正常開機。
- [ ] 舊碟第一次消失的根因是磁碟本體、電腦 USB port、接頭、轉接器還是供電。
- [ ] Jetson 原 USB port 是否也有相同掉線問題。
- [ ] 機器人目前實際 IP。
- [ ] SSH 是否自動啟動。
- [ ] 是否有 systemd/cron/shell 自動啟動 motor server 或 ROS launch。
- [ ] `/dev/myserial` 實際指向與 Rosmaster 控制板狀態。
- [ ] 相機、LiDAR、馬達控制板是否各自健康。
- [ ] Jetson 上的 `deploy_jetson2` 與 Windows repo 之間有哪些差異。
- [ ] 實際應使用哪一份 Rosmaster_Lib egg／本地副本。
- [ ] 首次上機後是否產生新的 EXT4 error counter 或 USB reset。

---

## 20. 最終決策原則

### 可以繼續的情況

- 正常開機
- 根檔案系統為 `rw`
- 寫入測試成功
- 當次 dmesg 無根碟 EXT4/JBD2/I/O/USB reset
- 正常重開一次仍健康
- 裝置與 dry-run 全部通過

### 必須停止的情況

- 任一新的根碟 I/O/EXT4/JBD2 錯誤
- 系統碟 USB reset/disconnect
- 根檔案系統唯讀
- Bus error、scp/Python 系統檔 I/O error 再現
- dry-run 造成硬體動作
- serial 或 camera frame 尚未確認卻準備執行 real
- 無人監看、輪子未架空或無法立即斷電

**本次成功目標不是立即完成抓取，而是先證明「系統碟、USB、供電與 OS 能穩定工作」，再逐層恢復 SSH、ROS、感測器、dry-run，最後才進入受監督的小幅硬體動作。**
