# iGibson Navigation + Grasping 交接驗證說明

本套件包含目前整合好的 iGibson 任務層、LiDAR/AMCL 安全閘門、v21 grasp
介面與 Route B 啟動/診斷文件。它是「程式與離線驗證交接包」，不是實機已通過
所有 P0 硬體關卡的證明。

## 1. 解壓後先做離線驗證

在套件根目錄執行（Python 3.10+，Windows/Linux 均可）：

```bash
python -m py_compile integration/nav_rl.py integration/ros_io.py \
  integration/mission_pipeline.py integration/preflight.py \
  integration/nav_rl_grasp_pipeline.py

python integration/nav_rl.py --selftest
python integration/ros_io.py --selftest
python integration/mission_fsm.py --selftest
python integration/mission_pipeline.py --selftest
python integration/nav_rl_grasp_pipeline.py --selftest
python tests/test_vision_grasp_bridge_pose.py
python tests/test_safety_guards.py
python tests/test_mission_end_to_end.py
python integration/preflight.py --offline
```

目前基準結果：`preflight = 20 pass / 0 warn / 0 fail`；安全測試 63 passed；
mission end-to-end 23 passed；vision bridge 13 passed（合計 124）。模型載入若因環境缺少 PyBullet、Stable-Baselines3
或 Ultralytics 而失敗，先安裝相同依賴，再把失敗輸出交回，不要直接宣告硬體通過。

## 2. Route B 靜態檢查

在 Linux、Jetson 或 Git Bash：

```bash
bash -n Navigation-RouteB-Handoff/scripts/startup/lib_route_b.sh
bash -n Navigation-RouteB-Handoff/scripts/startup/route_b_startup.sh
bash -n Navigation-RouteB-Handoff/scripts/startup/route_b_quick_start.sh
python2 -m py_compile \
  Navigation-RouteB-Handoff/scripts/diagnostics/scan_policy_orientation_gate.py \
  Navigation-RouteB-Handoff/scripts/diagnostics/verify_scan_policy_orientation.py
```

`scan_policy_orientation_gate.py` 是唯讀診斷，不會驅動底盤；它比較 `/scan`
與 `/route_b/policy_lidar_48`，輸出 raw 的 0°、+90°、−90°、180° sector，
policy 中央五束、frame、角度上下限與 ray 數量。

## 3. 實機上線順序（必須在 Jetson 執行）

先把 `Navigation-RouteB-Handoff` 放到 Route B 工作區，載入 ROS 與 Python 3
環境，然後：

```bash
cd ~/route_b_handoff_current
./scripts/startup/route_b_startup.sh orientation-test
```

確認底盤保持停止後，分別採集四個位置：

```bash
python2 scripts/diagnostics/scan_policy_orientation_gate.py \
  --placement front --output /tmp/orientation-front.json
python2 scripts/diagnostics/scan_policy_orientation_gate.py \
  --placement back --output /tmp/orientation-back.json
python2 scripts/diagnostics/scan_policy_orientation_gate.py \
  --placement left --output /tmp/orientation-left.json
python2 scripts/diagnostics/scan_policy_orientation_gate.py \
  --placement right --output /tmp/orientation-right.json
```

人工確認四份 JSON 後，用實際 TF yaw 與 policy offset 建立 marker（不要手寫）：

```bash
python2 scripts/diagnostics/verify_scan_policy_orientation.py \
  --evidence /tmp/orientation-front.json \
  --evidence /tmp/orientation-back.json \
  --evidence /tmp/orientation-left.json \
  --evidence /tmp/orientation-right.json \
  --operator-pass \
  --tf-yaw-deg <0 或 180> \
  --offset-deg <0 或 180> \
  --scan-launch <實際 launch 檔> \
  --operator <姓名> \
  --output-marker ~/.route_b_runtime/scan_orientation_verified
```

marker 通過後才可繼續：

```bash
./scripts/startup/route_b_startup.sh localization
# RViz 設定 Fixed Frame=map，執行 2D Pose Estimate
./scripts/startup/route_b_startup.sh await-localization
./scripts/startup/route_b_startup.sh dryrun --goal 13
```

`dryrun` 沒有 marker 會拒絕啟動。`--real` 也必須傳入同一份 marker；舊的
`--i-confirm-lidar-orientation` 旗標不再是方向證明。

marker 只能由 `verify_scan_policy_orientation.py` 產生。它會比對四份證據：
front 的 policy 中央束必須比後／左／右近至少 `--min-separation-m`（預設 0.25 m），
並由「板在前方時最近的 raw sector」量出 offset（`zero_deg`→0°、`raw180_deg`→180°）；
`--offset-deg` 與量測值不符會拒寫。手寫 marker 會產生一份沒有量測支撐的 PASS。

### AMCL 在靜止時的保鮮

AMCL 只在有動的時候更新並發佈 `/amcl_pose`，而夾取一次會讓底盤靜止兩分鐘以上。
任務層自己呼叫 `/request_nomotion_update`（靜止時，以及手臂佔住主迴圈之後補一次，
`--amcl-refresh-timeout` 預設 3 秒），**不需要**另外跑 Route B 的
`amcl_nomotion_keepalive.py`。前提是 rosbridge 有帶 rosapi 且 AMCL 真的提供該
service —— self-check 與 `preflight --onboard` 都會確認，`--real` 下確認不到就拒絕啟動。

## 4. 尚未能由程式單獨證明的項目

1. **LiDAR raw index 的實際物理方向**：AMCL 使用 TF 不代表 policy raw index
   的車頭方向正確；必須完成上面的四方向板子測試。
2. **AMCL 丟失定位的自動恢復**：目前安全行為是 `PAUSED`。操作員要在 RViz
   重新設定 2D Pose Estimate，確認 `map -> odom` 後再跑 localization gate。
3. **GLB 地圖外的動態障礙**：只能由 48-ray safety brake 提供保護，仍需現場
   以箱子、行人和狹窄通道測試。
4. **序列埠唯一擁有者**：實機執行 `fuser -v /dev/myserial` 與
   `fuser -v /dev/ydlidar`，每條 UART 應只有預期程序。
5. **手臂相機 extrinsics / grasp-home homography**：必須用實機量測結果取代
   predicted 值；未完成前 integrated real grasp 應保持拒絕。

## 5. 交接回報格式

請回傳：

- `HANDOFF_MANIFEST_SHA256.txt` 的 SHA-256 是否一致；
- 離線驗證每一個命令的 exit code 與最後 20 行輸出；
- 四份 orientation JSON 與 verifier 產生的 marker（不要只回傳 commit message）；
- 實機測試時的 ROS launch、TF yaw、policy offset、AMCL/odom/LiDAR 狀態。

