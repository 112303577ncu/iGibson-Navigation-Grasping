# 112303577ncu → koala：ArUco 角落初始化工具（2026-10-19）

## 接手入口與目前狀態

本次發布限 ncu 的角落工具、定位／等待配套、底盤初次零命令 telemetry、導航影像許可接口及測試證據。
**不要整枝合併來覆寫 koala 的接近、夾取、投放或手臂 controller。** 精選下列檔案；本次沒有更新其 mission_loop／sugarbox 接近程式與模型。
機器人已暫停：172.20.10.3，owner4443，四輪零、maintenance=true、travel、空爪，沒有巡航／取像程序。
這是停止時的紀錄；接手後須讀即時狀態，不沿用 PID／起點檔。

## 小工具已可怎麼用

`integration/g2_aruco_corner_seed.py` 把「新鮮 LiDAR 候選＋三張影像的 ID／左右位置」接成一次 AMCL 起點提交。
它只做選候選／寫 seed／等確認，不開 Serial、不發輪速、不動手臂。

1. koala 停車、收臂 travel、釋放 TCP7000 控制權。
2. Jetson 取得同步 capture；PC 以同一份 capture 算 LiDAR 候選。
3. PC 提供三個不同來源影格的 ID／影像左右位置；工具只在唯一候選通過時提交。
4. **只接受 `ok=true AND localized=true AND status=LOCALIZED`**；其餘停車、取得另一個觀察角度或處理具體故障。
5. 隨後由 koala 的總指揮啟動巡航，並在辨識候選時先停止巡航，再確認與接近。

輸入範本：`deploy/ros/aruco_corner_observations.example.json`。
必須保留 capture_id／boot_id／owner_pid／map_hash／registration_id／camera_id；影格用 Jetson 來源時基，不用網路接收時間。
只傳一個 ID、舊截圖或只看目前 AMCL ready，不能當作新初始化已完成。

### Jetson 相機未被串流占用時

```bash
cd /home/jetson/Documents/deploy_jetson2
/home/jetson/grasp_venv/bin/python3 deploy/ros/g2_corner_camera_capture.py   --output-dir /tmp/corner_batch --registration-id coarse_wall_layout_20261019_v2
```

下載該資料夾 capture.json／observations.json。**capture 原始位元組不能重新排版**。
若 koala 已持有後相機串流，沿用其來源影格，另呼叫 native ROS `g2_corner_snapshot.py`；不要開第二個相機 reader。

### PC 比對

```bash
python integration/g2_scan_match.py --snapshot corner_batch/capture.json   --scene maps/demo_box_20261018_patrol --reference-map-yaml active_map.yaml   --output corner_candidates.json
```

`active_map.yaml` 是從 Jetson 下載的**確切原始檔**；本機配合同一張 PGM。
現車 YAML 的 image 是 Linux 絕對路徑，hash2274a1c2...；repo 相對路徑版 hash063cd6bc...，不可混用，也不要改現車地圖來湊 hash。
Windows 匹配子程序給60秒上限；車在分析期間保持停止，capture 有效120秒，不能重打舊資料時間。

### Jetson 最後提交

將 candidates／observations／正式 registry 傳到 Jetson，再執行：

```bash
/home/jetson/grasp_venv/bin/python3 integration/g2_aruco_corner_seed.py   --snapshot /dev/shm/g2_corner_capture.json --candidates /tmp/corner_candidates.json   --observations /tmp/corner_observations.json   --landmarks maps/demo_box_20261018_patrol/aruco_corner_landmarks.json   --output /tmp/corner_result.json --real
```

本次 GitHub 的正式 registry 為 v2；現車正式路徑尚是 v1，v2 的成功實測用 `/tmp/g2_boot_20261019/registry_v2.json`。
接手時精選部署 v2，再讓 producer 帶相同 registration_id；不用重啟控制器。

## 必要配套

- `deploy/ros/g2_corner_snapshot.py`：native ROS Python2，無 AMCL 也可取新 scan／odom。
- `deploy/ros/g2_corner_camera_capture.py`：Py3.8＋OpenCV ArUco，V4L2 來源時間三幀，不跑 YOLO。
- `integration/g2_scan_match.py`、`integration/g2_aruco_corner_seed.py`、`integration/v23_task_resume_guard.py`。
- 常駐 `g2_localization_service.py`、`g2_localization_policy.py`、systemd unit：同 boot 有界恢復與 accepted_reference_stamp。
- `grasp/v23/chassis_server.py` 的 first-zero telemetry 只有三個小改動；現車已精選套用。若 koala 底盤檔不同，只取這段修正，勿整份覆寫。

詳細協議見 `docs/operations/ARUCO_CORNER_SEED_2026-10-19.md`，任務等待／恢復見 `TASK_WAIT_RESUME_2026-10-18.md`。

## 已實測與限制

- ID0／2／7 初始化成功；搬車後另一位置 ID2／7 成功；再搬車後以 C牆 ID1左／5右成功（attempt9_c_wall，五個新健康幀確認）。
- 正式 v2 只包含 **0、1、2、5、7**；3、4、6尚未納入。座標仍是安裝配置估計，保留20cm誤差界，不是精密相機／標記定位。
- 單獨 ID1 曾無法排除對稱解；小角度觀察後讀到1／5才成功。不是所有未知點都已驗收。
- AMCL child中斷6.03秒恢復；靜止124.54秒／246樣本全ready、odom+2473、RAM最低1569MB。
- **使用者回報巡航撞到糖果盒；路線結束不等於避障驗收通過。** 本輪沒有夾取／投放成功，未完成整輪 demo。
- 模型把牆面黑色固定夾、手臂相機誤認為糖果盒；量距 hull 外的候選先前也會漏掉。不能只等三張確認而持續前進，不能將近回波全部忽略。

## koala 後續優先順序

1. 精選工具與上述配套，以新 capture 驗證初始化結果，再接既有總指揮。
2. 補足剩下標記／不同起點；相機看不到有效標記時分段觀察，不挑最低 LiDAR 分數猜角落。
3. 補上候選即停、停穩後取得三個新影格確認、影像心跳逾時停車，以及低矮物／盲區保護；處理牆夾誤辨識後才重跑找物。
4. 夾持成功後才運送；盒位及視覺交接點仍需登錄，不能只憑地圖放開物體。

`g2_map_patrol.py` 本次另提供可選 `--vision-permit PATH --forward-pulse-max .08`。
許可 JSON 為 `{owner_pid, boot_id, source_stamp, allow}`，來源影像必須在1.2秒內，allow必須為true；不符會停車／等待或退出。
**這只是一個已測試的導航接口，預設不啟用；正式持續發布者仍要由 koala 串接。** ncu 的臨時PC取像／辨識腳本在C:/tmp，不是本次可直接啟動的完整任務程序。
新增負協方差樣本處理：停車等新有效樣本，不把負數當有效，也不因單筆壞值立即放棄全程。

## 證據

`docs/calibration/aruco_live_2026-10-19/`：attempt3／4／9初始化、AMCL恢復／idle、collision_assessment、attempt10～12停車候選及誤辨識影像。
舊證據內的 no motion／未push／三張registry均屬當時狀態；本頁為本次接手的最新入口。
