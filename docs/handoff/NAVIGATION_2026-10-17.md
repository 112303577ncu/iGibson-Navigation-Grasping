# 導航交接：2026-10-17

本文件是今日導航成果與接手順序；最新機器狀態先讀根目錄 [CODEX_HANDOFF.md](../../CODEX_HANDOFF.md)。最終流程為「巡航 → 辨識 → 接近夾取 → 導航至紙盒 → 投放 → 返回巡航」，目前尚未整輪驗收。

## 現在不可直接重跑舊巡航

- 常駐 owner 曾為投放預演重啟，目前記錄 PID **4505**，odom 原點已重設。PID 是紀錄，不是下次必須相同的設定；每次須以 health 與 Serial 實際持有者核對。
- `maps/demo_box_20261017/last_pose_seed.json` 已標 `valid=false`。不得使用文件中的舊 AMCL 初值或舊 map→odom；最近部分右轉後 odom 約 `[-0.0037,-0.0067,-0.2119]`，不能將其視為 map 座標。
- LiDAR 現為車體中心前 **10 cm**；舊圖以 **13 cm** 建立。新幾何僅靜止定位探測通過，尚未完成移動相容性驗證。
- 紙盒 **18×13.5×6 cm** 已移至距原兩面牆各約 **20–25 cm**；18 cm 長邊平行原後牆。新位置尚未完成實體 map 註冊與限制圖重建。
- 右轉 30° 對準僅完成 **12.14°** 後被近距回波閘停止；尚差約 **17.86°**。操作者其後確認剩餘近距回波均為固定機構／線材，這是後續自回波辨識的現場證據；本次文件／推送工作不自動續轉，也未完成新濾波設定驗證。
- 盒高已同步 6 cm，空夾爪投放預演成功並返回巡航姿態，底盤未動；詳見 [RELEASE_PREVIEW_2026-10-17.md](../operations/RELEASE_PREVIEW_2026-10-17.md)。沒有實際物品落盒驗收。預演 FK 前伸約 23.9 cm，與舊停靠假設 41 cm 不同，須核對座標基準及實際可達範圍。

`scene_layout.json` 的以下旗標刻意保留未通過，不能為了執行而直接改成 true：

| 欄位 | 現值／意義 |
|---|---|
| `sensor_geometry.validation.verified` | false；10 cm 幾何尚待動態驗證 |
| `navigation_mask_valid_for_current_bin_placement` | false；移盒後限制圖失效 |
| `bin.registration_verified` | false；實體盒位置未註冊 |
| `bin.pose_map_valid_for_current_placement` | false；保留的舊盒 pose 只供歷史查核 |
| `bin.dock_zone.enabled` | false；局部 30 cm 草案未啟用 |

`state`、`static_route_checks`、`patrol_verification` 是早先結果，不能蓋過上述最新失效旗標。`bin.placement_update.midpoint_pose_map_candidate` 只是假設候選，不是可導航投放點。

## 今日完成與待改善

以下移動成果均發生於後續 owner 重啟、10 cm 幾何修正與移盒之前。

| 項目 | 已完成／證據 | 尚需驗證 |
|---|---|---|
| 地圖與主巡航點 | 約 1.978×2.022 m，2 cm 格；紙盒對角內縮 60 cm 主點 | 新幾何與現場物件擺位相容性；無須預設一定整圖重建 |
| 固定場地逆時鐘巡航 | 兩圈完成；第二圈視覺／monitor 開啟，到點估計誤差 7.01–7.80 cm，最低可用 RAM 1593.90 MB | 新設定重跑；冷開機／搬動後重定位 |
| 沿線修正與任意場內點 | 18 cm 前視、縮短末段脈衝；一圈到點 3.54–4.55 cm；A→B→home 4.14–4.33 cm | 未知起點／全場任意姿態成功率，連續平滑運行 |
| 路徑偏離改善 | 巡航停穩樣本 P95 3.96→2.67 cm、最大 6.86→4.02 cm | 僅一組比較、AMCL/odom 估計，非尺量或連續誤差保證 |
| 後 RGB 相機同時巡航 | 一圈到點 3.66–4.42 cm，最低 RAM 1504.39 MB；199 幀讀取無失敗 | 深度全程負載、辨識覆蓋與機構遮擋；未證明無死角 |
| 紙盒前導航 | 可到準備區；短靠近被限制圖／停止投影阻擋而正確退出 | 現盒註冊、對準、可達投放點、負載投放及退出 |
| 短暫失效處理 | 資料過期停等、最多 3 秒等待／每次最多 12 次恢復 | 上述成功圈恢復次數均 0，還需受控故障測試 |
| 障礙處理 | 靜態 mask／有界 A*、前方停等與即時閘 | 動態繞行未接通；空場成功不等於避障成功 |
| ArUco | ID 0 盒旁、ID 1 對角的初步方案 | 列印尺寸、相機內外參、標記 map 座標及融合均待實體驗證 |
| 任務整合 | 獨立巡航、手臂空投放預演各自有實測 | 尚未接通並重複驗收整個決賽循環 |

現在巡航入口是 `deploy/ros/g2_map_patrol.py`：AMCL+odom、靜態路徑及有界脈衝，**沒有執行 iGibson 導航 PPO**。一般中心牆距 40 cm、選點內縮 60 cm、前牆煞車 50 cm；紙盒緩衝約 28.32 cm。局部 30 cm 牆距僅停靠草案，不能套用全場。紙盒高度低於約 9.5 cm 雷射面，不能依賴 LiDAR 穩定看見紙盒。

## 接手執行順序

1. 唯讀確認唯一 Serial owner、四輪零、控制板回傳、手臂 travel、RAM，以及新鮮 `/scan`、odom、TF。保留 resident；導航走 TCP 7000，不另起 Serial 驅動。
2. 重新辨認機構／線材回波，將操作者確認與角度／距離／多幀資料對照；若需擴充遮罩，只遮有證據的固定區，保留其餘環境回波與資料過期閘。
3. 建立本次有效 map 初值，在 10 cm TF 下核對牆線、掃描重合及定位；舊種子不可用。確認幾何一致後再留證更新驗證旗標，必要才重建地圖。
4. 對新紙盒位置完成實體 map 註冊，重建一般限制圖／停靠候選並核對手臂實際可達範圍；再測短靠近、空載投放及退出。
5. 新設定下重跑一圈與 home／任意合法點／投放準備點，記錄到點、偏線、RAM、停等原因與最後零輪上鎖。
6. 佈置 ArUco 並驗證重定位；在中心、四邊、四角放測試物驗證相機覆蓋。需要擺物／量測時才通知操作者。
7. 接通辨識／接近／夾取／運送／投放／回巡航，再測短暫資料中斷、恢復及多輪重複；結果不明的夾取／投放不可盲目重播。

## 可重用指令（Jetson）

唯讀檢查，不解鎖或移動底盤：

```bash
cd /home/jetson/Documents/deploy_jetson2
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
python3 grasp/v23/graspctl.py health /tmp/grasp_health.sock
python3 grasp/v23/graspctl.py status
fuser /dev/myserial
free -m
rostopic echo -n 1 /scan/header
rostopic echo -n 1 /odom_setmotor/header
```

定位啟動須先取得**新的可信 map 初值**；以下變數故意不給舊數值。先確認沒有另一個 GMapping／AMCL 發布 map→odom，使用原始 `demo_box.yaml`，不能用膨脹後的中心限制圖定位。定位程序也應有 runtime 與 RAM 外層限制。

```bash
: "${MAP_X:?填入本次有效map x}"
: "${MAP_Y:?填入本次有效map y}"
: "${MAP_YAW:?填入本次有效map yaw弧度}"
roslaunch deploy/ros/g2_scene_localization.launch \
  initial_x:="$MAP_X" initial_y:="$MAP_Y" initial_yaw:="$MAP_YAW"
```

另一個已 source ROS 的終端，可執行無移動 probe：

```bash
/usr/bin/python deploy/ros/g2_map_patrol.py \
  --scene maps/demo_box_20261017 --output /tmp/patrol_probe.json
```

**probe 的 `ok=true` 只代表當次讀取檢查，不會驗證移盒遮罩或 10 cm 幾何；只有 `--real` 才呼叫 `require_motion_ready()`。目前不應執行下列實車範本。** 必須完成前述幾何、盒註冊／限制圖及重定位，再按證據更新相應旗標，才可執行：

```bash
/usr/bin/python deploy/ros/g2_map_patrol.py \
  --scene maps/demo_box_20261017 --real --loops 1 --seconds 300 \
  --output /tmp/patrol_result.json
```

可用 `--goals /tmp/goals.json` 指定合法場內點，整份清單會先檢查；`dropoff` 別名仍拒絕未驗證停靠。目標 JSON 格式：`["home", {"name":"A", "point":[0.65,0.45]}, "home"]`，其中 A 是歷史測試點，仍須經本次限制圖確認。

入口起動要求約 1 GB 可用 RAM，執行中低於 768 MB 停止導航子程序；控制板／感測過期、owner 改變、手臂非 travel 等也停車。這些是受限入口的監督，直接 TCP 7000 的其他 client 並不自動受同一張 mask 約束。

## 證據索引

- [PATROL](../operations/PATROL_2026-10-17.md)：兩圈基準與控制限制。
- [NAV_PATH_REVIEW](../operations/NAV_PATH_REVIEW_2026-10-17.md)：偏線比較與自訂點。
- [DOCK_CAMERA](../operations/DOCK_CAMERA_2026-10-17.md)：停靠失敗、RGB／深度／YOLO限制及隊友校正疑點。
- [GEOMETRY_DOCK_POLICY](../operations/GEOMETRY_DOCK_POLICY_2026-10-17.md)：10 cm 幾何與局部規則草案。
- [BOX_REPOSITION](../operations/BOX_REPOSITION_2026-10-17.md)：移盒來源、靜止定位探測；文末「未轉向／未預演」已由本文件及投放紀錄更新。
- `docs/calibration/{patrol,nav_review,dock_camera,geometry_dock,box_reposition,release_preview}_2026-10-17/`：原始 JSON、log、影像及來源 hash。

相關導航測試持續通過；詳細測試數與各次版本見對應證據，不能把歷史成功當成新配置驗收。既有 controller／servo／floor／UI／FSM 回歸通過；Windows 中文 URDF 路徑僅使用測試程序轉接，未修生產路徑處理。
