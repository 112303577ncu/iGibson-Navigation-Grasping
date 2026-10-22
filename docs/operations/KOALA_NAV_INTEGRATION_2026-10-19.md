# 10/19 koala 更新與巡航／投放整合

2026-10-22發布註：下列本機程式、模型、測試與原始驗證資料已納入本次完整提交。
正式新流程使用 [固定runtime入口與恢復規則](MISSION_RECOVERY_2026-10-22.md)；
下文為10/19版本的整合紀錄，不以歷史固定20cm退出或舊commit覆蓋新版規格。
本次提交／推送不代表部署或新增實車驗收。

## 本次核對來源

以 `git ls-remote` 和 `git fetch` 核對 GitHub，koala 的
[`codex/approach-lidar-frame`](https://github.com/koala915/iGibson-Navigation-Grasping/tree/a2c6f19edf3903ce954480776bfa5c43f209ec8c)
已從 `eafba63` 更新至 `a2c6f19`（10/19 00:02，台灣時間），新增六個提交。
其 `v23-grasp-test` 仍為 `3eb79ec`；最新接近／投放成果在前述工作分支。

| 項目 | GitHub 最新進度 | 與導航的接點 |
|---|---|---|
| 紙盒辨識 | 新增 `detection/models/placement_box.pt`，SHA256 `8e256e80b770a6ff7a5a3187ab7cbeb7e30fb84a5d6a1fc1515f84a3025406a0` | 視覺接近以 placement 類別鎖定紙盒，與 sugarbox 夾取模型分開 |
| 投放 | 新增 `--target placement`；近緣目標 x=0.213m、y=-0.028m；到達檢查後才 release／stow | 地圖先送到可見交接點，再交給同一套後相機接近程式 |
| 實車紀錄 | 組員紀錄完整第3、4次均將糖果盒投入紙盒；兩次均未使用AMCL | 證明局部流程有成功案例，尚非整合巡航的連續驗收 |
| 偏角／遠處目標 | 偏12°先轉正至5°內；框外估計不允許直接宣告到達 | 地圖交接先面向盒，視覺必須重新取得實際目標 |
| E1 交接 | 無物連4個新幀後最多前挪3cm兩次；兩筆新讀數在4秒內／8mm內配對 | 不因一次空檔清掉有效讀數，仍不使用過期讀數假裝新幀 |
| 夾爪停滯 | 單獨張／合空爪需連3次無進展才判停滯；等待時不繼續推進指令 | 精選合入此修正，保留本分支 Serial transport、單調時間及盒高6cm |
| 任務串接 | 新增 Windows `mission_loop.py`，單一速度控制者、夾取→投放；`--no-patrol`可獨立測 | 匯入後加入地圖運送與退出再定位檢查，接進v23入口 |

實測來源：[koala 現場紀錄](https://github.com/koala915/iGibson-Navigation-Grasping/blob/a2c6f19edf3903ce954480776bfa5c43f209ec8c/docs/operations/placement_20261018/README.md)。
本地亦收進該 README、影像與原始log；來源hash見 `docs/calibration/koala_sync_2026-10-19/upstream_import.json`。

## 已完成的離線整合

`grasp/v23/demo.py mission` 轉到 Windows 任務程序；`object-approach` 轉到組員接近程序。
新版程式／模型已加入目前工作樹，**尚未commit／push或部署Jetson**；沒有合併組員整個分支。

完整巡航模式：

1. AMCL／ArUco提供可信位置，接入最近可達巡航路段；桌機同時辨識糖果盒。
2. 辨識穩定後終止巡航、停車並釋放控制權，組員接近／E1夾取並收臂。
3. 確認 holding=true，再由新 `v23_koala_map_handoff.py` 檢查定位、1024MB RAM及5個穩定新幀。
4. 原有G2地圖控制器導航到**經量測的相機交接點**，面向紙盒；導航期間原768MB閘保留。
5. 地圖控制器退出／重新鎖定後，桌機才解鎖並啟動 `--target placement`；視覺重新鎖目標、靠近、檢查到達、release／stow。
6. 檢查後退空間，退出20cm後，確認空爪、定位與中心通行遮罩；尚在禁航區就回報 `WAIT_EXIT_CORRIDOR`，不直接啟動下一輪巡航。

地圖與接近不並行占用TCP7000，不新增Serial owner。不自動重做結果不明的夾取／投放。
這是程式串接及離線測試，**還不是整輪實車通過**。

## 交接點為何仍未啟用

舊準備點 `[1.02558,0.66380]` 到紙盒候選中心僅約56cm。
組員紀錄：靠近到約80cm內才找盒時可能無法取得量測。因此直接沿用舊點可能讓視覺一開始就看不到盒。

新範本 `integration/placement_navigation_handoff.example.json` 提供約95cm距離的**候選**點
`[0.663,0.521]`，不改巡航路線與禁航遮罩；verified／盒位登錄／相機交接驗證均預設false。
必須量盒中心，並以持物巡航姿態確認相機能鎖定，填入calibration_id後才設true。
同一份設定須部署到PC及Jetson；橋接會核對整份設定SHA256與AMCL原始地圖hash。
完整模式的紙盒位置以此交接設定為準，不再拿舊 `placement_box_map.json` 的未驗證座標作盲走目標。

不需AMCL的組員獨立流程仍保留：

```powershell
# Windows；不帶--real只檢查，盒需在夾取後可由後相機找到。
python grasp/v23/demo.py mission --host 172.20.10.3 --no-patrol
python grasp/v23/demo.py object-approach --host 172.20.10.3 --target placement
```

完整巡航模式在完成上述量測後才可使用：

```powershell
python grasp/v23/demo.py mission --host 172.20.10.3 --scene maps/demo_box_20261018_patrol --map-handoff-config integration/placement_navigation_handoff.example.json
```

目前不應直接開完整real循環；範本未啟用時會先明確指出需要的校正，不會先開始夾取。

## 尚需處理的問題與優先順序

1. **開機部署與120秒等待／續行**：AMCL監督新版、task_resume_guard與本輪橋接須配套部署；檢查唯一Serial、odom／scan／TF及RAM。
2. **ArUco量測定位**：8張可解碼，但相機內外參、map錨點及metric producer仍未完成；任意搬車仍不能靠對稱四牆自動保證角落正確。
3. **盒位／相機交接點**：量盒中心，實測持物可見距離；確認候選交接點而非直接套用95cm。
4. **odom與遮擋**：組員記錄新地墊上odom偏快，末段約41cm盲走曾停短3–6cm；要先尺量長段與近盒停止重複性，不能只放寬到達門檻。
5. **控制器版本**：本輪只精選匯入夾爪停滯修正；本分支未含組員既有 `tcp_height_error_m` 深度補償。部署不能整份覆寫現車控制器，需核對現車 `GRASP_TCP_HEIGHT_ERROR_MM`／啟動參數並保留組員深度校正。
6. **USB／相機**：組員10/18兩次手臂動作後相機掉線；軟體可重啟bridge，但USB裝置消失仍需固定線材／供電排查。
7. **全域遮罩適用範圍**：40cm限制只由G2地圖導航執行，koala局部接近用自己的LiDAR／距離閘，未共用全域mask；不宣稱整車所有命令都受同一禁航區限制。局部投放走廊與20cm退出距離仍需實測。
8. **辨識與資源**：YOLO／SAM／PPO在Windows；Jetson維持常駐控制及定位。上游兩次成功仍不能取代三個起點、至少三次完整巡航→夾取→投放→退出→續巡航驗收。

## 圖與測試

最新路徑與禁航圖：`output/png/patrol_keepout_latest_2026-10-19.png`；來源是目前scene的
`outer_patrol_goals.json`及`navigation_center_mask.pgm`，不是另畫示意路線。
所有路段已檢查位於遮罩允許區，圖的紙盒為候選位置，不表示已登錄或機器人即時位置。

組員測試含AST原始碼檢查，需Python3.9以上的 `ast.unparse`；此機初次用Python3.8跑時出現17個測試框架相容性失敗，
改用已安裝Python3.12後194項全通過。執行程式本身仍以Python3.8語法核對；不可把測試環境失敗算成硬體故障。
定位／橋接與必跑回歸結果留存 `docs/calibration/koala_sync_2026-10-19/`。
