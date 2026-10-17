# 接續入口（2026-10-17）

決賽目標：定位 → 巡航辨識 → 接近夾取 → 收臂運送 → 紙盒投放 → 返回巡航。
分支：`112303577ncu/iGibson-Navigation-Grasping` 的 `v23-grasp-test`。

## 先讀這份

[導航成果、限制與接手命令](docs/handoff/NAVIGATION_2026-10-17.md)。只按下一步讀相關檔案，無須重掃專案。

## 最近實機狀態

- 最近 SSH 位址 `172.20.10.3`、帳號 `jetson`；部署目錄 `/home/jetson/Documents/deploy_jetson2`。
- resident 最近 PID4505，唯一 Serial owner；輪零、maintenance=true、手臂 travel、空夾爪。這是最後量測紀錄，接手須唯讀重查。
- owner 為投放預演重啟，odom 原點重設；`maps/demo_box_20261017/last_pose_seed.json` 已 `valid=false`，禁止重用舊 map→odom。
- LiDAR 前移量已由13改10 cm；舊圖僅做新幾何靜止定位探測，尚待移動驗證。
- 紙盒18×13.5×6 cm，距原兩面牆各20–25 cm；新 map 註冊與遮罩未完成。幾何／盒位旗標維持 false，局部30 cm停靠區未啟用。
- 右轉30°只完成12.14°；最後 odom 約[-0.0037,-0.0067,-0.2119]，不是 map 座標。操作者確認近回波均為固定機構／線材。未自動續轉，濾波尚未完成外物對照；不得再次從零轉30°。
- 盒高6 cm空夾爪投放預演成功，返回travel；無實物投放驗收。見[預演與部分轉向](docs/operations/RELEASE_PREVIEW_2026-10-17.md)。
- 暫時 AMCL／快照已停；vision／monitor最近恢復active。機器上的 service/controller 含相容性差異，部署前比對，不整份覆蓋組員修正。

## 下一次執行順序

1. 唯讀確認服務、唯一Serial、輪零、手臂姿態、板回傳、RAM、scan／odom／TF；導航仍走resident TCP7000。
2. 根據現場確認比對固定機構回波，僅建立有證據的局部濾波，完成外物漏檢對照。
3. 用新的可信map初值重開定位，驗證10 cm幾何；必要才重建地圖，不直接把旗標改true。
4. 註冊新紙盒位置、重建限制圖／可達停靠點，驗證靠近、投放與退出。
5. 重跑一圈、home及合法任意點，再接視覺夾取／負載運送／投放／續巡航。
6. ArUco列印後量黑框、固定支架、量map位置並驗相機參數；再驗相機覆蓋與故障恢復。

一般中心牆距40 cm、巡航選點內縮60 cm、直行前牆煞車50 cm；局部30 cm僅草案。
新步驟約需1 GB可用RAM，導航低於768 MB退出子程序。例行安全確認已由使用者授權，不重問；量測、擺物或實際故障資訊才詢問。

## 已有證據與界線

此前配置下逆時鐘巡航與自訂A→B→home通過；修正後到點估計約3.5–4.6 cm，停穩樣本最大偏線約4.0 cm，非尺量／連續保證。
巡航未使用iGibson PPO，尚未接通完整v23任務；冷開機重定位、動態繞障、全場辨識與實物落盒仍待驗證。
最新測試及列印檢查見導航交接與 `docs/calibration/`；原始失敗報告不改成成功。

ArUco列印檔與檢查見[列印審查](docs/calibration/aruco_2026-10-17/PRINT_REVIEW_2026-10-17.md)。
先前逐段紀錄保留於[封存歷程](docs/handoff/CODEX_HISTORY_2026-10-17.md)，不作最新狀態。
