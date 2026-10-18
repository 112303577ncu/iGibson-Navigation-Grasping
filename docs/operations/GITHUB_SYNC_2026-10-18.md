# 10/18 GitHub整合發布與組員流程核對

發布分支為 `112303577ncu/iGibson-Navigation-Grasping:v23-grasp-test`；發布前與origin共同基準為 `71bafd5`。只更新本人的fork分支，不合併或推送owner／組員分支。舊PDF位置的刪除與新位置保留一併記錄，沒有還原已搬移的檔案。

## 重新抓取的組員來源

10/18以git fetch與ls-remote核對[koala儲存庫](https://github.com/koala915/iGibson-Navigation-Grasping)：

| 分支 | 已確認版本 | 用途 |
|---|---|---|
| `codex/approach-lidar-frame` | `eafba63f3ba32673738f2daa29aa8ec80686e7ab` | 最新朝物件移動、E1微調／夾取交接與10/16實測紀錄；提交日期為10/17 |
| `v23-grasp-test` | `3eb79ece15b239ef0c497c0d0558a0a6e38dbd28` | 已合併穩定的校正／資產路徑支援 |
| `agent/confirm-real-bin-delivery` | `cf18fd71454cbdaf675cf17a92be610ee26dd9a3` | 8/7舊完整任務／投放成功紀錄；不是10月常駐v23流程的驗收 |

最新版朝物件分支本次沒有再增加新提交。已匯入檔案的來源與SHA256見
`docs/calibration/resident_nav_2026-10-18/upstream_import.json`；門口導航模型pair已存在本分支，與組員來源hash一致，不重複新增權重。

## 已收進本次發布的程式與流程

- `sugarbox_rl_approach_final2.py`：Windows端YOLO／SAM2／PPO接近候選，透過resident TCP輸出；重型推論不放進Jetson定位服務。
- `sugarbox_approach_grasp.py`：接近 → 停穩 → E1觀察 → 有界距離微調／重觀察 → grasp → stow。只允許特定resident手臂指令，不含投放release。其DRIVE候選與相機座標仍需目前硬體的校正驗證，不能因抓到程式就視為已通過實機。
- `target_approach.py`、`trash_target.py`及校正／幾何／訓練契約支援：保留組員的對準、bbox與延遲資訊介面；測試fixture依16個inlier校正資料調整，沒有假改實車校正值。
- `v23_delivery_navigation.py`：回最近巡航點／從最近路段巡航／投放準備區導航，各步驟獨立結果與明確等待原因。
- `v23_dock_approach.py`：只接受明確dropoff目標與已驗證標定；有限脈衝、距離閉迴路，不直接盲走固定15 cm。不發release，不重試結果不明的夾取／投放。
- 常駐AMCL監督器、舊獨立Serial入口阻擋、巡航等待及重算路徑、實測scene／證據、8張ArUco與配置PDF。

## 後續完整整合的接點

1. ArUco／AMCL確認目前map位置；沒有可靠定位時等待或限角搜尋，不從四個相似解任選最高分。
2. 接入最近可達巡航路段，逆時鐘巡航。辨識夾取物後，把底盤控制交給組員接近程序；只有一個速度輸出者。
3. 停穩後交E1／夾取，確認持物並收臂，再將底盤控制交回地圖導航。
4. 導航到投放準備區，再交給以紙盒或ArUco為目標的視覺靠近。現有sugarbox模型不等於紙盒模型。
5. 確認停靠才產生唯一投放交接token；組員回覆明確投放結果後，再驗證退出及接續巡航。未知結果不重送動作。

組員分支另有舊 `mission_pipeline.py`／`mission_fsm.py` 的選配bbox收尾、raw-LiDAR安全層整合。已抓取並核對，但沒有覆蓋本人的舊回歸入口或啟用另一個Serial owner。那些變更仍屬舊整合架構；目前決賽入口以resident navigation與明確交接為準。PPO主巡航避障留待整條流程穩定後先作影子比較。

## 驗證與未完成項目

離線導航219 passed／1 skipped；組員接近相關152 passed及178 subtests；既有controller138、servo37、FloorGuard641、UI50(skip1)、safety guards73、FSM通過；本次另驗證resident chassis 58 checks通過。

實車已有隨機起點經影像判讀排除角落歧義、回最近巡航點、至投放準備區及AMCL程序中斷恢復的紀錄。**還沒有**未知起點全自動ArUco定位、校正後紙盒視覺停靠、真實投放／退出、不中斷整圈及多輪完整任務驗收。詳細命令與結果在 [本輪操作文件](RESIDENT_NAVIGATION_FLOW_2026-10-18.md)。
