# 接續入口（2026-10-18）

## 最新：常駐定位與隨機起點導航

先讀 [本輪整合入口](docs/operations/RESIDENT_NAVIGATION_FLOW_2026-10-18.md)。以下較早段落的PID7157、AMCL缺失及尚未修復的舊入口敘述不代表最新狀態。

- Jetson172.20.10.3重開機後owner8117；唯一Serial owner不變。x3plus-g2-localization.service已enable、無40分鐘期限，無有效boot/owner/map綁定seed時等待，不猜角落。短暫故障先等待，持續故障有限重啟；實測終止AMCL後7.25秒恢復。
- 隨機搬車後scan找到四個相似解，最高分非正確角落。限角左轉89.28°後，影像看到紙盒左前方，據此排除歧義再用新鮮scan定位；這是本輪受監督影像判讀，尚非未知起點全自動定位。
- 實車回最近巡航點與導航至投放準備區均完成，AMCL估計最終誤差4.34／3.98 cm，恢復0次，最低可用RAM1448 MB。最後停在投放準備區、輪零、maintenance=true、travel。未執行最終視覺停靠／投放／退出。
- 新入口 grasp/v23/demo.py navigation --task return|patrol|dropoff；巡航可接入最近可達路段，最近既定點依合法路徑長度選擇。禁航區或無可靠定位不算可自動起步的任意點。證據 docs/calibration/resident_nav_2026-10-18/summary.json。
- koala新版接近模組已精選匯入（eafba63／3eb79），不整枝合併；舊獨立Serial入口已阻擋並提供resident視覺入口。新dropoff adapter只接校正後明確投放目標，現有sugarbox模型不能假裝辨識紙盒；ArUco與紙盒producer尚待實測接入。
- 新PDF output/pdf/aruco_walls_8_120mm_A4_two_per_page.pdf（4頁、每頁2張、黑框120 mm、ID0～7）；配置圖 output/pdf/aruco_walls_placement_2m.pdf。每面50／150 cm各一張，硬背板／場外固定支架、鏡頭等高。仍需列印、量測與map登錄，不能保證任何朝向都可見。
- 決賽採定位→接入巡航→辨識／夾取→準備區→組員視覺靠近／投放→確認結果／退出→續巡航；流程穩定後再加PPO影子測試及接管。當前巡航未使用PPO避障。
- 離線導航219 passed/1 skipped，koala152 passed及178 subtests；既有controller138、servo37、FloorGuard641、UI50(skip1)、safety guards73及FSM通過。GitHub來源及本次發布範圍見 [同步核對](docs/operations/GITHUB_SYNC_2026-10-18.md)，不覆寫組員手臂控制碼。

決賽目標：定位 → 巡航辨識 → 接近夾取 → 收臂運送 → 紙盒投放 → 返回巡航。
112303577ncu 主責導航／定位，夾取與投放由組員負責。分支 v23-grasp-test。

先讀 [10/18外移巡航與停靠](docs/operations/OUTER_PATROL_DOCK_2026-10-18.md)，只讀下一步需要的檔案。較早的NAVIGATION_RETEST紀錄保留原測試，不蓋過以下更新。

## 10/18後續更新（優先於下方較早紀錄）

- 巡航改用新scene的outer_patrol_goals.json，沿牆約55 cm、盒角切角，一般中心牆距40 cm不變。搭配--boundary-micro --keep-heading。初次因49–53 cm真實牆面煞車中斷，修正為短脈衝模式後接續完成；未不中斷重跑整圈。最後位置檢查通過，誤差4.16 cm。
- 已監督式進位29.52 cm；AMCL不確定中斷後保留原累積目標續接，未另走30 cm。現場再要求前進5 cm、右30°，實測4.67 cm與29.93°，使用者確認「已抵達」。沒有實物投放、自動重複停靠或退出驗收。
- 確認停靠位置只記odom及人工確認；最後AMCL診斷期限到期、定位資料缺失，最後odom約[1.0338,0.8202,0.2652]不是map。last_pose_seed已標失效。下一次導航先重定位並驗證退出，不能從盒旁直接跑普通巡航。
- 持續profile未改；轉向新增--confirmed-right-structure，只在明確opt-in時排除兩個已確認右側小矩形，不忽略整片後方。前述49–53 cm牆面保留。
- 相機可看到盒子，但尚未接視覺閉迴路或ArUco。現模型為sugarbox，不能當紙盒模型；舊vision_grasp_pipeline會自行持有Serial，只可轉接判斷邏輯到現有resident TCP7000。
- 新增deploy/ros/g2_relative_dock_probe.py，量測／續接參考限當次boot/owner。離線172 passed/1 skipped，既有回歸通過；最低可用RAM約1.59 GB。最後輪零、maintenance=true、travel、owner7157。成果尚未push。
- 交件優先：受監督常駐定位、ArUco絕對參考、視覺相對停靠、組員手臂／投放介面、退出及整輪重複。正式demo不沿用AMCL的40分鐘診斷期限。

## 較早的同日紀錄（更新以以上為準）

- Jetson172.20.10.3，部署 /home/jetson/Documents/deploy_jetson2。唯一Serial owner最近PID7157，輪零、maintenance=true、travel、空爪；須唯讀重查。
- 無ArUco，依操作者起點描述及新鮮scan比對重定位，非未知位置全自動定位。LiDAR前移10 cm的scan/TF及巡航角點路線已有實測。
- 使用 maps/demo_box_20261018_patrol、--inset .70。原一般中心牆距40 cm保留，加保守移盒排除區，未放寬通行區；舊10/17 scene保持失效狀態。
- 逆時鐘一圈角點均到達，估計誤差3.10–4.43 cm、恢復0次、最低RAM約1600 MB。但最後朝場內轉向帶來約4.3 cm平移，最終距起點8.35 cm，完整最終停車未驗收。
- 已到投放準備區，後續靜止到點估計誤差4.50 cm。現場量車體前緣至盒近緣35–40 cm，表示尚需底盤前進約30 cm；最後進位未做、實際紙盒偏角未確認、精準投放停靠未驗收。
- 盒18×13.5×6 cm，沿原角落距兩牆20–25 cm範圍作保守排除。準備區mask合法不代表精準盒位已註冊；局部30 cm停靠走廊及正式dropoff別名仍不可用。
- 巡航入口新增--inset、--face-point、轉向後6.5 cm終點重查。最後一項已離線驗證及部署，未實車重跑整圈；不能把偵測偏移當成修好偏移。
- 暫時AMCL為g2-localization-20261018.service，有2400秒期限。新scene種子僅此次boot/owner且未搬車可參考，重啟／搬車須重定位。
- 仍是AMCL/odom與有界路徑控制，未用iGibson PPO。無新回波profile、無實物投放、無整輪任務驗收。

## 下一步

1. 唯讀查唯一Serial、輪零、手臂、板回傳、RAM、scan/odom/TF及AMCL有效性。導航只走resident TCP7000。
2. 修正原地轉向平移，重跑最終停車；不直接放寬誤差或改成功旗標。
3. 與組員核對最後停靠位置／朝向及手臂可達座標，註冊盒位後測進位／投放／退出。35–40 cm盒距和30 cm建議前進量不可混用。
4. ArUco列印後量黑框、固定支架、驗相機參數與marker map座標，再接定位輔助。
5. 接通完整巡航→辨識→夾取→運送→投放→續巡航及故障恢復。

最新離線154 passed/1 skipped，既有controller/servo/floor/UI/FSM通過。Windows URDF中文路徑僅測試程序轉接，未修生產問題。證據 docs/calibration/nav_retest_2026-10-18/，10/18尚未push。

新步驟約需1 GB可用RAM，導航低於768 MB退出子程序。例行安全確認已有授權，不重問；量測、擺物、實際故障資訊才詢問。勿整份覆寫Jetson組員的手臂/service相容性修正。
