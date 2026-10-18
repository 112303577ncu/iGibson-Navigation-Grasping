# 10/18 常駐定位、隨機起點導航與投放交接

## 決賽流程與責任

定位確認 → 接入最近可達巡航路徑 → 逆時鐘巡航／辨識 → 接近夾取物 → 組員夾取／收臂 → 地圖導航到投放準備區 → 組員視覺精準靠近 → 組員投放 → 確認結果與退出 → 返回巡航。

先穩定上述流程，再將 PPO 避障以唯讀影子模式比較決策，最後才允許透過同一控制交接介面輸出速度。現在的地圖巡航仍是 AMCL／odom、有界路徑與短脈衝控制，沒有把 PPO 的動作同時送給底盤。

## 已完成與本輪實測

| 項目 | 結果與限制 |
|---|---|
| 隨機起點定位 | 全域 scan 比對產生四組解；最高、次高吻合率約96%，單靠最高分可能選錯角落。限角左轉89.28°後，相機看到紙盒在左前方，排除其他解，再以新鮮 scan 局部比對建立起點。這是本次受監督影像判讀，不是已完成紙盒全自動偵測。 |
| 返回最近巡航點 | 實車完成，AMCL 估計最終誤差4.34 cm，46次有界脈衝，恢復0次。最近點依可達路徑長度選擇，非只比較直線距離。 |
| 前往投放準備區 | 實車完成，AMCL 估計最終誤差3.98 cm，57次有界脈衝，恢復0次。沒有最終視覺靠近或放開物體。 |
| 記憶體 | 兩段導航最低 MemAvailable 約1448 MB；保留開始1024 MB、執行中768 MB閘。沒有把YOLO/SAM搬入Jetson定位服務。 |
| AMCL程序中斷 | 停穩時終止 `/amcl`，約7.25秒自動恢復，唯一Serial owner保持不變。先前測試曾耗盡重啟預算，修正時間讀取競態及短暫輸入異常立即殺程序的行為後重測通過。 |
| 最後狀態 | 位於投放準備區，四輪零、maintenance=true、travel。最後owner PID8117，boot及PID只代表本次紀錄，接續時需唯讀重查。 |

證據：[summary](../calibration/resident_nav_2026-10-18/summary.json)、同目錄的回復測試、兩段navigation報告、影像及seed資料。誤差來自AMCL估計，不能當成卷尺或外部定位的精度證明。

## 常駐定位與修復

- 新服務：`deploy/systemd/x3plus-g2-localization.service`，已在Jetson enable。沒有原診斷服務的2400秒期限，RuntimeMaxUSec為infinity；只管理自己的roslaunch程序群，不重啟grasp-service。
- `deploy/ros/g2_localization_service.py` 使用原始sensor map，先把地圖路徑轉成絕對路徑；map_server/amcl為required節點。禁止拿膨脹通行mask做定位。
- `/dev/shm/g2_localization_seed.json` 綁定boot ID、owner PID、raw-map hash、新鮮odom與來源。重開機、換owner、失效或大幅移動不沿用舊起點。沒有有效參考就等待，不自行猜角落。
- 健康狀態：`/dev/shm/g2_localization_health.json`。短暫資料中斷先回報不就緒，保留AMCL；持續異常才重啟子程序，10分鐘最多3次、至少間隔10秒。RAM壓力立即停止定位子程序；錯誤seed／反覆失敗等待新參考。
- 巡航每次輸出前同步檢查監督器及原scan／odom／TF／covariance。恢復需5個不同odom幀；路徑以恢復後的位置重算，不重播異常前速度。等待有上限，並非無限重試。
- 此監督器不會在重開機後從未知任意點自行辨別四個對稱角落；這項能力仍需已登錄ArUco或可靠外部線索。

## 入口與交接

以下在Jetson專案根目錄，使用 `/home/jetson/grasp_venv/bin/python3`。省略 `--real` 是唯讀檢查；先有有效定位才能規劃。

```bash
# 回最近的既定巡航點
python3 grasp/v23/demo.py navigation --task return --scene maps/demo_box_20261018_patrol --output /tmp/return.json --real

# 從最近可達路段接入，依逆時鐘繞行一圈
python3 grasp/v23/demo.py navigation --task patrol --scene maps/demo_box_20261018_patrol --output /tmp/patrol.json --real

# 前往投放準備區；未配置視覺目標時留下WAIT_VISUAL_TARGET交接狀態
python3 grasp/v23/demo.py navigation --task dropoff --scene maps/demo_box_20261018_patrol --output /tmp/dropoff.json --real
```

直接地圖點入口為native ROS Python：`deploy/ros/g2_map_patrol.py --point X Y`；語意目標為 `--destination nearest-patrol|patrol-cycle|dropoff-preparation`。前提是當前位置、目的地在核准通行mask，且有可達路徑。「任意點」不包含牆外、禁航區或沒有可靠定位的位置；不會偷偷把起點吸附進合法區。

`integration/v23_delivery_navigation.py` 將各步驟結果寫成固定報告。返回碼0是該導航階段完成，2是到準備區後等待視覺配置，1是故障。`release_ready=false`／`mission_complete=false` 不得當成投放完成。視覺停靠成功才產生唯一release_token供組員交接；程式不發release、不重試結果不明的手臂動作，退出與續巡航仍需投放結果及退出驗收。

## koala模組與共同Serial

- 精選匯入koala `codex/approach-lidar-frame` eafba63及v23分支3eb79的接近模組、純判斷、校正支援與測試；沒有合併整個分支。來源及hash見 `docs/calibration/resident_nav_2026-10-18/upstream_import.json`。
- koala新版入口本來已走TCP7000；新 `integration/v23_dock_approach.py` 重用其限幅對準律，加上明確dropoff角色、目標ID、標定ID、新幀、距離閉迴路、有界脈衝、通行mask與停止回報。不沿用盲走固定15 cm的收尾。
- 視覺producer應在電腦端運行，向Jetson提供latest-only JSON：`valid,role='dropoff',target_id,calibration_id,frame='base_footprint',coordinate_convention='x_forward_y_left',stamp,seq,x_m,y_m`。時間需與Jetson同步，來源影像至判斷總延遲應納入stamp；不得只在傳送時重打時間使舊影像變新。
- 模板 `deploy/ros/dropoff_visual_target.example.json` 尚未校正，預設禁止輸出。standoff以四輪中心量測，不是車體前緣。現在sugarbox模型是夾取物，不是紙盒。需組員紙盒模型或ArUco提供正確投放目標，不能僅改名稱冒用。
- `integration/resident_motion.py` 僅發既有resident TCP速度／stop，每次非零輸出需guard，沒有背景自動續發。
- 舊vision/nav入口的 `--resident-approach` 轉到此runner；未校正的舊整合grasp仍禁止real。舊獨立motor server與兩個直接Serial校正入口在開埠前明確拒絕；resident server只匯入速度映射，不受影響。沒有覆寫Jetson組員的手臂控制器。

## ArUco列印與安裝

- [8張列印版](../../output/pdf/aruco_walls_8_120mm_A4_two_per_page.pdf)：DICT_4X4_50，ID0～7，共4頁，每頁2張，黑框120 mm，100%實際大小列印。
- [配置圖](../../output/pdf/aruco_walls_placement_2m.pdf)：A=紙盒後牆、B=紙盒左牆、C=A對面、D=B對面；每面約50／150 cm各一張。ID0留在投放區附近，ID1在對角側。
- 貼硬背板，中心先與後相機等高、面向場內；低矮軟牆需要外側固定支架。避免反光、彎曲、遮擋，不把支架放進巡航通道。標記改位就要更新座標。
- 每面兩張是覆蓋改善方案，不保證任意點任意朝向都看得到。初始定位須有停穩、限角轉向搜尋、多幀確認；找不到就等待定位。已知定位後暫時不可見，可在odom／LiDAR／AMCL持續健康時巡航。
- 印後先量黑框，實測相機內參／外參、各標記位置及朝向，再登錄map。配置圖座標僅是安裝示意，不能直接當ROS map座標。
- PDF已逐頁渲染、8個ID解碼及向量120 mm尺寸檢查；尚未實體列印／鏡頭覆蓋驗收。生成器和manifest在本輪證據目錄。

標記識別與相對姿態需要字典、實際標記尺寸與相機標定；參考[OpenCV官方說明](https://docs.opencv.org/4.1.2/d5/dae/tutorial_aruco_detection.html)。全場覆蓋與安裝數量為本場地工程規劃，仍需實測。

## 測試與下一步

離線導航／恢復／交接219 passed、1 skipped；koala相關152 passed、178 subtests（Python3.11，AST測試需3.9以上；Jetson部署碼維持3.8）。controller138、servo37、FloorGuard641、UI50（skip1）、safety guards73及FSM通過。Windows中文URDF路徑透過既有測試程序adapter驗證，沒有修改生產驅動。

下一步依序：列印安裝與登錄8個標記 → 未知起點自動搜尋定位 → 新模式不中斷完整巡航 → 紙盒視覺producer／組員接近介面 → 真實投放結果／退出／續巡航 → 多輪整合 → PPO影子比較再接管。這些尚未完成的硬體驗收不以靜態測試取代。GitHub來源及發布範圍見 [同步核對](GITHUB_SYNC_2026-10-18.md)。
