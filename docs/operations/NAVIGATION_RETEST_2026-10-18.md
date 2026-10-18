# 巡航與投放準備區測試（2026-10-18）

## 結果與驗收界線

- 無 ArUco。操作者提供「接近巡航起點、紙盒在左前方」的先驗，以新鮮 LiDAR 比對既有地圖後啟動 AMCL；不是未知起點全自動重定位。
- 逆時鐘一圈全部角點到達，停穩到點估計誤差3.10–4.43 cm，159次有界脈衝、恢復0次，最低可用RAM1599.76 MB。
- **最終停車尚未完整驗收**：到起點時誤差4.43 cm，隨後朝場內轉向，odom又平移約4.27 cm，最後map估計距起點8.35 cm。原始報告的ok=true發生在新增轉向後位置檢查之前，不能單憑該值接受最終停靠。
- 從起點附近導航至投放準備區成功，15次脈衝、恢復0次；到點估計誤差4.43 cm，後續新鮮靜止讀取為4.50 cm，最低可用RAM1603.68 MB。
- 操作者量車體前緣至紙盒近緣約 **35–40 cm**，表示仍需底盤前進約 **30 cm** 才能投放。只驗證到準備區，**不是實物投放停靠驗收**。實際紙盒偏角尚未取得，未盲目前進最後30 cm，未動手臂。
- 誤差來自AMCL/odom，無外部尺量車體中心位置；不代表全程物理精度、故障恢復、動態避障或全場視覺覆蓋通過。

## 本輪設定

- Jetson172.20.10.3，唯一Serial owner PID7157；開機手臂自動travel成功，板回傳新鮮，輪零，scan約10 Hz、odom約20 Hz。
- TF base_footprint→laser=(0.10,0,0.095)m、yaw π。移入後480個抽樣牆線比對點約94.17%距occupied格小於5 cm，P90約3.26 cm；這不是車體位置誤差。
- 初始中心距右牆約32 cm，未進入40 cm中心限制；操作者保持朝向向左移入，重新擷取掃描並重設AMCL，未沿用移動前初值。
- 新場景 `maps/demo_box_20261018_patrol/`，舊10/17場景保留失效旗標。原40 cm中心限制、occupied/unknown限制保留。
- 紙盒兩面牆距20–25 cm全範圍加4 cm註冊餘量建立保守矩形，再保留28.32 cm盒緩衝；新mask是原mask再縮小，移除186個原可用格，沒有擴大通行區。精準盒位與局部30 cm停靠區仍未啟用。
- 原60 cm內縮巡航被新排除區拒絕；70 cm路線及角點6 cm轉向預留檢查通過。
- 沒有擴充自回波profile或忽略後半圈；未觸發回波停止不代表外物辨識已完整驗收。

## 程式與測試

`deploy/ros/g2_map_patrol.py`新增：

1. `--inset 0.60..0.90`調整巡航內縮，保留逐段mask檢查。
2. `--face-point X Y`到點後朝指定map點轉向，不因此允許接近該點。本次使用盒候選中心作近似朝向，未標成實體已註冊盒位。
3. 轉向後重新停穩並量終點誤差；超過6.5 cm回報 `final turn displaced centre beyond goal tolerance`。**最後這項只完成離線測試與部署，未實車重跑整圈**；偵測偏移不等於已修好轉向平移。

只改巡航入口，留存三階段 `.before_*_20261018`備份，未重啟Serial owner或覆寫組員手臂／service。

相關pytest **154 passed、1 skipped**。既有controller138、servo37、floor641、UI50（skip1）、FSM通過。Windows直接跑controller/floor遇既有中文絕對URDF路徑問題，僅在測試程序轉相對路徑，controller輸出改UTF-8；未改生產URDF邏輯，失敗與成功log均保留。

## 接續操作

1. 唯讀確認owner、fresh scan/odom/TF、RAM與travel。暫時AMCL為 `g2-localization-20261018.service`，期限2400秒、MemoryMax220 MB；失效須重新定位。
2. 新場景last_pose_seed僅此次boot/PID7157且未搬車可參考；重啟／搬車不能當冷開機初值。最後map估計約(0.985,0.645,0.322 rad)。
3. 處理原地轉向約4.3 cm平移，重跑最終停車驗收，不直接放寬誤差或改成功旗標。
4. 與投放組員確認底盤至盒距離、朝向及手臂可達座標，再註冊盒位、最後進位與退出。35–40 cm是盒距，30 cm是建議前進量，不能混用。
5. ArUco列印後再驗相機參數、marker map座標及定位輔助。

實車重跑須有當次可信定位及owner檢查：

```bash
cd /home/jetson/Documents/deploy_jetson2
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
/usr/bin/python deploy/ros/g2_map_patrol.py \
  --scene maps/demo_box_20261018_patrol --inset .70 \
  --real --loops 1 --seconds 360 --output /tmp/patrol_result.json
```

準備點goals JSON為 `[{"name":"dropoff_preparation","point":[1.0255803634,0.6638006272]}]`，可配 `--face-point 1.55994134056 0.83332819454`。這不是實物投放的dropoff別名，後者仍拒絕未驗證停靠。

證據在 `docs/calibration/nav_retest_2026-10-18/`；以summary.json綜合驗收，不覆寫原始patrol報告。最後四輪零、maintenance=true、moving=false、travel、owner未變。機器保持開機，本輪未push。
