# LiDAR 10cm修正與局部投放區規則

## 已部署

依使用者修正，LiDAR中心在車體中心前10cm。新增共用g2_geometry.LIDAR_FORWARD_M=0.10，巡航前距換算與建圖監督客戶端使用同值，TF也改10cm；既有一般G2/turn client本來就是10cm。固定自回波profile使用laser原始角度/距離，不因這次量測更正自動重學或擴大。

在底盤零輪/維護鎖下只重啟x3plus-g2-lidar，未重啟grasp-service。實機TF base_footprint→laser=(0.10,0,0.095)m，scan frame=laser、stamp age0.126秒；owner27713、odom有效、travel、四輪零。此輪沒有底盤/手臂移動。

既有demo_box地圖以13cm TF建立；記錄sensor_geometry與未驗證的新10cm幾何。地圖導航實機入口在未完成相容性/重定位驗證前拒絕移動，不將舊pose種子冒充已驗證。這不表示一定要重建整張圖：可先檢查新TF下定位/掃描一致性，若不合再重建。舊巡航數據保留為13cm設定下的結果。

## 巡航是否使用iGibson模型

沒有。g2_map_patrol.py使用原生ROS AMCL+odom定位、直線或有界A*靜態路徑、沿線前視修正及有界速度脈衝，LiDAR做停車/資料閘。沒有載入導航PPO，也沒有呼叫其predict。訓練導航權重仍在integration/nav_best_model，載入入口在integration/nav_rl.py的控制器。G2工具引用其中的幾何/前方煞車函式，不等於啟動PPO。常駐手臂模型與導航模型分開，不把巡航成功當作iGibson避障實機驗收。

## 40cm是否擋住停靠

限制分別為中心牆距40cm、紙盒外緣到車體中心約28.3238cm的緩衝、轉向6cm漂移預留、預計停止路徑，以及前牆煞車（巡航50cm、短段停靠實驗35cm）。

核對最後一次被擋的預計停止端點map(1.38226,0.79033)：擬合牆距43.10cm，紙盒外緣距27.83cm。這一點主要違反28.32cm紙盒緩衝，而非40cm牆距；換成30cm牆距仍被擋。加上現場紙盒偏左20°/車體前緣至盒緣30cm與圖不符，不能只放寬參數後繼續向舊候選點移動。

## 已建立但未啟用的局部草案

- 一般巡航中心牆距仍40cm。投放候選點附近才可用30cm牆距，範圍沿候選朝向前後[-28,+12]cm、中心線左右各12cm。
- 保留已知空地、raw occupied/unknown距離至少30cm、擬合牆內至少30cm、原紙盒28.32cm緩衝。新範圍只增加34個2cm格，不是整個場地放寬。
- g2_build_dock_mask.py輸出獨立PGM/JSON；Policy.enable_dock_zone驗證新幾何、紙盒實體註冊、啟用旗標、圖原點/解析度/尺寸/hash。g2_dock_probe --local-dock-zone只能搭配短段模式，先在普通範圍對正，最後靠近才啟用。框界與停止投影仍逐次檢查。
- 現在enabled=false、registration_verified=false，保留未通過狀態。先核對新TF定位與紙盒實際位置、手臂投放可達範圍，再依正確目標重建局部走廊。尚未執行放置/退出，不能宣稱草案解決了精準停靠。

離線57項通過；五組既有回歸通過（controller138、servo37、floor641、UI50/skip1、mission selftest）。UI首次巢狀測試在程序停止項目中斷，獨立重跑通過，兩份log均保留；Windows URDF仍只在測試程序轉相對路徑。證據docs/calibration/geometry_dock_2026-10-17/，局部草案maps/demo_box_20261017/dock_zone_candidate.json與navigation_dock_zone_candidate.pgm。本輪未push。
