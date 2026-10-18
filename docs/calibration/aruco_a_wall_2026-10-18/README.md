# A 牆實貼標記檢查（2026-10-18）

- 靜止初始朝向未拍到標記；受監督右轉取像後，DICT_4X4_50 成功解碼 ID 0、ID 2，各連續四張影像。
- 兩張標記分別完整入鏡；目前位置與光照可辨識，尚不代表任意位置皆可辨識，也尚未完成相機內參、map 錨點或 AMCL 輔助定位驗收。
- 轉向目標 90° 達脈衝次數上限後停車，沒有放寬上限或繼續補轉。最後輪命令全零、maintenance=true、travel，Serial owner 8117。記錄詳見 rotation.json、turn_report.json。
- 最低 MemAvailable 1394 MB，相機讀取失敗 0；相機與臨時 ROS snapshot 已停止。
- 沿用前次相機測試的 session-only 後右小矩形遮罩，未修改常駐 LiDAR profile。
- confirmed_id0_id2.jpg 為不同朝向兩張辨識成功影像的並排展示，不是同時看到兩張的單一畫面。逐張結果見 turn_detection.json。
