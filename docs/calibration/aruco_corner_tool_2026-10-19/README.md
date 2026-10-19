# ArUco粗略角落選擇工具：離線證據

沒有連線／移動機器人，沒有寫入真實AMCL seed，尚未部署或push。

- replay_snapshot.json：沿用10/18實際LiDAR／odom資料，額外加入**模擬**capture／boot綁定欄位。
- replay_landmarks.json：配置圖估計，測試中模擬確認旗標；simulation_only=true，工具禁止用於real提交。不是已量測登錄。
- replay_observations.json：使用當時人工紀錄「ID2左前、ID7右前」，建立三影格介面模擬，時間／seq不是重新從影像取出的三張來源資料。
- replay_hypotheses.json：以更新後g2_scan_match.py對這份實際scan重新計算四個候選及來源hash。
- replay_selection.json：僅候選3符合標記方向，與當時人工監督選擇一致；SELECTED_DRY_RUN、localized=false、seed_written=false。
- pytest.txt：選候選、輸入綁定、去重、重複請求只查結果、停穩／RAM、AMCL ack、native ROS mock及既有定位／導航回歸。
- controller／servo／floor_guard／ui／fsm.txt：AGENTS規定的既有離線回歸。

預設正式範本maps/demo_box_20261018_patrol/aruco_corner_landmarks.example.json仍保持未驗證；本目錄測試不能代替實際貼位、相機方向與時效確認。
