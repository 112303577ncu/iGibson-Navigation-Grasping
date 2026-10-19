> 最新接手狀態：請先讀 [koala小工具交接](../../operations/KOALA_CORNER_TOOL_HANDOFF_2026-10-19.md)。正式registry已擴為v2（0／1／2／5／7），巡航碰撞驗收失敗，現在暫停；下方保留較早階段紀錄。

## 10/19後續碰撞回報：巡航驗收失敗，暫停實車移動

使用者將車移向場內後，attempt4_center/以新相機ID2／7及scan重新初始化AMCL成功；外側最近路段接入後因turn drift reserve outside mask停車（2脈衝）。未放寬mask；後改用臨時75cm inset測試圈，attempt5_patrol_watch/記錄116脈衝、5次到點、AMCL估計最終誤差4.01cm、RAM最低1484.09MB，PC同步169張YOLO觀察，12張候選、0次三張穩定確認，未發夾取／投放。

**使用者回報撞到糖果盒，以上只證明軟體路線完成，不能視為碰撞／找物驗收通過。** 已另外發三次stop並maintenance-lock，health核對輪零、moving=false、holding=false、travel、owner4443。沒有自行後退或繼續巡航；後相機釋放。原始JSON不改成成功驗收；collision_stop.json及collision_assessment.json記錄覆核。

已知流程缺口：候選未達三幀時繼續移動，標定hull外的目標被排除，且無獨立的低矮物視覺停車閘。精確碰撞時間及接觸部位尚未確定，不能把所有近回波當作線材。先修正第一筆疑似物先停、停穩後確認，以及低矮物／相機盲區覆蓋，再實車測試；不宣稱本次已修好。持續外側路線未修改，完整夾取／投放交接仍未驗收。

# 10/19實車：目前位置的ArUco／LiDAR起點與AMCL

機器人172.20.10.3，boot b42f4b8d-3521-4120-90c0-f9571cb62008。
本輪沒有發底盤非零速度或夾取／投放命令。

## 完成

- 原始後相機影像辨識到ID0（左）、2（中）、7（右），三個不同V4L2來源影格與scan同步；來源影像延遲約25～39ms。
- 部分粗略registry只登錄這三張，以安裝配置的20cm位置誤差界保留不確定性；沒有宣稱metric landmark survey或全場任意位置驗收。
- attempt3/：重新計算LiDAR四候選，正確選擇index1；AMCL回報accepted_reference_stamp，通過五個不同odom健康新幀，結果LOCALIZED。
- amcl_child_recovery.json：只終止監督器自己的AMCL子程序群組，6.03秒恢復，新PID、Serial owner4443不變。
- idle_watch.json：124.54秒／246筆全部ready、輪零、travel；odom新增2473筆，可用RAM最低1569.27MB。
- final_summary.json為該測試後的完整health／localization；summary.json為精簡驗收及限制。
- robot_chassis_regression.txt：現車60項純邏輯／fake board／localhost回歸通過，測試輸出的「driving」是fake board，不是實車移動。

## 實作時修正

1. **開機停穩時間**：從未開過輪子的底盤，_stopped_since=-inf導致stopped_for_s=null。只增加第一個成功零命令時間供telemetry使用，保留原有手臂／底盤門檻。現車以remote_chassis_before.py為基準精選套用，不覆寫手臂controller／深度校正。明確服務重啟前owner8033，後為4443。
2. **地圖byte hash**：active_map.yaml保留現車的絕對image路徑，所以hash2274a1c2...，不同於repo相對路徑版。matcher新增--reference-map-yaml，核對geometry及本地相同raster，沒有改現車map。
3. **輸入來源byte hash**：attempt2/的camera helper把native JSON重新排版，導致候選來源hash與trusted capture不同；工具在寫seed前拒絕，seed_written=false。改為逐位元組複製後，以新capture重跑成功，未重打舊capture時間。
4. **牆面品質**：最低85%牆點須距地圖牆面5cm內，與搜尋85%截尾目標一致；低截尾均值不能掩蓋17%不吻合的尾部。保留候選表及模糊結果，不直接挑最低分。
5. **共同相機誤差**：新增保守區間覆蓋，要求所有ID共用同一個相機位置／方向；cell近似只擴張可能性，預算耗盡保留歧義。

## 尚未通過

patrol_preflight.json：0脈衝，current centre outside navigation mask。
依當時AMCL估計，車體中心距最近擬合牆約29.5cm，小於一般40cm中心通行限制。
需先向場內調整並確認入口；不應對這個原因自動一律後退。
其他5張標記、不同起點的初始化、完整巡航／投放／退出尚未在本輪驗收；原LiDAR自遮罩未放寬。

## 檔案與現車狀態

- deployment.json列出配套部署檔案、原／新hash與現車備份資料夾；後續核心與camera helper調整另以final_source_hashes.json核對。
- registry_review.json為影像／名義方位一致性檢查：nominal FOV約61°、原圖可解碼而鏡像圖不能；不是完整內參標定。
- coarse_registry_v1.json是本輪三張部分登錄，正式同內容存maps/demo_box_20261018_patrol/aruco_corner_landmarks.json；8張example仍停用。
- 三個常駐服務正常，最後輪零、travel、maintenance=true，後相機reader已釋放；沒有關機，尚未push。
