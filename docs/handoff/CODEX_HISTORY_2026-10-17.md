# Archived session notes; latest state is in root CODEX_HANDOFF.md

## Latest runtime 2026-10-17 (overrides older entries)

Empty release-preview succeeded; rim 6cm, measured FK pad bottom 11.4cm, returned travel. Owner restarted PID4505; odom reset, old map seed invalid. No real object delivery verified.

Requested right30 partially executed: accumulated right12.14deg, translation0.8cm. Stopped, motors zero, maintenance=true, travel, empty jaw. User confirmed rear fixed structure; session-only CLI exclusions: rear-right x[-18,-6]/y[-22,-13]cm; rear-left x[-20,-15]/y[10,14]cm. Other gates remain. New persistent FRONT-RIGHT cluster x[4.8,6.3]/y[-27.8,-24.6]cm blocks rotation; pending user identification. Do not classify this as rear structure. Resume with original reference and right30_partial.json, NOT another full30deg. Snapshot transient stopped; restart fresh before resume. Tests45passed. Evidence docs/calibration/release_preview_2026-10-17/right30_partial.json. Unpushed.

# Codex 下一步（2026-10-17）

最新現場補充：紙盒需右轉約30°對準，使用者確認目前距離可直接放置，不再要求量前後距離。尚未實際轉向/放置；release是固定向前動作且拒絕空夾爪，需正規空夾爪預演入口或實物測試，不能偽造holding。程式預設盒高5cm與實物6cm不同，須核對常駐cfg。盒map註冊與10cm幾何移動驗證仍未通過，不能直接重開舊巡航。

最新紙盒已移動：兩牆各20–25cm、18cm邊仍平行後牆。舊pose/keepout/dock失效，scene_layout標pose_map_valid_for_current_placement=false、navigation_mask_valid_for_current_bin_placement=false，未啟用局部30cm。22.5cm中點只記候選(1.55994,0.83333,1.12099)，不當成註冊。新10cm TF下AMCL唯讀probe通過、無移動，位置標準差仍約8cm，不能當成精確校正；暫時AMCL已停。新後RGB看不到完整紙盒。已詢問移盒後車體前緣至盒緣距離/左右角度，尚待回答，舊30cm/左20°不可沿用。新require_motion_ready會在盒搬動且mask未重建時擋實車，即使未來幾何通過也不能直接舊路線開走。owner27713、odom原位、零輪/鎖/travel，58相關測試通過。詳docs/operations/BOX_REPOSITION_2026-10-17.md。未push。

最高優先幾何修正：使用者更正車體中心→LiDAR為前10cm，已部署TF/巡航前距/建圖工具共用值。只重啟lidar，實機TF(0.10,0,0.095)、新鮮scan、owner27713/輪零/鎖/travel，無底盤動作。舊demo_box以13cm建，sensor_geometry標待驗證，實機地圖導航入口會拒絕移動直到新TF定位相容性通過；舊seed標幾何待驗，勿直接重用假設有效。可先重驗AMCL，不一定需整圖重建。
巡航未用iGibson PPO：目前AMCL/odom+靜態A*或直線+規則脈衝與LiDAR閘。最後停靠預計停止點實際違反紙盒28.32cm緩衝（預計27.83cm），牆距43.10cm，單改40cm牆距不能解決。已建局部30cm草案（沿dock[-28,+12]cm、左右12cm，只新增34格），普通40cm/紙盒緩衝不改；enabled=false/盒實體註冊false，新幾何/盒位置驗證前不啟用。工具g2_build_dock_mask.py、--local-dock-zone需短段模式。57相關通過；詳docs/operations/GEOMETRY_DOCK_POLICY_2026-10-17.md。未push。

最新紙盒/相機驗證優先：後RGB開啟一圈巡航成功、到點map估計3.66–4.42cm、0恢復，但紙盒精準停靠未通過。前牆50cm停等後新增僅最後短段35cm/≤0.08s靠近模式；最後距舊dock4.994cm因預計停止路徑超mask停止，保留ok=false，未放寬mask/未投放。現場實測車體前緣到盒緣30cm、盒在車頭左20°（取代舊右轉估計）；舊圖預測左1.49°、前向近面約15cm，實體對位不符，scene_layout已標失配，勿直接用舊dock或前進35cm。最終map約(1.30964,0.77609,0.19371)，odom(1.2762,0.7409,0.3012)，owner27713；輪零/鎖/travel/空，vision+monitor active，暫時AMCL/相機/深度已停，仍開機；seed已更新限未搬動。
後相機RGB199幀無讀失敗、最低RAM1466MB；ORBBEC深度用新astra_camera入口成功靜止5幀，但有效深度約54.5–56.8%，不代表全場覆蓋，深度未測完整巡航。後RGB抽樣YOLO imgsz320把車身/場外物誤判sugarbox，未送控制座標，全場辨識未通過。rear H參考的前輪/中心換算疑差13.5cm，僅記錄不改組員檔。53相關測試與五組回歸過。詳docs/operations/DOCK_CAMERA_2026-10-17.md及docs/calibration/dock_camera_2026-10-17/。ArUco先兩張場外硬支架朝內、高過牆遮擋；視野測後再補四角。下一步紙盒實體註冊/定位核對、相機辨識排除/實體目標覆蓋，再單障礙停等恢復；動態繞障未驗收。本輪未push。

最新路徑修正實測：四點逆時鐘+自訂A(0.65,0.45)→B(0.40,0.65)→home均成功，視覺/monitor開啟，owner27713未變；到點map估計3.54–4.55cm與4.14–4.33cm，0恢復，最低RAM1592.94MB。沿原線前視修正降低切角，巡航停穩樣本最大偏線6.86→4.02cm（排除進起點；非尺量/非連續運動上界）。新增有界A*與整份goals預檢；dropoff明確拒絕，停靠與退出尚未驗收。相關47通過、五組回歸通過。結束輪零/維護鎖/travel/空夾爪；暫時AMCL已停、vision/monitor active、機器仍開機。last_pose_seed已更新且附odom，僅未搬動時有效。詳docs/operations/NAV_PATH_REVIEW_2026-10-17.md。兩張12cm ArUco SVG與放置規劃在docs/calibration/aruco_2026-10-17/，尚未實體校正/接入定位。本輪未push。

正式分支是112303577ncu/iGibson-Navigation-Grasping的v23-grasp-test。

## 10/17 現場更新（優先於下方10/16紀錄）

最新已完成固定紙盒場地的巡航/移動AMCL兩圈：主巡航點為紙盒對角內縮60cm的map(0.10093,0.30826)，四點逆時鐘返回。第一圈視覺暫停、第二圈視覺/monitor開啟，均成功、0次過期恢復；到點map估計誤差6.6–8.7cm，非尺量。第二圈含最後朝場內，最終主點估計誤差8.91cm、朝向誤差4°、最低MemAvailable1593.90MB。owner全程27713。`g2_map_patrol.py`以原生ROS+map←odom+AMCL、獨立40cm中心遮罩（含紙盒）逐次檢查及預計停止路徑，限制已在此入口生效；TCP7000其他client未全域受限。記錄docs/operations/PATROL_2026-10-17.md、docs/calibration/patrol_2026-10-17/與patrol_verified.png。相關43通過、五組回歸通過；Windows URDF僅測試程序相對路徑轉接。
車目前停在對角巡航主點附近、朝場內，零輪/maintenance=true/巡航位/空夾爪；vision/monitor active。巡航與AMCL暫時服務均已停止，下一次需有合理初始pose重開定位，不使用過期TF；重用launch為deploy/ros/g2_scene_localization.launch（初始pose必填）。last_pose_seed.json僅限車未搬動且參考仍有效。整合接近/夾取/紙盒投放、負載運送、冷開機重定位/動態障礙仍待驗證，尚未接入v23 demo plan。本輪尚未push。以下「禁航未接入/移動AMCL未測」是較早狀態，以本段為準。

最新紙盒定位量測：後牆距6.5cm、左牆距10cm、18cm長邊平行後牆。scene_layout已填footprint與斜向dock候選：紙盒map(1.7584,0.8763)、dock(1.3594,0.7802,yaw0.23635rad)，中心投放距41.04cm，需驗證手臂可達/落盒精度。照片後牆=u最大、左牆=v最大是依照片/初始朝向的角落對應假設，尚未實測確認。新增紙盒外中心緩衝約28.32cm（車體角點半徑+5cm），404個巡航點重驗通過；限制圖仍未runtime生效。最新設定/預覽已同步機器；下方「仍待距牆量測、pose=null」為舊狀態。沒有本輪手臂/底盤動作。

紙盒最新尺量18×13.5×6cm，已記scene_layout.json；盒高低於9.5cm雷射平面。仍待照片後方牆/左側牆到盒緣距離，以及18cm長邊朝向；未得到前box pose/dock保持null，不猜位置。此更新優先於下方「未收到尺寸」敘述。

最新為含紙盒的新四邊建圖 maps/demo_box_20261017/，2cm/格、牆線估計1.978×2.022m；全程本SLAM會話已啟動。生成中心距牆/occupied≥40cm的獨立限制圖（不給AMCL），候選路線50cm內縮、404點最小障礙距42.05cm；尚未runtime接入，不可宣稱禁航區生效。紙盒不是紅桶；圖中無可明確認定紙盒的內部群聚，已請使用者量長寬高及兩個距牆距離，未收到前bin/停靠pose保持null。新圖已做5次帶GMapping初始先驗的靜止AMCL更新，可運行、非移動/重定位驗收；暫時AMCL已停。流程仍為巡航→辨識/接近→夾取→收臂運送→導航紙盒停靠→投放→收臂/續巡航。詳見 docs/operations/MAPPING_BOX_2026-10-17.md。
本輪轉2在約89.1°觸發5cm平移上限，未標示精密校正通過；確認停止/朝向後續掃。修正g2_mapping_route cleanup等待停止回授再上鎖，即使失敗仍記錄狀態，相關33通過，已部署。owner仍PID27713、開機中172.20.10.3；建圖/快照/AMCL暫時程序已停，vision/monitor已恢復。本輪所有變更尚未push。

追加原地360°重掃：四次odom合計361.10°，平移約4.56cm。末段91.5277°略超過91.5°允收，原報告ok=false保留；掃描已覆蓋一圈。新圖 maps/demo_2x2_spin_20261017/ 明顯斜牆/重影，拒絕取代前版沿牆圖；原主候選仍 maps/demo_2x2_20261017/。建圖/快照已停、vision/monitor恢復，輪零/巡航位/maintenance=true。尚未部署固定禁航區，50cm是直線中心煞車門檻，38cm是原路線靜態最小中心距；後續40cm中心邊界僅建議，未部署。固定投放容器在最終建圖前放置、停靠點/朝向後標註。見新圖README與comparison.png，不將原巡航點直接套新圖。

使用者修正手臂後home→stow成功；重啟owner PID27713，startup travel_verified、TCP ready。目前開機，SSH 172.20.10.3。已在約2×2m場地完成逆時鐘四邊/四次左轉，最後odom距初始原點13.15cm、方向差2.99°，非尺量回圈誤差。原先S3故障已恢復讀值。
新圖與逆時鐘巡航候選在 maps/demo_2x2_20261017/，2cm/格。候選404個中心取樣均在已知空地，最小中心障礙距38cm，但AMCL/實際巡航未驗證。GMapping從第一邊/第一次左轉之後才真正啟動，先前被失效ROS登記擋住；圖中四條牆線已呈現，原圖保留角落缺口。
建圖負載下rosbridge延遲曾中止第二邊/第二次轉彎。改用新本機ROS快照g2_native_nav_snapshot.py與監督式分段工具g2_mapping_route.py，保留原始轉角基準接續，未放寬感測過期門檻。明確confirmed-self-returns只供近車轉彎回波監督覆寫，前方牆面仍煞車，非通用避障驗收。
正式TF與本輪工具用雷射前13cm，其他一般client仍有10cm常數待統一。建圖/快照transient程序已停止，vision/monitor已恢復；最後四輪零、maintenance=true、巡航位、空夾爪。下一步新圖AMCL、角落禁行邊界、巡航候選實測。本輪尚未push。詳見[繞行結果](docs/operations/MAPPING_LOOP_2026-10-17.md)、[初期故障紀錄](docs/operations/MAPPING_2026-10-17.md)。

## 正式目標與下一次現場順序

正式展示：已建圖場地定位 → 巡航尋物 → 視覺接近/E1交接 → 夾取 → 收臂搬運 → 投放點導航 → 投放 → 接續任務。
112303577ncu先做基本冷開機確認，再做雷射座標/濾波對照 → 新地板/新濾波建圖與尺度 → AMCL → 巡航/投放點短路線，最後接全隊流程。定點夾取是模組測試/備援，不能取代導航主線。
詳見[10/17 GitHub與決賽目標核對](docs/planning/V23_FINAL_DEMO_REVIEW_2026-10-17.md)、[正式優先順序](docs/planning/V23_FINAL_DEMO_PRIORITIES.md)。只讀所需檔案，不重新掃描整個專案。

## 10/16成果與部署界線

- 直線odom合計88.85cm、尺量約88–89.5cm；左89.94°/右89.23°均現場確認約90°，保留倍率。微動實際0.6–1.0cm；左5°第1輪odom5.75°僅目測相近，使用者無量角器，精密量測暫緩。一般停止距離/超轉仍須處理。
- LiDAR左右方向已做紙板A/B確認。固定自回波profile已接到/scan；原始資料在/scan_raw，開機不自動學習。外物漏檢對照仍未驗收。車體前20/後13/左12/右12cm含線材；雷射實測前13cm，但正式TF/一般客戶端仍10cm，建圖前要核對統一，不直接套舊地圖。
- 實機原稿以最小補丁新增獨立/tmp/grasp_health.sock與推論心跳/tmp/x3plus_vision_health.json。手臂忙碌時73次健康查詢全成功、最慢10.9ms；home/stow時原生odom/TF最大間隔約65ms。
- v23-control-vision-monitor.service已enabled；限定控制/視覺、無ROS訂閱、無owner自動重啟。相機故障約12–15秒可自行恢復。最後維護鎖保留、底盤零速、巡航位、holding=false。冷開機尚未驗證。
- 部署重啟owner一次，PID30006→2518，odom原點已重置；所有後續動作重建reference，不能續用舊量測座標。完整Serial transport與owner故障恢復尚未部署。
- 監督器使用/home/jetson/health_compat_20261016/recovery四模組bundle；原稿備份/manifest在同目錄。vision有效argv已保存為zz-preserve-deployed-20261016.conf，保留組員allow-top-clipped/frame-report等參數。未覆寫組員校正。
- 曾發現ROS背景記錄影響rosbridge，已改預設RAM-only。RAM記錄依使用者要求停止：23分19秒、685筆，最低1749MB；complete=false代表提前停止，非滿30分鐘驗收，期間重啟程序不能證明無洩漏。所有結果已取回本機。
- 相關105項通過、skip1，五組回歸通過；controller/floor仍需測試程序相對URDF路徑轉接，Windows中文絕對路徑問題未修復。

## GitHub核對與操作限制

10/17核對：自己的origin在本次推送前為e980fa8；koala v23為3eb79ec，最新接近分支為d06bba3（新增10/12實測）；enoch獨立快照仍ebad46e。只fetch/review，未合併owner/隊友分支。
現行v23 plan只有home/grasp/stow/release/短直線drive；完整地圖巡航/投放導航入口仍待接通。舊mission_pipeline是獨立Serial且拒絕實機，不得直接啟動取代常駐owner。
不要再問已授權的現場安全；僅量距/量角/擺放物件時提示配合。保留真實資料/狀態閘，不偽造通過，不重播未知夾取/投放。不開第二Serial owner，不覆寫組員，不自行合併owner。

## 記錄入口

- [新地板里程計](docs/operations/ODOM_NEW_FLOOR_2026-10-16.md)
- [左右90°](docs/operations/ODOM_LEFT_90_2026-10-16.md)、[小角度](docs/operations/ODOM_SMALL_ANGLE_2026-10-16.md)
- [LiDAR固定濾波](docs/operations/LIDAR_SELF_FILTER_2026-10-16.md)
- [健康相容介面/自動修復/關機結果](docs/operations/HEALTH_COMPAT_RECOVERY_2026-10-16.md)
- [開機穩定性測試](docs/operations/BOOT_STABILITY_TEST_2026-10-16.md)
- JSON與有限log：docs/calibration/odom_new_floor_2026-10-16/。
- 10/16逐段舊交接已保存在[封存](docs/handoff/CODEX_HANDOFF_2026-10-16_ARCHIVE.md)，其中「等待回覆/未部署/正在記錄」為當時狀態，以本檔為準。
