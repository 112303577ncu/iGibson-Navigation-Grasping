## 2026-10-22 投放走廊入口（離線、未部署）

先讀 [投放走廊](docs/operations/PLACEMENT_CORRIDOR_2026-10-22.md)；`demo.py mission`統一進新入口。
需新準備schema2 runtime與Jetson新版 `g2_map_patrol.py --face-yaw`，只更新其中一端不能運行此模式。
盒位、地圖禁航區與實測旗標未擅自放寬；新guard原始碼hash須與PC規劃端相符。

## 2026-10-22 任務恢復入口（離線、未部署）

先讀 [任務故障表與限制](docs/operations/MISSION_RECOVERY_2026-10-22.md) 及CLAUDE最上方規格。
PC入口與Jetson監督器須配套；準備固定e8766f0完整runtime後才用新入口。
本版未SSH、未開實車。歷史PID／seed／停車狀態不是當前值；硬體驗證旗標仍保留false。

## 2026-10-21 八 ID 登錄與重新定位

正式地標表 v3 已含 ID0～7，保留 20cm 粗略位置誤差；PC／Jetson 雜湊一致。
新相機 ID0＋ID3 已唯一選解並獲 AMCL ack、LOCALIZED；詳細證據與 producer registration_id 見
[八 ID 正式表](docs/operations/ARUCO_REGISTRY_2026-10-21.md)。下方五 ID 與等待下次開機為較早紀錄。

## 新對話接續入口（2026-10-19 暫停後）

先讀 [完整接續與下次開機順序](docs/handoff/NEXT_CHAT_2026-10-19.md)。GitHub已推送e359fb5；目前暫停、輪零且鎖定，未關機。下次啟動須重新檢查與定位；先修正低矮物／牆夾誤辨識，再接巡航夾取。

## 10/19後續碰撞回報：巡航驗收失敗，暫停實車移動

使用者將車移向場內後，attempt4_center/以新相機ID2／7及scan重新初始化AMCL成功；外側最近路段接入後因turn drift reserve outside mask停車（2脈衝）。未放寬mask；後改用臨時75cm inset測試圈，attempt5_patrol_watch/記錄116脈衝、5次到點、AMCL估計最終誤差4.01cm、RAM最低1484.09MB，PC同步169張YOLO觀察，12張候選、0次三張穩定確認，未發夾取／投放。

**使用者回報撞到糖果盒，以上只證明軟體路線完成，不能視為碰撞／找物驗收通過。** 已另外發三次stop並maintenance-lock，health核對輪零、moving=false、holding=false、travel、owner4443。沒有自行後退或繼續巡航；後相機釋放。原始JSON不改成成功驗收；collision_stop.json及collision_assessment.json記錄覆核。

已知流程缺口：候選未達三幀時繼續移動，標定hull外的目標被排除，且無獨立的低矮物視覺停車閘。精確碰撞時間及接觸部位尚未確定，不能把所有近回波當作線材。先修正第一筆疑似物先停、停穩後確認，以及低矮物／相機盲區覆蓋，再實車測試；不宣稱本次已修好。持續外側路線未修改，完整夾取／投放交接仍未驗收。

# 接續入口（2026-10-18）

## 10/19實車：相機＋LiDAR初始化AMCL已成功（未push）

- Jetson172.20.10.3，boot b42f4b8d-3521-4120-90c0-f9571cb62008。配套部署AMCL監督器／task_resume_guard／corner工具後，以V4L2來源時間的實際三張影格（ID0左、2中、7右）排除候選，寫seed並通過AMCL ack及五個健康新幀，結果LOCALIZED。此位置單次實測，未完成所有任意起點驗收。
- 初始owner8033從未移動時stopped_for_s=null；已只在現車chassis精選加入first-zero telemetry，不改手臂controller。明確重啟後owner4443、travel、空爪、輪零。現車60項chassis回歸通過。
- AMCL child中斷後6.03秒恢復、Serial owner維持4443；隨後124.54秒／246樣本全ready，odom增加2473筆，等待段RAM最低1569.27MB。新supervisor/unit不再是「尚未部署」。
- 正式**部分**粗略表maps/demo_box_20261018_patrol/aruco_corner_landmarks.json目前只確認0／2／7。8張example仍false，其他ID、其他位置待驗證。位置仍保留20cm誤差界，沒有宣稱metric ArUco標定。
- 新g2_corner_camera_capture.py可在後相機未被其他reader持有時取得來源時間三幀＋同步scan；原始capture須逐位元組複製，不能重新排版破壞hash。已修正首輪實際整合拒絕案例。
- 現車map YAML含絕對image路徑，hash2274a1c2...；Windows匹配須用下載的active_map.yaml搭配--reference-map-yaml，保留現車地圖不改。
- 巡航僅做preflight，0脈衝，原因current centre outside navigation mask；先向場內調整再驗證巡航，不能套用一律後退規則。未發底盤非零／夾取／投放命令，相機已釋放，維持maintenance=true。
- 證據與備份manifest docs/calibration/aruco_live_2026-10-19/；操作更新 docs/operations/ARUCO_CORNER_SEED_2026-10-19.md。下方同日「離線待部署」是較早紀錄，以上優先。

## 10/19 ArUco角落選擇工具（已寫，離線驗證，未部署／未push）

- 新`deploy/ros/g2_corner_snapshot.py`可在沒有AMCL時唯讀取得停穩／唯一owner／TF驗證過的scan與odom，產生boot綁定capture ID。筆電用g2_scan_match.py計算，新增capture SHA256與map hash。
- 新`integration/g2_aruco_corner_seed.py`以三個新影格的ID＋畫面左右方向排除候選；唯一可信才允許Jetson `--real`核對live狀態並寫seed，等accepted_reference_stamp及五個健康新幀。預設只預覽，無底盤／手臂輸出。重複同capture只查結果、不重打時間。
- `maps/demo_box_20261018_patrol/aruco_corner_landmarks.example.json`目前是8張配置圖估計，registration_verified及相機方向確認皆false，不能直接當已登錄地標。下次需確認實貼ID／約略位置、相機左右／朝向／視野上界及來源影像時效；不用完整相機PnP標定。
- koala輸入範本`deploy/ros/aruco_corner_observations.example.json`；命令、結果狀態與配套部署見`docs/operations/ARUCO_CORNER_SEED_2026-10-19.md`。此工具是新增的LiDAR＋粗略ArUco橋接，不是metric ArUco producer，也不是投放導航交接工具。

## 10/19 koala 更新與地圖／視覺交接（離線，未部署／未push）

- GitHub已fetch koala `codex/approach-lidar-frame`最新`a2c6f19`（六個新提交）；匯入紙盒模型、placement接近／投放、E1配對與有限前挪、mission_loop及測試／現場證據。組員紀錄兩次投放成功但未使用AMCL，不等於本分支整輪驗收。
- 新v23入口`mission`／`object-approach`；完整流程新增`v23_koala_map_handoff.py`：持物確認→定位續行檢查→地圖到可視交接點／面向紙盒→釋放底盤控制→組員視覺接近／投放→退出／定位與mask核對。範本交接點未量測，預設false，完整real循環先擋在preflight；`--no-patrol`保留組員獨立流程。
- 最新路徑／禁航圖`output/png/patrol_keepout_latest_2026-10-19.png`，直接取目前outer route與中心mask；一般中心離牆40cm、巡航約55cm、盒角排除。紙盒仍候選座標。
- 部署前須核對現車控制器的TCP高度補償；本次只精選合入夾爪停滯修正，保留本分支Serial transport／monotonic／盒高6cm。詳見`docs/operations/KOALA_NAV_INTEGRATION_2026-10-19.md`；不得整份覆寫現車組員校正。

## 最新：常駐定位與隨機起點導航

- **下一步腳本已離線完成，尚未部署／自動串任務**：integration/v23_task_resume_guard.py提供begin／wait／resume。長時間任務等待不當作定位故障；續行須travel／停穩、5個穩定新幀，夾取／投放須同operation ID成功回報及holding一致。可選已校正ArUco metric producer橋接，一次只送一個seed並等待accepted_reference_stamp及新AMCL；只讀ID不通過，producer與相機校正仍待做。相關261 tests及既有回歸通過。下次開機先依 docs/operations/TASK_WAIT_RESUME_2026-10-18.md部署配套並測idle120秒→resume，再接入組員任務。此段為本輪新增，尚未push。

- **離線新增AMCL監督強化，尚未部署Jetson**：整段恢復20秒上限、具體reason_code／next_action、導航不重複追加等待、15秒systemd heartbeat、最近15秒健康cache回復、同boot持久化3次／600秒AMCL重啟預算。正常AMCL持續更新參考，不以運行時間判過期；缺新參考仍不盲開。218相關tests及既有回歸通過。下次開機先依 docs/operations/AMCL_WATCHDOG_2026-10-18.md 配套更新script／policy／patrol／unit，再做實車故障注入與巡航。本段優先於下方較早「監督器已部署」敘述，先前已部署版本沒有本輪所有強化。

- 已依要求送出安全關機：前置確認輪零、maintenance=true、travel、owner8117，隨後SSH逾時；下次須重新檢查boot／owner及未知起點，不使用舊seed。
- 下一輪優先依 docs/planning/ARUCO_LOCALIZATION_AND_PLACEMENT_APPROACH_2026-10-18.md：保留AMCL＋ArUco絕對參考，取得組員repo外的placement權重、核對座標、量release-preview落點，建立相機可視交接點再接近。已實際讀取Claude「放置盒接近交接」頁。現入口會覆寫外部YOLO模型環境變數、會拒絕持物啟動，且無placement模式；不能只換模型就跑投放。尚未做此模式程式修改或取得模型。

- 已從另一隨機點（ID2左前／ID7右前人工監督排除對稱解，新scan種子）導航至投放準備區：34.14秒、40脈衝、0恢復、AMCL估計終點誤差4.29cm、RAM最低1437MB。流程停在WAIT_VISUAL_TARGET，未精準靠盒或投放。證據 docs/calibration/dropoff_filter_2026-10-18/。
- **LiDAR持續設定已更新**：新預設 lidar_self_returns_2026-10-18.json，6個已確認機構小矩形，皆在車體中心32cm內；遮罩外近回波、32cm外所有回波保留，不整片忽略後方。三份實scan重播32cm外刪除0，舊10/16 profile留存。此更新優先於下方「持續profile未改」的歷史紀錄。
- 一併修正外部seed讀入120秒、啟動僅15秒的不一致；首次有效AMCL前120秒、內部回復仍15秒，移動／身分／預算閘不變。定位監督器升級後恢復ready，owner8117、輪零、maintenance=true、travel、turn_clear=true，相機無持有者。濾波／定位42 tests及既有回歸通過。ArUco仍未完成metric/map登錄，不能宣稱全自動任意點定位。

- 實貼8張ArUco後完成「目前點→最近路段→逆時鐘完整一圈→原接入點」，171.25秒、235脈衝、0恢復、AMCL估計終點誤差4.38cm、RAM最低1403MB。巡航56張取樣內ID0～7全見，各至少3張。起步以ID2方向人工監督排除對稱解，未完成metric ArUco定位／自動AMCL修正。證據 docs/calibration/aruco_patrol_2026-10-18/。最後輪零、maintenance=true、travel、owner8117、AMCL ready，相機停止。

- A牆實貼ID0、ID2已由後相機分別連續四張解碼成功；證據 docs/calibration/aruco_a_wall_2026-10-18/。本次只確認可見／解碼，未註冊map或接入AMCL。受監督右轉達脈衝上限後停穩，輪零、maintenance=true、travel、owner8117，RAM最低1394 MB；相機與臨時snapshot已停止。不可視為精確90°轉向或沿用舊停靠朝向。

- 後續相機環視已涵蓋369.25°／50張影像，四面白牆可見；未偵測到ArUco，未完成實貼ID覆蓋／定位驗收。近牆裁切與下中央支架遮擋須避開，安裝高度以完整標記／白邊入鏡為準。僅本輪臨時補了一小段已確認後機構自回波遮罩，未改常駐profile；最後停穩、AMCL ready、owner8117、RAM最低1391 MB。本輪最初朝向後多約9°，不可當作原精確停靠姿態。新證據見 docs/calibration/wall_visibility_2026-10-18/README.md（未包含於較早推送a3022f4）。

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
