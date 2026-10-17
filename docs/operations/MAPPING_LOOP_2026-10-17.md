# 2×2 m 逆時鐘繞行建圖結果

## 實車結果

- 使用者修正手臂後，status讀到 [90,81,104,80,91,30]。常駐home→stow成功，重啟owner後PID27713、startup travel_verified、TCP7000就緒。只用常駐owner控制手臂與底盤。
- 完成四邊直線與四次左轉。odom角度90.713°、90.965°、89.636°、89.930°；第二次轉彎因資料過期中斷後，核對停止位置/owner uptime，沿用原始角度基準接續，沒有重轉90°。
- 最終odom (-0.12709,0.03358,0.05221rad)，相對啟動原點距離13.15cm、方向差2.99°。不是尺量實際回圈精度。
- 直線沿用motor30/設定25cm的既有控制，停穩實際odom約29–38cm；停止/資料延遲仍有誤差。前方煞車門檻50cm（中心距），停車轉向前約38–47cm。不能宣稱短距離控制已修好。

## 地圖與候選路線

`maps/demo_2x2_20261017/`：demo_2x2.pgm/yaml、patrol_candidate.json、map_patrol_preview.png。

- 2cm/格，704×704含大量未知區，是GMapping格網擴張而非14m場地量測。圖中主要四條牆線可見，角落缺口/向外射線保留在原圖，沒有填牆或把未知區改成空地。
- 第一個建圖啟動被上一輪失效/g2_demo_gmapping登記擋住。確認無程序且節點拒絕連線後，只解除該節點的pub/sub/service登記。新會話active、map可讀後才繼續。因此第一邊/第一次左轉未收入此SLAM會話，後續不同朝向掃描形成四邊牆線；不是全程完整錄製。
- 最後map←odom平移約(-0.00471,0.01077)m、旋轉0.001521rad。將實際轉彎/停止位置轉成map座標，建立逆時鐘候選點。
- 路線404個中心取樣均為已知空地，最近障礙格中心距38cm。只是靜態地圖檢查，尚未驗證完整footprint、煞車包絡、AMCL或自主巡航。不能標示巡航ready。
- 缺口後續以獨立導航禁行邊界/場內路線限制處理，保留原始SLAM圖；目前尚未部署虛擬邊界。

## 負載與工具界線

- 新integration/g2_mapping_route.py只發TCP給常駐owner：分段直線、停穩短脈衝左轉、3秒資料等待、95°轉角/5cm平移硬上限。瞬間過期先停車/斷開client，再有限次等恢復；失敗則停止。未放寬既有感測過期門檻。
- 實機g2_nav_client較舊、不支援recover_sensor_s。新工具以signature判斷功能，未覆蓋隊友原client；第一次參數錯誤時未發速度。
- ROSbridge在建圖負載下出現stale_stamp，曾中止第二邊/第二次左轉。新增deploy/ros/g2_native_nav_snapshot.py原生訂閱scan/odom，原子更新單一/dev/shm快照，600秒/256MB虛擬記憶體上限、低於768MB退出。讀檔不會刷新感測資料年齡，沒有Serial/速度寫入。
- 原生快照仍曾出現過期、無進展、停穩逾時，失敗報告全保留。整批JSON輸出、tcp_nodelay、停止時要求資料餘裕，並暫停不需使用的vision及其monitor後完成後續行程。延遲根因未完全定案，未驗收長時間巡航。
- 轉彎以明確confirmed-self-returns覆寫近車掃掠回波閘、保留觀測與其他限制；是現場確認下的監督式操作，不是通用側向避障。前方牆面煞車不受此旗標影響，未從牆面重新學習profile。
- TF及本輪工具用雷射前13cm；一般client仍有10cm常數，正式導航前待統一。
- RAM快照約1675–1923MB，不是連續最低值紀錄。完成後停止建圖/快照transient程序，恢復vision/monitor；owner未再次重啟。最後四輪零、maintenance=true、巡航位、空夾爪，機器保持開機。

## 測試與後續

相關32通過；controller138、servo37、floor641、UI50（略過1）、mission selftest通過。controller/floor用僅測試程序的相對URDF路徑轉接，Windows中文絕對路徑問題未修。最初runpy跑servo缺import路徑，後以原始直接入口重跑37通過。

下一步：新圖AMCL與重定位 → 統一正式client雷射13cm → 導航禁行邊界/完整包絡檢查 → 候選路線實際巡航。不得與GMapping同時發布map→odom，也不得把監督式繞行當成自主巡航驗收。

原始JSON、失敗/成功分段與回歸紀錄在docs/calibration/mapping_2026-10-17/。本輪尚未push。
