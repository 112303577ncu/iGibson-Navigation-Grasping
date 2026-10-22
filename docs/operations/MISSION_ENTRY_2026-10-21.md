# 八 ID 定位接入新版任務入口

2026-10-22投放走廊版更新：先建立全新 `.runtime/koala-e8766f0-corridor`（schema2），
再使用 `demo.py mission`或本入口；同時更新Jetson的face-yaw支援。
下方較早的runtime目錄名需替換為此新目錄；見 [投放走廊](PLACEMENT_CORRIDOR_2026-10-22.md)。

入口：`integration/v23_mission_entry.py`，在 PC 執行。
以獨立、完整的 koala 工作目錄執行其 `mission_loop.py`；不覆寫本工作樹既有接近／手臂整合草稿。
2026-10-22離線版固定 koala `e8766f03d0de3e48419a488f330b62e3412afab3`，以
`g2_prepare_koala.py`建立完整runtime，套用取樣間隔運動誤差與headless清理修正。
每次入口結果記錄實際使用的五個 koala 模組 SHA256，便於比對測試版本。

## 啟動順序

1. 唯讀確認空爪、travel、輪零、沒有另一個底盤 client，取得維護鎖。
2. 核對 PC／Jetson 正式八 ID 表；不同時只備份並更新該 JSON。
3. 既有定位僅在 boot／owner／map／v3 registry／accepted_reference_stamp 都一致，且五筆不同
   odom 時間戳的健康新樣本通過時續用。不以重寫 seed 使正常 AMCL 續命。
4. 沒有可靠既有定位時，使用新相機三幀及 native scan capture，在 PC 比對、Jetson 選候選並
   寫一次 seed。只接受 `ok=true AND localized=true AND status=LOCALIZED`。
5. 每份 SSH 下載與上傳核對 SHA256；原始 capture 保留原位元組。
6. 先建立 PC 解碼器，再啟動本次後相機 sender；八個新影格通過後才進 koala 總控。
7. koala 原流程處理巡航接入、候選先停／停穩三幀、接近／E1 夾取、視覺投放、退出與續巡。

預設只做新版總控的唯讀預檢；`--real --i-am-beside-the-robot` 才會啟動實車階段。
`--relocalize` 強制新相機／LiDAR 初始化；沒有此需求時保留健康既有定位。
任一 gate 失敗不啟動下一階段；結束時發 stop，travel 時恢復維護鎖，停止本次自建 sender。
不重播未知結果的手臂命令、不修改地圖投放點未驗證旗標、不開第二個 Serial owner。

## 執行

```powershell
python integration/g2_prepare_koala.py --output .runtime/koala-e8766f0
python integration/v23_mission_entry.py --host 172.20.10.3 `
  --koala-root .runtime/koala-e8766f0 `
  --gstreamer C:/tmp/x3_gstreamer/gstreamer_cli/bin/gst-launch-1.0.exe
```

實車參數另加 `--real --i-am-beside-the-robot`。一般部署使用既有 SSH 認證；本機測試的
Paramiko 適配器留在 C:/tmp，不把登入憑證或 helper 放入 Git。
`--output` 必須是新的證據目錄，避免覆寫先前實驗；預設依台北時間建立新目錄。

## 已測界線

UDP 探測確認 Jetson→PC 可達，實際後相機 H.264 可解碼 138 幀；未修改 Windows 防火牆。
定位／相機／新入口／巡航安全共 165 項 pytest 通過；必要 controller／servo／FloorGuard／UI／FSM 回歸通過。
第一次入口實車測試在 AMCL 重啟預算用盡時回報 AMCL_TIMEOUT，沒有進巡航；保留原預算限制。
該次也發現測試用 SSH 適配器可能在 exit status 到達時過早關閉，造成 JPEG 不完整，已改為
讀到 stdout／stderr EOF，再配合 SHA256 檢查。沒有以這個結果宣稱夾取／投放成功。

第二次入口新初始化、影像 gate 及模型暖機都通過，實車 22 脈衝，到達最近路段接入點
`join_outer_ccw_0`（AMCL 誤差 4.34cm），往下一點時以 stop feedback timeout 停車。
該次記錄兩次 vision source stale；停車證明原本仍等候行進用影像許可，可能耗盡三秒停穩期限。
已將零輸出下的停車取樣與影像許可分開：保持感測／定位／owner／實際速度門檻，三個新 odom
樣本與至少 0.45s 停穩條件不變；下一次非零 pulse 仍需有效影像許可。已新增過期／候選影像
可確認零速但不能再開車、非零回饋與過期感測不能假裝停穩的回歸，備份並精選部署導航檔。
第三次入口以五筆新健康資料續用既有 AMCL，沒有重寫 seed；測試結果另記。

各次結果以 `docs/calibration/live_2026-10-21/entry_run*/entry_report.json` 與同目錄原始證據為準。
# 2026-10-22：放置後退出更新（離線）

入口新增自動載入 `g2_patrol_reentry.py`，需koala `loop-opt-2026-10-21` 相容checkout。
放置後以實際停點及轉向圈確認回巡航，不再以固定後退完成作為下一圈放行條件；
未部署，細節見 [放置後退出規則](POST_PLACE_PATROL_REENTRY_2026-10-22.md)。
