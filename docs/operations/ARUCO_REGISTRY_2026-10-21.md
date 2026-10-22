# 八 ID 正式定位表 v3（2026-10-21）

正式表：`maps/demo_box_20261018_patrol/aruco_corner_landmarks.json`。
`registration_id=coarse_wall_layout_20261021_v3`，包含 ID0～7。
地圖 hash、相機方向／視野界及原有五張地標座標不變；新增 ID3、4、6 採既有配置座標。
所有位置保留 20cm 誤差界，並非公尺級 ArUco 位姿 producer 或精密標記 survey。

## 登錄依據

重新解碼 `docs/calibration/aruco_patrol_2026-10-18/` 的 98 張影像，利用保存的定位錨點及各影格
odom 核對配置地標的粗略方向。八個 ID 各至少三張可解碼，所有解碼觀測均符合原有方向誤差界：

| 牆面 | ID | 解碼／方向相符幀數 |
|---|---|---|
| A | 0、2 | 7／7、19／19 |
| B | 3、4 | 12／12、42／42 |
| C | 5、1 | 8／8、3／3 |
| D | 6、7 | 6／6、3／3 |

詳細觀測：`docs/calibration/live_2026-10-21/registry_v3_historical_review.json`。
本次新相機另讀到 ID3 左／ID0 右；以 v3 重放時唯一保留候選 0，其餘兩個對稱候選被排除。
重放只驗證候選選擇，不代表已寫 live seed 或收到 AMCL ack。

## 部署與生效

只更新 Jetson 的正式地標 JSON，原表備份為同路徑加 `.before_v3_20261021`。
PC／Jetson v3 SHA256：`3934d5357209a18fa48d8b2280e4e66be3d0b492d49f192a82f9d0ea25900f00`。
不需重啟控制器；取樣與選候選的 producer 必須帶新的 registration_id。
舊 v1／v2 指令只作歷史紀錄，後續不可直接照抄。

實車新取樣、完整候選及結果保存於 `docs/calibration/live_2026-10-21/localization_v3_all_ids/`。
只有結果 `ok=true AND localized=true AND status=LOCALIZED` 才能當作本次初始化成功。

本次新取樣已成功：ID0＋ID3，capture `fae69df24e8a4f9899d07b8c317bb3c4`，
來源影像最大延遲 29.6ms。唯一候選 0 提交後回報 `LOCALIZED`，五個新健康幀確認；
`accepted_reference_stamp=1792559914.935698`，與本次 seed 相同。
新取樣／寫 seed 階段沒有底盤或手臂輸出；八張中本次直接使用的是 ID0、3，其他 ID 的
登錄依據為上表保存影像，不宣稱本次已在所有朝向／任意起點重驗。

## 驗證

角落定位／相機取樣／snapshot 測試 70 項通過，包含本次 ID0＋ID3 真實資料重放回歸。
八張未啟用的 example 維持未驗證，不拿範本替代正式表。

必要回歸：controller 138、servo 37、FloorGuard 641 通過；UI 50 案例通過（skip 1）、FSM selftest 通過。
Windows 中文路徑使 PyBullet 原生 URDF 載入失敗；兩項 FK suite 改用測試程序內的 ASCII 資產副本後通過，
沒有修改生產控制碼。原失敗及重試 log 一併保留在 `registry_v3_regressions/`。

## 巡航入口實測界線

koala `fa61b64` 的完整模式唯讀預檢通過。實際啟動後，PC 收不到後相機影像，停在第一圈開始前；
底盤 0 脈衝，沒有夾取／投放。Jetson sender 存在，但 PC UDP5600 四秒探測收到 0 封包；
尚未確認是網路或 Windows 入站規則所致，不把此問題當作地標登錄失敗。
已停止本次 sender、釋放後相機；最後輪零、maintenance=true、travel、空爪，AMCL 仍 ready。
總結與最終即時狀態：`registry_v3_summary.json`、`final_registry_v3_health.json`。
