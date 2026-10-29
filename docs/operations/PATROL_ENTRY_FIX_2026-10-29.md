# 巡航起步修正（2026-10-29）

## 行為

原本額外 6 cm 的轉向遮罩圈與盒子已膨脹的遮罩相疊，起步又只朝固定 `points[0]` 前進。
現在先判斷可否直接轉向；若尚不能轉，沿實際路線入口的第一段安全方向微動，必要時先短退。
第二個失敗位姿在新的保守盒子包絡下仍不足以直接轉，但可先退數毫米再轉。
轉向完成後由正常 `goto` 規劃器朝路線入口行駛。

- `BODY_RECTANGLES` 與 `back_path_problem` 原樣抽自 koala e8766f0 的 `placement_dock.py`，新模組不載入模型或 ROS。
- runtime 建構器讓投放退出與原生巡航共用該檢查函式，納入來源雜湊；倒退另外檢查完整平移矩形掃掠，拒絕已相交的盒子／牆。
- 近盒轉向使用物理盒子保守包絡 `navigation_mask_validation.envelope_room_uv`（已含盒位誤差），不再疊遮罩 6 cm；不直接信任舊的未驗證 `footprint_map`。
- 全方向車身掃掠半徑 0.308099 m，再留 3 cm；AMCL 的平移 3σ 只能到 2 cm，脈衝期間及停穩後平移不得超過 1 cm。
- 直線起步微動的方向誤差另限 3σ ≤1°。倒退只允許起步作用域、速度 ≤0.08 m/s、單脈衝 ≤0.08 秒；累計實測里程加每次預測停距不可超過 10 cm。
- 保留遠盒的遮罩轉向圈、牆邊 6 cm 餘量、中心遮罩、雷射煞停與定位／感測／owner／travel 閘。
- `--inset` 不會改 `preferred_patrol.route_file`；`patrol-cycle` 使用場景路線。

## 離線驗證

`tests/fixtures/patrol_entry_20261029/` 四份地圖／路線檔取自 koala `265a2bc`，保留原始批准雜湊。
三個起點：(1.04, 0.37, 87°)、(1.03, 0.45, 114°)、(1.06, 0.40, 75°)。
第一個原用舊 6 cm 盒位誤差圖，在本測試的 2 cm 圖上舊轉向圈已可通過；第二、三個仍可重現舊轉向圈拒絕。
回歸包含中間起點、真實車身／線材幾何、路線入口、短退、總倒退預算、盒子與牆碰撞、定位誤差、感測中斷與轉向漂移。

```powershell
$env:PYTHONPATH=(Get-Location).Path
python -m pytest tests/test_g2_patrol_entry.py tests/test_g2_patrol_policy.py tests/test_g2_navigation_goals.py tests/test_g2_boundary_recovery.py tests/test_g2_prepare_koala.py tests/test_g2_task_recovery.py tests/test_g2_placement_corridor.py -q
```

## 後續配套

沒有修改正式地圖、SSH、部署或實車移動。
部署時需同時攜帶 `deploy/ros/g2_map_patrol.py`、`integration/g2_patrol_entry.py`、`integration/g2_body_geometry.py` 及原有配套 helper；
用新 builder 建立 runtime 並核對 manifest，再依實車當下的新盒位地圖驗證。不要以本機舊盒位地圖覆蓋現車。
改好並完成實車驗收前，仍從場地中間起步。
