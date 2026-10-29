# 縮減巡航禁航區

正式目錄 `maps/demo_box_20261018_patrol` 沿用既有部署路徑，但盒位、盒子誤差包絡與基礎路線已同步到 koala `265a2bc`。
實體盒子位置採中心離兩側牆各45cm、盒位餘量2cm版本；不使用本機舊盒位。

## 修改

- 含線材完整車體旋轉半徑0.308099m；物理轉向距離加3cm，包含2cm定位誤差（3σ）及1cm轉向平移餘量。每次命令與停穩後檢查。
- 正式中心牆距35→34cm。舊6cm轉向圈不再重複加在牆距與盒子膨脹上；邊界恢復與投放後回巡航也使用相同規則。
- raw occupancy map只膨脹一次，occupied／unknown不當成自由空間；整格半對角仍正確計入。盒子中心禁航緩衝仍為0.283238m，接近盒子時另檢查完整車體。
- 原路線六個節點保留，僅將盒旁三點修到保留完整轉向淨空的位置；整段路線也檢查掃掠圓。放置準備點改為新路線上合法節點，沒有宣稱投放／伸臂已實車驗證。
- patrol-cycle、point或明確goals使用自己的檢查路徑，舊inset方形不再先行擋下有效路線。
- 保留雷射煞停、停止距離、定位與感測時效、唯一owner、travel與倒退預算。未降低雷射門檻或假造新的速度／煞停量測。

## 同一份新盒位地圖的離線比較

|區域|舊設定|新設定|增加|
|---|---:|---:|---:|
|中心可行格|3051|3211|160（5.24%）|
|中心可行面積|1.2204m²|1.2844m²|0.0640m²|
|可原地轉向格|2371|3133|762（32.14%）|
|可原地轉向面積|0.9484m²|1.2532m²|0.3048m²|

格網2cm。舊轉向格採原6cm圈，新轉向格採完整車身與物理盒子／牆淨空；實際移動仍受定位品質、動態雷射與停止路徑限制。
不能把可行面積當作實車任意起點成功率。

三次失敗起點重播通過；(1.06,0.40)的八個45°間隔朝向皆可轉向，第二個更近盒子的起點先短退後可轉。
另檢查擴大區域的車體矩形／多朝向淨空、盒牆碰撞、雜湊與格網／盒位／餘量篡改拒絕、重建一致性、巡航／恢復／放置退出一致性。
歷史35cm策略測試使用 `tests/fixtures/patrol_scene_20261018`，仍驗證舊批准流程。

```powershell
$env:PYTHONPATH=(Get-Location).Path
python -m pytest tests/test_g2_compact_patrol.py tests/test_g2_patrol_entry.py tests/test_g2_patrol_policy.py tests/test_g2_navigation_goals.py tests/test_g2_boundary_recovery.py tests/test_g2_prepare_koala.py tests/test_g2_task_recovery.py tests/test_g2_placement_corridor.py tests/test_g2_patrol_reentry.py -q
```

重建：

```powershell
python integration/g2_build_compact_patrol.py --scene tests/fixtures/patrol_entry_20261029 --raw-scene maps/demo_box_20261018_patrol --output maps/demo_box_20261018_patrol
```

新profile保存mask、raw map及盒子／牆幾何雜湊，並核對格網、實體車身尺寸及餘量。修改值或只換一張PGM會拒絕啟動。
需用更新後 `g2_prepare_koala.py` 重建 runtime；部署時巡航、policy、boundary recovery、body geometry、patrol entry及新場景須成套一致。
本輪只修改工作區與離線驗證，沒有連線實車、部署或移動。協方差品質閘不等於尺量定位誤差保證，3cm預算仍待實車驗收。

驗證結果：250項巡航／恢復／runtime／工作區回歸通過；指定控制器138、伺服回讀37、FloorGuard641、UI（1項略過）及mission FSM自測通過。Python語法與git diff格式檢查通過。
