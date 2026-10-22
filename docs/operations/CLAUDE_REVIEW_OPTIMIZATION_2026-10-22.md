# Claude 審查優化

來源：[koala PR #5 留言](https://github.com/koala915/iGibson-Navigation-Grasping/pull/5#issuecomment-6340960862)
與 [b3da080](https://github.com/koala915/iGibson-Navigation-Grasping/commit/b3da080177b88e164eca5604653f24c8f79bb916)。
本輪只修改、測試與推送程式，不連線車輛、不部署。

## 五項修正

1. 正式場景改為35cm中心距牆mask。批准紀錄綁定PGM SHA256、牆距、固定盒buffer與6cm轉向reserve；
   只改JSON數字或替換錯誤地圖會拒絕啟動。7cm轉向圈與定位品質要求保留。
2. 近盒不再因角速度超過0.02rad/s直接停車。0.6秒、12步預測保留每一位姿的直退出口，
   額外比較旋轉前後車體矩形到固定盒的距離，不能更深入保守包絡。保留原生角速度，
   可減前進速度；無安全命令仍停車，不能保證任何近牆方向皆可轉。
3. 未驗證投放箱座標僅作候選。放下前先通過原生arrival_problem，並要求本輪placement、
   非seeded、未blocked、報告5秒內、視覺記憶12秒內。12秒包住原生最大盲走約10.75秒，屬待實測預算。
   以停止位姿將報告的base盒緣加上朝向投影半深度轉到map；與候選差不得超過25cm。
   保留原保守障礙包絡，再增加觀測半對角線+4cm的包絡；確認實際停點仍可直退。
   taught-wall修正沿途與真正release RPC之前再次檢查退出。盒位登錄仍false，不覆寫正式地圖。
   已驗證盒位仍要求距原預定停點10cm內；未驗證者距本輪觀測推算停點10cm內。
4. AMCL與odom的15cm／10度連續性比較只在定位中斷後進行，不用夾取前的舊位置限制整段正常行駛。
   跨中斷須同owner／boot／map、五個新穩定樣本；保留30秒合併等待預算與未知手臂結果不重播。
5. 巡航雷射煞停恢復，牆與固定盒幾何為附加檢查。一般邊界恢復只向前；後方目標先旋轉，
   需先通過35cm物理圓包絡。投放／enoch原有直退流程保留，不能把一般旋轉脫困套用到無轉向空間的投放點。

## 準備與界線

```powershell
python integration/g2_prepare_koala.py --output .runtime/koala-e8766f0-review
```

入口預設使用新目錄；原目錄不覆寫，舊helper與新PC不一致時拒絕啟動。
上游仍固定e8766f0，未直接換用整個最新主線，也未冒稱取得所有後續等待修正。
35cm只代表中心導航政策，不是持物時車體全包絡的實測證明。
夾取前完整「夾取→退出→轉運」預約尚未實作，本輪解決Claude列出的五項審查問題。
視覺投影誤差、馬達有效速度、持物／伸臂包絡、完整循環仍需實車驗收。

## 離線驗證結果

- 任務／定位／邊界／放置矩陣：359 passed。
- 固定版上游七組回歸：315 passed、263 subtests passed。
- v21 controller 138、servo 37、FloorGuard 641，全通過；FSM selftest通過。
- UI：50 tests、skipped 1、OK。
- 實際pinned MotorClient AST執行：安全前進降至0.1125m/s、定位失效送stop、零速始終可送。
- 原生幾何：最佳停點負0.30rad/s保留退路；正0.30rad/s掃向盒體，拒絕。
  候選盒位偏差15cm的有效視覺到達可通過重驗，保留新舊兩個障礙包絡。
- 35cm地圖仍有10個合格取樣方向；最佳map yaw20度，名義直退19cm，誤差樣本最遠25cm。
- runtime來源／helper／規劃SHA256一致；Jetson檔案ASCII／相容語法檢查通過，沒有在Jetson執行。

地圖與規劃雜湊見 `docs/planning/placement_corridor_2026-10-22/plan.json`。
