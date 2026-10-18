# 外移巡航與人工校正停靠

## 實車結果

- 外移路線牆距約55 cm，比原70 cm向外15 cm；一般中心牆距40 cm保留，紙盒角落以三點切角。路線在 `maps/demo_box_20261018_patrol/outer_patrol_goals.json`。
- 初次外移跑完前五點，後段因AMCL不確定、停車回授等待及前方煞車累計耗盡恢復預算而停止。掃描證實煞車來源是49–53 cm連續牆面，不是應刪除的自回波。
- 新增 `--boundary-micro`：前距不足正常50 cm時，僅選最長0.08秒脈衝並保留35 cm前方煞車，較長脈衝仍用50 cm；停止投影、mask、資料新鮮度與RAM閘保留。兩段接續成功，0次恢復，最後停穩誤差4.16 cm。**尚未用新模式不中斷重跑完整一圈**。
- `--keep-heading`保留末段方向，避免不必要的朝場內停車轉向；最終6.5 cm位置重查已實車通過。不代表底盤原地轉向平移已被機械修好。
- 返回量測準備位置成功，停穩誤差約4.10 cm。新 `g2_relative_dock_probe.py`以同boot/owner的35–40 cm原盒距及位置參考作監督式直行；保留原圖車體空間、40 cm牆距、35 cm前煞、0.08秒脈衝、側移／角度／總距離上限，沒有永久放寬紙盒mask。
- 第一次約26 cm時因AMCL不確定停下；新增停止等待與累積目標續接，核對owner、停止odom及原始量測後只補剩餘距離。總計前進29.52 cm，未重走另一段30 cm。
- 依現場追加要求，再前進4.67 cm、右轉29.93°。使用者回覆「已抵達」。**這是人工引導的實體停靠確認，不是自動重複到點或實物投放驗收**。
- 最低可用RAM約1.59 GB。最後owner7157、輪零、maintenance=true、travel、holding=false，沒有執行手臂投放。

## 回波與相機

- 沒有改常駐self-return profile，也沒有刪除整個後半圈。三方向快照交集只有一個額外穩定角度bin，證據不足以全域擴大；候選另存留證。
- 轉向工具新增明確opt-in `--confirmed-right-structure`，只限兩個已觀察且經操作者確認的右側矩形：x[-5,2]/y[-29,-21] cm，以及x[4,7]/y[-29,-24] cm。其他近回波仍檢查；不是全域 `/scan` 濾波。既有後側確認旗標另保留，未用全域禁用掃掠閘的旗標。
- RGB可拍到紙盒，初始照片有機構遮擋，已存接近前與最後影像。相機目前僅取像檢查，**沒有影像閉迴路導航**。現有模型類別為sugarbox，不是紙盒，且後相機座標基準有既有疑點，不能直接用作最後距離。

## 定位與接續

- 暫時AMCL有2400秒診斷期限；最後保存停靠紀錄時已無新鮮定位資料。最後odom約[1.0338,0.8202,0.2652]，不能當map座標。新scene舊種子已標失效。
- 下次導航前先重新建立可信定位，確認目前紙盒旁位置與可退出路徑。正式demo需將AMCL改為受監督常駐，保留記憶體與資料健康監控，不沿用診斷期限。
- 正式設計：AMCL粗導航→交接底盤控制權→ArUco/相機相對對準及距離控制→手臂投放→收臂→退出及續巡航。共用resident TCP7000，不能直接啟動會自行持有Serial的舊整合入口。
- 172項相關pytest通過、1跳過，controller138／servo37／floor641／UI50（skip1）／FSM通過。Windows仍只做測試程序URDF相對路徑與UTF-8轉接。
- 證據在 `docs/calibration/nav_outer_2026-10-18/`，含中斷與成功報告、相機、參考量測、測試及summary。未push。

## 外移巡航指令

只在新鮮定位及可用起點下執行，不能從尚未驗證退出的紙盒近處直接照抄：

```bash
/usr/bin/python deploy/ros/g2_map_patrol.py \
  --scene maps/demo_box_20261018_patrol --inset .70 \
  --goals maps/demo_box_20261018_patrol/outer_patrol_goals.json \
  --boundary-micro --keep-heading --real --seconds 480 \
  --output /tmp/outer_patrol_result.json
```

自訂goals才是55 cm外移路線；此處inset .70只維持舊別名點設定，不決定自訂路線。相對進位的量測reference及resume報告限本次boot/owner，禁止下次開機重播。
