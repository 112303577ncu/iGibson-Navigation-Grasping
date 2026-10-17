# 6 cm 紙盒空投放預演與部分右轉

## 空夾爪預演

新增常駐命令 `release-preview`，沿既有手臂仲裁與防撞／地板檢查執行：巡航位 → E1 → 空爪閉合確認 → 原有六段伸臂 → 張爪 → 回位 → 巡航位。正式 `release` 仍要求已確認握物，不偽造holding；預演明確回傳 `object_delivery_verified=false`。

`DeployConfig.bin_rim_height` 同步實測6 cm，原有離盒底部餘量3 cm。成功預演約15.57秒，最低可用RAM1654.15 MB；152個監控樣本輪命令與odom均零，板回傳最老0.02秒。六段伸臂均到位，FK估計夾爪底部11.4 cm，完成張爪及stow。

FK記錄前伸x約23.9 cm，不應沿用舊停靠估計41 cm推論一定可落盒；兩者座標基準與實際可達範圍仍須核對。本次沒有實物投入紙盒。

## 已處理的預演失敗

- 第一次在手臂動作前因maintenance狀態被拒絕；修正為由既有 `run_arm_command` 取得arm claim，確認底盤零輪，避免在callback內要求與仲裁互斥的maintenance。
- 第二次空爪只閉合到約94°，8次迭代不足150°全程；未進入放開步驟。明確確認狀態後home→stow復原，再改用既有close配置40次上限，最後成功。
- 所有失敗JSON保留。常駐服務最後PID4505，odom已重設；地圖種子標失效，不能沿用重啟前的map→odom。

部署只插入必要變更，保留Jetson原有相容性修正。三個檔案的部署hash／備份路徑見 `docs/calibration/release_preview_2026-10-17/deployment.json`。不能將本機檔整份覆蓋到Jetson當成完全同版。vision／monitor恢復後maintenance曾讀到false，已明確上鎖；其來源未確定，接手需重查。

## 右轉30°未完成

原始起點累計右轉 **12.1386°** 後，右前方約25–28 cm持續回波觸發掃掠閘；平移約0.77 cm。最終零輪、maintenance=true、travel，odom約[-0.0037,-0.0067,-0.2119]。

使用者先確認後方固定機構，故 `--confirmed-rear-structure` 僅針對兩個車體座標矩形排除：後右x[-18,-6]/y[-22,-13]cm、後左x[-20,-15]/y[10,14]cm。只在明確帶此旗標的監督式轉向生效，不是全域 `/scan` 濾波，也不忽略整個後半圈。其餘感測新鮮度、RAM、角度、平移、群聚與停止閘仍保留。

使用者其後確認右前回波亦為固定機構／線材、現場無外部障礙；本輪文件／推送未再移動或加大排除。後續需採新鮮多幀對照建立相應局部濾波。若接續轉向，先核對owner／odom沒有變化，使用原始reference與部分報告的最後停穩pose續轉剩餘約17.86°；不能再執行整個30°。資料或參考改變就重新定位，不盲接。

第一次極短脈衝約0.54°後，立即上鎖曾被拒絕；工具cleanup現先等停止回授再上鎖，後續退出上鎖成功。

## 證據與測試

`docs/calibration/release_preview_2026-10-17/` 包含成功summary／result、失敗報告、journal、測試與部署hash，以及right30_partial／reference。
預演相關56項、v23 controller163／floor641、v21 controller138／servo37／floor641、UI50（skip1）、FSM已通過；後續轉向／導航相關45項通過。測試數對應各次版本，最新提交前測試另記於該目錄。
