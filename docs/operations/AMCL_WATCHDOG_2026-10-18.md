# AMCL 有界等待與恢復監督

狀態：程式、systemd單元與離線測試已完成。機器已關機，**本版尚未部署／實車驗證**；較早同日的seed期限修正與LiDAR profile實測不代表本版看門狗已驗收。

## 行為

沿用 deploy/ros/g2_localization_service.py，一個監督器管理自己的 map_server／AMCL roslaunch 群組；沒有第二個 Serial／底盤控制者，不重啟 grasp-service。

| 情況 | 處理 |
|---|---|
| 定位正常 | 持續更新已驗證的last-good位置，不因運行超過15／120秒自行到期 |
| scan、odom、TF或AMCL短暫異常 | ready=false，導航停等；保留健康子程序，待新鮮資料恢復 |
| 輸入持續異常8秒 | 停止自己的定位子程序；資料與參考仍有效時有限重啟 |
| AMCL啟動／回報卡住 | 啟動寬限12秒、持續定位異常8秒後停止子程序，再按預算處理 |
| 同一段恢復超過20秒 | 明確標示degraded與原因；更換錯誤種類不會重置計時。導航立即回報，不再追加另一輪20秒等待 |
| 沒有可信參考、seed過期、owner／boot／map改變 | needs_reference；等待新ArUco／外部參考，不重打舊時間或猜位置 |
| 多次重啟失敗 | AMCL最多3次／600秒，間隔至少10秒；顯示retry_after_s，預算存/dev/shm，同次開機重啟監督器不會清零 |
| 監督器本身卡死 | systemd 15秒未收到heartbeat判定故障；Restart=on-failure，單元仍受3次／600秒限制，不作無限重啟 |
| RAM不足 | 開始定位要求1024MB；低於768MB停止定位子程序，等待記憶體恢復 |

20秒是健康狀態檢查的恢復期限，不是保證實體故障在20秒內修好。監督器降級後繼續觀察；有效資料與參考回來可恢復ready。已退出的導航階段需由任務層重新派發目標，不重播中斷前速度。

## 防止錯誤「續命」

- 外部獨立參考最多120秒；健康AMCL的內部回復參考最多15秒，boot／owner／raw-map hash相符且odom位移≤10cm、轉角≤10°。
- /dev/shm/g2_localization_last_good.json只存已通過感測、TF、位姿跳變檢查的位置及已消費外部參考stamp。監督器快速退出後僅可恢復仍在15秒內的cache；過期就要求新參考。
- 若監督器卡住到systemd重啟時cache已過期，仍須新ArUco參考。尚未完成的ArUco自動producer不以「有8個ID可讀」替代；本版不能保證所有硬體故障／搬車都能無人恢復。
- 新AMCL必須有本次啟動後的publication，不能拿前次快取假裝已就緒。過大的定位跳變不會更新last-good cache。
- 等待／重啟預算使用monotonic時間；Python2使用/proc/uptime，NTP校時不會延長恢復等待。

## 狀態與操作

健康檔仍為 /dev/shm/g2_localization_health.json，保留舊ready／stamp／state介面，增加：

- reason_code、next_action：可辨認SCAN_STALE、ODOM_STALE、BOARD_STALE、TF_STALE、AMCL_STALE、RAM_PRESSURE、REFERENCE_REQUIRED、RESTART_BACKOFF等。
- wait_elapsed_s、wait_limit_s、wait_timed_out、degraded、last_good_age_s。
- scan_age_s、odom_age_s、reference_age_s、reference_kind、amcl_pid、retry_after_s。

systemd READY只代表監督程式已啟動，**不代表機器人已定位**；導航以健康JSON的ready與感測資料為準。

## 下次開機部署順序

1. 同步本次service、policy、g2_map_patrol.py及systemd單元，保留Jetson組員手臂改動。這四個檔案要配套更新；不能只換Type=notify單元卻仍使用不送heartbeat的舊程式。
2. 在Jetson專案根目錄先做Python2編譯，再安裝本單元：

```bash
/usr/bin/python -m py_compile deploy/ros/g2_localization_service.py integration/g2_localization_policy.py deploy/ros/g2_map_patrol.py
sudo install -m 644 deploy/systemd/x3plus-g2-localization.service /etc/systemd/system/x3plus-g2-localization.service
sudo systemctl daemon-reload
sudo systemctl enable --now x3plus-g2-localization.service
python3 -m json.tool /dev/shm/g2_localization_health.json
```

3. 重查唯一Serial owner、輪零、手臂travel、odom／scan。未知起點使用新標記／scan參考；無參考時needs_reference是預期狀態，不能取消此閘。
4. 驗證正常運行超過120秒、短暫AMCL延遲恢復、僅AMCL子程序退出後重啟，以及監督器SIGSTOP觸發systemd看門狗。記錄owner未變、輪零、復原時間與新publication；測試前後保留restart-budget，不用重啟服務繞過次數限制。
5. 最後重跑最近巡航路段／完整巡航、到投放準備區，之後才做紙盒視覺交接。

查看診斷：`journalctl -u x3plus-g2-localization.service -n 50 --no-pager`。沒有故障時不必手動重啟定位。

## 測試

tests/test_g2_amcl_watchdog_runtime.py直接執行實際service主迴圈，使用虛擬ROS／時間／子程序，不連機器人：涵蓋健康180秒、短／長scan中斷、AMCL卡住、預算耗盡、seed缺失後新參考恢復、60秒外部參考、過期cache拒絕、快速監督器退出後cache恢復、owner改變、RAM壓力、舊AMCL快取與位姿跳變。

相關導航／濾波／交接218 tests通過；既有controller138、servo37、FloorGuard641、UI50（skip1）、FSM通過。記錄在 docs/calibration/amcl_watchdog_2026-10-18/。systemd實際故障注入與Jetson Python2執行留待下次開機，不冒充離線測試已完成實車驗收。
