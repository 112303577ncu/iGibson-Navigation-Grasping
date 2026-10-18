# AMCL 監督器離線驗證

- 導航／濾波／交接：218 passed，見navigation_pytest.log。
- 既有回歸：controller138、servo37、FloorGuard641、UI50（skip1）、FSM通過；各log保留實際輸出。
- 實際service主迴圈以虛擬ROS／時間／子程序注入故障；測試不開Serial、不發速度，不需要Jetson。
- 新版監督器、健康cache、持久重啟預算及Type=notify／WatchdogSec=15尚未部署到已關機的Jetson。實車恢復時間、systemd看門狗及下次開機驗收均待做。
- 實作與部署步驟：docs/operations/AMCL_WATCHDOG_2026-10-18.md。
