# 任務等待與續行檢查：離線測試

- 新task guard：39 passed。
- 相關導航／監督／濾波／交接集合：261 passed，見navigation_pytest.txt。
- controller138、servo37、FloorGuard641、UI50（skip1）、FSM通過，輸出分別保留。
- 不連機器人，未部署、未實車驗收；ArUco輸入在測試中為合成metric觀測，不能當成相機校正或實體定位精度證明。
- 說明與下次開機順序：docs/operations/TASK_WAIT_RESUME_2026-10-18.md。
