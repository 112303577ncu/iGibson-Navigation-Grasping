# Codex 下一步

目前主線為 `v23-grasp-test`／v23 常駐服務，決賽入口是 `grasp/v23/demo.py`。
先讀 [最新交接](docs/handoff/V23_FINAL_DEMO_2026-10-07.md)，依其中部署狀態與待驗項目接續。
操作入口與限制見 [FINAL_DEMO.md](grasp/v23/FINAL_DEMO.md)。

本次新版尚未部署至 Jetson：SSH `172.20.10.2` 逾時。不得把本機測試通過寫成實機完成。
下一步為唯讀連線確認、比對遠端檔案／組員校正後逐檔部署，再做不動底盤的整合驗證。
今天沒有場地；不宣稱完整自主巡航、AMCL 或側向避障已驗收，不合併 owner 分支。
