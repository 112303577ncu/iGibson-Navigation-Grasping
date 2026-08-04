# 參考副本 — 不要執行

`x3plus_real_grasp.release_branch.py` 是 `_scripted_release()` / `run_release_only()`
的來源，2026-08-04 從它移植進 `../x3plus_real_grasp.py`。留在這裡只為了讓人日後
能核對移植是否忠實。

**不可拿它來夾取。** 它是 2026-07-31 之前分支出去的，缺少讓 v21 真正夾得起來的東西：

| 缺什麼 | 後果 |
|---|---|
| `_park_jaw_hold` / `jaw_contact` | 夾爪會一路推到 180° 全閉停點，長時間近堵轉扭矩（實機聽得到齒輪研磨聲，2026-07-31） |
| `_check_target_envelope` | 相機看得到但策略沒訓練過的目標不會被擋下 |
| `obj_provider` | pipeline 無法注入物體座標 |
| `move_home` | 沒有受保護的歸位路徑 |

移植時對 v21 做的唯一實質修改：帶著物體時夾爪目標是編碼器到不了的擠壓值，所以伸出的
每一步都要 `grip_is_hold=True`，且開爪前要清掉 `_grip_hold_rad` —— 否則開爪會被判成堵轉。

同目錄下 `deploy_contract.py`、`action_execution_v21.py` 與 v21 相同，不需要另存。
