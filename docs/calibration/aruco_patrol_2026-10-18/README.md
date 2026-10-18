# 實貼 ArUco 後完整巡航驗證

2026-10-18：從目前場內位置接入最近路段，依 join_outer_ccw_3 → 4 → 5 → 0 → 1 → 2 → 3 → 原接入點完成逆時鐘一圈，171.25 秒、235 次有界脈衝、0 次異常恢復。終點 AMCL 估計誤差 4.38 cm；未以外部量測驗證精度。導航最低 MemAvailable 1403 MB。

巡航期間 56 張相機取樣內，ID 0～7 全部成功解碼，各至少 3 張。完整錄影取樣 98 張（包含導航前後靜止時間），讀取失敗 0；不是所有位置／朝向皆能看到所有標記。

起步以 ID2 在畫面右前方，人工監督排除另外三組對稱 scan 解；獨立 scan 解與當前 AMCL 一致，因此未重設 seed。這是標記身分／牆面方向核對，尚非標定後 ArUco 位姿估測或自動修正 AMCL，也未驗收所有未知起點全自動定位。

最後輪命令全零、maintenance=true、travel、Serial 唯一 owner 8117，AMCL ready，相機已停止。詳見 summary.json、start_reference_review.json、final_health.json、patrol_trace.png、all_markers_contact.jpg。
