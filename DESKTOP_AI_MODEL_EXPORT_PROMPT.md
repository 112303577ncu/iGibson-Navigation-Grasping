# 給桌機 AI 的實作指令

請把下面整段交給在 `koala915/igibson_x3_test` 工作的 AI：

---

你正在 `koala915/igibson_x3_test` repository 工作。請為目前開啟的 PR #2
`grasp-home-fov-v18` 實作「模型 Release 匯出端」，讓另一個 repository
`koala915/claude/tree/main/x3plus` 能安全取得 v18，而不是手動複製裸權重或部署腳本。

必要背景：

- 正式部署控制器以 `claude/x3plus/grasp/x3plus_real_grasp.py` 為準。
- 不可用本 repo 的 `training/x3plus_real_grasp.py` 覆蓋部署端；兩份程式已分叉。
- v18 配對：
  - `trained_6d_models_v18/ppo_6d_final_ready_for_real_robot.zip`
  - `trained_6d_models_v18/vecnormalize_6d_final.pkl`
- v18 contract：obs 28D、action 6D。
- C3 sim home：`[0.0, -0.4, -1.4, -1.4, 0.0]`。
- C3 API home：`[90.0, 67.08, 9.79, 9.79, 90.0, 30.0]`。
- `arm_hw_invert=[false,false,false,false,false]`；S6 30=open、180=closed。
- 模擬驗收 97/100，但 C3 ruler FOV、實測 reach、C3 homography、Jetson dry-run 尚未完成，
  因此目前只能標記 `sim-approved`，不可標記 `hardware-approved`。

請完成：

1. 從 `koala915/claude` 的 `x3plus/model_tools/` 同步以下通用工具到本 repo 的
   `model_tools/`，保持格式及 schema 相容：
   - `model_package.py`
   - `export_model_package.py`
   - `install_model_package.py`
   - `verify_model_package.py`
   - `grasp_v18_manifest.template.json`
2. 將 manifest 的 `source.commit` 更新為匯出當下 PR branch 的實際 commit SHA。
3. 不得手寫假的 SHA256；必須由 exporter 從實際 `.zip`／`.pkl` 計算並寫入成品
   `manifest.json`。
4. 新增測試，至少覆蓋：
   - 正確 package 可驗證；
   - 任一權重被修改會 SHA256 fail；
   - model／VecNormalize 缺一會 fail；
   - grasp contract 不是 28D／6D 會 fail；
   - path traversal archive 會被拒絕。
5. 在有 Stable-Baselines3 的桌機環境執行 `verify_model_package.py --deep`，確認：
   - PPO observation space `(28,)`；
   - PPO action space `(6,)`；
   - VecNormalize `obs_rms.mean.shape == (28,)`。
6. 產生 `dist/grasp-v18.0.0-candidate.tar.gz`，但不要 commit `dist/`。
7. 更新 `.gitignore` 忽略 `dist/`。
8. 新增或更新文件，寫明發布命令：

```bash
gh release create grasp-v18.0.0-candidate \
  dist/grasp-v18.0.0-candidate.tar.gz \
  --repo koala915/igibson_x3_test \
  --title "Grasp v18.0.0 candidate" \
  --notes "97/100 sim-approved; C3 hardware gates remain required"
```

9. 在 PR 回報：成品檔名、兩個 SHA256、source commit、deep verification 結果。
10. 除非使用者明確授權，先不要真的建立 GitHub Release，也不要 push／merge。

安全限制：

- model 與 VecNormalize 必須永遠成對。
- Release tag／asset 視為 immutable；內容變更必須升版本。
- 不得把 token 寫入 repository、log 或命令範例。
- 不得把未完成實機 gates 的 package 標為 `hardware-approved`。
- 不得修改或宣稱已更新另一個 repository 的 Jetson 部署預設。

完成後執行所有新增測試及現有的 v18 deterministic evaluation／dry-run（可執行範圍內），
並列出未完成的硬體 gates。

---

