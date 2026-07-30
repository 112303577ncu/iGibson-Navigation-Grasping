# X3Plus 模型交接流程（最低成本過渡方案）

這套流程保留兩個 GitHub repository，但不再手動複製裸 `.zip`／`.pkl`，也不再用訓練
repository 裡的舊部署腳本覆蓋 Jetson 版本。

## 模型包契約

每個 Release asset 是一個 `.tar.gz`，內含：

```text
grasp-v18.0.0-candidate/
├── manifest.json
├── model.zip
└── vecnormalize.pkl
```

`manifest.json` 綁定模型、VecNormalize、SHA256、28D/6D 契約、grasp-home、來源 commit、
模擬評測及實機 gates。模型與 VecNormalize 不可拆開替換。

狀態定義：

- `candidate`：尚未完成正式模擬驗收。
- `sim-approved`：完成模擬驗收，可安裝與 dry-run，不可 `--real`。
- `hardware-approved`：所有 manifest hardware gates 均為 `true`，才可由 wrapper 執行 `--real`。
- `retired`：不再使用，只保留可追溯性。

## 桌機：匯出與發布

將 `model_tools/` 複製或同步到 `igibson_x3_test` 後，從 v18 branch 執行：

```bash
python model_tools/export_model_package.py \
  --manifest-template model_tools/grasp_v18_manifest.template.json \
  --model trained_6d_models_v18/ppo_6d_final_ready_for_real_robot.zip \
  --vecnorm trained_6d_models_v18/vecnormalize_6d_final.pkl \
  --output dist/grasp-v18.0.0-candidate.tar.gz

python model_tools/install_model_package.py \
  dist/grasp-v18.0.0-candidate.tar.gz \
  --install-root dist/verify-install

python model_tools/verify_model_package.py \
  dist/verify-install/grasp-v18.0.0-candidate --deep
```

確認來源 commit 已填入 manifest template，且 exporter 輸出的兩個 SHA256 已記錄後發布：

```bash
gh release create grasp-v18.0.0-candidate \
  dist/grasp-v18.0.0-candidate.tar.gz \
  --repo koala915/igibson_x3_test \
  --title "Grasp v18.0.0 candidate" \
  --notes "97/100 sim-approved; C3 hardware gates remain required"
```

Release tag 不要重複覆寫。同一個 tag 的 asset 應視為 immutable；有任何內容變更就升版本。

## 筆電／Jetson：下載與驗證

私有 repository 需要 read-only token。可以使用已登入的 `gh`，或以環境變數提供 token：

```bash
export GITHUB_TOKEN=REDACTED_READ_ONLY_TOKEN
python3 model_tools/fetch_model_release.py \
  --repo koala915/igibson_x3_test \
  --tag grasp-v18.0.0-candidate \
  --install-root grasp/model_packages
```

若 Jetson 不方便登入 GitHub，可在筆電下載 `.tar.gz` 後用 `scp` 傳過去，再離線安裝：

```bash
python3 model_tools/install_model_package.py \
  grasp-v18.0.0-candidate.tar.gz \
  --install-root grasp/model_packages
```

驗證 SHA256、manifest 及實際 SB3 observation/action/VecNormalize shape：

```bash
python3 model_tools/verify_model_package.py \
  grasp/model_packages/grasp-v18.0.0-candidate --deep
```

## 安全啟動

候選版只允許 dry-run：

```bash
python3 model_tools/run_grasp_package.py \
  grasp/model_packages/grasp-v18.0.0-candidate -- \
  --obj-x 0.26 --obj-y 0.0 --obj-z 0.02
```

若在命令尾端加 `--real`，wrapper 會要求：

1. manifest `status` 必須是 `hardware-approved`；
2. 所有 `hardware_gates` 必須是 `true`；
3. model／VecNormalize SHA256 必須正確；
4. wrapper 自動傳入成對路徑及該版本的 `grasp_home_api_deg`。

v18 完成 C3 ruler FOV、reach、homography 與 Jetson dry-run 後，更新 manifest、重新匯出，
發布正式且不可覆寫的新 tag（例如 `grasp-v18.0.0`），不要修改 candidate Release asset。

## 不應做的事

- 不要把 `igibson_x3_test/training/x3plus_real_grasp.py` 覆蓋到 Jetson。
- 不要只傳模型 `.zip`，漏掉 VecNormalize 或 manifest。
- 不要讓 v18 從 v17 grasp-home 啟動。
- 不要在實機 gates 未完成時把 `sim-approved` 改名假裝成 production。
- 不要把具寫入權限的 GitHub token commit 或寫進腳本。

