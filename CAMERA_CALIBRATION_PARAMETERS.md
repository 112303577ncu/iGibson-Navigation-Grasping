# X3Plus 相機校正參數

更新日期：2026-07-14  
適用專案：Yahboom X3Plus Sim-to-Real 夾取部署

## 1. 使用說明

本文件整理 X3Plus 目前兩顆相機已完成校正的正式參數：

- 手臂相機（ARM Camera）
- 後方／桅杆相機（REAR Camera）

正式影像尺寸為 **640 × 480 px**。若影像經過縮放或裁切，`fx`、`fy`、`cx`、`cy` 不可直接沿用，必須依縮放比例調整或重新校正。

參數意義：

| 參數 | 意義 | 單位 |
|---|---|---|
| `fx`、`fy` | 水平／垂直焦距 | px |
| `cx`、`cy` | 光學中心座標 | px |
| `k1`、`k2`、`k3` | 徑向畸變係數 | 無單位 |
| `p1`、`p2` | 切向畸變係數 | 無單位 |
| `H` | 相機離地高度 | m |
| `theta` | 相機向下俯角 | degree |

---

## 2. 手臂相機（ARM Camera）

### 2.1 相機辨識

- 類型：Sonix USB Camera
- 辨識方式：**非 `SN0001`** 的 Sonix 相機
- 不可假設固定為 `/dev/video0` 或 `/dev/video1`，應以 `startup_device_check.py` 的 `ARM camera` 標籤為準。

### 2.2 相機內參

| 參數 | 數值 | 單位 |
|---|---:|---|
| 影像寬度 | 640 | px |
| 影像高度 | 480 | px |
| `fx` | 919.08 | px |
| `fy` | 919.41 | px |
| `cx` | 212.23 | px |
| `cy` | 168.42 | px |

內參矩陣：

```text
K_ARM =
[ 919.08    0.00  212.23 ]
[   0.00  919.41  168.42 ]
[   0.00    0.00    1.00 ]
```

### 2.3 鏡頭畸變

OpenCV 係數順序：

```text
D_ARM = [k1, k2, p1, p2, k3]
```

| 參數 | 數值 |
|---|---:|
| `k1` | -0.3764 |
| `k2` | -0.0748 |
| `p1` | -0.0015 |
| `p2` | 0.0035 |
| `k3` | 0.4793 |

完整係數：

```text
D_ARM = [-0.3764, -0.0748, -0.0015, 0.0035, 0.4793]
```

### 2.4 內參校正品質

| 項目 | 結果 |
|---|---:|
| RMS 重投影誤差 | 0.495 px |
| 最差單張誤差 | 0.914 px |
| bbox 底邊中點畸變位移 | 約 9.00 px |
| 畸變處理決策 | 必須去畸變 |

手臂相機的正式處理順序：

```text
YOLO bbox 底邊中點
→ undistort
→ 地面距離與側向偏移換算
```

不可直接使用原始 bbox 像素計算距離。

### 2.5 安裝幾何與地面距離模型

| 參數 | 數值 | 單位 |
|---|---:|---|
| `H_ARM` | 0.332 | m |
| `THETA_ARM` | 36.40 | degree |
| theta 標準差 | 約 0.18 | degree |
| 已驗證距離範圍 | 0.25–0.50 | m |
| 六個校正點最大回推誤差 | ≤ 0.43 | cm |
| Runtime selftest 四點最大誤差 | ≤ 0.33 | cm |

距離換算公式：

```text
distance = H_ARM / tan(
    THETA_ARM + atan((y_undistorted - CY_ARM) / FY_ARM)
)
```

注意：手臂相機裝在 `arm_link4` 上，`H_ARM` 與 `THETA_ARM` 只適用於校正時的 **nav home／觀測姿勢**。手臂姿勢改變後，這兩個安裝幾何參數會失效；相機內參與畸變係數不受手臂姿勢影響。

### 2.6 校正檔案

```text
arm_cam_intrinsics_candidate_rms0495_cx212.json
→ 正式命名為 arm_cam_intrinsics.json
```

---

## 3. 後方／桅杆相機（REAR Camera）

### 3.1 相機辨識

- 類型：Sonix USB Camera
- USB 序號：`SN0001`
- 不可假設固定為 `/dev/video0` 或 `/dev/video1`，應以 `startup_device_check.py` 的 `REAR camera (SN0001)` 標籤為準。

### 3.2 相機內參

| 參數 | 數值 | 單位 |
|---|---:|---|
| 影像寬度 | 640 | px |
| 影像高度 | 480 | px |
| `fx` | 544.16 | px |
| `fy` | 544.82 | px |
| `cx` | 316.98 | px |
| `cy` | 244.79 | px |

內參矩陣：

```text
K_REAR =
[ 544.16    0.00  316.98 ]
[   0.00  544.82  244.79 ]
[   0.00    0.00    1.00 ]
```

### 3.3 鏡頭畸變

OpenCV 係數順序：

```text
D_REAR = [k1, k2, p1, p2, k3]
```

| 參數 | 數值 |
|---|---:|
| `k1` | 0.1172 |
| `k2` | -0.5603 |
| `p1` | 0.0049 |
| `p2` | -0.0103 |
| `k3` | 0.6489 |

完整係數：

```text
D_REAR = [0.1172, -0.5603, 0.0049, -0.0103, 0.6489]
```

### 3.4 內參校正品質

| 項目 | 結果 |
|---|---:|
| RMS 重投影誤差 | 0.374 px |
| 最差單張誤差 | 0.587 px |
| bbox 底邊中點畸變位移 | 約 1.79 px |
| 地面測距畸變決策 | 可暫時忽略 |

「可暫時忽略」只適用於目前 bbox 底邊中點的地面測距。若未來需要處理畫面邊緣、精密座標或整張影像校正，仍應使用 `D_REAR`。

### 3.5 安裝幾何與地面距離模型

| 參數 | 數值 | 單位 |
|---|---:|---|
| `H_REAR` | 0.503 | m |
| `THETA_REAR` | 16.35 | degree |
| theta 標準差 | 約 0.088 | degree |
| 已驗證距離範圍 | 0.70–1.50 | m |
| 最大回推誤差 | ≤ 0.98 | cm |

距離換算公式：

```text
distance = H_REAR / tan(
    THETA_REAR + atan((y_raw - CY_REAR) / FY_REAR)
)
```

### 3.6 校正檔案

```text
rear_cam_intrinsics.json
```

---

## 4. 校正棋盤設定

兩顆相機使用相同設定：

| 項目 | 設定 |
|---|---:|
| 棋盤外觀 | 7 × 10 方格 |
| OpenCV 內角點 | 9 × 6 |
| 每格邊長 | 20 mm |
| 建議擷取數量 | 18 張 |
| 校正影像尺寸 | 640 × 480 px |

---

## 5. 可直接複製的正式常數

```python
# Image size
FRAME_W = 640
FRAME_H = 480

# ARM camera intrinsics
FX_ARM = 919.08
FY_ARM = 919.41
CX_ARM = 212.23
CY_ARM = 168.42
DIST_ARM = (-0.3764, -0.0748, -0.0015, 0.0035, 0.4793)

# ARM camera ground-distance model
H_ARM = 0.332
THETA_ARM = 36.40

# REAR camera intrinsics
FX_REAR = 544.16
FY_REAR = 544.82
CX_REAR = 316.98
CY_REAR = 244.79
DIST_REAR = (0.1172, -0.5603, 0.0049, -0.0103, 0.6489)

# REAR camera ground-distance model
H_REAR = 0.503
THETA_REAR = 16.35
```

---

## 6. 尚未完成的 Phase 3 參數

以下數值目前只是程式 placeholder，**不是已校正結果**：

| 參數 | 程式目前值 | 狀態 |
|---|---:|---|
| `CAM_TO_BASE_X` | 0.0 m | 尚未校正 |
| `CAM_TO_BASE_Y` | 0.0 m | 尚未校正 |
| `SIGN_Y` | 1.0 | 尚未實測定案 |
| `z_offset` | 未設定 | 尚未校正 |

目前已完成的座標鏈：

```text
影像像素
→ 鏡頭去畸變
→ 相機地面距離／側向偏移
```

尚未完成的座標鏈：

```text
相機地面座標
→ X3Plus base_link 絕對座標
```

必須完成 Phase 3 後，才能宣稱視覺輸出的 `(x, y, z)` 已完整對齊 PPO／機器人座標。

---

## 7. 版本與使用警告

1. 正式參數來源為 `CALIBRATION_PLAN.md`、`progress.md` 與 `integration/vision_grasp_pipeline.py`。
2. 舊工具中可能仍出現 `650` 或 `957.6253` 等歷史暫用焦距，這些不是正式校正值，不應複製到正式 pipeline。
3. 手臂相機 bbox 底邊中點必須先去畸變；後方相機目前僅在地面測距用途下允許忽略畸變。
4. 改變解析度、影像裁切方式、相機固定位置或手臂觀測姿勢後，必須重新確認相關參數。
