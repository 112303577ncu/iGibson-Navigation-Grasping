# 📦 sugarbox (藍色盒子) YOLOv11 物件偵測模型使用指南

本文件為您詳細說明訓練完成的 **YOLOv11** 最佳模型權重檔（`best.pt`）、模型規格、資料集檔案結構，以及如何在各式應用場景中匯入與部署模型。

---

## 🎯 1. 核心模型檔案與路徑說明

### 🌟 最佳模型權重 (Best Model Weight)
- **檔案路徑**：`D:\train_pic\sugarbox_yolo11_model\best.pt`
- **檔案大小**：約 `5.5 MB`
- **模型架構**：YOLOv11 Nano (`yolo11n`)
- **偵測類別**：
  - `ID 0`: **`sugarbox`**（藍色盒子）
- **主要特點**：
  - 經歷 410 張多樣化數據增強圖片（包含強光/陰影、色溫偏移、高斯/運動模糊、噪訊與合成背景）的 50 輪高強度訓練。
  - **mAP@50-95 達到 99.5%**，對於各種光線與複雜背景具備極高魯棒性。

### 📁 相關目錄與檔案一覽

| 目錄 / 檔案路徑 | 類型 | 說明 |
| :--- | :--- | :--- |
| `D:\train_pic\sugarbox_yolo11_model\best.pt` | **模型權重** | **主要使用檔**：經過 Validation 精準度最高的最佳權重。 |
| `D:\train_pic\sugarbox_yolo11_model\last.pt` | 模型權重 | 訓練完成最後一輪（第 50 Epoch）的權重。 |
| `D:\train_pic\predict_results` | 圖片資料夾 | 模型對驗證集進行推論預測後輸出的視覺化圖檔。 |
| `D:\train_pic\annotations_preview` | 圖片資料夾 | 原始照片自動標註後的標註框確認圖檔。 |
| `D:\train_pic\dataset\data.yaml` | 配置文件 | YOLO 格式資料集配置文件（包含類別名稱與相對路徑）。 |
| `D:\train_pic\runs\sugarbox_yolo11_augmented` | 訓練日誌 | 包含訓練過程 Loss 曲線、混淆矩陣 (Confusion Matrix)、F1-Score 曲線等分析圖。 |

---

## 💻 2. 如何使用 `best.pt` 進行推論 (Code Examples)

在開始前，請確保已安裝 `ultralytics` 套件：
```bash
pip install ultralytics
```

### 範例 A：對單張或多張圖片進行辨識 (Python)

```python
from ultralytics import YOLO

# 1. 載入訓練好的模型權重
model = YOLO(r'D:\train_pic\sugarbox_yolo11_model\best.pt')

# 2. 進行推論 (source 可傳入單張圖片、資料夾路徑或圖片網址)
results = model.predict(
    source=r'D:\train_pic\Pic\20260731_231400.jpg',
    conf=0.25,     # 置信度門檻 (Confidence Threshold)
    save=True,     # 是否將標有紅/綠框的結果圖片存檔
    save_txt=True  # 是否儲存文字格式的偵測結果
)

# 3. 解析與讀取辨識結果
for result in results:
    boxes = result.boxes
    for box in boxes:
        cls_id = int(box.cls[0])            # 類別 ID (0)
        cls_name = result.names[cls_id]     # 類別名稱 ('sugarbox')
        confidence = float(box.conf[0])     # 置信度 (0.0 ~ 1.0)
        bounding_box = box.xyxy[0].tolist() # 邊界框座標 [xmin, ymin, xmax, ymax]
        
        print(f"偵測到: {cls_name} | 置信度: {confidence:.2%} | 座標: {bounding_box}")
```

---

### 範例 B：即時攝影機 (Webcam) / 影片即時辨識 (Python)

```python
import cv2
from ultralytics import YOLO

# 載入模型
model = YOLO(r'D:\train_pic\sugarbox_yolo11_model\best.pt')

# 開啟預設攝影機 (0) 或 影片檔路徑 (例如 'video.mp4')
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # 執行推論
    results = model(frame, conf=0.5)

    # 在影像畫面上繪製邊界框與標籤
    annotated_frame = results[0].plot()

    # 顯示即時畫面
    cv2.imshow("sugarbox YOLOv11 Real-Time Detection", annotated_frame)

    # 按 'q' 鍵退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

### 範例 C：使用命令列 (CLI) 快速測試

您可以直接在終端機 (PowerShell / Command Prompt) 使用 YOLO CLI 命令：

```powershell
# 對指定照片進行測試並儲存結果
yolo predict model="D:\train_pic\sugarbox_yolo11_model\best.pt" source="D:\train_pic\Pic\20260731_231400.jpg" conf=0.25
```

---

## 🚀 3. 模型匯出 (Exporting Model for Deployment)

若您未來需要將模型部署至 C++、嵌入式設備（如 Jetson Nano / Raspberry Pi）、Android/iOS 手機App 或網頁端，可以輕鬆匯出為 ONNX 或其他格式：

```python
from ultralytics import YOLO

# 載入模型
model = YOLO(r'D:\train_pic\sugarbox_yolo11_model\best.pt')

# 匯出為 ONNX 格式 (跨平台通用格式)
model.export(format='onnx', dynamic=True)
# 導出的檔案將位於：D:\train_pic\sugarbox_yolo11_model\best.onnx
```

支援的匯出格式包括：
- `onnx` (通用跨平台)
- `engine` (NVIDIA TensorRT 超高速推理)
- `tflite` (TensorFlow Lite - 行動端/樹莓派)
- `torchscript` (PyTorch C++ API 部署)

---

## 📌 4. 備份與移轉說明

若您需要將模型移至其他電腦或伺服器使用：
- **您只需要複製這一個檔案**：`D:\train_pic\sugarbox_yolo11_model\best.pt`
- 該權重檔內已包含模型結構、類別名稱（`0: sugarbox`）與所有網路參數，**無需附帶額外配置文件即可獨立運行**！
