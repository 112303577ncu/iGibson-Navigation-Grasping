import os
import cv2
from ultralytics import YOLO

# ========= 可改參數 =========
# best.pt 位於 detection/models/best.pt（本檔在 detection/debug_tools/ 下）
MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "models", "best.pt")
VIDEO_PATH = r"C:\Users\user\Downloads\20260309_170503.mp4"
OUTPUT_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\output_detected.mp4"
CONF_THRESHOLD = 0.4
IMG_SIZE = 640
# ==========================


def main():
    # 載入模型
    model = YOLO(MODEL_PATH)

    # 開啟影片
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"[ERROR] 無法開啟影片: {VIDEO_PATH}")
        return

    # 讀取影片資訊
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    if fps <= 0:
        fps = 30.0

    # 輸出影片編碼
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(OUTPUT_PATH, fourcc, fps, (width, height))

    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1

        # YOLO 推論
        results = model.predict(
            source=frame,
            conf=CONF_THRESHOLD,
            imgsz=IMG_SIZE,
            verbose=False
        )

        result = results[0]
        annotated_frame = result.plot()

        # 可選：印出每個框的資訊
        if result.boxes is not None:
            for box in result.boxes:
                cls_id = int(box.cls[0].item())
                conf = float(box.conf[0].item())
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                class_name = model.names[cls_id]
                print(
                    f"frame={frame_count}, class={class_name}, "
                    f"conf={conf:.2f}, center=({cx},{cy})"
                )

                # 在中心點再畫一個小圓點
                cv2.circle(annotated_frame, (cx, cy), 4, (0, 0, 255), -1)

        # 寫入輸出影片
        out.write(annotated_frame)

        # 即時顯示
        cv2.imshow("YOLO Video Detection", annotated_frame)

        # 按 q 離開
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

    print(f"[OK] 完成，輸出影片已存成: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()