import os
import cv2
import math
from ultralytics import YOLO

# ========= 1. 檔案與硬體參數 =========
# best.pt 位於同層的 models/ 子資料夾（detection/models/best.pt）
MODEL_PATH = os.path.join(os.path.dirname(__file__), "models", "best.pt")
JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")
VIDEO_PATH = f"http://{JETSON_IP}:8080/stream?topic=/arm_cam/image_raw"
CONF_THRESHOLD = 0.3 
IMG_SIZE = 640

# ========= 2. 手臂相機專用物理參數 =========
# 2026-07-08 Phase 1/2 實測（arm_cam_intrinsics.json, RMS 0.495px；nav home 姿勢有效）
CX_CAM = 212.23
CY = 168.42
FX = 919.08
FY = 919.41
DIST = (-0.3764, -0.0748, -0.0015, 0.0035, 0.4793)  # k1 k2 p1 p2 k3

H = 0.332           # 相機離地高度（nav home 實測）
FIXED_THETA = 36.40  # Phase 2 中位數俯仰角（以「去畸變後」像素解出，std 0.18°）

# ========= 3. 座標系轉換參數 =========
ARM_TO_REAR_OFFSET = 0.22  # 手臂鏡頭到後置鏡頭的物理距離 (22公分)
# =======================================

def undistort_pixel(u, v, iters=8):
    """把單一像素座標去畸變（plumb-bob，與 cv2.undistortPoints(..., P=K) 同法）。
    手臂相機在 bbox 底邊中點的畸變位移約 9px，距離模型前必須先過這步。"""
    k1, k2, p1, p2, k3 = DIST
    xd = (u - CX_CAM) / FX
    yd = (v - CY) / FY
    x, y = xd, yd
    for _ in range(iters):
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return CX_CAM + x * FX, CY + y * FY


def estimate_distance(y_max):
    """計算物體到手臂鏡頭的『真實物理距離』（y_max 需為去畸變後的像素列）"""
    theta = math.radians(FIXED_THETA)
    alpha = math.atan((y_max - CY) / FY)
    total_angle = theta + alpha

    if total_angle <= 0:
        return -1.0

    distance = H / math.tan(total_angle)
    return distance


def estimate_lateral_offset(cx_pixel, ground_dist_m):
    """右正左負；水平像素換算必須使用相機 optical depth。"""
    if ground_dist_m <= 0:
        return 0.0
    theta = math.radians(FIXED_THETA)
    optical_depth = ground_dist_m * math.cos(theta) + H * math.sin(theta)
    return optical_depth * (cx_pixel - CX_CAM) / FX


def main():
    print("[INFO] 載入 YOLO 模型中...")
    model = YOLO(MODEL_PATH)

    print(f"[INFO] 正在開啟手臂相機: {VIDEO_PATH}")
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("[ERROR] 無法連線到相機！")
        return

    print(">>> [系統就緒] 進入近戰視覺測試模式 (統一座標系) <<<")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model.predict(source=frame, conf=CONF_THRESHOLD, imgsz=IMG_SIZE, verbose=False)
        result = results[0]
        annotated_frame = result.plot()

        if result.boxes is not None and len(result.boxes) > 0:
            box = result.boxes[0] 
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx_box = int((x1 + x2) / 2)

            # 1. 先去畸變，再算距離手臂相機的真實距離 (用來算左右偏差)
            cx_u, y2_u = undistort_pixel(cx_box, y2)
            arm_dist = estimate_distance(y2_u)

            if arm_dist > 0:
                # 2. 計算左右偏移（去畸變座標 + 傾斜相機 optical depth）
                target_offset_x = estimate_lateral_offset(cx_u, arm_dist)
                
                # 3. 🚀 【座標統一】把距離轉換成「距離後置主相機」的數據
                unified_dist = arm_dist + ARM_TO_REAR_OFFSET
                
                # 視覺標示
                cv2.circle(annotated_frame, (cx_box, int(y2)), 8, (0, 0, 255), -1)
                side_text = "Right" if target_offset_x > 0 else "Left"
                
                # 畫在畫面上的文字 (顯示統一後的總距離)
                cv2.putText(annotated_frame, f"Total Dist: {unified_dist:.2f}m", (int(x1), int(y2) + 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(annotated_frame, f"Offset: {abs(target_offset_x):.2f}m ({side_text})", (int(x1), int(y2) + 45), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 255), 2)
                
                # 在終端機印出詳細數據供你檢查
                print(f"手臂距離: {arm_dist:.2f}m | 統一總距離: {unified_dist:.2f}m | 偏移: {target_offset_x:.2f}m | Y像素: {int(y2)}")

        # 畫面顯示與準星
        h, w, _ = annotated_frame.shape  
        center_x = int(CX_CAM) 
        cv2.line(annotated_frame, (center_x, 0), (center_x, h), (0, 255, 255), 2)
        cv2.putText(annotated_frame, "Center (X=0)", (center_x + 10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("Arm Camera - Unified Coordinate Test", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
