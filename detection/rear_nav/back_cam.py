import cv2
import math
import csv
import json
import os
import time
import numpy as np
from pathlib import Path
from ultralytics import YOLO


# ============================================================
# 1. 模型與後鏡頭串流
# ============================================================

YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v8i.yolov11(0517)\runs\detect\train\weights\best.pt"

JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")
TOPIC_REAR = "/back_cam/image_raw"
URL_REAR = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_REAR}"

YOLO_CONF = 0.3
IMG_SIZE = 640


# ============================================================
# 2. 後置相機物理參數
# ============================================================

FRAME_W = 640
FRAME_H = 480

# 你以前距離公式用的內參
# 如果你現在有更準的標定值，可以改這裡
FY_REAR = 956.0294
CY_REAR = 221.3649

FX_REAR = 957.6253
CX_REAR = 320.0

# 後鏡頭離地高度，單位 m
# 如果你實際不是 50 cm，就改這裡
H_REAR = 0.50


# ============================================================
# 3. 輸出檔案
# ============================================================

RAW_CSV_PATH = Path("rear_camera_calibration_points.csv")
RESULT_JSON_PATH = Path("rear_camera_calibration_result.json")


# ============================================================
# 4. 工具函式
# ============================================================

def select_largest_box(results):
    """
    選畫面中最大的 YOLO box。
    校正時建議畫面只放一個垃圾，避免選錯。
    """
    if results.boxes is None or len(results.boxes) == 0:
        return None

    best_box = None
    best_area = -1

    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        area = max(1.0, x2 - x1) * max(1.0, y2 - y1)

        if area > best_area:
            best_area = area
            best_box = box

    return best_box


def theta_from_point(real_dist_m, y2):
    """
    用一組資料反推相機俯仰角 theta。

    real_dist_m:
        目標真實前方距離，單位 m

    y2:
        YOLO box 底部像素座標

    公式：
        alpha = atan((y2 - CY) / FY)
        total_angle = atan(H / real_dist)
        theta = total_angle - alpha
    """
    alpha = math.atan((y2 - CY_REAR) / FY_REAR)
    total_angle = math.atan(H_REAR / real_dist_m)
    theta_rad = total_angle - alpha
    theta_deg = math.degrees(theta_rad)
    return theta_deg


def estimate_distance_from_y2(y2, theta_deg):
    """
    用校正好的 theta 估算距離。
    """
    theta = math.radians(theta_deg)
    alpha = math.atan((y2 - CY_REAR) / FY_REAR)
    total_angle = theta + alpha

    if total_angle <= 0:
        return -1.0

    dist_m = H_REAR / math.tan(total_angle)
    return dist_m


def estimate_offset_x(cx_box, dist_m):
    """
    估算左右偏移。
    右正左負。
    """
    return dist_m * (cx_box - CX_REAR) / FX_REAR


def robust_average(values):
    """
    用 median + 去掉太離群的點。
    校正點少時比單純平均穩一點。
    """
    if len(values) == 0:
        return None, []

    arr = np.array(values, dtype=float)

    if len(arr) < 4:
        return float(np.mean(arr)), list(arr)

    med = np.median(arr)
    abs_dev = np.abs(arr - med)
    mad = np.median(abs_dev)

    if mad < 1e-9:
        kept = arr
    else:
        # 2.5 MAD 以內保留
        kept = arr[abs_dev <= 2.5 * mad]

    if len(kept) == 0:
        kept = arr

    return float(np.mean(kept)), list(kept)


def save_points_csv(points):
    with RAW_CSV_PATH.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "index",
                "real_dist_m",
                "x1",
                "y1",
                "x2",
                "y2",
                "cx_box",
                "cy_box",
                "box_h",
                "box_w",
                "theta_deg"
            ]
        )
        writer.writeheader()
        writer.writerows(points)


def save_result_json(result):
    with RESULT_JSON_PATH.open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=4, ensure_ascii=False)


def draw_status_panel(frame, target_found, info_lines):
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (630, 185), (0, 0, 0), -1)
    frame[:] = cv2.addWeighted(overlay, 0.50, frame, 0.50, 0)

    cv2.putText(
        frame,
        "REAR CAMERA CALIBRATION - NO ROS",
        (25, 38),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    target_text = "FOUND" if target_found else "LOST"
    color = (0, 255, 0) if target_found else (0, 0, 255)

    cv2.putText(
        frame,
        f"TARGET: {target_text}",
        (25, 68),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        color,
        2
    )

    y = 96
    for line in info_lines[:4]:
        cv2.putText(
            frame,
            line,
            (25, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            1
        )
        y += 25

    cv2.putText(
        frame,
        "C: capture point | F: finish | Q: quit",
        (25, 462),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        2
    )


# ============================================================
# 5. 主程式
# ============================================================

def main():
    print("[INFO] 後置相機校正程式，不使用 ROS")
    print("[INFO] 功能：用 YOLO box 底部 y2 + 你輸入的真實距離，反推後鏡頭俯角 theta")
    print()
    print(f"[INFO] Camera URL: {URL_REAR}")
    print(f"[INFO] raw csv: {RAW_CSV_PATH.resolve()}")
    print(f"[INFO] result json: {RESULT_JSON_PATH.resolve()}")
    print()

    print("[INFO] Loading YOLO model...")
    model = YOLO(YOLO_MODEL_PATH)

    print("[INFO] Opening rear camera stream...")
    cap = cv2.VideoCapture(URL_REAR)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print("[ERROR] 無法開啟後置相機串流。")
        print("[CHECK] 確認 Jetson web_video_server 有開，網址是：")
        print(URL_REAR)
        return

    points = []
    last_box_info = None
    theta_preview = None

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("[WARN] 無法讀取 frame，重試中...")
                time.sleep(0.05)
                continue

            frame = cv2.resize(frame, (FRAME_W, FRAME_H))

            results = model.predict(
                source=frame,
                conf=YOLO_CONF,
                imgsz=IMG_SIZE,
                verbose=False
            )[0]

            annotated = results.plot()
            box = select_largest_box(results)

            target_found = False
            info_lines = []

            if box is not None:
                target_found = True

                x1, y1, x2, y2 = box.xyxy[0].tolist()

                x1_i = int(x1)
                y1_i = int(y1)
                x2_i = int(x2)
                y2_i = int(y2)

                cx_box = int((x1 + x2) / 2)
                cy_box = int((y1 + y2) / 2)

                box_w = int(max(1, x2 - x1))
                box_h = int(max(1, y2 - y1))

                last_box_info = {
                    "x1": x1_i,
                    "y1": y1_i,
                    "x2": x2_i,
                    "y2": y2_i,
                    "cx_box": cx_box,
                    "cy_box": cy_box,
                    "box_w": box_w,
                    "box_h": box_h
                }

                # 畫底部點與中心線
                cv2.circle(annotated, (cx_box, y2_i), 6, (0, 255, 0), -1)
                cv2.line(annotated, (int(CX_REAR), 0), (int(CX_REAR), FRAME_H), (255, 255, 255), 1)
                cv2.line(annotated, (0, y2_i), (FRAME_W, y2_i), (0, 255, 0), 1)

                info_lines.append(f"y2 bottom pixel: {y2_i}")
                info_lines.append(f"cx: {cx_box} | box_h: {box_h}")

                if theta_preview is not None:
                    dist_preview = estimate_distance_from_y2(y2_i, theta_preview)
                    if dist_preview > 0:
                        offset_x = estimate_offset_x(cx_box, dist_preview)
                        info_lines.append(f"preview theta: {theta_preview:.2f} deg")
                        info_lines.append(f"preview dist: {dist_preview:.2f} m | x: {offset_x:.2f} m")

                        cv2.putText(
                            annotated,
                            f"Dist: {dist_preview:.2f}m | X: {offset_x:.2f}m",
                            (x1_i, min(FRAME_H - 10, y2_i + 28)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.62,
                            (0, 255, 255),
                            2
                        )
                    else:
                        info_lines.append(f"preview theta: {theta_preview:.2f} deg")
                        info_lines.append("preview dist invalid")
                else:
                    info_lines.append("press C to capture calibration point")
                    info_lines.append("need real distance input")

            else:
                last_box_info = None
                info_lines.append("No YOLO target.")
                info_lines.append("Put one trash object in view.")

            draw_status_panel(annotated, target_found, info_lines)

            cv2.imshow("Rear Camera Calibration NO ROS", annotated)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("[INFO] Quit without finishing.")
                break

            elif key == ord("c"):
                if last_box_info is None:
                    print("[WARN] 目前沒有偵測到目標，不能記錄校正點。")
                    continue

                print("\n----------------------------------------")
                print("[CAPTURE] 新增一個校正點")
                print(f"目前 YOLO box bottom y2 = {last_box_info['y2']}")
                print("請輸入這個垃圾到後鏡頭/車體參考點的真實前方距離，單位：公尺")
                print("例如 1.2 公尺就輸入 1.2")
                print("----------------------------------------")

                try:
                    real_dist_m = float(input("真實距離 m = ").strip())
                except Exception:
                    print("[WARN] 輸入錯誤，略過。")
                    continue

                if real_dist_m <= 0:
                    print("[WARN] 距離必須 > 0，略過。")
                    continue

                theta_deg = theta_from_point(real_dist_m, last_box_info["y2"])

                row = {
                    "index": len(points) + 1,
                    "real_dist_m": real_dist_m,
                    "x1": last_box_info["x1"],
                    "y1": last_box_info["y1"],
                    "x2": last_box_info["x2"],
                    "y2": last_box_info["y2"],
                    "cx_box": last_box_info["cx_box"],
                    "cy_box": last_box_info["cy_box"],
                    "box_h": last_box_info["box_h"],
                    "box_w": last_box_info["box_w"],
                    "theta_deg": theta_deg
                }

                points.append(row)

                theta_values = [p["theta_deg"] for p in points]
                theta_preview, kept = robust_average(theta_values)

                print(f"[OK] 已新增第 {len(points)} 點")
                print(f"     real_dist = {real_dist_m:.2f} m")
                print(f"     y2 = {last_box_info['y2']}")
                print(f"     theta from this point = {theta_deg:.3f} deg")
                print(f"     current theta avg = {theta_preview:.3f} deg")
                print()

            elif key == ord("f"):
                if len(points) < 2:
                    print("[WARN] 至少建議記錄 2 個點以上，最好 4~6 個點。")
                    continue

                theta_values = [p["theta_deg"] for p in points]
                theta_avg, kept = robust_average(theta_values)

                errors = []
                for p in points:
                    pred_dist = estimate_distance_from_y2(p["y2"], theta_avg)
                    err = pred_dist - p["real_dist_m"]
                    errors.append(err)
                    p["pred_dist_m"] = pred_dist
                    p["error_m"] = err
                    p["abs_error_m"] = abs(err)

                mean_abs_error = float(np.mean([abs(e) for e in errors]))
                max_abs_error = float(np.max([abs(e) for e in errors]))

                result = {
                    "theta_rear_deg": theta_avg,
                    "camera_height_m": H_REAR,
                    "fx": FX_REAR,
                    "fy": FY_REAR,
                    "cx": CX_REAR,
                    "cy": CY_REAR,
                    "points_count": len(points),
                    "theta_values_deg": theta_values,
                    "theta_values_used_by_robust_avg": kept,
                    "mean_abs_error_m": mean_abs_error,
                    "max_abs_error_m": max_abs_error,
                    "points": points
                }

                save_points_csv(points)
                save_result_json(result)

                print("\n===================================================")
                print("後置相機校正完成")
                print("===================================================")
                print(f"THETA_REAR = {theta_avg:.3f} deg")
                print(f"mean abs error = {mean_abs_error * 100:.1f} cm")
                print(f"max abs error  = {max_abs_error * 100:.1f} cm")
                print()
                print("之後你的後置相機距離公式用：")
                print(f"THETA_REAR = {theta_avg:.3f}")
                print()
                print(f"raw csv     = {RAW_CSV_PATH.resolve()}")
                print(f"result json = {RESULT_JSON_PATH.resolve()}")
                print("===================================================")
                print()

                # 完成後不強制離開，讓你可以看 preview
                theta_preview = theta_avg

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C detected.")

    finally:
        cap.release()
        cv2.destroyAllWindows()

        if len(points) > 0:
            try:
                save_points_csv(points)
                print(f"[INFO] 已保存目前校正點到：{RAW_CSV_PATH.resolve()}")
            except Exception as e:
                print(f"[WARN] save csv failed: {e}")

        print("[INFO] 程式結束。")


if __name__ == "__main__":
    main()
