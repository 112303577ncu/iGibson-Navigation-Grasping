import cv2
import math
import sys
import rospy
from geometry_msgs.msg import Twist
from ultralytics import YOLO

ENABLE_MOTION = "--real" in sys.argv[1:]

# ========= 1. 檔案與硬體參數 (Jetson Nano 本機環境) =========
# ⚠️ 請確保 best.pt 跟你這個 Python 檔案放在 JupyterLab 的同一個資料夾裡
MODEL_PATH = "best.pt" 

# 因為程式現在跑在 Nano 上，直接讀取本機的 8080 串流最穩
VIDEO_PATH = "http://127.0.0.1:8080/stream?topic=/usb_cam/image_raw" 
CONF_THRESHOLD = 0.3 
IMG_SIZE = 640

# ========= 2. 手臂相機專用物理參數 =========
CX_CAM = 320.0
CY = 240.0
FX = 650.0  
FY = 650.0  

H = 0.33            # 相機離地高度 (33公分)
FIXED_THETA = 41.5  # 透過實測反推的精準俯仰角

# ========= 3. 座標系轉換參數 =========
ARM_TO_REAR_OFFSET = 0.22  # 🚀 [修正] 手臂鏡頭到後置鏡頭的物理距離 (22公分)
# =======================================

def estimate_distance(y_max):
    theta = math.radians(FIXED_THETA) 
    alpha = math.atan((y_max - CY) / FY)
    total_angle = theta + alpha
    if total_angle <= 0: return -1.0
    return H / math.tan(total_angle)

def main():
    if not ENABLE_MOTION:
        print("[SAFETY] Vision-only mode; add --real to permit /cmd_vel motion.")
    # --- 啟動 ROS 節點與馬達廣播器 ---
    rospy.init_node('arm_cam_navigator', anonymous=True)
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("[INFO] 載入 YOLO 模型中...")
    model = YOLO(MODEL_PATH)

    print(f"[INFO] 正在連線本機相機串流: {VIDEO_PATH}")
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("[ERROR] 無法連線到相機串流！請確認 Launch 檔有啟動。")
        return

    print(">>> [系統就緒] 進入近戰導航模式 (包含底盤移動) <<<")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret: break

        results = model.predict(source=frame, conf=CONF_THRESHOLD, imgsz=IMG_SIZE, verbose=False)
        result = results[0]
        
        trash_found = False
        target_dist = -1.0
        target_offset_x = 0.0

        if result.boxes is not None and len(result.boxes) > 0:
            box = result.boxes[0] 
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx_box = int((x1 + x2) / 2)
            
            # 1. 計算手臂距離
            arm_dist = estimate_distance(y2)
            
            if arm_dist > 0:
                trash_found = True
                # 2. 計算左右偏移
                target_offset_x = arm_dist * (cx_box - CX_CAM) / FX
                # 3. 計算統一座標 (僅供顯示/地圖使用，近戰控制我們直接看 arm_dist)
                unified_dist = arm_dist + ARM_TO_REAR_OFFSET
                
                print(f"[目標鎖定] 統一距離: {unified_dist:.2f}m | 手臂距離: {arm_dist:.2f}m | 偏移: {target_offset_x:.2f}m")

        # ==========================================
        # 🤖 機器人底盤控制邏輯 (P-Control)
        # ==========================================
        if trash_found:
            # 1. 轉向：垃圾偏右 (X>0)，車子右轉 (Z<0)。常數 1.5 讓轉向靈敏一點
            vel_msg.angular.z = -target_offset_x * 1.5  

            # 2. 前進：如果垃圾距離手臂相機大於 8 公分 (0.08m)，就繼續慢慢往前督
            if arm_dist > 0.08:
                vel_msg.linear.x = 0.1  # 安全龜速 (0.1 m/s)
            else:
                # 督到垃圾了！煞車！
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                print(">>> 🎯 到達目標位置！停車！ <<<")
        else:
            # 沒看到垃圾，原地煞車待命
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        # 發射速度指令給底盤！
        if not ENABLE_MOTION:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        cmd_pub.publish(vel_msg)

    # 程式如果被強制結束 (Ctrl+C)，確保車子煞車
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    cmd_pub.publish(vel_msg)
    cap.release()

if __name__ == "__main__":
    main()
