import cv2
import math
import os
import time
import sys
import numpy as np
import roslibpy
from stable_baselines3 import PPO
from ultralytics import YOLO

ENABLE_MOTION = "--real" in sys.argv[1:]

# ========= 1. 檔案絕對路徑 =========
RL_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\best_model.zip" 
YOLO_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\runs\detect\train6\weights\best.pt"

# ========= 2. 網路與相機串流設定 =========
JETSON_IP = os.getenv("X3PLUS_JETSON_HOST", "172.31.28.252")
ROS_PORT = 9090

TOPIC_REAR = "/back_cam/image_raw" 
TOPIC_ARM  = "/arm_cam/image_raw"

URL_REAR = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_REAR}"
URL_ARM  = f"http://{JETSON_IP}:8080/stream?topic={TOPIC_ARM}"

# ========= 3. 物理與安全參數設定 =========
MAX_LINEAR_VEL = 0.2
MAX_ANGULAR_VEL = 0.8
SAFE_SPEED_RATIO = 0.3  # AI 油門封印
SAFE_TURN_RATIO = 0.3   # AI 方向盤封印
e_stop_active = False

latest_lidar_52 = np.ones(52) * 5.0  
current_linear_vel = 0.0

# 🎯 視覺圓圈雷達設定 (新加入)
CENTER_X, CENTER_Y = 320, 240
SWITCH_RADIUS = 120  # 判定圓圈的半徑 (越大越容易觸發切換)

# ========= 4. 核心運算函數 =========
def get_observation(lidar_data, goal_dist, goal_angle, current_vel):
    processed_lidar = np.clip(lidar_data / 5.0, 0, 1.0)
    obs = np.zeros(55)
    obs[0:52] = processed_lidar
    obs[52] = goal_dist
    obs[53] = goal_angle
    obs[54] = current_vel
    return obs.astype(np.float32)

# ========= 5. 主程式 =========
def main():
    global current_linear_vel, e_stop_active

    if not ENABLE_MOTION:
        print("[SAFETY] Vision-only mode; add --real to permit /cmd_vel motion.")

    print("[INFO] 載入 YOLO 視覺神經...")
    yolo_model = YOLO(YOLO_MODEL_PATH)
    
    print("[INFO] 載入 RL 決策大腦...")
    rl_model = PPO.load(RL_MODEL_PATH)

    print(f"[INFO] 連線至 Jetson Nano ROS ({JETSON_IP})...")
    client = roslibpy.Ros(host=JETSON_IP, port=ROS_PORT)
    client.run()
    
    # 🔌 關鍵修復：宣告 Topic 後必須 advertise，否則 0.000 煞車指令送不出去！
    cmd_pub = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
    cmd_pub.advertise() 

    print(f"[INFO] 開啟雙鏡頭串流...")
    cap_rear = cv2.VideoCapture(URL_REAR)
    cap_arm = cv2.VideoCapture(URL_ARM)

    print(">>> [系統就緒] 雙鏡頭自動切換導航模式啟動 <<<")
    print(" [ S ] 鍵：啟動/解除 緊急煞車 (E-Stop)")
    print(" [ Q ] 鍵：安全退出程式")

    active_camera = 'REAR' 
    target_dt = 0.1        

    try:
        while True:
            loop_start = time.time()

            ret_r, frame_rear = cap_rear.read()
            ret_a, frame_arm = cap_arm.read()

            if not ret_r or not ret_a:
                print("[警告] 遺失畫面訊號...")
                time.sleep(0.1)
                continue

            # 將畫面縮放到一致大小，確保 UI 整齊
            frame_rear = cv2.resize(frame_rear, (640, 480))
            frame_arm = cv2.resize(frame_arm, (640, 480))

            goal_dist = 5.0  
            goal_angle = 0.0

            # 根據當前啟動的鏡頭，決定要把哪張圖餵給 YOLO
            frame_to_process = frame_rear if active_camera == 'REAR' else frame_arm
            results = yolo_model.predict(source=frame_to_process, conf=0.3, imgsz=640, verbose=False)[0]
            
            # 取得畫好框框的影像
            annotated_frame = results.plot()
            
            if results.boxes is not None and len(results.boxes) > 0:
                box = results.boxes[0]
                cx_box = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                cy_box = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
                
                # 計算垃圾中心點與畫面正中心的像素距離
                pixel_dist_to_center = math.hypot(cx_box - CENTER_X, cy_box - CENTER_Y)
                
                # 簡單粗暴的角度計算 (給 RL 用的)
                goal_angle = (320 - cx_box) / 320.0 
                goal_dist = pixel_dist_to_center / 320.0 # 歸一化距離

                # 🎯 圓圈雷達切換邏輯 🎯
                if active_camera == 'REAR':
                    if pixel_dist_to_center < SWITCH_RADIUS:
                        print(">>> 🎯 目標進入圓圈！切換至【機械手臂鏡頭】！ <<<")
                        active_camera = 'ARM'
                elif active_camera == 'ARM':
                    if pixel_dist_to_center > SWITCH_RADIUS + 50: # 加上 50 緩衝，避免在邊界瘋狂來回切換
                        print(">>> 👁️ 目標遠離！切換回【後置主鏡頭】！ <<<")
                        active_camera = 'REAR'

            # --- 畫面合成 (雙視窗並排) ---
            # 依照當前模式，決定左右兩邊要顯示什麼
            if active_camera == 'REAR':
                display_rear = annotated_frame
                display_arm = frame_arm
                # 畫上圓圈雷達
                cv2.circle(display_rear, (CENTER_X, CENTER_Y), SWITCH_RADIUS, (0, 0, 255), 2)
                cv2.putText(display_rear, "[ ACTIVE ]", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                display_rear = frame_rear
                display_arm = annotated_frame
                cv2.putText(display_arm, "[ ACTIVE ]", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 水平合併兩張影像 (變成 1280 x 480 的寬螢幕)
            dashboard = cv2.hconcat([display_rear, display_arm])
            cv2.putText(dashboard, "REAR CAMERA", (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(dashboard, "ARM CAMERA", (660, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # --- 模擬 RL 大腦思考 ---
            obs = get_observation(latest_lidar_52, goal_dist, goal_angle, current_linear_vel)
            action, _ = rl_model.predict(obs, deterministic=True)
            
            linear_x = float(action[0]) * MAX_LINEAR_VEL * SAFE_SPEED_RATIO
            angular_z = float(action[1]) * MAX_ANGULAR_VEL * SAFE_TURN_RATIO

            # 急停啟動時，覆蓋速度為 0
            if e_stop_active:
                linear_x = 0.0
                angular_z = 0.0
                cv2.putText(dashboard, "!!! E-STOP ACTIVE !!!", (450, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)

            current_linear_vel = linear_x

            # 發送馬達指令
            if not ENABLE_MOTION:
                linear_x = 0.0
                angular_z = 0.0
            cmd_pub.publish(roslibpy.Message({
                'linear': {'x': linear_x, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z}
            }))

            cv2.imshow("Twin-Cam AI Dashboard", dashboard)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):  
                e_stop_active = not e_stop_active
                if e_stop_active:
                    print("\n🚨 [緊急煞車] 已啟動！")
                else:
                    print("\n🟢 [急停解除] AI 恢復控制！")

            loop_time = time.time() - loop_start
            if loop_time < target_dt:
                time.sleep(target_dt - loop_time)

    except KeyboardInterrupt:
        print("\n\n🚨 偵測到 Ctrl+C！啟動終極煞車程序...")

    finally:
        print("[INFO] 正在切斷馬達動力...")
        for _ in range(5):  # 連續發送 5 次 0.0，確保車子收到！
            cmd_pub.publish(roslibpy.Message({'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}))
            time.sleep(0.05)
        
        cmd_pub.unadvertise() # 正確關閉頻道
        client.terminate()
        cap_rear.release()
        cap_arm.release()
        cv2.destroyAllWindows()
        print(">>> 系統安全關閉完畢 <<<")

if __name__ == "__main__":
    main()
