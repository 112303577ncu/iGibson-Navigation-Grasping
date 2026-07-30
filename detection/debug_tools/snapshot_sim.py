import cv2
import numpy as np
import math
import time
from stable_baselines3 import PPO

# ========= 1. 載入與路徑設定 =========
RL_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\best_model.zip"

# ========= 2. 模擬環境參數 =========
WIDTH, HEIGHT = 600, 600
CENTER = (WIDTH // 2, HEIGHT // 2)

# 機器人初始狀態
robot_pos = np.array([100.0, 500.0]) # 左下角啟動
robot_angle = 0.0                    # 面向右方 (弧度)
goal_pos = np.array([500.0, 100.0])  # 目標在右上方

# 物理限制 (與實體車同步)
MAX_LINEAR_VEL = 0.2
MAX_ANGULAR_VEL = 0.8
DT = 0.1 # 模擬時間步長

def main():
    global robot_pos, robot_angle
    
    print("[INFO] 正在載入 RL 模型進行 2D 模擬測試...")
    try:
        model = PPO.load(RL_MODEL_PATH)
    except:
        print("❌ 找不到模型檔，請確認路徑是否正確！")
        return

    cv2.namedWindow("2D RL Robot Simulator")

    while True:
        # --- 1. 計算給大腦的 Observation ---
        # 計算距離與夾角
        dx = goal_pos[0] - robot_pos[0]
        dy = goal_pos[1] - robot_pos[1]
        dist_to_goal = math.hypot(dx, dy)
        
        # 計算目標相對於機器人車頭的角度
        angle_to_goal = math.atan2(-dy, dx) - robot_angle
        # 角度正規化到 -pi ~ pi
        angle_to_goal = (angle_to_goal + math.pi) % (2 * math.pi) - math.pi
        
        # 模擬 52 條光達數據 (假設前方沒障礙物，全部給 5.0m) [cite: 2]
        fake_lidar = np.ones(52) * 5.0
        
        # 組裝 Observation (55 維) [cite: 3]
        obs = np.zeros(55)
        obs[0:52] = np.clip(fake_lidar / 5.0, 0, 1.0)
        obs[52] = np.clip(dist_to_goal / 500.0, 0, 1.0) # 距離正規化
        obs[53] = angle_to_goal / math.pi               # 角度正規化 (-1 ~ 1)
        obs[54] = 0.0                                   # 當前線速度

        # --- 2. RL 大腦決策 ---
        action, _ = model.predict(obs.astype(np.float32), deterministic=True)
        
        # action[0] 是線速度, action[1] 是角速度
        v = float(action[0]) * MAX_LINEAR_VEL * 50  # 放大一點在畫面上才看得出移動
        w = float(action[1]) * MAX_ANGULAR_VEL

        # --- 3. 更新物理模擬 (簡單運動學) ---
        robot_angle += w * DT
        robot_pos[0] += v * math.cos(robot_angle) * DT
        robot_pos[1] -= v * math.sin(robot_angle) * DT # OpenCV Y軸向下

        # --- 4. 繪製畫面 ---
        canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        
        # 畫目標點 (綠色)
        cv2.circle(canvas, tuple(goal_pos.astype(int)), 10, (0, 255, 0), -1)
        
        # 畫機器人 (藍色圓形)
        rx, ry = robot_pos.astype(int)
        cv2.circle(canvas, (rx, ry), 15, (255, 0, 0), -1)
        
        # 畫車頭朝向線
        head_x = int(rx + 20 * math.cos(robot_angle))
        head_y = int(ry - 20 * math.sin(robot_angle))
        cv2.line(canvas, (rx, ry), (head_x, head_y), (255, 255, 255), 2)

        # 顯示數值
        cv2.putText(canvas, f"Action V: {action[0]:.2f} W: {action[1]:.2f}", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(canvas, f"Dist: {dist_to_goal:.1f}", (20, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

        cv2.imshow("2D RL Robot Simulator", canvas)

        # 到達目標重置
        if dist_to_goal < 20:
            print("✨ 到達目標！重置位置...")
            robot_pos = np.array([100.0, 500.0])
            time.sleep(1)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()