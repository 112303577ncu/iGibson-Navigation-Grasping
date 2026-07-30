import cv2
import numpy as np
import math
import time
from stable_baselines3 import PPO

RL_MODEL_PATH = r"C:\Users\user\Downloads\trash_identify.v3i.yolov11 (1)\best_model.zip"

WIDTH, HEIGHT = 600, 600
robot_pos = np.array([100.0, 500.0]) 
robot_angle = 0.0                    
goal_pos = np.array([500.0, 100.0])  

MAX_LINEAR_VEL = 0.2
MAX_ANGULAR_VEL = 0.8
DT = 0.1 

def main():
    global robot_pos, robot_angle
    model = PPO.load(RL_MODEL_PATH)
    cv2.namedWindow("2D RL Robot Simulator")

    while True:
        dx, dy = goal_pos[0] - robot_pos[0], goal_pos[1] - robot_pos[1]
        dist_to_goal = math.hypot(dx, dy)
        angle_to_goal = math.atan2(-dy, dx) - robot_angle
        angle_to_goal = (angle_to_goal + math.pi) % (2 * math.pi) - math.pi
        
        obs = np.zeros(55)
        obs[0:52] = 1.0 # 假設沒障礙物
        obs[52] = np.clip(dist_to_goal / 500.0, 0, 1.0) 
        obs[53] = angle_to_goal / math.pi               
        obs[54] = 0.0                                   

        action, _ = model.predict(obs.astype(np.float32), deterministic=True)
        v = float(action[0]) * MAX_LINEAR_VEL * 50  
        w = float(action[1]) * MAX_ANGULAR_VEL

        robot_angle += w * DT
        robot_pos[0] += v * math.cos(robot_angle) * DT
        robot_pos[1] -= v * math.sin(robot_angle) * DT 

        canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        cv2.circle(canvas, tuple(goal_pos.astype(int)), 10, (0, 255, 0), -1)
        cv2.circle(canvas, tuple(robot_pos.astype(int)), 15, (255, 0, 0), -1)
        
        cv2.imshow("2D RL Robot Simulator", canvas)
        if dist_to_goal < 20: robot_pos = np.array([100.0, 500.0]); time.sleep(1)
        if cv2.waitKey(30) & 0xFF == ord('q'): break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()