import os,json,time
from ultralytics import YOLO
import torch
torch.set_num_threads(1)
folder="/home/jetson/rear_camera_audit_20261017"
model=YOLO("/home/jetson/Documents/deploy_jetson2/detection/models/best.pt")
items=[]
for i in [0,32,64,96,128,164]:
 name="frame_%03d.jpg"%i
 result=model.predict(source=os.path.join(folder,name),imgsz=320,conf=.25,device="cpu",verbose=False)[0]
 boxes=[]
 for b in result.boxes:
  boxes.append({"class":model.names[int(b.cls[0].item())],"confidence":float(b.conf[0].item()),"xyxy":b.xyxy[0].tolist()})
 items.append({"file":name,"detections":boxes})
json.dump({"ok":True,"classes":model.names,"items":items,"scope":"RGB snapshots only; paper collection box is not a trained class; no field detection recall claimed"},open("/tmp/rear_yolo_probe.json","w"),indent=2)
