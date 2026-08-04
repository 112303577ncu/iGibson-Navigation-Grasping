"""前鏡頭即時辨識 sugarbox。按 q 離開。"""
from pathlib import Path
import cv2
from ultralytics import YOLO

model = YOLO(str(Path(__file__).parent / 'best.pt'))
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

while cap.isOpened():
    ok, frame = cap.read()
    if not ok:
        break
    cv2.imshow('sugarbox', model.predict(frame, conf=0.25, verbose=False)[0].plot())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
