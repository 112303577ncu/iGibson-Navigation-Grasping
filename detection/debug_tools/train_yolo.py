from ultralytics import YOLO

def main():
    model = YOLO("yolo11n.pt")  # 先用 nano，比較快

    model.train(
        data=r"C:\Users\user\Downloads\arm_cam_trash.v1i.yolov11\data.yaml",   # Roboflow 下載下來的 yaml
        epochs=1000,
        imgsz=640,
        batch=8,
        workers=0,
        device=0  # 有GPU就用0，沒GPU可改成 cpu
    )

if __name__ == "__main__":
    main()