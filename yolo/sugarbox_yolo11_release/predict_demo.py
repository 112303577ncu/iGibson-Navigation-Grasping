import sys
from pathlib import Path
from ultralytics import YOLO

def main():
    model_path = Path(__file__).parent / 'best.pt'
    if not model_path.exists():
        print(f'Error: {model_path} not found!')
        return

    model = YOLO(str(model_path))
    print('sugarbox YOLOv11 model loaded successfully!')
    print('Class names:', model.names)

    if len(sys.argv) > 1:
        img_path = sys.argv[1]
        print(f'Running inference on: {img_path}')
        results = model.predict(source=img_path, conf=0.25, save=True)
        print('Inference complete! Check output folder.')
    else:
        print('Usage: python predict_demo.py <path_to_image_or_video>')

if __name__ == '__main__':
    main()
