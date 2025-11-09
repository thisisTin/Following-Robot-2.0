from ultralytics import YOLO

try:
    print("Đang tải mô hình YOLOv8n...")
    model = YOLO('yolov8n.pt') 
    
    print("Bắt đầu export trực tiếp sang định dạng TensorFlow.js (tfjs)...")
    
    # Export thẳng ra định dạng web (sẽ tạo thư mục yolov8n_web_model)
    # imgsz=640 để đảm bảo kích thước input là 640x640
    model.export(format='tfjs', imgsz=640, optimize=True) 
    
    print("==================================================")
    print("EXPORT THÀNH CÔNG!")
    print("Kiểm tra thư mục 'yolov8n_web_model' mới được tạo.")
    print("Bên trong sẽ có file 'model.json' và các file '.bin'.")
    print("==================================================")

except ImportError as e:
    print(f"LỖI: {e}")
    print("Vui lòng chạy lệnh sau để cài đặt các thư viện bị thiếu:")
    print("pip install onnx2tf tf_keras tensorflowjs")
except Exception as e:
    print(f"Đã xảy ra lỗi trong quá trình export: {e}")


