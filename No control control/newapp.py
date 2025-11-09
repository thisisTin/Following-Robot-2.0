print("--- Script starting ---")

# 1. Tắt tự động kiểm tra (PHẢI LÀM TRƯỚC)
import ultralytics
ultralytics.checks.AUTOUPDATE = False

# 2. Bây giờ mới import YOLO
from ultralytics import YOLO
print("--- YOLO imported ---")

# 3. Tải model của bạn (ví dụ: yolov8n.pt)
model = YOLO('yolov8n.pt') 

# 4. Chạy export
try:
    print("--- Starting export to tfjs... ---")
    model.export(format='tfjs')
    print("--- Export successful! ---")
except Exception as e:
    print(f"--- EXPORT FAILED ---")
    print(f"Error: {e}")

print("--- Script finished ---")
