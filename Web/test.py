import cv2
from ultralytics import YOLO

# --- Initialize ---
model = YOLO('yolov8n.pt') 
# Open Webcam
cap = cv2.VideoCapture(0)

# Capping
success, img = cap.read()
if not success:
    print("Cannot connect cam")
    exit()

FRAME_HEIGHT, FRAME_WIDTH, _ = img.shape
ZONE_LEFT = FRAME_WIDTH * 0.45
ZONE_RIGHT = FRAME_WIDTH * 0.65

print(f"Frame: {FRAME_WIDTH}x{FRAME_HEIGHT} | Model: YOLOv8-Nano")

# --- Main Loop ---
while cap.isOpened():
    success, image = cap.read()
    if not success:
        break

    # Flip Cam
    image = cv2.flip(image, 1)

    # --- YOLOv8n Running ---
    # Dùng model.track() để theo dõi ID của đối tượng
    # persist=True giúp giữ lại ID giữa các frame
    # Dòng đã tối ưu
    results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.4)
    # classes=[0] nghĩa là CHỈ TÌM NGƯỜI ('person' là class 0)

    found_person = False

    # --- 4. Phân tích kết quả ---
    if results[0].boxes:
        # results[0].boxes chứa các hộp tìm thấy
        for box in results[0].boxes:
            found_person = True
            
            # Lấy tọa độ hộp (xyxy = xmin, ymin, xmax, ymax)
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            
            # Lấy ID của người (để theo dõi)
            track_id = -1
            if box.id is not None:
                track_id = int(box.id[0])

            # Vẽ hộp và ID
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image, f"Person ID: {track_id}", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # --- 5. Ra quyết định (Logic 3 vùng) ---
            centerX = (x1 + x2) / 2
            
            if centerX < ZONE_LEFT:
                print(f"ID {track_id}: QUYẾT ĐỊNH: RẼ TRÁI")
            elif centerX > ZONE_RIGHT:
                print(f"ID {track_id}: QUYẾT ĐỊNH: RẼ PHẢI")
            else:
                print(f"ID {track_id}: QUYẾT ĐỊNH: ĐI THẲNG")

            # Chỉ bám theo người đầu tiên tìm thấy
            break 

    if not found_person:
        print("QUYẾT ĐỊNH: DỪNG")

    # Vẽ vạch 3 vùng (để debug)
    cv2.line(image, (int(ZONE_LEFT), 0), (int(ZONE_LEFT), FRAME_HEIGHT), (255, 0, 0), 2)
    cv2.line(image, (int(ZONE_RIGHT), 0), (int(ZONE_RIGHT), FRAME_HEIGHT), (255, 0, 0), 2)

    # Hiển thị kết quả
    cv2.imshow('YOLO v8 nano', image)
    
    # Nhấn 'q' để thoát
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()