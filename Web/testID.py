import cv2
import numpy as np

# --- Các hằng số ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2

# --- Biến quản lý trạng thái toàn cục ---
STATE = "SEARCHING"
current_bbox = None # (x, y, w, h)

# -----------------------------------------------------------------
# HÀM GIẢ LẬP (Sẽ được gọi bởi logic)
# -----------------------------------------------------------------

def run_pid_control(bbox):
    """
    HÀM GIẢ LẬP PID:
    Chỉ tính toán độ lệch (error) và in ra, không điều khiển motor.
    """
    (x, y, w, h) = bbox
    target_center_x = x + (w // 2)
    
    # Tính độ lệch (error)
    error = target_center_x - FRAME_CENTER_X
    
    print(f"  [PID SIM] Đang bám đuổi. Error = {error}")
    # (Trong code thật, bạn sẽ gọi pid.update(error) và ra lệnh PWM ở đây)

# -----------------------------------------------------------------
# HÀM CALLBACK (Xử lý tương tác)
# -----------------------------------------------------------------

def on_mouse_click(event, x, y, flags, param):
    """
    HÀM GIẢ LẬP YOLO DETECTOR:
    Khi click chuột, chúng ta giả vờ là YOLO vừa tìm thấy 1 mục tiêu
    tại vị trí click.
    """
    global STATE, current_bbox
    
    # Chỉ hành động khi click chuột trái
    if event == cv2.EVENT_LBUTTONDOWN:
        if STATE == "SEARCHING":
            print("\n[YOLO SIM] Click chuột! Giả lập YOLO tìm thấy mục tiêu.")
            # Tạo 1 bbox giả lập quanh vị trí click
            w, h = 80, 80 # Kích thước bbox giả
            x_tl, y_tl = x - (w // 2), y - (h // 2)
            current_bbox = (x_tl, y_tl, w, h)
            
            # -> Chuyển trạng thái
            STATE = "TRACKING"
            print("  -> Chuyển sang [STATE: TRACKING]")
            
        elif STATE == "TRACKING":
            # Nếu đang tracking mà click, giả lập là YOLO "hiệu chỉnh"
            print("\n[YOLO SIM] Click chuột! Giả lập YOLO hiệu chỉnh lại vị trí.")
            w, h = 80, 80
            x_tl, y_tl = x - (w // 2), y - (h // 2)
            current_bbox = (x_tl, y_tl, w, h)
            print(f"  [Correct] Tracker được reset về vị trí mới: {current_bbox}")


# --- Cửa sổ và vòng lặp chính ---
cv2.namedWindow("Test Sandbox")
cv2.setMouseCallback("Test Sandbox", on_mouse_click)

print("--- Bắt đầu Sandbox Logic ---")
print("  - CLICK chuột để giả lập [YOLO tìm thấy mục tiêu].")
print("  - Bấm 'l' (lose) để giả lập [Tracker mất dấu].")
print("  - Bấm 'q' (quit) để thoát.")

while True:
    # 1. Tạo một frame camera giả (ảnh đen)
    frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)

    # 2. Xử lý logic dựa trên trạng thái (STATE)
    
    if STATE == "SEARCHING":
        # Ở trạng thái tìm kiếm, chỉ hiển thị text
        cv2.putText(frame, "STATE: SEARCHING", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, "Click chuot de gia lap YOLO", (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

    elif STATE == "TRACKING":
        # Ở trạng thái bám đuổi
        cv2.putText(frame, "STATE: TRACKING", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, "Bam 'l' de gia lap mat dau", (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        
        if current_bbox:
            # Vẽ bbox đang được "track"
            (x, y, w, h) = current_bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # -> Gọi PID (giả lập)
            run_pid_control(current_bbox)
    
    # 3. Hiển thị frame
    cv2.imshow("Test Sandbox", frame)

    # 4. Chờ phím bấm
    key = cv2.waitKey(30) & 0xFF

    if key == ord('q'):
        print("\n--- Thoát Sandbox ---")
        break
    
    if key == ord('l'):
        # Bấm 'l' để giả lập Tracker mất dấu
        if STATE == "TRACKING":
            print("\n[TRACKER SIM] Bam 'l'! Gia lap Tracker mat dau.")
            current_bbox = None
            STATE = "SEARCHING"
            print("  -> Chuyển về [STATE: SEARCHING]")

cv2.destroyAllWindows()