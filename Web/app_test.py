from flask import Flask, render_template_string, Response
import cv2
import numpy as np
from ultralytics import YOLO
import time

# --- (TODO: CHỈNH SỬA CẤU HÌNH CỦA BẠN) ---

# 1. Đường dẫn đến model YOLO của bạn
YOLO_MODEL_PATH = "yolov8n.pt" 

# 2. ID của camera (0 thường là webcam laptop)
CAMERA_ID = 0

# 3. ID của class bạn muốn bám (0 là 'person' trong COCO)
TARGET_CLASS_ID = 0 

# 4. Ngưỡng tin cậy (confidence) để chấp nhận 1 detection
CONF_THRESHOLD = 0.5 

# 5. Kích thước frame (nên để thấp để YOLO chạy nhanh hơn)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2

# 6. Sau bao nhiêu frame TRONG KHI TRACKING thì chạy lại YOLO để hiệu chỉnh?
RE_DETECT_INTERVAL = 30 

# --- (HẾT TODO) ---


# --- Biến toàn cục (QUAN TRỌNG cho Flask) ---
STATE = "SEARCHING"     # Trạng thái hiện tại
tracker = None          # Đối tượng tracker
frame_counter = 0       # Bộ đếm frame
yolo_model = None       # Model YOLO
cap = None              # Đối tượng camera

# --- Khởi tạo Flask App ---
app = Flask(__name__)

# -----------------------------------------------------------------
# --- CÁC HÀM LOGIC (YOLO, PID Sim) ---
# (Giữ nguyên từ file test)
# -----------------------------------------------------------------

def run_yolo_detector(frame, model):
    """
    Chạy YOLO trên 1 frame và trả về 1 bbox (x, y, w, h)
    của mục tiêu tốt nhất (to nhất), hoặc None.
    """
    print("  [YOLO] Đang chạy phát hiện (nặng)...")
    results = model.predict(frame, conf=CONF_THRESHOLD, classes=[TARGET_CLASS_ID], verbose=False)
    
    best_box = None
    max_area = 0
    
    for res in results:
        for box in res.boxes:
            if box.cls == TARGET_CLASS_ID:
                xyxy = box.xyxy[0].cpu().numpy()
                (x1, y1, x2, y2) = (int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3]))
                w, h = x2 - x1, y2 - y1
                x, y = x1, y1
                area = w * h
                
                if area > max_area:
                    max_area = area
                    best_box = (x, y, w, h)
                    
    return best_box

def run_pid_simulation(bbox):
    """(SIMULATE) Tính toán PID và IN ra lệnh PWM giả lập."""
    (x, y, w, h) = (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))
    target_center_x = x + (w // 2)
    error = target_center_x - FRAME_CENTER_X
    
    # Giả lập Kp=0.1
    correction = error * 0.1 
    base_speed = 150
    L_PWM = max(0, min(255, base_speed - correction))
    R_PWM = max(0, min(255, base_speed + correction))
    
    print(f"  [PID SIM] Error: {error:4.0f} -> PWM L/R: {L_PWM:3.0f}/{R_PWM:3.0f}")

def stop_motors_simulation():
    """(SIMULATE) Hàm này dừng robot."""
    print("  [MOTOR SIM] Đang dừng (PWM 0/0)")

# -----------------------------------------------------------------
# --- HÀM GENERATOR VIDEO STREAM (ĐÃ TÍCH HỢP LOGIC) ---
# -----------------------------------------------------------------

def generate_frames():
    global STATE, tracker, frame_counter, yolo_model, cap
    
    print("\n--- Bắt đầu generate_frames ---")
    print(f"Trạng thái ban đầu: {STATE}")
    
    while True:
        # 1. Đọc frame từ camera
        ret, frame = cap.read()
        if not ret:
            print("Lỗi đọc frame, thử đọc lại...")
            time.sleep(0.5)
            continue
        
        # Lật frame (webcam thường bị ngược)
        frame = cv2.flip(frame, 1)
        
        start_time = time.time() # Đo FPS

        # -----------------------------------------------------
        # TRẠNG THÁI 1: TÌM KIẾM (DÙNG YOLO)
        # -----------------------------------------------------
        if STATE == "SEARCHING":
            cv2.putText(frame, "STATE: SEARCHING (YOLO)", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            bbox_yolo = run_yolo_detector(frame, yolo_model)
            
            if bbox_yolo is not None:
                print(f"[YOLO] Đã tìm thấy mục tiêu. Khởi tạo Tracker.")
                
                tracker = cv2.TrackerCSRT_create()
                tracker.init(frame, bbox_yolo)
                
                STATE = "TRACKING"
                frame_counter = 0
                print("  -> Chuyển sang [STATE: TRACKING]")
            else:
                stop_motors_simulation()

        # -----------------------------------------------------
        # TRẠNG THÁI 2: BÁM ĐUỔI (DÙNG TRACKER)
        # -----------------------------------------------------
        elif STATE == "TRACKING":
            frame_counter += 1
            success, bbox_tracker = tracker.update(frame)
            
            if success:
                cv2.putText(frame, f"STATE: TRACKING (Tracker)", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                run_pid_simulation(bbox_tracker)
                
                (x, y, w, h) = (int(bbox_tracker[0]), int(bbox_tracker[1]), int(bbox_tracker[2]), int(bbox_tracker[3]))
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (x + w//2, y + h//2), 5, (0, 255, 0), -1)

                if frame_counter % RE_DETECT_INTERVAL == 0:
                    print(f"  [Correct] Đã đến frame {frame_counter}. Chạy YOLO để hiệu chỉnh...")
                    bbox_yolo = run_yolo_detector(frame, yolo_model)
                    
                    if bbox_yolo is not None:
                        print("  [Correct] YOLO xác nhận lại. Reset tracker.")
                        tracker = cv2.TrackerCSRT_create()
                        tracker.init(frame, bbox_yolo)
                    else:
                        print("  [Correct] YOLO không thấy (lag/mờ), tracker tiếp tục bám.")

            else:
                print(f"[TRACKER] !!! TRACKER ĐÃ MẤT DẤU !!!")
                tracker = None
                STATE = "SEARCHING"
                print("  -> Chuyển về [STATE: SEARCHING]")
                stop_motors_simulation()

        # -----------------------------------------------------
        # HIỂN THỊ FPS VÀ ĐƯỜNG TÂM
        # -----------------------------------------------------
        end_time = time.time()
        fps = 1 / (end_time - start_time + 1e-6)
        cv2.putText(frame, f"FPS: {fps:.1f}", (FRAME_WIDTH - 120, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        cv2.line(frame, (FRAME_CENTER_X, 0), (FRAME_CENTER_X, FRAME_HEIGHT), (255, 255, 0), 1)
        
        # -----------------------------------------------------
        # MÃ HÓA VÀ STREAM FRAME (KHÁC BIỆT CHÍNH)
        # -----------------------------------------------------
        
        # Mã hóa frame thành JPEG
        (flag, encodedImage) = cv2.imencode(".jpg", frame)
        if not flag:
            continue
        
        # Đẩy (yield) frame ra cho trình duyệt
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
              bytearray(encodedImage) + b'\r\n')

# -----------------------------------------------------------------
# --- CÁC ROUTE CỦA FLASK ---
# -----------------------------------------------------------------

@app.route("/")
def index():
    """Trang chủ hiển thị video stream."""
    # Đây là một trang HTML đơn giản
    return render_template_string(
        """
        <html>
        <head>
            <title>Test Logic (YOLO + Tracker)</title>
            <style>
                body { background-color: #111; color: #eee; }
                h1 { text-align: center; }
                img { 
                    display: block; 
                    margin-left: auto; 
                    margin-right: auto; 
                    border: 2px solid #555;
                }
            </style>
        </head>
        <body>
            <h1>Test Logic (YOLO + Tracker)</h1>
            <p style="text-align:center;">
                Xem log PID và Trạng thái trong Terminal
            </p>
            <img src="{{ url_for('video_feed') }}">
        </body>
        </html>
        """
    )

@app.route("/video_feed")
def video_feed():
    """Route cung cấp video stream."""
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

# -----------------------------------------------------------------
# --- KHỞI ĐỘNG APP ---
# -----------------------------------------------------------------

if __name__ == '__main__':
    try:
        # Tải model (chỉ 1 lần)
        print("Đang tải model YOLO, vui lòng chờ...")
        yolo_model = YOLO(YOLO_MODEL_PATH)
        yolo_model.predict(np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3)), verbose=False)
        print("Đã tải xong model YOLO.")
        
        # Mở camera (chỉ 1 lần)
        cap = cv2.VideoCapture(CAMERA_ID)
        if not cap.isOpened():
            raise IOError(f"Không thể mở camera ID {CAMERA_ID}")
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        print(f"Đã mở camera ID {CAMERA_ID} với kích thước {FRAME_WIDTH}x{FRAME_HEIGHT}.")

        # Chạy Flask app
        print("\n--- Mở trình duyệt và truy cập: http://127.0.0.1:5000 ---")
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

    except Exception as e:
        print(f"!!! LỖI KHỞI ĐỘNG: {e}")
    finally:
        # Dọn dẹp khi tắt app
        if cap:
            cap.release()
            print("Đã giải phóng camera.")