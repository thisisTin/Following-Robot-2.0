# --- 1. Imports ---
import cv2
from ultralytics import YOLO
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial
from rplidar import RPLidar # Thư viện Lidar

# --- Cổng & Tốc độ ---

LIDAR_PORT = '/dev/ttyUSB1'  # Cổng Lidar (có thể là USB0 hoặc USB1)
SERIAL_PORT = '/dev/ttyUSB0' # Cổng ESP32
BAUD_RATE = 9600             

# --- 2. AI Model Initialization ---
print("Loading AI Models...")
try:
    model = YOLO('yolov8n.pt') 
    print("Models loaded successfully.")
except Exception as e:
    print(f"FATAL: Could not load YOLO model. {e}")
    model = None # Xử lý lỗi nếu không tải được model

# --- 3. Web Server Initialization ---
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_very_secret_key'
socketio = SocketIO(app, async_mode='threading')

# --- 4. Global Variables & Serial Initialization ---
global_frame = None                  # Khung hình stream
robot_state = "MANUAL"                 # Trạng thái mặc định là MANUAL
manual_command = "STOP"              # Lệnh joystick
lock = threading.Lock()              # Khóa luồng
target_person_id = None              # ID của người đang được bám theo
light_state = False                  # Trạng thái đèn

# --- Biến Lidar ---
lidar = None
MIN_SAFE_DISTANCE = 0.5 # (mét) - Ngưỡng phanh an toàn (50cm)
lidar_scan_data = {
    'front_distance': float('inf'), # Khoảng cách phía trước
    'back_distance': float('inf')  # Khoảng cách phía sau
}

# --- CÁC HẰNG SỐ ĐIỀU KHIỂN P-CONTROLLER (PID) ---
# (Bạn CẦN "canh chỉnh" các giá trị này trên robot thật)
TARGET_AREA = 50000       # Diện tích box (pixel^2) mà robot cố gắng duy trì
KP_DISTANCE = 0.004       # Hằng số P cho khoảng cách (Area) (Đã sửa từ 0.003)
KP_TURN = 0.2             # Hằng số P cho việc rẽ (Turn)
MAX_FWD_SPEED = 200       # Tốc độ tiến/lùi tối đa (PWM)
MAX_TURN_SPEED = 160      # Tốc độ rẽ tối đa (PWM)
MIN_MOVE_PWM = 180        # (SỬA LỖI) Ngưỡng PWM tối thiểu để motor chạy

# Serial Communication
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  
    print(f"Serial Port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None # Đặt là None nếu thất bại

# --- 5. Robot Hardware Functions (Đã cấu trúc lại) ---

def clamp(n, minn, maxn):
    """Hàm tiện ích để giới hạn một giá trị trong một khoảng."""
    return max(min(maxn, n), minn)

def set_robot_pwm(left_pwm, right_pwm, intent=""):
    """
    Hàm cấp thấp: Gửi PWM cuối cùng sau khi kiểm tra Lidar và Deadzone.
    """
    global ser, lidar_scan_data, lock, MIN_SAFE_DISTANCE, MIN_MOVE_PWM

    left_pwm = int(left_pwm)
    right_pwm = int(right_pwm)

    # --- 1. LOGIC LIDAR (PHANH AN TOÀN 2 CHIỀU) ---
    current_front_distance = float('inf')
    current_back_distance = float('inf') 
    with lock:
        current_front_distance = lidar_scan_data.get('front_distance', float('inf'))
        current_back_distance = lidar_scan_data.get('back_distance', float('inf')) 

    is_moving_forward = left_pwm > 0 or right_pwm > 0
    is_moving_backward = left_pwm < 0 or right_pwm < 0 

    # Kiểm tra va chạm TIẾN
    if is_moving_forward and current_front_distance < MIN_SAFE_DISTANCE:
        print(f"LIDAR OVERRIDE (FRONT): Obstacle detected at {current_front_distance:.2f}m! Stopping.")
        left_pwm = 0  # Ghi đè
        right_pwm = 0 # Ghi đè
        intent = f"LIDAR_STOP_FWD (was {intent})"
    
    # Kiểm tra va chạm LÙI
    elif is_moving_backward and current_back_distance < MIN_SAFE_DISTANCE:
        print(f"LIDAR OVERRIDE (BACK): Obstacle detected at {current_back_distance:.2f}m! Stopping.")
        left_pwm = 0 # Ghi đè
        right_pwm = 0 # Ghi đè
        intent = f"LIDAR_STOP_BCK (was {intent})"
    
    # --- 2. LOGIC DEADZONE (VÙNG CHẾT MOTOR) ---
    def _boost_pwm(pwm_val):
        """Đẩy PWM nhỏ lên ngưỡng tối thiểu (MIN_MOVE_PWM)"""
        # Nếu PWM dương và nhỏ hơn ngưỡng -> đẩy lên ngưỡng
        if 0 < pwm_val < MIN_MOVE_PWM:
            return MIN_MOVE_PWM
        # Nếu PWM âm và lớn hơn ngưỡng (ví dụ -70 > -180) -> đẩy xuống ngưỡng
        if 0 > pwm_val > -MIN_MOVE_PWM:
            return -MIN_MOVE_PWM
        # Nếu không thì giữ nguyên
        return pwm_val

    left_pwm = int(_boost_pwm(left_pwm))
    right_pwm = int(_boost_pwm(right_pwm))
    
    # Giới hạn PWM cuối cùng
    left_pwm = clamp(left_pwm, -255, 255)
    right_pwm = clamp(right_pwm, -255, 255)

    # --- 3. GỬI LỆNH SERIAL ---
    serial_command = f"MOVE:{left_pwm}:{right_pwm}\n"  
    
    if ser:
        try:
            ser.write(serial_command.encode())
            # Giảm spam log, chỉ in khi di chuyển
            if left_pwm != 0 or right_pwm != 0:
                 print(f"INTENT: {intent} -> EXECUTING: {serial_command.strip()}")  
        except Exception as e:
            print(f"Serial write error: {e}")
    else:
        # Chế độ giả lập
        if left_pwm != 0 or right_pwm != 0:
            print(f"INTENT: {intent} -> SIMULATED: L_PWM:{left_pwm} R_PWM:{right_pwm}")

def execute_robot_move(command, intent=""):
    """ 
    Hàm cấp cao: Dịch lệnh (FORWARD, LEFT...) từ Joystick thành PWM.
    """
    if intent == "": intent = command # Đặt intent mặc định là tên lệnh

    SPEED = 190
    TURN_SPEED = 220
    CURVE_SPEED_SLOW = int(SPEED * 0.5)  
    CURVE_SPEED_FAST = SPEED
    
    cmd_map = {
        "FORWARD": (SPEED, SPEED),
        "LEFT": (-TURN_SPEED, TURN_SPEED),      # Xoay tại chỗ
        "RIGHT": (TURN_SPEED, -TURN_SPEED),     # Xoay tại chỗ
        "BACKWARD": (-SPEED, -SPEED),  
        "FORWARD_LEFT": (CURVE_SPEED_SLOW, CURVE_SPEED_FAST),  # Rẽ cua
        "FORWARD_RIGHT": (CURVE_SPEED_FAST, CURVE_SPEED_SLOW), # Rẽ cua
        "BACKWARD_LEFT": (-CURVE_SPEED_FAST, -CURVE_SPEED_SLOW),
        "BACKWARD_RIGHT": (-CURVE_SPEED_SLOW, -CURVE_SPEED_FAST),
        "STOP": (0, 0)
    }
    
    left_pwm, right_pwm = cmd_map.get(command, (0, 0))
    
    # Gọi hàm gác cổng cấp thấp để thực thi
    set_robot_pwm(left_pwm, right_pwm, intent)

def toggle_light_relay(new_state):
    """Bật/tắt đèn qua Serial."""
    global ser
    
    serial_command = "LIGHT:ON\n" if new_state else "LIGHT:OFF\n"
    print(f"RELAY: Turning light {'ON' if new_state else 'OFF'}")
    
    if ser:
        try:
            ser.write(serial_command.encode())
            print(f"SERIAL SENT: {serial_command.strip()}")
        except Exception as e:
            print(f"Serial write error: {e}")

# --- 6. Main Robot Logic Threads ---

def serial_read_thread():
    """Luồng đọc Serial: Lắng nghe phản hồi từ ESP32."""
    global ser
    if not ser: return # Thoát nếu không kết nối được
    print("Serial reading thread started...")
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # In phản hồi (ví dụ: "ROBOT_PWM:180:180")
                    print(f"ESP32 RESPONSE: {line}")  
            time.sleep(0.01) # Ngăn CPU load 100%
        except Exception as e:
            print(f"Error reading from serial: {e}")
            time.sleep(1) # Chờ 1s nếu bị lỗi

def lidar_logic_thread():
    """Luồng Lidar: Quét 360 độ và tìm vật cản trước/sau."""
    global lidar, lidar_scan_data, lock
    try:
        print("Connecting to Lidar...")
        lidar = RPLidar(LIDAR_PORT)
        print("Lidar connected successfully.")
        
        # Quét liên tục
        for scan in lidar.iter_scans(scan_type='normal', min_len=100):
            front_distance_mm = float('inf')
            back_distance_mm = float('inf') 
            
            for quality, angle, distance in scan:
                # --- Cung phía trước (0-15 & 345-360) ---
                if (0 <= angle <= 15) or (345 <= angle <= 360):
                    if distance > 0: # Bỏ qua điểm đo lỗi
                        if distance < front_distance_mm:
                            front_distance_mm = distance
                
                # --- Cung phía sau (165-195, cung 30 độ) ---
                if (165 <= angle <= 195):
                    if distance > 0:
                        if distance < back_distance_mm:
                            back_distance_mm = distance
            
            # Cập nhật biến toàn cục một cách an toàn
            with lock:
                # Cập nhật khoảng cách trước
                if front_distance_mm == float('inf'):
                    lidar_scan_data['front_distance'] = float('inf') # An toàn (vô cực)
                else:
                    lidar_scan_data['front_distance'] = front_distance_mm / 1000.0 # Đổi sang mét
                
                # Cập nhật khoảng cách sau
                if back_distance_mm == float('inf'):
                    lidar_scan_data['back_distance'] = float('inf')
                else:
                    lidar_scan_data['back_distance'] = back_distance_mm / 1000.0
            
            time.sleep(0.01) # Ngăn CPU load 100%

    except Exception as e:
        print(f"Error connecting or reading Lidar: {e}")
    finally: # Đảm bảo Lidar dừng lại khi luồng bị lỗi
        if lidar: 
            print("Stopping Lidar...")
            lidar.stop()
            lidar.disconnect()

def robot_logic_thread():
    """Luồng chính: Camera, AI, P-Controller, Gửi lệnh Web."""
    global global_frame, robot_state, manual_command, light_state, target_person_id, model

    # Kiểm tra model trước khi bắt đầu
    if model is None:
        print("FATAL: YOLO Model not loaded. Robot logic thread cannot start.")
        return

    # Khởi tạo camera
    cap = cv2.VideoCapture(0)
    # Đặt độ phân giải thấp để tăng FPS
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    FRAME_HEIGHT, FRAME_WIDTH = 480, 640 
    FRAME_CENTER_X = FRAME_WIDTH / 2
    
    prev_frame_time = 0
    print("Robot logic thread started...")
    
    # --- Biến Tối ưu hóa ---
    frame_count = 0
    AI_SKIP_FRAMES = 2    # Chạy AI mỗi 2 khung hình để tăng độ ổn định
    INFO_SKIP_FRAMES = 15 # Gửi thông tin lên web mỗi 15 khung hình
    
    # Biến lưu trữ P-Controller
    last_known_area = 0
    last_known_centerX = FRAME_CENTER_X
    
    jpeg_quality = [int(cv2.IMWRITE_JPEG_QUALITY), 80] # Chất lượng ảnh stream

    while True:
        # Tự động kết nối lại camera nếu bị ngắt
        if not cap.isOpened():
            print("Camera not open. Trying to reconnect...")
            cap.release()
            cap = cv2.VideoCapture(0) 
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            time.sleep(1)
            continue 

        success, image = cap.read()
        if not success:
            print("Camera read failed, skipping frame.")
            time.sleep(1)
            continue 
            
        frame_count += 1
        image = cv2.flip(image, 1) # Lật ngang ảnh
        
        # Đọc biến toàn cục
        with lock:
            current_state = robot_state
            current_manual_cmd = manual_command
            current_target_id = target_person_id

        hud_text = f"STATE: {current_state}"
        hud_color = (0, 0, 255) 
        
        run_ai_this_frame = (frame_count % AI_SKIP_FRAMES == 0)
        boxes_to_send = [] # Danh sách box gửi lên web

        if current_state == "MANUAL":
            hud_color = (255, 0, 0) # Blue
            
            if run_ai_this_frame:
                # Vẫn chạy AI để hiển thị box (dùng tracker tùy chỉnh)
                # <<< THAY ĐỔI (1): SỬ DỤNG TRACKER TÙY CHỈNH TẠI ĐÂY >>>
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.5, tracker="my_tracker.yaml")
                if results[0].boxes and results[0].boxes.id is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        box_id = int(box.id[0])
                        
                        # (THÊM ĐỂ DEBUG) In ra diện tích box
                        box_area = (x2 - x1) * (y2 - y1)
                        if frame_count % INFO_SKIP_FRAMES == 0: # Chỉ in 1 lần / 15 frames
                            print(f"DEBUG (Manual): ID {box_id} Area: {box_area:.0f}")

                        # Sửa lỗi JSON (ép kiểu int)
                        rect_list = [int(x1), int(y1), int(x2), int(y2)] 
                        boxes_to_send.append({'id': box_id, 'rect': rect_list})

                        # Vẽ box màu vàng (chờ click)
                        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        cv2.putText(image, f"ID: {box_id}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Thực thi lệnh joystick
            execute_robot_move(current_manual_cmd, "MANUAL_JOYSTICK")
            
            # Reset P-Controller
            last_known_area = 0
            last_known_centerX = FRAME_CENTER_X


        elif current_state == "FOLLOWING":
            hud_color = (0, 250, 0) # Green
            found_target_this_frame = False # Cờ kiểm tra
            
            if run_ai_this_frame:
                # Chạy AI (dùng tracker tùy chỉnh)
                # <<< THAY ĐỔI (2): SỬ DỤNG TRACKER TÙY CHỈNH TẠI ĐÂY >>>
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.5, tracker="my_tracker.yaml")
                
                if results[0].boxes and results[0].boxes.id is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        box_id = int(box.id[0])

                        rect_list = [int(x1), int(y1), int(x2), int(y2)]
                        boxes_to_send.append({'id': box_id, 'rect': rect_list})
                        
                        # So sánh ID
                        if box_id == current_target_id:
                            found_target_this_frame = True
                            hud_text = f"FOLLOWING ID: {box_id}"
                            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 3) # Box xanh
                            
                            # --- CẬP NHẬT BIẾN P-CONTROLLER ---
                            last_known_centerX = (x1 + x2) / 2
                            last_known_area = (x2 - x1) * (y2 - y1)
                            
                        else:
                            # Box vàng (người khác)
                            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                
                # Nếu không tìm thấy mục tiêu trong khung hình AI này
                if not found_target_this_frame:
                    last_known_area = 0 # Đặt về 0 để robot dừng
            
            # --- LOGIC P-CONTROLLER (Chạy ở MỌI khung hình) ---
            if last_known_area == 0:
                # Đã mất dấu, dừng lại
                set_robot_pwm(0, 0, "STOP (Lost Target)")
                hud_text = f"FOLLOWING (No Target)"
            else:
                # In debug 1 lần / 15 frames
                if frame_count % INFO_SKIP_FRAMES == 0: 
                     print(f"DEBUG (Follow): Area: {last_known_area:.0f} (Target: {TARGET_AREA})")

                # --- Tính toán P (Proportional) ---
                # Lỗi khoảng cách
                error_area = TARGET_AREA - last_known_area
                fwd_speed = KP_DISTANCE * error_area
                fwd_speed = clamp(fwd_speed, -MAX_FWD_SPEED, MAX_FWD_SPEED)
                
                # Lỗi rẽ
                error_turn = FRAME_CENTER_X - last_known_centerX
                turn_speed = KP_TURN * error_turn
                turn_speed = clamp(turn_speed, -MAX_TURN_SPEED, MAX_TURN_SPEED)
                
                # Kết hợp
                left_pwm = fwd_speed + turn_speed
                right_pwm = fwd_speed - turn_speed
                
                set_robot_pwm(left_pwm, right_pwm, "PID_FOLLOW")


        # --- HUD & Frame Update ---
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time) if (new_frame_time - prev_frame_time) > 0 else 0
        prev_frame_time = new_frame_time

        cv2.putText(image, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        cv2.putText(image, f"FPS: {int(fps)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        
        # --- GỬI DỮ LIỆU LÊN WEB ---
        # Gửi thông tin HUD (State, FPS)
        if frame_count % INFO_SKIP_FRAMES == 0:
            with lock:
                current_light_state = light_state  
            socketio.emit('robot_info', {'fps': int(fps), 'state': current_state, 'light': current_light_state})
        
        # Gửi danh sách Box (để vẽ)
        if run_ai_this_frame and len(boxes_to_send) > 0:
            socketio.emit('detected_boxes', {'boxes': boxes_to_send})

        # Cập nhật global_frame cho luồng video
        with lock:
            _, buffer = cv2.imencode('.jpg', image, jpeg_quality)
            global_frame = buffer.tobytes()

# --- 7. Flask HTTP Routes (Giữ nguyên) ---
@app.route('/')
def index():
    """ Phục vụ tệp index.html """
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """ Cung cấp luồng video MJPEG """
    def gen_frames():
        global global_frame
        while True:
            with lock:
                frame_bytes = global_frame
            if frame_bytes is None:
                time.sleep(0.1)
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- 8. Socket.IO Events (Đã cập nhật logic 2 nút) ---
@socketio.on('connect')
def handle_connect():
    """ Khi client kết nối """
    print('Client connected!')
    with lock:
        # Gửi trạng thái hiện tại
        emit('robot_info', {'fps': 0, 'state': robot_state, 'light': light_state})

@socketio.on('robot_command')
def handle_robot_command(data):
    """Chỉ xử lý joystick từ đây"""
    global robot_state, manual_command
    command = data.get('command')
    
    with lock:
        if command.startswith('MANUAL_'):
            # Chỉ nhận lệnh joystick khi ở MANUAL
            if robot_state == "MANUAL":
                manual_command = command.split('_')[1] 

@socketio.on('set_target_id')
def handle_set_target(data):
    """Xử lý khi click vào box"""
    global robot_state, target_person_id
    target_id = data.get('id')
    
    if target_id is not None:
        with lock:
            target_person_id = int(target_id)
            robot_state = "FOLLOWING" # Chuyển sang FOLLOW
            print(f"*** NEW TARGET ACQUIRED: ID {target_person_id} ***")
            current_light_state = light_state
            
        # Gửi cập nhật trạng thái ngay lập tức
        emit('robot_info', {'fps': 0, 'state': "FOLLOWING", 'light': current_light_state}, broadcast=True)

@socketio.on('cancel_target')
def handle_cancel_target():
    """Xử lý khi nhấn "Hủy Theo Dõi" """
    global robot_state, target_person_id
    with lock:
        print(f"*** TARGET CANCELED (was ID {target_person_id}) ***")
        target_person_id = None
        robot_state = "MANUAL" # Chuyển về MANUAL
        current_light_state = light_state
        
    # Gửi cập nhật trạng thái ngay lập tức
    emit('robot_info', {'fps': 0, 'state': "MANUAL", 'light': current_light_state}, broadcast=True)


@socketio.on('toggle_light')
def handle_toggle_light():
    """ Xử lý bật/tắt đèn """
    global light_state
    with lock:
        light_state = not light_state 
        current_light_state = light_state
        current_state = robot_state
    
    toggle_light_relay(current_light_state)  # Gửi lệnh Serial
    
    # Gửi cập nhật trạng thái
    emit('robot_info', {'fps': 0, 'state': current_state, 'light': current_light_state}, broadcast=True)

# --- 9. Start Application ---
if __name__ == '__main__':
    # Kiểm tra model trước khi khởi động
    if model is None:
        print("STOPPING: YOLO model failed to load.")
    else:
        # Khởi động các luồng
        print("Starting Robot Thread...")
        robot_thread = threading.Thread(target=robot_logic_thread, daemon=True)
        robot_thread.start()
        
        if ser: # Chỉ khởi động nếu kết nối serial thành công
            serial_thread = threading.Thread(target=serial_read_thread, daemon=True)
            serial_thread.start()
        
        print("Starting Lidar Thread...")
        lidar_thread = threading.Thread(target=lidar_logic_thread, daemon=True)
        lidar_thread.start()

        # Chạy máy chủ web (ở luồng chính)
        print("Starting Web Server at http://0.0.0.0:5001")
        socketio.run(app, host='0.0.0.0', port=5001, debug=False)