# --- 1. Imports ---
import cv2
from ultralytics import YOLO
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial
from rplidar import RPLidar

# --- Cổng & Tốc độ ---
LIDAR_PORT = '/dev/ttyUSB0' 
SERIAL_PORT = '/dev/ttyUSB1' 
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
global_frame = None                  
robot_state = "MANUAL"                 
manual_command = "STOP"              
lock = threading.Lock()              
target_person_id = None              
light_state = False                  

# --- Biến Lidar ---
lidar = None
MIN_SAFE_DISTANCE = 0.3# (mét)
lidar_scan_data = {
    'front_distance': float('inf'),
    'back_distance': float('inf') # <<< THAY ĐỔI: Thêm khoảng cách sau
}

# --- CÁC HẰNG SỐ ĐIỀU KHIỂN P-CONTROLLER (PID) ---
# (Bạn CẦN tinh chỉnh các giá trị này)
TARGET_AREA = 50000       # Diện tích box (pixel^2) mà robot cố gắng duy trì
# (KP_DISTANCE đã được cập nhật từ 0.003 -> 0.004 để tăng tốc độ)
KP_DISTANCE = 0.006       # Hằng số P cho khoảng cách (Area)
KP_TURN = 0.1           # Hằng số P cho rẽ (X)

MAX_FWD_SPEED = 222       # Tốc độ tiến/lùi tối đa (PWM)
MAX_TURN_SPEED = 140    # Tốc độ rẽ tối đa (PWM)

MIN_MOVE_PWM = 190       # Ngưỡng PWM tối thiểu để motor chạy

# Serial Communication
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  
    print(f"Serial Port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None 

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

    # --- 1. LOGIC LIDAR (PHANH AN TOÀN) ---
    current_front_distance = float('inf')
    current_back_distance = float('inf') # <<< THAY ĐỔI
    with lock:
        current_front_distance = lidar_scan_data.get('front_distance', float('inf'))
        current_back_distance = lidar_scan_data.get('back_distance', float('inf')) # <<< THAY ĐỔI

    is_moving_forward = left_pwm > 0 or right_pwm > 0
    is_moving_backward = left_pwm < 0 or right_pwm < 0 # <<< THAY ĐỔI

    # Kiểm tra va chạm TIẾN
    if is_moving_forward and current_front_distance < MIN_SAFE_DISTANCE:
        print(f"LIDAR OVERRIDE (FRONT): Obstacle detected at {current_front_distance:.2f}m! Stopping.")
        left_pwm = 0  
        right_pwm = 0 
        intent = f"LIDAR_STOP_FWD (was {intent})"
    
    # <<< THAY ĐỔI: Kiểm tra va chạm LÙI >>>
    elif is_moving_backward and current_back_distance < MIN_SAFE_DISTANCE:
        print(f"LIDAR OVERRIDE (BACK): Obstacle detected at {current_back_distance:.2f}m! Stopping.")
        left_pwm = 0
        right_pwm = 0
        intent = f"LIDAR_STOP_BCK (was {intent})"
    
    # --- 2. LOGIC DEADZONE (VÙNG CHẾT MOTOR) ---
    def _boost_pwm(pwm_val):
        """Đẩy PWM nhỏ lên ngưỡng tối thiểu (MIN_MOVE_PWM)"""
        if 0 < pwm_val < MIN_MOVE_PWM:
            return MIN_MOVE_PWM
        if 0 > pwm_val > -MIN_MOVE_PWM:
            return -MIN_MOVE_PWM
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
            if left_pwm != 0 or right_pwm != 0:
                 print(f"INTENT: {intent} -> EXECUTING: {serial_command.strip()}")  
        except Exception as e:
            print(f"Serial write error: {e}")
    else:
        if left_pwm != 0 or right_pwm != 0:
            print(f"INTENT: {intent} -> SIMULATED: L_PWM:{left_pwm} R_PWM:{right_pwm}")

def execute_robot_move(command, intent=""):
    """ 
    Hàm cấp cao: Dịch lệnh (FORWARD, LEFT...) từ Joystick thành PWM.
    """
    if intent == "": intent = command

    SPEED = 190
    TURN_SPEED = 220
    CURVE_SPEED_SLOW = int(SPEED * 0.5)  
    CURVE_SPEED_FAST = SPEED
    
    cmd_map = {
        "FORWARD": (SPEED, SPEED),
        "LEFT": (-TURN_SPEED, TURN_SPEED),      
        "RIGHT": (TURN_SPEED, -TURN_SPEED),     
        "BACKWARD": (-SPEED, -SPEED),  
        "FORWARD_LEFT": (CURVE_SPEED_SLOW, CURVE_SPEED_FAST),  
        "FORWARD_RIGHT": (CURVE_SPEED_FAST, CURVE_SPEED_SLOW), 
        "BACKWARD_LEFT": (-CURVE_SPEED_FAST, -CURVE_SPEED_SLOW),
        "BACKWARD_RIGHT": (-CURVE_SPEED_SLOW, -CURVE_SPEED_FAST),
        "STOP": (0, 0)
    }
    
    left_pwm, right_pwm = cmd_map.get(command, (0, 0))
    set_robot_pwm(left_pwm, right_pwm, intent)

def toggle_light_relay(new_state):
    global ser
    if new_state:
        serial_command = "LIGHT:ON\n"
        print("RELAY: Turning light ON")
    else:
        serial_command = "LIGHT:OFF\n"
        print("RELAY: Turning light OFF")
    if ser:
        try:
            ser.write(serial_command.encode())
            print(f"SERIAL SENT: {serial_command.strip()}")
        except Exception as e:
            print(f"Serial write error: {e}")

# --- 6. Main Robot Logic Threads ---

def serial_read_thread():
    global ser
    if not ser: return
    print("Serial reading thread started...")
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"ESP32 RESPONSE: {line}")  
            time.sleep(0.01) 
        except Exception as e:
            print(f"Error reading from serial: {e}")
            time.sleep(1)

def lidar_logic_thread():
    global lidar, lidar_scan_data, lock
    try:
        print("Connecting to Lidar...")
        lidar = RPLidar(LIDAR_PORT)
        print("Lidar connected successfully.")
        
        for scan in lidar.iter_scans(scan_type='normal', min_len=100):
            front_distance_mm = float('inf')
            back_distance_mm = float('inf') # <<< THAY ĐỔI
            
            for quality, angle, distance in scan:
                # --- Cung phía trước (0-15 & 345-360) ---
                if (0 <= angle <= 15) or (345 <= angle <= 360):
                    if distance > 0: 
                        if distance < front_distance_mm:
                            front_distance_mm = distance
                
                # <<< THAY ĐỔI: Cung phía sau (165-195) >>>
                if (165 <= angle <= 195):
                    if distance > 0:
                        if distance < back_distance_mm:
                            back_distance_mm = distance
            
            with lock:
                # Cập nhật khoảng cách trước
                if front_distance_mm == float('inf'):
                    lidar_scan_data['front_distance'] = float('inf') 
                else:
                    lidar_scan_data['front_distance'] = front_distance_mm / 1000.0
                
                # <<< THAY ĐỔI: Cập nhật khoảng cách sau >>>
                if back_distance_mm == float('inf'):
                    lidar_scan_data['back_distance'] = float('inf')
                else:
                    lidar_scan_data['back_distance'] = back_distance_mm / 1000.0
            
            time.sleep(0.01) 

    except Exception as e:
        print(f"Error connecting or reading Lidar: {e}")
    finally: # Đảm bảo Lidar dừng lại khi luồng bị lỗi
        if lidar: 
            lidar.stop()
            lidar.disconnect()

def robot_logic_thread():
    global global_frame, robot_state, manual_command, light_state, target_person_id, model

    # Kiểm tra model trước khi bắt đầu
    if model is None:
        print("FATAL: YOLO Model not loaded. Robot logic thread cannot start.")
        return

    # Khởi tạo camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    FRAME_HEIGHT, FRAME_WIDTH = 480, 640 
    FRAME_CENTER_X = FRAME_WIDTH / 2
    
    prev_frame_time = 0
    print("Robot logic thread started...")
    
    frame_count = 0
    # (Đã sửa lỗi mất dấu)
    AI_SKIP_FRAMES = 1  # Chạy AI mỗi 2 khung hình để tăng độ ổn định
    INFO_SKIP_FRAMES = 15 
    
    # Biến lưu trữ P-Controller
    last_known_area = 0
    last_known_centerX = FRAME_CENTER_X
    
    jpeg_quality = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

    while True:
        # Kiểm tra camera (tự kết nối lại)
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
        image = cv2.flip(image, 1) 
        
        with lock:
            current_state = robot_state
            current_manual_cmd = manual_command
            current_target_id = target_person_id

        hud_text = f"STATE: {current_state}"
        hud_color = (0, 0, 255) 
        
        run_ai_this_frame = (frame_count % AI_SKIP_FRAMES == 0)
        boxes_to_send = []

        if current_state == "MANUAL":
            hud_color = (255, 0, 0) # Blue
            
            if run_ai_this_frame:
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.3, tracker="botsort.yaml")
                if results[0].boxes and results[0].boxes.id is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        box_id = int(box.id[0])
                        
                        # (THÊM ĐỂ DEBUG) Tính và in ra diện tích box
                        box_area = (x2 - x1) * (y2 - y1)
                        print(f"DEBUG (Manual): ID {box_id} Area: {box_area:.0f}")

                        rect_list = [int(x1), int(y1), int(x2), int(y2)] # Sửa lỗi JSON
                        boxes_to_send.append({'id': box_id, 'rect': rect_list})

                        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        cv2.putText(image, f"ID: {box_id}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            execute_robot_move(current_manual_cmd, "MANUAL_JOYSTICK")
            
            last_known_area = 0
            last_known_centerX = FRAME_CENTER_X


        elif current_state == "FOLLOWING":
            hud_color = (0, 250, 0) # Green
            found_target_this_frame = False 
            
            if run_ai_this_frame:
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.3, tracker="botsort.yaml")
                
                if results[0].boxes and results[0].boxes.id is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        box_id = int(box.id[0])

                        rect_list = [int(x1), int(y1), int(x2), int(y2)] # Sửa lỗi JSON
                        boxes_to_send.append({'id': box_id, 'rect': rect_list})
                        
                        if box_id == current_target_id:
                            found_target_this_frame = True
                            hud_text = f"FOLLOWING ID: {box_id}"
                            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                            
                            # --- CẬP NHẬT BIẾN P-CONTROLLER ---
                            last_known_centerX = (x1 + x2) / 2
                            last_known_area = (x2 - x1) * (y2 - y1)
                                
                        else:
                            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                
                if not found_target_this_frame:
                    last_known_area = 0 
            
            # --- LOGIC P-CONTROLLER (Chạy ở MỌI khung hình) ---
            if last_known_area == 0:
                set_robot_pwm(0, 0, "STOP (Lost Target)")
                hud_text = f"FOLLOWING (No Target)"
            else:
                # (THÊM ĐỂ DEBUG)
                if frame_count % INFO_SKIP_FRAMES == 0: # Chỉ in 1 lần / 15 frames
                     print(f"DEBUG (Follow): Area: {last_known_area:.0f} (Target: {TARGET_AREA})")

                error_area = TARGET_AREA - last_known_area
                fwd_speed = KP_DISTANCE * error_area
                fwd_speed = clamp(fwd_speed, -MAX_FWD_SPEED, MAX_FWD_SPEED)
                
                error_turn = FRAME_CENTER_X - last_known_centerX
                turn_speed = KP_TURN * error_turn
                turn_speed = clamp(turn_speed, -MAX_TURN_SPEED, MAX_TURN_SPEED)
                
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
        if frame_count % INFO_SKIP_FRAMES == 0:
            with lock:
                current_light_state = light_state  
            socketio.emit('robot_info', {'fps': int(fps), 'state': current_state, 'light': current_light_state})
        
        # Chỉ gửi box nếu AI vừa chạy VÀ có box
        if run_ai_this_frame and len(boxes_to_send) > 0:
            socketio.emit('detected_boxes', {'boxes': boxes_to_send})

        with lock:
            _, buffer = cv2.imencode('.jpg', image, jpeg_quality)
            global_frame = buffer.tobytes()

# --- 7. Flask HTTP Routes (Giữ nguyên) ---
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
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

# --- 8. Socket.IO Events (Giữ nguyên) ---
@socketio.on('connect')
def handle_connect():
    print('Client connected!')
    with lock:
        emit('robot_info', {'fps': 0, 'state': robot_state, 'light': light_state})

@socketio.on('robot_command')
def handle_robot_command(data):
    """Chỉ xử lý joystick từ đây"""
    global robot_state, manual_command
    command = data.get('command')
    
    with lock:
        if command.startswith('MANUAL_'):
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
            robot_state = "FOLLOWING" 
            print(f"*** NEW TARGET ACQUIRED: ID {target_person_id} ***")
            current_light_state = light_state
            
        emit('robot_info', {'fps': 0, 'state': "FOLLOWING", 'light': current_light_state}, broadcast=True)

@socketio.on('cancel_target')
def handle_cancel_target():
    """Xử lý khi nhấn "Hủy Theo Dõi" """
    global robot_state, target_person_id
    with lock:
        print(f"*** TARGET CANCELED (was ID {target_person_id}) ***")
        target_person_id = None
        robot_state = "MANUAL" 
        current_light_state = light_state
        
    emit('robot_info', {'fps': 0, 'state': "MANUAL", 'light': current_light_state}, broadcast=True)


@socketio.on('toggle_light')
def handle_toggle_light():
    global light_state
    with lock:
        light_state = not light_state 
        current_light_state = light_state
        current_state = robot_state
    toggle_light_relay(current_light_state)  
    emit('robot_info', {'fps': 0, 'state': current_state, 'light': current_light_state}, broadcast=True)

# --- 9. Start Application ---
if __name__ == '__main__':
    # Kiểm tra model trước khi khởi động
    if model is None:
        print("STOPPING: YOLO model failed to load.")
    else:
        print("Starting Robot Thread...")
        robot_thread = threading.Thread(target=robot_logic_thread, daemon=True)
        robot_thread.start()
        
        if ser:
            serial_thread = threading.Thread(target=serial_read_thread, daemon=True)
            serial_thread.start()
        
        print("Starting Lidar Thread...")
        lidar_thread = threading.Thread(target=lidar_logic_thread, daemon=True)
        lidar_thread.start()

        print("Starting Web Server at http://0.0.0.0:5001")
        socketio.run(app, host='0.0.0.0', port=5001, debug=False)