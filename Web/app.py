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
LIDAR_PORT = '/dev/ttyUSB1'  # Cổng Lidar (đã đổi)
SERIAL_PORT = '/dev/ttyUSB0' # Cổng ESP32 (đã đổi)
BAUD_RATE = 9600             

# --- 2. AI Model Initialization ---
print("Loading AI Models...")
try:
    model = YOLO('yolov8n.pt') 
    print("Models loaded successfully.")
except Exception as e:
    print(f"FATAL: Could not load YOLO model. {e}")
    model = None

# --- 3. Web Server Initialization ---
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_very_secret_key'
socketio = SocketIO(app, async_mode='threading')

# --- 4. Global Variables & Serial Initialization ---
global_frame = None
# <<< THAY ĐỔI: Trạng thái mặc định là IDLE >>>
robot_state = "IDLE"                 
manual_command = "STOP"
lock = threading.Lock()
target_person_id = None # ID của người đang được bám theo
light_state = False

# --- Biến Lidar (Giữ nguyên) ---
lidar = None
MIN_SAFE_DISTANCE = 0.5 
lidar_scan_data = {
    'front_distance': float('inf'),
    'back_distance': float('inf')
}

# --- HẰNG SỐ P-CONTROLLER (Giữ nguyên) ---
TARGET_AREA = 50000
KP_DISTANCE = 0.004
KP_TURN = 0.2
MAX_FWD_SPEED = 200
MAX_TURN_SPEED = 160
MIN_MOVE_PWM = 180

# Serial Communication (Giữ nguyên)
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  
    print(f"Serial Port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None

# --- 5. Robot Hardware Functions (Giữ nguyên) ---
# (Các hàm clamp, set_robot_pwm, execute_robot_move, toggle_light_relay
# được giữ nguyên y hệt như file app.py mới của bạn)

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def set_robot_pwm(left_pwm, right_pwm, intent=""):
    global ser, lidar_scan_data, lock, MIN_SAFE_DISTANCE, MIN_MOVE_PWM
    left_pwm = int(left_pwm)
    right_pwm = int(right_pwm)

    # --- 1. LOGIC LIDAR (Giữ nguyên) ---
    current_front_distance = float('inf')
    current_back_distance = float('inf') 
    with lock:
        current_front_distance = lidar_scan_data.get('front_distance', float('inf'))
        current_back_distance = lidar_scan_data.get('back_distance', float('inf')) 

    is_moving_forward = left_pwm > 0 or right_pwm > 0
    is_moving_backward = left_pwm < 0 or right_pwm < 0 

    if is_moving_forward and current_front_distance < MIN_SAFE_DISTANCE:
        print(f"LIDAR OVERRIDE (FRONT): Obstacle detected at {current_front_distance:.2f}m! Stopping.")
        left_pwm = 0
        right_pwm = 0
        intent = f"LIDAR_STOP_FWD (was {intent})"
    elif is_moving_backward and current_back_distance < MIN_SAFE_DISTANCE:
        print(f"LIDAR OVERRIDE (BACK): Obstacle detected at {current_back_distance:.2f}m! Stopping.")
        left_pwm = 0
        right_pwm = 0
        intent = f"LIDAR_STOP_BCK (was {intent})"
    
    # --- 2. LOGIC DEADZONE (Giữ nguyên) ---
    def _boost_pwm(pwm_val):
        if 0 < pwm_val < MIN_MOVE_PWM:
            return MIN_MOVE_PWM
        if 0 > pwm_val > -MIN_MOVE_PWM:
            return -MIN_MOVE_PWM
        return pwm_val

    left_pwm = int(_boost_pwm(left_pwm))
    right_pwm = int(_boost_pwm(right_pwm))
    left_pwm = clamp(left_pwm, -255, 255)
    right_pwm = clamp(right_pwm, -255, 255)

    # --- 3. GỬI LỆNH SERIAL (Giữ nguyên) ---
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
    if intent == "": intent = command
    SPEED = 190
    TURN_SPEED = 220
    CURVE_SPEED_SLOW = int(SPEED * 0.5)  
    CURVE_SPEED_FAST = SPEED
    cmd_map = {
        "FORWARD": (SPEED, SPEED), "LEFT": (-TURN_SPEED, TURN_SPEED),
        "RIGHT": (TURN_SPEED, -TURN_SPEED), "BACKWARD": (-SPEED, -SPEED),
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
    serial_command = "LIGHT:ON\n" if new_state else "LIGHT:OFF\n"
    print(f"RELAY: Turning light {'ON' if new_state else 'OFF'}")
    if ser:
        try:
            ser.write(serial_command.encode())
            print(f"SERIAL SENT: {serial_command.strip()}")
        except Exception as e:
            print(f"Serial write error: {e}")

# --- 6. Main Robot Logic Threads ---

# (Luồng serial_read_thread và lidar_logic_thread giữ nguyên)
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
            back_distance_mm = float('inf') 
            for quality, angle, distance in scan:
                if (0 <= angle <= 15) or (345 <= angle <= 360):
                    if distance > 0 and distance < front_distance_mm:
                        front_distance_mm = distance
                if (165 <= angle <= 195):
                    if distance > 0 and distance < back_distance_mm:
                        back_distance_mm = distance
            with lock:
                lidar_scan_data['front_distance'] = front_distance_mm / 1000.0 if front_distance_mm != float('inf') else float('inf')
                lidar_scan_data['back_distance'] = back_distance_mm / 1000.0 if back_distance_mm != float('inf') else float('inf')
            time.sleep(0.01)
    except Exception as e:
        print(f"Error connecting or reading Lidar: {e}")
    finally:
        if lidar: 
            print("Stopping Lidar...")
            lidar.stop()
            lidar.disconnect()

# <<< THAY ĐỔI LỚN: Cấu trúc lại luồng robot_logic_thread >>>
def robot_logic_thread():
    global global_frame, robot_state, manual_command, light_state, target_person_id, model

    if model is None:
        print("FATAL: YOLO Model not loaded. Robot logic thread cannot start.")
        return

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    
    FRAME_HEIGHT, FRAME_WIDTH = 360, 480
    FRAME_CENTER_X = FRAME_WIDTH / 2
    
    prev_frame_time = 0
    print("Robot logic thread started...")
    
    frame_count = 0
    AI_SKIP_FRAMES = 2
    INFO_SKIP_FRAMES = 15
    
    last_known_area = 0
    last_known_centerX = FRAME_CENTER_X
    
    jpeg_quality = [int(cv2.IMWRITE_JPEG_QUALITY), 50]

    while True:
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
            current_target_id = target_person_id # Đọc ID mục tiêu

        hud_text = f"STATE: {current_state}"
        hud_color = (0, 0, 255) 
        
        run_ai_this_frame = False # Tắt AI theo mặc định
        boxes_to_send = []

        # --- State Machine (Đã cấu trúc lại) ---

        if current_state == "IDLE":
            hud_color = (0, 255, 255) # Yellow
            set_robot_pwm(0, 0, "IDLE_STOP")
            last_known_area = 0
            # Reset target ID khi quay về IDLE
            with lock:
                target_person_id = None
        
        elif current_state == "MANUAL":
            hud_color = (255, 0, 0) # Blue
            # Chỉ thực thi lệnh joystick, KHÔNG chạy AI
            execute_robot_move(current_manual_cmd, "MANUAL_JOYSTICK")
            last_known_area = 0
            # Reset target ID khi vào MANUAL
            with lock:
                target_person_id = None

        elif current_state == "FOLLOWING":
            hud_color = (0, 250, 0) # Green
            run_ai_this_frame = (frame_count % AI_SKIP_FRAMES == 0)
            found_target_this_frame = False
            
            if run_ai_this_frame:
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.3, tracker="my_tracker.yaml")
                
                locked_on_target_box = None # Box của mục tiêu đã khóa
                other_boxes = [] # Box của người khác

                if results[0].boxes and results[0].boxes.id is not None:
                    for box in results[0].boxes:
                        box_id = int(box.id[0])
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        rect_list = [int(x1), int(y1), int(x2), int(y2)]
                        box_data = {'id': box_id, 'rect': rect_list}
                        
                        # --- Logic Auto-Lock ---
                        if current_target_id is None:
                            # Chưa có mục tiêu -> Khóa vào người đầu tiên thấy
                            locked_on_target_box = box
                            with lock:
                                target_person_id = box_id # Ghi lại ID đã khóa
                                current_target_id = box_id # Cập nhật local
                            print(f"*** NEW TARGET ACQUIRED (Auto-Lock): ID {target_person_id} ***")
                        
                        elif box_id == current_target_id:
                            # Đã có mục tiêu -> Tìm đúng ID đó
                            locked_on_target_box = box
                        
                        else:
                            # Đây là người khác
                            other_boxes.append(box_data)

                # --- Xử lý mục tiêu đã khóa (nếu tìm thấy) ---
                if locked_on_target_box is not None:
                    found_target_this_frame = True
                    x1, y1, x2, y2 = locked_on_target_box.xyxy[0].cpu().numpy().astype(int)
                    
                    # Cập nhật P-Controller
                    last_known_centerX = (x1 + x2) / 2
                    last_known_area = (x2 - x1) * (y2 - y1)
                    
                    # Thêm box mục tiêu vào danh sách gửi
                    rect_list = [int(x1), int(y1), int(x2), int(y2)]
                    boxes_to_send.append({'id': current_target_id, 'rect': rect_list})
                    # Thêm các box khác
                    boxes_to_send.extend(other_boxes)
                    
                    # Vẽ box xanh cho mục tiêu
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    hud_text = f"FOLLOWING ID: {current_target_id}"

                else: # Không tìm thấy mục tiêu (kể cả khi AI chạy)
                    found_target_this_frame = False
                    boxes_to_send.extend(other_boxes) # Gửi các box khác
                
                # Vẽ các box màu vàng (người khác)
                for box_data in other_boxes:
                    r = box_data['rect']
                    cv2.rectangle(image, (r[0], r[1]), (r[2], r[3]), (0, 255, 255), 2)


            if not found_target_this_frame and run_ai_this_frame:
                last_known_area = 0 # Reset P-controller nếu AI chạy và không thấy
            
            # --- LOGIC P-CONTROLLER (Chạy ở MỌI khung hình) ---
            if last_known_area == 0:
                set_robot_pwm(0, 0, "STOP (Lost Target)")
                hud_text = f"FOLLOWING (No Target)"
            else:
                # (Logic P-Controller giữ nguyên)
                error_area = TARGET_AREA - last_known_area
                fwd_speed = KP_DISTANCE * error_area
                fwd_speed = clamp(fwd_speed, -MAX_FWD_SPEED, MAX_FWD_SPEED)
                
                error_turn = FRAME_CENTER_X - last_known_centerX
                turn_speed = KP_TURN * error_turn
                turn_speed = clamp(turn_speed, -MAX_TURN_SPEED, MAX_TURN_SPEED)
                
                left_pwm = fwd_speed + turn_speed
                right_pwm = fwd_speed - turn_speed
                
                set_robot_pwm(left_pwm, right_pwm, "PID_FOLLOW")


        # --- HUD & Frame Update (Giữ nguyên) ---
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time) if (new_frame_time - prev_frame_time) > 0 else 0
        prev_frame_time = new_frame_time

        cv2.putText(image, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        cv2.putText(image, f"FPS: {int(fps)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        
        # --- GỬI DỮ LIỆU LÊN WEB ---
        if frame_count % INFO_SKIP_FRAMES == 0:
            with lock:
                current_light_state = light_state
                # <<< THAY ĐỔI: Gửi cả target_id lên web >>>
                current_target_id_for_web = target_person_id
            socketio.emit('robot_info', {
                'fps': int(fps), 
                'state': current_state, 
                'light': current_light_state,
                'target_id': current_target_id_for_web # Gửi ID mục tiêu
            })
        
        # Chỉ gửi box nếu AI vừa chạy (chỉ ở chế độ FOLLOWING)
        if run_ai_this_frame and len(boxes_to_send) > 0:
            socketio.emit('detected_boxes', {'boxes': boxes_to_send})

        with lock:
            _, buffer = cv2.imencode('.jpg', image, jpeg_quality)
            global_frame = buffer.tobytes()

# --- 7. Flask HTTP Routes (Giữ nguyên) ---
@app.route('/')
def index():
    return render_template('index2.html')

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

# --- 8. Socket.IO Events (Đã cập nhật logic 3 nút) ---
@socketio.on('connect')
def handle_connect():
    print('Client connected!')
    with lock:
        # Gửi trạng thái hiện tại khi kết nối
        emit('robot_info', {
            'fps': 0, 
            'state': robot_state, 
            'light': light_state,
            'target_id': target_person_id
        })

# <<< THAY ĐỔI: Sử dụng logic 3 nút (giống file cũ) >>>
@socketio.on('robot_command')
def handle_robot_command(data):
    global robot_state, manual_command
    command = data.get('command')
    
    # print(f"Web command received: {command}") # Bật nếu cần debug

    with lock:
        # Xử lý 3 nút chế độ
        if command == 'TOGGLE_FOLLOW':
            if robot_state == "FOLLOWING":
                robot_state = "IDLE"
            else:
                robot_state = "FOLLOWING"
                
        elif command == 'SET_MANUAL': 
            robot_state = "MANUAL"
        elif command == 'SET_IDLE': 
            robot_state = "IDLE"
                
        # Xử lý joystick (chỉ khi ở chế độ MANUAL)
        elif command.startswith('MANUAL_'):
            if robot_state == "MANUAL":
                manual_command = command.split('_')[1]
                
    # Gửi cập nhật trạng thái ngay lập tức
    with lock:
        emit('robot_info', {
            'fps': 0, 
            'state': robot_state, 
            'light': light_state,
            'target_id': target_person_id
        }, broadcast=True)

# <<< THAY ĐỔI: Xóa 'set_target_id' và 'cancel_target' >>>
# Các hàm @socketio.on('set_target_id') và @socketio.on('cancel_target')
# đã bị xóa vì không còn dùng logic click-to-follow.

@socketio.on('toggle_light')
def handle_toggle_light():
    global light_state
    with lock:
        light_state = not light_state 
        current_light_state = light_state
        current_state = robot_state
        current_target_id_for_web = target_person_id
    
    toggle_light_relay(current_light_state)
    
    emit('robot_info', {
        'fps': 0, 
        'state': current_state, 
        'light': current_light_state,
        'target_id': current_target_id_for_web
    }, broadcast=True)

# --- 9. Start Application (Giữ nguyên) ---
if __name__ == '__main__':
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