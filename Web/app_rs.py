# --- 1. Imports ---
import cv2
from ultralytics import YOLO
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial
from rplidar import RPLidar

# --- (***THAY ĐỔI 1: THÊM THƯ VIỆN NHẬN DIỆN KHUÔN MẶT***) ---
import face_recognition
import pickle
import os
import numpy as np

# --- Cổng & Tốc độ ---
LIDAR_PORT = '/dev/ttyUSB1'  
SERIAL_PORT = '/dev/ttyUSB0' 
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
robot_state = "IDLE"                 # <-- Khởi động ở IDLE
manual_command = "STOP"              
lock = threading.Lock()              
light_state = False                  
target_person_id = None              # Đây sẽ là YOLO track_id
tracker = None                       # Đây là CSRT Tracker (để bám mượt)

# --- (***THAY ĐỔI 2: BIẾN NHẬN DIỆN KHUÔN MẶT***) ---
ENCODINGS_DIR = "Register-ID"        # Thư mục chứa file .pkl
known_face_data = []                 # List chứa data {name, encodings}
face_recognition_cache = {}          # Cache: {track_id: "Tên"}
RECOGNIZE_FACE_INTERVAL = 15         # Chạy nhận diện 15 frame/lần

# --- Biến Lidar (Giữ nguyên) ---
lidar = None
MIN_SAFE_DISTANCE = 0.5 
lidar_scan_data = {'front_distance': float('inf'), 'back_distance': float('inf')}

# --- Hằng số P-Controller (Giữ nguyên) ---
TARGET_AREA = 50000       
KP_DISTANCE = 0.004       
KP_TURN = 0.2             
MAX_FWD_SPEED = 200       
MAX_TURN_SPEED = 160      
MIN_MOVE_PWM = 180
RE_DETECT_INTERVAL = 30        

# Serial Communication (Giữ nguyên)
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  
    print(f"Serial Port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None 

# --- 5. Robot Hardware Functions (Giữ nguyên) ---
def clamp(n, minn, maxn): return max(min(maxn, n), minn)

def set_robot_pwm(left_pwm, right_pwm, intent=""):
    global ser, lidar_scan_data, lock, MIN_SAFE_DISTANCE, MIN_MOVE_PWM
    left_pwm, right_pwm = int(left_pwm), int(right_pwm)
    # 1. Lidar
    with lock:
        current_front_distance = lidar_scan_data.get('front_distance', float('inf'))
        current_back_distance = lidar_scan_data.get('back_distance', float('inf')) 
    is_moving_forward = left_pwm > 0 or right_pwm > 0
    is_moving_backward = left_pwm < 0 or right_pwm < 0 
    if is_moving_forward and current_front_distance < MIN_SAFE_DISTANCE:
        left_pwm, right_pwm, intent = 0, 0, f"LIDAR_STOP_FWD (was {intent})"
    elif is_moving_backward and current_back_distance < MIN_SAFE_DISTANCE:
        left_pwm, right_pwm, intent = 0, 0, f"LIDAR_STOP_BCK (was {intent})"
    # 2. Deadzone
    def _boost_pwm(pwm_val):
        if 0 < pwm_val < MIN_MOVE_PWM: return MIN_MOVE_PWM
        if 0 > pwm_val > -MIN_MOVE_PWM: return -MIN_MOVE_PWM
        return pwm_val
    left_pwm, right_pwm = int(_boost_pwm(left_pwm)), int(_boost_pwm(right_pwm))
    left_pwm, right_pwm = clamp(left_pwm, -255, 255), clamp(right_pwm, -255, 255)
    # 3. Gửi Serial
    serial_command = f"MOVE:{left_pwm}:{right_pwm}\n"  
    if ser:
        try:
            ser.write(serial_command.encode())
            # Chỉ in log khi có thay đổi/hành động
            if left_pwm != 0 or right_pwm != 0 or "STOP" in intent or "IDLE" in intent:
                 print(f"INTENT: {intent} -> EXECUTING: {serial_command.strip()}")  
        except Exception as e: print(f"Serial write error: {e}")
    else:
        if left_pwm != 0 or right_pwm != 0 or "STOP" in intent or "IDLE" in intent:
            print(f"INTENT: {intent} -> SIMULATED: L_PWM:{left_pwm} R_PWM:{right_pwm}")

def execute_robot_move(command, intent=""):
    if intent == "": intent = command
    SPEED, TURN_SPEED = 190, 220
    CURVE_SLOW, CURVE_FAST = int(SPEED * 0.5), SPEED
    cmd_map = {
        "FORWARD": (SPEED, SPEED), "LEFT": (-TURN_SPEED, TURN_SPEED),
        "RIGHT": (TURN_SPEED, -TURN_SPEED), "BACKWARD": (-SPEED, -SPEED),  
        "FORWARD_LEFT": (CURVE_SLOW, CURVE_FAST), "FORWARD_RIGHT": (CURVE_FAST, CURVE_SLOW),
        "BACKWARD_LEFT": (-CURVE_FAST, -CURVE_SLOW), "BACKWARD_RIGHT": (-CURVE_SLOW, -CURVE_FAST),
        "STOP": (0, 0)
    }
    set_robot_pwm(*cmd_map.get(command, (0, 0)), intent)

def toggle_light_relay(new_state):
    global ser
    serial_command = "LIGHT:ON\n" if new_state else "LIGHT:OFF\n"
    print(f"RELAY: Turning light {'ON' if new_state else 'OFF'}")
    if ser:
        try: ser.write(serial_command.encode())
        except Exception as e: print(f"Serial write error: {e}")

# --- 6. Main Robot Logic Threads ---

def serial_read_thread(): # (Giữ nguyên)
    global ser
    if not ser: return 
    print("Serial reading thread started...")
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line: print(f"ESP32 RESPONSE: {line}")  
            time.sleep(0.01) 
        except Exception as e: print(f"Error reading from serial: {e}"); time.sleep(1) 

def lidar_logic_thread(): # (Giữ nguyên)
    global lidar, lidar_scan_data, lock
    try:
        print("Connecting to Lidar..."); lidar = RPLidar(LIDAR_PORT); print("Lidar connected.")
        for scan in lidar.iter_scans(scan_type='normal', min_len=100):
            front_mm, back_mm = float('inf'), float('inf') 
            for _, angle, distance in scan:
                if (0 <= angle <= 15) or (345 <= angle <= 360):
                    if distance > 0 and distance < front_mm: front_mm = distance
                if (165 <= angle <= 195):
                    if distance > 0 and distance < back_mm: back_mm = distance
            with lock:
                lidar_scan_data['front_distance'] = front_mm / 1000.0 if front_mm != float('inf') else float('inf')
                lidar_scan_data['back_distance'] = back_mm / 1000.0 if back_mm != float('inf') else float('inf')
            time.sleep(0.01) 
    except Exception as e: print(f"Error connecting or reading Lidar: {e}")
    finally: 
        if lidar: print("Stopping Lidar..."); lidar.stop(); lidar.disconnect()

# --- (***THAY ĐỔI 3: CÁC HÀM HELPER MỚI***) ---

def load_known_faces():
    """Tải tất cả file .pkl từ thư mục Register-ID"""
    global known_face_data
    known_face_data = []
    if not os.path.exists(ENCODINGS_DIR):
        print(f"Cảnh báo: Thư mục '{ENCODINGS_DIR}' không tồn tại. Sẽ không nhận diện được mặt.")
        return
        
    for filename in os.listdir(ENCODINGS_DIR):
        if filename.endswith(".pkl"):
            filepath = os.path.join(ENCODINGS_DIR, filename)
            try:
                with open(filepath, 'rb') as f:
                    data = pickle.load(f)
                    if "name" in data and "encodings" in data:
                        known_face_data.append(data)
                        print(f"  -> Đã tải dữ liệu cho: {data['name']} (có {len(data['encodings'])} ảnh)")
            except Exception as e:
                print(f"Lỗi khi tải file {filename}: {e}")
    print(f"Tổng cộng đã tải {len(known_face_data)} người đã đăng ký.")

def recognize_face(frame_crop_rgb):
    """Nhận diện 1 khuôn mặt từ ảnh crop"""
    global known_face_data
    try:
        # Dùng 'hog' cho nhanh hơn 'cnn'
        face_locations = face_recognition.face_locations(frame_crop_rgb, model="hog")
        if not face_locations:
            return "Unknown" 
            
        face_encoding = face_recognition.face_encodings(frame_crop_rgb, face_locations)[0]
        
        for person_data in known_face_data:
            # So sánh với TẤT CẢ các ảnh đã đăng ký của 1 người
            matches = face_recognition.compare_faces(person_data["encodings"], face_encoding, tolerance=0.5)
            if True in matches:
                return person_data["name"] # TRẢ VỀ TÊN
                
        return "Unknown"
    except Exception as e:
        # Thường là lỗi 'IndexError' nếu crop quá nhỏ
        return "Unknown"

# --- (***THAY ĐỔI 4: ROBOT LOGIC THREAD (ĐẠI TU)***) ---
# (Đây là logic 3 trạng thái ĐÚNG)
# -----------------------------------------------------------------
def robot_logic_thread():
    global global_frame, robot_state, manual_command, light_state, model
    global target_person_id, tracker, face_recognition_cache

    if model is None: print("FATAL: YOLO Model not loaded."); return

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    FRAME_HEIGHT, FRAME_WIDTH = 480, 640 
    FRAME_CENTER_X = FRAME_WIDTH / 2
    
    prev_frame_time = 0
    print("Robot logic thread started (với logic SỬA LỖI)...")
    
    frame_count = 0
    INFO_SKIP_FRAMES = 15 
    jpeg_quality = [int(cv2.IMWRITE_JPEG_QUALITY), 80] # Tăng chất lượng ảnh 1 chút

    # --- Các hàm Helper nội bộ ---
    def run_p_controller(bbox, frame_center_x):
        (x, y, w, h) = (int(t) for t in bbox)
        current_centerX, current_area = x + (w // 2), w * h
        if frame_count % INFO_SKIP_FRAMES == 0: 
             print(f"DEBUG (Follow): Area: {current_area:.0f} (Target: {TARGET_AREA})")
        error_area = TARGET_AREA - current_area
        fwd_speed = clamp(KP_DISTANCE * error_area, -MAX_FWD_SPEED, MAX_FWD_SPEED)
        error_turn = frame_center_x - current_centerX
        turn_speed = clamp(KP_TURN * error_turn, -MAX_TURN_SPEED, MAX_TURN_SPEED)
        set_robot_pwm(fwd_speed + turn_speed, fwd_speed - turn_speed, "PID_FOLLOW")

    def find_target_box(results, target_id):
        # Tìm bbox (x,y,w,h) của 1 track_id cụ thể
        if results[0].boxes and results[0].boxes.id is not None:
            for box in results[0].boxes:
                if int(box.id[0]) == target_id:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    return (x1, y1, x2-x1, y2-y1) # (x, y, w, h)
        return None 

    while True:
        if not cap.isOpened():
            print("Camera not open. Trying to reconnect..."); time.sleep(1)
            cap.release(); cap = cv2.VideoCapture(0) 
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            continue 

        success, image = cap.read()
        if not success:
            print("Camera read failed, skipping frame."); time.sleep(1); continue 
            
        frame_count += 1
        image = cv2.flip(image, 1) 
        
        with lock:
            current_state = robot_state
            current_manual_cmd = manual_command
            current_target_id = target_person_id

        hud_text = f"STATE: {current_state}"
        hud_color = (0, 0, 255) 
        boxes_to_send = [] 

        # -----------------------------------------------------
        # TRẠNG THÁI: IDLE (BẬT AI + NHẬN DIỆN)
        # (Đây là LỖI 1: Code cũ của bạn không chạy AI ở đây)
        # -----------------------------------------------------
        if current_state == "IDLE":
            hud_color = (0, 255, 255) # Vàng
            
            # 1. Chạy YOLO tracker
            results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.5, tracker="my_tracker.yaml")
            
            if results[0].boxes and results[0].boxes.id is not None:
                boxes_cpu = results[0].boxes.xyxy.cpu().numpy().astype(int)
                track_ids = results[0].boxes.id.cpu().numpy().astype(int)

                for box, track_id in zip(boxes_cpu, track_ids):
                    x1, y1, x2, y2 = box
                    
                    # 2. Logic nhận diện khuôn mặt (có cache)
                    name = face_recognition_cache.get(track_id, None)
                    
                    # Chỉ quét lại 15 frame 1 lần
                    if name is None or frame_count % RECOGNIZE_FACE_INTERVAL == 0:
                        #print(f"Đang nhận diện ID: {track_id}...")
                        try:
                            crop = image[y1:y2, x1:x2]
                            rgb_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                            name = recognize_face(rgb_crop)
                            face_recognition_cache[track_id] = name # Lưu vào cache
                        except Exception as e:
                             face_recognition_cache[track_id] = "Unknown" 
                        
                    # 3. Vẽ box và gửi data
                    rect_list = [int(x1), int(y1), int(x2), int(y2)] 
                    # Gửi ID LÊN WEB ĐỂ CLICK
                    boxes_to_send.append({'id': int(track_id), 'rect': rect_list})
                    
                    box_color = (0, 255, 0) if name != "Unknown" else (0, 255, 255)
                    label = f"{name} (ID: {track_id})"
                    
                    cv2.rectangle(image, (x1, y1), (x2, y2), box_color, 2)
                    cv2.putText(image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
            else:
                 # Nếu không thấy ai, xóa cache cũ
                 if frame_count % 30 == 0: face_recognition_cache.clear()
            
            # ĐẢM BẢO ROBOT ĐỨNG YÊN
            set_robot_pwm(0, 0, "IDLE_STATE")

        # -----------------------------------------------------
        # TRẠNG THÁI: MANUAL (TẮT AI)
        # (Đây là LỖI 2: Code cũ của bạn đã làm đúng)
        # -----------------------------------------------------
        elif current_state == "MANUAL":
            hud_color = (255, 0, 0) # Blue
            hud_text = "STATE: MANUAL (AI OFF)"
            
            # KHÔNG CHẠY AI (YOLO/Face)
            
            execute_robot_move(current_manual_cmd, "MANUAL_JOYSTICK")
            
            face_recognition_cache.clear() # Xóa cache khi chuyển chế độ
            boxes_to_send = []

        # -----------------------------------------------------
        # TRẠNG THÁI: FOLLOWING (BẬT TRACKER MƯỢT)
        # (Đây là LỖI 3: Code cũ của bạn bị giật, code này dùng Tracker)
        # -----------------------------------------------------
        # -----------------------------------------------------
        # TRẠNG THÁI: FOLLOWING (ĐÃ SỬA LỖI "BỊP")
        # -----------------------------------------------------
        elif current_state == "FOLLOWING":
            hud_color = (0, 250, 0) # Green
            
            if tracker is None:
                # A. KHỞI TẠO TRACKER (Chạy 1 lần)
                hud_text = f"FOLLOW (Finding ID: {current_target_id})"
                print(f"STATE: FOLLOWING. Đang chạy YOLO 1 lần để tìm ID {current_target_id}...")
                
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.5, tracker="my_tracker.yaml")
                target_bbox = find_target_box(results, current_target_id)
                
                if target_bbox:
                    print(f"  -> Đã tìm thấy ID {current_target_id}. Khởi tạo Tracker.")
                    tracker = cv2.legacy.TrackerMOSSE_create()
                    tracker.init(image, target_bbox)
                    run_p_controller(target_bbox, FRAME_CENTER_X)
                    (x,y,w,h) = (int(t) for t in target_bbox)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                else:
                    print(f"  -> Không tìm thấy ID {current_target_id}. Dừng chờ.")
                    set_robot_pwm(0, 0, "STOP (Cannot find target)")

            else:
                # B. ĐANG TRACKING (Chạy Tracker siêu nhẹ) 
                hud_text = f"TRACKING ID: {current_target_id}"
                success, bbox_tracker = tracker.update(image)
                
                if success:
                    # BÁM TỐT -> Chạy P-Controller
                    run_p_controller(bbox_tracker, FRAME_CENTER_X)
                    (x,y,w,h) = (int(t) for t in bbox_tracker)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)

                    # --- (*** LOGIC SỬA LỖI: KIỂM TRA CHÉO ***) ---
                    if frame_count % RE_DETECT_INTERVAL == 0:
                        print(f"  [Re-Check] Đã đến frame {frame_count}. Chạy YOLO để kiểm tra...")
                        results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.5, tracker="my_tracker.yaml")
                        yolo_bbox = find_target_box(results, current_target_id)
                        
                        if yolo_bbox is None:
                            # YOLO KHÔNG THẤY MỤC TIÊU (CAMERA BỊ CHE)
                            print("  [Re-Check] LỖI: YOLO không tìm thấy ID. Tracker đã mất dấu!")
                            with lock: tracker = None
                            set_robot_pwm(0, 0, "STOP (YOLO Re-Check Failed)")
                        else:
                            # YOLO THẤY -> Hiệu chỉnh tracker về vị trí đúng
                            print("  [Re-Check] OK. YOLO xác nhận. Hiệu chỉnh lại tracker.")
                            tracker = cv2.TrackerCSRT_create()
                            tracker.init(image, yolo_bbox)
                    # --- (*** HẾT LOGIC SỬA LỖI ***) ---
                            
                else:
                    # TRACKER CSRT TỰ BÁO LÀ MẤT DẤU (hiếm khi xảy ra)
                    print(f"!!! TRACKER CSRT TỰ BÁO FAILED. Sẽ chạy lại YOLO để tìm.")
                    with lock: tracker = None 
                    set_robot_pwm(0, 0, "STOP (Tracker Lost)")
            
            face_recognition_cache.clear()
            boxes_to_send = []
        
        # --- HUD & Frame Update (Giữ nguyên) ---
        new_frame_time = time.time(); fps = 1 / (new_frame_time - prev_frame_time) if (new_frame_time - prev_frame_time) > 0 else 0
        prev_frame_time = new_frame_time
        cv2.putText(image, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        cv2.putText(image, f"FPS: {int(fps)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        
        # --- GỬI DỮ LIỆU LÊN WEB (Giữ nguyên) ---
        if frame_count % INFO_SKIP_FRAMES == 0:
            with lock: 
                current_light_state = light_state  
                current_target_id_for_web = target_person_id
            socketio.emit('robot_info', {
                'fps': int(fps), 
                'state': current_state, 
                'light': current_light_state,
                'target_id': current_target_id_for_web # Gửi ID mục tiêu
            })
        
        # Chỉ gửi box khi ở IDLE
        if current_state == "IDLE":
             if len(boxes_to_send) > 0 or frame_count % 5 == 0: 
                socketio.emit('detected_boxes', {'boxes': boxes_to_send})
        else:
             if frame_count % 5 == 0: 
                socketio.emit('detected_boxes', {'boxes': []}) # Xóa box

        with lock:
            _, buffer = cv2.imencode('.jpg', image, jpeg_quality)
            global_frame = buffer.tobytes()

# --- 7. Flask HTTP Routes (Giữ nguyên) ---
@app.route('/')
def index():
    # Đảm bảo bạn đang dùng file HTML có các nút bấm
    # render_template('index.html') hoặc 'index2.html'
    return render_template('index.html') # Đổi tên file nếu cần

@app.route('/video_feed')
def video_feed():
    def gen_frames():
        global global_frame
        while True:
            with lock: frame_bytes = global_frame
            if frame_bytes is None: time.sleep(0.1); continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- 8. Socket.IO Events ---

@socketio.on('connect')
def handle_connect():
    print('Client connected!')
    with lock: emit('robot_info', {
        'fps': 0, 
        'state': robot_state, 
        'light': light_state,
        'target_id': target_person_id
    })

# --- (***THAY ĐỔI 5: SOCKET LOGIC (SỬA LỖI BẤM NÚT)***) ---
# (Đây là logic socket ĐÚNG để CHỌN NGƯỜI và CHỌN CHẾ ĐỘ)

@socketio.on('robot_command')
def handle_robot_command(data):
    """Joystick TỰ ĐỘNG chuyển sang MANUAL (nếu không đang follow)"""
    global robot_state, manual_command, target_person_id, tracker
    command = data.get('command')
    
    with lock:
        if command.startswith('MANUAL_'):
            # Nếu đang ở IDLE, joystick sẽ tự chuyển sang MANUAL
            if robot_state == "IDLE": 
                print(f"*** Nhận lệnh Joystick, tự động chuyển sang MANUAL ***")
                robot_state = "MANUAL"
                emit('robot_info', {'state': "MANUAL", 'light': light_state}, broadcast=True)

            # Chỉ nhận lệnh khi đang ở MANUAL
            if robot_state == "MANUAL":
                manual_command = command.split('_')[1]
                target_person_id = None; tracker = None
            
@socketio.on('set_mode_manual')
def handle_set_mode_manual():
    """Xử lý khi BẤM NÚT 'Manual'"""
    global robot_state, target_person_id, tracker
    with lock:
        if robot_state == "MANUAL": return # Đang ở Manual rồi
        print("*** Nhận lệnh NÚT BẤM: Chuyển sang MANUAL ***")
        robot_state = "MANUAL"
        target_person_id = None
        tracker = None
        emit('robot_info', {'state': "MANUAL", 'light': light_state}, broadcast=True)

@socketio.on('set_mode_idle')
def handle_set_mode_idle():
    """Xử lý khi BẤM NÚT 'Idle' (hoặc Hủy)"""
    global robot_state, target_person_id, tracker
    with lock:
        if robot_state == "IDLE": return # Đang ở Idle rồi
        print(f"*** Nhận lệnh NÚT BẤM: Chuyển về IDLE ***")
        robot_state = "IDLE" 
        target_person_id = None
        tracker = None 
        emit('robot_info', {'state': "IDLE", 'light': light_state}, broadcast=True)

@socketio.on('set_target_id')
def handle_set_target(data):
    """Xử lý khi CLICK VÀO BOX (chuyển sang FOLLOWING)"""
    global robot_state, target_person_id, tracker
    target_id = data.get('id')
    
    if target_id is not None:
        with lock:
            target_person_id = int(target_id)
            robot_state = "FOLLOWING" 
            tracker = None # Sẽ được logic thread tự khởi tạo
            print(f"*** NEW TARGET ACQUIRED (Click): ID {target_person_id} ***")
            emit('robot_info', {'state': "FOLLOWING", 'light': light_state, 'target_id': target_person_id}, broadcast=True)

@socketio.on('cancel_target')
def handle_cancel_target():
    """Xử lý khi nhấn 'Hủy Theo Dõi' -> Quay về IDLE"""
    handle_set_mode_idle() # Gọi luôn hàm set_mode_idle

@socketio.on('toggle_light')
def handle_toggle_light(): # (Giữ nguyên)
    global light_state
    with lock:
        light_state = not light_state 
        current_light_state = light_state
        current_state = robot_state
        current_target_id_for_web = target_person_id
    toggle_light_relay(current_light_state)  
    emit('robot_info', {
        'state': current_state, 
        'light': current_light_state,
        'target_id': current_target_id_for_web
    }, broadcast=True)

# --- 9. Start Application ---
if __name__ == '__main__':
    if model is None:
        print("STOPPING: YOLO model failed to load.")
    else:
        # Tải dữ liệu khuôn mặt LÊN TRƯỚC
        print("Starting Face Data Loader...")
        load_known_faces() # <-- QUAN TRỌNG
        
        # Khởi động các luồng
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