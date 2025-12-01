# --- 1. Imports ---
import cv2
from ultralytics import YOLO
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial
import face_recognition
import pickle
import os
import numpy as np

# --- Cổng & Tốc độ --- 
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
robot_state = "IDLE"                 
manual_command = "STOP"              
lock = threading.Lock()              
light_state = False                  
target_person_id = None            
tracker = None                       

# --- (Biến nhận diện khuôn mặt) ---
ENCODINGS_DIR = "Register-ID"        
known_face_data = []                 
face_recognition_cache = {}          
RECOGNIZE_FACE_INTERVAL = 15         

# --- Biến Cảm biến (Thay Lidar bằng HC-SR04) ---
lidar_scan_data = {'front_distance': 999.0, 'back_distance': 999.0}
MIN_SAFE_DISTANCE = 0.5 # 50 cm

# ==============================================================================
# --- CẤU HÌNH TỐC ĐỘ & PID (ĐÃ TUNE LẠI) ---
# ==============================================================================
TARGET_AREA = 60000        

# 1. PID Tuning
KP_DISTANCE = 0.0015       
KD_DISTANCE = 0.6          
KP_TURN = 0.35             
KD_TURN = 0.1              

# 2. AUTO MODE (KẸP 180-195)
MIN_MOVE_PWM = 180         # Sàn: Dưới mức này kích lên 180
MAX_FWD_SPEED_AUTO = 195   # Trần: Cắt ngay nếu vượt quá 195
MAX_TURN_SPEED_AUTO = 185  

# 3. MANUAL MODE
MAX_SPEED_MANUAL = 255     # Mặc định, sẽ được Slider cập nhật

RE_DETECT_INTERVAL = 30    

# Serial Communication
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  
    print(f"Serial Port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None 

# --- 5. Robot Hardware Functions ---

def clamp(n, minn, maxn): 
    return max(min(maxn, n), minn)

def set_robot_pwm(left_pwm, right_pwm, intent=""):
    global ser, lidar_scan_data, lock, MIN_SAFE_DISTANCE, MIN_MOVE_PWM
    left_pwm, right_pwm = int(left_pwm), int(right_pwm)
    
    # 1. FAILSAFE (SỬ DỤNG DỮ LIỆU TỪ ESP32)
    with lock:
        d_front = lidar_scan_data.get('front_distance', 999.0)
        d_back = lidar_scan_data.get('back_distance', 999.0)
    
    is_fwd = left_pwm > 0 or right_pwm > 0
    is_bck = left_pwm < 0 or right_pwm < 0
    
    blocked = False
    if is_fwd and d_front < MIN_SAFE_DISTANCE:
        left_pwm, right_pwm = 0, 0
        intent = f"FAILSAFE_STOP_FWD (Dist: {d_front:.2f}m)"
        blocked = True
    elif is_bck and d_back < MIN_SAFE_DISTANCE:
        left_pwm, right_pwm = 0, 0
        intent = f"FAILSAFE_STOP_BCK (Dist: {d_back:.2f}m)"
        blocked = True

    # 2. Deadzone Boost (CHỈ AUTO & KHÔNG BỊ BLOCK)
    if "PD_FOLLOW" in intent and not blocked:
        def _boost_pwm(pwm_val):
            if pwm_val == 0: return 0
            # Nếu yếu quá -> Kích lên 180
            if 0 < pwm_val < MIN_MOVE_PWM: return MIN_MOVE_PWM
            if 0 > pwm_val > -MIN_MOVE_PWM: return -MIN_MOVE_PWM
            return pwm_val
        left_pwm = int(_boost_pwm(left_pwm))
        right_pwm = int(_boost_pwm(right_pwm))
    
    # Clamp
    left_pwm = clamp(left_pwm, -255, 255)
    right_pwm = clamp(right_pwm, -255, 255)

    # 3. Gửi Serial
    serial_command = f"MOVE:{left_pwm}:{right_pwm}\n"  
    if ser:
        try:
            ser.write(serial_command.encode())
            # LOGGING: Chỉ in khi có lệnh điều khiển hoặc Failsafe
            if left_pwm != 0 or right_pwm != 0 or "STOP" in intent or "FAILSAFE" in intent:
                 if "PD_FOLLOW" not in intent: 
                     print(f"INTENT: {intent} -> EXECUTING: {serial_command.strip()}")
        except Exception as e: print(f"Serial write error: {e}")

def execute_robot_move(command, intent=""):
    if intent == "": intent = command
    # Dùng biến toàn cục (được chỉnh bởi Slider)
    SPEED = MAX_SPEED_MANUAL - 15
    TURN_SPEED = MAX_SPEED_MANUAL
    CURVE_SLOW = int(SPEED * 0.4)
    CURVE_FAST = SPEED
    
    cmd_map = {
        "FORWARD": (SPEED, SPEED), "LEFT": (-TURN_SPEED, TURN_SPEED),
        "RIGHT": (TURN_SPEED, -TURN_SPEED), "BACKWARD": (-SPEED, -SPEED),  
        "FORWARD_LEFT": (CURVE_SLOW, CURVE_FAST), "FORWARD_RIGHT": (CURVE_FAST, CURVE_SLOW),
        "BACKWARD_LEFT": (-CURVE_FAST, -CURVE_SLOW), "BACKWARD_RIGHT": (-CURVE_SLOW, -CURVE_FAST),
        "STOP": (0, 0)
    }
    set_robot_pwm(*cmd_map.get(command, (0, 0)), "MANUAL_" + intent)

def toggle_light_relay(new_state):
    global ser
    serial_command = "LIGHT:ON\n" if new_state else "LIGHT:OFF\n"
    if ser:
        try: ser.write(serial_command.encode())
        except Exception as e: print(f"Serial write error: {e}")

# --- 6. Main Robot Logic Threads ---

def serial_read_thread(): 
    # Đọc dữ liệu từ ESP32: STATUS:L:R:Front:Back:Obs
    global ser, lidar_scan_data, lock
    if not ser: return 
    print("Serial reading thread started...")
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("STATUS:"):
                    parts = line.split(':')
                    if len(parts) >= 6:
                        with lock:
                            # Đổi cm -> m để khớp với biến MIN_SAFE_DISTANCE (0.5)
                            lidar_scan_data['front_distance'] = float(parts[3]) / 100.0
                            lidar_scan_data['back_distance'] = float(parts[4]) / 100.0
                elif line: 
                    print(f"ESP32 RESPONSE: {line}")  
            time.sleep(0.01) 
        except Exception as e: time.sleep(1) 

# --- Helper ---

def load_known_faces():
    global known_face_data
    known_face_data = []
    if not os.path.exists(ENCODINGS_DIR):
        print(f"Cảnh báo: Thư mục '{ENCODINGS_DIR}' không tồn tại.")
        return
    for filename in os.listdir(ENCODINGS_DIR):
        if filename.endswith(".pkl"):
            try:
                with open(os.path.join(ENCODINGS_DIR, filename), 'rb') as f:
                    data = pickle.load(f)
                    known_face_data.append(data)
            except: pass
    print(f"Tổng cộng đã tải {len(known_face_data)} người.")

def recognize_face(frame_crop_rgb):
    global known_face_data
    try:
        face_locations = face_recognition.face_locations(frame_crop_rgb, model="hog")
        if not face_locations: return "Unknown"
        face_encoding = face_recognition.face_encodings(frame_crop_rgb, face_locations)[0]
        for person_data in known_face_data:
            if True in face_recognition.compare_faces(person_data["encodings"], face_encoding, 0.5):
                return person_data["name"]
        return "Unknown"
    except: return "Unknown"

# --- (ROBOT LOGIC THREAD - PD CONTROLLER) ---
def robot_logic_thread():
    global global_frame, robot_state, manual_command, light_state, model
    global target_person_id, tracker, face_recognition_cache

    if model is None: print("FATAL: YOLO Model not loaded."); return

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    FRAME_CENTER_X = 320
    
    prev_frame_time = 0
    print("Robot logic thread started...")
    
    frame_count = 0
    INFO_SKIP_FRAMES = 15 
    jpeg_quality = [int(cv2.IMWRITE_JPEG_QUALITY), 80] 

    # --- Biến cho PD CONTROLLER ---
    prev_error_area = 0.0
    prev_error_turn = 0.0
    prev_time_pd = time.time()

    # --- Hàm PD Controller ---
    def run_pd_controller(bbox, frame_center_x):
        nonlocal prev_error_area, prev_error_turn, prev_time_pd

        current_time_pd = time.time()
        time_delta = current_time_pd - prev_time_pd
        if time_delta == 0:  time_delta = 1e-6 

        (x, y, w, h) = (int(t) for t in bbox)
        current_centerX, current_area = x + (w // 2), w * h
        
        # --- SỬA LOGIC 1: ĐẢO CHIỀU TIẾN/LÙI ---
        # Target (Lớn) - Current (Nhỏ) = Dương -> Tiến
        error_area = TARGET_AREA - current_area 
        error_turn = frame_center_x - current_centerX

        if frame_count % INFO_SKIP_FRAMES == 0: 
            print(f"DEBUG (PID): Area={current_area:.0f} | ErrA={error_area:.0f} | ErrT={error_turn:.0f}")

        d_area = (error_area - prev_error_area) / time_delta
        d_turn = (error_turn - prev_error_turn) / time_delta

        p_term_area = KP_DISTANCE * error_area
        d_term_area = KD_DISTANCE * derivative_area
        fwd_speed = p_term_area + d_term_area
        
        # --- KẸP TRẦN AUTO (195) ---
        fwd_speed = clamp(fwd_speed, -MAX_FWD_SPEED_AUTO, MAX_FWD_SPEED_AUTO)
        
        p_term_turn = KP_TURN * error_turn
        d_term_turn = KD_TURN * derivative_turn 
        turn_speed = p_term_turn + d_term_turn
        
        # --- KẸP TRẦN RẼ (185) ---
        turn_speed = clamp(turn_speed, -MAX_TURN_SPEED_AUTO, MAX_TURN_SPEED_AUTO)
        
        prev_error_area = error_area
        prev_error_turn = error_turn
        prev_time_pd = current_time_pd

        # --- SỬA LOGIC 2: ĐẢO CHIỀU RẼ ---
        # Left = Fwd - Turn
        left_pwm = fwd_speed - turn_speed 
        right_pwm = fwd_speed + turn_speed 
        
        set_robot_pwm(left_pwm, right_pwm, "PD_FOLLOW")

    def find_target_box(results, target_id):
        if results[0].boxes and results[0].boxes.id is not None:
            for box in results[0].boxes:
                if int(box.id[0]) == target_id:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    return (x1, y1, x2-x1, y2-y1) 
        return None 

    while True:
        if not cap.isOpened():
            print("Camera error. Reconnecting..."); time.sleep(1)
            cap.release(); cap = cv2.VideoCapture(0) 
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            continue 

        success, image = cap.read()
        if not success:
            print("Camera read failed."); time.sleep(1); continue 
            
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
        # IDLE
        # -----------------------------------------------------
        if current_state == "IDLE":
            hud_color = (0, 255, 255) 
            results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, tracker="my_tracker.yaml")
            if results[0].boxes and results[0].boxes.id is not None:
                boxes_cpu = results[0].boxes.xyxy.cpu().numpy().astype(int)
                track_ids = results[0].boxes.id.cpu().numpy().astype(int)
                for b, i in zip(boxes_cpu, track_ids):
                    name = face_recognition_cache.get(i, None)
                    if name is None or frame_count % RECOGNIZE_FACE_INTERVAL == 0:
                        try:
                            crop = cv2.cvtColor(image[b[1]:b[3], b[0]:b[2]], cv2.COLOR_BGR2RGB)
                            face_recognition_cache[i] = recognize_face(crop)
                        except: pass
                    
                    boxes_to_send.append({'id': int(i), 'rect': [int(x) for x in b]})
                    cv2.rectangle(image, (b[0],b[1]), (b[2],b[3]), (0,255,0), 2)
                    cv2.putText(image, f"{face_recognition_cache.get(i,'Unknown')} ({i})", (b[0],b[1]-10), 0, 0.5, (0,255,0), 2)
            
            set_robot_pwm(0, 0, "IDLE_STATE")
            if frame_count % 5 == 0: socketio.emit('detected_boxes', {'boxes': boxes_to_send})

        # -----------------------------------------------------
        # MANUAL
        # -----------------------------------------------------
        elif current_state == "MANUAL":
            hud_color = (255, 0, 0) 
            hud_text = "STATE: MANUAL (AI OFF)"
            execute_robot_move(current_manual_cmd, "MANUAL_JOYSTICK")
            face_recognition_cache.clear() 
            if frame_count % 5 == 0: socketio.emit('detected_boxes', {'boxes': []})

        # -----------------------------------------------------
        # FOLLOWING (LOGIC RE-CHECK GIỮ NGUYÊN)
        # -----------------------------------------------------
        elif current_state == "FOLLOWING":
            hud_color = (0, 250, 0) 
            
            if tracker is None:
                hud_text = f"FOLLOW (Finding ID: {current_target_id})"
                # Re-init using YOLO
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, tracker="my_tracker.yaml")
                target_bbox = find_target_box(results, current_target_id)
                
                if target_bbox:
                    print(f"🎯 FOUND ID {current_target_id}. INIT TRACKER.")
                    tracker = cv2.legacy.TrackerMOSSE_create() 
                    tracker.init(image, target_bbox)
                    run_pd_controller(target_bbox, FRAME_CENTER_X) 
                else:
                    set_robot_pwm(0, 0, "SEARCHING") # Không dừng hẳn
            else:
                hud_text = f"TRACKING ID: {current_target_id}"
                success, bbox_tracker = tracker.update(image)
                
                if success:
                    run_pd_controller(bbox_tracker, FRAME_CENTER_X)
                    cv2.rectangle(image, (int(bbox_tracker[0]), int(bbox_tracker[1])), (int(bbox_tracker[0]+bbox_tracker[2]), int(bbox_tracker[1]+bbox_tracker[3])), (0,255,255), 3)
                    
                    # LOGIC RE-CHECK
                    if frame_count % RE_DETECT_INTERVAL == 0:
                        res = model.track(image, persist=True, verbose=False, imgsz=320, tracker="my_tracker.yaml")
                        yolo_box = find_target_box(res, current_target_id)
                        if yolo_box:
                            tracker = cv2.legacy.TrackerMOSSE_create()
                            tracker.init(image, yolo_box)
                        else:
                            print("❌ RE-CHECK FAILED. RESET TRACKER.")
                            tracker = None 
                else:
                    print("❌ TRACKER LOST.")
                    tracker = None

            if frame_count % 5 == 0: socketio.emit('detected_boxes', {'boxes': []})
        
        # --- HUD ---
        new_frame_time = time.time(); fps = 1 / (new_frame_time - prev_frame_time) if (new_frame_time - prev_frame_time) > 0 else 0
        prev_frame_time = new_frame_time
        cv2.putText(image, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        
        # Cảnh báo Failsafe trên màn hình
        with lock: fd = lidar_scan_data.get('front_distance', 999)
        if fd < MIN_SAFE_DISTANCE:
             cv2.putText(image, f"⚠️ STOP! FRONT: {fd:.2f}m", (10, 240), 0, 1.2, (0,0,255), 3)
        else:
             cv2.putText(image, f"F_Dist: {fd:.2f}m", (10, 60), 0, 0.7, (0,255,255), 2)

        if frame_count % 15 == 0:
            with lock: 
                cur_l = light_state
                cur_t = target_person_id
            socketio.emit('robot_info', {
                'fps': int(fps), 
                'state': current_state, 
                'light': current_light_state,
                'target_id': current_target_id_for_web
            })

        with lock:
            _, buffer = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            global_frame = buffer.tobytes()

# --- 7. Flask HTTP Routes ---
@app.route('/')
def index(): return render_template('index2.html') 

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            with lock: f = global_frame
            if f is None: time.sleep(0.1); continue
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + f + b'\r\n')
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- 8. Socket Events ---
@socketio.on('robot_command')
def on_cmd(data):
    global robot_state, manual_command, tracker
    cmd = data.get('command')
    if cmd.startswith('MANUAL_'):
        with lock: 
            robot_state = "MANUAL"
            manual_command = cmd.split('_')[1]
            tracker = None

@socketio.on('set_mode_manual')
def on_manual():
    with lock: global robot_state; robot_state = "MANUAL"

@socketio.on('set_mode_idle')
def on_idle():
    with lock: 
        print("CANCEL FOLLOW -> IDLE")
        global robot_state, tracker, target_person_id
        robot_state, target_person_id, tracker = "IDLE", None, None

@socketio.on('set_target_id')
def on_target(data):
    with lock:
        global robot_state, target_person_id, tracker
        robot_state = "FOLLOWING"
        target_person_id = int(data.get('id'))
        tracker = None

@socketio.on('toggle_light')
def on_light():
    global light_state
    light_state = not light_state
    toggle_light_relay(light_state)

# Slider Update
@socketio.on('update_manual_speed')
def handle_update_speed(data):
    global MAX_SPEED_MANUAL
    try: MAX_SPEED_MANUAL = int(data.get('speed', 200))
    except: pass

if __name__ == '__main__':
    load_known_faces()
    threading.Thread(target=robot_logic_thread, daemon=True).start()
    threading.Thread(target=serial_read_thread, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5001, debug=False, allow_unsafe_werkzeug=True)