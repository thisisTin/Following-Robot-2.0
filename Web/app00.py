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

# --- 4. Global Variables ---
global_frame = None                  
robot_state = "IDLE"                 
manual_command = "STOP"              
lock = threading.Lock()              
light_state = False                  

# --- BIẾN THEO DÕI ---
target_person_id = None              
target_person_name = None            
tracker = None                       
is_searching_for_target = False      
latest_bboxes = {} 

# --- NHẬN DIỆN KHUÔN MẶT ---
ENCODINGS_DIR = "Register-ID"        
known_face_data = []                 
face_recognition_cache = {}          
RECOGNIZE_FACE_INTERVAL = 10         

# --- Biến Cảm biến ---
sensor_data = {'front': 999.0, 'back': 999.0, 'obstacle': False}

# --- CẤU HÌNH FAILSAFE ---
SAFE_DIST_AUTO = 50     # cm (Follow/Idle)
SAFE_DIST_MANUAL = 15   # cm (Manual)
current_python_failsafe_cm = SAFE_DIST_AUTO 

# --- HẰNG SỐ PD CONTROLLER (VALUES TUNE CŨ) ---
TARGET_AREA = 45000       

# Tune cũ (Ổn định):
KP_DISTANCE = 0.0228
KD_DISTANCE = 0.7747       
KP_TURN = 0.45
KD_TURN = 0.08             

# TỐC ĐỘ 
MAX_FWD_SPEED = 200        
MAX_TURN_SPEED = 190       
MIN_MOVE_PWM = 180         
RE_DETECT_INTERVAL = 30        

# Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  
    print(f"Serial Port {SERIAL_PORT} opened.")
    time.sleep(2) 
except serial.SerialException as e:
    print(f"ERROR: Serial {SERIAL_PORT} failed. {e}")
    ser = None 

# --- 5. Robot Hardware Functions ---

def clamp(n, minn, maxn): return max(min(maxn, n), minn)

def set_failsafe_distance_cm(dist_cm):
    """Gửi lệnh thay đổi ngưỡng dừng an toàn xuống Arduino."""
    global ser, current_python_failsafe_cm
    current_python_failsafe_cm = dist_cm
    cmd = f"SET_STOP_DIST:{dist_cm}\n"
    if ser:
        try: ser.write(cmd.encode())
        except Exception: pass

def set_robot_pwm(left_pwm, right_pwm, intent=""):
    global ser, sensor_data, lock, current_python_failsafe_cm, MIN_MOVE_PWM
    left_pwm, right_pwm = int(left_pwm), int(right_pwm)
    
    # 1. Failsafe Logic
    with lock:
        dist_front = sensor_data['front']
        dist_back = sensor_data['back']
    
    if (left_pwm > 0 or right_pwm > 0) and dist_front < current_python_failsafe_cm: 
        left_pwm, right_pwm = 0, 0 
    
    if (left_pwm < 0 or right_pwm < 0) and dist_back < current_python_failsafe_cm: 
        left_pwm, right_pwm = 0, 0

    # 2. Boost
    def _boost_pwm(pwm_val):
        if 0 < pwm_val < MIN_MOVE_PWM: return MIN_MOVE_PWM
        if 0 > pwm_val > -MIN_MOVE_PWM: return -MIN_MOVE_PWM
        return pwm_val
    
    if left_pwm != 0: left_pwm = _boost_pwm(left_pwm)
    if right_pwm != 0: right_pwm = _boost_pwm(right_pwm)

    left_pwm = clamp(left_pwm, -255, 255)
    right_pwm = clamp(right_pwm, -255, 255)

    # 3. Serial
    serial_command = f"MOVE:{left_pwm}:{right_pwm}\n"  
    if ser:
        try: ser.write(serial_command.encode())
        except Exception: pass

def execute_robot_move(command, intent=""):
    SPEED = MAX_FWD_SPEED 
    TURN_SPEED = MAX_TURN_SPEED
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
    cmd = "LIGHT:ON\n" if new_state else "LIGHT:OFF\n"
    if ser: ser.write(cmd.encode())

# --- 6. Threads ---

def serial_read_thread(): 
    global ser, sensor_data, lock
    time.sleep(1)
    set_failsafe_distance_cm(SAFE_DIST_AUTO)
    
    if not ser: return 
    print("Serial reading thread started...")
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("STATUS:"):
                    parts = line.split(':')
                    if len(parts) >= 5: 
                        with lock:
                            try:
                                sensor_data['front'] = float(parts[3])
                                sensor_data['back'] = float(parts[4])
                            except ValueError: pass
            time.sleep(0.02) 
        except Exception: time.sleep(0.1) 

def load_known_faces():
    global known_face_data
    known_face_data = []
    if not os.path.exists(ENCODINGS_DIR): return
    for filename in os.listdir(ENCODINGS_DIR):
        if filename.endswith(".pkl"):
            try:
                with open(os.path.join(ENCODINGS_DIR, filename), 'rb') as f:
                    data = pickle.load(f)
                    known_face_data.append(data)
            except: pass
    print(f"Loaded {len(known_face_data)} faces.")

def recognize_face(frame_crop_rgb):
    global known_face_data
    try:
        # Upsample 2 lần để thấy mặt nhỏ khi ở xa
        face_locs = face_recognition.face_locations(frame_crop_rgb, number_of_times_to_upsample=2, model="hog")
        if not face_locs: return "Unknown"
        encoding = face_recognition.face_encodings(frame_crop_rgb, face_locs)[0]
        for p in known_face_data:
            matches = face_recognition.compare_faces(p["encodings"], encoding, tolerance=0.55)
            if True in matches: return p["name"]
        return "Unknown"
    except: return "Unknown"

# --- HELPER: TẠO TRACKER ---
def create_tracker():
    try: return cv2.legacy.TrackerMOSSE_create()
    except: return cv2.TrackerCSRT_create()

# --- MAIN LOGIC THREAD ---
frame_count = 0 

def robot_logic_thread():
    global global_frame, robot_state, manual_command, model
    global target_person_id, target_person_name, tracker, face_recognition_cache
    global frame_count, is_searching_for_target, latest_bboxes

    if model is None: return
    cap = cv2.VideoCapture(0)
    cap.set(3, 640); cap.set(4, 480)
    FRAME_CENTER_X = 320
    
    print("Logic thread started... Waiting for Connection")
    
    prev_error_area = 0.0
    prev_error_turn = 0.0
    prev_time_pd = time.time()

    def run_pd_controller(bbox):
        nonlocal prev_error_area, prev_error_turn, prev_time_pd
        
        current_time_pd = time.time()
        time_delta = current_time_pd - prev_time_pd
        if time_delta == 0: time_delta = 1e-6

        x, y, w, h = [int(v) for v in bbox]
        cx, area = x + w//2, w*h
        
        # --- THUẬT TOÁN CŨ CỦA BẠN ---
        # error = area - TARGET
        # Nếu xa (area < target) -> error âm -> Speed âm
        # (Giả định phần cứng của bạn: Speed âm là Tiến)
        error_area = area - TARGET_AREA 
        error_turn = FRAME_CENTER_X - cx
        
        # D-term
        derivative_area = (error_area - prev_error_area) / time_delta
        derivative_turn = (error_turn - prev_error_turn) / time_delta

        # Output PID
        fwd_speed = (KP_DISTANCE * error_area) + (KD_DISTANCE * derivative_area)
        fwd_speed = clamp(fwd_speed, -MAX_FWD_SPEED, MAX_FWD_SPEED)
        
        turn_speed = (KP_TURN * error_turn) + (KD_TURN * derivative_turn)
        turn_speed = clamp(turn_speed, -MAX_TURN_SPEED, MAX_TURN_SPEED)

        prev_error_area = error_area
        prev_error_turn = error_turn
        prev_time_pd = current_time_pd
        
        set_robot_pwm(fwd_speed + turn_speed, fwd_speed - turn_speed, "PD_FOLLOW")

    while True:
        success, image = cap.read()
        if not success: time.sleep(0.1); continue
        
        frame_count += 1
        image = cv2.flip(image, 1) 
        
        with lock:
            curr_state = robot_state
            curr_target_id = target_person_id
            curr_target_name = target_person_name
        
        boxes_to_send = []

        if curr_state == "IDLE":
            results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, tracker="my_tracker.yaml")
            latest_bboxes.clear()
            
            if results[0].boxes and results[0].boxes.id is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
                ids = results[0].boxes.id.cpu().numpy().astype(int)
                
                for box, trk_id in zip(boxes, ids):
                    x1, y1, x2, y2 = box
                    # Cache vị trí box để click là follow ngay
                    latest_bboxes[int(trk_id)] = (x1, y1, x2-x1, y2-y1)
                    
                    name = face_recognition_cache.get(trk_id, None)
                    if name is None or frame_count % RECOGNIZE_FACE_INTERVAL == 0:
                        try:
                            crop = image[y1:y2, x1:x2]
                            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                            name = recognize_face(rgb)
                            face_recognition_cache[trk_id] = name
                        except: pass
                    
                    # Logic tự tìm lại (Re-acquire)
                    if is_searching_for_target and name == curr_target_name:
                        print(f"FOUND LOST TARGET: {name}. RESUMING...")
                        with lock:
                            target_person_id = int(trk_id) 
                            robot_state = "FOLLOWING"      
                            is_searching_for_target = False
                            tracker = None                 
                        break 

                    boxes_to_send.append({'id': int(trk_id), 'rect': [int(x1), int(y1), int(x2), int(y2)]})
            
            set_robot_pwm(0, 0, "IDLE")

        elif curr_state == "MANUAL":
            execute_robot_move(manual_command, "MANUAL")
            is_searching_for_target = False 

        elif curr_state == "FOLLOWING":
            lost_tracking = False
            
            # Cố gắng học tên nếu chưa biết
            if curr_target_name == "Unknown" and frame_count % 5 == 0:
                 pass 
            
            if tracker is None:
                init_box = None
                if curr_target_id in latest_bboxes:
                    init_box = latest_bboxes[curr_target_id]
                else:
                    results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, tracker="my_tracker.yaml")
                    if results[0].boxes and results[0].boxes.id is not None:
                         ids = results[0].boxes.id.cpu().numpy().astype(int)
                         boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
                         if curr_target_id in ids:
                             idx = np.where(ids == curr_target_id)[0][0]
                             x1, y1, x2, y2 = boxes[idx]
                             init_box = (x1, y1, x2-x1, y2-y1)
                
                if init_box:
                    tracker = create_tracker()
                    tracker.init(image, init_box)
                    run_pd_controller(init_box) 
                    
                    if curr_target_name == "Unknown":
                        try:
                            x, y, w, h = init_box
                            crop = image[y:y+h, x:x+w]
                            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                            found_name = recognize_face(rgb)
                            if found_name != "Unknown":
                                with lock: 
                                    target_person_name = found_name
                                    face_recognition_cache[curr_target_id] = found_name
                        except: pass
                else:
                    print(f"FAILED TO INIT TRACKER for ID {curr_target_id}")
                    lost_tracking = True
            else:
                success, box = tracker.update(image)
                if success:
                    run_pd_controller(box) 
                    x,y,w,h = [int(v) for v in box]
                    cv2.rectangle(image, (x,y), (x+w, y+h), (0,255,0), 3)
                    
                    # Gửi box về web (Fix lỗi blank khi follow)
                    boxes_to_send.append({'id': curr_target_id, 'rect': [x, y, x+w, y+h]})

                    if curr_target_name == "Unknown" and frame_count % 5 == 0:
                        try:
                            x, y, w, h = [int(v) for v in box]
                            h_img, w_img, _ = image.shape
                            x = max(0, x); y = max(0, y)
                            w = min(w, w_img - x); h = min(h, h_img - y)
                            if w > 20 and h > 20:
                                crop = image[y:y+h, x:x+w]
                                rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                                found_name = recognize_face(rgb)
                                if found_name != "Unknown":
                                    with lock: 
                                        target_person_name = found_name
                                        face_recognition_cache[curr_target_id] = found_name
                        except: pass

                    if frame_count % RE_DETECT_INTERVAL == 0:
                        results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, tracker="my_tracker.yaml")
                        found_yolo = False
                        if results[0].boxes and results[0].boxes.id is not None:
                            if curr_target_id in results[0].boxes.id.cpu().numpy().astype(int):
                                found_yolo = True
                        if not found_yolo:
                             lost_tracking = True
                else:
                    lost_tracking = True

            if lost_tracking:
                with lock:
                    robot_state = "IDLE"
                    tracker = None
                    # Nếu có tên thì tìm lại, không thì thôi
                    if curr_target_name and curr_target_name != "Unknown":
                        is_searching_for_target = True 
                    else:
                        is_searching_for_target = False 
                set_robot_pwm(0, 0, "LOST")

        if frame_count % 5 == 0:
            socketio.emit('robot_info', {
                'fps': 0, 'state': curr_state, 'target_id': curr_target_id
            })
            socketio.emit('detected_boxes', {'boxes': boxes_to_send})

        with lock:
            _, buf = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            global_frame = buf.tobytes()

@app.route('/')
def index(): return render_template('index2.html')

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            with lock: f = global_frame
            if f: yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + f + b'\r\n')
            else: time.sleep(0.05)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('update_manual_speed')
def handle_speed_update(data):
    global MAX_FWD_SPEED, MAX_TURN_SPEED
    try:
        new_speed = int(data.get('speed', 180))
        MAX_FWD_SPEED = new_speed
        MAX_TURN_SPEED = min(new_speed + 20, 255)
    except: pass

@socketio.on('robot_command')
def handle_cmd(data):
    global robot_state, manual_command, is_searching_for_target
    cmd = data.get('command')
    if cmd.startswith('MANUAL_'):
        with lock:
            if robot_state != "MANUAL": 
                robot_state = "MANUAL"
                set_failsafe_distance_cm(SAFE_DIST_MANUAL) 
            manual_command = cmd.split('_')[1]
            is_searching_for_target = False

@socketio.on('set_mode_manual')
def set_manual():
    global robot_state, is_searching_for_target
    with lock: 
        robot_state = "MANUAL"
        is_searching_for_target = False
    set_failsafe_distance_cm(SAFE_DIST_MANUAL)

@socketio.on('set_mode_idle')
def set_idle():
    global robot_state, is_searching_for_target
    with lock: 
        robot_state = "IDLE"
        is_searching_for_target = False
    set_failsafe_distance_cm(SAFE_DIST_AUTO)

@socketio.on('cancel_target')
def cancel_follow():
    # --- LOGIC FIX NÚT HỦY ---
    global robot_state, target_person_id, tracker, target_person_name, is_searching_for_target
    with lock:
        robot_state = "IDLE"
        target_person_id = None
        target_person_name = None # Xóa tên để không tìm lại
        tracker = None
        is_searching_for_target = False # Tắt cờ tìm kiếm
    set_robot_pwm(0, 0, "FORCE_STOP")
    set_failsafe_distance_cm(SAFE_DIST_AUTO)
    print(">>> CANCEL FOLLOW: Reset to IDLE")

@socketio.on('set_target_id')
def set_target(data):
    global robot_state, target_person_id, target_person_name, tracker, face_recognition_cache, is_searching_for_target, current_python_failsafe_cm
    tid = int(data.get('id'))
    with lock:
        target_person_id = tid
        name = face_recognition_cache.get(tid, "Unknown")
        target_person_name = name
        
        robot_state = "FOLLOWING"
        tracker = None
        is_searching_for_target = False 
        current_python_failsafe_cm = SAFE_DIST_AUTO 
    set_failsafe_distance_cm(SAFE_DIST_AUTO)
    
@socketio.on('toggle_light')
def toggle_light():
    global light_state
    light_state = not light_state
    toggle_light_relay(light_state)

if __name__ == '__main__':
    load_known_faces()
    
    t1 = threading.Thread(target=robot_logic_thread, daemon=True)
    t2 = threading.Thread(target=serial_read_thread, daemon=True)
    
    if not t1.is_alive(): 
        try: t1.start()
        except RuntimeError: pass
    if not t2.is_alive(): 
        try: t2.start()
        except RuntimeError: pass

    socketio.run(app, host='0.0.0.0', port=5001, debug=False)