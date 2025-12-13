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

# --- 2. AI Model ---
print("Loading AI Models...")
try:
    model = YOLO('yolov8n.pt') 
    print("Models loaded.")
except Exception as e:
    model = None 

# --- 3. Web Server ---
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret'
socketio = SocketIO(app, async_mode='threading')

# --- 4. Global Variables ---
global_frame = None                  
robot_state = "IDLE"                 
manual_command = "STOP"              
lock = threading.Lock()              
light_state = False                  

# --- Tracking ---
target_person_id = None              
target_person_name = None            # Lưu tên để tìm lại
tracker = None                       
# Cờ này bật lên khi mất dấu để báo hiệu robot đang tìm lại mục tiêu cũ
is_reacquiring = False               

# --- Face ID ---
ENCODINGS_DIR = "Register-ID"        
known_face_data = []                 
face_recognition_cache = {}          
RECOGNIZE_FACE_INTERVAL = 10         

# --- Sensors & Failsafe ---
sensor_data = {'front': 999.0, 'back': 999.0}
SAFE_DIST_AUTO = 50     
SAFE_DIST_MANUAL = 15   
current_failsafe_cm = SAFE_DIST_AUTO 

# --- PID & SPEED ---
TARGET_AREA = 40000       

KP_DISTANCE = 0.0228
KD_DISTANCE = 0.7747       
KP_TURN = 0.45
KD_TURN = 0.08             

MAX_FWD_SPEED = 195        
MAX_TURN_SPEED = 210       
MIN_MOVE_PWM = 188         
RE_DETECT_INTERVAL = 30        

# --- Serial ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  
    print(f"Serial Port {SERIAL_PORT} opened.")
    time.sleep(2) 
except: ser = None 

# --- 5. Helper Functions ---

def clamp(n, minn, maxn): return max(min(maxn, n), minn)

def set_failsafe_distance_cm(dist_cm):
    global current_failsafe_cm
    current_failsafe_cm = dist_cm
    if ser:
        try: ser.write(f"SET_STOP_DIST:{dist_cm}\n".encode())
        except: pass

def set_robot_pwm(left_pwm, right_pwm, intent=""):
    global ser, sensor_data, lock, current_failsafe_cm, MIN_MOVE_PWM
    left_pwm, right_pwm = int(left_pwm), int(right_pwm)
    
    with lock:
        dist_front = sensor_data['front']
        dist_back = sensor_data['back']
    
    # --- FAILSAFE THÔNG MINH ---
    # Chặn TIẾN (>0) nếu vướng trước
    if (left_pwm > 0 or right_pwm > 0) and dist_front < current_failsafe_cm: 
        left_pwm, right_pwm = 0, 0 
    
    # Chặn LÙI (<0) nếu vướng sau
    if (left_pwm < 0 or right_pwm < 0) and dist_back < current_failsafe_cm: 
        left_pwm, right_pwm = 0, 0

    # Deadzone Boost
    def _boost(val):
        if 0 < val < MIN_MOVE_PWM: return MIN_MOVE_PWM
        if 0 > val > -MIN_MOVE_PWM: return -MIN_MOVE_PWM
        return val
    
    if left_pwm != 0: left_pwm = _boost(left_pwm)
    if right_pwm != 0: right_pwm = _boost(right_pwm)

    left_pwm = clamp(left_pwm, -255, 255)
    right_pwm = clamp(right_pwm, -255, 255)

    if ser:
        try: ser.write(f"MOVE:{left_pwm}:{right_pwm}\n".encode())
        except: pass
    
    return left_pwm, right_pwm

def execute_robot_move(command, intent=""):
    SPEED = MAX_FWD_SPEED 
    TURN_SPEED = MAX_TURN_SPEED
    CURVE = int(SPEED * 0.5)
    cmd_map = {
        "FORWARD": (SPEED, SPEED), "LEFT": (-TURN_SPEED, TURN_SPEED),
        "RIGHT": (TURN_SPEED, -TURN_SPEED), "BACKWARD": (-SPEED, -SPEED),  
        "FORWARD_LEFT": (CURVE, SPEED), "FORWARD_RIGHT": (SPEED, CURVE),
        "BACKWARD_LEFT": (-SPEED, -CURVE), "BACKWARD_RIGHT": (-CURVE, -SPEED),
        "STOP": (0, 0)
    }
    return set_robot_pwm(*cmd_map.get(command, (0, 0)), intent)

def toggle_light_relay(new_state):
    global ser
    cmd = "LIGHT:ON\n" if new_state else "LIGHT:OFF\n"
    if ser: ser.write(cmd.encode())

# --- 6. Threads ---

def serial_read_thread(): 
    global ser, sensor_data
    if not ser: return 
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
                            except: pass
            time.sleep(0.01) 
        except: time.sleep(0.1) 

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

def recognize_face(frame_crop_rgb):
    global known_face_data
    try:
        # Tăng upsample để bắt mặt tốt hơn
        face_locs = face_recognition.face_locations(frame_crop_rgb, number_of_times_to_upsample=2, model="hog")
        if not face_locs: return "Unknown"
        encoding = face_recognition.face_encodings(frame_crop_rgb, face_locs)[0]
        for p in known_face_data:
            matches = face_recognition.compare_faces(p["encodings"], encoding, tolerance=0.55)
            if True in matches: return p["name"]
        return "Unknown"
    except: return "Unknown"

# --- HELPER TRACKER ---
def create_tracker():
    try: return cv2.legacy.TrackerMOSSE_create()
    except: return cv2.TrackerCSRT_create()

# --- MAIN LOGIC ---
frame_count = 0 

def robot_logic_thread():
    global global_frame, robot_state, manual_command, model, target_person_id, target_person_name, tracker, frame_count, is_reacquiring

    if model is None: return
    cap = cv2.VideoCapture(0)
    cap.set(3, 640); cap.set(4, 480)
    FRAME_CENTER_X = 320
    
    prev_error_area = 0.0
    prev_error_turn = 0.0
    prev_time_pd = time.time()

    telemetry_data = {'area_input': 0, 'error': 0, 'pwm_fwd': 0, 'pwm_l': 0, 'pwm_r': 0, 'dist_front': 999.0}

    def run_pid(bbox):
        nonlocal prev_error_area, prev_error_turn, prev_time_pd, telemetry_data
        
        now = time.time()
        dt = now - prev_time_pd
        if dt == 0: dt = 1e-6

        x, y, w, h = [int(v) for v in bbox]
        cx, area = x + w//2, w*h
        
        # --- [SỬA HƯỚNG DI CHUYỂN] ---
        # TARGET - AREA:
        # Xa (Area nhỏ) -> Error DƯƠNG -> Speed DƯƠNG -> TIẾN
        # Gần (Area lớn) -> Error ÂM -> Speed ÂM -> LÙI
        error_area = TARGET_AREA - area 
        error_turn = FRAME_CENTER_X - cx
        
        d_area = (error_area - prev_error_area) / dt
        d_turn = (error_turn - prev_error_turn) / dt

        fwd = (KP_DISTANCE * error_area) + (KD_DISTANCE * d_area)
        turn = (KP_TURN * error_turn) + (KD_TURN * d_turn)
        
        fwd = clamp(fwd, -MAX_FWD_SPEED, MAX_FWD_SPEED)
        turn = clamp(turn, -MAX_TURN_SPEED, MAX_TURN_SPEED)

        prev_error_area = error_area
        prev_error_turn = error_turn
        prev_time_pd = now
        
        l_pwm, r_pwm = set_robot_pwm(fwd + turn, fwd - turn, "AUTO")
        
        telemetry_data.update({'area_input': int(area), 'error': int(error_area), 'pwm_fwd': int(fwd), 'pwm_l': l_pwm, 'pwm_r': r_pwm})

    while True:
        success, image = cap.read()
        if not success: time.sleep(0.1); continue
        
        frame_count += 1
        image = cv2.flip(image, 1) 
        telemetry_data['dist_front'] = sensor_data['front']
        
        with lock:
            curr_state = robot_state
            curr_target_id = target_person_id
            curr_target_name = target_person_name
        
        boxes_to_send = []

        if curr_state == "IDLE":
            telemetry_data.update({'pwm_l': 0, 'pwm_r': 0, 'error': 0, 'area_input': 0, 'pwm_fwd': 0})
            
            results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, tracker="my_tracker.yaml")
            
            if results[0].boxes and results[0].boxes.id is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
                ids = results[0].boxes.id.cpu().numpy().astype(int)
                
                for box, trk_id in zip(boxes, ids):
                    x1, y1, x2, y2 = box
                    name = face_recognition_cache.get(trk_id, None)
                    
                    if name is None or frame_count % RECOGNIZE_FACE_INTERVAL == 0:
                        try:
                            crop = image[y1:y2, x1:x2]
                            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                            name = recognize_face(rgb)
                            face_recognition_cache[trk_id] = name
                        except: pass
                    
                    # --- LOGIC TÌM LẠI NGƯỜI CŨ (Nguyên bản) ---
                    # Nếu đang ở chế độ "Tìm lại" (is_reacquiring) VÀ thấy đúng ID cũ HOẶC Tên cũ
                    # -> Tự động chuyển sang FOLLOW
                    if is_reacquiring:
                        match_id = (curr_target_id is not None and int(trk_id) == curr_target_id)
                        match_name = (curr_target_name is not None and name == curr_target_name and name != "Unknown")
                        
                        if match_id or match_name:
                            print(f"FOUND TARGET AGAIN: {name if name else trk_id}. RESUMING FOLLOW.")
                            with lock:
                                target_person_id = int(trk_id) # Cập nhật ID mới nếu YOLO đổi ID
                                robot_state = "FOLLOWING"
                                tracker = None # Để khởi tạo lại tracker mới
                            break # Thoát vòng lặp box để vào mode follow ngay

                    boxes_to_send.append({'id': int(trk_id), 'rect': [int(x1), int(y1), int(x2), int(y2)]})
            
            set_robot_pwm(0, 0, "IDLE")

        elif curr_state == "MANUAL":
            l, r = execute_robot_move(manual_command, "MANUAL")
            telemetry_data['pwm_l'] = l; telemetry_data['pwm_r'] = r

        elif curr_state == "FOLLOWING":
            if tracker is None:
                # Tìm box của ID mục tiêu để init tracker
                res = model.track(image, persist=True, verbose=False, imgsz=320, tracker="my_tracker.yaml")
                init_box = None
                if res[0].boxes and res[0].boxes.id is not None:
                    ids = res[0].boxes.id.cpu().numpy().astype(int)
                    if curr_target_id in ids:
                        idx = np.where(ids == curr_target_id)[0][0]
                        x1,y1,x2,y2 = res[0].boxes.xyxy.cpu().numpy().astype(int)[idx]
                        init_box = (x1, y1, x2-x1, y2-y1)
                
                if init_box:
                    tracker = create_tracker()
                    tracker.init(image, init_box)
                    run_pid(init_box)
                    
                    # Nếu chưa biết tên, thử nhận diện ngay
                    if curr_target_name is None or curr_target_name == "Unknown":
                        try:
                            x, y, w, h = init_box
                            crop = image[y:y+h, x:x+w]
                            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                            name = recognize_face(rgb)
                            if name != "Unknown":
                                with lock: target_person_name = name
                        except: pass
                else:
                    # KHÔNG TÌM THẤY -> VỀ IDLE NHƯNG BẬT CỜ "TÌM LẠI"
                    print("LOST TARGET ON INIT. SWITCHING TO IDLE TO SEARCH.")
                    with lock: 
                        robot_state = "IDLE"
                        tracker = None
                        is_reacquiring = True # Bật cờ tìm lại
            
            if tracker:
                ok, box = tracker.update(image)
                if ok:
                    run_pid(box)
                    x,y,w,h = [int(v) for v in box]
                    boxes_to_send.append({'id': curr_target_id, 'rect': [x, y, x+w, y+h]})
                    
                    # Update tên khi đang follow
                    if (curr_target_name is None or curr_target_name == "Unknown") and frame_count % 10 == 0:
                        try:
                            crop = image[y:y+h, x:x+w]
                            rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                            name = recognize_face(rgb)
                            if name != "Unknown":
                                with lock: target_person_name = name
                        except: pass

                    # Check YOLO định kỳ
                    if frame_count % RE_DETECT_INTERVAL == 0:
                        res = model.track(image, persist=True, verbose=False, imgsz=320, tracker="my_tracker.yaml")
                        found = False
                        if res[0].boxes and res[0].boxes.id is not None:
                            if curr_target_id in res[0].boxes.id.cpu().numpy().astype(int): found = True
                        
                        if not found:
                            print("YOLO LOST TARGET. SWITCHING TO IDLE TO SEARCH.")
                            with lock: 
                                robot_state = "IDLE"
                                tracker = None
                                is_reacquiring = True
                else:
                    # Tracker mất dấu -> VỀ IDLE ĐỂ TÌM
                    print("TRACKER LOST. SWITCHING TO IDLE TO SEARCH.")
                    with lock: 
                        robot_state = "IDLE"
                        tracker = None
                        is_reacquiring = True
            else:
                set_robot_pwm(0,0,"NO_TRACKER")

        # LUÔN GỬI TELEMETRY
        socketio.emit('telemetry', telemetry_data)
        
        if frame_count % 2 == 0: 
            socketio.emit('detected_boxes', {'boxes': boxes_to_send})
            socketio.emit('robot_info', {
                'state': 'SEARCHING' if (curr_state == 'IDLE' and is_reacquiring) else curr_state, 
                'target_id': curr_target_id
            })

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
def handle_speed(data):
    global MAX_FWD_SPEED, MAX_TURN_SPEED
    try:
        val = int(data.get('speed', 180))
        MAX_FWD_SPEED = val
        MAX_TURN_SPEED = min(val + 20, 255)
    except: pass

@socketio.on('robot_command')
def handle_cmd(data):
    global robot_state, manual_command, is_reacquiring
    cmd = data.get('command')
    if cmd.startswith('MANUAL_'):
        with lock:
            robot_state = "MANUAL"
            manual_command = cmd.split('_')[1]
            is_reacquiring = False # Tắt tìm kiếm khi lái tay
            set_failsafe_distance_cm(SAFE_DIST_MANUAL)

@socketio.on('set_mode_idle')
def set_idle():
    global robot_state, is_reacquiring
    with lock: 
        robot_state = "IDLE"
        is_reacquiring = False # Reset tìm kiếm
    set_failsafe_distance_cm(SAFE_DIST_AUTO)

@socketio.on('set_mode_manual')
def set_manual():
    global robot_state, is_reacquiring
    with lock: 
        robot_state = "MANUAL"
        is_reacquiring = False
    set_failsafe_distance_cm(SAFE_DIST_MANUAL)

@socketio.on('cancel_target')
def cancel():
    global robot_state, tracker, target_person_id, target_person_name, is_reacquiring
    with lock: 
        robot_state = "IDLE"
        tracker = None
        target_person_id = None
        target_person_name = None
        is_reacquiring = False # Hủy hoàn toàn
    set_robot_pwm(0,0,"STOP")
    set_failsafe_distance_cm(SAFE_DIST_AUTO)

@socketio.on('set_target_id')
def set_target(data):
    global robot_state, target_person_id, tracker, is_reacquiring
    tid = int(data.get('id'))
    with lock:
        target_person_id = tid
        robot_state = "FOLLOWING"
        tracker = None
        is_reacquiring = True # Bật cờ này để nếu mất dấu thì tự tìm lại
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
        except: pass
    if not t2.is_alive(): 
        try: t2.start()
        except: pass
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)