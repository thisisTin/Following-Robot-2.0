# --- 1. Imports ---
import cv2
from ultralytics import YOLO
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial
from rplidar import RPLidar

# --- 1. DEFINE SERIAL PORT ---
LIDAR_PORT = '/dev/ttyUSB0' 
SERIAL_PORT = '/dev/ttyUSB1' 
BAUD_RATE = 9600             

# --- 2. AI Model Initialization ---
print("Loading AI Models...")
model = YOLO('yolov8n.pt') 
print("Models loaded successfully.")

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

# --- Biáº¿n Lidar ---
lidar = None
MIN_SAFE_DISTANCE = 0.5 # (mÃ©t)
lidar_scan_data = {
    'front_distance': float('inf') 
}

# --- CÃC Háº°NG Sá» ÄIá»€U KHIá»N P-CONTROLLER (PID) ---
# (Báº¡n Cáº¦N tinh chá»nh cÃ¡c giÃ¡ trá» nÃ y)
KP_DISTANCE = 0.003  # Háº±ng sá» P cho khoáº£ng cÃ¡ch (Area)
KP_TURN = 0.4        # Háº±ng sá» P cho ráº½ (X)

MAX_FWD_SPEED = 200  # Tá»c Äá» tiáº¿n/lÃ¹i tá»i Äa (PWM)
MAX_TURN_SPEED = 150 # Tá»c Äá» ráº½ tá»i Äa (PWM)

# Serial Communication
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  
    print(f"Serial Port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None 

# --- 5. Robot Hardware Functions (ÄÃ£ cáº¥u trÃºc láº¡i) ---

def clamp(n, minn, maxn):
    """HÃ m tiá»n Ã­ch Äá» giá»i háº¡n má»t giÃ¡ trá» trong má»t khoáº£ng."""
    return max(min(maxn, n), minn)

def set_robot_pwm(left_pwm, right_pwm, intent=""):
    """
    HÃ m cáº¥p tháº¥p: Gá»­i PWM cuá»i cÃ¹ng sau khi kiá»m tra Lidar.
    ÄÃ¢y lÃ  hÃ m DUY NHáº¤T ÄÆ°á»£c phÃ©p nÃ³i chuyá»n vá»i Serial.
    """
    global ser, lidar_scan_data, lock

    left_pwm = int(left_pwm)
    right_pwm = int(right_pwm)

    # --- LOGIC NÃ Váº¬T Cáº¢N (LIDAR) (ÄÃ KÃCH HOáº T) ---
    current_front_distance = float('inf')
    with lock:
        current_front_distance = lidar_scan_data.get('front_distance', float('inf'))

    # Kiá»m tra náº¿u báº¥t ká»³ bÃ¡nh nÃ o Äang cá» Äi tá»i
    is_moving_forward = left_pwm > 0 or right_pwm > 0

    if is_moving_forward and current_front_distance < MIN_SAFE_DISTANCE:
        print(f"LIDAR OVERRIDE: Obstacle detected at {current_front_distance:.2f}m! Stopping.")
        left_pwm = 0  # Ghi ÄÃ¨ lá»nh
        right_pwm = 0 # Ghi ÄÃ¨ lá»nh
        intent = f"LIDAR_STOP (was {intent})"
    # --- Káº¾T THÃC LOGIC LIDAR ---

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
    HÃ m cáº¥p cao: Dá»ch lá»nh (FORWARD, LEFT...) tá»« Joystick thÃ nh PWM.
    HÃ m nÃ y chá» dÃ¹ng cho JOYSTICK.
    """
    if intent == "": intent = command

    # Tá»c Äá» cá» Äá»nh cho Joystick
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
    
    # Gá»i hÃ m cáº¥p tháº¥p (ÄÃ£ bao gá»m Lidar check)
    set_robot_pwm(left_pwm, right_pwm, intent)


def toggle_light_relay(new_state):
# ... (HÃ m nÃ y giá»¯ nguyÃªn) ...
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
# ... (HÃ m nÃ y giá»¯ nguyÃªn) ...
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
# ... (HÃ m nÃ y giá»¯ nguyÃªn) ...
    global lidar, lidar_scan_data, lock
    try:
        print("Connecting to Lidar...")
        lidar = RPLidar(LIDAR_PORT)
        print("Lidar connected successfully.")
        
        for scan in lidar.iter_scans(scan_type='normal', min_len=100):
            front_distance_mm = float('inf')
            
            for quality, angle, distance in scan:
                if (0 <= angle <= 15) or (345 <= angle <= 360):
                    if distance > 0: 
                        if distance < front_distance_mm:
                            front_distance_mm = distance
            
            with lock:
                if front_distance_mm == float('inf'):
                    lidar_scan_data['front_distance'] = float('inf') 
                else:
                    lidar_scan_data['front_distance'] = front_distance_mm / 1000.0
            time.sleep(0.01) 

    except Exception as e:
        print(f"Error connecting or reading Lidar: {e}")
        if lidar: lidar.stop(); lidar.disconnect()
    finally:
        if lidar: lidar.stop(); lidar.disconnect()

def robot_logic_thread():
    global global_frame, robot_state, manual_command, light_state, target_person_id

    # --- Sá»¬A Lá»I CAMERA ---
    # Khá»i táº¡o camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Bá» qua kiá»m tra ban Äáº§u, di chuyá»n vÃ o vÃ²ng láº·p
    # success, img = cap.read()
    # if not success:
    #     print("FATAL: Cannot read camera, check connection.")
    #     return # <-- ÄÃ¢y lÃ  lá»i, xÃ³a bá»
        
    FRAME_HEIGHT, FRAME_WIDTH = 480, 640 # Äáº·t thá»§ cÃ´ng
    
    # --- NGÆ¯á» NG ÄIá»€U KHIá»N P-CONTROLLER ---
    # NgÆ°á»¡ng X (trÃ¡i/pháº£i)
    FRAME_CENTER_X = FRAME_WIDTH / 2
    
    # NgÆ°á»¡ng Area (khoáº£ng cÃ¡ch) - TÃNH Báº°NG DIá»N TÃCH BOX
    # (Báº¡n Cáº¦N tinh chá»nh giÃ¡ trá» nÃ y)
    # ÄÃ¢y lÃ  diá»n tÃ­ch box "lÃ½ tÆ°á»ng" mÃ  robot sáº½ cá» gáº¯ng duy trÃ¬
    TARGET_AREA = (FRAME_WIDTH * FRAME_HEIGHT) * 0.25 

    prev_frame_time = 0
    print("Robot logic thread started...")
    
    frame_count = 0
    AI_SKIP_FRAMES = 3  
    INFO_SKIP_FRAMES = 15 
    
    # *** THAY Äá»I: KhÃ´ng cáº§n last_ai_command ná»¯a ***
    # (ChÃºng ta sáº½ tÃ­nh PWM á» má»i khung hÃ¬nh, nhÆ°ng chá» cháº¡y AI á» skip frame)
    
    # Biáº¿n lÆ°u trá»¯ P-Controller
    last_known_area = 0
    last_known_centerX = FRAME_CENTER_X
    
    jpeg_quality = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

    while True:
        # --- Sá»¬A Lá»I CAMERA ---
        # Kiá»m tra xem camera cÃ³ Äang má» khÃ´ng
        if not cap.isOpened():
            print("Camera not open. Trying to reconnect...")
            cap.release()
            cap = cv2.VideoCapture(0) # Cá» gáº¯ng káº¿t ná»i láº¡i
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            time.sleep(1)
            continue # Bá» qua vÃ²ng láº·p nÃ y

        success, image = cap.read()
        if not success:
            print("Camera read failed, skipping frame.")
            time.sleep(1)
            continue # Bá» qua vÃ²ng láº·p nÃ y
        # --- Káº¾T THÃC Sá»¬A Lá»I ---
            
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
            
            # Cháº¡y AI ná»n Äá» hiá»n thá» box
            if run_ai_this_frame:
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.5, tracker="bytetrack.yaml")
                if results[0].boxes and results[0].boxes.id is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        box_id = int(box.id[0])
                        
                        # --- Sá»¬A Lá»I JSON: Ãp kiá»u numpy.int64 vá» int ---
                        rect_list = [int(x1), int(y1), int(x2), int(y2)]
                        boxes_to_send.append({'id': box_id, 'rect': rect_list})
                        # --- Káº¾T THÃC Sá»¬A Lá»I ---

                        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        cv2.putText(image, f"ID: {box_id}", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # á» cháº¿ Äá» MANUAL, chá» thá»±c thi lá»nh Joystick
            execute_robot_move(current_manual_cmd, "MANUAL_JOYSTICK")
            
            # Reset P-Controller
            last_known_area = 0
            last_known_centerX = FRAME_CENTER_X


        elif current_state == "FOLLOWING":
            hud_color = (0, 250, 0) # Green
            found_target_this_frame = False # Cá» kiá»m tra
            
            # YÃªu cáº§u: Pháº£i cÃ³ má»¥c tiÃªu má»i bÃ¡m theo
            if current_target_id is None:
                hud_text = "FOLLOWING (No Target)"
                set_robot_pwm(0, 0, "STOP (No Target)")
            
            # Chá» cháº¡y AI (náº·ng) trÃªn cÃ¡c khung hÃ¬nh skip
            elif run_ai_this_frame:
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.5, tracker="bytetrack.yaml")
                
                if results[0].boxes and results[0].boxes.id is not None:
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        box_id = int(box.id[0])

                        # --- Sá»¬A Lá»I JSON: Ãp kiá»u numpy.int64 vá» int ---
                        rect_list = [int(x1), int(y1), int(x2), int(y2)]
                        boxes_to_send.append({'id': box_id, 'rect': rect_list})
                        # --- Káº¾T THÃC Sá»¬A Lá»I ---
                        
                        # KIá»M TRA Má»¤C TIÃU
                        if box_id == current_target_id:
                            found_target_this_frame = True
                            hud_text = f"FOLLOWING ID: {box_id}"
                            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                            
                            # --- Cáº¬P NHáº¬T BIáº¾N P-CONTROLLER ---
                            last_known_centerX = (x1 + x2) / 2
                            last_known_area = (x2 - x1) * (y2 - y1)
                                
                        else:
                            # Váº½ box ngÆ°á»i khÃ¡c (mÃ u vÃ ng)
                            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                
                if not found_target_this_frame:
                    # Máº¥t dáº¥u má»¥c tiÃªu trong khung hÃ¬nh AI nÃ y
                    last_known_area = 0 # Dá»«ng láº¡i
            
            # --- LOGIC P-CONTROLLER (Cháº¡y á» Má»I khung hÃ¬nh) ---
            # LuÃ´n tÃ­nh toÃ¡n PWM dá»±a trÃªn dá»¯ liá»u *cuá»i cÃ¹ng* nhÃ¬n tháº¥y
            
            if last_known_area == 0:
                # ÄÃ£ máº¥t dáº¥u, dá»«ng láº¡i
                set_robot_pwm(0, 0, "STOP (Lost Target)")
            else:
                # 1. TÃ­nh toÃ¡n Tá»c Äá» Tiáº¿n/LÃ¹i (P-Distance)
                error_area = TARGET_AREA - last_known_area
                fwd_speed = KP_DISTANCE * error_area
                # Giá»i háº¡n tá»c Äá»
                fwd_speed = clamp(fwd_speed, -MAX_FWD_SPEED, MAX_FWD_SPEED)
                
                # 2. TÃ­nh toÃ¡n Tá»c Äá» Ráº½ (P-Turning)
                error_turn = FRAME_CENTER_X - last_known_centerX
                turn_speed = KP_TURN * error_turn
                # Giá»i háº¡n tá»c Äá»
                turn_speed = clamp(turn_speed, -MAX_TURN_SPEED, MAX_TURN_SPEED)
                
                # 3. Káº¿t há»£p 2 tá»c Äá»
                left_pwm = fwd_speed + turn_speed
                right_pwm = fwd_speed - turn_speed
                
                # 4. Giá»i háº¡n PWM cuá»i cÃ¹ng
                left_pwm = clamp(left_pwm, -255, 255)
                right_pwm = clamp(right_pwm, -255, 255)

                # 5. Gá»­i lá»nh PWM (ÄÃ£ bao gá»m Lidar check)
                set_robot_pwm(left_pwm, right_pwm, "PID_FOLLOW")


        # --- HUD & Frame Update ---
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time) if (new_frame_time - prev_frame_time) > 0 else 0
        prev_frame_time = new_frame_time

        cv2.putText(image, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        cv2.putText(image, f"FPS: {int(fps)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        
        # --- Gá»¬I Dá»® LIá»U LÃN WEB ---
        if frame_count % INFO_SKIP_FRAMES == 0:
            with lock:
                current_light_state = light_state  
            socketio.emit('robot_info', {'fps': int(fps), 'state': current_state, 'light': current_light_state})
        
        if run_ai_this_frame and len(boxes_to_send) > 0:
            socketio.emit('detected_boxes', {'boxes': boxes_to_send})

        with lock:
            _, buffer = cv2.imencode('.jpg', image, jpeg_quality)
            global_frame = buffer.tobytes()

# --- 7. Flask HTTP Routes (Giá»¯ nguyÃªn) ---
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

# --- 8. Socket.IO Events (Giá»¯ nguyÃªn) ---
@socketio.on('connect')
def handle_connect():
    print('Client connected!')
    with lock:
        emit('robot_info', {'fps': 0, 'state': robot_state, 'light': light_state})

@socketio.on('robot_command')
def handle_robot_command(data):
    global robot_state, manual_command, target_person_id
    command = data.get('command')
    print(f"Web command received: {command}")

    with lock:
        if command == 'TOGGLE_FOLLOW':
            if robot_state == "FOLLOWING": 
                robot_state = "MANUAL" 
                target_person_id = None 
            else: 
                robot_state = "MANUAL" 
                print("Waiting for target selection...")
                
        elif command == 'SET_MANUAL': 
            robot_state = "MANUAL"
            target_person_id = None 
                
        elif command.startswith('MANUAL_'):
            if robot_state == "MANUAL":
                manual_command = command.split('_')[1] 
                
    with lock:
        emit('robot_info', {'fps': 0, 'state': robot_state, 'light': light_state}, broadcast=True)

@socketio.on('set_target_id')
def handle_set_target(data):
    global robot_state, target_person_id
    target_id = data.get('id')
    
    if target_id is not None:
        with lock:
            target_person_id = int(target_id)
            robot_state = "FOLLOWING" 
            print(f"*** NEW TARGET ACQUIRED: ID {target_person_id} ***")
            
        emit('robot_info', {'fps': 0, 'state': "FOLLOWING", 'light': light_state}, broadcast=True)

@socketio.on('cancel_target')
def handle_cancel_target():
    global robot_state, target_person_id
    with lock:
        print(f"*** TARGET CANCELED: ID {target_person_id} ***")
        target_person_id = None
        robot_state = "MANUAL" 
        
    # Gá»­i láº¡i state má»i cho cÃ¡c client
    emit('robot_info', {'fps': 0, 'state': "MANUAL", 'light': light_state}, broadcast=True)


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