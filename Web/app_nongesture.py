# --- 1. Imports ---
import cv2
from ultralytics import YOLO
# import mediapipe as mp  # (Tạm tắt - Lỗi trên Pi 5)
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial
# (Uncomment this when on Raspberry Pi)
# import RPi.GPIO as GPIO

# --- 2. AI Model Initialization ---
print("Loading AI Models...")
model = YOLO('yolov8n.pt') 
# (Tạm tắt MediaPipe)
# mp_hands = mp.solutions.hands
# hands = mp_hands.Hands(min_detection_confidence=0.6, min_tracking_confidence=0.5, max_num_hands=1)
# mp_drawing = mp.solutions.drawing_utils
print("Models loaded successfully (MediaPipe DISABLED).")

# --- 3. Web Server Initialization ---
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_very_secret_key'
socketio = SocketIO(app, async_mode='threading')

# --- 4. Global Variables & Serial Initialization ---
global_frame = None                  # Stores the latest camera frame for streaming
robot_state = "IDLE"                 # Current robot mode (IDLE, FOLLOWING, MANUAL)
manual_command = "STOP"              # Current manual joystick command
lock = threading.Lock()              # Thread lock to protect shared variables

# Serial Communication Setup (Adjust port name as needed for your OS)
SERIAL_PORT = '/dev/ttyUSB1' # MAC '/dev/ttyACM0' <-- SỬA CỔNG NÀY
BAUD_RATE = 9600             # (Đang là 9600, khớp với ESP32)
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)  
    print(f"Serial Port {SERIAL_PORT} opened successfully at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None # Set to None if serial failed

# (Tạm tắt Gesture)
# gesture_timer = 0
# GESTURE_HOLD_TIME = 3              # seconds
# current_gesture = None
# last_gesture_for_timer = None

# Hardware state variables
light_state = False                  # Tracks if the light relay is ON or OFF
# LIGHT_PIN = 17

# --- 5. Robot Hardware Functions (Serial Implementation) ---
# (Giữ nguyên các hàm execute_robot_move, toggle_light_relay)

def execute_robot_move(command):
    """ Executes the actual motor commands by sending PWM values via Serial. """
    global ser
    
    # (Sử dụng tốc độ cao 190/220 như đã sửa)
    SPEED = 190
    TURN_SPEED = 220
    CURVE_SPEED_SLOW = int(SPEED * 0.5)  
    CURVE_SPEED_FAST = SPEED
    
    cmd_map = {
        "FORWARD": (SPEED, SPEED),
        "LEFT": (-TURN_SPEED, TURN_SPEED),      # Pivot Left
        "RIGHT": (TURN_SPEED, -TURN_SPEED),     # Pivot Right
        "BACKWARD": (-SPEED, -SPEED),  
        "FORWARD_LEFT": (CURVE_SPEED_SLOW, CURVE_SPEED_FAST),  # Curved turn left
        "FORWARD_RIGHT": (CURVE_SPEED_FAST, CURVE_SPEED_SLOW), # Curved turn right
        "BACKWARD_LEFT": (-CURVE_SPEED_FAST, -CURVE_SPEED_SLOW),
        "BACKWARD_RIGHT": (-CURVE_SPEED_SLOW, -CURVE_SPEED_FAST),
        "STOP": (0, 0)
    }
    
    left_pwm, right_pwm = cmd_map.get(command, (0, 0))

    # Construct Serial Command: MOVE:left_speed:right_speed\n
    serial_command = f"MOVE:{left_pwm}:{right_pwm}\n"  
    
    if ser:
        try:
            ser.write(serial_command.encode())
            # This is the line sent to ESP32's Serial Monitor
            # (Giảm log để đỡ spam console)
            if command != "STOP":
                 print(f"SERIAL SENT: {serial_command.strip()} -> L_PWM:{left_pwm} R_PWM:{right_pwm}")  
        except Exception as e:
            print(f"Serial write error: {e}")
    else:
        # Sửa lại log giả lập cho đúng
        if command != "STOP":
            print(f"ROBOT SIMULATED: {command} -> L_PWM:{left_pwm} R_PWM:{right_pwm}")


def toggle_light_relay(new_state):
    """Controls the light relay via Serial."""
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

# (TAm tắt Gesture)
# def count_fingers(hand_landmarks):
#     ... (logic)

# --- 6. Main Robot Logic Thread ---
def serial_read_thread():
    """Reads all incoming data from the ESP32 Serial port and prints it."""
    global ser
    if not ser:
        print("Serial reader thread failed: Serial port is not open.")
        return

    print("Serial reading thread started...")
    while True:
        try:
            if ser.in_waiting > 0:
                # Read line until '\n'
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # Log the response received from ESP32
                    print(f"ESP32 RESPONSE: {line}")  
            time.sleep(0.01) # Small delay to prevent high CPU usage
        except Exception as e:
            print(f"Error reading from serial: {e}")
            time.sleep(1)

def robot_logic_thread():
    """ This function runs in a separate thread, handling all AI, Camera, and Robot control logic. """
    global global_frame, robot_state, manual_command, light_state
    # (Tạm tắt Gesture)
    # global gesture_timer, current_gesture, last_gesture_for_timer

    # Initialize Camera
    cap = cv2.VideoCapture(0)
    
    # *** OPTIMIZATION 1: Set smaller camera resolution ***
    # Yêu cầu camera chỉ cung cấp 640x480 thay vì 1080p hoặc 720p
    # Giúp cap.read() nhanh hơn
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    success, img = cap.read()
    if not success:
        print("FATAL: Cannot read camera, check connection.")
        return
        
    FRAME_HEIGHT, FRAME_WIDTH, _ = img.shape
    # Defines the "center" zone for following
    ZONE_LEFT = FRAME_WIDTH * 0.25
    ZONE_RIGHT = FRAME_WIDTH * 0.75
    
    prev_frame_time = 0
    print("Robot logic thread started...")
    
    # --- Biến cho Tối ưu FPS ---
    frame_count = 0
    
    # *** OPTIMIZATION 2: AI Skip-Frame settings ***
    # Chỉ chạy AI (YOLO) mỗi 3 khung hình
    AI_SKIP_FRAMES = 3  
    # Gửi thông tin (FPS, state) qua socket mỗi 15 khung hình
    INFO_SKIP_FRAMES = 15 
    
    # Lưu trữ lệnh AI cuối cùng. Robot sẽ tiếp tục chạy lệnh này
    # ngay cả trong các khung hình "bỏ qua"
    last_ai_command = "STOP"
    # Lưu trữ bounding box cuối cùng để vẽ lên các khung hình bỏ qua
    last_known_boxes = [] 
    
    # *** OPTIMIZATION 3: JPEG Quality settings ***
    # Giảm chất lượng JPEG xuống 80% để tăng tốc cv2.imencode
    # 75-85 là một mức cân bằng tốt
    jpeg_quality = [int(cv2.IMWRITE_JPEG_QUALITY), 80]


    while True:
        success, image = cap.read()
        if not success:
            print("Camera read failed, skipping frame.")
            time.sleep(0.1)
            continue
            
        frame_count += 1
        image = cv2.flip(image, 1) # Flip camera horizontally
        # (Tạm tắt Gesture)
        # image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Read shared variables safely
        with lock:
            current_state = robot_state
            current_manual_cmd = manual_command

        hud_text = f"STATE: {current_state}"
        hud_color = (0, 0, 255) # Default: Red (STOP/IDLE)
        # (Tạm tắt Gesture)
        # current_gesture = None
        
        # --- (Tạm tắt Gesture) Gesture Recognition ---
        # if current_state != "MANUAL":
        #    ... (logic)

        # --- State Machine Logic ---
        if current_state == "IDLE":
            hud_color = (0, 255, 255) # Yellow
            execute_robot_move("STOP") # Send STOP command (PWM 0,0)
            last_ai_command = "STOP" # Đảm bảo lệnh AI cũng là STOP
            last_known_boxes = []    # Xóa các box cũ

        elif current_state == "FOLLOWING":
            hud_color = (0, 250, 0) # Green
            
            # *** OPTIMIZATION 2: Logic Skip-Frame ***
            run_ai_this_frame = (frame_count % AI_SKIP_FRAMES == 0)

            if run_ai_this_frame:
                # --- CHỈ CHẠY AI NẶNG NỀ TRONG KHUNG HÌNH NÀY ---
                results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.5)
                found_person = False
                
                # Xóa các box cũ trước khi tìm box mới
                last_known_boxes.clear() 
                
                if results[0].boxes:
                    for box in results[0].boxes:
                        found_person = True
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        
                        # Lưu box này để vẽ
                        last_known_boxes.append((x1, y1, x2, y2))
                        
                        centerX = (x1 + x2) / 2
                        
                        # Xác định lệnh AI
                        if centerX < ZONE_LEFT: 
                            last_ai_command = "LEFT"
                        elif centerX > ZONE_RIGHT: 
                            last_ai_command = "RIGHT"
                        else: 
                            last_ai_command = "FORWARD"
                        
                        break # Chỉ bám theo 1 người
                
                if not found_person:
                    last_ai_command = "STOP"
            
            # *** LUÔN LUÔN thực thi lệnh (ngay cả trên khung hình bỏ qua) ***
            # Robot sẽ tiếp tục rẽ trái/phải/tiến ngay cả khi AI không chạy
            # Điều này làm robot phản ứng mượt mà hơn
            execute_robot_move(last_ai_command)
            
            # --- Vẽ HUD ---
            # Vẽ các box đã lưu (từ khung hình AI gần nhất)
            for (x1, y1, x2, y2) in last_known_boxes:
                 cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Vẽ tracking zones
            cv2.line(image, (int(ZONE_LEFT), 0), (int(ZONE_LEFT), FRAME_HEIGHT), (255, 0, 0), 2)
            cv2.line(image, (int(ZONE_RIGHT), 0), (int(ZONE_RIGHT), FRAME_HEIGHT), (255, 0, 0), 2)

        elif current_state == "MANUAL":
            hud_color = (255, 0, 0) # Blue
            execute_robot_move(current_manual_cmd) 
            last_ai_command = "STOP" # Reset lệnh AI khi ở manual
            last_known_boxes = []    # Xóa các box cũ

        # --- HUD & Frame Update ---
        new_frame_time = time.time()
        # Handle division by zero for FPS
        if (new_frame_time - prev_frame_time) > 0:
            fps = 1 / (new_frame_time - prev_frame_time)
        else:
            fps = 0
            
        prev_frame_time = new_frame_time

        cv2.putText(image, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        cv2.putText(image, f"FPS: {int(fps)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hud_color, 2)
        
        # (Tạm tắt Gesture)
        # if gesture_timer > 0:
        #    ... (logic)

        # *** OPTIMIZATION 4: Giảm tần suất Socket.IO ***
        if frame_count % INFO_SKIP_FRAMES == 0:
            with lock:
                current_light_state = light_state  
            socketio.emit('robot_info', {'fps': int(fps), 'state': current_state, 'light': current_light_state})

        # Encode frame to JPEG and store in global variable for streaming
        with lock:
            # *** OPTIMIZATION 3: Sử dụng chất lượng JPEG thấp hơn ***
            _, buffer = cv2.imencode('.jpg', image, jpeg_quality)
            global_frame = buffer.tobytes()

# --- 7. Flask HTTP Routes (Unchanged) ---
@app.route('/')
def index():
    """ Serves the main HTML page. """
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """ Provides the JPEG video stream. """
    def gen_frames():
        global global_frame
        while True:
            with lock:
                frame_bytes = global_frame
            if frame_bytes is None:
                time.sleep(0.1)
                continue
            # Yield frame in multipart format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- 8. Socket.IO Events (Unchanged) ---
@socketio.on('connect')
def handle_connect():
    """ Handles a new client connection. """
    print('Client connected!')
    with lock:
        # Send current state on connect
        emit('robot_info', {'fps': 0, 'state': robot_state, 'light': light_state})

@socketio.on('robot_command')
def handle_robot_command(data):
    """ Handles commands from the web client (mode changes, joystick). """
    global robot_state, manual_command
    command = data.get('command')
    
    print(f"Web command received: {command}")

    with lock:
        # Nút bấm trên web VẪN HOẠT ĐỘNG
        if command == 'TOGGLE_FOLLOW':
            if robot_state == "FOLLOWING": robot_state = "IDLE"
            else: robot_state = "FOLLOWING"
                
        elif command == 'SET_MANUAL': robot_state = "MANUAL"
        elif command == 'SET_IDLE': robot_state = "IDLE"
                
        elif command.startswith('MANUAL_'):
            # This logic automatically handles new commands like 'MANUAL_FORWARD_LEFT'
            if robot_state == "MANUAL":
                manual_command = command.split('_')[1] # e.g., "FORWARD_LEFT"
                
    # Send updated state to all clients
    with lock:
        emit('robot_info', {'fps': 0, 'state': robot_state, 'light': light_state}, broadcast=True)

@socketio.on('toggle_light')
def handle_toggle_light():
    """ Handles the 'toggle_light' event from the button. """
    global light_state
    with lock:
        light_state = not light_state # Flip the state
        current_light_state = light_state
        current_state = robot_state
    
    # Send Serial command to ESP32
    toggle_light_relay(current_light_state)  
    
    # Emit the new state to all clients
    emit('robot_info', {'fps': 0, 'state': current_state, 'light': current_light_state}, broadcast=True)

# --- 9. Start Application ---
if __name__ == '__main__':
    print("Starting Robot Thread...")
    robot_thread = threading.Thread(target=robot_logic_thread, daemon=True)
    robot_thread.start()
    
    # *** BUG FIX: Thêm code để khởi động luồng đọc Serial ***
    if ser:
        serial_thread = threading.Thread(target=serial_read_thread, daemon=True)
        serial_thread.start()
    
    print("Starting Web Server at http://0.0.0.0:5001")
    # Using port 5001 to avoid macOS AirPlay conflict
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)