# --- 1. Imports ---
# import cv2  # (Tạm tắt AI)
# from ultralytics import YOLO # (Tạm tắt AI)
# import mediapipe as mp # (Tạm tắt AI)
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial
# (Uncomment this when on Raspberry Pi)
# import RPi.GPIO as GPIO

# --- 2. AI Model Initialization ---
print("Loading AI Models... (SKIPPED)")
# (Tạm tắt AI)
# print("Loading AI Models...")
# Initialize YOLO model for object tracking
# model = YOLO('yolov8n.pt') 
# Initialize MediaPipe Hands for gesture recognition
# mp_hands = mp.solutions.hands
# hands = mp_hands.Hands(min_detection_confidence=0.6, min_tracking_confidence=0.5, max_num_hands=1)
# mp_drawing = mp.solutions.drawing_utils
# print("Models loaded successfully.")

# --- 3. Web Server Initialization ---
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_very_secret_key'
socketio = SocketIO(app, async_mode='threading')

# --- 4. Global Variables & Serial Initialization ---
# global_frame = None                     # (Tạm tắt AI) Stores the latest camera frame for streaming
robot_state = "IDLE"                    # Current robot mode (IDLE, FOLLOWING, MANUAL)
manual_command = "STOP"                 # Current manual joystick command
lock = threading.Lock()                 # Thread lock to protect shared variables

# Serial Communication Setup (Adjust port name as needed for your OS)
# SERIAL_PORT = '' # MAC '/dev/ttyACM0' # PI '/dev/ttyUSB0'
# (Hãy chắc chắn bạn đã đổi cổng này cho Pi 5)
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) 
    print(f"Serial Port {SERIAL_PORT} opened successfully.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None # Set to None if serial failed

# (Tạm tắt AI - Gesture)
# gesture_timer = 0
# GESTURE_HOLD_TIME = 3                   # seconds
# current_gesture = None
# last_gesture_for_timer = None

# Hardware state variables
light_state = False                     # Tracks if the light relay is ON or OFF
# LIGHT_PIN = 17

# --- 5. Robot Hardware Functions (Serial Implementation) ---

def execute_robot_move(command):
    """ Executes the actual motor commands by sending PWM values via Serial. """
    global ser
    
    # Define speed (PWM 0-255)
    SPEED = 120 
    
    # Map high-level command to (Left_PWM, Right_PWM)
    # (Logic này giữ nguyên)
    cmd_map = {
        "FORWARD": (SPEED, SPEED),
        "LEFT": (-SPEED, SPEED),      # Pivot Left
        "RIGHT": (SPEED, -SPEED),     # Pivot Right
        "BACKWARD": (-SPEED, -SPEED), 
        "FORWARD_LEFT": (int(SPEED*0.5), SPEED),  # Curved turn left
        "FORWARD_RIGHT": (SPEED, int(SPEED*0.5)), # Curved turn right
        "BACKWARD_LEFT": (-SPEED, -int(SPEED*0.5)),
        "BACKWARD_RIGHT": (-int(SPEED*0.5), -SPEED),
        "STOP": (0, 0)
    }
    
    left_pwm, right_pwm = cmd_map.get(command, (0, 0))

    # Construct Serial Command: MOVE:left_speed:right_speed\n
    serial_command = f"MOVE:{left_pwm}:{right_pwm}\n" 
    
    if ser:
        try:
            ser.write(serial_command.encode())
            # This is the line sent to ESP32's Serial Monitor
            print(f"SERIAL SENT: {serial_command.strip()} -> L_PWM:{left_pwm} R_PWM:{right_pwm}") 
        except Exception as e:
            print(f"Serial write error: {e}")
    else:
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

# (Tạm tắt AI)
# def count_fingers(hand_landmarks):
#     """ Counts the number of extended fingers from MediaPipe landmarks. (Logic unchanged)"""
#     finger_count = 0
#     tip_ids = [mp_hands.HandLandmark.THUMB_TIP, mp_hands.HandLandmark.INDEX_FINGER_TIP,
#                mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.RING_FINGER_TIP,
#                mp_hands.HandLandmark.PINKY_TIP]
#     pip_ids = [mp_hands.HandLandmark.THUMB_IP, mp_hands.HandLandmark.INDEX_FINGER_PIP,
#                mp_hands.HandLandmark.MIDDLE_FINGER_PIP, mp_hands.HandLandmark.RING_FINGER_PIP,
#                mp_hands.HandLandmark.PINKY_PIP]
#     
#     # Thumb (check X-axis)
#     if hand_landmarks.landmark[tip_ids[0]].x < hand_landmarks.landmark[pip_ids[0]].x:
#         finger_count += 1
#     # Other 4 fingers (check Y-axis)
#     for i in range(1, 5):
#         if hand_landmarks.landmark[tip_ids[i]].y < hand_landmarks.landmark[pip_ids[i]].y:
#             finger_count += 1
#     return finger_count

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
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"ESP32 RESPONSE: {line}") 
            time.sleep(0.01)
        except Exception as e:
            print(f"Error reading from serial: {e}")
            time.sleep(1)

def robot_logic_thread():
    """ 
    This function runs in a separate thread.
    (ĐÃ TẮT AI) - Chỉ còn xử lý trạng thái IDLE và MANUAL.
    """
    global robot_state, manual_command, light_state
    
    # (Tạm tắt AI - Camera)
    # cap = cv2.VideoCapture(0)
    # success, img = cap.read()
    # if not success:
    #     print("FATAL: Cannot read camera, check connection.")
    #     return
    # FRAME_HEIGHT, FRAME_WIDTH, _ = img.shape
    # ZONE_LEFT = FRAME_WIDTH * 0.25
    # ZONE_RIGHT = FRAME_WIDTH * 0.75
    # prev_frame_time = 0
    
    print("Robot logic thread started... (AI DISABLED)")
    
    while True:
        # (Tạm tắt AI - Đọc camera)
        # success, image = cap.read()
        # if not success:
        #     print("Camera read failed, skipping frame.")
        #     time.sleep(0.1)
        #     continue
        # image = cv2.flip(image, 1) 
        # image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 

        # Đọc biến an toàn
        with lock:
            current_state = robot_state
            current_manual_cmd = manual_command
            current_light = light_state # Đọc thêm trạng thái đèn

        # (Tạm tắt AI - Gesture)
        # hud_text = f"STATE: {current_state}"
        # hud_color = (0, 0, 255) 
        # current_gesture = None
        # if current_state != "MANUAL":
        #     hand_results = hands.process(image_rgb)
        #     ... (toàn bộ logic gesture) ...
        
        # --- State Machine Logic (Simplified) ---
        if current_state == "IDLE":
            execute_robot_move("STOP") # Send STOP command (PWM 0,0)

        elif current_state == "FOLLOWING":
            # (Tạm tắt AI) - Trạng thái này sẽ không được kích hoạt từ web
            execute_robot_move("STOP")
            pass

        elif current_state == "MANUAL":
            # Execute joystick command
            execute_robot_move(current_manual_cmd) 

        # --- HUD & Frame Update (Tạm tắt AI) ---
        # new_frame_time = time.time()
        # if (new_frame_time - prev_frame_time) > 0:
        #     fps = 1 / (new_frame_time - prev_frame_time)
        # else:
        #     fps = 0
        # prev_frame_time = new_frame_time
        # ... (toàn bộ logic cv2.putText) ...

        # Gửi thông tin tới web (Không có FPS)
        socketio.emit('robot_info', {'fps': 0, 'state': current_state, 'light': current_light})

        # (Tạm tắt AI - Gửi frame)
        # with lock:
        #     _, buffer = cv2.imencode('.jpg', image)
        #     global_frame = buffer.tobytes()

        # Thêm sleep để giảm tải CPU
        time.sleep(0.05) # ~20 lần/giây

# --- 7. Flask HTTP Routes ---
@app.route('/')
def index():
    """ Serves the main HTML page. """
    return render_template('index.html')

# Tạo 1 ảnh JPEG placeholder (1x1 pixel)
black_pixel_jpeg = (
    b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00\xff\xdb\x00C\x00\x03\x02\x02\x02\x02\x02\x03\x02\x02\x02\x03\x03\x03\x03\x04\x06\x04\x04\x04\x04\x04\x08\x06\x06\x05\x06\t\x08\n\n\t\x08\t\t\n\x0c\x0f\x0c\n\x0b\x0e\x0b\t\t\r\x11\r\x0e\x0f\x10\x10\x11\x10\n\x0c\x12\x13\x12\x10\x13\x0f\x10\x10\x10\xff\xdb\x00C\x01\x03\x03\x03\x04\x03\x04\x08\x04\x04\x08\x10\x0b\t\x0b\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xc4\x00\x1a\x00\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\xff\xc4\x00\x14\x10\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\xda\x00\x08\x01\x01\x00\x00\x00\x01\x04\xff\xd9'
)

@app.route('/video_feed')
def video_feed():
    """ Provides the JPEG video stream. (Tạm tắt AI - Trả về ảnh đen) """
    def gen_frames():
        while True:
            # Trả về 1 ảnh đen thay vì video thật
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + black_pixel_jpeg + b'\r\n')
            time.sleep(1) # Gửi 1 frame/giây
            
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- 8. Socket.IO Events ---
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
        # (Tạm tắt AI)
        # if command == 'TOGGLE_FOLLOW':
        #     if robot_state == "FOLLOWING": robot_state = "IDLE"
        #     else: robot_state = "FOLLOWING"
                
        if command == 'SET_MANUAL': robot_state = "MANUAL"
        elif command == 'SET_IDLE': robot_state = "IDLE"
            
        elif command.startswith('MANUAL_'):
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
    
    toggle_light_relay(current_light_state) 
    
    emit('robot_info', {'fps': 0, 'state': current_state, 'light': current_light_state}, broadcast=True)

# --- 9. Start Application ---
if __name__ == '__main__':
    print("Starting Robot Thread...")
    robot_thread = threading.Thread(target=robot_logic_thread, daemon=True)
    robot_thread.start()
    
    # (Bắt đầu Serial read thread nếu bạn có gửi data từ ESP32 về)
    if ser:
        serial_thread = threading.Thread(target=serial_read_thread, daemon=True)
        serial_thread.start()
    
    print("Starting Web Server at http://0.0.0.0:5001")
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)
