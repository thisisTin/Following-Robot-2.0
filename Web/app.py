# --- 1. Imports ---
import cv2
from ultralytics import YOLO
import mediapipe as mp
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial
# (Uncomment this when on Raspberry Pi)
# import RPi.GPIO as GPIO 

# --- 2. AI Model Initialization ---
print("Loading AI Models...")
# Initialize YOLO model for object tracking
model = YOLO('yolov8n.pt') 
# Initialize MediaPipe Hands for gesture recognition
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.6, min_tracking_confidence=0.5, max_num_hands=1)
mp_drawing = mp.solutions.drawing_utils
print("Models loaded successfully.")

# --- 3. Web Server Initialization ---
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_very_secret_key'
socketio = SocketIO(app, async_mode='threading')

# --- 4. Global Variables & Serial Initialization ---
global_frame = None                     # Stores the latest camera frame for streaming
robot_state = "IDLE"                    # Current robot mode (IDLE, FOLLOWING, MANUAL)
manual_command = "STOP"                 # Current manual joystick command
lock = threading.Lock()                 # Thread lock to protect shared variables

# Serial Communication Setup (Adjust port name as needed for your OS)
SERIAL_PORT = 'COM6' # Example: Use '/dev/ttyACM0' on Linux/Pi or 'COM3' on Windows
BAUD_RATE = 115200
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) 
    print(f"Serial Port {SERIAL_PORT} opened successfully.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None # Set to None if serial failed

# Gesture control variables
gesture_timer = 0
GESTURE_HOLD_TIME = 3                   # seconds
current_gesture = None
last_gesture_for_timer = None

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
    # Note: Directions are relative to the robot.
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

def count_fingers(hand_landmarks):
    """ Counts the number of extended fingers from MediaPipe landmarks. (Logic unchanged)"""
    finger_count = 0
    tip_ids = [mp_hands.HandLandmark.THUMB_TIP, mp_hands.HandLandmark.INDEX_FINGER_TIP,
               mp_hands.HandLandmark.MIDDLE_FINGER_TIP, mp_hands.HandLandmark.RING_FINGER_TIP,
               mp_hands.HandLandmark.PINKY_TIP]
    pip_ids = [mp_hands.HandLandmark.THUMB_IP, mp_hands.HandLandmark.INDEX_FINGER_PIP,
               mp_hands.HandLandmark.MIDDLE_FINGER_PIP, mp_hands.HandLandmark.RING_FINGER_PIP,
               mp_hands.HandLandmark.PINKY_PIP]
    
    # Thumb (check X-axis)
    if hand_landmarks.landmark[tip_ids[0]].x < hand_landmarks.landmark[pip_ids[0]].x:
        finger_count += 1
    # Other 4 fingers (check Y-axis)
    for i in range(1, 5):
        if hand_landmarks.landmark[tip_ids[i]].y < hand_landmarks.landmark[pip_ids[i]].y:
            finger_count += 1
    return finger_count

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
                    
                    # Bạn có thể thêm logic ở đây để cập nhật trạng thái web 
                    # dựa trên phản hồi của ESP32 (Ví dụ: trạng thái cảm biến vật cản)
                    
            time.sleep(0.01) # Small delay to prevent high CPU usage
        except Exception as e:
            print(f"Error reading from serial: {e}")
            time.sleep(1)

def robot_logic_thread():
    """ This function runs in a separate thread, handling all AI, Camera, and Robot control logic. """
    global global_frame, robot_state, manual_command, gesture_timer, current_gesture, last_gesture_for_timer, light_state
    # Initialize Camera
    cap = cv2.VideoCapture(0)
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
    
    while True:
        success, image = cap.read()
        if not success:
            print("Camera read failed, skipping frame.")
            time.sleep(0.1)
            continue

        image = cv2.flip(image, 1) # Flip camera horizontally
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # Convert for MediaPipe
        
        # Read shared variables safely
        with lock:
            current_state = robot_state
            current_manual_cmd = manual_command

        hud_text = f"STATE: {current_state}"
        hud_color = (0, 0, 255) # Default: Red (STOP/IDLE)
        current_gesture = None
        
        # --- Gesture Recognition (Timer Logic) ---
        if current_state != "MANUAL":
            # (MediaPipe processing and state change logic remains the same)
            # ... (Existing gesture logic here)
            hand_results = hands.process(image_rgb)
            if hand_results.multi_hand_landmarks:
                for hand_landmarks in hand_results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    fingers_up = count_fingers(hand_landmarks)
                    
                    if fingers_up >= 4: current_gesture = "OPEN_HAND"
                    elif fingers_up <= 1: current_gesture = "CLOSED_HAND"

            # State change timer
            if current_gesture is None:
                gesture_timer = 0
                last_gesture_for_timer = None
            else:
                if last_gesture_for_timer != current_gesture:
                    gesture_timer = time.time() # Start timer
                    last_gesture_for_timer = current_gesture
                else:
                    elapsed = time.time() - gesture_timer
                    if elapsed > GESTURE_HOLD_TIME:
                        # Hold time complete, change state
                        with lock:
                            if current_gesture == "OPEN_HAND" and current_state == "IDLE":
                                print("GESTURE: Activating FOLLOW mode")
                                robot_state = "FOLLOWING"
                            elif current_gesture == "CLOSED_HAND" and current_state == "FOLLOWING":
                                print("GESTURE: Deactivating FOLLOW mode to IDLE")
                                robot_state = "IDLE"
                        gesture_timer = 0 # Reset timer
        
        # --- State Machine Logic ---
        if current_state == "IDLE":
            hud_color = (0, 255, 255) # Yellow
            execute_robot_move("STOP") # Send STOP command (PWM 0,0)

        elif current_state == "FOLLOWING":
            hud_color = (0, 255, 0) # Green
            # Run YOLO model
            results = model.track(image, persist=True, classes=[0], verbose=False, imgsz=320, conf=0.5)
            found_person = False
            if results[0].boxes:
                for box in results[0].boxes:
                    found_person = True
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    centerX = (x1 + x2) / 2
                    
                    # AI movement command
                    if centerX < ZONE_LEFT: execute_robot_move("LEFT")
                    elif centerX > ZONE_RIGHT: execute_robot_move("RIGHT")
                    else: execute_robot_move("FORWARD") # Maintain distance
                    break
            if not found_person:
                execute_robot_move("STOP")
            
            # Draw tracking zones
            cv2.line(image, (int(ZONE_LEFT), 0), (int(ZONE_LEFT), FRAME_HEIGHT), (255, 0, 0), 2)
            cv2.line(image, (int(ZONE_RIGHT), 0), (int(ZONE_RIGHT), FRAME_HEIGHT), (255, 0, 0), 2)

        elif current_state == "MANUAL":
            hud_color = (255, 0, 0) # Blue
            # Execute joystick command (sends corresponding PWM via Serial)
            execute_robot_move(current_manual_cmd) 

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
        
        if gesture_timer > 0:
            elapsed_time = time.time() - gesture_timer
            remaining_time = max(0, GESTURE_HOLD_TIME - elapsed_time)
            cv2.putText(image, f"Gesture: {remaining_time:.1f}s", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        # Emit info to all connected web clients
        with lock:
            current_light_state = light_state 
        socketio.emit('robot_info', {'fps': int(fps), 'state': current_state, 'light': current_light_state})

        # Encode frame to JPEG and store in global variable for streaming
        with lock:
            _, buffer = cv2.imencode('.jpg', image)
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
    
    print("Starting Web Server at http://0.0.0.0:5001")
    # Using port 5001 to avoid macOS AirPlay conflict
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)