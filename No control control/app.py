# --- 1. Imports ---
# (Tất cả import giữ nguyên)
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO, emit
import serial

# --- 2. AI Model Initialization ---
print("Loading AI Models... (SKIPPED)")
# (Giữ nguyên)

# --- 3. Web Server Initialization ---
app = Flask(__name__)
app.config['SECRET_KEY'] = 'your_very_secret_key'
socketio = SocketIO(app, async_mode='threading')

# --- 4. Global Variables & Serial Initialization ---
# (Tất cả giữ nguyên, đảm bảo SERIAL_PORT là đúng)
robot_state = "IDLE"
manual_command = "STOP"
lock = threading.Lock()
SERIAL_PORT = '/dev/ttyUSB0' # <-- Đảm bảo cổng này đúng!
BAUD_RATE = 115200
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) 
    print(f"Serial Port {SERIAL_PORT} opened successfully.")
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port {SERIAL_PORT}. {e}")
    ser = None
light_state = False

# --- 5. Robot Hardware Functions (ĐÃ CẬP NHẬT TỐC ĐỘ) ---

def execute_robot_move(command):
    """ Executes the actual motor commands by sending PWM values via Serial. """
    global ser
    
    # === THAY ĐỔI TỐC ĐỘ THEO YÊU CẦU CỦA BẠN ===
    SPEED = 190       # Tốc độ chạy (Trước là 120)
    TURN_SPEED = 220  # Tốc độ xoay (Trước là 120)
    # Tốc độ cua (Curve) sẽ dùng 1 nửa tốc độ chạy
    CURVE_SPEED_SLOW = int(SPEED * 0.5) # ~95
    CURVE_SPEED_FAST = SPEED # 190
    
    # Map high-level command to (Left_PWM, Right_PWM)
    cmd_map = {
        "FORWARD": (SPEED, SPEED),                   # (190, 190)
        "LEFT": (-TURN_SPEED, TURN_SPEED),           # (-220, 220) Xoay tại chỗ
        "RIGHT": (TURN_SPEED, -TURN_SPEED),          # (220, -220) Xoay tại chỗ
        "BACKWARD": (-SPEED, -SPEED),                # (-190, -190)
        
        "FORWARD_LEFT": (CURVE_SPEED_SLOW, CURVE_SPEED_FAST),  # (95, 190) Cua trái
        "FORWARD_RIGHT": (CURVE_SPEED_FAST, CURVE_SPEED_SLOW), # (190, 95) Cua phải
        
        "BACKWARD_LEFT": (-CURVE_SPEED_FAST, -CURVE_SPEED_SLOW), # (-190, -95)
        "BACKWARD_RIGHT": (-CURVE_SPEED_SLOW, -CURVE_SPEED_FAST),# (-95, -190)
        
        "STOP": (0, 0)
    }
    
    left_pwm, right_pwm = cmd_map.get(command, (0, 0))

    # Construct Serial Command: MOVE:left_speed:right_speed\n
    serial_command = f"MOVE:{left_pwm}:{right_pwm}\n" 
    
    if ser:
        try:
            ser.write(serial_command.encode())
            print(f"SERIAL SENT: {serial_command.strip()} -> L_PWM:{left_pwm} R_PWM:{right_pwm}") 
        except Exception as e:
            print(f"Serial write error: {e}")
    else:
        print(f"ROBOT SIMULATED: {command} -> L_PWM:{left_pwm} R_PWM:{right_pwm}")

# (Hàm toggle_light_relay giữ nguyên)
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

# --- 6. Main Robot Logic Thread ---
# (serial_read_thread và robot_logic_thread giữ nguyên y hệt)
def serial_read_thread():
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
    global robot_state, manual_command, light_state
    print("Robot logic thread started... (AI DISABLED)")
    while True:
        with lock:
            current_state = robot_state
            current_manual_cmd = manual_command
            current_light = light_state 
        
        if current_state == "IDLE":
            execute_robot_move("STOP")
        elif current_state == "FOLLOWING":
            execute_robot_move("STOP")
            pass
        elif current_state == "MANUAL":
            execute_robot_move(current_manual_cmd) 

        socketio.emit('robot_info', {'fps': 0, 'state': current_state, 'light': current_light})
        time.sleep(0.05) # ~20 lần/giây

# --- 7. Flask HTTP Routes ---
# (Tất cả route giữ nguyên y hệt)
@app.route('/')
def index():
    return render_template('index.html')

# === ĐÃ SỬA LỖI SYNTAX ERROR TẠI ĐÂY ===
# (Lỗi là \xx00, đã sửa thành \x00)
black_pixel_jpeg = (
    b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00\xff\xdb\x00C\x00\x03\x02\x02\x02\x02\x02\x03\x02\x02\x02\x03\x03\x03\x03\x04\x06\x04\x04\x04\x04\x04\x08\x06\x06\x05\x06\t\x08\n\n\t\x08\t\t\n\x0c\x0f\x0c\n\x0b\x0e\x0b\t\t\r\x11\r\x0e\x0f\x10\x10\x11\x10\n\x0c\x12\x13\x12\x10\x13\x0f\x10\x10\x10\xff\xdb\x00C\x01\x03\x03\x03\x04\x03\x04\x08\x04\x04\x08\x10\x0b\t\x0b\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\x10\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xc4\x00\x1a\x00\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x03\x04\x05\x06\xff\xc4\x00\x14\x10\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xff\xda\x00\x08\x01\x01\x00\x00\x00\x01\x04\xff\xd9'
)
@app.route('/video_feed')
def video_feed():
    def gen_frames():
        while True:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + black_pixel_jpeg + b'\r\n')
            time.sleep(1) 
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- 8. Socket.IO Events ---
# (Tất cả socket event giữ nguyên y hệt)
@socketio.on('connect')
def handle_connect():
    print('Client connected!')
    with lock:
        emit('robot_info', {'fps': 0, 'state': robot_state, 'light': light_state})

@socketio.on('robot_command')
def handle_robot_command(data):
    global robot_state, manual_command
    command = data.get('command')
    print(f"Web command received: {command}")
    with lock:
        if command == 'SET_MANUAL': robot_state = "MANUAL"
        elif command == 'SET_IDLE': robot_state = "IDLE"
        elif command.startswith('MANUAL_'):
            if robot_state == "MANUAL":
                manual_command = command.split('_')[1]
    with lock:
        emit('robot_info', {'fps': 0, 'state': robot_state, 'light': light_state}, broadcast=True)

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
    
    # === SỬA LỖI LOGIC NHỎ ===
    # (Dòng log bị lỗi "0.S.0.0", sửa thành "0.0.0.0")
    print("Starting Web Server at http://0.0.0.0:5001")
    socketio.run(app, host='0.0.0.0', port=5001, debug=False)