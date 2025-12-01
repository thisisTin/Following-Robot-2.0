#include <Arduino.h>
#include <NewPing.h>

// === CẤU HÌNH PHẦN CỨNG ===
#define LED_BUILTIN 2 

// Chân điều khiển động cơ (L298N)
#define ENA 14
#define IN1 27
#define IN2 26
#define ENB 15
#define IN3 33 
#define IN4 32 
#define relay 13

// Cảm biến siêu âm (HC-SR04)
#define TRIG_PIN_FRONT 22 
#define ECHO_PIN_FRONT 23 
#define TRIG_PIN_BACK 19 
#define ECHO_PIN_BACK 4  
#define MAX_DISTANCE 400 // Khoảng cách đo tối đa (cm)

// === CẤU HÌNH FAILSAFE ===
// YÊU CẦU: < 50cm là dừng ngay lập tức
const int STOP_DISTANCE_FRONT = 50; // cm
const int STOP_DISTANCE_BACK = 50;  // cm

// Khởi tạo cảm biến
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarBack(TRIG_PIN_BACK, ECHO_PIN_BACK, MAX_DISTANCE);

// Biến toàn cục
int leftSpeed = 0; 
int rightSpeed = 0;
bool obstacleDetected = false; 
bool obstacleFront = false; 
bool obstacleBack = false;  
unsigned long lastObstacleCheck = 0; 
const int obstacleCheckInterval = 50; // Kiểm tra nhanh hơn (50ms)
String inputString = "";
bool stringComplete = false;
int current_stop_distance = 50;
// --- CÁC HÀM ĐIỀU KHIỂN ---

void stopMotors() { 
    analogWrite(ENA, 0); 
    analogWrite(ENB, 0); 
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    leftSpeed = 0; 
    rightSpeed = 0; 
}

void lightTest() { 
    digitalWrite(relay, HIGH); delay(200); 
    digitalWrite(relay, LOW); delay(200); 
}

void moveMotors(int targetLeftSpeed, int targetRightSpeed) { 
    // Logic chặn chiều di chuyển nếu có vật cản
    if (obstacleFront && (targetLeftSpeed > 0 || targetRightSpeed > 0)) {
        // Nếu có vật cản trước mà cố đi tiến -> Dừng
        stopMotors();
        return;
    }
    if (obstacleBack && (targetLeftSpeed < 0 || targetRightSpeed < 0)) {
        // Nếu có vật cản sau mà cố đi lùi -> Dừng
        stopMotors();
        return;
    }

    leftSpeed = targetLeftSpeed; 
    rightSpeed = targetRightSpeed; 

    // Motor Trái
    if (targetLeftSpeed >= 0) { 
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
    } else { 
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
        targetLeftSpeed = -targetLeftSpeed; 
    } 

    // Motor Phải
    if (targetRightSpeed >= 0) { 
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); 
    } else { 
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
        targetRightSpeed = -targetRightSpeed; 
    } 

    // Giới hạn PWM
    targetLeftSpeed = constrain(targetLeftSpeed, 0, 255); 
    targetRightSpeed = constrain(targetRightSpeed, 0, 255); 
    analogWrite(ENA, targetLeftSpeed); 
    analogWrite(ENB, targetRightSpeed); 
}

// Gửi trạng thái lên Python: STATUS:L:R:Front:Back:Obstacle
void sendStatus() { 
    int distFront = sonarFront.ping_cm();
    if (distFront == 0) distFront = MAX_DISTANCE;
    
    int distBack = sonarBack.ping_cm(); 
    if (distBack == 0) distBack = MAX_DISTANCE;

    Serial.print("STATUS:"); 
    Serial.print(leftSpeed); Serial.print(":"); 
    Serial.print(rightSpeed); Serial.print(":");
    Serial.print(distFront); Serial.print(":");  
    Serial.print(distBack); Serial.print(":"); 
    Serial.println(obstacleDetected ? "YES" : "NO"); 
}

void checkObstacles() { 
    int dF = sonarFront.ping_cm(); 
    if (dF == 0) dF = MAX_DISTANCE; 
    
    int dB = sonarBack.ping_cm(); 
    if (dB == 0) dB = MAX_DISTANCE; 

    // SỬ DỤNG BIẾN ĐỘNG current_stop_distance
    obstacleFront = (dF > 0 && dF < current_stop_distance);
    obstacleBack = (dB > 0 && dB < current_stop_distance);
    obstacleDetected = obstacleFront || obstacleBack;

    if (obstacleFront && (leftSpeed > 0 || rightSpeed > 0)) {
        stopMotors();
    }
    if (obstacleBack && (leftSpeed < 0 || rightSpeed < 0)) {
        stopMotors();
    }
}

void processCommand(String command) { 
    command.trim(); 
    if (command.startsWith("MOVE:")) { 
        // Format: MOVE:LeftPWM:RightPWM
        int firstColon = command.indexOf(':'); 
        int secondColon = command.indexOf(':', firstColon + 1);         
        if (firstColon > 0 && secondColon > 0) { 
            int tLeft = command.substring(firstColon + 1, secondColon).toInt(); 
            int tRight = command.substring(secondColon + 1).toInt();             
            moveMotors(tLeft, tRight); 
            Serial.println("OK:MOVE"); 
        } 
    }
    else if (command.startsWith("SET_STOP_DIST:")) { // LỆNH MỚI
        int firstColon = command.indexOf(':');
        int dist = command.substring(firstColon + 1).toInt();
        if (dist >= 5 && dist <= MAX_DISTANCE) { // Giới hạn an toàn
            current_stop_distance = dist;
            Serial.print("OK:SET_STOP_DIST:");
            Serial.println(current_stop_distance);
        } else {
            Serial.println("ERROR:INVALID_STOP_DIST");
        }
    }  
    else if (command.startsWith("LIGHT:")) { 
        if (command.indexOf("ON") > 0) digitalWrite(relay, HIGH);
        else digitalWrite(relay, LOW);
        Serial.println("OK:LIGHT");
    } 
    else if (command == "STOP") { 
        stopMotors(); 
        Serial.println("OK:STOP"); 
    } 
    else if (command == "STATUS") { 
        sendStatus(); 
    } 
}

void setup() { 
    Serial.begin(9600); 
    inputString.reserve(50); 
    
    pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); 
    pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(relay, OUTPUT); pinMode(LED_BUILTIN, OUTPUT); 
    stopMotors(); 
    lightTest(); 
    Serial.println("ROBOT READY. FAILSAFE: 50cm");
    Serial.print("ROBOT READY. DYNAMIC FAILSAFE: ");
    Serial.print(current_stop_distance);
    Serial.println("cm");
} 

void loop() { 
    // Đọc Serial không chặn (Non-blocking)
    while (Serial.available()) { 
        char inChar = (char)Serial.read();             
        if (inChar == '\n' || inChar == '\r') { 
            if (inputString.length() > 0) stringComplete = true;
        } else { 
            inputString += inChar; 
        }     
    }

    if (stringComplete) { 
        digitalWrite(LED_BUILTIN, HIGH);
        processCommand(inputString); 
        inputString = ""; 
        stringComplete = false; 
        digitalWrite(LED_BUILTIN, LOW);
    }
    
    // Kiểm tra vật cản định kỳ
    if (millis() - lastObstacleCheck >= obstacleCheckInterval) { 
        checkObstacles(); 
        lastObstacleCheck = millis(); 
    } 
}