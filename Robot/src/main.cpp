#include <Arduino.h>
// #include <NewPing.h> // (Tạm tắt cảm biến)

// Define PINS ESP32 with L298N 
#define ENA 14  // Left Motor PWM
#define IN1 27  // Left Motor Direction 1
#define IN2 26  // Left Motor Direction 2
#define ENB 15  // Right Motor PWM
#define IN3 33  // Right Motor Direction 1
#define IN4 32  // Right Motor Direction 2

// Light Switch (DEMO mode) 
#define relay 13 // Relay/Light Control Pin

// (Tạm tắt cảm biến)
// #define TRIG_PIN_FRONT 22 //TRIG_PIN
// #define ECHO_PIN_FRONT 23 //ECHO_PIN
// #define MAX_DISTANCE 500 // MAXIMUM DISTANCE (cm) 
// NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);

// Define Variables
int leftSpeed = 0; 
int rightSpeed = 0; 
// bool obstacleDetected = false; // (Tạm tắt cảm biến)
unsigned long lastObstacleCheck = 0; 
// const int obstacleCheckInterval = 100; // (ms)
String inputString = ""; // Serial String buffer
bool stringComplete = false; // Received Flag

// Stop All Motor
void stopMotors() { 
    analogWrite(ENA, 0); 
    analogWrite(ENB, 0); 
    leftSpeed = 0; 
    rightSpeed = 0; 
}

// (Các hàm lightTest, fake_progress_bar giữ nguyên...)
void lightTest(){
    digitalWrite(relay, HIGH); delay(500);
    digitalWrite(relay,LOW); delay(500);
}
void fake_progress_bar() {
    Serial.print("Loading [");
    for (int i = 0; i < 20; i++) { Serial.print("#"); delay(50); }
    Serial.println("] 100%"); 
}


// Robot Motion
void moveMotors(int targetLeftSpeed, int targetRightSpeed) { 
    leftSpeed = targetLeftSpeed;
    rightSpeed = targetRightSpeed;
    
    // (Logic motor giữ nguyên...)
    if (targetLeftSpeed >= 0) { 
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
    } else { 
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
        targetLeftSpeed = -targetLeftSpeed; 
    } 
    if (targetRightSpeed >= 0) { 
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
    } else { 
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); 
        targetRightSpeed = -targetRightSpeed; 
    } 
    targetLeftSpeed = constrain(targetLeftSpeed, 0, 255); 
    targetRightSpeed = constrain(targetRightSpeed, 0, 255); 
    analogWrite(ENA, targetLeftSpeed); 
    analogWrite(ENB, targetRightSpeed);
} 

// (Hàm sendStatus giữ nguyên, nhưng bỏ phần cảm biến)
void sendStatus() { 
    // int distFront = sonarFront.ping_cm(); // (Tạm tắt cảm biến)
    // if (distFront == 0) distFront = MAX_DISTANCE;     
    Serial.print("STATUS:"); 
    Serial.print(leftSpeed); Serial.print(":"); 
    Serial.print(rightSpeed); Serial.print(":"); 
    Serial.print(0); // Gửi 0 cho khoảng cách
    Serial.print(":");  
    Serial.println("NO"); // Luôn báo không có vật cản
}

// Control Unit - Main Command Processor
void processCommand(String command) { 
    command.trim(); 
    
    if (command == "CHING") { 
        Serial.println("CHON_DING_DONG"); 
    } 
    else if (command.startsWith("MOVE:")) { 
        int firstColon = command.indexOf(':'); 
        int secondColon = command.indexOf(':', firstColon + 1); 
        if (firstColon > 0 && secondColon > 0) { 
            String leftSpeedStr = command.substring(firstColon + 1, secondColon); 
            String rightSpeedStr = command.substring(secondColon + 1); 
            int targetLeftSpeed = leftSpeedStr.toInt(); 
            int targetRightSpeed = rightSpeedStr.toInt(); 
            
            // (Đã bỏ check vật cản)
            moveMotors(targetLeftSpeed, targetRightSpeed); 
            
            Serial.print("ROBOT_PWM:"); 
            Serial.print(targetLeftSpeed); // In ra tốc độ YÊU CẦU
            Serial.print(":");
            Serial.println(targetRightSpeed);
            Serial.println("OK:MOVE"); 
        } 
    } 
    else if (command.startsWith("LIGHT:")) { 
        String state = command.substring(6); 
        state.trim(); 
        if (state == "ON") { 
            digitalWrite(relay, HIGH); Serial.println("OK:LIGHT_ON"); 
        } else if (state == "OFF") { 
            digitalWrite(relay, LOW); Serial.println("OK:LIGHT_OFF"); 
        } 
    } 
    // (Các lệnh STOP, GET_DIST, STATUS, ERROR giữ nguyên...)
    else if (command == "STOP") { 
        stopMotors(); 
        Serial.println("OK:STOP"); 
    } 
    else if (command == "GET_DIST") { 
        Serial.print("DIST:0:\n"); // (Tạm tắt cảm biến)
    } 
    else if (command == "STATUS") { 
        sendStatus(); 
    } 
    else { 
        Serial.print("ERROR:UNKNOWN_COMMAND:"); 
        Serial.println(command); 
    } 
} 

// (Tạm tắt cảm biến)
// void checkObstacles() { 
//     int distanceFront = sonarFront.ping_cm(); 
//     bool isMovingForward = (leftSpeed > 0 || rightSpeed > 0);
//     if (isMovingForward && (distanceFront > 0 && distanceFront < 30)){
//         if (!obstacleDetected) { 
//              Serial.println("OBSTACLE_DETECTED"); 
//         }
//         obstacleDetected = true; 
//     } 
//     else { 
//         obstacleDetected = false; 
//     } 
// }

void setup() { 
    Serial.begin(115200); 
    inputString.reserve(50); 
    pinMode(ENA, OUTPUT); 
    pinMode(IN1, OUTPUT); 
    pinMode(IN2, OUTPUT); 
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);   
    pinMode(IN4, OUTPUT);
    pinMode(relay, OUTPUT); 
    
    stopMotors(); 
    Serial.println("ESP32 Motor Stopped");
    delay(200);
    lightTest(); 
    Serial.println("Light Test Completed");
    fake_progress_bar();
    Serial.println("Robot is Up and Prime!!! (SENSOR DISABLED)"); // Thông báo đã tắt cảm biến
} 

void loop() { 
    if (stringComplete) { 
        processCommand(inputString); 
        inputString = ""; 
        stringComplete = false; 
    } 
    while (Serial.available()) { 
        char inChar = (char)Serial.read(); 
        if (inChar == '\n') { 
            stringComplete = true; 
        } 
        else { 
            inputString += inChar; 
        }     
    }

    // (Tạm tắt cảm biến)
    // if (millis() - lastObstacleCheck >= obstacleCheckInterval) { 
    //     checkObstacles(); 
    //     lastObstacleCheck = millis(); 
    // } 
    // (Tạm tắt logic an toàn)
    // if (obstacleDetected) { 
    //     if(leftSpeed > 0 || rightSpeed > 0) { 
    //         stopMotors(); 
    //         Serial.println("AUTO_STOP: Obstacle!");
    //     }
    // }
}