#include <Arduino.h>

// === SỬA LỖI LED_BUILTIN ===
#define LED_BUILTIN 2 

// (Tất cả các định nghĩa chân, hàm moveMotors, stopMotors... giữ nguyên)
#define ENA 14
#define IN1 27
#define IN2 26
#define ENB 15
#define IN3 33
#define IN4 32
#define relay 13

// (Các biến global)
int leftSpeed = 0; 
int rightSpeed = 0; 
String inputString = "";
bool stringComplete = false;

// *** CÁC HÀM CŨ GIỮ NGUYÊN NHƯ FILE NO_SENSOR ***
void stopMotors() { analogWrite(ENA, 0); analogWrite(ENB, 0); leftSpeed = 0; rightSpeed = 0; }
void lightTest(){ digitalWrite(relay, HIGH); delay(500); digitalWrite(relay,LOW); delay(500); }
void fake_progress_bar() { Serial.print("Loading ["); for (int i = 0; i < 20; i++) { Serial.print("#"); delay(50); } Serial.println("] 100%"); }
void moveMotors(int targetLeftSpeed, int targetRightSpeed) { leftSpeed = targetLeftSpeed; rightSpeed = targetRightSpeed; if (targetLeftSpeed >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); } else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); targetLeftSpeed = -targetLeftSpeed; } if (targetRightSpeed >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); } else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); targetRightSpeed = -targetRightSpeed; } targetLeftSpeed = constrain(targetLeftSpeed, 0, 255); targetRightSpeed = constrain(targetRightSpeed, 0, 255); analogWrite(ENA, targetLeftSpeed); analogWrite(ENB, targetRightSpeed); }
void sendStatus() { Serial.print("STATUS:"); Serial.print(leftSpeed); Serial.print(":"); Serial.print(rightSpeed); Serial.print(":"); Serial.print(0); Serial.print(":"); Serial.println("NO"); }
void processCommand(String command) { command.trim(); if (command == "CHING") { Serial.println("CHON_DING_DONG"); } else if (command.startsWith("MOVE:")) { int firstColon = command.indexOf(':'); int secondColon = command.indexOf(':', firstColon + 1); if (firstColon > 0 && secondColon > 0) { String leftSpeedStr = command.substring(firstColon + 1, secondColon); String rightSpeedStr = command.substring(secondColon + 1); int targetLeftSpeed = leftSpeedStr.toInt(); int targetRightSpeed = rightSpeedStr.toInt(); moveMotors(targetLeftSpeed, targetRightSpeed); Serial.print("ROBOT_PWM:"); Serial.print(targetLeftSpeed); Serial.print(":"); Serial.println(targetRightSpeed); Serial.println("OK:MOVE"); } } else if (command.startsWith("LIGHT:")) { String state = command.substring(6); state.trim(); if (state == "ON") { digitalWrite(relay, HIGH); Serial.println("OK:LIGHT_ON"); } else if (state == "OFF") { digitalWrite(relay, LOW); Serial.println("OK:LIGHT_OFF"); } } else if (command == "STOP") { stopMotors(); Serial.println("OK:STOP"); } else if (command == "GET_DIST") { Serial.print("DIST:0:\n"); } else if (command == "STATUS") { sendStatus(); } else { Serial.print("ERROR:UNKNOWN_COMMAND:"); Serial.println(command); } }


// --- 7. Hàm Setup (Giữ nguyên LED_BUILTIN) ---
void setup() { 
    Serial.begin(9600); 
    inputString.reserve(50); 
    pinMode(ENA, OUTPUT); 
    pinMode(IN1, OUTPUT); 
    pinMode(IN2, OUTPUT); 
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);   
    pinMode(IN4, OUTPUT);
    pinMode(relay, OUTPUT); 
    pinMode(LED_BUILTIN, OUTPUT); // Khai báo đèn LED xanh
    
    stopMotors(); 
    Serial.println("ESP32 Motor Stopped");
    delay(200);
    lightTest(); 
    Serial.println("Light Test Completed");
    fake_progress_bar();
    Serial.println("Robot is Up and Prime!!! (SENSOR DISABLED - FINAL FIX)");
} 

// --- 8. Vòng lặp (Sửa) ---
void loop() { 
    if (Serial.available() > 0) {
      digitalWrite(LED_BUILTIN, HIGH); 
      while (Serial.available()) { 
          char inChar = (char)Serial.read(); 
          
          // === SỬA LỖI TẠI ĐÂY ===
          // Chấp nhận \n (Unix) hoặc \r (một số serial app)
          if (inChar == '\n' || inChar == '\r') { 
          // ======================
              stringComplete = true; 
              // Đọc hết phần rác còn lại trong buffer nếu có (ví dụ \r\n)
              while(Serial.available()) { Serial.read(); } 
          } else { 
              inputString += inChar; 
          }     
      }
    } else {
      digitalWrite(LED_BUILTIN, LOW); 
    }

    if (stringComplete) { 
        // Bật LED sáng luôn khi xử lý lệnh
        digitalWrite(LED_BUILTIN, HIGH);
        
        processCommand(inputString); 
        inputString = ""; 
        stringComplete = false; 
    } 
}