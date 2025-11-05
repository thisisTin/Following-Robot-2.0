#include <Arduino.h>
#include <NewPing.h> 

// Define PINS ESP32 with L298N 
#define ENA 14  // Left Motor PWM
#define IN1 27  // Left Motor Direction 1
#define IN2 26  // Left Motor Direction 2
#define ENB 15  // Right Motor PWM
#define IN3 33  // Right Motor Direction 1
#define IN4 32  // Right Motor Direction 2

// Light Switch (DEMO mode) 
#define relay 13 // Relay/Light Control Pin

// Define PINS ESP32 with HC-SR04 
#define TRIG_PIN_FRONT 22 //TRIG_PIN
#define ECHO_PIN_FRONT 23 //ECHO_PIN
#define MAX_DISTANCE 500 // MAXIMUM DISTANCE (cm) 
// Create HC-SR04 SonarFront
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);

// Define Variables
int leftSpeed = 0; 
int rightSpeed = 0; 
bool obstacleDetected = false; 
unsigned long lastObstacleCheck = 0; 
const int obstacleCheckInterval = 100; // (ms)
String inputString = ""; // Serial String buffer
bool stringComplete = false; // Received Flag

// Side Functions
// 1. Stop All Motor (Set all motor signals/PWM to low)
void stopMotors() { 
    analogWrite(ENA, 0); 
    analogWrite(ENB, 0); 
    leftSpeed = 0; 
    rightSpeed = 0; 
}

// 2. Light Blinking To Test The LED
void lightTest(){
    digitalWrite(relay, HIGH);
    delay(1500);
    digitalWrite(relay,LOW);
    delay(1500);
}

// 3. Progress Bar
void fake_progress_bar() {
    Serial.print("Loading [");
    for (int i = 0; i < 20; i++) {
        Serial.print("#");
        delay(150);      
    }
    Serial.println("] 100%"); 
}

// 4. Robot Motion
void moveMotors(int targetLeftSpeed, int targetRightSpeed) { 
    // Store actual PWM values for status updates
    leftSpeed = targetLeftSpeed;
    rightSpeed = targetRightSpeed;
    
    // Left Motor Direction Logic
    if (targetLeftSpeed >= 0) { // Forward
        digitalWrite(IN1, HIGH); 
        digitalWrite(IN2, LOW); 
    }
    else { // Backward
        digitalWrite(IN1, LOW); 
        digitalWrite(IN2, HIGH); 
        targetLeftSpeed = -targetLeftSpeed; // Use positive value for PWM
    } 
    
    // Right Motor Direction Logic
    if (targetRightSpeed >= 0) { // Forward
        digitalWrite(IN3, HIGH); 
        digitalWrite(IN4, LOW); 
    } 
    else { // Backward
        digitalWrite(IN3, LOW); 
        digitalWrite(IN4, HIGH); 
        targetRightSpeed = -targetRightSpeed; // Use positive value for PWM
    } 

    // 5. Speed Constrain from 0-255 
    targetLeftSpeed = constrain(targetLeftSpeed, 0, 255); 
    targetRightSpeed = constrain(targetRightSpeed, 0, 255); 
    
    // Set actual PWM to motors
    analogWrite(ENA, targetLeftSpeed); 
    analogWrite(ENB, targetRightSpeed);
} 

// 6. Send Status Function
void sendStatus() { 
    int distFront = sonarFront.ping_cm(); 
    // Set maximun distance if can not track
    if (distFront == 0) distFront = MAX_DISTANCE;     
    Serial.print("STATUS:"); 
    Serial.print(leftSpeed); // Actual Left PWM
    Serial.print(":"); 
    Serial.print(rightSpeed); // Actual Right PWM
    Serial.print(":"); 
    Serial.print(distFront); 
    Serial.print(":");  
    Serial.println(obstacleDetected ? "YES" : "NO"); 
}

// 7. Control Unit - Main Command Processor
void processCommand(String command) { 
    command.trim(); // Clear [Space]
    
    if (command == "CHING") { // Check connection 
        Serial.println("CHON_DING_DONG"); 
    } 
    // MOVE: MOVE:left_speed:right_speed (e.g., MOVE:120:120)
    else if (command.startsWith("MOVE:")) { 
        int firstColon = command.indexOf(':'); 
        int secondColon = command.indexOf(':', firstColon + 1); 
        if (firstColon > 0 && secondColon > 0) { 
            String leftSpeedStr = command.substring(firstColon + 1, secondColon); 
            String rightSpeedStr = command.substring(secondColon + 1); 
            int targetLeftSpeed = leftSpeedStr.toInt(); 
            int targetRightSpeed = rightSpeedStr.toInt(); 
            moveMotors(targetLeftSpeed, targetRightSpeed); 
            
            // LOG PWM for debugging (Your Request)
            Serial.print("ROBOT_PWM:"); 
            Serial.print(targetLeftSpeed);
            Serial.print(":");
            Serial.println(targetRightSpeed);
            Serial.println("OK:MOVE"); 
        } 
    } 
    // LIGHT: LIGHT:ON or LIGHT:OFF
    else if (command.startsWith("LIGHT:")) { 
        String state = command.substring(6); 
        state.trim(); 
        if (state == "ON") { 
            digitalWrite(relay, HIGH); 
            Serial.println("OK:LIGHT_ON"); 
        } 
        else if (state == "OFF") { 
            digitalWrite(relay, LOW); 
            Serial.println("OK:LIGHT_OFF"); 
        } 
    } 
    // Stop All Motors 
    else if (command == "STOP") { 
        stopMotors(); 
        Serial.println("OK:STOP"); 
    } 
    // Get distance data
    else if (command == "GET_DIST") { 
        int distanceFront = sonarFront.ping_cm();  
        if (distanceFront == 0) distanceFront = MAX_DISTANCE; 
        Serial.print("DIST:");
        Serial.print(distanceFront); 
        Serial.println(":"); 
    } 
    // Serial received command named "Status" 
    else if (command == "STATUS") { 
        sendStatus(); 
    } 
    else { 
        Serial.print("ERROR:UNKNOWN_COMMAND:"); 
        Serial.println(command); 
    } 
} 

// Check the available Obstacles (infront)
void checkObstacles() { 
    // check HC-SR04 sensor distance
    int distanceFront = sonarFront.ping_cm(); 
    // Obstacle Detect When Moving (distance < 30cm)
    if ((leftSpeed != 0 || rightSpeed != 0) && (distanceFront > 0 && distanceFront < 30)){
        obstacleDetected = true; 
        Serial.println("OBSTACLE_DETECTED"); 
    } 
    else { 
        obstacleDetected = false; 
    } 
}

void setup() { 
    // Serial begin: Baud rate= 115200 (baud)
    Serial.begin(115200); 
    inputString.reserve(50); 
    // Setup pinMODE
    pinMode(ENA, OUTPUT); 
    pinMode(IN1, OUTPUT); 
    pinMode(IN2, OUTPUT); 
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);   
    pinMode(IN4, OUTPUT);
    pinMode(relay, OUTPUT); // Setup Relay pin
    
    stopMotors(); // Stop Motor Function
    Serial.println("ESP32 Motor Stopped");
    delay(200);
    lightTest(); // Light Test Function
    Serial.println("Light Test Completed");
    fake_progress_bar();
    Serial.println("Robot is Up and Prime!!!");
} 

void loop() { 
    // Process serial command if complete
    if (stringComplete) { 
        processCommand(inputString); 
        inputString = ""; 
        stringComplete = false; 
    } 
    // Serial receiving process...
    while (Serial.available()) { 
        char inChar = (char)Serial.read(); 
        if (inChar == '\n') { // Newline character marks end of command
            stringComplete = true; 
        } 
        else { 
            inputString += inChar; 
        }     
    }

    // Obstacle Checking
    if (millis() - lastObstacleCheck >= obstacleCheckInterval) { 
        checkObstacles(); 
        lastObstacleCheck = millis(); 
    } 

    // If obstacle existed, immediately stop all motors
    if (obstacleDetected) { 
        stopMotors(); 
    }
}