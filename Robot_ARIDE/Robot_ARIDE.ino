#include <Serial.h>
#include <NewPing.h> 

// Define PINS ESP32 with L298N 
#define ENA 13
#define IN1 27
#define IN2 26
#define ENB 15
#define IN3 16 
#define IN4 17

//Light Switch (DEMO mode) 
#define relay 34 

// Define PINS ESP32 with HC-SR04 (maximum distance=4500)
#define TRIG_PIN_FRONT 22 //TRIG_PIN
#define ECHO_PIN_FRONT 23 //ECHO_PIN
#define MAX_DISTANCE 500 // MAXIMUM DISTANCE (cm) 
// Create HC-SR04 SonarFront
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);

//{ Not Ready yet
// NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE); 
// NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE); 
//}
// Define Variables
int leftSpeed = 0; 
int rightSpeed = 0; 
bool obstacleDetected = false; 
unsigned long lastObstacleCheck = 0; 
const int obstacleCheckInterval = 100; // (ms)
String inputString = ""; // Serial String
bool stringComplete = false; // Received Flag

// Side Functions
// 1. Stop All Motor (đưa tín hiệu tất cả các chân(ENA; IN) về mức thấp)
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
void moveMotors(int leftSpeed, int rightSpeed) { 
    // Left Motor
    if (leftSpeed >= 0) { 
        digitalWrite(IN1, HIGH); 
        digitalWrite(IN2, LOW); 
    }
    else { 
        digitalWrite(IN1, LOW); 
        digitalWrite(IN2, HIGH); 
        leftSpeed = -leftSpeed; 
    } 
// Right Motor
    if (rightSpeed >= 0) { 
        digitalWrite(IN3, HIGH); 
        digitalWrite(IN4, LOW); 
    } 
    else { 
        digitalWrite(IN3, LOW); 
        digitalWrite(IN4, HIGH); 
        rightSpeed = -rightSpeed; 
    } 

    // 5. Speed Constrain from 0-255 
leftSpeed = constrain(leftSpeed, 0, 255); 
rightSpeed = constrain(rightSpeed, 0, 255); 
analogWrite(ENA, leftSpeed); 
analogWrite(ENB, rightSpeed);
} 

// 6. Send Status Function
void sendStatus() { 
int distFront = sonarFront.ping_cm(); 
// Set maximun distance if can not track
    if (distFront == 0) distFront = MAX_DISTANCE;     
    Serial.print("STATUS:"); 
    Serial.print(leftSpeed); 
    Serial.print(":"); 
    Serial.print(rightSpeed); 
    Serial.print(":"); 
    Serial.print(distFront); 
    Serial.print(":");  
    Serial.println(obstacleDetected ? "YES" : ""); 
}

// 7. Control Unit
void processCommand(String command) { 
command.trim(); //Clear [Space]
// check connection 
    if (command == "CHING") { 
        Serial.println("CHON_DING_DONG"); 
    } 
// MOVE: MOVE:left_speed:right_speed 
    else if (command.startsWith("MOVE:")) { 
    int firstColon = command.indexOf(':'); 
    int secondColon = command.indexOf(':', firstColon + 1); 
        if (firstColon > 0 && secondColon > 0) { 
        String leftSpeedStr = command.substring(firstColon + 1, secondColon); 
        String rightSpeedStr = command.substring(secondColon + 1); 
        leftSpeed = leftSpeedStr.toInt(); 
        rightSpeed = rightSpeedStr.toInt(); 
        moveMotors(leftSpeed, rightSpeed); 
        Serial.println("OK:MOVE"); 
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
// IF the distance equal to 0, set to max_distance 
    if (distanceFront == 0) distanceFront = MAX_DISTANCE; 
        Serial.print("DIST:");
        Serial.print(distanceFront); 
        Serial.print(":"); 
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
// Obstacle Detect When Moving
    if ((leftSpeed != 0 || rightSpeed != 0) && (distanceFront > 0 && distanceFront < 30)){
        obstacleDetected = true; 
        Serial.println("OBSTACLE"); 
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
// Stop Motor
stopMotors(); //Stop Motor Function
Serial.println("ESP32 Motor Stopped");
delay(200);
lightTest(); //Light Test Function
Serial.println("Light Test Completed");
fake_progress_bar();
Serial.println("Robot is Up and Prime!!!");
} 

void loop() { 

    if (stringComplete) { 
        processCommand(inputString); 
        inputString = ""; 
        stringComplete = false; 
    } 
//Serial receiving process...
    while (Serial.available()) { 
        char inChar = (char)Serial.read(); 
        if (inChar == '\n') { 
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

// if the obstacle existed, stop all the motors
    if (obstacleDetected) { 
        stopMotors(); 
    }

} 
