#include "AGVMCU.h"

// Global instance
AGVMCU agvmcu;

AGVMCU::AGVMCU() 
    : distanceThreshold(10.0), distanceBelowThreshold(false),
      totalSteps(0), currentStepIndex(0), currentX(-1), currentY(-1), 
      currentDir('E'), currentState(STATE_IDLE),
      lastAbortPress(0), lastStartPress(0), lastStopPress(0),
      distanceBelowThresholdStart(0) {
    xMutex = xSemaphoreCreateMutex();
    xQueue = xQueueCreate(20, sizeof(Message));
}

void AGVMCU::begin(long baudRate) {
    Serial.begin(baudRate);
    
    // Initialize motor control pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Initialize button pins with internal pull-up resistors
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(ABORT_BUTTON_PIN, INPUT_PULLUP);

    // Initialize all pins to safe state
    stopMotors();
    
    Serial.println("AGV System Ready - ESP32-S3 Dual Core");
    
    // Create tasks for both cores
    xTaskCreatePinnedToCore(core0Task, "Core0", 10000, this, 2, NULL, 0);
    xTaskCreatePinnedToCore(core1Task, "Core1", 10000, this, 1, NULL, 1);
}

// Core 0 - High Priority Input Handling
void AGVMCU::core0Task(void* parameter) {
    AGVMCU* agv = (AGVMCU*)parameter;
    for(;;) {
        agv->handleCore0();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void AGVMCU::handleCore0() {
    handleSerialInput();
    handleButtons();
}

// Core 1 - Main Logic Processing
void AGVMCU::core1Task(void* parameter) {
    AGVMCU* agv = (AGVMCU*)parameter;
    for(;;) {
        agv->handleCore1();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void AGVMCU::handleCore1() {
    Message msg;
    
    // Process all messages in queue
    while(xQueueReceive(xQueue, &msg, 0) == pdTRUE) {
        processMessage(msg);
    }
    
    // Handle state-specific continuous processing
    if(xSemaphoreTake(xMutex, portMAX_DELAY)) {
        switch(currentState) {
            case STATE_MOVING:
                // Movement handled in navigateToNextStep
                break;
            case STATE_OBSTACLE_RECOVERY:
                // In obstacle recovery - waiting for QR
                break;
            default:
                break;
        }
        xSemaphoreGive(xMutex);
    }
}

void AGVMCU::processMessage(Message msg) {
    if(xSemaphoreTake(xMutex, portMAX_DELAY)) {
        switch(msg.type) {
            case MSG_PATH:
                parsePath(msg.data);
                if(totalSteps > 0) {
                    currentStepIndex = 0;
                    if(currentX == -1 || currentY == -1) {
                        currentState = STATE_WAITING_FOR_QR;
                        Serial.println("Waiting for initial QR...");
                    } else {
                        currentState = STATE_MOVING;
                        navigateToNextStep();
                    }
                }
                break;
                
            case MSG_QR: {
                String coordData = msg.data;
                int firstComma = coordData.indexOf(',');
                int secondComma = coordData.indexOf(',', firstComma + 1);
                
                if(secondComma != -1) {
                    int qrX = coordData.substring(0, firstComma).toInt();
                    int qrY = coordData.substring(firstComma + 1, secondComma).toInt();
                    float qrAngle = coordData.substring(secondComma + 1).toFloat();

                    currentX = qrX;
                    currentY = qrY;
                    correctDirectionUsingQRAngle(qrAngle);

                    switch(currentState) {
                        case STATE_WAITING_FOR_QR:
                            currentState = STATE_MOVING;
                            navigateToNextStep();
                            break;
                            
                        case STATE_WAITING_QR_CONFIRMATION: {
                            Step targetStep = path[currentStepIndex];
                            if(isAtPosition(targetStep.x, targetStep.y)) {
                                currentStepIndex++;
                                publishCurrentPosition();
                                
                                if(currentStepIndex >= totalSteps) {
                                    Serial.println("DESTINATION REACHED");
                                    currentState = STATE_GOAL_REACHED;
                                } else {
                                    currentState = STATE_MOVING;
                                    navigateToNextStep();
                                }
                            } else {
                                Serial.println("NAV_FAILED: Wrong QR position");
                                rotateAngle(180);
                                currentState = STATE_STOPPED;
                            }
                            break;
                        }
                        
                        case STATE_OBSTACLE_RECOVERY:
                            Serial.print("currentpos after blocked : ");
                            Serial.print(currentX); Serial.print(","); Serial.println(currentY);
                            Serial.print("blocked qr : ");
                            Serial.print(currentX); Serial.print(","); Serial.println(currentY);
                            currentState = STATE_IDLE;
                            totalSteps = 0;
                            break;
                            
                        case STATE_ABORTED:
                            Serial.print("currentpos after abort : ");
                            Serial.print(currentX); Serial.print(","); Serial.println(currentY);
                            currentState = STATE_IDLE;
                            totalSteps = 0;
                            break;
                    }
                }
                break;
            }
            
            case MSG_DISTANCE: {
                float currentDistance = msg.data.toFloat();
                if(currentDistance < distanceThreshold) {
                    if(!distanceBelowThreshold) {
                        distanceBelowThreshold = true;
                        distanceBelowThresholdStart = millis();
                        
                        if(currentState == STATE_MOVING) {
                            stopMotors();
                            handleObstacle();
                        }
                    }
                } else {
                    distanceBelowThreshold = false;
                }
                break;
            }
            
            case MSG_COMMAND:
                if(msg.data == "STOP") handleStop();
                else if(msg.data == "START") handleStart();
                else if(msg.data == "ABORT") handleAbort();
                break;
                
            case MSG_BUTTON:
                if(msg.data == "STOP") handleStop();
                else if(msg.data == "START") handleStart();
                else if(msg.data == "ABORT") handleAbort();
                break;
        }
        xSemaphoreGive(xMutex);
    }
}

void AGVMCU::handleSerialInput() {
    if(Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        Message msg;
        
        if(input == "ABORT" || input == "STOP" || input == "START") {
            msg.type = MSG_COMMAND;
            msg.data = input;
            xQueueSend(xQueue, &msg, portMAX_DELAY);
        }
        else if(input.startsWith("DISTANCE:")) {
            msg.type = MSG_DISTANCE;
            msg.data = input.substring(9);
            xQueueSend(xQueue, &msg, portMAX_DELAY);
        }
        else if(input.startsWith("QR:")) {
            msg.type = MSG_QR;
            msg.data = input.substring(3);
            xQueueSend(xQueue, &msg, portMAX_DELAY);
        }
        else if(input.startsWith("(")) {
            msg.type = MSG_PATH;
            msg.data = input;
            xQueueSend(xQueue, &msg, portMAX_DELAY);
        }
    }
}

void AGVMCU::handleButtons() {
    unsigned long now = millis();
    Message msg;

    if(digitalRead(STOP_BUTTON_PIN) == LOW && (now - lastStopPress > 500)) {
        lastStopPress = now;
        msg.type = MSG_BUTTON;
        msg.data = "STOP";
        xQueueSend(xQueue, &msg, portMAX_DELAY);
    }

    if(digitalRead(START_BUTTON_PIN) == LOW && (now - lastStartPress > 500)) {
        lastStartPress = now;
        msg.type = MSG_BUTTON;
        msg.data = "START";
        xQueueSend(xQueue, &msg, portMAX_DELAY);
    }

    if(digitalRead(ABORT_BUTTON_PIN) == LOW && (now - lastAbortPress > 500)) {
        lastAbortPress = now;
        msg.type = MSG_BUTTON;
        msg.data = "ABORT";
        xQueueSend(xQueue, &msg, portMAX_DELAY);
    }
}

// Motor Control Functions
void AGVMCU::moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}

void AGVMCU::moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}

void AGVMCU::stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void AGVMCU::rotateAngle(float degrees) {
    if(degrees > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);

    unsigned long rotateTime = (unsigned long)(abs(degrees) * 8.5);
    delay(rotateTime);
    stopMotors();
    delay(500);
}

void AGVMCU::rotateToDirection(char from, char to) {
    int angleDiff = 0;
    if((from == 'E' && to == 'N') || (from == 'N' && to == 'W') || 
       (from == 'W' && to == 'S') || (from == 'S' && to == 'E')) {
        angleDiff = 90;
    } else if((from == 'E' && to == 'S') || (from == 'S' && to == 'W') || 
              (from == 'W' && to == 'N') || (from == 'N' && to == 'E')) {
        angleDiff = -90;
    } else if(from != to) {
        angleDiff = 180;
    }
    
    if(angleDiff != 0) {
        rotateAngle(angleDiff);
        currentDir = to;
    }
}

void AGVMCU::correctDirectionUsingQRAngle(float qrAngle) {
    float targetAngle = 0.0;
    switch(currentDir) {
        case 'E': targetAngle = 0.0; break;
        case 'N': targetAngle = 90.0; break;
        case 'W': targetAngle = 180.0; break;
        case 'S': targetAngle = -90.0; break;
    }
    
    float angleError = targetAngle - qrAngle;
    while(angleError > 180) angleError -= 360;
    while(angleError < -180) angleError += 360;
    
    if(abs(angleError) > 5.0) {
        rotateAngle(angleError);
    }
}

void AGVMCU::navigateToNextStep() {
    if(currentStepIndex >= totalSteps) {
        Serial.println("DESTINATION REACHED");
        currentState = STATE_GOAL_REACHED;
        return;
    }

    Step targetStep = path[currentStepIndex];

    if(currentDir != targetStep.dir) {
        rotateToDirection(currentDir, targetStep.dir);
    }

    currentState = STATE_MOVING;
    
    // Move one cell (2 seconds)
    moveForward();
    delay(2000);
    stopMotors();
    
    currentState = STATE_WAITING_QR_CONFIRMATION;
}

bool AGVMCU::isAtPosition(int x, int y) {
    return (currentX == x && currentY == y);
}

void AGVMCU::parsePath(String raw) {
    totalSteps = 0;
    raw.trim();
    int i = 0;
    while(i < raw.length() && totalSteps < 50) {
        if(raw[i] == '(') {
            int commaIndex = raw.indexOf(',', i);
            int endIndex = raw.indexOf(')', commaIndex);
            if(commaIndex == -1 || endIndex == -1) break;
            int x = raw.substring(i + 1, commaIndex).toInt();
            int y = raw.substring(commaIndex + 1, endIndex).toInt();
            char dir = 'E';
            if(endIndex + 1 < raw.length()) {
                dir = raw[endIndex + 1];
            }
            if(dir == 'N' || dir == 'S' || dir == 'E' || dir == 'W') {
                path[totalSteps++] = {x, y, dir};
            }
            i = endIndex + 2;
        } else {
            i++;
        }
    }
}

void AGVMCU::publishCurrentPosition() {
    Serial.print("CURRENT_POS:");
    Serial.print(currentX); Serial.print(","); Serial.print(currentY);
    Serial.print(","); Serial.println(currentDir);
}

void AGVMCU::publishRerouteCommand(int src_x, int src_y, int dst_x, int dst_y) {
    Serial.print("REROUTE:SRC:(");
    Serial.print(src_x); Serial.print(","); Serial.print(src_y);
    Serial.print("):DST:(");
    Serial.print(dst_x); Serial.print(","); Serial.print(dst_y);
    Serial.println("):ONCE");
}

void AGVMCU::handleAbort() {
    stopMotors();
    currentState = STATE_ABORTED;
    
    // Turnaround and move forward
    rotateAngle(180);
    moveForward();
    delay(2000);
    stopMotors();
    
    Serial.println("ABORT: Waiting for QR scan...");
}

void AGVMCU::handleStop() {
    stopMotors();
    currentState = STATE_STOPPED;
}

void AGVMCU::handleStart() {
    if(currentState == STATE_STOPPED && totalSteps > 0) {
        currentState = STATE_MOVING;
        navigateToNextStep();
    }
}

void AGVMCU::handleObstacle() {
    stopMotors();
    currentState = STATE_OBSTACLE_RECOVERY;
    
    // Turnaround and move forward
    rotateAngle(180);
    moveForward();
    delay(2000);
    stopMotors();
    
    Serial.println("OBSTACLE: Waiting for QR scan...");
}
