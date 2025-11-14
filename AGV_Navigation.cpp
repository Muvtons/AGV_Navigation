#include "AGV_Navigation.h"

AGV_Navigation::AGV_Navigation(int ena, int in1, int in2, int enb, int in3, int in4,
                               int startBtn, int stopBtn, int abortBtn, int defaultBtn) {
    // Store pin numbers
    _ena = ena; _in1 = in1; _in2 = in2; _enb = enb; _in3 = in3; _in4 = in4;
    _startBtn = startBtn; _stopBtn = stopBtn; _abortBtn = abortBtn; _defaultBtn = defaultBtn;
}

void AGV_Navigation::begin() {
    Serial.begin(115200);
    
    // Motor pins
    pinMode(_ena, OUTPUT);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_enb, OUTPUT);
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);
    
    // Button pins
    pinMode(_startBtn, INPUT_PULLUP);
    pinMode(_stopBtn, INPUT_PULLUP);
    pinMode(_abortBtn, INPUT_PULLUP);
    pinMode(_defaultBtn, INPUT_PULLUP);
    
    stopMotors();
    Serial.println("AGV Navigation Library Ready");
}

void AGV_Navigation::run() {
    handleButtons();
    
    // Handle abort/distance timeout recovery movement
    if ((abortActive || distanceTimeoutActive) && waitingForQRAfterAbort) {
        moveForward();
        delay(100);
        return;
    }
    
    if (awaitingPositionAfterNewPath) return;
    if (movementCompleted) return;
    if (!isStarted || isStopped || goalReached || currentStep >= totalSteps || currentX == -1) return;
    
    Step target = steps[currentStep];
    
    if (currentX == target.x && currentY == target.y) {
        if (currentDir != target.dir) {
            stopMotors();
            rotateToDirection(currentDir, target.dir);
            currentDir = target.dir;
        }
        
        if (currentStep == totalSteps - 1) {
            Serial.print("✅ Goal reached: ");
            Serial.print(currentX); Serial.print(","); Serial.println(currentY);
            goalReached = true;
            stopMotors();
            
            currentX = -1; currentY = -1;
            totalSteps = 0; currentStep = 0;
            goalReached = false;
            isStarted = false;
            return;
        }
        
        Step nextTarget = steps[currentStep + 1];
        Serial.print("Step "); Serial.print(currentStep);
        Serial.print(": Moving from ("); Serial.print(currentX);
        Serial.print(","); Serial.print(currentY);
        Serial.print(") toward ("); Serial.print(nextTarget.x);
        Serial.print(","); Serial.print(nextTarget.y);
        Serial.print(") facing "); Serial.println(target.dir);
        
        moveOneCell();
        movementCompleted = true;
    }
}

void AGV_Navigation::processCommand(String command) {
    command.trim();
    
    if (command == "ABORT") {
        handleAbort();
        return;
    }
    
    if (command.startsWith("DISTANCE:")) {
        String distanceStr = command.substring(9);
        float currentDistance = distanceStr.toFloat();
        checkDistanceCondition(currentDistance);
        return;
    }
    
    if (command.startsWith("(")) {
        parsePath(command);
        if (totalSteps > 0) {
            goalReached = false;
            isStopped = false;
            currentStep = 0;
            movementCompleted = false;
            
            if (currentX == -1 || currentY == -1) {
                awaitingPositionAfterNewPath = true;
                isStarted = false;
                Serial.print("✅ New path loaded. Steps: ");
                Serial.println(totalSteps);
                Serial.println("⏳ Awaiting initial QR position...");
            } else {
                awaitingPositionAfterNewPath = false;
                isStarted = true;
                Serial.print("✅ New path loaded and position known (");
                Serial.print(currentX); Serial.print(","); Serial.print(currentY);
                Serial.println("). Starting immediately.");
            }
        }
        return;
    }
    
    if (command.startsWith("QR:")) {
        String coordData = command.substring(3);
        int firstComma = coordData.indexOf(',');
        int secondComma = coordData.indexOf(',', firstComma + 1);
        
        if (secondComma != -1) {
            int qrX = coordData.substring(0, firstComma).toInt();
            int qrY = coordData.substring(firstComma + 1, secondComma).toInt();
            float qrAngle = coordData.substring(secondComma + 1).toFloat();
            
            currentX = qrX;
            currentY = qrY;
            
            Serial.print("📍 QR Position: "); 
            Serial.print(currentX); Serial.print(","); Serial.print(currentY);
            Serial.print(" Angle: "); Serial.println(qrAngle);
            
            correctDirectionUsingQRAngle(qrAngle);
            
            // Handle QR during recovery
            if (abortActive && waitingForQRAfterAbort) {
                Serial.println("✅ QR detected after abort recovery!");
                stopMotors();
                abortActive = false;
                waitingForQRAfterAbort = false;
                return;
            }
            
            if (distanceTimeoutActive && waitingForQRAfterAbort) {
                Serial.println("✅ QR detected after distance timeout recovery!");
                stopMotors();
                distanceTimeoutActive = false;
                waitingForQRAfterAbort = false;
                return;
            }
            
            // Normal QR handling
            if (awaitingPositionAfterNewPath && totalSteps > 0) {
                awaitingPositionAfterNewPath = false;
                isStarted = true;
                Serial.println("✅ Initial QR received. Starting navigation.");
            }
            
            if (movementCompleted && currentStep < totalSteps - 1) {
                Step nextTarget = steps[currentStep + 1];
                if (currentX == nextTarget.x && currentY == nextTarget.y) {
                    Serial.println("✅ QR confirmed arrival at next step.");
                    currentStep++;
                    movementCompleted = false;
                    publishCurrentPosition();
                }
            }
        }
        return;
    }
    
    if (command.length() > 0) {
        Serial.print("❓ Unknown command: ");
        Serial.println(command);
    }
}

String AGV_Navigation::getStatus() {
    String status = "Position: ";
    status += String(currentX) + "," + String(currentY);
    status += " Dir: " + String(currentDir);
    status += " Step: " + String(currentStep) + "/" + String(totalSteps);
    status += " Started: " + String(isStarted);
    status += " WaitingQR: " + String(awaitingPositionAfterNewPath);
    return status;
}

bool AGV_Navigation::isNavigating() { return isStarted && !isStopped; }
bool AGV_Navigation::isWaitingForQR() { return awaitingPositionAfterNewPath; }

// ========== PRIVATE METHODS (Copy all your existing functions here) ==========

void AGV_Navigation::handleButtons() {
    static unsigned long lastAbortPress = 0;
    unsigned long now = millis();
    
    if (digitalRead(_stopBtn) == LOW && !isStopped) {
        isStopped = true;
        stopMotors();
        Serial.println("⏹️ STOP pressed.");
        delay(300);
    }
    
    if (digitalRead(_startBtn) == LOW && !isStarted && !awaitingPositionAfterNewPath) {
        isStarted = true;
        isStopped = false;
        Serial.println("▶️ START pressed.");
        delay(300);
    }
    
    if (digitalRead(_abortBtn) == LOW) {
        if (now - lastAbortPress > 500) {
            lastAbortPress = now;
            handleAbort();
        }
    }
    
    if (digitalRead(_defaultBtn) == LOW) {
        Serial.println("🟩 DEFAULT button pressed.");
        delay(300);
        String defaultPath = "(1,1)E(2,1)E(2,2)N(3,2)E";
        parsePath(defaultPath);
        
        if (totalSteps > 0) {
            awaitingPositionAfterNewPath = (currentX == -1);
            isStarted = !awaitingPositionAfterNewPath;
            currentStep = 0;
            movementCompleted = false;
            goalReached = false;
            Serial.println("✅ Default path loaded.");
        }
    }
}

void AGV_Navigation::triggerRecoveryProcedure(const char* source) {
    Serial.print("🛑 Recovery triggered by: ");
    Serial.println(source);
    stopMotors();
    Serial.println("⛔ Motors stopped.");
    Serial.println("🔄 Turning around 180°...");
    rotateAngle(180);
    Serial.println("✅ Turnaround complete.");
    Serial.println("🚶 Moving forward until next QR is detected...");
    waitingForQRAfterAbort = true;
    
    isStarted = false;
    isStopped = true;
    awaitingPositionAfterNewPath = false;
    movementCompleted = false;
    distanceBelowThreshold = false;
    goalReached = false;
    totalSteps = 0;
    currentStep = 0;
}

// ... COPY ALL YOUR EXISTING FUNCTIONS EXACTLY AS THEY ARE ...
// handleAbort(), handleDistanceTimeout(), moveOneCell(), rotateAngle(), 
// correctDirectionUsingQRAngle(), checkDistanceCondition(), parsePath(),
// moveForward(), stopMotors(), rotateToDirection(), publishCurrentPosition()
// ... (Copy the complete implementations from your original code)

// Add the missing variable for distance checking
bool distanceBelowThreshold = false;
unsigned long distanceBelowThresholdStart = 0;
const float DISTANCE_THRESHOLD = 0.1;
const unsigned long DISTANCE_TIMEOUT_MS = 10000;

void AGV_Navigation::handleDistanceTimeout() {
    Serial.println("🛑 DISTANCE TIMEOUT - Triggering recovery procedure!");
    distanceTimeoutActive = true;
    triggerRecoveryProcedure("Distance Timeout");
    Serial.print("DISTANCE_TIMEOUT_POS:");
    Serial.print(currentX); Serial.print(","); Serial.println(currentY);
}

void AGV_Navigation::checkDistanceCondition(float currentDistance) {
    unsigned long currentTime = millis();
    if (currentDistance < DISTANCE_THRESHOLD) {
        if (!distanceBelowThreshold) {
            distanceBelowThreshold = true;
            distanceBelowThresholdStart = currentTime;
            Serial.print("📏 Distance below threshold: ");
            Serial.print(currentDistance); Serial.println("m - Starting timer...");
        } else if (currentTime - distanceBelowThresholdStart >= DISTANCE_TIMEOUT_MS) {
            handleDistanceTimeout();
        }
    } else if (distanceBelowThreshold) {
        distanceBelowThreshold = false;
        Serial.print("✅ Distance back to normal: ");
        Serial.print(currentDistance); Serial.println("m");
    }
}

// ... Continue copying all other functions exactly as in your original code ...
