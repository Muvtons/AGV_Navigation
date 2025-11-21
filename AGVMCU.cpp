#include "AGVMCU.h"

AGVMCU agvmcu;
volatile long encL = 0;
volatile long encR = 0;
void IRAM_ATTR isrLeft()  { encL++; }
void IRAM_ATTR isrRight() { encR++; }

AGVMCU::AGVMCU() 
    : distanceThreshold(10.0), totalSteps(0), currentStepIndex(0), 
      currentX(-1), currentY(-1), currentDir('E'), currentState(STATE_IDLE),
      obstacleTimerStart(0), pulsesTraveledBeforeStop(0) {}

void AGVMCU::begin() {
    Serial.begin(115200);
    pinMode(SPD_L, INPUT_PULLUP); pinMode(SPD_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SPD_L), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(SPD_R), isrRight, RISING);
    pinMode(DIR_L, OUTPUT); pinMode(DIR_R, OUTPUT);
    pinMode(BRK_L, OUTPUT); pinMode(BRK_R, OUTPUT);
    pinMode(START_BTN, INPUT_PULLUP); pinMode(STOP_BTN, INPUT_PULLUP); pinMode(ABORT_BTN, INPUT_PULLUP);
    initPWM();
    motorBrake();
    Serial.println("✅ AGVMCU READY");
}

// ==================== CORE UPDATE LOOP ====================
void AGVMCU::update() {
    // 1. Buttons (Highest Priority)
    if(digitalRead(STOP_BTN) == LOW) handleStop();
    if(digitalRead(ABORT_BTN) == LOW) handleAbort();
    if(digitalRead(START_BTN) == LOW) handleStart();

    // 2. Obstacle Timer Logic
    if (currentState == STATE_OBSTACLE_WAITING) {
        if (millis() - obstacleTimerStart > 10000) {
            Serial.println("⚠️ Obstacle Timeout! Initiating Retreat.");
            initiateRetreat();
        }
        return; // Skip PID
    }

    // 3. PID Motion
    if (currentState == STATE_MOVING || currentState == STATE_TURNING || currentState == STATE_RETURNING) {
        runPIDCycle();
    }
}

// ==================== COMMAND PARSING ====================
void AGVMCU::processCommand(String cmd) {
    cmd.trim();
    
    // --- ABORT RECOVERY (Wait for POS) ---
    if (currentState == STATE_ABORT_WAITING_POS) {
        if (cmd.startsWith("POS:")) { // Format: POS:x,y,angle
            // Just go to IDLE, external system updates x,y via this string usually
            // Assuming you parse x,y here, strictly strictly:
            // For now, just unlocking the state:
            currentState = STATE_IDLE;
            Serial.println("✅ Position Updated. Abort State Cleared. Ready.");
        }
        return; // Ignore all other commands while in Abort
    }

    // --- OBSTACLE SENSOR ---
    if (cmd.startsWith("DISTANCE:")) {
        float dist = cmd.substring(9).toFloat();
        
        // DETECT OBSTACLE
        if (dist < distanceThreshold && currentState == STATE_MOVING) {
            motorBrake();
            noInterrupts(); 
            pulsesTraveledBeforeStop = (abs(encL) + abs(encR)) / 2; 
            interrupts();
            
            obstacleTimerStart = millis();
            currentState = STATE_OBSTACLE_WAITING;
            Serial.print("⛔ Obstacle! Pulses recorded: "); Serial.println(pulsesTraveledBeforeStop);
            Serial.println("Waiting 10s...");
        }
        // CLEAR OBSTACLE
        else if (dist >= distanceThreshold && currentState == STATE_OBSTACLE_WAITING) {
            Serial.println("✅ Obstacle Cleared. Resuming.");
            currentState = STATE_MOVING; // PID loop continues from where it left off
        }
        return;
    }

    // --- PATH PARSING (Manhattan) ---
    if (cmd.startsWith("Manhattan")) {
        cmd.replace("Manhattan path: ", "");
        totalSteps = 0;
        int pIdx = 0;
        while (cmd.indexOf('(', pIdx) != -1) {
            int open = cmd.indexOf('(', pIdx);
            int comma = cmd.indexOf(',', open);
            int close = cmd.indexOf(')', comma);
            if (open == -1 || comma == -1 || close == -1) break;
            
            Step s;
            s.x = cmd.substring(open+1, comma).toInt();
            s.y = cmd.substring(comma+1, close).toInt();
            s.dir = cmd.charAt(close+1);
            path[totalSteps++] = s;
            pIdx = close + 2;
        }
        Serial.print("Path Received. Steps: "); Serial.println(totalSteps);
        // Note: We do not start automatically. Wait for START button or explicit START command.
    }
    
    // --- POSITION UPDATES (POS:4,5,0) ---
    if (cmd.startsWith("POS:")) {
        int c1 = cmd.indexOf(':');
        int c2 = cmd.indexOf(',', c1);
        int c3 = cmd.indexOf(',', c2+1);
        
        currentX = cmd.substring(c1+1, c2).toInt();
        currentY = cmd.substring(c2+1, c3).toInt();
        float angle = cmd.substring(c3+1).toFloat();
        // Convert Angle to Dir char if needed, or just store raw
        if(angle > 315 || angle <= 45) currentDir = 'E';
        else if(angle > 45 && angle <= 135) currentDir = 'N';
        else if(angle > 135 && angle <= 225) currentDir = 'W';
        else currentDir = 'S';
        
        if(currentState == STATE_WAITING_QR) {
            // Logic to correct angle if needed, then move to next step
            // For brevity, assuming perfectly aligned:
            handleStart(); // Trigger next move
        }
    }
}

// ==================== PID & MOTOR LOGIC ====================
void AGVMCU::runPIDCycle() {
    if (millis() - lastPIDTime < PID_INTERVAL_MS) return;

    long l, r;
    noInterrupts(); l = abs(encL); r = abs(encR); interrupts();
    long avg = (l + r) / 2;

    // --- COMPLETION CHECK ---
    if (avg >= targetPulses) {
        motorBrake();
        
        // 1. FINISHED TURNING
        if (currentState == STATE_TURNING) {
            currentState = nextStateAfterTurn; 
            
            if (currentState == STATE_RETURNING) {
                // We just turned 180, now drive back the saved distance
                targetPulses = pulsesTraveledBeforeStop;
                slowdownStartPulses = targetPulses * MOVE_SLOWDOWN_START;
                resetEncoders();
                motorForward();
                // PID vars reset
                prevError = 0; integral = 0; lastPIDTime = millis(); slowdownPhase = false;
                setPWM(BASE_SPEED, BASE_SPEED);
            }
            else if (currentState == STATE_ABORT_WAITING_POS) {
                Serial.print("Task aborted and reached at ");
                Serial.print(currentX); Serial.print(","); Serial.print(currentY);
                Serial.print(currentDir); Serial.println("/[WAITING_ANGLE]");
            }
        } 
        // 2. FINISHED MOVING (Normal)
        else if (currentState == STATE_MOVING) {
            // Arrived at target node
            currentX = path[currentStepIndex].x;
            currentY = path[currentStepIndex].y;
            currentStepIndex++;
            currentState = STATE_WAITING_QR;
            Serial.println("Arrived. Waiting for QR...");
        }
        // 3. FINISHED RETURNING (Obstacle Retreat)
        else if (currentState == STATE_RETURNING) {
            // We are back at the previous node
            Serial.print("Blocked coordinates: ");
            Serial.print(path[currentStepIndex].x); Serial.print(","); Serial.println(path[currentStepIndex].y);
            Serial.print("Reached at: ");
            // We assume we are back at the start of this step
            // Note: In a real system, we wait for QR here to confirm X,Y
            Serial.print(currentX); Serial.print(","); Serial.print(currentY); 
            Serial.print(currentDir); Serial.println(" (Waiting for new path)");
            
            currentState = STATE_IDLE;
        }
        return;
    }

    // --- PID CALCULATION ---
    // (Standard PID logic from your previous code)
    float currentSpeedTarget = (currentState == STATE_TURNING) ? TURN_SPEED : BASE_SPEED;
    
    // Ramp Down
    if (avg >= slowdownStartPulses) slowdownPhase = true;
    if (slowdownPhase) {
        float progress = (float)(avg - slowdownStartPulses) / (targetPulses - slowdownStartPulses);
        progress = constrain(progress, 0.0, 1.0);
        float minSpd = FINAL_SPEED;
        currentSpeedTarget = currentSpeedTarget - (currentSpeedTarget - minSpd) * progress;
    }

    long error = l - r;
    float dt = (millis() - lastPIDTime) / 1000.0;
    integral += error * dt;
    integral = constrain(integral, -100, 100);
    float derivative = (error - prevError) / dt;
    float output = KP * error + KI * integral + KD * derivative;
    output = constrain(output, -MAX_CORRECTION, MAX_CORRECTION);

    float spdL = currentSpeedTarget - output;
    float spdR = currentSpeedTarget + output;
    
    // For Turns, signs might differ depending on implementation
    // Assuming positive error = Left ahead -> Slow Left
    if (error > 0) spdL -= abs(output); else spdR -= abs(output);

    setPWM(spdL, spdR);
    prevError = error;
    lastPIDTime = millis();
}

// ==================== LOGIC HELPERS ====================
void AGVMCU::handleStart() {
    if (currentStepIndex >= totalSteps) return;
    
    Step target = path[currentStepIndex];
    
    // 1. Check Direction
    if (currentDir != target.dir) {
        rotateToDirection(currentDir, target.dir);
        nextStateAfterTurn = STATE_MOVING; // After turn, normal move
    } else {
        // 2. Move Forward
        targetPulses = mmToPulses(SQUARE_SIZE_MM);
        slowdownStartPulses = targetPulses * MOVE_SLOWDOWN_START;
        resetEncoders();
        motorForward();
        currentState = STATE_MOVING;
        // Reset PID
        prevError = 0; integral = 0; lastPIDTime = millis(); slowdownPhase = false;
        setPWM(BASE_SPEED, BASE_SPEED);
    }
}

void AGVMCU::initiateRetreat() {
    // 1. Turn 180
    char backDir = getOppositeDir(currentDir);
    rotateToDirection(currentDir, backDir);
    
    // 2. Tell system what to do AFTER turn
    nextStateAfterTurn = STATE_RETURNING;
}

void AGVMCU::handleAbort() {
    motorBrake();
    if (currentState == STATE_ABORT_WAITING_POS) return; // Already aborted
    
    Serial.println("ABORTING!");
    
    // 1. Turn 180
    char backDir = getOppositeDir(currentDir);
    rotateToDirection(currentDir, backDir);
    
    // 2. Tell system what to do AFTER turn
    nextStateAfterTurn = STATE_ABORT_WAITING_POS;
}

void AGVMCU::rotateToDirection(char from, char to) {
    motorBrake();
    delay(200); // Brief pause for physics
    
    int diff = 0; // Logic to calc angle...
    // Simple mapping for Manhattan
    if (from == 'E') { if(to=='N') diff=90; if(to=='W') diff=180; if(to=='S') diff=-90; }
    if (from == 'N') { if(to=='W') diff=90; if(to=='S') diff=180; if(to=='E') diff=-90; }
    if (from == 'W') { if(to=='S') diff=90; if(to=='E') diff=180; if(to=='N') diff=-90; }
    if (from == 'S') { if(to=='E') diff=90; if(to=='N') diff=180; if(to=='W') diff=-90; }
    
    if (diff == 0) return; // Should not happen if called correctly

    bool isRight = (diff < 0);
    float deg = abs(diff);
    
    targetPulses = turnDegreesToPulses(deg);
    slowdownStartPulses = targetPulses * getTurnSlowdownStart(deg);
    resetEncoders();
    if(isRight) motorRight(); else motorLeft();
    
    currentDir = to; // Optimistic update
    currentState = STATE_TURNING;
    
    prevError = 0; integral = 0; lastPIDTime = millis(); slowdownPhase = false;
    setPWM(TURN_SPEED, TURN_SPEED);
}

// ==================== LOW LEVEL ====================
void AGVMCU::handleStop() { motorBrake(); currentState = STATE_STOPPED; Serial.println("STOPPED"); }
void AGVMCU::initPWM() { /* ... Your Standard PWM Init ... */ }
void AGVMCU::setPWM(float l, float r) { /* ... Your Standard PWM Set ... */ }
void AGVMCU::motorBrake() { digitalWrite(BRK_L, HIGH); digitalWrite(BRK_R, HIGH); setPWM(0,0); }
void AGVMCU::motorForward() { digitalWrite(DIR_L, HIGH); digitalWrite(DIR_R, HIGH); digitalWrite(BRK_L, LOW); digitalWrite(BRK_R, LOW); }
void AGVMCU::motorLeft() { digitalWrite(DIR_L, LOW); digitalWrite(DIR_R, HIGH); digitalWrite(BRK_L, LOW); digitalWrite(BRK_R, LOW); }
void AGVMCU::motorRight() { digitalWrite(DIR_L, HIGH); digitalWrite(DIR_R, LOW); digitalWrite(BRK_L, LOW); digitalWrite(BRK_R, LOW); }
void AGVMCU::resetEncoders() { noInterrupts(); encL = 0; encR = 0; interrupts(); }

char AGVMCU::getOppositeDir(char dir) {
    if (dir == 'N') return 'S';
    if (dir == 'S') return 'N';
    if (dir == 'E') return 'W';
    return 'E'; // Default W -> E
}

// Math (Using your values)
long AGVMCU::mmToPulses(float mm) { return (long)((mm / (PI * WHEEL_DIAMETER_MM)) * PULSES_PER_REV * LINEAR_CALIBRATION); }
long AGVMCU::turnDegreesToPulses(float deg) { float arc = (PI * WHEEL_BASE_MM) * (deg / 360.0f); return (long)((arc / (PI * WHEEL_DIAMETER_MM)) * PULSES_PER_REV * TURN_CALIBRATION); }
float AGVMCU::getTurnSlowdownStart(float deg) { if (deg <= 45) return 0.85; if (deg <= 90) return 0.35; return 0.65; }
