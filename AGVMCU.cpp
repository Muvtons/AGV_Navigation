#include "AGVMCU.h"
#include <string.h> // Required for strncmp, strchr, etc.

AGVMCU agvmcu;

// Global Interrupt Variables
volatile long encL = 0;
volatile long encR = 0;
void IRAM_ATTR isrLeft()  { encL++; }
void IRAM_ATTR isrRight() { encR++; }

AGVMCU::AGVMCU() 
    : distanceThreshold(10.0), totalSteps(0), currentStepIndex(0), 
      currentX(-1), currentY(-1), currentDir('E'), targetDirPending('E'),
      currentState(STATE_IDLE), obstacleTimerStart(0), 
      pulsesTraveledBeforeStop(0), lastButtonPressTime(0) {}

void AGVMCU::begin() {
    Serial.begin(115200);
    
    // 1. SAFETY FIRST: Lock motors immediately to prevent startup twitch
    pinMode(DIR_L, OUTPUT); digitalWrite(DIR_L, LOW);
    pinMode(DIR_R, OUTPUT); digitalWrite(DIR_R, LOW);
    pinMode(BRK_L, OUTPUT); digitalWrite(BRK_L, HIGH); // Force Brakes ON
    pinMode(BRK_R, OUTPUT); digitalWrite(BRK_R, HIGH); // Force Brakes ON

    // 2. Encoders
    pinMode(SPD_L, INPUT_PULLUP);
    pinMode(SPD_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SPD_L), isrLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(SPD_R), isrRight, RISING);

    // 3. Buttons
    pinMode(START_BTN, INPUT_PULLUP);
    pinMode(STOP_BTN, INPUT_PULLUP);
    pinMode(ABORT_BTN, INPUT_PULLUP);

    // 4. PWM
    initPWM();
    setPWM(0,0); // Ensure 0 speed
    
    Serial.println("✅ AGVMCU INITIALIZED & SAFE (NO-HEAP MODE)");
}

// ================= CORE UPDATE LOOP =================
void AGVMCU::update() {
    // 1. DEBOUNCED BUTTONS (Priority 1)
    if (millis() - lastButtonPressTime > 100) { // 100ms debounce
        if (digitalRead(ABORT_BTN) == LOW) {
            lastButtonPressTime = millis();
            handleAbort();
        }
        else if (digitalRead(STOP_BTN) == LOW) {
            lastButtonPressTime = millis();
            handleStop();
        }
        else if (digitalRead(START_BTN) == LOW) {
            lastButtonPressTime = millis();
            handleStart();
        }
    }

    // 2. OBSTACLE TIMER LOGIC
    if (currentState == STATE_OBSTACLE_WAITING) {
        if (millis() - obstacleTimerStart > 10000) { // 10 Seconds
            Serial.println("⚠️ Obstacle Timeout! Initiating Retreat.");
            initiateRetreat();
        }
        return; // Skip PID while waiting
    }

    // 3. PID EXECUTION
    // Runs only when active
    if (currentState == STATE_MOVING || currentState == STATE_TURNING || currentState == STATE_RETURNING) {
        runPIDCycle();
    }
}

// ================= NO-HEAP COMMAND PARSING =================
// This function uses char arrays only. No String objects allowed.
void AGVMCU::processCommand(char* cmd) {
    // Skip leading whitespace/newlines if any
    while (*cmd == ' ' || *cmd == '\n' || *cmd == '\r') cmd++;

    // --- ABORT RECOVERY ---
    // If aborted, ONLY listen for "POS:" updates to unlock
    if (currentState == STATE_ABORT_WAITING_POS) {
        if (strncmp(cmd, "POS:", 4) == 0) {
            currentState = STATE_IDLE; 
            Serial.println("✅ Position Updated. Abort Cleared.");
            // Fallthrough to update variables in "POS:" section below
        } else {
            return; // Ignore other commands
        }
    }

    // --- OBSTACLE SENSOR (Format: DISTANCE:12.5) ---
    if (strncmp(cmd, "DISTANCE:", 9) == 0) {
        // cmd + 9 skips "DISTANCE:"
        float dist = atof(cmd + 9);
        
        // DETECT OBSTACLE (Atomic Stop)
        if (dist < distanceThreshold && currentState == STATE_MOVING) {
            // 1. Change State FIRST
            currentState = STATE_OBSTACLE_WAITING;
            
            // 2. Physical Stop
            motorBrake();
            
            // 3. Save State
            noInterrupts(); 
            pulsesTraveledBeforeStop = (abs(encL) + abs(encR)) / 2; 
            interrupts();
            
            obstacleTimerStart = millis();
            Serial.print("⛔ Obstacle! Paused at pulse count: "); 
            Serial.println(pulsesTraveledBeforeStop);
        }
        // CLEAR OBSTACLE
        else if (dist >= distanceThreshold && currentState == STATE_OBSTACLE_WAITING) {
            Serial.println("✅ Obstacle Cleared. Resuming.");
            currentState = STATE_MOVING; 
            // We do NOT reset encoders here, we want to finish the remaining pulses of the original move
        }
        return;
    }

    // --- PATH PARSING (Format: Manhattan path: (4,5)W(3,5)W...) ---
    if (strncmp(cmd, "Manhattan", 9) == 0) {
        totalSteps = 0;
        
        // Find first parenthesis
        char* ptr = strchr(cmd, '(');
        
        while (ptr != NULL) {
            if(totalSteps >= 50) {
                Serial.println("⚠️ Path too long. Truncating at 50.");
                break;
            }

            // Format: (x,y)D
            // ptr points to '('
            int x = atoi(ptr + 1); // Parse X (atoi stops at non-digit)
            
            ptr = strchr(ptr, ','); // Find comma
            if (!ptr) break;
            
            int y = atoi(ptr + 1); // Parse Y
            
            ptr = strchr(ptr, ')'); // Find closing paren
            if (!ptr) break;
            
            // Direction is the char immediately after ')'
            char dir = *(ptr + 1);
            
            path[totalSteps].x = x;
            path[totalSteps].y = y;
            path[totalSteps].dir = dir;
            totalSteps++;
            
            // Find next parenthesis
            ptr = strchr(ptr + 1, '(');
        }
        Serial.print("Path Received. Steps: "); Serial.println(totalSteps);
    }

    // --- POSITION UPDATE (Format: POS:x,y,angle) ---
    if (strncmp(cmd, "POS:", 4) == 0) {
        // cmd+4 is start of x
        char* ptr = cmd + 4;
        
        currentX = atoi(ptr);
        
        ptr = strchr(ptr, ',');
        if (!ptr) return;
        currentY = atoi(ptr + 1);
        
        ptr = strchr(ptr + 1, ',');
        if (!ptr) return;
        float angle = atof(ptr + 1);
        
        // Snap angle to direction
        if(angle > 315 || angle <= 45) currentDir = 'E';
        else if(angle > 45 && angle <= 135) currentDir = 'N';
        else if(angle > 135 && angle <= 225) currentDir = 'W';
        else currentDir = 'S';
        
        // If we were waiting for QR confirmation, trigger next move
        if(currentState == STATE_WAITING_QR) {
            handleStart();
        }
    }
}

// ================= PID & LOGIC ENGINE =================
void AGVMCU::runPIDCycle() {
    if (millis() - lastPIDTime < PID_INTERVAL_MS) return;

    long l, r;
    noInterrupts(); l = abs(encL); r = abs(encR); interrupts();
    long avg = (l + r) / 2;

    // --- CHECK COMPLETION ---
    if (avg >= targetPulses) {
        motorBrake();
        
        // 1. TURN COMPLETE
        if (currentState == STATE_TURNING) {
            // CRITICAL: Update Direction ONLY now that turn is done
            currentDir = targetDirPending; 
            Serial.print("Turn Done. Facing: "); Serial.println(currentDir);
            
            currentState = nextStateAfterTurn; // Transition
            
            // If logic dictates we drive immediately after turn (Retreating):
            if (currentState == STATE_RETURNING) {
                targetPulses = pulsesTraveledBeforeStop; // Drive back exact distance
                slowdownStartPulses = targetPulses * MOVE_SLOWDOWN_START;
                resetEncoders();
                motorForward();
                prevError=0; integral=0; lastPIDTime=millis(); slowdownPhase=false;
                setPWM(BASE_SPEED, BASE_SPEED);
            }
            // If Aborted
            else if (currentState == STATE_ABORT_WAITING_POS) {
                Serial.print("Task aborted and reached at ");
                Serial.print(currentX); Serial.print(","); Serial.print(currentY);
                Serial.print(currentDir); Serial.println("/[WAITING_ANGLE]");
            }
        }
        // 2. MOVE COMPLETE
        else if (currentState == STATE_MOVING) {
            // Update Logic Coordinates
            currentX = path[currentStepIndex].x;
            currentY = path[currentStepIndex].y;
            currentStepIndex++;
            
            currentState = STATE_WAITING_QR;
            Serial.println("Node Reached. Waiting for QR...");
        }
        // 3. RETREAT COMPLETE
        else if (currentState == STATE_RETURNING) {
            // We are back at previous node
            Serial.print("Blocked coordinates: ");
            Serial.print(path[currentStepIndex].x); Serial.print(","); Serial.println(path[currentStepIndex].y);
            
            Serial.print("Reached at: ");
            Serial.print(currentX); Serial.print(","); Serial.print(currentY);
            Serial.print(currentDir); Serial.println(" (Waiting for new path)");
            
            currentState = STATE_IDLE;
        }
        return;
    }

    // --- PID MATH ---
    // Select Speed
    float currentSpeedTarget = (currentState == STATE_TURNING) ? TURN_SPEED : BASE_SPEED;
    
    // Ramping
    if (avg >= slowdownStartPulses) slowdownPhase = true;
    if (slowdownPhase) {
        float progress = (float)(avg - slowdownStartPulses) / (targetPulses - slowdownStartPulses);
        progress = constrain(progress, 0.0, 1.0);
        float minSpd = FINAL_SPEED;
        // Linear ramp for move, Quadratic for turn (smoother)
        if(currentState == STATE_TURNING) 
             currentSpeedTarget = currentSpeedTarget - (currentSpeedTarget - minSpd) * (progress * progress);
        else 
             currentSpeedTarget = currentSpeedTarget - (currentSpeedTarget - minSpd) * progress;
    }

    long error = l - r;
    float dt = (millis() - lastPIDTime) / 1000.0;
    
    integral += error * dt;
    integral = constrain(integral, -100, 100); // Anti-windup
    
    float derivative = (error - prevError) / dt;
    float output = KP * error + KI * integral + KD * derivative;
    output = constrain(output, -MAX_CORRECTION, MAX_CORRECTION);

    float spdL = currentSpeedTarget - output;
    float spdR = currentSpeedTarget + output;
    
    // Basic differential steering
    if (error > 0) spdL -= abs(output); else spdR -= abs(output);

    setPWM(spdL, spdR);
    prevError = error;
    lastPIDTime = millis();
}

// ================= HANDLERS =================
void AGVMCU::handleStart() {
    if (currentStepIndex >= totalSteps) return;
    
    Step target = path[currentStepIndex];
    
    if (currentDir != target.dir) {
        rotateToDirection(currentDir, target.dir);
        nextStateAfterTurn = STATE_MOVING; // Standard flow
    } else {
        targetPulses = mmToPulses(SQUARE_SIZE_MM);
        slowdownStartPulses = targetPulses * MOVE_SLOWDOWN_START;
        resetEncoders();
        motorForward();
        currentState = STATE_MOVING;
        
        prevError=0; integral=0; lastPIDTime=millis(); slowdownPhase=false;
        setPWM(BASE_SPEED, BASE_SPEED);
    }
}

void AGVMCU::initiateRetreat() {
    // 1. Turn 180
    char backDir = getOppositeDir(currentDir);
    rotateToDirection(currentDir, backDir);
    
    // 2. Set flag to Retreat AFTER turn
    nextStateAfterTurn = STATE_RETURNING;
}

void AGVMCU::handleAbort() {
    motorBrake();
    if (currentState == STATE_ABORT_WAITING_POS) return;
    
    Serial.println("ABORT TRIGGERED!");
    
    // 1. Turn 180
    char backDir = getOppositeDir(currentDir);
    rotateToDirection(currentDir, backDir);
    
    // 2. Set flag to Wait AFTER turn
    nextStateAfterTurn = STATE_ABORT_WAITING_POS;
}

void AGVMCU::rotateToDirection(char from, char to) {
    motorBrake();
    delay(100); // Settling time
    
    int diff = 0;
    if (from == 'E') { if(to=='N') diff=90; if(to=='W') diff=180; if(to=='S') diff=-90; }
    if (from == 'N') { if(to=='W') diff=90; if(to=='S') diff=180; if(to=='E') diff=-90; }
    if (from == 'W') { if(to=='S') diff=90; if(to=='E') diff=180; if(to=='N') diff=-90; }
    if (from == 'S') { if(to=='E') diff=90; if(to=='N') diff=180; if(to=='W') diff=-90; }
    
    if (diff == 0) return; 

    bool isRight = (diff < 0);
    float deg = abs(diff);
    
    targetPulses = turnDegreesToPulses(deg);
    slowdownStartPulses = targetPulses * getTurnSlowdownStart(deg);
    
    // CRITICAL: Don't update currentDir yet!
    targetDirPending = to; 
    
    resetEncoders();
    if(isRight) motorRight(); else motorLeft();
    
    currentState = STATE_TURNING;
    prevError=0; integral=0; lastPIDTime=millis(); slowdownPhase=false;
    setPWM(TURN_SPEED, TURN_SPEED);
}

// ================= HELPERS =================
void AGVMCU::handleStop() { 
    motorBrake(); 
    currentState = STATE_STOPPED; 
    Serial.println("STOPPED"); 
}

// ESP32 LEDC PWM (Legacy API for compatibility)
void AGVMCU::initPWM() {
    ledc_timer_config_t timer = { .speed_mode=LEDC_LOW_SPEED_MODE, .duty_resolution=LEDC_TIMER_12_BIT, .timer_num=LEDC_TIMER_0, .freq_hz=2000, .clk_cfg=LEDC_AUTO_CLK };
    ledc_timer_config(&timer);
    ledc_channel_config_t ch0 = { .gpio_num=PWM_L, .speed_mode=LEDC_LOW_SPEED_MODE, .channel=LEDC_CHANNEL_0, .intr_type=LEDC_INTR_DISABLE, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 };
    ledc_channel_config(&ch0);
    ledc_channel_config_t ch1 = { .gpio_num=PWM_R, .speed_mode=LEDC_LOW_SPEED_MODE, .channel=LEDC_CHANNEL_1, .intr_type=LEDC_INTR_DISABLE, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 };
    ledc_channel_config(&ch1);
}

void AGVMCU::setPWM(float left, float right) {
    left = constrain(left, 0, 100); right = constrain(right, 0, 100);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)(left / 100.0 * 4095));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)(right / 100.0 * 4095));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void AGVMCU::motorBrake()   { digitalWrite(BRK_L, HIGH); digitalWrite(BRK_R, HIGH); setPWM(0,0); }
void AGVMCU::motorForward() { digitalWrite(DIR_L, HIGH); digitalWrite(DIR_R, HIGH); digitalWrite(BRK_L, LOW); digitalWrite(BRK_R, LOW); }
void AGVMCU::motorLeft()    { digitalWrite(DIR_L, LOW);  digitalWrite(DIR_R, HIGH); digitalWrite(BRK_L, LOW); digitalWrite(BRK_R, LOW); }
void AGVMCU::motorRight()   { digitalWrite(DIR_L, HIGH); digitalWrite(DIR_R, LOW);  digitalWrite(BRK_L, LOW); digitalWrite(BRK_R, LOW); }

void AGVMCU::resetEncoders() { 
    noInterrupts(); encL=0; encR=0; interrupts(); 
}

char AGVMCU::getOppositeDir(char dir) {
    if (dir == 'N') return 'S';
    if (dir == 'S') return 'N';
    if (dir == 'E') return 'W';
    return 'E'; 
}

long AGVMCU::mmToPulses(float mm) { 
    return (long)((mm / (PI * WHEEL_DIAMETER_MM)) * PULSES_PER_REV * LINEAR_CALIBRATION); 
}
long AGVMCU::turnDegreesToPulses(float deg) { 
    float arc = (PI * WHEEL_BASE_MM) * (deg / 360.0f); 
    return (long)((arc / (PI * WHEEL_DIAMETER_MM)) * PULSES_PER_REV * TURN_CALIBRATION); 
}
float AGVMCU::getTurnSlowdownStart(float deg) { 
    if (deg <= 45) return 0.85; if (deg <= 90) return 0.35; return 0.65; 
}
