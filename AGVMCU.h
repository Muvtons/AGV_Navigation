#ifndef AGVMCU_H
#define AGVMCU_H

#include <Arduino.h>

// Motor control pins for L298N
#define ENA 12
#define IN1 14
#define IN2 27
#define ENB 13
#define IN3 26
#define IN4 25

// Button pins
#define START_BUTTON_PIN 33
#define STOP_BUTTON_PIN 32
#define ABORT_BUTTON_PIN 35

// States
enum State {
    STATE_IDLE,
    STATE_WAITING_FOR_QR,
    STATE_MOVING,
    STATE_WAITING_QR_CONFIRMATION,
    STATE_STOPPED,
    STATE_ABORTED,
    STATE_GOAL_REACHED,
    STATE_OBSTACLE_RECOVERY
};

// Step structure for path
struct Step {
    int x;
    int y;
    char dir;
};

class AGVMCU {
private:
    // Motor control
    void moveForward();
    void moveBackward();
    void stopMotors();
    void rotateAngle(float degrees);
    
    // Navigation
    void rotateToDirection(char from, char to);
    void correctDirectionUsingQRAngle(float qrAngle);
    void navigateToNextStep();
    bool isAtPosition(int x, int y);
    void parsePath(String raw);
    
    // Input handlers
    void handleSerialInput();
    void handleButtons();
    
    // Command handlers
    void handleAbort();
    void handleStop(); 
    void handleStart();
    void handleObstacle();
    
    // Utility
    void publishCurrentPosition();
    
    // Navigation variables
    Step path[50];
    int totalSteps;
    int currentStepIndex;
    int currentX, currentY;
    char currentDir;
    State currentState;
    
    // Obstacle handling
    float distanceThreshold;
    bool distanceBelowThreshold;
    unsigned long distanceBelowThresholdStart;
    
    // Button debouncing
    unsigned long lastAbortPress, lastStartPress, lastStopPress;
    
    // Input buffer
    String inputBuffer;

public:
    AGVMCU();
    void begin(long baudRate = 115200);
    void run(); // Main loop function to be called continuously
};

// Global instance
extern AGVMCU agvmcu;

#endif
