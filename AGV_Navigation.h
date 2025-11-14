#ifndef AGV_NAVIGATION_H
#define AGV_NAVIGATION_H

#include <Arduino.h>

class AGV_Navigation {
public:
    // Constructor with pin definitions
    AGV_Navigation(int ena, int in1, int in2, int enb, int in3, int in4,
                   int startBtn, int stopBtn, int abortBtn, int defaultBtn);
    
    // Main initialization
    void begin();
    
    // Main loop function - call this in your Arduino loop()
    void run();
    
    // Process serial commands from ROS2
    void processCommand(String command);
    
    // Get current status
    String getStatus();
    bool isNavigating();
    bool isWaitingForQR();

private:
    // Motor control
    int _ena, _in1, _in2, _enb, _in3, _in4;
    
    // Button pins
    int _startBtn, _stopBtn, _abortBtn, _defaultBtn;
    
    // Navigation state (same as your original code)
    bool isStarted = false;
    bool isStopped = false;
    bool goalReached = false;
    bool awaitingPositionAfterNewPath = false;
    bool movementCompleted = false;
    bool abortActive = false;
    bool waitingForQRAfterAbort = false;
    bool distanceTimeoutActive = false;
    
    int currentX = -1, currentY = -1;
    char currentDir = 'E';
    int totalSteps = 0, currentStep = 0;
    
    struct Step {
        int x, y;
        char dir;
    };
    static const int maxSteps = 20;
    Step steps[maxSteps];
    
    // Private methods (same as your functions)
    void handleButtons();
    void handleSerialInput();
    void handleAbort();
    void handleDistanceTimeout();
    void triggerRecoveryProcedure(const char* source);
    void moveOneCell();
    void rotateAngle(float degrees);
    void correctDirectionUsingQRAngle(float qrAngle);
    void checkDistanceCondition(float currentDistance);
    void parsePath(String raw);
    void moveForward();
    void stopMotors();
    void rotateToDirection(char from, char to);
    void publishCurrentPosition();
};

#endif
