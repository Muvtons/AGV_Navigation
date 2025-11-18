#ifndef AGVMCU_H
#define AGVMCU_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

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

// Message types for inter-core communication
enum MessageType {
    MSG_PATH,
    MSG_QR,
    MSG_DISTANCE,
    MSG_COMMAND,
    MSG_BUTTON
};

struct Message {
    MessageType type;
    String data;
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
    
    // Core 0 functions
    static void core0Task(void* parameter);
    void handleCore0();
    void handleSerialInput();
    void handleButtons();
    
    // Core 1 functions  
    static void core1Task(void* parameter);
    void handleCore1();
    void processMessage(Message msg);
    
    // Command handlers
    void handleAbort();
    void handleStop(); 
    void handleStart();
    void handleObstacle();
    
    // Utility
    void publishCurrentPosition();
    void publishRerouteCommand(int src_x, int src_y, int dst_x, int dst_y);
    
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
    
    // Shared data protection
    SemaphoreHandle_t xMutex;
    QueueHandle_t xQueue;

public:
    AGVMCU();
    void begin(long baudRate = 115200);
};

// Global instance
extern AGVMCU agvmcu;

#endif
