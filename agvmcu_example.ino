#include "AGVMCU.h"

// ================= MEMORY SAFE SERIAL BUFFER =================
// This replaces the String class reading logic.
// We allocate this once at startup, so heap memory is never fragmented.
const int MAX_CMD_LEN = 200;
char serialBuffer[MAX_CMD_LEN];
int bufferIndex = 0;

void setup() {
    // Initialize hardware and logic
    agvmcu.begin();
}

void loop() {
    // 1. Run the State Machine & PID Loop
    agvmcu.update();
    
    // 2. Non-blocking Serial Reader
    // Reads one character per loop cycle
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // Check for end of command (Newline)
        if (c == '\n') {
            serialBuffer[bufferIndex] = '\0'; // Null terminate the string
            agvmcu.processCommand(serialBuffer); // Process the static buffer
            bufferIndex = 0; // Reset buffer for next command
        } 
        // Check for carriage return (ignore it)
        else if (c != '\r') {
            // Add char to buffer if there is space
            if (bufferIndex < MAX_CMD_LEN - 1) {
                serialBuffer[bufferIndex++] = c;
            } else {
                // Buffer Overflow Protection
                // If command is too long, we discard it to protect memory
                bufferIndex = 0; 
                Serial.println("⚠️ Command too long! Discarded buffer.");
            }
        }
    }
}
