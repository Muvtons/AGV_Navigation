#include "AGVMCU.h"

void setup() {
    // Initialize the AGV system - it starts working automatically!
    agvmcu.begin(115200);
}

void loop() {
    // This single call keeps the entire navigation system running
    agvmcu.update();
}
