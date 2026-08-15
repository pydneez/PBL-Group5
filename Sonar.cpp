#include "Sonar.h"

// --- Global Timing & State Variables ---
unsigned long lastSensorTriggerTime = 0;
const long SENSOR_CYCLE_INTERVAL = 45; // 45ms between sensors to prevent cross-talk echo
int activeSensorIndex = 0;             // 0 = Front, 1 = Left, 2 = Right

// Temporary state capture variables
volatile unsigned long pulseStartTime = 0;
volatile bool waitingForEcho = false;

// Cached global distances returned instantly when requested
volatile float cachedDistFront = -1;
volatile float cachedDistLeft  = -1;
volatile float cachedDistRight = -1;

// Private helper to send a physical trigger pulse to the ultrasonic sensor
void triggerPhysicalPulse(int trigPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pulseStartTime = 0;    // Reset tracking flag
  waitingForEcho = true; // Open the window for reading the echo
}

// Private helper that drives the non-blocking state machine background tasks
void updateSonarStateMachine() {
  unsigned long currentTime = millis();

  // 1. Check if the active sensor timed out waiting for an echo (30ms timeout)
  if (waitingForEcho && (micros() - pulseStartTime > 30000) && pulseStartTime != 0) {
    waitingForEcho = false;
    if (activeSensorIndex == 0)      cachedDistFront = -1;
    else if (activeSensorIndex == 1) cachedDistLeft  = -1;
    else if (activeSensorIndex == 2) cachedDistRight = -1;
  }

  // 2. Time to advance to and trigger the next sensor in the loop sequence
  if (currentTime - lastSensorTriggerTime >= SENSOR_CYCLE_INTERVAL) {
    lastSensorTriggerTime = currentTime;
    
    // Increment through 0 (Front), 1 (Left), 2 (Right)
    activeSensorIndex = (activeSensorIndex + 1) % 3;
    waitingForEcho = false; 

    if (activeSensorIndex == 0)      triggerPhysicalPulse(TRIG_FRONT);
    else if (activeSensorIndex == 1) triggerPhysicalPulse(TRIG_LEFT);
    else if (activeSensorIndex == 2) triggerPhysicalPulse(TRIG_RIGHT);
  }

  // 3. Keep updating the live echoes instantly without blocking the microcontroller execution
  int currentEchoPin = (activeSensorIndex == 0) ? ECHO_FRONT : ((activeSensorIndex == 1) ? ECHO_LEFT : ECHO_RIGHT);
  
  if (waitingForEcho) {
    // If the pulse hasn't started tracking yet, watch for the pin to go HIGH
    if (pulseStartTime == 0 && digitalRead(currentEchoPin) == HIGH) {
      pulseStartTime = micros();
    } 
    // If it started tracking and now dropped to LOW, the echo has successfully returned!
    else if (pulseStartTime != 0 && digitalRead(currentEchoPin) == LOW) {
      long duration = micros() - pulseStartTime;
      float calculatedDistance = duration * 0.0343 / 2.0;
      
      // Filter out any invalid sensor artifacts or extreme out of bounds readings
      if (calculatedDistance > 450.0 || calculatedDistance <= 0) {
        calculatedDistance = -1; 
      }

      // Save the freshly calculated distance into our cached global repository
      if (activeSensorIndex == 0)      cachedDistFront = calculatedDistance;
      else if (activeSensorIndex == 1) cachedDistLeft  = calculatedDistance;
      else if (activeSensorIndex == 2) cachedDistRight = calculatedDistance;

      waitingForEcho = false; // Transaction complete
    }
  }
}

void sonarInit() {
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  Serial.println("Non-Blocking Sonar Setup Complete");
}

// These functions now seamlessly return the last known value instantly (0ms block)
float sonarGetFrontCm() { 
  updateSonarStateMachine();
  return cachedDistFront; 
}

float sonarGetLeftCm()  { 
  updateSonarStateMachine();
  return cachedDistLeft; 
}

float sonarGetRightCm() { 
  updateSonarStateMachine();
  return cachedDistRight; 
}

// Retained to preserve header matching compilation hooks cleanly
float sonarGetDistanceCm(int trigPin, int echoPin) {
  updateSonarStateMachine();
  if (trigPin == TRIG_FRONT) return cachedDistFront;
  if (trigPin == TRIG_LEFT)  return cachedDistLeft;
  if (trigPin == TRIG_RIGHT) return cachedDistRight;
  return -1;
}
