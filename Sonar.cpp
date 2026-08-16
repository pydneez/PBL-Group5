#include "Sonar.h"

// --- Front sonar: dedicated, throttled, pulseIn()-based ---
// pulseIn() blocks for the duration of the actual echo pulse (bounded by
// FRONT_ECHO_TIMEOUT_US below), but that's a feature here, not a bug: at
// close range the echo pulse is only a few hundred microseconds wide (e.g.
// ~400us at 7cm, ~120us at 2cm) -- too short to reliably catch by polling
// digitalRead() from the main loop in between other blocking work (e.g. the
// BNO055 I2C read every APPROACH_BELT iteration). pulseIn() busy-waits
// internally so it can't miss the edges. Front is what the collision-stop
// logic depends on, so reliability here matters more than staying
// non-blocking; FRONT_PING_INTERVAL_MS keeps the blocking bounded and
// infrequent. Behavior confirmed against a standalone bench test
// (test_sonar_wall.ino) before porting in here.
static unsigned long lastFrontPingAt = 0;
static const long FRONT_PING_INTERVAL_MS = 60;
static const unsigned long FRONT_ECHO_TIMEOUT_US = 15000; // ~257cm max range
static float cachedDistFront = -1;

static void pingFront() {
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);

  unsigned long duration = pulseIn(ECHO_FRONT, HIGH, FRONT_ECHO_TIMEOUT_US);
  cachedDistFront = (duration > 0) ? (duration * 0.0343f / 2.0f) : -1;
}

float sonarGetFrontCm() {
  unsigned long now = millis();
  if (now - lastFrontPingAt >= FRONT_PING_INTERVAL_MS) {
    lastFrontPingAt = now;
    pingFront();
  }
  return cachedDistFront;
}

// --- Left/right sonar: unchanged non-blocking round-robin state machine ---
// Neither is on the collision-stop path (used for side clearance/centering
// instead), so the missed-short-pulse-at-close-range tradeoff that front
// had doesn't bite the same way here, and there's no need to pay pulseIn()'s
// blocking cost for two more sensors every cycle.
static unsigned long lastSensorTriggerTime = 0;
static const long SENSOR_CYCLE_INTERVAL = 45; // ms between sensors to prevent cross-talk echo
static int activeSensorIndex = 0;             // 0 = Left, 1 = Right

static volatile unsigned long pulseStartTime = 0;
static volatile bool waitingForEcho = false;

static volatile float cachedDistLeft  = -1;
static volatile float cachedDistRight = -1;

static void triggerPhysicalPulse(int trigPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pulseStartTime = 0;    // Reset tracking flag
  waitingForEcho = true; // Open the window for reading the echo
}

static void updateSideSonarStateMachine() {
  unsigned long currentTime = millis();

  // 1. Check if the active sensor timed out waiting for an echo (30ms timeout)
  if (waitingForEcho && (micros() - pulseStartTime > 30000) && pulseStartTime != 0) {
    waitingForEcho = false;
    if (activeSensorIndex == 0) cachedDistLeft  = -1;
    else                        cachedDistRight = -1;
  }

  // 2. Time to advance to and trigger the next sensor in the loop sequence
  if (currentTime - lastSensorTriggerTime >= SENSOR_CYCLE_INTERVAL) {
    lastSensorTriggerTime = currentTime;

    activeSensorIndex = (activeSensorIndex + 1) % 2;
    waitingForEcho = false;

    if (activeSensorIndex == 0) triggerPhysicalPulse(TRIG_LEFT);
    else                        triggerPhysicalPulse(TRIG_RIGHT);
  }

  // 3. Keep updating the live echo instantly without blocking
  int currentEchoPin = (activeSensorIndex == 0) ? ECHO_LEFT : ECHO_RIGHT;

  if (waitingForEcho) {
    if (pulseStartTime == 0 && digitalRead(currentEchoPin) == HIGH) {
      pulseStartTime = micros();
    } else if (pulseStartTime != 0 && digitalRead(currentEchoPin) == LOW) {
      long duration = micros() - pulseStartTime;
      float calculatedDistance = duration * 0.0343 / 2.0;

      if (calculatedDistance > 450.0 || calculatedDistance <= 0) {
        calculatedDistance = -1;
      }

      if (activeSensorIndex == 0) cachedDistLeft  = calculatedDistance;
      else                        cachedDistRight = calculatedDistance;

      waitingForEcho = false;
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

  Serial.println("Sonar Setup Complete (front: blocking pulseIn, left/right: non-blocking)");
}

float sonarGetLeftCm()  {
  updateSideSonarStateMachine();
  return cachedDistLeft;
}

float sonarGetRightCm() {
  updateSideSonarStateMachine();
  return cachedDistRight;
}

// Retained to preserve header matching compilation hooks cleanly
float sonarGetDistanceCm(int trigPin, int echoPin) {
  (void)echoPin;
  if (trigPin == TRIG_FRONT) return sonarGetFrontCm();
  if (trigPin == TRIG_LEFT)  return sonarGetLeftCm();
  if (trigPin == TRIG_RIGHT) return sonarGetRightCm();
  return -1;
}
