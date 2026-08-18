#include "Sonar.h"

// --- Front and Back sonar: dedicated, throttled, pulseIn()-based ---
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
static const long FRONT_PING_INTERVAL_MS = 45;
static const unsigned long FRONT_ECHO_TIMEOUT_US = 15000; // ~257cm max range
static float cachedDistFront = -1;

static unsigned long lastBackPingAt = 0;
static const long BACK_PING_INTERVAL_MS = 30;
static const unsigned long BACK_ECHO_TIMEOUT_US = 15000; // ~257cm max range
static float cachedDistBack = -1;

static void pingFront() {
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);

  unsigned long duration = pulseIn(ECHO_FRONT, HIGH, FRONT_ECHO_TIMEOUT_US);
  cachedDistFront = (duration > 0) ? (duration * 0.0343f / 2.0f) : -1;
}

static void pingBack() {
  digitalWrite(TRIG_BACK, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_BACK, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_BACK, LOW);

  unsigned long duration = pulseIn(ECHO_BACK, HIGH, BACK_ECHO_TIMEOUT_US);
  cachedDistBack = (duration > 0) ? (duration * 0.0343f / 2.0f) : -1;
}

float sonarGetFrontCm() {
  unsigned long now = millis();
  if (now - lastFrontPingAt >= FRONT_PING_INTERVAL_MS) {
    lastFrontPingAt = now;
    pingFront();
  }
  return cachedDistFront;
}

float sonarGetBackCm() {
  unsigned long now = millis();
  if (now - lastBackPingAt >= BACK_PING_INTERVAL_MS) {
    lastBackPingAt = now;
    pingBack();
  }
  return cachedDistBack;
}

// Retained to preserve header matching compilation hooks cleanly
float sonarGetDistanceCm(int trigPin, int echoPin) {
  (void)echoPin;
  if (trigPin == TRIG_FRONT) return sonarGetFrontCm();
  if (trigPin == TRIG_BACK)  return sonarGetBackCm();
  return -1;
}

static unsigned long lastSensorTriggerTime = 0;

static volatile unsigned long pulseStartTime = 0;
static volatile bool waitingForEcho = false;


static void triggerPhysicalPulse(int trigPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pulseStartTime = 0;    // Reset tracking flag
  waitingForEcho = true; // Open the window for reading the echo
}

void sonarInit() {
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);

  Serial.println("Sonar Setup Complete (front: blocking pulseIn, left/right: non-blocking)");
}
