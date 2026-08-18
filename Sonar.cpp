#include "Sonar.h"

static unsigned long lastBackPingAt = 0;
static const long BACK_PING_INTERVAL_MS = 50;
static const unsigned long BACK_ECHO_TIMEOUT_US = 15000; // ~257cm max range
static float cachedDistBack = -1;

static void pingBack() {
  digitalWrite(TRIG_BACK, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_BACK, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_BACK, LOW);

  unsigned long duration = pulseIn(ECHO_BACK, HIGH, BACK_ECHO_TIMEOUT_US);
  cachedDistBack = (duration > 0) ? (duration * 0.0343f / 2.0f) : -1;
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
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);

  Serial.println("Sonar Setup Complete");
}
