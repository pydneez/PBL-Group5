#include "Sonar.h"

float sonarGetDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout, ~5m max range

  if (duration == 0) {
    return -1; // no echo: out of range or no reflective surface
  }

  return duration * 0.0343 / 2;
}

void sonarInit() {
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  Serial.println("Sonar Setup Complete");
}

float sonarGetFrontCm() { return sonarGetDistanceCm(TRIG_FRONT, ECHO_FRONT); }
float sonarGetLeftCm()  { return sonarGetDistanceCm(TRIG_LEFT,  ECHO_LEFT); }
float sonarGetRightCm() { return sonarGetDistanceCm(TRIG_RIGHT, ECHO_RIGHT); }
