#pragma once

#include <Arduino.h>
#include "Config.h" // Bring in your pin definitions

// Function Declarations (kept in your original sequence)
void setWheel(int pwmPin, int in1Pin, int in2Pin, int speed);
void stopAll();
void motorsInit();
void encodersSetDirection(bool leftForward, bool rightForward);
void driveSides(int leftSpeed, int rightSpeed);

// Motion helpers
void driveForward();
void driveBackward();
void turnLeft();
void turnRight();