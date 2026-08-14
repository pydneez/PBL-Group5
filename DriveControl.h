#pragma once

#include <Arduino.h>
#include "Config.h"

// Per-wheel PID speed correction for straight-line driving. 
// Not used for turning -- turns will be closed-loop against IMU heading instead,

// ticks can't tell which direction it's rotating

void driveControlInit();

// Call once when entering a fresh DRIVE_FORWARD/DRIVE_BACKWARD state
// Clears encoder ticks and PID history left over from whatever ran previously.
void driveControlReset();

// Call every loop() while driving straight. leftSpeed/rightSpeed are signed
// target PWM (same convention as driveSides())\
// magnitude is corrected per-wheel against encoder ticks, sign sets direction. 

// Internally throttles the actual PID recompute to DRIVE_PID_INTERVAL_MS; 
// every other call just re-applies the last corrected PWM so the motors keep driving continuously.
void driveControlUpdate(int leftSpeed, int rightSpeed);
