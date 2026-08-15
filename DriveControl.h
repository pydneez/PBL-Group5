#pragma once

#include <Arduino.h>
#include "Config.h"

// Per-wheel PID speed correction for straight-line driving. 
// Not used for turning -- turns will be closed-loop against IMU heading instead,

// ticks can't tell which direction it's rotating

void driveControlInit();

// Call once when entering a fresh DRIVE_FORWARD_TO_CENTER/DRIVE_BACKWARD state
// Clears encoder ticks and PID history left over from whatever ran previously.
void driveControlReset();

// Call every loop() while driving straight. leftSpeed/rightSpeed are signed
// target PWM (same convention as driveSides())\
// magnitude is corrected per-wheel against encoder ticks, sign sets direction. 

// Internally throttles the actual PID recompute to DRIVE_PID_INTERVAL_MS;
// every other call just re-applies the last corrected PWM so the motors keep driving continuously.
void driveControlUpdate(int leftSpeed, int rightSpeed);

// Ramps all four wheels down to 0 from whatever they were last commanded to,
// over DRIVE_DECEL_MS, instead of cutting power in one step like stopAll()
// does -- an instant full-speed-to-zero stop jolts the chassis. Then holds
// at 0 for DRIVE_SETTLE_MS so residual momentum/coasting actually dies out
// before returning, so the caller's next move (e.g. reversing direction)
// starts from a genuinely stationary robot. Call this instead of stopAll()
// when ending a DRIVE_FORWARD_TO_CENTER/DRIVE_BACKWARD run. Blocking, but short
// (DRIVE_DECEL_MS + DRIVE_SETTLE_MS at most) -- not for the emergency stop
// path, which needs to stay instant.
void driveControlStop();

// Drives straight ahead (via driveControlUpdate) while polling the front
// sonar every call. Linearly eases speed down from cruiseSpeed toward
// SONAR_FRONT_MIN_SPEED as the front distance closes in on stopCm (see
// SONAR_FRONT_SLOWDOWN_CM in Config.h), so the robot arrives at the wall/
// belt slowing down instead of hitting it at full speed. A -1 reading (no
// echo -- out of range / nothing ahead yet, see Sonar.h) is treated as
// "keep cruising", not "extremely close".
//
// Call every loop() once sonar polling should be active. Returns true the
// moment the front distance is within stopCm -- driveControlStop() has
// already been called internally when that happens, so the caller just
// needs to transition state.
bool driveControlCruiseToSonarStop(int cruiseSpeed, float stopCm);

// Call once when entering a fresh straight-driving state (alongside
// driveControlReset()). Latches the heading-hold target to whichever
// cardinal (0/90/180/270) the robot's current IMU heading is closest to --
// snapped ONCE here rather than every loop() tick, so noise near a 45 deg
// boundary can't flip-flop the target mid-drive. This is what lets the
// robot straighten itself onto a clean cardinal even when it was placed a
// few degrees off (e.g. at 85 deg it latches 90, then corrects toward it).
void driveControlStraightStart();

// Drives straight at `speed` (signed, same convention as driveControlUpdate)
// while biasing the two wheel-side targets to correct back toward the
// heading latched by driveControlStraightStart() -- layered on top of, not
// instead of, the per-wheel encoder PID already in driveControlUpdate().
void driveControlUpdateStraight(int speed);

// Closed-loop turning against IMU heading (see the comment above: turns
// don't use encoder ticks since a single-channel encoder can't tell which
// direction it's rotating).

// Call once when entering a fresh turn state. relativeDeg is signed and
// relative to the current heading -- positive turns right/clockwise,
// negative turns left/counter-clockwise (e.g. +90 or -90 for a quarter turn).
void turnControlStart(float relativeDeg);

// Call every loop() while turning; pwm is the max turn speed (ramped down
// near the target). Returns true once the turn is done (within tolerance,
// settled, or timed out -- see Config.h's TURN_* constants) and stops the
// motors.
bool turnControlUpdate(int pwm);
