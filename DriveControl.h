#pragma once

#include <Arduino.h>
#include "Config.h"

// Per-wheel PID speed correction for straight-line driving. 
// Not used for turning -- turns will be closed-loop against IMU heading instead,

// ticks can't tell which direction it's rotating

void driveControlInit();

// Call once when entering a fresh DRIVE_FORWARD_TO_CENTER/DRIVE_BACKWARD_TO_CENTER state
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
// when ending a DRIVE_FORWARD_TO_CENTER/DRIVE_BACKWARD_TO_CENTER run. Blocking, but short
// (DRIVE_DECEL_MS + DRIVE_SETTLE_MS at most) -- not for the emergency stop
// path, which needs to stay instant.
void driveControlStop();

// Drives straight (via driveControlUpdateStraight) while polling
// sonarGetCm() every call -- pass sonarGetFrontCm for a forward approach
// (cruiseSpeed >= 0) or sonarGetBackCm for a reverse approach (cruiseSpeed <
// 0). Every internal speed is derived from cruiseSpeed's own sign/magnitude,
// not hardcoded to forward, so the same deadband/hold state machine works
// for both. Four zones, keyed off stopCm/slowCm and SONAR_BELT_DEADBAND
// (Config.h):
//   - closer than stopCm - SONAR_BELT_DEADBAND: overshot (too close) --
//     gentle backup at SONAR_FRONT_MIN_SPEED in the opposite direction from
//     cruiseSpeed, instead of grinding into the wall.
//   - within +/- SONAR_BELT_DEADBAND of stopCm: holds at zero PWM and starts a
//     SONAR_HOLD_MS timer; any reading that drifts back outside the
//     deadband before the timer elapses cancels the hold and falls back to
//     the backup/approach zones above -- only a reading that stays inside
//     the deadband continuously for SONAR_HOLD_MS commits to "reached".
//   - between stopCm+SONAR_BELT_DEADBAND and slowCm: still approaching -- speed
//     eases linearly from cruiseSpeed down to SONAR_FRONT_MIN_SPEED (same
//     direction as cruiseSpeed).
//   - beyond slowCm: full cruiseSpeed.
// A -1 reading (no echo -- see Sonar.h) is treated as "keep cruising" only
// before the distance has ever been seen inside slowCm; once it has, a
// dropped reading means "close and unsure", so this holds at
// SONAR_FRONT_MIN_SPEED (in cruiseSpeed's direction) instead of assuming the
// way is clear (and cancels any in-progress deadband hold).
//
// Call every loop() once sonar polling should be active. Returns true only
// once the distance has held within the stopCm deadband continuously for
// SONAR_HOLD_MS -- driveControlStop() (the full decel/settle ramp) has
// already been called internally when that happens, so the caller just
// needs to transition state.
bool driveControlCruiseToSonarStop(int cruiseSpeed, float stopCm, float slowCm, float (*sonarGetCm)());

// Call once when entering a fresh straight-driving state (alongside
// driveControlReset()). Latches the heading-hold target to the robot's
// current IMU heading at this instant -- snapped ONCE here rather than
// every loop() tick, so the target can't drift mid-drive. Arena walls
// aren't guaranteed to sit on clean cardinal (0/90/180/270) headings, so
// this holds whatever heading the robot was actually placed at instead of
// snapping to the nearest cardinal.
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

// Same as turnControlStart(), but targetHeadingDeg is an absolute heading in
// the IMU's own reference frame (0-360, whatever heading imuGetHeading()
// read as 0 at boot/last recalibration -- NOT magnetic north, see IMU.h)
// rather than a delta from wherever the robot currently happens to be
// facing. Use this when the caller cares about ending up pointed the same
// direction regardless of entry heading (e.g. "face 90" from either 10 or
// 260); use turnControlStart() when the caller already knows the delta it
// wants (e.g. "turn around 180 from wherever we're facing now").
void turnControlStartAbsolute(float targetHeadingDeg);

// Call every loop() while turning; pwm is the max turn speed (ramped down
// near the target). Returns true once the turn is done (within tolerance,
// settled, or timed out -- see Config.h's TURN_* constants) and stops the
// motors.
bool turnControlUpdate(int pwm);
