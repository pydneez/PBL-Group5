#pragma once

#include <Arduino.h>
#include "Config.h"

// ============================================================================
// HEADING REFERENCE (shared by straight driving and turning)
//
// desiredHeading is maintained in software and only ever changed by exact
// increments via headingAdvance(). It is NEVER re-seeded from the sensor
// mid-run. That matters: the old turnControlStart() did
//     target = imuGetHeading() + relativeDeg
// which baked each turn's settling error into the next turn's baseline, so
// error accumulated with every turn and could never be recovered. With a
// software reference, a turn that finishes 3 deg short leaves a 3 deg error
// that the NEXT straight drive actively corrects, instead of becoming the new
// definition of "straight".
// ============================================================================

// Zeroes desiredHeading to whatever the robot is pointing at right now. Call
// once at the start of a run -- e.g. with the robot's nose on the start tape,
// where its true field orientation is known.
void headingReset();

// Rotates the reference by an exact amount. Positive = clockwise/right, to
// match turnControlStart()'s convention.
void headingAdvance(float deg);

float headingGetDesired();

// Shortest signed error from current heading to the reference, -180..180.
// Positive means the robot needs to rotate clockwise to get back on target.
float headingError();

// ============================================================================
// STRAIGHT-LINE DRIVING
// ============================================================================

void driveControlInit();

// Call once when entering a fresh drive state. 
// Clears odometry and the heading controller's derivative history.
void driveControlReset();

// Call every loop() while driving straight. baseSpeed is signed: 
// positive drives forward, negative reverses.
//  Both sides get the same base speed; the
// IMU heading error is applied as a differential on top of it.

// This REPLACES the old four-independent-wheel-speed PID. That version drove
// every wheel toward a fixed tick target regardless of the commanded speed,
// and equal wheel speed does not mean a straight path once wheel diameter,
// slip and weight distribution differ -- which they do, or TURN_SCALE_*
// wouldn't need to exist.
void driveControlUpdate(int baseSpeed);

// Distance travelled since the last driveControlReset(), in cm, averaged over
// all four wheels. Magnitude only: the single-channel encoders can't tell
// forward from backward, so reversing also increases this.
// Requires DRIVE_TICKS_PER_CM to be measured -- see DRIVE_CALIBRATE.
float driveControlDistanceCm();

// Ramps all four wheels down to 0 over DRIVE_DECEL_MS, then holds for
// DRIVE_SETTLE_MS so residual momentum dies out. Still blocking (~550 ms) --
// call it at the END of a drive state, never in the emergency stop path.
void driveControlStop();

// ============================================================================
// TURNING (closed-loop against the shared heading reference)
// ============================================================================

// Call once when entering a fresh turn state. relativeDeg is signed:
// positive turns right/clockwise, negative turns left/counter-clockwise.
void turnControlStart(float relativeDeg);

// Call every loop() while turning; pwm is the max turn speed (ramped down
// near the target). Returns true once the turn is done -- see Config.h's
// TURN_* constants -- and stops the motors.
bool turnControlUpdate(int pwm);