#pragma once

#include <Arduino.h>
#include "Config.h"

// Attaches the servo and moves it to the open position.
void gripperInit();
void liftInit();

// Third actuator: the box gate that releases carried cube(s) at the drop
// zone. Not physically wired yet (GATE_SERVO_PIN is PIN_NOT_WIRED in
// Config.h, waiting on the 3D-printed box) -- gateInit() detects that and
// every gateXxx() call below becomes a safe no-op until a real pin exists,
// same pattern Encoder.cpp uses for unwired encoder pins.
void gateInit();

enum class GripperState {
  OPENING,  // commanded open, still within GRIPPER_MS
  OPEN,
  CLOSING,  // commanded closed, still within GRIPPER_MS
  CLOSED
};

enum class LiftState {
  LIFTING_UP,  // commanded open
  LIFTED_UP,
  LIFTING_DOWN,  // commanded closed, still within LIFTING_MS
  LIFTED_DOWN
};

enum class GateState {
  OPENING,  // commanded open, still within GATE_OPEN_HOLD_MS
  OPEN,
  CLOSING,  // commanded closed, still within GATE_OPEN_HOLD_MS
  CLOSED
};

// Non-blocking: commands the servo and returns immediately.
// No-op if already open/opening (or closed/closing).
// Call gripperControlUpdate() every loop() afterward to let the request actually settle.
void gripperRequestOpen();
void gripperRequestClose();

void liftRequestUp();
void liftRequestDown();

void gateRequestOpen();
void gateRequestClose();

// Call unconditionally every loop(), regardless of the active TaskState --
// this is what lets the gripper keep moving toward its requested position
// while the robot is simultaneously driving/turning, instead of blocking
// the whole state machine like the old delay()-based version did.
void gripperControlUpdate();
void liftControlUpdate();
void gateControlUpdate();

GripperState gripperGetState();
LiftState lifterGetState();
GateState gateGetState();

// True once the servo has held its commanded position for GRIPPER_MS --
// i.e not still transitioning.
bool gripperIsSettled();
bool lifterIsSettled();

// Always true (no-op settled) while GATE_SERVO_PIN is PIN_NOT_WIRED -- see
// gateInit() above.
bool gateIsSettled();
