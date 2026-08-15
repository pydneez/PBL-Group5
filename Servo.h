#pragma once

#include <Arduino.h>
#include "Config.h"

// Attaches the servo and moves it to the open position.
void gripperInit();
void liftInit();

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

// Non-blocking: commands the servo and returns immediately.
// No-op if already open/opening (or closed/closing). 
// Call gripperControlUpdate() every loop() afterward to let the request actually settle.
void gripperRequestOpen();
void gripperRequestClose();

void liftRequestUp();
void liftRequestDown();

// Call unconditionally every loop(), regardless of the active TaskState --
// this is what lets the gripper keep moving toward its requested position
// while the robot is simultaneously driving/turning, instead of blocking
// the whole state machine like the old delay()-based version did.
void gripperControlUpdate();
void liftControlUpdate();

GripperState gripperGetState();
LiftState lifterGetState();

// True once the servo has held its commanded position for GRIPPER_MS --
// i.e not still transitioning.
bool gripperIsSettled();
bool lifterIsSettled();
