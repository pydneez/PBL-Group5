#pragma once

#include <Arduino.h>
#include "Config.h"

// Attaches the servo and moves it to the open position.
void gripperInit();

enum class GripperState {
  OPENING,  // commanded open, still within GRIPPER_MS of the servo settling
  OPEN,
  CLOSING,  // commanded closed, still within GRIPPER_MS of the servo settling
  CLOSED
};

// Non-blocking: commands the servo and returns immediately.
// No-op if already open/opening (or closed/closing). 
// Call gripperControlUpdate() every loop() afterward to let the request actually settle.
void gripperRequestOpen();
void gripperRequestClose();

// Call unconditionally every loop(), regardless of the active TaskState --
// this is what lets the gripper keep moving toward its requested position
// while the robot is simultaneously driving/turning, instead of blocking
// the whole state machine like the old delay()-based version did.
void gripperControlUpdate();

GripperState gripperGetState();

// True once the servo has held its commanded position for GRIPPER_MS --
// i.e. GripperState::OPEN or GripperState::CLOSED, not still transitioning.
bool gripperIsSettled();
