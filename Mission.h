#pragma once

#include <Arduino.h>
#include "Config.h"
#include "Pixy.h"

// ===========================================================================
// MISSION SEQUENCE
//
// One competition round, start tape to buzzer. Non-blocking: missionUpdate()
// is called every loop() and returns immediately.
//
// Design rules this follows, and why:
//
//  * Every turn targets an ABSOLUTE field heading, never a relative "turn 90
//    left". A turn that lands 6 deg short leaves a 6 deg error the next
//    correction removes, instead of that error becoming the new baseline.
//
//  * The robot approaches the belt STRAIGHT ON, on a fixed field heading, and
//    never steers toward a cube. Chasing a laterally-moving cube with a
//    skid-steer is an unwinnable pursuit, and it destroys the known heading
//    that makes the return trip deterministic. Vision drives grab TIMING only.
// ===========================================================================

enum class MissionState : uint8_t {
  IDLE,
  ZERO_HEADING,
  LEAVE_START,
  TURN_TO_BELT,
  APPROACH_BELT,
  MEASURE_BELT,
  SELECT_COLOUR,
  CLOSE_IN,
  WAIT_AND_GRAB,
  VERIFY_GRAB,
  STOW,
  DECIDE,
  BACK_OFF,
  TURN_TO_DROP,
  DRIVE_TO_DROP,
  LOCATE_MARKER,
  ALIGN_MARKER,
  RELEASE,
  RETURN_TURN,
  RECOVER,
  FINISHED
};

void missionInit();

void missionStart();

void missionAbort();

void missionUpdate();

MissionState missionGetState();
const char* missionStateName(MissionState s);
bool missionIsRunning();

// Seconds left in the round.
float missionTimeLeftS();