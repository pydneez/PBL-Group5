#include "Mission.h"
#include "DriveControl.h"
#include "Motors.h"
#include "Encoder.h"
#include "IMU.h"
#include "Sonar.h"
#include "Gripper.h"


// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static MissionState mstate = MissionState::IDLE;
static unsigned long mstateEnteredAt = 0;
static unsigned long matchStartMs = 0;
static bool running = false;

static int carriedCount = 0;
static CubeColor carriedColour = CubeColor::NONE;

static float beltSpeedPxPerS = 0.0f;
static float beltSpeedCmPerS = 0.0f;
static bool beltMeasured = false;

static int recoverAttempts = 0;
static unsigned long lastLogAt = 0;

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

const char* missionStateName(MissionState s) {
  switch (s) {
    case MissionState::IDLE:          return "IDLE";
    case MissionState::ZERO_HEADING:  return "ZERO_HEADING";
    case MissionState::LEAVE_START:   return "LEAVE_START";
    case MissionState::TURN_TO_BELT:  return "TURN_TO_BELT";
    case MissionState::APPROACH_BELT: return "APPROACH_BELT";
    case MissionState::MEASURE_BELT:  return "MEASURE_BELT";
    case MissionState::SELECT_COLOUR: return "SELECT_COLOUR";
    case MissionState::CLOSE_IN:      return "CLOSE_IN";
    case MissionState::WAIT_AND_GRAB: return "WAIT_AND_GRAB";
    case MissionState::VERIFY_GRAB:   return "VERIFY_GRAB";
    case MissionState::STOW:          return "STOW";
    case MissionState::DECIDE:        return "DECIDE";
    case MissionState::BACK_OFF:      return "BACK_OFF";
    case MissionState::TURN_TO_DROP:  return "TURN_TO_DROP";
    case MissionState::DRIVE_TO_DROP: return "DRIVE_TO_DROP";
    case MissionState::LOCATE_MARKER: return "LOCATE_MARKER";
    case MissionState::ALIGN_MARKER:  return "ALIGN_MARKER";
    case MissionState::RELEASE:       return "RELEASE";
    case MissionState::RETURN_TURN:   return "RETURN_TURN";
    case MissionState::RECOVER:       return "RECOVER";
    case MissionState::FINISHED:      return "FINISHED";
  }
  return "?";
}

static const char* colourName(CubeColor c) {
  if (c == CubeColor::GREEN) return "GREEN";
  if (c == CubeColor::RED)   return "RED";
  return "NONE";
}

static const char* gripName() {
  switch (gripperGetState()) {
    case GripperState::OPENING: return "OPENING";
    case GripperState::OPEN:    return "OPEN";
    case GripperState::CLOSING: return "CLOSING";
    case GripperState::CLOSED:  return "CLOSED";
  }
  return "?";
}

static const char* liftName() {
  switch (lifterGetState()) {
    case LiftState::LIFTING_UP:   return "LIFTING_UP";
    case LiftState::LIFTED_UP:    return "UP";
    case LiftState::LIFTING_DOWN: return "LIFTING_DOWN";
    case LiftState::LIFTED_DOWN:  return "DOWN";
  }
  return "?";
}


static unsigned long timeInMissionState() {
  return millis() - mstateEnteredAt;
}

MissionState missionGetState() { 
  return mstate; 
}

bool missionIsRunning() { return running; }

