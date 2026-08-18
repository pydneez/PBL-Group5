#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Sonar.h"
#include "Pixy.h"
#include "Actuator.h"
#include "Encoder.h"
#include "DriveControl.h"
#include "IMU.h"

// ---------------- TaskState ----------------
enum class TaskState {
  IMU_CALIBRATE,     // waiting for the BNO055 to report fully calibrated before anything else runs
  IDLE,              // waiting for a Serial start command
  DRIVE_FORWARD_TO_CENTER,     // driving straight
  TURN_RIGHT_TO_BELT, // closed-loop 90 deg turn to face the belt
  TURN_LEFT_TO_BELT,         // closed-loop 90 deg turn against IMU heading
  APPROACH_BELT,     // blind drive at BELT_APPROACH_SPEED for BELT_APPROACH_MS; physical bumper (not sonar) sets the final stop distance
  DETECT_CUBE,       // waiting for Pixy to identify
  GRIPPER_OPEN,
  GRIPPER_CLOSE,
  LIFT_UP,
  LIFT_DOWN,
  BACKWARD_FROM_BELT, 
  TURN_TO_DROPZONE,  // fixed turn: red -> left, green -> right
  LOCATE_DROPZONE,   // waiting for Pixy to confirm the matching drop-off marker
  TURN_180_RECHECK_DROPZONE, // marker not found on the first look -- turn 180 and scan again
  APPROACH_DROPZONE,
  DROPPING_CUBE,     // stopped within SONAR_DROPZONE_STOP_CM; grabber not implemented yet
  DRIVE_BACKWARD_TO_CENTER,   // driving away from drop-off wall
  GATE_OPEN,        // releases carried cube(s) at the drop zone -- gate servo not wired yet, see Actuator.cpp
  GATE_CLOSE,
  DONE
};


// Set by DETECT_CUBE; drives both the turn direction and which drop-off
// marker LOCATE_DROPZONE looks for. Cleared each time IDLE is (re)entered.
CubeColor detectedCubeColor = CubeColor::NONE;

int carriedCubeCount = 0;
CubeColor carriedColor = CubeColor::NONE;

bool approachedFromYellow = true;

// True while the current LIFT_DOWN..GRIPPER_OPEN chain was kicked off by
// DETECT_CUBE (the real belt pickup loop), false when it was kicked off
// manually from IDLE's 'g' test key 
bool autoPickup = true;

// True once TURN_180_RECHECK_DROPZONE has already been used for the current drop-off leg
bool dropzoneRecoveryUsed = false;

// True only while TURN_180_RECHECK_DROPZONE is being used to turn back to
// the original heading after a second miss
bool dropzoneReturningFromRecheck = false;

TaskState state = TaskState::IDLE;
unsigned long stateEnteredAt = 0;

const char* stateName(TaskState s);
void setState(TaskState newState);

const char* stateName(TaskState s) {
  switch (s) {
    case TaskState::IMU_CALIBRATE:    return "IMU_CALIBRATE";
    case TaskState::IDLE:             return "IDLE";
    case TaskState::DRIVE_FORWARD_TO_CENTER:    return "DRIVE_FORWARD_TO_CENTER";
    case TaskState::DRIVE_BACKWARD_TO_CENTER:   return "DRIVE_BACKWARD_TO_CENTER";
    case TaskState::TURN_RIGHT_TO_BELT: return "TURN_RIGHT_TO_BELT";
    case TaskState::TURN_LEFT_TO_BELT: return "TURN_LEFT_TO_BELT";
    
    case TaskState::APPROACH_BELT: return "APPROACH_BELT";
    case TaskState::BACKWARD_FROM_BELT: return "BACKWARD_FROM_BELT";
    case TaskState::DETECT_CUBE:      return "DETECT_CUBE";

    case TaskState::TURN_TO_DROPZONE: return "TURN_TO_DROPZONE";
    case TaskState::LOCATE_DROPZONE:  return "LOCATE_DROPZONE";
    case TaskState::TURN_180_RECHECK_DROPZONE: return "TURN_180_RECHECK_DROPZONE";
    case TaskState::APPROACH_DROPZONE: return "APPROACH_DROPZONE";
    case TaskState::DROPPING_CUBE:    return "DROPPING_CUBE";
    
    case TaskState::GRIPPER_OPEN: return "GRIPPER_OPEN";
    case TaskState::GRIPPER_CLOSE: return "GRIPPER_CLOSE";
    case TaskState::LIFT_UP: return "LIFT_UP";
    case TaskState::LIFT_DOWN: return "LIFT_DOWN";
    case TaskState::GATE_OPEN: return "GATE_OPEN";
    case TaskState::GATE_CLOSE: return "GATE_CLOSE";
    case TaskState::DONE:   return "DONE";
  }
  return "?";
}

void setState(TaskState newState) {
  state = newState;
  stateEnteredAt = millis();
  Serial.print("-> ");
  Serial.println(stateName(state));
  if (state == TaskState::IDLE) {
    Serial.println("Type 'c' + Enter to recalibrate the IMU, or any other character + Enter to start...");
    detectedCubeColor = CubeColor::NONE;
    carriedCubeCount = 0;
    carriedColor = CubeColor::NONE;
    dropzoneRecoveryUsed = false;
    dropzoneReturningFromRecheck = false;
    approachedFromYellow = true;
  }
}

unsigned long timeInState() {
  return millis() - stateEnteredAt;
}


bool runSingleTurn(float relativeDeg, unsigned long& lastEnteredAt) {
  if (stateEnteredAt != lastEnteredAt) {
    lastEnteredAt = stateEnteredAt;
    turnControlStart(relativeDeg);
  }
  return turnControlUpdate(TURN_SPEED);
}

// ---------------- SETUP ----------------
void setup() {
    Serial.begin(115200);
    
    delay(500); 

    pixyInit();        
    sonarInit();         
    encoderInit();      

    driveControlInit(); 

    motorsInit();      
    gripperInit();      
    liftInit();          
    gateInit();    

    stopAll();

    if (imuInit()) {
      if (imuHasValidCalibration()) {
        Serial.println("IMU: valid calibration restored from EEPROM -- skipping calibration wait.");
        // When powered
        setState(TaskState::DRIVE_FORWARD_TO_CENTER);
        //setState(TaskState::IDLE);
      } else {
        setState(TaskState::IMU_CALIBRATE);
      }
    } else {
      Serial.println("IMU not detected -- check wiring/I2C address (Config.h: BNO055_I2C_ADDR). Turning will not be heading-corrected.");
      setState(TaskState::IDLE);
    }
}

// ----------------  MAIN LOOP ----------------
void loop() {
  switch (state) {
    case TaskState::IMU_CALIBRATE: {
      static unsigned long lastPrintAt = 0;

      if (Serial.available() > 0) {
        char c = Serial.read();
        while (Serial.available() > 0) Serial.read(); // drain the buffer
        if (c == 'c' || c == 'C') {
          imuRestartCalibration();
          setState(TaskState::IMU_CALIBRATE); // re-enter so the timeout clock restarts too
        }
      }

      if (millis() - lastPrintAt >= 200) {
        lastPrintAt = millis();
        ImuCalibration cal = imuGetCalibration();
      }

      if (imuIsFullyCalibrated()) {
        imuSaveCalibration();
        Serial.println("IMU fully calibrated.");
        setState(TaskState::IDLE);
      }
      break;
    }

    case TaskState::IDLE:
      stopAll();
      // Turns are relative to whatever heading the robot is facing when a
      // turn state is entered (see turnControlStart()), not a zeroed
      // absolute reference, so IDLE doesn't need to reset anything IMU-side
      // between runs -- calibration itself happens once in IMU_CALIBRATE above.

      if (Serial.available() > 0) {
        char c = Serial.read();
        while (Serial.available() > 0) Serial.read(); 
        if (c == 'c' || c == 'C') {
          setState(TaskState::DETECT_CUBE);
        } else if (c == 'r' || c == 'R') {
          setState(TaskState::TURN_RIGHT_TO_BELT);
        } else if (c == 'f' || c == 'F') {
          setState(TaskState::DRIVE_FORWARD_TO_CENTER);
        } else if (c == 'b' || c == 'B') {
          setState(TaskState::DRIVE_BACKWARD_TO_CENTER);
        } else if (c == 'a' || c == 'A') {
          setState(TaskState::APPROACH_BELT);
        } else if (c == 'w' || c == 'W') {
          setState(TaskState::APPROACH_DROPZONE);
        } else if (c == 'l' || c == 'L') {
          setState(TaskState::LIFT_DOWN);
        }
      }
      break;

    case TaskState::DRIVE_FORWARD_TO_CENTER: {

      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        driveControlReset();
        driveControlStraightStart();
      }

      driveControlUpdateStraight(CRUISE_SPEED);

      if (timeInState() >= DRIVE_MS) {
        driveControlStop();
        if (approachedFromYellow) {
          setState(TaskState::TURN_RIGHT_TO_BELT);
        } else {
          setState(TaskState::TURN_LEFT_TO_BELT);
        }
      }
      break;
    }

    case TaskState::DRIVE_BACKWARD_TO_CENTER: {

      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        driveControlReset();
      }

      driveControlUpdateStraight(-CRUISE_SPEED);

      if (timeInState() >= DRIVE_MS) {
        driveControlStop();
        setState(TaskState::IDLE);
      }
      break;
    }

    case TaskState::TURN_RIGHT_TO_BELT: {
      static unsigned long lastEnteredAt = 0;
      if (runSingleTurn(90, lastEnteredAt)) {
        setState(TaskState::APPROACH_BELT);
      }
      break;
    }

    case TaskState::APPROACH_BELT: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        driveControlReset();
        driveControlStraightStart();
      }

      driveControlUpdateStraight(BELT_APPROACH_SPEED);

      if (timeInState() >= BELT_APPROACH_MS) {
        stopAll();
        setState(TaskState::DETECT_CUBE);
      }
      break;
    }

    case TaskState::BACKWARD_FROM_BELT: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        driveControlReset();
        driveControlStraightStart();
      }

      driveControlUpdateStraight(-CRUISE_SPEED);

      if (timeInState() >= BACK_MS) {
        driveControlStop();

        // turn to the correct drop zone according to the colour picked up
        setState(TaskState::TURN_TO_DROPZONE);
      
      }
      break;
    }

    case TaskState::APPROACH_DROPZONE: {
      static unsigned long lastEnteredAt = 0;
      static unsigned long lastPrintAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        driveControlReset();
        driveControlStraightStart();
      }

      if (millis() - lastPrintAt >= 200) {
        lastPrintAt = millis();
        Serial.print("APPROACH_DROP_ZONE back sonar: ");
        Serial.print(sonarGetBackCm());
        Serial.println(" cm");
      }

      bool reachedDropOff = driveControlCruiseToSonarStop(DROPZONE_APPROACH_SPEED, SONAR_DROPZONE_STOP_CM, SONAR_DROPZONE_SLOW_CM, sonarGetBackCm);

      if (reachedDropOff) {
        setState(TaskState::GATE_OPEN);
      }
      break;
    }

    case TaskState::TURN_LEFT_TO_BELT: {
      static unsigned long lastEnteredAt = 0;
      if (runSingleTurn(-90, lastEnteredAt)) {
        setState(TaskState::APPROACH_BELT);
      }
      break;
    }

    case TaskState::GRIPPER_OPEN: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        gripperRequestOpen();
      }
      if (gripperIsSettled()) {
          carriedCubeCount++;
          carriedColor = detectedCubeColor;
          Serial.print("GRIPPER_OPEN: carrier now holds ");
          Serial.print(carriedCubeCount);
          Serial.print(" ");
          Serial.println(carriedColor == CubeColor::RED ? "RED" : "GREEN");

          if (carriedCubeCount < MAX_CARRIED_CUBES) {
            setState(TaskState::DETECT_CUBE);
          } else {
            setState(TaskState::BACKWARD_FROM_BELT);
          }
       
      }
      break;
    }

    case TaskState::GRIPPER_CLOSE: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        gripperRequestClose();
      }
      if (gripperIsSettled()) {
        setState(TaskState::LIFT_UP);
      }
      break;
    }
    
    case TaskState::LIFT_UP: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        liftRequestUp();
      }
      if (lifterIsSettled()) {
        setState(TaskState::GRIPPER_OPEN);
      }
      break;
    }

    case TaskState::LIFT_DOWN: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        liftRequestDown();
      }
      if (lifterIsSettled()) {
        setState(TaskState::GRIPPER_CLOSE);
      }
      break;
    }


    case TaskState::GATE_OPEN: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        gateRequestOpen();
        detectedCubeColor = CubeColor::NONE;
        carriedCubeCount = 0;
        carriedColor = CubeColor::NONE;
      }
      if (timeInState() >= GATE_OPEN_HOLD_MS) {
        setState(TaskState::GATE_CLOSE);
      }
      break;
    }

    case TaskState::GATE_CLOSE: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        gateRequestClose();
      }
      if (gateIsSettled()) {
        setState(TaskState::DRIVE_FORWARD_TO_CENTER);
      }
      break;
    }

    case TaskState::DETECT_CUBE: {
      stopAll();

      static unsigned long lastEnteredAt = 0;
      static int consecutiveMatches = 0;
      static bool announcedExtendedWait = false;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        consecutiveMatches = 0;
        announcedExtendedWait = false;
        Serial.println("DETECT_CUBE: waiting for a red/green cube...");
      }

      PixyDetection d = pixyDetect(PixyDetectMode::SEEK_CUBE);
      // Carrier already holds a color -- reject a different one outright 
      if (d.found && carriedCubeCount > 0 && d.color != carriedColor) {
        d.found = false;
      }
      // Only count frames where the cube has actually drifted into the
      // PICKUP_ZONE_* window (Pixy.h/Config.h) -- being seen isn't enough,
      // it has to be in reach of the gripper before LIFT_DOWN fires.
      if (d.found && d.inPickupZone) {
        consecutiveMatches++;
        Serial.print("DETECT_CUBE: candidate ");
        Serial.print(d.color == CubeColor::RED ? "RED" : "GREEN");
        Serial.print(" x="); Serial.print(d.x);
        Serial.print(" y="); Serial.print(d.y);
        Serial.print(" consecutive="); Serial.println(consecutiveMatches);
      } else {
        consecutiveMatches = 0;
      }

      if (consecutiveMatches >= DETECT_CUBE_CONFIRM_FRAMES) {
        detectedCubeColor = d.color;
        autoPickup = true;
        setState(TaskState::LIFT_DOWN);
        Serial.print("DETECT_CUBE: confirmed ");
        Serial.println(detectedCubeColor == CubeColor::RED ? "RED" : "GREEN");
      } else if (carriedCubeCount > 0 && timeInState() >= DETECT_CUBE_TIMEOUT_MS) {
        // Already holding at least one cube of carriedColor
        // -- settle for what we've got
        Serial.print("DETECT_CUBE: timed out, but carrying ");
        Serial.print(carriedCubeCount);
        Serial.println(" cube(s) already -- heading to drop-off.");
        setState(TaskState::BACKWARD_FROM_BELT);

      } else if (carriedCubeCount == 0 && timeInState() >= GAME_TIMEOUT_MS) {
        // Carrying nothing even after the full extended wait -- truly give up.
        Serial.println("DETECT_CUBE: timed out waiting for a cube -- returning to IDLE");
        setState(TaskState::IDLE);

      } else if (carriedCubeCount == 0 && timeInState() >= DETECT_CUBE_TIMEOUT_MS && !announcedExtendedWait) {
        // Carrying nothing -- the short per-attempt timeout doesn't apply
        // (nothing to "settle for"), so just keep scanning up to
        // GAME_TIMEOUT_MS instead of bailing to IDLE empty-handed.
        announcedExtendedWait = true;
        Serial.println("DETECT_CUBE: no cube seen yet and carrying nothing -- extending wait.");
      }
      break;
    }

    case TaskState::TURN_TO_DROPZONE: {
      // turning toward the color actually sitting in the carrier (carriedColor),
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        // fresh drop-off leg gets its own recheck attempt
        dropzoneRecoveryUsed = false;
        dropzoneReturningFromRecheck = false;
      }
      if (runSingleTurn(carriedColor == CubeColor::RED ? 90 : -90, lastEnteredAt)) {
        approachedFromYellow = (carriedColor != CubeColor::RED);
        setState(TaskState::APPROACH_DROPZONE);

      }
      break;
    }

    case TaskState::TURN_180_RECHECK_DROPZONE: {
      static unsigned long lastEnteredAt = 0;
      if (runSingleTurn(180, lastEnteredAt)) {
        // this assume we will never miss twice !!!!!

        setState(dropzoneReturningFromRecheck ? TaskState::APPROACH_DROPZONE : TaskState::LOCATE_DROPZONE);
      }
      break;
    }
  }

  gripperControlUpdate();
  liftControlUpdate();
  gateControlUpdate();
}