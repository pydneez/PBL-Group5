#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Sonar.h"
#include "Pixy.h"
#include "Servo.h"
#include "Encoder.h"
#include "DriveControl.h"
#include "IMU.h"

// ---------------- TaskState ----------------
enum class TaskState {
  IMU_CALIBRATE,     // waiting for the BNO055 to report fully calibrated before anything else runs
  IDLE,              // waiting for a Serial start command
  DETECT_CUBE,       // waiting for Pixy to identify
  TURN_TO_DROPZONE,  // fixed turn: red -> left, green -> right
  LOCATE_DROPZONE,   // waiting for Pixy to confirm the matching drop-off marker
  DRIVE_FORWARD_TO_CENTER,     // driving straight, open-loop; sonar polling starts after DRIVE_BEFORE_SONAR_MS
  DROPPING_CUBE,     // stopped within SONAR_DROPZONE_STOP_CM; grabber not implemented yet
  DRIVE_BACKWARD,   // driving away from drop-off wall
  TURN_RIGHT_TO_BELT, // closed-loop 90 deg turn to face the belt
  APPROACH_BELT,     // driving straight at APPROACH_SPEED; stops within SONAR_BELT_STOP_CM
  APPROACH_DROP_OFF,
  TURN_LEFT_TO_BELT,         // closed-loop 90 deg turn against IMU heading
  TURN_180,
  GRIPPER_OPEN,
  GRIPPER_CLOSE,
  ENCODER_TEST,
  TURN_TEST,
  DRIVE_PID_TEST,    // forward 2000ms, backward 2000ms, once -- re-baseline DRIVE_PID_TARGET_TICKS_PER_INTERVAL after a weight change
  DONE
};


// Set by DETECT_CUBE; drives both the turn direction and which drop-off
// marker LOCATE_DROPZONE looks for. Cleared each time IDLE is (re)entered.
CubeColor detectedCubeColor = CubeColor::NONE;

// Counts completed round trips of the DRIVE_FORWARD_TO_CENTER<->DRIVE_BACKWARD PID
// calibration shuttle. Reset to 0 when IDLE kicks off a fresh run.
int driveTestRoundTrip = 0;

TaskState state = TaskState::IDLE;
unsigned long stateEnteredAt = 0;

// Written out manually so the IDE's auto-generated prototypes
// (inserted above the enum) don't reference TaskState before it's defined.
const char* stateName(TaskState s);
void setState(TaskState newState);

const char* stateName(TaskState s) {
  switch (s) {
    case TaskState::IMU_CALIBRATE:    return "IMU_CALIBRATE";
    case TaskState::IDLE:             return "IDLE";
    case TaskState::DRIVE_FORWARD_TO_CENTER:    return "DRIVE_FORWARD_TO_CENTER";
    case TaskState::DRIVE_BACKWARD:   return "DRIVE_BACKWARD";
    case TaskState::TURN_RIGHT_TO_BELT: return "TURN_RIGHT_TO_BELT";
    case TaskState::TURN_LEFT_TO_BELT: return "TURN_LEFT_TO_BELT";
    case TaskState::TURN_180: return "TURN_180";
    
    case TaskState::APPROACH_BELT: return "APPROACH_BELT";
    case TaskState::DETECT_CUBE:      return "DETECT_CUBE";

    case TaskState::TURN_TO_DROPZONE: return "TURN_TO_DROPZONE";
    case TaskState::LOCATE_DROPZONE:  return "LOCATE_DROPZONE";
    //case TaskState::PICKING_UP_CUBE:    return "PICKING_UP_CUBE";
    case TaskState::APPROACH_DROP_OFF: return "APPROACH_DROP_OFF";
    case TaskState::DROPPING_CUBE:    return "DROPPING_CUBE";
    
    case TaskState::GRIPPER_OPEN: return "GRIPPER_OPEN";
    case TaskState::GRIPPER_CLOSE: return "GRIPPER_CLOSE";

    case TaskState::ENCODER_TEST: return "ENCODER_TEST";
    case TaskState::TURN_TEST: return "TURN_TEST";
    case TaskState::DRIVE_PID_TEST: return "DRIVE_PID_TEST";
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
  }
}

unsigned long timeInState() {
  return millis() - stateEnteredAt;
}

// Runs a single closed-loop turn: starts it once when the state is freshly
// entered, then just polls turnControlUpdate() on every later call. Returns
// true once the turn has completed AND the robot has actually stopped at
// the target heading -- finishTurn() (DriveControl.cpp) cuts power
// immediately and settles before reporting done, so a true return here
// means it's safe to move on to the next TaskState.
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
    motorsInit();
    sonarInit();
    pixyInit();

    gripperInit();
    liftInit();
    
    encoderInit();
    driveControlInit();
    stopAll();

    if (imuInit()) {
      if (imuHasValidCalibration()) {
        Serial.println("IMU: valid calibration restored from EEPROM -- skipping calibration wait.");
        // When powered
        //setState(TaskState::DRIVE_FORWARD_TO_CENTER);
        setState(TaskState::IDLE);
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
        //Serial.print("IMU calibrating -- move it gently through different orientations. Sys:");
        //Serial.print(cal.system);
        //Serial.print(" Gyro:"); Serial.print(cal.gyro);
        //Serial.print(" Accel:"); Serial.print(cal.accel);
        //Serial.print(" Mag:"); Serial.println(cal.mag);
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
        while (Serial.available() > 0) Serial.read(); // drain the buffer
        if (c == 'c' || c == 'C') {
          imuRestartCalibration();
          setState(TaskState::IMU_CALIBRATE);
        } else if (c == 't' || c == 'T') {
          setState(TaskState::TURN_TEST);
        } else if (c == 'l' || c == 'L') {
          setState(TaskState::TURN_LEFT_TO_BELT);
        } else if (c == 'r' || c == 'R') {
          setState(TaskState::TURN_RIGHT_TO_BELT);
        } else if (c == 'u' || c == 'U') {
          setState(TaskState::TURN_180);
        } else if (c == 'f' || c == 'F') {
          setState(TaskState::DRIVE_FORWARD_TO_CENTER);
        } else if (c == 'b' || c == 'B') {
          setState(TaskState::DRIVE_BACKWARD);
        } else if (c == 'g' || c == 'G') {
          setState(TaskState::GRIPPER_CLOSE);
        } else if (c == 'i' || c == 'I') {
          setState(TaskState::IDLE);
        } else if (c == 'p' || c == 'P') {
          setState(TaskState::DRIVE_PID_TEST);
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

        // TURN_RIGHT to face the  belt
        //if (runSingleTurn(90, lastEnteredAt)) {
        //setState(TaskState::DETECT_CUBE);
        setState(TaskState::TURN_RIGHT_TO_BELT);
      
      }
      break;
    }

    case TaskState::DRIVE_BACKWARD: {

      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        driveControlReset();
      }

      driveControlUpdate(-CRUISE_SPEED, -CRUISE_SPEED);

      if (timeInState() >= BACK_MS) {
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
      static unsigned long lastPrintAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        driveControlReset();
        driveControlStraightStart();
      }

      if (millis() - lastPrintAt >= 200) {
        lastPrintAt = millis();
        Serial.print("APPROACH_BELT front sonar: ");
        Serial.print(sonarGetFrontCm());
        Serial.println(" cm");
      }

      // Cruises at APPROACH_SPEED, easing down as the front sonar closes in
      bool reachedBelt = driveControlCruiseToSonarStop(APPROACH_SPEED, SONAR_BELT_STOP_CM);

      if (reachedBelt) {
        // TODO: hand off to the belt-measurement/grab state once it exists;
        setState(TaskState::IDLE);
      }
      break;
    }

    case TaskState::APPROACH_DROP_OFF: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        driveControlReset();
        driveControlStraightStart();
      }

      bool reachedDropOff = driveControlCruiseToSonarStop(CRUISE_SPEED, SONAR_DROPZONE_STOP_CM);

      if (reachedDropOff) {
        // TODO: hand off to collector door open; doens't exist yet
        setState(TaskState::IDLE);
      }
      break;
    }

    case TaskState::TURN_LEFT_TO_BELT: {
      static unsigned long lastEnteredAt = 0;
      if (runSingleTurn(-90, lastEnteredAt)) {
        setState(TaskState::IDLE);
      }
      break;
    }

    case TaskState::TURN_180: {
      static unsigned long lastEnteredAt = 0;
      if (runSingleTurn(-180, lastEnteredAt)) {
        setState(TaskState::IDLE);
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
        setState(TaskState::IDLE);
      }
      break;
    }

    case TaskState::GRIPPER_CLOSE: {
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        gripperRequestClose();
      }
      delay(2000);
      if (gripperIsSettled()) {
        setState(TaskState::GRIPPER_OPEN);
      }
      break;
    }



    case TaskState::TURN_TO_DROPZONE: {
      static unsigned long lastEnteredAt = 0;
      if (runSingleTurn(detectedCubeColor == CubeColor::RED ? -90 : 90, lastEnteredAt)) {
        setState(TaskState::LOCATE_DROPZONE);
      }
      break;
    }

    case TaskState::LOCATE_DROPZONE: {
      stopAll(); // stay put while confirming the marker; only Pixy is polled here

      setState(TaskState::APPROACH_DROP_OFF);     
    }

    
    case TaskState::DROPPING_CUBE: {
      stopAll();
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        Serial.println("Cube is dropping");
        gripperRequestOpen();
      }
      if (gripperIsSettled()) {
        setState(TaskState::DRIVE_BACKWARD);
      }
      break;
    }

    case TaskState::TURN_TEST: {
      static unsigned long lastEnteredAt = 0;
      static int turnIndex = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        turnIndex = 0;
        turnControlStart((turnIndex % 2 == 0) ? 90 : -90);
      }

      if (turnControlUpdate(TURN_SPEED)) {
        turnIndex++;
        if (turnIndex < TURN_TEST_REPEATS) {
          turnControlStart((turnIndex % 2 == 0) ? 90 : -90);
        } else {
          setState(TaskState::IDLE);
        }
      }
      break;
    }
    case TaskState::DRIVE_PID_TEST: {
      // Plain driveControlUpdate() on purpose, not driveControlUpdateStraight()
      // -- this is re-baselining DRIVE_PID_TARGET_TICKS_PER_INTERVAL itself
      // (Config.h) after the weight/CG shifted, so it should measure raw
      // ticks-per-interval at a fixed commanded PWM, not muddy that with the
      // heading-hold layer biasing left/right unevenly.
      static unsigned long lastEnteredAt = 0;
      static unsigned long phaseStartAt = 0;
      static int phase = 0; // 0 = forward, 1 = backward, 2 = done

      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        phase = 0;
        phaseStartAt = millis();
        driveControlReset();
        Serial.println("=== DRIVE_PID_TEST: forward phase (2000ms) ===");
      }

      if (phase == 0) {
        driveControlUpdate(CRUISE_SPEED, CRUISE_SPEED);
        if (millis() - phaseStartAt >= 2000) {
          driveControlStop();
          driveControlReset();
          phase = 1;
          phaseStartAt = millis();
          Serial.println("=== DRIVE_PID_TEST: backward phase (2000ms) ===");
        }
      } else if (phase == 1) {
        driveControlUpdate(-CRUISE_SPEED, -CRUISE_SPEED);
        if (millis() - phaseStartAt >= 2000) {
          driveControlStop();
          Serial.println("=== DRIVE_PID_TEST: done -- paste the 'DriveControl ... t: o:' lines above back to Claude ===");
          setState(TaskState::IDLE);
        }
      }
      break;
    }

    case TaskState::ENCODER_TEST: {
      static unsigned long lastPrintAt = 0;
      static unsigned long lastEnteredAt = 0;
      if (stateEnteredAt != lastEnteredAt) {
        lastEnteredAt = stateEnteredAt;
        encoderResetAll();
        lastPrintAt = millis();
      }

      driveForward();

      if (millis() - lastPrintAt >= ENCODER_PRINT_INTERVAL_MS) {
        lastPrintAt = millis();
        Serial.print("Ticks/interval  LF:");
        Serial.print(encoderGetAndResetTicks(WheelId::LEFT_FRONT));
        Serial.print("  LR:");
        Serial.print(encoderGetAndResetTicks(WheelId::LEFT_REAR));
        Serial.print("  RF:");
        Serial.print(encoderGetAndResetTicks(WheelId::RIGHT_FRONT));
        Serial.print("  RR:");
        Serial.println(encoderGetAndResetTicks(WheelId::RIGHT_REAR));
      }

      if (timeInState() >= ENCODER_TEST_MS) {
        stopAll();
        setState(TaskState::GRIPPER_OPEN);
      }
      break;
    }


  }

  // Runs every tick regardless of which TaskState is active, so a gripper
  // open/close started in one state (e.g. DROPPING_CUBE) keeps progressing
  // even after the robot has moved on to driving/turning.
  gripperControlUpdate();

  // Add an emergency stop that works from ANY TaskState:
  if (digitalRead(A0) == HIGH) {   // example: a bumper switch on A0
    stopAll();
    setState(TaskState::IDLE);
  }
}