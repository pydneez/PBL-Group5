#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Sonar.h"
#include "Pixy.h"

// ---------------- DriveState ----------------
enum class DriveState {
  IDLE,              // waiting for a Serial start command
  DETECT_CUBE,       // waiting for Pixy to identify the picked-up cube's color
  TURN_TO_DROPZONE,  // fixed turn: red -> left, green -> right
  LOCATE_DROPZONE,   // waiting for Pixy to confirm the matching drop-off marker
  DRIVE_FORWARD,     // driving straight, open-loop; sonar polling starts after DRIVE_BEFORE_SONAR_MS
  DROPPING_CUBE,     // stopped within SONAR_DROPZONE_STOP_CM; grabber not implemented yet
  DRIVE_BACKWARD     // driving away from drop-off wall
};

// Set by DETECT_CUBE; drives both the turn direction and which drop-off
// marker LOCATE_DROPZONE looks for. Cleared each time IDLE is (re)entered.
CubeColor detectedCubeColor = CubeColor::NONE;

// ---------------- RobotState ----------------
enum class RobotState {
  WAIT_START // perform calibration
  
};

DriveState state = DriveState::IDLE;
unsigned long stateEnteredAt = 0;

// Written out manually so the IDE's auto-generated prototypes 
// (inserted above the enum) don't reference DriveState before it's defined.
const char* stateName(DriveState s);
void setState(DriveState newState);

const char* stateName(DriveState s) {
  switch (s) {
    case DriveState::IDLE:             return "IDLE";
    case DriveState::DETECT_CUBE:      return "DETECT_CUBE";
    case DriveState::TURN_TO_DROPZONE: return "TURN_TO_DROPZONE";
    case DriveState::LOCATE_DROPZONE:  return "LOCATE_DROPZONE";
    case DriveState::DRIVE_FORWARD:    return "DRIVE_FORWARD";
    case DriveState::DROPPING_CUBE:    return "DROPPING_CUBE";
    case DriveState::DRIVE_BACKWARD:   return "DRIVE_BACKWARD";
  }
  return "?";
}

void setState(DriveState newState) {
  state = newState;
  stateEnteredAt = millis();
  Serial.print("-> ");
  Serial.println(stateName(state));
  if (state == DriveState::IDLE) {
    Serial.println("Type any character + Enter to start...");
  }
}

unsigned long timeInState() {
  return millis() - stateEnteredAt;
}

// ---------------- SETUP ----------------
void setup() {
    Serial.begin(115200);
    motorsInit();
    sonarInit();
    pixyDetectInit();
    stopAll();

    setState(DriveState::IDLE);
}

// ----------------  MAIN LOOP ----------------
void loop() {
  switch (state) {
    case DriveState::IDLE:
      stopAll();
      // TODO: replace with IMU calibration / value reset once the IMU is wired in.

      if (Serial.available() > 0) {
        while (Serial.available() > 0) Serial.read(); // drain the buffer
        detectedCubeColor = CubeColor::NONE;
        setState(DriveState::DETECT_CUBE);
      }
      break;

    case DriveState::DETECT_CUBE: {
      stopAll(); // stay put while identifying the cube

      PixyDetection d = pixyDetect(PixyDetectMode::SEEK_CUBE);
      if (d.found) {
        detectedCubeColor = d.color;
        Serial.print("Cube color detected: ");
        Serial.println(d.color == CubeColor::RED ? "RED -> turning left" : "GREEN -> turning right");
        setState(DriveState::TURN_TO_DROPZONE);
      }
      break;
    }

    case DriveState::TURN_TO_DROPZONE: {
      if (detectedCubeColor == CubeColor::RED) {
        driveSides(-TURN_SPEED, TURN_SPEED); // red -> turn left
      } else if (detectedCubeColor == CubeColor::GREEN) {
        driveSides(TURN_SPEED, -TURN_SPEED); // green -> turn right
      }

      if (timeInState() >= TURN_MS) {
        stopAll();
        setState(DriveState::LOCATE_DROPZONE);
      }
      break;
    }

    case DriveState::LOCATE_DROPZONE: {
      stopAll(); // stay put while confirming the marker; only Pixy is polled here

      PixyDetectMode targetMode = (detectedCubeColor == CubeColor::RED)
          ? PixyDetectMode::SEEK_RED_DROP
          : PixyDetectMode::SEEK_GREEN_DROP;

      PixyDetection d = pixyDetect(targetMode);
      if (d.found) {
        Serial.println("Drop-off marker confirmed -> driving forward.");
        setState(DriveState::DRIVE_FORWARD);
      }
      break;
    }

    case DriveState::DRIVE_FORWARD: {
      driveSides(CRUISE_SPEED, CRUISE_SPEED);  

      // Pixy and Sonar is not polled
      // Sonar itself is only polled once DRIVE_BEFORE_SONAR_MS has elapsed.
      if (timeInState() >= DRIVE_BEFORE_SONAR_MS) {
        float frontCm = sonarGetFrontCm();
        Serial.print("Front distance: ");
        Serial.println(frontCm);

        // frontCm == -1 means no echo (out of range) -- must not be treated as "close"
        if (frontCm > 0 && frontCm <= SONAR_DROPZONE_STOP_CM) {
          stopAll();
          setState(DriveState::DROPPING_CUBE);
        }
      }
      break;
    }

    case DriveState::DROPPING_CUBE:
      stopAll();
      Serial.println("Cube is dropping");
      // TODO: trigger grabber release here once the grabber is implemented,
      // and wait for it to actually finish before backing up instead of
      // transitioning immediately.
      setState(DriveState::DRIVE_BACKWARD);
      break;

    case DriveState::DRIVE_BACKWARD:
      driveSides(-CRUISE_SPEED, -CRUISE_SPEED);
      if (timeInState() >= BACK_MS) {
        stopAll();
        setState(DriveState::IDLE);
      }
      break;
  }

  // Add an emergency stop that works from ANY DriveState:
  if (digitalRead(A0) == HIGH) {   // example: a bumper switch on A0
    stopAll();
    setState(DriveState::IDLE);
  }
}