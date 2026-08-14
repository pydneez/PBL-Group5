#include "Gripper.h"
#include <Servo.h>

static Servo gripperServo;
static GripperState state = GripperState::OPENING;
static unsigned long stateEnteredAt = 0;

void gripperInit() {
  gripperServo.attach(GRIPPER_SERVO_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  state = GripperState::OPENING;
  stateEnteredAt = millis();
  Serial.println("Gripper Setup Complete");
}

void gripperRequestOpen() {
  if (state == GripperState::OPEN || state == GripperState::OPENING) return;
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  state = GripperState::OPENING;
  stateEnteredAt = millis();
}

void gripperRequestClose() {
  if (state == GripperState::CLOSED || state == GripperState::CLOSING) return;
  gripperServo.write(GRIPPER_CLOSE_ANGLE);
  state = GripperState::CLOSING;
  stateEnteredAt = millis();
}

void gripperControlUpdate() {
  if (millis() - stateEnteredAt < GRIPPER_MS) return;

  if (state == GripperState::OPENING) {
    state = GripperState::OPEN;
  } else if (state == GripperState::CLOSING) {
    state = GripperState::CLOSED;
  }
}

GripperState gripperGetState() {
  return state;
}

bool gripperIsSettled() {
  return state == GripperState::OPEN || state == GripperState::CLOSED;
}
