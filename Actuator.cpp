#include "Actuator.h"
#include <Servo.h>

static Servo gripperServo;
static Servo liftServo;
static GripperState grip_state = GripperState::OPENING;
static LiftState lift_state = LiftState::LIFTING_UP;
static unsigned long gripState  EnteredAt = 0;
static unsigned long liftStateEnteredAt = 0;

void gripperInit() {
  gripperServo.attach(GRIPPER_SERVO_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);

  grip_state = GripperState::OPENING;
  gripStateEnteredAt = millis();
  Serial.println("Gripper Setup Complete");
}

void liftInit() {
  liftServo.attach(LIFT_SERVO_PIN);
  liftServo.write(LIFT_UP_ANGLE);

  lift_state = LiftState::LIFTING_UP;
  liftStateEnteredAt = millis();
  Serial.println("Lifter Setup Complete");
}

void gripperRequestOpen() {
  if (grip_state == GripperState::OPEN || grip_state == GripperState::OPENING) return;
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  grip_state = GripperState::OPENING;
  gripStateEnteredAt = millis();
}

void gripperRequestClose() {
  if (grip_state == GripperState::CLOSED || grip_state == GripperState::CLOSING) return;
  gripperServo.write(GRIPPER_CLOSE_ANGLE);
  grip_state = GripperState::CLOSING;
  gripStateEnteredAt = millis();
}

void liftRequestUp() {
  if (lift_state == LiftState::LIFTED_UP || lift_state == LiftState::LIFTING_UP) return;
  liftServo.write(LIFT_UP_ANGLE);
  lift_state = LiftState::LIFTING_UP;
  liftStateEnteredAt = millis();
}

void liftRequestDown() {
  if (lift_state == LiftState::LIFTED_DOWN || lift_state == LiftState::LIFTING_DOWN) return;
  liftServo.write(LIFT_DOWN_ANGLE);
  lift_state = LiftState::LIFTING_DOWN;
  liftStateEnteredAt = millis();
}

void gripperControlUpdate() {
  if (millis() - gripStateEnteredAt < GRIPPER_MS) return;

  if (grip_state == GripperState::OPENING) {
    grip_state = GripperState::OPEN;
  } else if (grip_state == GripperState::CLOSING) {
    grip_state = GripperState::CLOSED;
  }
}

void liftControlUpdate() {
  if (millis() - liftStateEnteredAt < LIFTER_MS) return;

  if (lift_state == LiftState::LIFTING_UP) {
    lift_state = LiftState::LIFTED_UP;
  } else if (lift_state == LiftState::LIFTING_DOWN) {
    lift_state = LiftState::LIFTED_DOWN;
  }
}

GripperState gripperGetState() {
  return grip_state;
}

LiftState lifterGetState() {
  return lift_state;
}

bool gripperIsSettled() {
  return grip_state == GripperState::OPEN || grip_state == GripperState::CLOSED;
}

bool lifterIsSettled() {
  return lift_state == LiftState::LIFTED_UP || lift_state == LiftState::LIFTED_DOWN;
}
