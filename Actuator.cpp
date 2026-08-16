#include "Actuator.h"
#include <Servo.h>

static Servo gripperServo;
static Servo liftServo;
static Servo gateServo;
static GripperState grip_state = GripperState::OPENING;
static LiftState lift_state = LiftState::LIFTING_UP;
static GateState gate_state = GateState::CLOSED;
static unsigned long gripStateEnteredAt = 0;
static unsigned long liftStateEnteredAt = 0;
static unsigned long gateStateEnteredAt = 0;

// True once GATE_SERVO_PIN is real hardware (not PIN_NOT_WIRED) -- lets
// every gateXxx() function below no-op safely until the box/gate servo is
// physically mounted, same pattern Encoder.cpp's attachEncoderIfWired()
// uses for unwired encoder pins.
static bool gateWired = false;

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

void gateInit() {
  gateWired = (GATE_SERVO_PIN != PIN_NOT_WIRED);
  if (!gateWired) {
    Serial.println("Gate: GATE_SERVO_PIN not wired yet -- gate control disabled until Config.h has a real pin.");
    gate_state = GateState::CLOSED; // already-settled so nothing downstream stalls waiting on it
    return;
  }
  gateServo.attach(GATE_SERVO_PIN);
  gateServo.write(GATE_CLOSE_ANGLE); // closed at boot -- nothing should fall out before the drop zone
  gate_state = GateState::CLOSING;
  gateStateEnteredAt = millis();
  Serial.println("Gate Setup Complete");
}

void gateRequestOpen() {
  if (!gateWired) return;
  if (gate_state == GateState::OPEN || gate_state == GateState::OPENING) return;
  gateServo.write(GATE_OPEN_ANGLE);
  gate_state = GateState::OPENING;
  gateStateEnteredAt = millis();
}

void gateRequestClose() {
  if (!gateWired) return;
  if (gate_state == GateState::CLOSED || gate_state == GateState::CLOSING) return;
  gateServo.write(GATE_CLOSE_ANGLE);
  gate_state = GateState::CLOSING;
  gateStateEnteredAt = millis();
}

void gateControlUpdate() {
  if (!gateWired) return;
  if (millis() - gateStateEnteredAt < GATE_MS) return;

  if (gate_state == GateState::OPENING) {
    gate_state = GateState::OPEN;
  } else if (gate_state == GateState::CLOSING) {
    gate_state = GateState::CLOSED;
  }
}

GripperState gripperGetState() {
  return grip_state;
}

LiftState lifterGetState() {
  return lift_state;
}

GateState gateGetState() {
  return gate_state;
}

bool gripperIsSettled() {
  return grip_state == GripperState::OPEN || grip_state == GripperState::CLOSED;
}

bool lifterIsSettled() {
  return lift_state == LiftState::LIFTED_UP || lift_state == LiftState::LIFTED_DOWN;
}

bool gateIsSettled() {
  return gate_state == GateState::OPEN || gate_state == GateState::CLOSED;
}
