#include "Motors.h"

// Guarantees IN1 and IN2 are never both HIGH.
void setWheel(int pwmPin, int in1Pin, int in2Pin, int speed) {
    if (speed > 0) {
        digitalWrite(in2Pin, LOW);
        digitalWrite(in1Pin, HIGH);
    } else if (speed < 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
    }
    analogWrite(pwmPin, constrain(abs(speed), -255, 255));
}

void motorsInit() {
    pinMode(PIN_L_FRONT, OUTPUT); pinMode(PIN_L_IN1, OUTPUT); pinMode(PIN_L_IN2, OUTPUT);
    pinMode(PIN_L_REAR,  OUTPUT); pinMode(PIN_L_IN3, OUTPUT); pinMode(PIN_L_IN4, OUTPUT);
    pinMode(PIN_R_FRONT, OUTPUT); pinMode(PIN_R_IN1, OUTPUT); pinMode(PIN_R_IN2, OUTPUT);
    pinMode(PIN_R_REAR,  OUTPUT); pinMode(PIN_R_IN3, OUTPUT); pinMode(PIN_R_IN4, OUTPUT);
    stopAll();
}

// TODO: replace with real encoder-based direction tracking once encoders are wired in.
void encodersSetDirection(bool leftForward, bool rightForward) {
    (void)leftForward;
    (void)rightForward;
}

void driveSides(int leftSpeed, int rightSpeed) {
    encodersSetDirection(leftSpeed >= 0, rightSpeed >= 0);

    // LEFT
    setWheel(PIN_L_FRONT, PIN_L_IN1, PIN_L_IN2, leftSpeed);  // FRONT
    setWheel(PIN_L_REAR,  PIN_L_IN3, PIN_L_IN4, leftSpeed);  // REAR

    // RIGHT
    setWheel(PIN_R_FRONT, PIN_R_IN1, PIN_R_IN2, rightSpeed); // FRONT
    setWheel(PIN_R_REAR,  PIN_R_IN3, PIN_R_IN4, rightSpeed); // REAR
}

void stopAll() {
    driveSides(0, 0); // Set speed of all wheel to 0
}

void driveForward() {
    driveSides(CRUISE_SPEED, CRUISE_SPEED);
}

void driveBackward() {
    driveSides(-CRUISE_SPEED, -CRUISE_SPEED);
}

void turnLeft() {
    driveSides(-TURN_SPEED, TURN_SPEED);
}

void turnRight() {
    driveSides(TURN_SPEED, -TURN_SPEED);
}