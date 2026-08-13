#include "Gripper.h"
#include <Servo.h>

static Servo gripperServo;

void gripperInit() {
  gripperServo.attach(GRIPPER_SERVO_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  gripperOpen();
  Serial.println("Gripper Setup Complete");
}

void gripperOpen() {
    gripperServo.write(GRIPPER_OPEN_ANGLE);
}

void gripperClose() {
    gripperServo.write(GRIPPER_CLOSE_ANGLE);
    delay(2000); // temporary time delay (blocking code), 
    // TODO: actually keep the gripper close until reach the drop off point
    
}
    