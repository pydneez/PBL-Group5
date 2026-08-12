#pragma once

// ---------------- MOTOR PINS ----------------
#define PIN_R_IN1  37   // right front direction
#define PIN_R_IN2  36
#define PIN_R_IN3  35   // right rear direction
#define PIN_R_IN4  34
#define PIN_R_FRONT  2    // right front speed (PWM)
#define PIN_R_REAR  3    // right rear speed (PWM)

#define PIN_L_IN1  33   // left front direction
#define PIN_L_IN2  32
#define PIN_L_IN3  31   // left rear direction
#define PIN_L_IN4  30
#define PIN_L_FRONT  4    // left front speed (PWM)
#define PIN_L_REAR  5    // left rear speed (PWM)

// ---------------- TIMING / SPEED ----------------
#define DRIVE_MS    5000   // how long to drive forward
#define TURN_MS     3000   // how long to turn
#define BACK_MS     5000   // how long to reverse
#define CRUISE_SPEED 150
#define TURN_SPEED   170


// ---------------- PIXY2 CAMERA ----------------
// Native Pixy2 CCC resolution (matches the block x:0-315, y:0-207 range).
#define PIXY_FRAME_WIDTH     316
#define PIXY_FRAME_HEIGHT    208
#define PIXY_FRAME_CENTER_X  (PIXY_FRAME_WIDTH / 2)

// Trained signature IDs (taught in PixyMon)
#define PIXY_SIG_RED         1   // red cube
#define PIXY_SIG_GREEN       2   // green cube
#define PIXY_SIG_DROP_RED    3   // red drop-off zone marker
#define PIXY_SIG_DROP_GREEN  4   // green drop-off zone marker

// Reject blocks smaller than this (px) on either axis: filters camera
// noise / specular glints (e.g. off the untrained white cube) that could
// otherwise stray into a trained signature's color range. Tune by testing
// with PixyMon once the real cubes/lighting are in place.
#define PIXY_MIN_BLOCK_SIZE  8


// ---------------- SONAR / ULTRASONIC ----------------
#define TRIG_FRONT 47
#define ECHO_FRONT 46
#define TRIG_LEFT 49
#define ECHO_LEFT 48
#define TRIG_RIGHT 51
#define ECHO_RIGHT 52

// Front distance (cm) at which the robot is close enough to the drop-off wall
#define SONAR_DROPZONE_STOP_CM  15

// Measured side clearance (cm) when the robot is centered in the drop-off wall
#define SONAR_SIDE_CENTER_CM    25

// Placeholder: how long to drive straight (open-loop, no PID yet) before the
// front sonar starts being polled. Retune once PID/speed is characterized.
#define DRIVE_BEFORE_SONAR_MS  2000


