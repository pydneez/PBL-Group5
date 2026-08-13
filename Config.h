#pragma once

// ---------------- MOTOR PINS ----------------
#define PIN_L_IN1  29   // left front direction
#define PIN_L_IN2  28
#define PIN_L_IN3  27   // left rear direction
#define PIN_L_IN4  26
#define PIN_L_FRONT  8    // left front speed (PWM)
#define PIN_L_REAR  9     // left rear speed (PWM)

#define PIN_R_IN1  32   // right front direction
#define PIN_R_IN2  33
#define PIN_R_IN3  30   // right rear direction
#define PIN_R_IN4  31
#define PIN_R_FRONT  6    // right front speed (PWM)
#define PIN_R_REAR  7    // right rear speed (PWM)

// ---------------- TIMING / SPEED ----------------
#define DRIVE_MS    2000   // how long to drive forward
#define TURN_MS     1000   // how long to turn
#define BACK_MS     2000   // how long to reverse
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
#define TRIG_RIGHT 45   
#define ECHO_RIGHT 44  

// Front distance (cm) at which the robot is close enough to the drop-off wall
#define SONAR_DROPZONE_STOP_CM  15

// Measured side clearance (cm) when the robot is centered in the drop-off wall
#define SONAR_SIDE_CENTER_CM    25

// Placeholder: how long to drive straight (open-loop, no PID yet) before the
// front sonar starts being polled. Retune once PID/speed is characterized.
#define DRIVE_BEFORE_SONAR_MS  2000

// ---------------- ENCODERS (LM393 IR slot sensor, single-channel) ----------------
// must be attachInterrupt()-capable on your board (e.g. on a Mega2560: 2, 3, 18, 19, 20, 21)
#define ENCODER_LF_PIN  2   // left front
#define ENCODER_LR_PIN  3   // left rear
#define ENCODER_RF_PIN  18   // right front
#define ENCODER_RR_PIN  19   // right rear

// How long ENCODER_TEST drives forward for, and how often it prints
// ticks-per-interval for each wheel while doing so.
#define ENCODER_TEST_MS           3000
#define ENCODER_PRINT_INTERVAL_MS 200

// ---------------- DRIVE PID (straight-line only -- turns will use IMU heading instead) ----------------
// How often the per-wheel correction recomputes; between updates each wheel
// holds its last commanded PWM.
#define DRIVE_PID_INTERVAL_MS 200

// Target ticks-per-interval at CRUISE_SPEED (150 PWM) -- placeholder derived
// from an ENCODER_TEST run (~16-28 ticks/200ms across the 4 wheels, ~21 avg).
// Retune from a longer/steadier run once the chassis is fully assembled.
#define DRIVE_PID_TARGET_TICKS_PER_INTERVAL 21

// Gains are NOT tuned -- same placeholder status as PID.h's comment describes.
#define DRIVE_PID_KP 1.0f
#define DRIVE_PID_KI 0.0f
#define DRIVE_PID_KD 0.0f

// Correction is clamped to +/- this many PWM units so one noisy reading
// can't slam a wheel's speed to 0 or 255.
#define DRIVE_PID_MAX_CORRECTION 50



// ---------------- IMU (Adafruit BNO055, I2C) ----------------
// Hardware I2C on a Mega2560 is fixed to these pins -- not configurable, and
// not passed to pinMode() anywhere; listed here for reference only.
#define IMU_SDA_PIN 20
#define IMU_SCL_PIN 21

#define BNO055_I2C_ADDR   0x29   // BNO055_ADDRESS_A; use 0x29 (BNO055_ADDRESS_B) if the ADR pin is pulled high
#define BNO055_SENSOR_ID  55     // arbitrary unique ID for the Adafruit_Sensor API


// ---------------- SERVO (MG90) ----------------
#define GRIPPER_SERVO_PIN 34
#define GRIPPER_OPEN_ANGLE   60
#define GRIPPER_CLOSE_ANGLE  0

#define PIN_NOT_WIRED -1
// Placeholder: how long to hold each gripper state so the servo has time to
// physically reach the commanded angle before the state machine moves on.
#define GRIPPER_MS  500


// ---------------- SERVO (MG996R) ----------------
#define TORQUE_SERVO_PIN -1
#define TORQUE_SERVO_UP_ANGLE -1    
#define TORQUE_SERVO_DONW_ANGLE -1