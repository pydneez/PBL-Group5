#pragma once

// ---------------- MOTOR PINS ----------------
#define PIN_L_IN1  25   // left front direction 
#define PIN_L_IN2  24
#define PIN_L_IN3  23   // left rear direction 
#define PIN_L_IN4  22
#define PIN_L_FRONT  12    // left front speed (PWM)
#define PIN_L_REAR  13     // left rear speed (PWM)

#define PIN_R_IN1  28   // right front direction -- swapped
#define PIN_R_IN2  29
#define PIN_R_IN3  26   // right rear direction 
#define PIN_R_IN4  27
#define PIN_R_FRONT  8    // right front speed (PWM)
#define PIN_R_REAR  9    // right rear speed (PWM)

// ---------------- TIMING / SPEED ----------------
#define DRIVE_MS    1000
#define TURN_MS     800   
#define BACK_MS    70   
#define CRUISE_SPEED 220

#define BELT_APPROACH_MS 370
#define BELT_APPROACH_SPEED 190

// Ceiling speed for APPROACH_DROPZONE --
#define DROPZONE_APPROACH_SPEED -120 // backward

#define TURN_SPEED   160

// ---------------- SONAR / ULTRASONIC ----------------
#define TRIG_BACK 30
#define ECHO_BACK 31

#define SONAR_DROPZONE_STOP_CM  15
#define SONAR_DROPZONE_SLOW_CM 50
#define SONAR_DROPZONE_DEADBAND 10

// How long the distance must stay continuously inside the +/-
#define SONAR_HOLD_MS 300

// Floor PWM for the wall/belt approach 
#define SONAR_MIN_SPEED    60

// ---------------- PIXY2 CAMERA ----------------
#define PIXY_DEBUG_PRINT_BLOCKS 0

// x:0-315, y:0-207 range
#define PIXY_FRAME_WIDTH     316
#define PIXY_FRAME_HEIGHT    208
#define PIXY_FRAME_CENTER_X  (PIXY_FRAME_WIDTH / 2)
#define PIXY_FRAME_CENTER_Y  (PIXY_FRAME_HEIGHT / 2)


// Trained signature IDs (taught in PixyMon)
#define PIXY_SIG_RED         1   // red cube
#define PIXY_SIG_GREEN       4   // green cube
#define PIXY_SIG_DROP_RED    7   // purple marker 
#define PIXY_SIG_DROP_GREEN  2   // yellow marker 

// Reject blocks smaller than this (px) on either axis
#define PIXY_MIN_BLOCK_SIZE  8

#define PICKUP_ZONE_TARGET_X     127
#define PICKUP_ZONE_TARGET_Y     127
#define PICKUP_ZONE_ALLOWANCE_X  6
#define PICKUP_ZONE_ALLOWANCE_Y  45
#define PICKUP_ZONE_TARGET_AREA 7500
#define PICKUP_ZONE_ALLOWANCE_AREA 7000

#define DETECT_CUBE_CONFIRM_FRAMES 1

#define DETECT_CUBE_TIMEOUT_MS 6500
#define GAME_TIMEOUT_MS 160000

#define MAX_CARRIED_CUBES 4


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

// ---------------- DRIVE PID ----------------
// How often the per-wheel correction recomputes
#define DRIVE_PID_INTERVAL_MS 100
// (straight-line only -- turns will use IMU heading instead) 

// Target ticks-per-interval at CRUISE_SPEED (200 PWM) -- EXTRAPOLATED, not
// directly measured. Every DRIVE_PID_TEST run so far ended up saturated
// near ~250 actual PWM (200 requested against an unreachable target, then
// 250 requested outright), so no run has actually sampled tick rate at a
// genuinely unsaturated 200 PWM yet. This value is that ~250-PWM data
// (overall avg ~12.4 ticks/interval; left side ~14.4, right side ~10.3 --
// see CRUISE_SPEED comment) scaled down linearly by 200/250. Treat it as a
// starting point: rerun DRIVE_PID_TEST now that there's real headroom, check
// that "o:" settles near 200 without pinning at a rail, and retune this
// constant from that real reading.
#define DRIVE_PID_TARGET_TICKS_PER_INTERVAL 10

// Gains are NOT tuned -- same placeholder status as PID.h's comment describes.
#define DRIVE_PID_KP 2.0f
#define DRIVE_PID_KI 0.0f
#define DRIVE_PID_KD 0.0f

// Correction is clamped to +/- this many PWM units so one noisy reading
// can't slam a wheel's speed to 0 or 255.
#define DRIVE_PID_MAX_CORRECTION 50

// Below this commanded |PWM|, the tick-rate PID is skipped and the wheel
// drives open-loop at baseSpeed instead. DRIVE_PID_TARGET_TICKS_PER_INTERVAL
// was measured at CRUISE_SPEED, not at the BELT_APPROACH_SPEED/SONAR_FRONT_MIN_SPEED
// range, so the PID saw permanent error there and pinned its correction at
// +DRIVE_PID_MAX_CORRECTION for the whole approach -- e.g. commanding 10
// actually drove ~60, which is why APPROACH_BELT kept hitting the wall
// despite a "slow" configured speed. Set above APPROACH_SPEED so the entire
// belt/dropzone approach (including the SONAR_FRONT_MIN_SPEED floor and the
// gentle backup) stays open-loop end to end -- retune
// DRIVE_PID_TARGET_TICKS_PER_INTERVAL for that speed band instead of lowering
// this if per-wheel correction turns out to be needed down there too.
#define DRIVE_PID_MIN_SPEED_FOR_CORRECTION 100

// Heading-hold gain for driveControlUpdateStraight() -- differential PWM
// bias per degree of error from the cardinal heading latched at the start
// of the straight leg (see driveControlStraightStart()). Deliberately
// small: too high fights the per-wheel encoder PID above and turns a gentle
// self-correction into a zig-zag. Start low and raise only if the robot
// isn't visibly correcting back toward the cardinal heading.
#define DRIVE_HEADING_KP 3.0f

// Heading-hold correction is clamped to +/- this many PWM units, same
// reasoning as DRIVE_PID_MAX_CORRECTION -- one noisy heading reading can't
// suddenly swing a wheel's target speed.
#define DRIVE_HEADING_MAX_CORRECTION 40

// How long driveControlStop() takes to ramp PWM down to 0 (in this many
// steps), instead of cutting power in one step -- smooths the jolt/bounce a
// full-speed-to-zero stop causes on the chassis.
#define DRIVE_DECEL_MS 350
#define DRIVE_DECEL_STEPS 8

// After the ramp above reaches 0, driveControlStop() 
// holds here for this long before returning 
#define DRIVE_SETTLE_MS 200



// ---------------- IMU (Adafruit BNO055, I2C) ----------------
#define IMU_SDA_PIN 20
#define IMU_SCL_PIN 21

#define BNO055_I2C_ADDR   0x28   
#define BNO055_SENSOR_ID  55     

#define IMU_EEPROM_CALIB_ADDR 0

// Safety net: if full calibration (sys/gyro/accel/mag all == 3) isn't
// reached within this long, IMU_CALIBRATE gives up waiting and proceeds anyway
#define IMU_CALIBRATE_TIMEOUT_MS 10000

// ---------------- TURN CONTROL (closed-loop against IMU heading) ----------------
// A turn counts as "done" once within this many degrees of the target heading.
#define TURN_HEADING_TOLERANCE_DEG 1

// Hovering within TURN_HEADING_TOLERANCE_DEG*2.5 for this long also counts as
// done, so small oscillation around the target doesn't stall a turn forever.
#define TURN_NEAR_MS 200

// Safety net: a turn is accepted as done after this long regardless of error,
// so a stuck/miscalibrated turn can't hang the state machine.
//
// Was 1000, but at TURN_SPEED=100 a 90deg turn genuinely needs most of that
// window just to travel the distance -- measured ending in a stall 5.4deg
// short of tolerance (see TURN_PWM_FLOOR), with the timeout firing before it
// could finish converging rather than because it was actually stuck. Raised
// to give real convergence a chance to complete before the safety net cuts
// it off.
#define TURN_TIMEOUT_MS 3500

#define TURN_PWM_FLOOR 130

#define TURN_RAMP_SPAN_DEG 50

// Per-wheel PWM multiplier applied ONLY during turning (driveSides()/
// straight driving is unaffected). Compensates for a wheel contributing
// less to rotation than the others -- e.g. RR suspected of poor ground
// contact, which loads extra weight onto FR and makes it grip too well to
// slip, so the chassis pivots on that corner instead of the center. This is
// a software band-aid for a hardware traction difference, not a fix for
// it -- if the mechanical cause (check RR's ground contact) gets fixed
// later, these should go back to 1.0.
#define TURN_SCALE_LF 1.0f
#define TURN_SCALE_LR 1.0f
#define TURN_SCALE_RF 0.8f
#define TURN_SCALE_RR 1.1f

// RF-specific override used only when turning LEFT (RF driving forward).
// Turn-tick logs show RF free-spinning well ahead of the other three wheels
// when driven forward (e.g. 24 ticks vs LF15/LR11/RR13 on one logged left
// turn), while right turns (where RF drives backward and is barely moving at
// all) land within ~2-3deg. TURN_SCALE_RF above stays as-is for right turns.
// Start at 0.7 and retune from the "turned=" log line -- lower if still
// overshooting, raise if it now undershoots or stalls RF at turn start.
//
// NOTE: retested at 0.7 -- RF is no longer the tick-count outlier, but
// left-turn overshoot barely moved (still ~11deg, vs ~2-3deg on right).
// The per-wheel outlier also isn't stable run-to-run (LF was the high
// wheel on that retest, not RF) -- so this alone isn't the fix, see
// TURN_LEFT_SPEED_SCALE below for the actual lever that's now targeting
// the overshoot directly.
#define TURN_SCALE_RF_LEFT 0.7f

// Left turns consistently land ~10-12deg past target regardless of which
// wheel's tick count happens to be highest that run (see TURN_SCALE_RF_LEFT
// note above) -- right turns consistently land within ~2-3deg at the same
// TURN_SPEED/TURN_PWM_FLOOR/TURN_RAMP_SPAN_DEG. That points at a direction-
// level asymmetry (e.g. uneven weight distribution giving the left-
// backward/right-forward combination more net rotational torque than the
// reverse combination) rather than one specific wheel's fault -- so instead
// of continuing to chase individual per-wheel scales, this scales the
// *overall* commanded turnPwm down for left turns only, applied AFTER the
// TURN_PWM_FLOOR clamp in turnControlUpdate() so the effective floor (and
// the TURN_KICK_PWM burst) both actually come down for left turns instead
// of being re-clamped back up. Right turns (err>0 branch) are untouched.
// Start at 0.85 and retune from "turned=" -- lower if still overshooting,
// raise if it undershoots or the turn starts eating into TURN_TIMEOUT_MS.
#define TURN_LEFT_SPEED_SCALE 0.85f


// Must stay above TURN_SPEED/TURN_PWM_FLOOR or it isn't a "burst" at all --
// was dropped to 70ms/80 PWM (below the 150 PWM cruise) alongside the
// TURN_SPEED bump to 150, which silently defeated it: logged turns now
// start with a full 150ms window of 0 ticks on all four wheels (stalled),
// eating into TURN_TIMEOUT_MS and undershooting. Restored proportional to
// the old 160ms/130 PWM kick (which worked at TURN_SPEED=100) scaled up for
// the new cruise speed -- retest and watch the first "Turn ticks" sample
// for any wheel still at/near 0.
#define TURN_KICK_MS 150
#define TURN_KICK_PWM 200

#define TURN_SETTLE_MS 300

// How often turnControlUpdate() samples per-wheel encoder ticks during a turn
#define TURN_ENCODER_SAMPLE_MS 150


// ---------------- GRIPPER SERVO (MG90) ----------------
#define GRIPPER_SERVO_PIN 34
#define GRIPPER_OPEN_ANGLE   180 // original position dont change!!!!!
#define GRIPPER_CLOSE_ANGLE  97
//#define GRIPPER_MS  1185
#define GRIPPER_MS  770


#define PIN_NOT_WIRED -1

// ---------------- BOX GATE SERVO (releases carried cube(s) at the drop zone) ----------------
#define GATE_SERVO_PIN 36
#define GATE_OPEN_ANGLE   45 
#define GATE_CLOSE_ANGLE  0 // original position

#define GATE_OPEN_HOLD_MS 1200
#define GATE_CLOSE_SETTLE_MS 300

// ---------------- TORQUE SERVO (MG996R) ----------------
#define LIFT_SERVO_PIN 35
// real value
#define LIFT_STOP 90
#define LIFT_UP_NUM 180
#define LIFT_DOWN_NUM 0


#define LIFTER_MS 980