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
// Safety-net max drive time for DRIVE_FORWARD_TO_CENTER -- the sonar (see
// SONAR_DROPZONE_STOP_CM / DRIVE_BEFORE_SONAR_MS below) is what actually
// stops the robot; this just keeps a stuck/failed sonar reading from driving
// forever. Must stay well above DRIVE_BEFORE_SONAR_MS or the timeout fires
// before sonar polling even starts. Was 900ms when this was purely a
// fixed-time drive -- retune down once the real distance-to-wall is known.
#define DRIVE_MS    1000
#define TURN_MS     800   
#define BACK_MS     100   // how long to reverse from the wall
#define CRUISE_SPEED 200

// Ceiling speed for the belt approach (APPROACH_BELT) 
#define BELT_APPROACH_SPEED 65

// Ceiling speed for the wall/dropzone approach (APPROACH_DROPZONE) --
#define DROPZONE_APPROACH_SPEED -80 // backward

#define TURN_SPEED   150

// ---------------- SONAR / ULTRASONIC ----------------
#define TRIG_FRONT 38
#define ECHO_FRONT 39
#define TRIG_BACK 30
#define ECHO_BACK 31

// Front distance (cm) at which the robot is close enough to the drop-off wall
#define SONAR_DROPZONE_STOP_CM  12 
#define SONAR_DROPZONE_SLOW_CM 30
#define SONAR_BELT_STOP_CM  4
#define SONAR_BELT_SLOW_CM 20

#define SONAR_BELT_DEADBAND 1
#define SONAR_WALL_DEADBAND 5

// How long the distance must stay continuously inside the +/-
// SONAR_BELT_DEADBAND window before commits to "reached". 
#define SONAR_HOLD_MS 500

// Floor PWM for the wall/belt approach 
// (easing down as distance closes in, backup 
// if it overshoots past STOP_CM - SONAR_BELT_DEADBAND).
#define SONAR_FRONT_MIN_SPEED    60

// ---------------- PIXY2 CAMERA ----------------
// Set to 1 to print per-block detection details (signature/position/size/
// distance-to-pickup-zone) to Serial each frame; 0 for silent normal operation.
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

// Reject blocks smaller than this (px) on either axis: filters camera
// noise / specular glints (e.g. off the untrained white cube)
#define PIXY_MIN_BLOCK_SIZE  8

// Pickup-trigger window: camera is mounted at bearing 225, belt carries
// cubes toward bearing 270, so a cube drifts through frame until it lines
// up here -- that's the position DETECT_CUBE (PBL.ino) requires before
// LIFT_DOWN fires. Target/allowance (not raw min/max) so the window is easy
// to re-center once belt speed is known: currently x:60-70, y:114-116.
#define PICKUP_ZONE_TARGET_X     127
#define PICKUP_ZONE_TARGET_Y     127
#define PICKUP_ZONE_ALLOWANCE_X  5
#define PICKUP_ZONE_ALLOWANCE_Y  10

// DETECT_CUBE (PBL.ino) requires this many consecutive frames reporting the
// same pixyDetect(SEEK_CUBE) match before committing to LIFT_DOWN -- guards
// against a single noisy/spurious frame (specular glint, momentary misread)
// triggering a real mechanical grab. Not yet measured against a real false-
// positive rate; lower if this feels sluggish to react, raise if it commits
// on noise.
#define DETECT_CUBE_CONFIRM_FRAMES 1

#define DETECT_CUBE_TIMEOUT_MS 10000
#define GAME_TIMEOUT_MS 140000

// Carrier capacity: DETECT_CUBE loops back for another cube after each
// successful pickup until this many are carried, then heads to the drop zone
#define MAX_CARRIED_CUBES 3

// LOCATE_DROPZONE requires this many consecutive frames reporting the
// matching drop marker before trusting it -- same debounce reasoning as
// DETECT_CUBE_CONFIRM_FRAMES.
#define LOCATE_DROPZONE_CONFIRM_FRAMES 3

// Safety net: how long LOCATE_DROPZONE scans for 
// matching marker before giving up on THIS attempt. 
// First timeout triggers a 180 recheck turn
// 2nd timeout: proceeds to APPROACH_DROPZONE on the original heading 
#define LOCATE_DROPZONE_TIMEOUT_MS 1000


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

// After the ramp above reaches 0, driveControlStop() holds here for this
// long before returning -- lets residual chassis momentum/coasting actually
// die out, so the next motion (e.g. reversing direction) starts from a
// genuinely stationary robot instead of one still drifting from the last move.
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
#define TURN_TIMEOUT_MS 1400

#define TURN_PWM_FLOOR 90

// Degrees of remaining error at which PWM starts ramping down toward
// TURN_PWM_FLOOR. Deliberately NOT derived from TURN_HEADING_TOLERANCE_DEG
// (they used to be tied together at tolerance*4=8deg, which meant the robot
// was still near full speed until very close to the tight tolerance window
// -- measured as a consistent ~5.5deg coast-through-stop overshoot). Wider
// span here means it's already slow well before the tolerance check, so
// physical momentum has less speed left to carry it past target.
//
// Was widened to 45 then narrowed back to 20 -- at 20, PWM stays pinned at
// TURN_PWM_FLOOR=70 for only the last ~14deg (100*dist/20 < 70 below that),
// leaving the ~70+ preceding degrees at full 100 PWM cruise with no
// deceleration at all. Not enough runway to shed that momentum, so it
// coasted through the tolerance window and had to reverse/correct back
// (measured: overshoot on the first swing, then a readjust pass). 30 splits
// the difference between that and the 45 which risked stalling at the old
// floor=90 -- watch tick counts on retest for the stall signature (any
// wheel dropping to 0) if this still isn't enough runway.
#define TURN_RAMP_SPAN_DEG 50

// Per-wheel PWM multiplier applied ONLY during turning (driveSides()/
// straight driving is unaffected). Compensates for a wheel contributing
// less to rotation than the others -- e.g. RR suspected of poor ground
// contact, which loads extra weight onto FR and makes it grip too well to
// slip, so the chassis pivots on that corner instead of the center. This is
// a software band-aid for a hardware traction difference, not a fix for
// it -- if the mechanical cause (check RR's ground contact) gets fixed
// later, these should go back to 1.0.
//
// Tune by watching where the pivot actually lands: raise TURN_SCALE_RR in
// small steps (1.1, 1.2, ...) until the turn visibly centers; back off if
// it swings past center toward the opposite corner.
#define TURN_SCALE_LF 1.0f
#define TURN_SCALE_LR 1.0f
#define TURN_SCALE_RF 1.0f
#define TURN_SCALE_RR 1.0f


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

// Once a turn reaches tolerance, motors cut immediately (unlike
// driveControlStop()'s ramp) -- a turn's target IS the angle, so continuing
// to actively drive during a ramp-down directly overshoots it. This is just
// a passive wait after the instant stop, before reading the final heading
// for the "turned=" diagnostic -- lets real physical coast-down (not an
// active drive command) finish so the reported heading reflects a
// genuinely stationary robot.
#define TURN_SETTLE_MS 150

// How often turnControlUpdate() samples per-wheel encoder ticks during a
// turn, purely for diagnostics (which wheel is actually turning slower --
// e.g. to find the cause of an off-center pivot point) -- NOT used for
// closed-loop correction. Turns stay heading-only against the IMU; a
// single-channel encoder can't tell which wheel is going forward vs.
// backward mid-turn, so tick counts alone can't drive the turn itself, only
// report relative wheel speed.
#define TURN_ENCODER_SAMPLE_MS 150

// Number of turns TURN_TEST runs, alternating +90/-90 each time, before
// returning to IDLE. Pure in-place pivoting -- unlike the drive shuttle,
// needs no straight-line arena space at all.
#define TURN_TEST_REPEATS 1


// ---------------- GRIPPER SERVO (MG90) ----------------
#define GRIPPER_SERVO_PIN 34
#define GRIPPER_OPEN_ANGLE   180 // original position dont change!!!!!
#define GRIPPER_CLOSE_ANGLE  100
//#define GRIPPER_MS  1185
#define GRIPPER_MS  750


#define PIN_NOT_WIRED -1

// ---------------- BOX GATE SERVO (releases carried cube(s) at the drop zone) ----------------
#define GATE_SERVO_PIN 36
#define GATE_OPEN_ANGLE   45 
#define GATE_CLOSE_ANGLE  0 // original position

// How long TaskState::GATE_OPEN (PBL.ino) holds the gate open
#define GATE_OPEN_HOLD_MS 1500

// ---------------- TORQUE SERVO (MG996R) ----------------
#define LIFT_SERVO_PIN 35
// real value
#define LIFT_STOP 90
#define LIFT_UP_NUM 180
#define LIFT_DOWN_NUM 0


#define LIFTER_MS 930