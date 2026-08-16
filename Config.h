#pragma once

// ---------------- MOTOR PINS ----------------
#define PIN_L_IN1  24   // left front direction 
#define PIN_L_IN2  25
#define PIN_L_IN3  23   // left rear direction 
#define PIN_L_IN4  22
#define PIN_L_FRONT  12    // left front speed (PWM)
#define PIN_L_REAR  13     // left rear speed (PWM)

#define PIN_R_IN1  28   // right front direction -- swapped
#define PIN_R_IN2  29
#define PIN_R_IN3  26   // right rear direction 
#define PIN_R_IN4  27
#define PIN_R_FRONT  10    // right front speed (PWM)
#define PIN_R_REAR  11    // right rear speed (PWM)

// ---------------- TIMING / SPEED ----------------
// Safety-net max drive time for DRIVE_FORWARD_TO_CENTER -- the sonar (see
// SONAR_DROPZONE_STOP_CM / DRIVE_BEFORE_SONAR_MS below) is what actually
// stops the robot; this just keeps a stuck/failed sonar reading from driving
// forever. Must stay well above DRIVE_BEFORE_SONAR_MS or the timeout fires
// before sonar polling even starts. Was 900ms when this was purely a
// fixed-time drive -- retune down once the real distance-to-wall is known.
#define DRIVE_MS    900 // beore it was 900 until the center
#define TURN_MS     800   // unused now that turns are closed-loop against IMU heading (see TURN_* below)
#define BACK_MS     1500   // how long to reverse
#define CRUISE_SPEED 180

// Ceiling speed for the wall/belt approach -- eased down to
// SONAR_FRONT_MIN_SPEED as the front sonar closes in (see
// driveControlCruiseToSonarStop()). Must stay above SONAR_FRONT_MIN_SPEED or
// the ease-down math inverts. Placeholder above the measured stall floor --
// retune once real approach behavior is characterized.
#define APPROACH_SPEED 90
#define TURN_SPEED   100

// Number of forward/backward round trips the DRIVE_FORWARD_TO_CENTER<->DRIVE_BACKWARD
// PID calibration shuttle runs before returning to IDLE. 
// Net displacement stays near zero each round trip 
#define DRIVE_TEST_REPEATS 2


// ---------------- PIXY2 CAMERA ----------------
// Set to 1 to print per-block detection details (signature/position/size/
// score) to Serial each frame; 0 for silent normal operation.
#define PIXY_DEBUG_PRINT_BLOCKS 0

// x:0-315, y:0-207 range
#define PIXY_FRAME_WIDTH     316
#define PIXY_FRAME_HEIGHT    208
#define PIXY_FRAME_CENTER_X  (PIXY_FRAME_WIDTH / 2)
#define PIXY_FRAME_CENTER_Y  (PIXY_FRAME_HEIGHT / 2)


// Trained signature IDs (taught in PixyMon)
#define PIXY_SIG_RED         1   // red cube
#define PIXY_SIG_GREEN       2   // green cube
#define PIXY_SIG_DROP_RED    3   // red drop-off zone marker
#define PIXY_SIG_DROP_GREEN  4   // green drop-off zone marker

// Reject blocks smaller than this (px) on either axis: filters camera
// noise / specular glints (e.g. off the untrained white cube)
#define PIXY_MIN_BLOCK_SIZE  8

// Added to a GREEN candidate's bounding-box area when SEEK_CUBE scores
// red vs. green (green cubes are worth more at the drop-off, see
// Pixy.cpp's scoring comment). 0 = no priority, pure largest/closest-wins
// (today's behavior). Tune upward once real cube sizes are known from
// PixyMon/Serial.
#define CUBE_GREEN_BONUS_AREA 0

// DETECT_CUBE (PBL.ino) requires this many consecutive frames reporting the
// same pixyDetect(SEEK_CUBE) match before committing to LIFT_DOWN -- guards
// against a single noisy/spurious frame (specular glint, momentary misread)
// triggering a real mechanical grab. Not yet measured against a real false-
// positive rate; lower if this feels sluggish to react, raise if it commits
// on noise.
#define DETECT_CUBE_CONFIRM_FRAMES 3

// Safety net: if no cube is confirmed within this long, DETECT_CUBE gives up
// and returns to IDLE rather than sitting at the belt forever waiting on a
// Pixy misread or an empty belt. Placeholder -- retune once real belt
// cube-arrival timing is known.
#define DETECT_CUBE_TIMEOUT_MS 4000

// ---------------- SONAR / ULTRASONIC ----------------
#define TRIG_FRONT 38
#define ECHO_FRONT 39
#define TRIG_LEFT 46
#define ECHO_LEFT 47
#define TRIG_RIGHT 44   
#define ECHO_RIGHT 45  

// Front distance (cm) at which the robot is close enough to the drop-off wall
#define SONAR_DROPZONE_STOP_CM  15 
#define SONNAR_DROPZONE_SLOW_CM 25
#define SONAR_BELT_STOP_CM  4
#define SONAR_BELT_SLOW_CM 10

#define SONAR_DEADBAND 2

// How long the front distance must stay continuously inside the +/-
// SONAR_DEADBAND window before driveControlCruiseToSonarStop() commits to
// "reached" (full decel-ramp stop, return true). Any excursion back outside
// the deadband resets this timer. Guards against committing on a single
// lucky reading that's about to drift back out -- see the hold logic there.
#define SONAR_HOLD_MS 300

// Floor PWM for the wall/belt approach (both forward, easing down as
// distance closes in, and the gentle backup if it overshoots past
// STOP_CM - SONAR_DEADBAND).\ 
// 60 was enough to get all four turning together from 0. 
// If a wheel is dragging during a slow approach, raise this before touching anything else.
#define SONAR_FRONT_MIN_SPEED    65

// Measured side clearance (cm) when the robot is centered in the drop-off wall
#define SONAR_SIDE_CENTER_CM    25

// Placeholder: how long to drive straight (open-loop, no PID yet) before the
// front sonar starts being polled. Retune once PID/speed is characterized.
#define DRIVE_BEFORE_SONAR_MS  500

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

// Target ticks-per-interval at CRUISE_SPEED (150 PWM) -- placeholder derived
// from the clean (pre-wall) portion of a real DRIVE_FORWARD_TO_CENTER run under active
// PID (LF 46.0, LR 44.0, RF 62.1, RR 53.6 avg -- overall ~51). Retune as more
// runs come in.
#define DRIVE_PID_TARGET_TICKS_PER_INTERVAL 51

// Gains are NOT tuned -- same placeholder status as PID.h's comment describes.
#define DRIVE_PID_KP 2.0f
#define DRIVE_PID_KI 0.0f
#define DRIVE_PID_KD 0.0f

// Correction is clamped to +/- this many PWM units so one noisy reading
// can't slam a wheel's speed to 0 or 255.
#define DRIVE_PID_MAX_CORRECTION 50

// Below this commanded |PWM|, the tick-rate PID is skipped and the wheel
// drives open-loop at baseSpeed instead. DRIVE_PID_TARGET_TICKS_PER_INTERVAL
// was measured at CRUISE_SPEED, not at the APPROACH_SPEED/SONAR_FRONT_MIN_SPEED
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

#define BNO055_I2C_ADDR   0x29   
#define BNO055_SENSOR_ID  55     

#define IMU_EEPROM_CALIB_ADDR 0

// Safety net: if full calibration (sys/gyro/accel/mag all == 3) isn't
// reached within this long, IMU_CALIBRATE gives up waiting and proceeds
// anyway on whatever calibration it has (typically EEPROM-restored offsets
// from a prior session) rather than hanging the whole run. A full
// from-scratch calibration (no EEPROM data yet) should be done ahead of a
// match, not relied on to finish within this window.
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

// PWM floor applied while ramping down near the target, so friction can't
// stall the turn before it reaches tolerance.
//
// Was 80, but logged data showed that stalling outright: per-wheel encoder
// ticks during a real turn went to 0 on ALL FOUR wheels for 1.5+ seconds
// once the ramp pinned PWM at 80 (dist ~10deg, inside TURN_RAMP_SPAN_DEG),
// eventually hitting TURN_TIMEOUT_MS instead of ever reaching tolerance --
// not a startup/breakaway issue (the kick phase clearly worked; ticks were
// healthy for the first ~1.3s), just insufficient torque to sustain motion
// at that PWM under real load. Was then raised to 150 when TURN_SPEED was
// still 200, which worked as a floor (below cruise). But TURN_SPEED has
// since dropped to 100 and this wasn't revisited -- at 150 the "floor" was
// actually 50% ABOVE cruise speed, so the ramp zone sped the turn up right
// before the tolerance check instead of slowing it down, guaranteeing an
// overshoot (measured: 90deg turn overshot to 131.6deg before timing out).
// Dropped back below TURN_SPEED so the ramp is a real deceleration again --
// but 70 turned out to be back in the stalling range documented above
// (measured: RR ticks hit 0, LR near 0, heading went flat/no-coast in the
// settle trace, and the turn only ended via TURN_TIMEOUT_MS, not tolerance).
// Split the difference between "stalls" (<=80) and "too fast to decelerate"
// (>=TURN_SPEED=100).
#define TURN_PWM_FLOOR 70

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
#define TURN_RAMP_SPAN_DEG 30

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

// At the very start of a turn, drive at TURN_KICK_PWM (unconditionally,
// overriding the ramp) for this long -- a brief full-power burst to
// guarantee every wheel actually breaks static friction and starts moving
// together, instead of the front-right wheel occasionally stalling
// (motor audibly spinning, wheel not turning) at a lower starting PWM and
// becoming a fixed pivot point while the other three sweep around it.
// Bounded by the same tolerance/near/timeout checks as normal turning, so
// it can't drive past target even for a very small relativeDeg.
//
// TURN_KICK_PWM was 255 (max) but that produced a worse regression: ALL
// FOUR wheels stalling (motors audibly running, no wheel movement) on right
// turns specifically -- consistent with a voltage brownout from commanding
// every motor to full power simultaneously, not a friction issue (a corner-
// weight imbalance would affect at most 2 wheels, not all 4). Backed off to
// something still above TURN_SPEED but well short of max while this is
// being diagnosed -- lower further if the stall persists.
//
// Was 200 against TURN_SPEED=200, i.e. no burst at all above cruise. Once
// TURN_SPEED dropped to 100 this became a 2x burst that was never re-tuned,
// adding to the overshoot alongside TURN_PWM_FLOOR (see that comment).
// Halved to stay a real-but-smaller burst above the new cruise speed.
#define TURN_KICK_MS 160
#define TURN_KICK_PWM 130

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
#define GRIPPER_CLOSE_ANGLE  65
//#define GRIPPER_MS  1185
#define GRIPPER_MS  750

#define DOOR_SERVO_PIN 34
#define DOOR_OPEN_ANGLE   180 // original position 
#define DOOR_CLOSE_ANGLE  120 // PLACEHOLDER -> still need to figure out
#define DOOR_MS  750

// Derived, not tuned directly -- see GRIPPER_SERVO_MS_PER_DEG above.
// #define GRIPPER_MS  ((GRIPPER_OPEN_ANGLE - GRIPPER_CLOSE_ANGLE) * GRIPPER_SERVO_MS_PER_DEG + SERVO_SETTLE_MARGIN_MS)

#define PIN_NOT_WIRED -1

// ---------------- TORQUE SERVO (MG996R) ----------------
#define LIFT_SERVO_PIN 35
#define LIFT_UP_ANGLE 200
#define LIFT_DOWN_ANGLE 0
#define LIFTER_MS 780