#include "DriveControl.h"
#include "Motors.h"
#include "Encoder.h"
#include "PID.h"
#include "IMU.h"

static PidController pidLF, pidLR, pidRF, pidRR;

static unsigned long lastUpdateAt = 0;
static bool needsSeed = true;

// Last commanded (signed) PWM per wheel -- held between PID updates so the
// motors keep driving continuously instead of only moving on update ticks.
static int outLF = 0, outLR = 0, outRF = 0, outRR = 0;

void driveControlInit() {
  pidInit(pidLF, DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD, -DRIVE_PID_MAX_CORRECTION, DRIVE_PID_MAX_CORRECTION);
  pidInit(pidLR, DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD, -DRIVE_PID_MAX_CORRECTION, DRIVE_PID_MAX_CORRECTION);
  pidInit(pidRF, DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD, -DRIVE_PID_MAX_CORRECTION, DRIVE_PID_MAX_CORRECTION);
  pidInit(pidRR, DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD, -DRIVE_PID_MAX_CORRECTION, DRIVE_PID_MAX_CORRECTION);
}

void driveControlReset() {
  encoderResetAll();
  pidReset(pidLF);
  pidReset(pidLR);
  pidReset(pidRF);
  pidReset(pidRR);
  needsSeed = true;
}

// Flags a wheel reporting well under what the other three see -- a real PID
// gain issue moves all four together, so a lone outlier usually means an
// encoder/wiring/mounting problem on that wheel specifically, not something
// DRIVE_PID_KP/KI/KD can fix.
static void flagIfOutlier(const char* label, unsigned long ticks, unsigned long a, unsigned long b, unsigned long c) {
  unsigned long avgOthers = (a + b + c) / 3;
  if (avgOthers >= 5 && ticks < avgOthers / 2) {
    Serial.print("  !! "); Serial.print(label);
    Serial.print(" ticks ("); Serial.print(ticks);
    Serial.print(") far below other wheels' avg ("); Serial.print(avgOthers);
    Serial.println(") -- check that wheel's encoder wiring/mounting or traction");
  }
}

// ticks/setpoint are magnitudes (single-channel encoders can't see direction);
// baseSpeed's sign is reapplied to the corrected magnitude on the way out.
static int correctWheel(PidController& pid, unsigned long ticks, int baseSpeed, float dtSeconds) {
  if (baseSpeed == 0) return 0;
  float correction = pidCompute(pid, DRIVE_PID_TARGET_TICKS_PER_INTERVAL, (float)ticks, dtSeconds);
  int magnitude = constrain(abs(baseSpeed) + (int)correction, 0, 255);
  return (baseSpeed > 0) ? magnitude : -magnitude;
}

void driveControlUpdate(int leftSpeed, int rightSpeed) {
  unsigned long now = millis();

  if (needsSeed) {
    // Drive open-loop at the commanded speed immediately, rather than
    // sitting at 0 for the first interval waiting on the first correction.
    outLF = leftSpeed;
    outLR = leftSpeed;
    outRF = rightSpeed;
    outRR = rightSpeed;
    lastUpdateAt = now;
    needsSeed = false;
  } else if (now - lastUpdateAt >= DRIVE_PID_INTERVAL_MS) {
    float dtSeconds = (now - lastUpdateAt) / 1000.0f;
    lastUpdateAt = now;

    unsigned long ticksLF = encoderGetAndResetTicks(WheelId::LEFT_FRONT);
    unsigned long ticksLR = encoderGetAndResetTicks(WheelId::LEFT_REAR);
    unsigned long ticksRF = encoderGetAndResetTicks(WheelId::RIGHT_FRONT);
    unsigned long ticksRR = encoderGetAndResetTicks(WheelId::RIGHT_REAR);

    outLF = correctWheel(pidLF, ticksLF, leftSpeed, dtSeconds);
    outLR = correctWheel(pidLR, ticksLR, leftSpeed, dtSeconds);
    outRF = correctWheel(pidRF, ticksRF, rightSpeed, dtSeconds);
    outRR = correctWheel(pidRR, ticksRR, rightSpeed, dtSeconds);

    // Logged unconditionally for now -- gains are still placeholders and 
    // this is the data needed to tune them. Remove/gate behind a flag once tuned.
    Serial.print("DriveControl  LF t:"); Serial.print(ticksLF); Serial.print(" o:"); Serial.print(outLF);
    Serial.print("  LR t:"); Serial.print(ticksLR); Serial.print(" o:"); Serial.print(outLR);
    Serial.print("  RF t:"); Serial.print(ticksRF); Serial.print(" o:"); Serial.print(outRF);
    Serial.print("  RR t:"); Serial.print(ticksRR); Serial.print(" o:"); Serial.println(outRR);

    flagIfOutlier("LF", ticksLF, ticksLR, ticksRF, ticksRR);
    flagIfOutlier("LR", ticksLR, ticksLF, ticksRF, ticksRR);
    flagIfOutlier("RF", ticksRF, ticksLF, ticksLR, ticksRR);
    flagIfOutlier("RR", ticksRR, ticksLF, ticksLR, ticksRF);
  }

  setWheel(PIN_L_FRONT, PIN_L_IN1, PIN_L_IN2, outLF);
  setWheel(PIN_L_REAR,  PIN_L_IN3, PIN_L_IN4, outLR);
  setWheel(PIN_R_FRONT, PIN_R_IN1, PIN_R_IN2, outRF);
  setWheel(PIN_R_REAR,  PIN_R_IN3, PIN_R_IN4, outRR);
}

void driveControlStop() {
  int startLF = outLF, startLR = outLR, startRF = outRF, startRR = outRR;
  int stepDelayMs = DRIVE_DECEL_MS / DRIVE_DECEL_STEPS;

  for (int i = DRIVE_DECEL_STEPS - 1; i >= 0; i--) {
    float frac = (float)i / DRIVE_DECEL_STEPS;
    setWheel(PIN_L_FRONT, PIN_L_IN1, PIN_L_IN2, (int)(startLF * frac));
    setWheel(PIN_L_REAR,  PIN_L_IN3, PIN_L_IN4, (int)(startLR * frac));
    setWheel(PIN_R_FRONT, PIN_R_IN1, PIN_R_IN2, (int)(startRF * frac));
    setWheel(PIN_R_REAR,  PIN_R_IN3, PIN_R_IN4, (int)(startRR * frac));
    delay(stepDelayMs);
  }

  stopAll();
  delay(DRIVE_SETTLE_MS); // let residual momentum die out before the caller starts the next move
}

static float turnTargetHeading = 0;
static unsigned long turnTickStartMs = 0;
static unsigned long turnNearMs = 0;

static float wrap360(float heading) {
  while (heading < 0)    heading += 360.0f;
  while (heading >= 360) heading -= 360.0f;
  return heading;
}

// Shortest signed error from the current heading to `target`, -180..180.
static float turnHeadingError(float target) {
  float error = target - imuGetHeading();
  while (error > 180.0f)  error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

// per-wheel TURN_SCALE_* multipliers
// lets one wheel be pushed harder than the others during a turn specifically, 
// to compensate for it contributing less to rotation (e.g. reduced ground contact) 
// without touching straight-line driving at all. 

// setWheel() already clamps the final PWM to +/-255, so an
// over-scaled value here is safe, just gets capped.
static void driveTurnWheels(int leftPwm, int rightPwm) {
  setWheel(PIN_L_FRONT, PIN_L_IN1, PIN_L_IN2, (int)(leftPwm  * TURN_SCALE_LF));
  setWheel(PIN_L_REAR,  PIN_L_IN3, PIN_L_IN4, (int)(leftPwm  * TURN_SCALE_LR));
  setWheel(PIN_R_FRONT, PIN_R_IN1, PIN_R_IN2, (int)(rightPwm * TURN_SCALE_RF));
  setWheel(PIN_R_REAR,  PIN_R_IN3, PIN_R_IN4, (int)(rightPwm * TURN_SCALE_RR));
}

static float turnStartHeading = 0;

// Prints calibration.system + raw magnetometer, and warns if system==0 --
// per IMU.h, heading is only trustworthy once system>0. Added to chase down
// turn accuracy after moving heavy components (e.g. the battery) near the
// BNO055: a nearby ferrous/magnetic mass changes the sensor's magnetic
// environment and can silently invalidate the EEPROM-restored hard-iron
// calibration, which nothing elsewhere in the turn path checks for.
static void logImuHealth(const char* when) {
  ImuCalibration cal = imuGetCalibration();
  ImuVector3 mag = imuGetRawMagnetometer();
  Serial.print("  IMU "); Serial.print(when);
  Serial.print(": cal sys="); Serial.print(cal.system);
  Serial.print(" gyro="); Serial.print(cal.gyro);
  Serial.print(" accel="); Serial.print(cal.accel);
  Serial.print(" mag="); Serial.print(cal.mag);
  Serial.print("  rawMag(uT) x="); Serial.print(mag.x, 1);
  Serial.print(" y="); Serial.print(mag.y, 1);
  Serial.print(" z="); Serial.println(mag.z, 1);
  if (cal.system == 0) {
    Serial.println("  !! IMU system calibration is 0 -- heading is not trustworthy right now (see IMU.h). Recalibrate ('c' at IDLE).");
  }
}

void turnControlStart(float relativeDeg) {
  turnStartHeading = imuGetHeading();
  turnTargetHeading = wrap360(turnStartHeading + relativeDeg);
  turnTickStartMs = 0;
  turnNearMs = 0;
  // So the first "Turn ticks" diagnostic sample reflects only this turn's
  // motion, not leftover ticks from the previous state's stop ramp/settle.
  encoderResetAll();
  Serial.print("Turn start: heading="); Serial.print(turnStartHeading, 1);
  Serial.print(" target="); Serial.println(turnTargetHeading, 1);
  logImuHealth("at turn start");
}

// Prints how far the heading actually moved (vs. the requested relativeDeg from turnControlStart()) 
// so you can check real turn accuracy from the Serial log, then stops the motors and clears the turn's timing state.

// Unlike driveControlStop()'s ramp, this cuts power immediately rather than easing off:
// a turn's target IS the angle
// TURN_SETTLE_MS is a passive wait, that's supplied afterward, not a drive command
static bool finishTurn(const char* reason) {
  stopAll();

  // Trace heading through the settle window instead of a blind delay() --
  // this tells us definitively whether the robot is still physically
  // coasting after power is cut (and for how long), rather than guessing.
  const int settleSteps = 6;
  int settleStepMs = TURN_SETTLE_MS / settleSteps;
  for (int i = 0; i < settleSteps; i++) {
    delay(settleStepMs);
    Serial.print("  settle+"); Serial.print((i + 1) * settleStepMs);
    Serial.print("ms heading="); Serial.println(imuGetHeading(), 1);
  }

  turnTickStartMs = 0;
  turnNearMs = 0;

  float finalHeading = imuGetHeading();
  float turned = finalHeading - turnStartHeading;
  while (turned > 180.0f)  turned -= 360.0f;
  while (turned < -180.0f) turned += 360.0f;

  Serial.print("Turn done ("); Serial.print(reason);
  Serial.print("): heading="); Serial.print(finalHeading, 1);
  Serial.print(" turned="); Serial.print(turned, 1);
  Serial.println(" deg");
  logImuHealth("at turn end");
  return true;
}

// The PWM eases off near the target so we settle instead of slamming past;
// hovering close for TURN_NEAR_MS or hitting TURN_TIMEOUT_MS also counts as
// done, so a turn can never get stuck (e.g. on a stale/uncalibrated heading).
bool turnControlUpdate(int pwm) {
  // Diagnostic-only: which wheel is actually spinning slower during a real
  // turn (as opposed to inferring it from straight-line driving, which
  // loads each wheel very differently). Not used to correct the turn.
  static unsigned long lastEncoderSampleAt = 0;
  if (millis() - lastEncoderSampleAt >= TURN_ENCODER_SAMPLE_MS) {
    lastEncoderSampleAt = millis();
    unsigned long tLF = encoderGetAndResetTicks(WheelId::LEFT_FRONT);
    unsigned long tLR = encoderGetAndResetTicks(WheelId::LEFT_REAR);
    unsigned long tRF = encoderGetAndResetTicks(WheelId::RIGHT_FRONT);
    unsigned long tRR = encoderGetAndResetTicks(WheelId::RIGHT_REAR);
    Serial.print("  Turn ticks  LF:"); Serial.print(tLF);
    Serial.print("  LR:"); Serial.print(tLR);
    Serial.print("  RF:"); Serial.print(tRF);
    Serial.print("  RR:"); Serial.println(tRR);
    flagIfOutlier("LF", tLF, tLR, tRF, tRR);
    flagIfOutlier("LR", tLR, tLF, tRF, tRR);
    flagIfOutlier("RF", tRF, tLF, tLR, tRR);
    flagIfOutlier("RR", tRR, tLF, tLR, tRF);
    // Same cadence as the tick sample above -- catches calibration/magnetic
    // interference that only shows up once the motors are actually drawing
    // current mid-turn, not just at the (stationary) turn start.
    logImuHealth("mid-turn");
  }

  float err = turnHeadingError(turnTargetHeading);
  float dist = abs(err);

  if (turnTickStartMs == 0) turnTickStartMs = millis();

  if (dist <= TURN_HEADING_TOLERANCE_DEG) {
    return finishTurn("reached tolerance");
  }

  if (dist <= TURN_HEADING_TOLERANCE_DEG * 2.5f) {
    if (turnNearMs == 0) turnNearMs = millis();
    else if (millis() - turnNearMs >= TURN_NEAR_MS) {
      return finishTurn("settled near target");
    }
  } else {
    turnNearMs = 0;
  }

  if (millis() - turnTickStartMs > TURN_TIMEOUT_MS) {
    return finishTurn("timeout");
  }

  int turnPwm = pwm;
  if (millis() - turnTickStartMs < TURN_KICK_MS) {
    // Brief full-power burst to break static friction on every wheel before
    // falling back to the normal ramp -- see TURN_KICK_MS/PWM in Config.h.
    turnPwm = TURN_KICK_PWM;
  } else if (dist < TURN_RAMP_SPAN_DEG) {
    turnPwm = (int)(pwm * dist / TURN_RAMP_SPAN_DEG);
    turnPwm = max(turnPwm, TURN_PWM_FLOOR);
  }

  // err>0 -> target is clockwise of current heading -> turn right (matches
  // turnRight()'s driveSides(+,-) convention in Motors.cpp).
  if (err > 0) driveTurnWheels( turnPwm, -turnPwm);
  else         driveTurnWheels(-turnPwm,  turnPwm);
  return false;
}
