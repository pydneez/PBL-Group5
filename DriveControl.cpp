#include "DriveControl.h"
#include "Motors.h"
#include "Encoder.h"
#include "PID.h"

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
  }

  setWheel(PIN_L_FRONT, PIN_L_IN1, PIN_L_IN2, outLF);
  setWheel(PIN_L_REAR,  PIN_L_IN3, PIN_L_IN4, outLR);
  setWheel(PIN_R_FRONT, PIN_R_IN1, PIN_R_IN2, outRF);
  setWheel(PIN_R_REAR,  PIN_R_IN3, PIN_R_IN4, outRR);
}
