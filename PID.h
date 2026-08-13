#pragma once

#include <Arduino.h>

// Generic, reusable PID controller -- not specific to wheels or encoders.
// Gains are NOT tuned: there's no hardware yet to validate against. Tune
// kp/ki/kd once real encoder data is available.
//
// Not wired into any control loop yet. Intended future usage, once the
// encoders are physically wired and gains are tuned (one instance per wheel
// for a per-wheel speed PID):
//
//   PidController pidLF;
//   pidInit(pidLF, /*kp=*/1.0f, /*ki=*/0.0f, /*kd=*/0.0f, /*outMin=*/-255, /*outMax=*/255);
//   ...
//   unsigned long ticks = encoderGetAndResetTicks(WheelId::LEFT_FRONT);
//   float correction = pidCompute(pidLF, targetTicksPerInterval, (float)ticks, dtSeconds);

struct PidController {
  float kp, ki, kd;
  float outMin, outMax;

  float integral;
  float prevError;
  bool hasPrevError;
};

void pidInit(PidController& pid, float kp, float ki, float kd, float outMin, float outMax);

// dtSeconds: time elapsed since the previous call, in seconds.
float pidCompute(PidController& pid, float setpoint, float measured, float dtSeconds);

// Clears accumulated integral/derivative history. Call before reusing a PID
// after a stop, so a stale integral term doesn't cause a jump on restart.
void pidReset(PidController& pid);
