#include "PID.h"

void pidInit(PidController& pid, float kp, float ki, float kd, float outMin, float outMax) {
  pid.kp = kp;
  pid.ki = ki;
  pid.kd = kd;
  pid.outMin = outMin;
  pid.outMax = outMax;
  pidReset(pid);
}

float pidCompute(PidController& pid, float setpoint, float measured, float dtSeconds) {
  float error = setpoint - measured;

  pid.integral += error * dtSeconds;

  float derivative = 0;
  if (pid.hasPrevError && dtSeconds > 0) {
    derivative = (error - pid.prevError) / dtSeconds;
  }
  pid.prevError = error;
  pid.hasPrevError = true;

  // TODO: consider clamping pid.integral once real gains exist -- unclamped
  // integral can wind up while output is saturated at outMin/outMax.
  float output = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
  return constrain(output, pid.outMin, pid.outMax);
}

void pidReset(PidController& pid) {
  pid.integral = 0;
  pid.prevError = 0;
  pid.hasPrevError = false;
}
