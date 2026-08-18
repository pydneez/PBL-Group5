#pragma once

#include <Arduino.h>
#include "Config.h"

// Returns false if the BNO055 wasn't detected on the I2C bus (check wiring/address).
bool imuInit();

float imuGetHeading();

// Each field is 0-3 (3 = fully calibrated). "system" reflects overall fusion
// health; the library's own guidance is to ignore other readings while it's 0.
struct ImuCalibration {
  uint8_t system;
  uint8_t gyro;
  uint8_t accel;
  uint8_t mag;
};
ImuCalibration imuGetCalibration();

bool imuIsFullyCalibrated();

bool imuHasValidCalibration();

// Raw magnetic field in microtesla, unfused. Unlike imuGetHeading() (which
// runs through the onboard sensor-fusion filter and can partially smooth
// over brief interference), this is the most direct way to see whether
// something nearby -- e.g. a motor driver -- is disturbing the magnetometer.
struct ImuVector3 {
  float x;
  float y;
  float z;
};

ImuVector3 imuGetRawMagnetometer();

void imuSaveCalibration();

bool imuRestartCalibration();
