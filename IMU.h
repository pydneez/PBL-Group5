#pragma once

#include <Arduino.h>
#include "Config.h"

// Returns false if the BNO055 wasn't detected on the I2C bus (check wiring/address).
bool imuInit();

// 0-360 degrees, relative fusion heading (IMUPLUS: accel+gyro only, no
// magnetometer -- NOT referenced to magnetic north). Every caller only cares
// about deltas from a heading read at the start of a maneuver, so dropping
// the magnetometer trades away absolute compass bearing for immunity to the
// motor EMI that was corrupting it (see imuInit() in IMU.cpp).
// Only trustworthy once calibration.system > 0 -- see imuGetCalibration().
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

// Debug-only: prints the chip's actual operating mode register and system
// status/error registers to Serial. Unlike imuGetHeading()/imuGetCalibration()
// (which silently read back all-zero on a failed I2C transaction), this reads
// registers that reveal WHY: mode != IMUPLUS means something reset/never left
// config mode; a nonzero system error means the chip is alive and answering
// I2C but reports an internal fault (see BNO055 datasheet 4.3.59).
void imuPrintDiagnostics();
