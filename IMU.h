#pragma once

#include <Arduino.h>
#include "Config.h"

// Returns false if the BNO055 wasn't detected on the I2C bus (check wiring/address).
bool imuInit();

// 0-360 degrees, magnetic-north-referenced (NDOF fusion: accel+gyro+magnetometer).
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

// True once the BNO055 has known-good calibration loaded for this session --
// either imuInit() just restored it from EEPROM, or imuSaveCalibration() has
// since written a fresh one. False right after imuRestartCalibration() until
// recalibrated. Meant to let a caller skip a fresh calibration wait at boot
// when EEPROM data is already trusted (the offsets take effect the instant
// imuInit() restores them -- this doesn't wait on the live status bits to
// re-confirm, it just reports whether known offsets are loaded).
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

// Writes the current sensor offsets to EEPROM (keyed by this sensor's unique
// ID) so imuInit() can restore them on a future boot. Call once
// imuIsFullyCalibrated() is true; no-ops after the first successful save (or
// after imuInit() already restored offsets) so it's safe to call every loop.
void imuSaveCalibration();

// Erases the stored calibration in EEPROM and re-runs imuInit(), which
// performs a full sensor reset -- the BNO055 comes back up uncalibrated
// instead of restoring the old offsets, so you can redo calibration from
// scratch. Follow up with imuSaveCalibration() once imuIsFullyCalibrated()
// is true again to persist the new result. Returns false if the BNO055
// isn't detected on the reset.
bool imuRestartCalibration();
