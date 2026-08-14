#include "IMU.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

static Adafruit_BNO055 bno(BNO055_SENSOR_ID, BNO055_I2C_ADDR, &Wire);

// True once this session's offsets are known-good in EEPROM, either because
// imuInit() just restored them or imuSaveCalibration() just wrote them --
// either way there's nothing left to save.
static bool calibrationSavedThisSession = false;

bool imuInit() {
  if (!bno.begin(OPERATION_MODE_NDOF)) {
    return false;
  }
  delay(1000); // let the sensor settle before use (matches Adafruit's own example)

  int eeAddress = IMU_EEPROM_CALIB_ADDR;
  long storedId;
  EEPROM.get(eeAddress, storedId);

  sensor_t sensor;
  bno.getSensor(&sensor);

  if (storedId == sensor.sensor_id) {
    adafruit_bno055_offsets_t calibData;
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibData);
    bno.setSensorOffsets(calibData);
    calibrationSavedThisSession = true;
    Serial.println("IMU: restored calibration offsets from EEPROM.");
  } else {
    Serial.println("IMU: no stored calibration for this sensor ID yet -- calibrate before trusting heading.");
  }

  // Crystal must be configured AFTER loading calibration data into the BNO055.
  bno.setExtCrystalUse(true);
  return true;
}

float imuGetHeading() {
  // VECTOR_EULER: x = heading, y = roll, z = pitch. Only heading is exposed
  // here since that's all turning/driving needs; roll/pitch are available
  // the same way if needed later.
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("Heading: "); Serial.println(euler.x());
  Serial.print("Roll:    "); Serial.println(euler.y());
  Serial.print("Pitch:   "); Serial.println(euler.z());
  return euler.x();
  
}

ImuCalibration imuGetCalibration() {
  ImuCalibration cal{};
  bno.getCalibration(&cal.system, &cal.gyro, &cal.accel, &cal.mag);
  return cal;
}

bool imuIsFullyCalibrated() {
  return bno.isFullyCalibrated();
}

bool imuHasValidCalibration() {
  return calibrationSavedThisSession;
}

ImuVector3 imuGetRawMagnetometer() {
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  return { (float)mag.x(), (float)mag.y(), (float)mag.z() };
}

void imuSaveCalibration() {
  if (calibrationSavedThisSession) return;

  // getSensorOffsets() itself no-ops (leaving offsets untouched/uninitialized)
  // unless isFullyCalibrated() is true at this exact moment -- check the
  // return value, or a not-quite-calibrated call could write garbage to EEPROM.
  adafruit_bno055_offsets_t offsets;
  if (!bno.getSensorOffsets(offsets)) {
    Serial.println("IMU: not fully calibrated -- skipping EEPROM save.");
    return;
  }

  sensor_t sensor;
  bno.getSensor(&sensor);

  int eeAddress = IMU_EEPROM_CALIB_ADDR;
  long id = sensor.sensor_id;
  EEPROM.put(eeAddress, id);
  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, offsets);

  calibrationSavedThisSession = true;
  Serial.println("IMU: calibration saved to EEPROM.");
}

bool imuRestartCalibration() {
  long invalidId = -1; // never matches BNO055_SENSOR_ID, so imuInit() below skips restoring
  EEPROM.put(IMU_EEPROM_CALIB_ADDR, invalidId);
  calibrationSavedThisSession = false;
  Serial.println("IMU: cleared stored calibration from EEPROM. Resetting sensor...");
  return imuInit();
}
