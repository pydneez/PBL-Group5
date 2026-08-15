#include "IMU.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>

static Adafruit_BNO055 bno(BNO055_SENSOR_ID, BNO055_I2C_ADDR, &Wire);

// True once this session's offsets are known-good in EEPROM, either because
// imuInit() just restored them or imuSaveCalibration() just wrote them --
// either way there's nothing left to save.
static bool calibrationSavedThisSession = true;

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
  //bno.setExtCrystalUse(true);
  return true;
}

float imuGetHeading() {
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
  return event.orientation.x;
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
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  return { event.magnetic.x, event.magnetic.y, event.magnetic.z };
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

void imuPrintDiagnostics() {
  uint8_t mode = bno.getMode();
  uint8_t sysStatus, selfTest, sysError;
  bno.getSystemStatus(&sysStatus, &selfTest, &sysError);
  Serial.print("  IMU diag: mode=0x"); Serial.print(mode, HEX);
  Serial.print(" (NDOF=0x0C)  sysStatus="); Serial.print(sysStatus);
  Serial.print(" selfTest=0x"); Serial.print(selfTest, HEX);
  Serial.print(" sysError="); Serial.println(sysError);
}

bool imuRestartCalibration() {
  long invalidId = -1; // never matches BNO055_SENSOR_ID, so imuInit() below skips restoring
  EEPROM.put(IMU_EEPROM_CALIB_ADDR, invalidId);
  calibrationSavedThisSession = false;
  Serial.println("IMU: cleared stored calibration from EEPROM. Resetting sensor...");
  return imuInit();
}
