#pragma once

#include <Arduino.h>
#include "Config.h"

// What color a detected cube/marker is.
enum class CubeColor : uint8_t {
  NONE  = 0,
  RED   = PIXY_SIG_RED,
  GREEN = PIXY_SIG_GREEN
};

// What pixyDetect() should look for. 
enum class PixyDetectMode : uint8_t {
  SEEK_CUBE,       // looking for a red or green cube 
  SEEK_RED_DROP,   // carrying a red cube, aiming at the red zone marker
  SEEK_GREEN_DROP  // carrying a green cube, aiming at the green zone marker 
};

// Result of one detection pass.
struct PixyDetection {
  bool found;
  CubeColor color;
  int16_t x, y;                // block center, raw Pixy coordinates
  int16_t width, height;       // block bounding box, px
  int16_t offsetFromCenterX;   // x - PIXY_FRAME_CENTER_X; negative = left of center, positive = right
  bool inPickupZone;           // true once x,y settle inside the PICKUP_ZONE_* window -- grabber trigger
};

void pixyInit();
PixyDetection pixyDetect(PixyDetectMode mode);
