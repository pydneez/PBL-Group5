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
// Mutually exclusive mode: 
// a cube sitting at a drop-off zone will still be in frame while
// we're aiming at that zone's marker, so we only ever match the one
// signature relevant to the current mode instead of scanning for all of them.
enum class PixyDetectMode : uint8_t {
  SEEK_CUBE,       // looking for a red or green cube to pick up (sig 1 or 2)
  SEEK_RED_DROP,   // carrying a red cube, aiming at the red zone marker (sig 3)
  SEEK_GREEN_DROP  // carrying a green cube, aiming at the green zone marker (sig 4)
};

// Result of one detection pass. 
// Note: Pixy2 only reports a real m_angle for
// multi-signature "color code" blocks; plain signatures like ours always
// report angle 0, so no rotation field is exposed here. width/height can be
// used as a rough "how close is it" proxy instead (a rotated cube's box
// grows/skews but its area still increases as it nears the camera).
struct PixyDetection {
  bool found;
  CubeColor color;
  int16_t x, y;                // block center, raw Pixy coordinates
  int16_t width, height;       // block bounding box, px
  int16_t offsetFromCenterX;   // x - PIXY_FRAME_CENTER_X; negative = left of center, positive = right
};

void pixyDetectInit();
PixyDetection pixyDetect(PixyDetectMode mode);
