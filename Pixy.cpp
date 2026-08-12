#include <SPI.h>
#include <Pixy2.h>
#include "Pixy.h"

// callers only interact through pixyDetectInit()/pixyDetect(),
// they never need the Pixy2/Block types directly.
static Pixy2 pixy;

void pixyDetectInit() {
  pixy.init();
}

// Returns true and fills outColor if signature `sig` is the one relevant to `mode`.
static bool signatureMatchesMode(uint16_t sig, PixyDetectMode mode, CubeColor* outColor) {
  switch (mode) {

    // At Conveyor Belt
    case PixyDetectMode::SEEK_CUBE:
      if (sig == PIXY_SIG_RED)   {
        *outColor = CubeColor::RED;
        return true; }
      if (sig == PIXY_SIG_GREEN) {
        *outColor = CubeColor::GREEN;
        return true; }
      return false;

    // After Picking Up Red Cube
    case PixyDetectMode::SEEK_RED_DROP:
      if (sig == PIXY_SIG_DROP_RED) {
        *outColor = CubeColor::RED;
        return true; }
      return false;

    // After Picking Up Green Cube
    case PixyDetectMode::SEEK_GREEN_DROP:
      if (sig == PIXY_SIG_DROP_GREEN) { 
        *outColor = CubeColor::GREEN;
        return true; }
      return false;
  }
  return false;
}

PixyDetection pixyDetect(PixyDetectMode mode) {
  PixyDetection best{};
  best.found = false;
  best.color = CubeColor::NONE;

  pixy.ccc.getBlocks();

  uint32_t bestArea = 0;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    Block& b = pixy.ccc.blocks[i];

    if (b.m_width < PIXY_MIN_BLOCK_SIZE || b.m_height < PIXY_MIN_BLOCK_SIZE) continue;

    CubeColor color;
    if (!signatureMatchesMode(b.m_signature, mode, &color)) continue;

    // Among matches, keep the largest block: it's the closest/most reliable
    // candidate, and it's the one heuristic that holds regardless of the
    // cube's rotation (bounding-box area still grows as it nears the camera).
    uint32_t area = (uint32_t)b.m_width * b.m_height;
    if (!best.found || area > bestArea) {
      best.found = true;
      best.color = color;
      best.x = b.m_x;
      best.y = b.m_y;
      best.width = b.m_width;
      best.height = b.m_height;
      best.offsetFromCenterX = (int16_t)b.m_x - PIXY_FRAME_CENTER_X;
      bestArea = area;
    }
  }

  return best;
}
