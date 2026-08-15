#include <SPI.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>
#include "Pixy.h"

static Pixy2 pixy;

void pixyInit() {
  pixy.init();
  Serial.println("Pixy Setup Complete");
}

// Returns true and fills outColor if signature `sig` is the one relevant to `mode`.
static bool signatureMatchesMode(uint16_t sig, PixyDetectMode mode, CubeColor* outColor) {
  switch (mode) {

    // At Conveyor Belt: either cube color is a candidate here; pixyDetect()'s
    // scoring loop picks the best one across all matching blocks.
    case PixyDetectMode::SEEK_CUBE:
      if (sig == PIXY_SIG_RED) {
        *outColor = CubeColor::RED;
        return true;
      }
      if (sig == PIXY_SIG_GREEN) {
        *outColor = CubeColor::GREEN;
        return true;
      }
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

#if PIXY_DEBUG_PRINT_BLOCKS
  Serial.print("-- frame: "); Serial.print(pixy.ccc.numBlocks); Serial.println(" block(s) --");
#endif

  int bestIndex = -1;
  uint32_t bestScore = 0;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    uint16_t signature = pixy.ccc.blocks[i].m_signature;
    uint16_t x          = pixy.ccc.blocks[i].m_x;
    uint16_t y           = pixy.ccc.blocks[i].m_y;
    uint16_t width        = pixy.ccc.blocks[i].m_width;
    uint16_t height        = pixy.ccc.blocks[i].m_height;
    uint32_t area = (uint32_t)width * height;

#if PIXY_DEBUG_PRINT_BLOCKS
    Serial.print("  [") ; Serial.print(i); Serial.print("] sig:"); Serial.print(signature);
    Serial.print(" x:"); Serial.print(x);
    Serial.print(" y:"); Serial.print(y);
    Serial.print(" w:"); Serial.print(width);
    Serial.print(" h:"); Serial.print(height);
    Serial.print(" area:"); Serial.print(area);
#endif

    if (width < PIXY_MIN_BLOCK_SIZE || height < PIXY_MIN_BLOCK_SIZE) {
#if PIXY_DEBUG_PRINT_BLOCKS
      Serial.println("  -> rejected (below PIXY_MIN_BLOCK_SIZE)");
#endif
      continue;
    }

    CubeColor color;
    if (!signatureMatchesMode(signature, mode, &color)) {
#if PIXY_DEBUG_PRINT_BLOCKS
      Serial.println("  -> rejected (signature not relevant to mode)");
#endif
      continue;
    }

    // GREEN gets a flat bonus added to its area before comparing, since
    // it's worth more at the drop-off -- see CUBE_GREEN_BONUS_AREA
    // above: 0 by default, so this is pure largest/closest-wins until
    // it's tuned with real cube sizes.
    uint32_t score = area + (color == CubeColor::GREEN ? CUBE_GREEN_BONUS_AREA : 0);

#if PIXY_DEBUG_PRINT_BLOCKS
    Serial.print(" score:"); Serial.print(score);
    Serial.println(score > bestScore || bestIndex == -1 ? "  -> candidate (new best)" : "  -> candidate (not best)");
#endif

    if (!best.found || score > bestScore) {
      best.found = true;
      best.color = color;
      best.x = x;
      best.y = y;
      best.width = width;
      best.height = height;
      best.offsetFromCenterX = (int16_t)x - PIXY_FRAME_CENTER_X;
      bestScore = score;
      bestIndex = i;
    }
  }

#if PIXY_DEBUG_PRINT_BLOCKS
  if (bestIndex >= 0) {
    Serial.print("  winner: block ["); Serial.print(bestIndex); Serial.println("]");
  }
#endif

  return best;
}

// see multiple cube(s) in its field of view
// see its location? pixy.ccc.blocks[0].m_x; pixy.ccc.numBlocks[i].m_y

// prioritize high-points block -> green block (signature 2) (+3 points)
// choose a certain block and track the object location  
// move towards it (optimistic assumption that the robot faces the belt directly)
// as it moves torwards the belt -> take disnce from the front sonar pin (would this be blocking? please write non-blocking)
// stop moving towards the conveyor belt when sonar sensor reach threshold 

// but what if the robot come to the conveyor belt at an angle because that is how we track the object location
// is it possible for after grabbing the cube and backing up, it can turn towards to right direction (not technially exactly 90 degree turns anymore)
//although when the robot turn to face the direction to dorp thecube off, it can also check whether is see the right colur for the dop off point, can it adjust the body then






