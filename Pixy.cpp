#include <SPI.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>
#include "Pixy.h"

static Pixy2 pixy;

void pixyInit() {
  pixy.init();
  Serial.println("Pixy Setup Complete");
}

// True once (x, y) is within PICKUP_ZONE_ALLOWANCE_{X,Y} of the pickup
// target. See PICKUP_ZONE_TARGET_* in Config.h.
static bool isInPickupZone(int16_t x, int16_t y) {
  return abs((int)x - PICKUP_ZONE_TARGET_X) <= PICKUP_ZONE_ALLOWANCE_X &&
         abs((int)y - PICKUP_ZONE_TARGET_Y) <= PICKUP_ZONE_ALLOWANCE_Y;
}

// Outcome of scoring a single block in pixyDetect(), used for debug printing.
enum class BlockOutcome { REJECTED_SIZE, REJECTED_SIGNATURE, CANDIDATE };

static void printFrameHeader(uint8_t numBlocks) {
  if (!PIXY_DEBUG_PRINT_BLOCKS) return;
  Serial.print("-- frame: "); Serial.print(numBlocks); Serial.println(" block(s) --");
}

static void printBlock(int i, uint16_t sig, uint16_t x, uint16_t y,
                        uint16_t width, uint16_t height, uint32_t area,
                        BlockOutcome outcome, uint32_t distSqToZone, bool isNewBest) {
  if (!PIXY_DEBUG_PRINT_BLOCKS) return;

  Serial.print("  ["); Serial.print(i); Serial.print("] sig:"); Serial.print(sig);
  Serial.print(" x:"); Serial.print(x);
  Serial.print(" y:"); Serial.print(y);
  Serial.print(" w:"); Serial.print(width);
  Serial.print(" h:"); Serial.print(height);
  Serial.print(" area:"); Serial.print(area);

  switch (outcome) {
    case BlockOutcome::REJECTED_SIZE:
      Serial.println("  -> rejected (below PIXY_MIN_BLOCK_SIZE)");
      break;
    case BlockOutcome::REJECTED_SIGNATURE:
      Serial.println("  -> rejected (signature not relevant to mode)");
      break;
    case BlockOutcome::CANDIDATE:
      Serial.print(" distSqToZone:"); Serial.print(distSqToZone);
      Serial.println(isNewBest ? "  -> candidate (new best)" : "  -> candidate (not best)");
      break;
  }
}

static void printWinner(int bestIndex) {
  if (!PIXY_DEBUG_PRINT_BLOCKS) return;
  if (bestIndex >= 0) {
    Serial.print("  winner: block ["); Serial.print(bestIndex); Serial.println("]");
  }
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

  printFrameHeader(pixy.ccc.numBlocks);

  int bestIndex = -1;
  uint32_t bestDistSq = UINT32_MAX;
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    uint16_t signature = pixy.ccc.blocks[i].m_signature;
    uint16_t x          = pixy.ccc.blocks[i].m_x;
    uint16_t y           = pixy.ccc.blocks[i].m_y;
    uint16_t width        = pixy.ccc.blocks[i].m_width;
    uint16_t height        = pixy.ccc.blocks[i].m_height;
    uint32_t area = (uint32_t)width * height;

    if (width < PIXY_MIN_BLOCK_SIZE || height < PIXY_MIN_BLOCK_SIZE) {
      printBlock(i, signature, x, y, width, height, area, BlockOutcome::REJECTED_SIZE, 0, false);
      continue;
    }

    CubeColor color;
    if (!signatureMatchesMode(signature, mode, &color)) {
      printBlock(i, signature, x, y, width, height, area, BlockOutcome::REJECTED_SIGNATURE, 0, false);
      continue;
    }

    // Rank by proximity to the pickup zone, not color or area -- whichever
    // cube (red or green) is nearest to the trigger window wins, so we
    // grab the first one to arrive rather than waiting for a preferred color.
    int32_t dx = (int32_t)x - PICKUP_ZONE_TARGET_X;
    int32_t dy = (int32_t)y - PICKUP_ZONE_TARGET_Y;
    uint32_t distSq = (uint32_t)(dx * dx + dy * dy);

    printBlock(i, signature, x, y, width, height, area, BlockOutcome::CANDIDATE, distSq, distSq < bestDistSq || bestIndex == -1);

    if (!best.found || distSq < bestDistSq) {
      best.found = true;
      best.color = color;
      best.x = x;
      best.y = y;
      best.width = width;
      best.height = height;
      best.offsetFromCenterX = (int16_t)x - PIXY_FRAME_CENTER_X;
      best.inPickupZone = isInPickupZone(x, y);
      bestDistSq = distSq;
      bestIndex = i;
    }
  }

  printWinner(bestIndex);

  return best;
}


