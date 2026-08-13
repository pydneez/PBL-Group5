#pragma once

#include <Arduino.h>
#include "Config.h"

enum class WheelId : uint8_t { LEFT_FRONT, LEFT_REAR, RIGHT_FRONT, RIGHT_REAR };

void encoderInit();

// Cumulative tick count since boot (or the last encoderResetAll()).
unsigned long encoderGetTicks(WheelId wheel);

// Returns ticks accumulated since the last call for this wheel, then resets
// its counter to zero. Call once per control interval to get "ticks this
// interval" as the speed measurement for a per-wheel PID.
unsigned long encoderGetAndResetTicks(WheelId wheel);

void encoderResetAll();
