#pragma once

#include <Arduino.h>
#include "Config.h"

// Attaches the servo and moves it to the open position.
void gripperInit();

void gripperOpen();
void gripperClose();
