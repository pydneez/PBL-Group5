#pragma once

#include <Arduino.h>
#include "Config.h"

// Generic single-sensor read. Returns -1 if no echo was received within the
// timeout (out of range / no reflective surface) -- callers MUST check for
// this before comparing against a "stop" threshold, since -1 would otherwise
// read as "extremely close".
float sonarGetDistanceCm(int trigPin, int echoPin);

void sonarInit();
float sonarGetFrontCm();
float sonarGetLeftCm();
float sonarGetRightCm();
