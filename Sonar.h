#pragma once

#include <Arduino.h>
#include "Config.h"
float sonarGetDistanceCm(int trigPin, int echoPin);

void sonarInit();
float sonarGetBackCm();
