#pragma once

#include <Arduino.h>
#include "State.h"

void sendRC();
float getAltitude();
bool mspAltitudeFresh();
void resetAltitudeFusion();
