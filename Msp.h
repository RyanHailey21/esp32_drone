#pragma once

#include <Arduino.h>
#include "State.h"

void sendMSP(uint8_t cmd, uint8_t* data, uint8_t len);
void sendRC();
float getAltitude();
