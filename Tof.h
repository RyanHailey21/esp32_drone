#pragma once

#include <Arduino.h>

void setupTof();
bool readTofAltitude(float& altitudeM);
float tofBlendWeight(float altitudeM, bool valid);
