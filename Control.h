#pragma once

#include "State.h"
#include "Msp.h"

uint16_t holdPID(float altitude);
void disarmToIdle(const char* reason);
void startHoverTest();
void startMission();
void startAutoHoverCal();
void startAltHold();
void startLanding(float currentAlt);
