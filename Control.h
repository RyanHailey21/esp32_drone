#pragma once

#include "State.h"
#include "Msp.h"

uint16_t holdCascaded(float altitude, bool isMission);
void     resetCascadeController(float currentAlt);
void     primeCascadeController(float currentAlt);
void disarmToIdle(const char* reason);
void startHoverTest();
void startMission();
void startAutoHoverCal();
void startAltHold();
void startLanding(float currentAlt);
