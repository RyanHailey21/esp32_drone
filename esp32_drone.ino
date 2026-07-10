#include <Arduino.h>
#include <NimBLEDevice.h>
#include "Config.h"
#include "State.h"
#include "Msp.h"
#include "Control.h"
#include "Ble.h"
#include "Mission.h"
#include "Tof.h"

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 1000);  // wait up to 1s for USB host, then continue
    fcSerial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
    setupTof();

    pinMode(STATUS_LED, OUTPUT);

    setupBLE();

    Serial.println("[BOOT] Ready. Control via BLE — open quad_tuner.html in Chrome.");
    Serial.printf("[BOOT] Mission profile: SPRINT -> HOLD -> PUNCH -> %s\n",
        MISSION_TYPE ? "AUTOROTOR CUT" : "POWERED LAND");
}

void loop() {
    uint32_t loopStartUs = micros();
    runMissionLoop();
    uint32_t elapsedUs = micros() - loopStartUs;
    if (elapsedUs < CONTROL_LOOP_PERIOD_US) {
        delayMicroseconds(CONTROL_LOOP_PERIOD_US - elapsedUs);
    }
}
