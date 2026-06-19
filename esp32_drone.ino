#include <Arduino.h>
#include <NimBLEDevice.h>
#include "Config.h"
#include "State.h"
#include "Msp.h"
#include "Control.h"
#include "Ble.h"
#include "Mission.h"

void setup() {
    Serial.begin(115200);
    fcSerial.begin(115200, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);

    ledcAttach(MOTOR_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTOR_PWM_PIN, 0);

    pinMode(STATUS_LED, OUTPUT);

    setupBLE();

    Serial.println("[BOOT] Ready. Control via BLE — open quad_tuner.html in Chrome.");
    Serial.println("[BOOT] Mission profile: SPRINT → HOLD → PUNCH → CUT");
}

void loop() {
    runMissionLoop();
    delay(20);
}
