/**
 * PowerLink basic example
 *
 * Demonstrates connecting to the CH32V003 power controller, reading button
 * states, and using the callback and polling APIs.
 */

#include <Arduino.h>
#include <PowerLink.h>

PowerLink power;  // default pins: RX=2, TX=8, 9600 baud

void onButtons(uint8_t state, uint8_t prev) {
    Serial.printf("State changed: 0x%02X -> 0x%02X\n", prev, state);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Connecting to power controller...");

    power.onStateChange(onButtons);
    power.begin();     // blocks until handshake completes

    Serial.println("Connected.");

    // Optionally change the hard power-off hold time (default 5000ms on CH32 side).
    // power.setHardOffTime(3000);
}

void loop() {
    power.update();

    // Polling API — check individual buttons
    if (power.buttonPressed(PowerLink::BTN_0)) {
        Serial.println("Front button 0 pressed");
    }
    if (power.buttonReleased(PowerLink::BTN_0)) {
        Serial.println("Front button 0 released");
    }

    // Direct state read
    if (power.stateChanged()) {
        uint8_t s = power.getState();
        Serial.printf("Power: %d  Btn0: %d  Btn1: %d  Btn2: %d\n",
                       power.isPressed(PowerLink::BTN_POWER),
                       power.isPressed(PowerLink::BTN_0),
                       power.isPressed(PowerLink::BTN_1),
                       power.isPressed(PowerLink::BTN_2));
    }
}
