#include "BLSTR.h"

void stopMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
}

void setMotorSpeed(int speedVal1, int speedVal2) {
  speedVal1 = constrain(speedVal1, 0, 255);
  speedVal2 = constrain(speedVal2, 0, 255);  
  if (!batteryWakeMode || !flywheelsEnabled) {
    stopMotors();
    return;
  }

  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, HIGH);
  analogWrite(M1_PWM, speedVal1);
  analogWrite(M2_PWM, speedVal2);
}

void startFlywheels() {
  if (!batteryWakeMode) return;

  flywheelsEnabled = true;
  spinup = false;
  startmillis = millis();
}

void stopFlywheels() {
  clearM1TriggerBoost();
  flywheelsEnabled = false;
  spinup = false;
  stopMotors();
}

void toggleFlywheels() {
  if (flywheelsEnabled) {
    stopFlywheels();
  } else {
    startFlywheels();
  }
}

void shutdownBlaster() {
  saveLifetimeShots(true);

  // Make everything safe first
  stopFlywheels();

  pixels.clear();
  pixels.show();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(28, 28);
  display.print(F("POWERING OFF"));
  display.display();

  delay(300);

  // Release battery/main-board latch.
  digitalWrite(HOLD_PIN, LOW);

  // Normally the Nano loses power here.
  // If execution continues, USB must still be powering it.
  delay(1000);

  // We survived releasing the battery latch -> enter USB SAFE mode.
  batteryWakeMode = false;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(32, 12);
  display.print(F("USB SAFE MODE"));

  display.setCursor(17, 28);
  display.print(F("Charging / Program"));

  display.display();
}

void handlePowerButton() {
  static bool pendingSingleTap = false;
  static unsigned long firstTapRelease = 0;
  bool wakeBtnState = digitalRead(WAKE_BTN);
  unsigned long now = millis();

  // Ignore the initial wake press until the button has been released once.
  if (!powerButtonArmed) {
    if (wakeBtnState == LOW) {
      powerButtonArmed = true;
      lastWakeBtnState = LOW;
      powerPressHandled = false;
    }
    return;
  }

  // USB/programming mode must never arm the flywheels.
  if (!batteryWakeMode) {
    pendingSingleTap = false;
    lastWakeBtnState = wakeBtnState;
    return;
  }

  // Start timing a new press.
  if (wakeBtnState == HIGH && lastWakeBtnState == LOW) {
    wakeBtnHighStart = now;
    powerPressHandled = false;
  }

  // A long press always shuts the blaster down, from IDLE or ARMED.
  if (wakeBtnState == HIGH && !powerPressHandled &&
      now - wakeBtnHighStart >= POWER_BUTTON_LONG_PRESS_MS) {
    pendingSingleTap = false;
    Serial.println(F("POWER: long press - shutting down"));
    powerPressHandled = true;
    shutdownBlaster();
    return;
  }

  // A short press is recognized on release.
  if (wakeBtnState == LOW && lastWakeBtnState == HIGH && !powerPressHandled) {
    unsigned long pressDuration = now - wakeBtnHighStart;
    if (pressDuration >= POWER_BUTTON_DEBOUNCE_MS) {
      if (pendingSingleTap && now - firstTapRelease <= POWER_BUTTON_DOUBLE_TAP_MS) {
        pendingSingleTap = false;
        activeReticleStyle = (activeReticleStyle % 5) + 1;
        Serial.print(F("RETICLE: "));
        Serial.println(activeReticleStyle);
      } else {
        pendingSingleTap = true;
        firstTapRelease = now;
      }
    }
    powerPressHandled = true;
  }

  // Commit a single tap only after the double-tap window expires.
  if (pendingSingleTap && wakeBtnState == LOW &&
      now - firstTapRelease > POWER_BUTTON_DOUBLE_TAP_MS) {
    pendingSingleTap = false;
    toggleFlywheels();
    Serial.println(flywheelsEnabled ? F("POWER: ARMED") : F("POWER: IDLE"));
  }

  if (wakeBtnState == LOW && lastWakeBtnState == LOW) {
    powerPressHandled = false;
  }

  lastWakeBtnState = wakeBtnState;
}

void handleUSBModeButton() {
  static bool lastButtonState = LOW;
  static unsigned long pressStart = 0;

  bool buttonState = digitalRead(WAKE_BTN);

  // Rising edge: physical button pressed
  if (buttonState == HIGH && lastButtonState == LOW) {
    pressStart = millis();
  }

  // Small debounce: button must remain HIGH for 40 ms
  if (buttonState == HIGH &&
      lastButtonState == HIGH &&
      millis() - pressStart >= 40) {

    // Latch the main battery/buck power BEFORE the button is released
    digitalWrite(HOLD_PIN, HIGH);

    // Give buck a moment to come up
    delay(100);

    // We are now in normal battery-powered mode
    batteryWakeMode = true;

    // Start in IDLE / flywheels disabled
    flywheelsEnabled = false;
    stopFlywheels();

    // Wait for release so this same press isn't interpreted
    // by the normal power-button state machine.
    while (digitalRead(WAKE_BTN) == HIGH) {
      delay(5);
    }

    // Reset whatever button state your normal handler uses here
    lastButtonState = LOW;

    // Show the same safety warnings used during a normal battery startup.
    showStartupSafetyWarnings();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();

    delay(100);
    return;
  }

  lastButtonState = buttonState;
}

