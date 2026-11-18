// Set to 1 temporarily to print estimated free SRAM once per second.
#define BLSTR_SRAM_DIAGNOSTICS 0

#if BLSTR_SRAM_DIAGNOSTICS
extern char __heap_start, *__brkval;
static int blstrFreeSRAM() {
  char stackTop;
  return (int)(&stackTop - (__brkval ? __brkval : &__heap_start));
}
static void blstrPrintFreeSRAM() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 1000UL) return;
  lastPrint = millis();
  Serial.print(F("Free SRAM: "));
  Serial.println(blstrFreeSRAM());
}
#endif

#include "BLSTR.h"

void setup() {
  Serial.begin(115200);
  delay(50);

  randomSeed(micros() ^ analogRead(A7));

  pinMode(HOLD_PIN, OUTPUT);
  pinMode(WAKE_BTN, INPUT);   // external pulldown on PCB
  delay(50);                  // let D8 settle

  batteryWakeMode = digitalRead(WAKE_BTN) == HIGH;

  // Only latch the main power board after a real physical-button wake.
  // In USB SAFE mode HOLD stays LOW, so unplugging USB powers the Nano off
  // instead of leaving it running from the battery/buck in an ambiguous state.
  digitalWrite(HOLD_PIN, batteryWakeMode ? HIGH : LOW);

  // Safe defaults immediately
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);

  stopMotors();

  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  Serial.println(F("BLSTR Main Code. Version 1.0.0 ------------ 9/16/2026"));

  loadLifetimeShots();

  Serial.print(F("Startup mode: "));
  Serial.println(batteryWakeMode ? F("BATTERY / IDLE") : F("USB SAFE"));

  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(V_SENS, INPUT);

  // Let the 5V rail settle
  delay(800);

  // Try OLED before doing anything else noisy
  if (!initOLED()) {
    Serial.println(F("OLED failed"));
    while (1) {
      delay(100);
    }
  }

  showHackPackBootAnimation();

  showStartupSafetyWarnings();

  // Hall sensor after display is stable
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onHallPulse, RISING);

  // Servo last
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  if (batteryWakeMode) {
    servoWriteAngle(SERVO_REST_ANGLE, 300);
  }

  startmillis = millis();
  lastRPMMillis = millis();
}

void loop() {
#if BLSTR_SRAM_DIAGNOSTICS
  blstrPrintFreeSRAM();
#endif
  handlePowerButton();

  if (!batteryWakeMode) {
    stopFlywheels();

    // Allow physical power button to leave USB SAFE mode
    handleUSBModeButton();

    // If the button just switched us into battery mode,
    // leave this USB branch immediately.
    if (batteryWakeMode) {
      return;
    }

    updateBatteryCache();

    refreshOLEDIfDue();

    return;
  }

  updateBatteryCache();

  spinMotors();
}
