#include "BLSTR.h"

float readBatteryVoltage() {
  long total = 0;

  // High impedance divider benefits from a dummy read + small settling delay.
  analogRead(V_SENS);
  delay(2);

  for (int i = 0; i < 20; i++) {
    total += analogRead(V_SENS);
    delay(2);
  }

  float raw = total / 20.0;
  float adcRef = batteryWakeMode ? ADC_REF_BUCK : ADC_REF_USB_SAFE;
  float pinVoltage = raw * adcRef / 1023.0;
  return pinVoltage * BATTERY_DIVIDER_RATIO;
}

int batteryPercent(float voltage) {
  float percent = (voltage - BAT_EMPTY) * 100.0 / (BAT_FULL - BAT_EMPTY);
  percent = constrain(percent, 0, 100);
  return (int)(percent + 0.5);
}

void updateBatteryCache() {
  // A6 is connected to the pre-PMOS battery divider, so it can be measured
  // even in USB SAFE mode while HOLD_PIN is LOW and the main buck is off.
  // readBatteryVoltage() selects the USB or buck ADC reference automatically.
  unsigned long now = millis();
  if (now - lastBatteryMillis < BATTERY_UPDATE_MS) return;
  lastBatteryMillis = now;

  cachedBatteryVoltage = readBatteryVoltage();
  cachedBatteryPercent = batteryPercent(cachedBatteryVoltage);
}

