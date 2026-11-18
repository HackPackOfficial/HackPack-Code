#include "BLSTR.h"

uint8_t chargePercent = 0;
bool chargeActive = false;
static bool chargeTriggerWasDown = false;
static unsigned long chargeStartedAt = 0;

// M1 boost deadline. Kept local to the motor controller to save SRAM.
static unsigned long m1BoostUntil = 0;

void requestM1TriggerBoost() {
  if (!batteryWakeMode || !flywheelsEnabled || !spinup) return;
  m1BoostUntil = millis() + M1_TRIGGER_BOOST_MS;
  // Apply immediately, even before the next motor control update.
  analogWrite(M1_PWM, 255);
}

void clearM1TriggerBoost() {
  m1BoostUntil = 0;
}


void onHallPulse() {
  static unsigned long lastPulseUs = 0;
  unsigned long now = micros();
  if (now - lastPulseUs < 300) return;
  lastPulseUs = now;
  hallPulseCount++;
}

float motor2PwmToTargetRPM(uint8_t pwm) {
  if (pwm >= 255) return RPM_AT_PWM_255;
  if (pwm <= 150) return RPM_AT_PWM_150 * ((float)pwm / 150.0);
  if (pwm <= 180) {
    return RPM_AT_PWM_150 + (RPM_AT_PWM_180 - RPM_AT_PWM_150) * ((float)(pwm - 150) / 30.0);
  }
  if (pwm <= 220) {
    return RPM_AT_PWM_180 + (RPM_AT_PWM_220 - RPM_AT_PWM_180) * ((float)(pwm - 180) / 40.0);
  }
  return RPM_AT_PWM_220 + (RPM_AT_PWM_255 - RPM_AT_PWM_220) * ((float)(pwm - 220) / 35.0);
}

int shadowM2CorrectionToM1(int m2Command) {
  int m1Base = max((int)MOTOR1_SPEED, (int)MOTOR1_MIN_RUNNING_PWM);
  int m2Base = max((int)MOTOR2_SPEED, (int)MOTOR2_MIN_RUNNING_PWM);

  float correctionRatio = (float)m2Command / (float)m2Base;
  int m1Command = (int)(m1Base * correctionRatio + 0.5);
  return constrain(m1Command, (int)MOTOR1_MIN_RUNNING_PWM, 255);
}

void updateFlywheelClosedLoop() {
  if (!flywheelsEnabled || !spinup) return;

  if (MOTOR2_SPEED >= 255) {
    commandedM2PWM = 255;
    commandedM1PWM = shadowM2CorrectionToM1(commandedM2PWM);
    rpmIntegral = 0.0;
    lastRpmError = 0.0;
    return;
  }

  unsigned long now = millis();
  if (now - lastControlMillis < RPM_CONTROL_MS) return;

  float dt = (now - lastControlMillis) / 1000.0;
  lastControlMillis = now;
  float targetRPM = motor2PwmToTargetRPM(MOTOR2_SPEED);
  float error = targetRPM - rpm;

  rpmIntegral += error * dt;
  rpmIntegral = constrain(rpmIntegral, -RPM_INTEGRAL_LIMIT, RPM_INTEGRAL_LIMIT);

  float derivative = (dt > 0.0) ? (error - lastRpmError) / dt : 0.0;
  lastRpmError = error;
  float correction = RPM_KP * error + RPM_KI * rpmIntegral + RPM_KD * derivative;

  commandedM2PWM = constrain((int)(MOTOR2_SPEED + correction + 0.5), 0, 255);
  commandedM1PWM = shadowM2CorrectionToM1(commandedM2PWM);
}

void spinMotors(){
  // Charge begins at the normal running speeds and raises BOTH motors
  // progressively toward full PWM while the trigger is held.
  bool triggerPressed = (digitalRead(TRIGGER_PIN) == LOW);
  unsigned long now = millis();
  if (!flywheelsEnabled || !spinup) {
    chargeActive = false;
    chargePercent = 0;
    chargeTriggerWasDown = triggerPressed; // never fire a pre-spinup press
  } else if (triggerPressed) {
    if (!chargeTriggerWasDown) {
      chargeStartedAt = now;
      chargeActive = true;
    }
    chargeTriggerWasDown = true;
    if (chargeActive) {
      unsigned long elapsed = now - chargeStartedAt;
      chargePercent = (uint8_t)min(100UL, elapsed * 100UL / CHARGE_FULL_MS);
    }
  }

  int speedVal1 = 0;
  int speedVal2 = 0;

  if (flywheelsEnabled) {
    int m1Base = max((int)MOTOR1_SPEED, (int)MOTOR1_MIN_RUNNING_PWM);

    if (!spinup) {
      int ramp = min(int((millis() - startmillis) / 10), 255);
      speedVal1 = constrain(ramp, (int)MOTOR1_MIN_RUNNING_PWM, m1Base);
      speedVal2 = min(ramp, (int)MOTOR2_SPEED);

      if (ramp >= m1Base && ramp >= MOTOR2_SPEED) {
        spinup = true;
        commandedM1PWM = m1Base;
        commandedM2PWM = MOTOR2_SPEED;
        rpmIntegral = 0.0;
        lastRpmError = 0.0;
        lastControlMillis = millis();
      }
    } else {
      updateFlywheelClosedLoop();
      speedVal1 = commandedM1PWM;
      speedVal2 = commandedM2PWM;
      // Charge overrides the normal closed-loop commands for both motors.
      // Using elapsed time directly gives single-PWM-step resolution.
      if (chargeActive && triggerPressed) {
        unsigned long elapsed = min(millis() - chargeStartedAt, CHARGE_FULL_MS);
        speedVal1 += (int)(((long)(255 - speedVal1) * elapsed) / CHARGE_FULL_MS);
        speedVal2 += (int)(((long)(255 - speedVal2) * elapsed) / CHARGE_FULL_MS);
      } else if ((long)(m1BoostUntil - millis()) > 0) {
        speedVal1 = 255; // existing post-shot M1 recovery behavior
      }
    }

    setMotorSpeed(speedVal1, speedVal2);
  } else {
    clearM1TriggerBoost();
    commandedM1PWM = 0;
    commandedM2PWM = 0;
    rpmIntegral = 0.0;
    stopMotors();
  }

  // ---------- RPM calculation + display update ----------
  now = millis();
  if (now - lastRPMMillis >= 200) {
    noInterrupts();
    unsigned long pulses = hallPulseCount;
    hallPulseCount = 0;
    interrupts();

    float dt = (now - lastRPMMillis) / 1000.0;
    float newRPM = (pulses / (float)pulsesPerRev) * (60.0 / dt);

    rpm = rpm * 0.7 + newRPM * 0.3;
    // rpm values over 60,000 are considerered errant, and removed from the calculation.
    if (rpm > 60000) rpm = 0;

    verifyShotFromRPM(rpm);

    Serial.print(F("PWM: "));
    Serial.print(speedVal2);
    Serial.print(F("   RPM: "));
    Serial.print(rpm);
    Serial.print(F("   Battery: "));
    Serial.print(cachedBatteryVoltage, 2);
    Serial.print(F("V "));
    Serial.print(cachedBatteryPercent);
    Serial.println(F("%"));


    lastRPMMillis = now;
  }

  refreshOLEDIfDue();

  // Fire exactly one disc when the charged trigger is released.
  if (!flywheelsEnabled || !spinup || triggerPressed) return;
  if (chargeTriggerWasDown) {
    chargeTriggerWasDown = false;
    bool shouldFire = chargeActive && (millis() - chargeStartedAt >= 50UL);
    chargeActive = false;
    chargePercent = 0;
    if (shouldFire && flywheelsEnabled && spinup) {
      requestM1TriggerBoost(); // retain existing M1 recovery around the shot
      armShotVerification();
      fireFullAutoCycle();
    }
  }
}
