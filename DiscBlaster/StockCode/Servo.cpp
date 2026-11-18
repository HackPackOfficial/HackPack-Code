#include "BLSTR.h"

void servoPulseUs(int pulseUs) {
  pulseUs = constrain(pulseUs, 544, 2400);
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(20000 - pulseUs);
}

void servoWriteAngle(int angle, int durationMs) {
  angle = constrain(angle, 0, 180);
  int pulseUs = map(angle, 0, 180, 544, 2400);
  unsigned long start = millis();

  while (millis() - start < (unsigned long)durationMs) {
    servoPulseUs(pulseUs);
  }
}

void fireServoCycle(int pushMs, int returnMs) {
  servoWriteAngle(SERVO_FIRE_ANGLE, pushMs);

  // M1 was already boosted on trigger press. Keep it at full power
  // throughout the return stroke, then extend the boost for recovery.
  servoWriteAngle(SERVO_REST_ANGLE, returnMs);
  requestM1TriggerBoost();
}

void fireFullAutoCycle() {
  fireServoCycle(FULL_AUTO_PUSH_MS, FULL_AUTO_RETURN_MS);

  // Slower M1 settings need more time between discs to recover.
  // 255 = +0ms, 140 = +460ms.
  int m1ForDelay = constrain((int)MOTOR1_SPEED, (int)MOTOR1_MIN_RUNNING_PWM, 255);
  unsigned long extraRecoveryMs = (255 - m1ForDelay) * M1_RECOVERY_MS_PER_PWM;
  delay(extraRecoveryMs);
}

