#include "BLSTR.h"
#include <EEPROM.h>

unsigned long lastVerifiedShotTime = 0;

uint16_t shotRecordChecksum(uint16_t magic, uint32_t sequence, uint32_t count) {
  uint16_t checksum = 0xA5A5;
  checksum ^= magic;
  checksum ^= (uint16_t)(sequence & 0xFFFF);
  checksum ^= (uint16_t)(sequence >> 16);
  checksum ^= (uint16_t)(count & 0xFFFF);
  checksum ^= (uint16_t)(count >> 16);
  return checksum;
}

void loadLifetimeShots() {
  bool found = false;
  ShotRecord newest;
  int newestSlot = -1;

  for (int slot = 0; slot < SHOT_EEPROM_SLOTS; slot++) {
    ShotRecord record;
    EEPROM.get(slot * sizeof(ShotRecord), record);

    if (record.magic != SHOT_EEPROM_MAGIC) continue;
    if (record.checksum != shotRecordChecksum(record.magic, record.sequence, record.count)) continue;

    if (!found || (int32_t)(record.sequence - newest.sequence) > 0) {
      newest = record;
      newestSlot = slot;
      found = true;
    }
  }

  if (found) {
    lifetimeShots = newest.count;
    shotSaveSequence = newest.sequence;
    shotEEPROMCurrentSlot = newestSlot;
  } else {
    lifetimeShots = 0;
    shotSaveSequence = 0;
    shotEEPROMCurrentSlot = -1;
  }

  shotsSinceSave = 0;

  Serial.print(F("Lifetime shots loaded: "));
  Serial.println(lifetimeShots);
}

void saveLifetimeShots(bool forceSave) {
  if (!forceSave && shotsSinceSave < SHOTS_PER_EEPROM_SAVE) return;
  if (shotsSinceSave == 0 && shotEEPROMCurrentSlot >= 0) return;

  shotEEPROMCurrentSlot++;
  if (shotEEPROMCurrentSlot >= SHOT_EEPROM_SLOTS) shotEEPROMCurrentSlot = 0;

  shotSaveSequence++;

  ShotRecord record;
  record.magic = SHOT_EEPROM_MAGIC;
  record.sequence = shotSaveSequence;
  record.count = lifetimeShots;
  record.checksum = shotRecordChecksum(record.magic, record.sequence, record.count);

  int address = shotEEPROMCurrentSlot * sizeof(ShotRecord);
  EEPROM.put(address, record);
  shotsSinceSave = 0;
}

void armShotVerification() {
  shotPreRPM = rpm;
  shotVerificationPending = true;
}

void verifyShotFromRPM(float postShotRPM) {
  if (!shotVerificationPending) return;

  float rpmDrop = shotPreRPM - postShotRPM;
  shotVerificationPending = false;

  Serial.print(F("SHOT CHECK: pre="));
  Serial.print(shotPreRPM);
  Serial.print(F(" post="));
  Serial.print(postShotRPM);
  Serial.print(F(" drop="));
  Serial.print(rpmDrop);

  if (shotPreRPM >= SHOT_MIN_VALID_PRE_RPM && rpmDrop >= SHOT_RPM_DROP_THRESHOLD) {
    lifetimeShots++;
    shotsSinceSave++;

    lastVerifiedShotTime = millis();

    Serial.print(F(" VERIFIED  lifetime="));
    Serial.println(lifetimeShots);

    saveLifetimeShots(false);
  } else {
    Serial.println(F(" DRY / NOT COUNTED"));
  }
}

