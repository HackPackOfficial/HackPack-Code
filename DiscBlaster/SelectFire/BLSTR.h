#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include "config.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define M1_DIR 7
#define M1_PWM 10
#define M2_DIR 4
#define M2_PWM 5
#define TRIGGER_PIN 14
#define SERVO_PIN 6
#define IR_PIN 3
#define HALL_PIN 2
#define V_SENS A6
#define HOLD_PIN 12
#define WAKE_BTN 8

extern Adafruit_SSD1306 display;

extern const uint8_t HACK_PACK_LOGO[] PROGMEM;
extern const int HACK_PACK_LOGO_WIDTH;
extern const int HACK_PACK_LOGO_HEIGHT;
extern const int HACK_PACK_LOGO_X;
extern const int HACK_PACK_LOGO_Y;

extern long startmillis;
extern bool spinup;
extern long pressTime;
extern bool pressStart;
extern bool flywheelsEnabled;
extern float rpmIntegral;
extern float lastRpmError;
extern unsigned long lastControlMillis;
extern unsigned long lastVerifiedShotTime;
extern int commandedM1PWM;
extern int commandedM2PWM;
extern bool powerButtonArmed;
extern bool lastWakeBtnState;
extern unsigned long wakeBtnHighStart;
extern bool powerPressHandled;
extern volatile unsigned long hallPulseCount;
extern unsigned long lastRPMMillis;
extern float rpm;
extern uint32_t lifetimeShots;
extern uint16_t shotsSinceSave;
extern bool shotVerificationPending;
extern float shotPreRPM;
extern uint32_t shotSaveSequence;
extern int shotEEPROMCurrentSlot;
extern bool batteryWakeMode;
extern float cachedBatteryVoltage;
extern int cachedBatteryPercent;
extern unsigned long lastBatteryMillis;
extern unsigned long lastLedMillis;

void servoPulseUs(int pulseUs);
void servoWriteAngle(int angle, int durationMs);
void fireServoCycle(int pushMs, int returnMs);
void fireFullAutoCycle();
void stopMotors();
void setMotorSpeed(int speedVal1, int speedVal2);
void startFlywheels();
void stopFlywheels();
void toggleFlywheels();
void shutdownBlaster();
void handlePowerButton();
void handleUSBModeButton();
void onHallPulse();
float readBatteryVoltage();
int batteryPercent(float voltage);
void updateBatteryCache();
void updateSafeModeBatteryPixels();
void updateFrontCometPulse();
extern uint8_t activeReticleStyle;
int drawBaseReticle();
void drawBatteryWidget(int x, int y, int pct, float voltage);
void drawOLED(float rpmValue, int battPct, float battVoltage);
void refreshOLEDIfDue();
void i2cBusRecover();
bool initOLED();
void showHackPackBootAnimation();
void showHackPackCurtainAnimation();
void showHackPackDissolveAnimation();
void showHackPackCircularAnimation();
bool waitForWarningButton();
void showStartupSafetyWarnings();
uint16_t shotRecordChecksum(uint16_t magic, uint32_t sequence, uint32_t count);
void loadLifetimeShots();
void saveLifetimeShots(bool forceSave);
void armShotVerification();
void verifyShotFromRPM(float postShotRPM);
float motor2PwmToTargetRPM(uint8_t pwm);
int shadowM2CorrectionToM1(int m2Command);
void updateFlywheelClosedLoop();
void spinMotors();
void cycleFireMode();
extern uint8_t fireMode;
void requestM1TriggerBoost();
void clearM1TriggerBoost();
