#pragma once
#include "Mode.h"

/*
************************************************************************************
* The following is the header file for the calibration mode, which is responsible for calibrating the robot's pitch and yaw offsets,
* if the default values are not accurate. These are primarly for auto mode. The relevant values are then saved to the EEPROM.
************************************************************************************
*/

// Calibration Values
#define TILT_ADJUST_MIN 0.0
#define TILT_ADJUST_MAX 35.0
#define YAW_OFFSET_MIN_BOUND 5.0
#define YAW_OFFSET_MAX_BOUND 25.0

// EEPROM Addresses - Make sure these are spaced properly to avoid overwriting each other.
#define tiltAddress 79          // float (4 bytes)
#define yawOffsetAddress 83     // int16_t (2 bytes, but spaced 4 just in case)
#define calibrateAddressTilt 87 // bool (1 byte)
#define calibrateAddressYaw 88  // bool (1 byte)

class CalibrationMode : public Mode
{
public:
    void enter() override;
    void exit() override;
    void runStateMachine() override;
    const char *name() override;

private:
    void setColor(CRGB color) override;

    enum StatesCalibration_t
    {
        CALIBRATION_RESET, // Reset calibration
        CALIBRATION_TILT,  // Tilt offset adjustment
        CALIBRATION_YAW,   // Yaw offset adjustment
        CALIBRATION_IDLE,
        CALIBRATION_SELECT // Select calibration type
    };

    StatesCalibration_t calibrationState = CALIBRATION_IDLE;
    StatesCalibration_t savedCalibrationState = CALIBRATION_IDLE;

    // Non-blocking LED blink variables
    bool ledOn = false;
    uint32_t lastBlinkTime = 0;
    const uint32_t BLINK_INTERVAL = 200; // 200ms on/off cycle

    void calibrationRoutinePitch(); // Tilt offset calibration
    void calibrationRoutineYaw();   // Yaw offset calibration
    void resetCalibration();
    void blinkLED(CRGB color);
    void updateBlinkLED(CRGB color);
};
