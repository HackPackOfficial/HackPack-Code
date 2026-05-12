#pragma once
#include "Mode.h"

class CalibrationMode : public Mode {
    public:
       void enter() override;
       void exit() override;
       void runStateMachine() override;
       const char* name() override;
       
    private:
        void setColor(CRGB color) override;

        enum StatesCalibration_t {
            CALIBRATION_RESET, // Reset calibration
            CALIBRATION_TILT,  // Tilt offset adjustment
            CALIBRATION_YAW,    // Yaw offset adjustment
            CALIBRATION_IDLE,
            CALIBRATION_SELECT // Select calibration type
        };

        StatesCalibration_t calibrationState = CALIBRATION_IDLE;
        
        // Calibration Values
        #define CALIBRATION_DISTANCE_1 120  // Close target
        #define CALIBRATION_DISTANCE_2 290  // Far target
        #define TILT_ADJUST_MIN 0.0
        #define TILT_ADJUST_MAX 15.0
        #define YAW_OFFSET_MIN 2.0
        #define YAW_OFFSET_MAX 28.0
        
        // EEPROM Addresses
        #define tiltAddress 79           // float (4 bytes) - only 2 bytes apart
        #define yawOffsetAddress 83      // uint32_t (4 bytes) - only 4 bytes apart
        #define calibrateAddressTilt 87  // bool (1 byte) - only 4 bytes apart
        #define calibrateAddressYaw 88   // bool (1 byte) - only 1 byte apart

        // Calibration Flags
        bool MAGIC_VALUE_TILT = false; //false = not calibrated, true = calibrated
        bool MAGIC_VALUE_YAW = false; //false = not calibrated, true = calibrated
        bool initiateCalibration = false;

        // Non-blocking LED blink variables
        bool ledOn = false;
        unsigned long lastBlinkTime = 0;
        const unsigned long BLINK_INTERVAL = 500; // 500ms on/off cycle

        void calibrationRoutine1();  // Tilt offset calibration
        void calibrationRoutine2();  // Yaw offset calibration
        void resetCalibration();
        void calibrationShots(float distance);
        void blinkLED(CRGB color);
        void updateBlinkLED(CRGB color);
        
};