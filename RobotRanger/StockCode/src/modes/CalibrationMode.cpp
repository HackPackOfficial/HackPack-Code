#include "modes_headers/CalibrationMode.h"

/*
************************************************************************************
* The following is the implementation file for the calibration mode. The header file contains
* the declarations for the class and the functions as well as mode description.
************************************************************************************
*/

void CalibrationMode::enter()
{
    SERIAL_PRINTLN("Calibration Mode entered");
    resetPositions();
    calibrationState = CALIBRATION_IDLE;
    setColor(CRGB(
        225,
        225,
        225
    )); // Gray
}

void CalibrationMode::exit()
{
    SERIAL_PRINTLN("CalibrationMode exited");
}

void CalibrationMode::runStateMachine()
{
    switch (calibrationState)
    {
    case CALIBRATION_IDLE:
        if (robot.calibrating)
        {
            calibrationState = savedCalibrationState;
        }
        else
        {
            if (robot.currButton == 1)
            {
                calibrationState = CALIBRATION_SELECT;
                robot.currButton = 0;
                delay(200);
            }
            break;
        }
    case CALIBRATION_SELECT:
    {
        // Read twice to allow ADC to settle after potential button reads on adjacent pin
        analogRead(POT_PIN);  // Dummy read to settle ADC
        delayMicroseconds(100);  // Small delay for ADC settling
        int16_t analogValue = map(analogRead(POT_PIN), 0, 1030, 0, 3);
        if (analogValue == 0)
        {
            updateBlinkLED(CRGB(
                150, 
                0, 
                250
            )); // Purple for Reset
        }
        else if (analogValue == 1)
        {
            updateBlinkLED(CRGB(
                225,
                225,
                225
            )); // Grey for Stage 1 (Tilt)
        }
        else if (analogValue == 2)
        {
            updateBlinkLED(CRGB(
                0,
                255,
                255
            )); // Cyan for Stage 2 (Yaw)
        }
        if (robot.currButton == 1)
        {
            calibrationState = static_cast<StatesCalibration_t>(analogValue);
            delay(200);
        }
    }
    break;
    case CALIBRATION_TILT:
        setColor(CRGB(
            225,
            225,
            225
        ));
        robot.calibrating = false;
        if (robot.currButton == 1)
        {
            calibrationRoutinePitch(); // Tilt offset calibration
        }
        break;
    case CALIBRATION_YAW:
        setColor(CRGB(
            0,
            255,
            255
        ));
        robot.calibrating = false;
        if (robot.currButton == 1)
        {
            calibrationRoutineYaw(); // Yaw offset calibration
        }
        break;
    case CALIBRATION_RESET:
        setColor(CRGB(
            150,
            0,
            250
        ));
        if (robot.currButton == 1)
        {
            resetCalibration();
            delay(200);
            robot.pitchServo.writeMicroseconds(endPWM);
            delay(500);
            robot.pitchServo.writeMicroseconds(startPWM);
        }
        break;
    }
}

const char *CalibrationMode::name()
{
    return "Calibration_Mode";
}

void CalibrationMode::setColor(CRGB color)
{
    robot.leds[0] = color;
    FastLED.show();
}

/*
************************************************************************************
* The following functions are used to calibrate the pitch and yaw offsets respectively.
* It reads the potentiometer value and maps it to the relevant adjustment range.
* It then saves the values to the EEPROM and sets the calibration flag to false to indicate that the calibration is complete.
************************************************************************************
*/

void CalibrationMode::calibrationRoutinePitch()
{
    // Map potentiometer to tilt adjustment range
    // Read twice to allow ADC to settle after potential button reads on adjacent pin
    SERIAL_PRINTLN(analogRead(POT_PIN));  // Dummy read to settle ADC
    delayMicroseconds(250);  // Small delay for ADC settling
    float potValue = analogRead(POT_PIN);  // Actual reading
    SERIAL_PRINTLN(potValue);
    robot.TILT_ADJUST = floatMap(potValue, 23.0f, 1000.0f, TILT_ADJUST_MIN, TILT_ADJUST_MAX);
    SERIAL_PRINTLN(robot.TILT_ADJUST);

    // Save to EEPROM
    EEPROM.put(tiltAddress, robot.TILT_ADJUST);
    EEPROM.put(calibrateAddressTilt, false);

    robot.calibrating = true;
    robot.runningTiltCalibration = true;
    savedCalibrationState = CALIBRATION_TILT;
}

void CalibrationMode::calibrationRoutineYaw()
{
    // Yaw offset calibration
    // Read twice to allow ADC to settle after potential button reads on adjacent pin
    analogRead(POT_PIN);  // Dummy read to settle ADC
    delayMicroseconds(100);  // Small delay for ADC settling
    robot.YAW_OFFSET_MIN = map(analogRead(POT_PIN), 23, 1000, YAW_OFFSET_MIN_BOUND, YAW_OFFSET_MAX_BOUND);
    robot.YAW_OFFSET_MAX = robot.YAW_OFFSET_MIN + robot.YAW_EXTRA;

    // Save to EEPROM
    EEPROM.put(yawOffsetAddress, robot.YAW_OFFSET_MIN);
    EEPROM.put(calibrateAddressYaw, false);

    robot.calibrating = true;
    robot.runningYawCalibration = true;
    savedCalibrationState = CALIBRATION_YAW;
}

/*
************************************************************************************
* The following function is used to reset the calibration, restoring "factory settings" or default values.
* It sets the calibration flags to true to indicate that the calibration is incomplete.
************************************************************************************
*/

void CalibrationMode::resetCalibration()
{
    robot.defaultOffsets();
    EEPROM.put(calibrateAddressTilt, true);
    EEPROM.put(calibrateAddressYaw, true);
    savedCalibrationState = CALIBRATION_RESET;
}


/*
************************************************************************************
* The following function is used to blink the LED.
* It sets the LED color to the relevant color and blinks it on and off. 
* This is used to indicate the calibration state when selecting between stages.
************************************************************************************
*/

void CalibrationMode::blinkLED(CRGB color)
{
    // This function is kept for compatibility but now uses non-blocking logic
    updateBlinkLED(color);
}

void CalibrationMode::updateBlinkLED(CRGB color)
{
    uint32_t currentTime = millis();

    // Check if it's time to toggle the LED
    if (currentTime - lastBlinkTime >= BLINK_INTERVAL)
    {
        ledOn = !ledOn; // Toggle LED state
        lastBlinkTime = currentTime;

        if (ledOn)
        {
            setColor(color);
        }
        else
        {
            FastLED.clear();
            FastLED.show();
        }
    }
}
