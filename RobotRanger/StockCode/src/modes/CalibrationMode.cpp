#include "CalibrationMode.h"

void CalibrationMode::enter()
{
   SERIAL_PRINTLN("Calibration Mode entered");
    resetPositions();
    calibrationState = CALIBRATION_IDLE;
    setColor(CRGB(225, 225, 225)); // Gray
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
        if (robot.currButton == 1)
        {
            calibrationState = CALIBRATION_SELECT;
            robot.currButton = 0;
            delay(200);
        }
        if(robot.calibrating) {
            calibrationState = CALIBRATION_SELECT;
            robot.calibrating = false;
        }
        break;
    case CALIBRATION_SELECT:
    {
        int analogValue = map(analogRead(POT_PIN), 0, 1030, 0, 3);
        
        if (analogValue == 0)
        {
            updateBlinkLED(CRGB(150, 0, 250)); // Purple for Reset
        }
        else if (analogValue == 1)
        {
            updateBlinkLED(CRGB(225, 225, 225)); // Grey for Stage 1 (Tilt)
        }
        else if (analogValue == 2)
        {
            updateBlinkLED(CRGB(0, 255, 255)); // Cyan for Stage 2 (Yaw)
        }
        if (robot.currButton == 1)
        {
            calibrationState = static_cast<StatesCalibration_t>(analogValue);
            delay(200);
        }
    }
    break;
    case CALIBRATION_TILT:
        robot.calibrating = false;
        setColor(CRGB(225, 225, 225));
        if(robot.currButton == 1)
        {
            calibrationRoutine1(); // Tilt offset calibration
        }
        break;
    case CALIBRATION_YAW:
        setColor(CRGB(0, 255, 255));
        if(robot.currButton == 1)
        {
            calibrationRoutine2(); // Yaw offset calibration
        }
        break;
    case CALIBRATION_RESET:
        robot.calibrating = false;
        setColor(CRGB(150, 0, 250));
        if(robot.currButton == 1)
        {
            resetCalibration();
            delay(200);
            robot.pitchServo.writeMicroseconds(endPWM);
            delay(500);
            robot.pitchServo.writeMicroseconds(startPWM);
            calibrationState = CALIBRATION_SELECT;
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

void CalibrationMode::calibrationRoutine1()
{

    // Map potentiometer to tilt adjustment range
    robot.TILT_ADJUST = map(analogRead(POT_PIN), 0, 1023, TILT_ADJUST_MIN, TILT_ADJUST_MAX);
    setColor(CRGB(225, 225, 225));

    // Shoot at calibration distances
    calibrationShots(CALIBRATION_DISTANCE_1);
    calibrationShots(CALIBRATION_DISTANCE_2);

    // Save to EEPROM
    EEPROM.put(tiltAddress, robot.TILT_ADJUST);
    EEPROM.put(calibrateAddressTilt, false);

    calibrationState = CALIBRATION_SELECT;
}

void CalibrationMode::calibrationRoutine2()
{
    // Yaw offset calibration
    robot.OFFSET_MIN = map(analogRead(POT_PIN), 0, 1023, YAW_OFFSET_MIN, YAW_OFFSET_MAX);
    setColor(CRGB(0, 255, 255)); // Blue for yaw calibration

    // Save to EEPROM
    EEPROM.put(yawOffsetAddress, robot.OFFSET_MIN);
    EEPROM.put(calibrateAddressYaw, false);

    robot.calibrating = true;
    calibrationState = CALIBRATION_SELECT;
}

void CalibrationMode::calibrationShots(float distance)
{
    int tilt = calculateLaunchAngle(distance);
    robot.pitchServo.write(tilt + robot.TILT_ADJUST);
    launchRoutine(400, 300);
    delay(1000);
}

void CalibrationMode::blinkLED(CRGB color)
{
    // This function is kept for compatibility but now uses non-blocking logic
    updateBlinkLED(color);
}

void CalibrationMode::updateBlinkLED(CRGB color)
{
    unsigned long currentTime = millis();

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

void CalibrationMode::resetCalibration()
{
    defaultOffsets();
    robot.calibrating = false;
    EEPROM.put(calibrateAddressTilt, true);
    EEPROM.put(calibrateAddressYaw, true);
}