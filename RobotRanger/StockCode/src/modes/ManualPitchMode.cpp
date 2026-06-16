#include "modes_headers/ManualPitchMode.h"

/*
************************************************************************************
* The following is the implementation file for the manual pitch mode. The header file contains
* the declarations for the class and the functions as well as mode description.
************************************************************************************
*/

void ManualPitchMode::enter()
{
    setColor(CRGB(
        128,
        0,
        128
    )); // Purple for Pitch
    initialPotValue = analogRead(POT_PIN);
    lastPitchAngle = startPWM; // Start position
    robot.pitchServo.writeMicroseconds(lastPitchAngle);
    SERIAL_PRINTLN("ManualPitchMode entered");
}

void ManualPitchMode::exit()
{
    // Don't reset positions when exiting, keep the last pitch position
    SERIAL_PRINTLN("ManualPitchMode exited");
}

void ManualPitchMode::runStateMachine()
{
    manualPitch();
    delay(SMOOTHING);
}

const char *ManualPitchMode::name()
{
    return "Manual_Pitch_Mode";
}

void ManualPitchMode::setColor(CRGB color)
{
    robot.leds[0] = color;
    FastLED.show();
}

/*
************************************************************************************
* The following function is used to manually control the pitch servo.
* It reads the potentiometer value and maps it to the pitch range.
* It then moves the servo to the new position.
************************************************************************************
*/

void ManualPitchMode::manualPitch()
{
    int rawValue = analogRead(POT_PIN);

    // Only move if potentiometer has been touched (changed from initial value)
    if (abs(rawValue - initialPotValue) > 10)
    {
        int16_t maxTip = conversionPitchMicro(180 - robot.TILT_ADJUST);
        int16_t angleValue = map(rawValue, 0, 1023, maxTip, startPWM); // Reversed mapping for pitch
        robot.pitchServo.writeMicroseconds(angleValue);
        lastPitchAngle = angleValue; // Update lock position
    }

    if (robot.currButton == 1)
        launchRoutine(robot.waitTime);
}