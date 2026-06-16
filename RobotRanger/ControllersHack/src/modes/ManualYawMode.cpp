#include "modes_headers/ManualYawMode.h"

/*
************************************************************************************
* The following is the implementation file for the manual yaw mode. The header file contains
* the declarations for the class and the functions as well as mode description.
************************************************************************************
*/

void ManualYawMode::enter()
{
    setColor(CRGB(
        0,
        0,
        255
    )); // Blue for Yaw
    initialPotValue = analogRead(POT_PIN);
    lastYawAngle = visualYawCenterPWM; // Center position
    robot.yawServo.writeMicroseconds(lastYawAngle);
    SERIAL_PRINTLN("ManualYawMode entered");
}

void ManualYawMode::exit()
{
    // Don't reset positions when exiting, keep the last yaw position
    SERIAL_PRINTLN("ManualYawMode exited");
}

void ManualYawMode::runStateMachine()
{
    manualYaw();
    delay(SMOOTHING);
}

const char *ManualYawMode::name()
{
    return "Manual_Yaw_Mode";
}

void ManualYawMode::setColor(CRGB color)
{
    robot.leds[0] = color;
    FastLED.show();
}

/*
************************************************************************************
* The following function is used to manually control the yaw servo.
* It reads the potentiometer value and maps it to the yaw range.
* It then moves the servo to the new position.
************************************************************************************
*/

void ManualYawMode::manualYaw()
{
    int16_t rawValue = analogRead(POT_PIN);

    // Only move if potentiometer has been touched (changed from initial value)
    if (abs(rawValue - initialPotValue) > 10)
    { // Small threshold to account for noise
        int16_t angleValue = map(rawValue, 0, 1023, startPWM, endPWM);
        robot.yawServo.writeMicroseconds(angleValue);
        lastYawAngle = angleValue; // Update lock position
    }

    if (robot.currButton == 1)
        launchRoutine(robot.waitTime);
}