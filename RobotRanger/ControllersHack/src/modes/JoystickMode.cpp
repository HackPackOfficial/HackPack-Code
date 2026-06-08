#include "modes_headers/JoystickMode.h"

/*
************************************************************************************
* The following is the implementation file for the manual pitch mode. The header file contains
* the declarations for the class and the functions as well as mode description.
************************************************************************************
*/

void JoystickMode::enter()
{
    resetPositions();
    pinMode(JSswitch, INPUT_PULLUP);
    pinMode(JSpinX, INPUT);
    pinMode(JSpinY, INPUT);
    SERIAL_PRINTLN("Joystick Mode entered");
}

void JoystickMode::exit()
{
    SERIAL_PRINTLN("Joystick Mode exited");
}

void JoystickMode::runStateMachine()
{
    moveServos();
    delay(SMOOTHING);
}

const char *JoystickMode::name()
{
    return "Manual_Pitch_Mode";
}

void JoystickMode::setColor(CRGB color)
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

void JoystickMode::moveServos()
{
    int16_t rawX = analogRead(JSpinX);
    int16_t rawY = analogRead(JSpinY);
    int16_t yawAngleValue =  map(rawX, 0, 1023, startPWM, endPWM);
    int16_t pitchAngleValue = map(rawY, 0, 1023, endPWM, startPWM); // Reversed mapping for pitch
    robot.yawServo.writeMicroseconds(yawAngleValue); 
    robot.pitchServo.writeMicroseconds(pitchAngleValue);

    if (!digitalRead(JSswitch))
        launchRoutine(robot.waitTime);
}