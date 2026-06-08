#include "modes_headers/IRMode.h"
#include <IRremote.hpp>

/*
************************************************************************************
* The following is the implementation file for the manual pitch mode. The header file contains
* the declarations for the class and the functions as well as mode description.
************************************************************************************
*/

void IRMode::enter()
{
    resetPositions();
    IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
    SERIAL_PRINTLN("IR Mode Entered");
}

void IRMode::exit()
{
    SERIAL_PRINTLN("IR Mode Exited");
}

void IRMode::runStateMachine()
{
    interpretIR();
}

const char *IRMode::name()
{
    return "IR_Mode";
}

void IRMode::setColor(CRGB color)
{
    robot.leds[0] = color;
    FastLED.show();
}

void IRMode::interpretIR()
{
    if (IrReceiver.decode())
    {
        if (IrReceiver.decodedIRData.protocol == UNKNOWN)
        {
            SERIAL_PRINTLN("we are cooked");
        }

        IrReceiver.resume();
        currCommand = IrReceiver.decodedIRData.command;
        if(currCommand == lastCommand) {
            currentIncrement += RAMP_INCREMENT; 
        } else {
            currentIncrement = START_INCREMENT; 
        }
        switch (currCommand)
        {
        case up: // pitch up
            moveRelevantServo(true, &robot.pitchServo);
            break;

        case down: // pitch down
            moveRelevantServo(false, &robot.pitchServo);
            break;

        case left: // fast counterclockwise rotation
            moveRelevantServo(true, &robot.yawServo);
            break;

        case right: // fast clockwise rotation
            moveRelevantServo(false, &robot.yawServo);
            break;

        case ok: // firing routine
            launchRoutine(robot.waitTime);
            break;
        case star:
            // filler
            break;

        case hashtag:
            // filler
            break;

        default:
            break;
        }
        lastCommand = currCommand; 
    }
}

void IRMode::moveRelevantServo(bool dir, Servo* servo)
{
    int16_t currPosition = servo->readMicroseconds();
    int32_t increment = (dir) ? currentIncrement : -currentIncrement;
    int32_t nextPosition = currPosition + increment;
    
    if (nextPosition <= startPWM)
    {
        nextPosition = startPWM;
    }
    else if (nextPosition >= endPWM)
    {
        nextPosition = endPWM;
    }
    SERIAL_PRINTLN(nextPosition);
    servo->writeMicroseconds(nextPosition);
    delay(SMOOTHING/5);
}
