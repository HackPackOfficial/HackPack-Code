#include "Controllers.h"
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>

void ControllersModeHack::enter()
{
    SERIAL_PRINTLN("Controllers Hack Entered");
    setColor(CRGB(200, 100, 0));
    resetPositions();
    //configureHack();
    controlScheme = CONTROLLER_JS;
}

void ControllersModeHack::exit()
{
    resetPositions();
}

void ControllersModeHack::runStateMachine()
{
    switch (controlScheme)
    {
    case CONTROLLER_IR:
        SERIAL_PRINTLN("IR");
        handleIR();

        break;
    case CONTROLLER_JS:
        digitalWrite(LED_BUILTIN, HIGH);
        handleJS();
     
        break;
    case CONTROLLER_RF:
        handleRF();
        SERIAL_PRINTLN("RF");
        break;
    case CONTROLLER_OTHER:
        handleOther(); // do whatever else you want if needed
        break;
    default:
        break;
    }
}

const char *ControllersModeHack::name()
{
    return "Controllers_Hack";
}

void ControllersModeHack::setColor(CRGB color)
{
    robot.leds[0] = color;
    FastLED.show();
}

void ControllersModeHack::configureHack()
{
    if (!digitalRead(SELECT_IR))
    {
        controlScheme = CONTROLLER_IR;
    }
    else if (!digitalRead(SELECT_JS))
    {
        controlScheme = CONTROLLER_JS;
    }
    else if (!digitalRead(SELECT_RF))
    {
        controlScheme = CONTROLLER_RF;
    }
    else
    {
        digitalWrite(LED_BUILTIN, HIGH); // signals that no controller is found
    }
}

void ControllersModeHack::handleIR()
{
    if (IrReceiver.decode())
    {
        IrReceiver.resume();
        switch (IrReceiver.decodedIRData.command)
        {
        case up: // pitch up
            robot.pitchServo.write(robot.pitchServo.read() + pitchIncrement);

            break;

        case down: // pitch down
            robot.pitchServo.write(robot.pitchServo.read() - pitchIncrement);
            break;

        case left: // rotate left
            robot.yawServo.write(robot.yawServo.read() - yawIncrement);
            break;

        case right: // rotate right
            robot.yawServo.write(robot.yawServo.read() + yawIncrement);
            break;

        case ok: // launching routine
            launchRoutine();
            break;

        case star:
            digitalWrite(LASER_PIN, HIGH);
            break;

        case hashtag:
            resetPositions();
            break;

            // can add cmds 0-9 here if we want to use them

        default:
            digitalWrite(LASER_PIN, LOW);
            break;
        }
    }
    delay(5);
}

void ControllersModeHack::handleJS()
{
    //int32_t x_value = analogRead(SELECT_RF);
    //int32_t y_value = analogRead(SELECT_JS);

    int32_t x_value = analogRead(A1);
    int32_t y_value = analogRead(A2);

    int yawValue = map(x_value, 0, 1023, endPWM, startPWM); // Reversed mapping for pitch
    robot.yawServo.writeMicroseconds(yawValue);
    int pitchValue = map(y_value, 0, 1023, endPWM, startPWM); // Reversed mapping for pitch
    robot.pitchServo.writeMicroseconds(pitchValue);

    if (!digitalRead(A0))
        launchRoutine();

    delay(5);
}

void ControllersModeHack::handleRF()
{
    /* code */
}

void ControllersModeHack::handleOther()
{
    /* some other way that you want to add */
}