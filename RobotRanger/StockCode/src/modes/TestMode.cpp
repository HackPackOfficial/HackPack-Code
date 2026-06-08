#include "modes_headers/TestMode.h"

/*
************************************************************************************
* The following is the implementation file for the testing mode. This is intentionally left
* out of the main.cpp mode manager, as it is a testing space for the Crunchlabs PDEs or curious Hack Pack users.
* You can access this mode by appending it to the mode manager array.
************************************************************************************
*/

void TestMode::enter()
{
    SERIAL_PRINTLN("TestMode entered");
    setColor(CRGB(20, 0, 160)); // Purple
    resetPositions();
}

void TestMode::exit()
{
    // SERIAL_PRINTLN("TestMode exited");
}

void TestMode::runStateMachine()
{
    if (robot.currButton == 1)
    {
        testRoutine3();
    }
    testRoutine4();
}

const char *TestMode::name()
{
    return "Test_Mode";
}

void TestMode::testRoutine()
{
    if (Serial.available() > 0)
    {
        // Read the incoming distance
        float distance = Serial.parseFloat();
        // Clear any remaining input
        while (Serial.available())
        {
            Serial.read();
        }

        // Calculate the launch angle for this distance
        int angle = calculateLaunchAngle(distance);
        if (angle > 0)
        {
            SERIAL_PRINT("Shooting at distance: ");
            SERIAL_PRINT(distance);
            SERIAL_PRINT("mm with angle: ");
            SERIAL_PRINTLN(angle);

            // Move to the calculated angle and launch
            robot.pitchServo.write(angle);
            delay(100);
            launchRoutine(0);
        }
    }
}

void TestMode::testRoutine2()
{
    robot.ranger.readSingle(true);
    robot.data = robot.ranger.ranging_data;
    int distance = robot.data.range_mm;
    float offset = 0;

    if (robot.data.peak_signal_count_rate_MCPS >= SIGNAL_STRENGTH)
    {
        SERIAL_PRINTLN(distance);
        SERIAL_PRINTLN(robot.data.peak_signal_count_rate_MCPS);
        if (distance <= 50 && distance >= 25 && (robot.data.peak_signal_count_rate_MCPS <= 30 || robot.data.peak_signal_count_rate_MCPS >= 100))
        {
            distance = 0;
        }
        else if (distance <= 25)
        {
            offset = 5;
            distance = distance + offset;
        }
        else
        {
            offset = map(distance, 25, MAXD, 0, 10);
            distance = distance - offset;
        }
        SERIAL_PRINTLN(distance);
        delay(250);
    }
}

void TestMode::testRoutine3()
{
    robot.ranger.readSingle(true);
    robot.data = robot.ranger.ranging_data;
    SERIAL_PRINTLN(robot.data.range_mm);
}

void TestMode::testRoutine4()
{
    SERIAL_PRINT("Pot Value: ");
    SERIAL_PRINTLN(analogRead(POT_PIN));
}

void TestMode::setColor(CRGB color)
{
    robot.leds[0] = color;
    FastLED.show();
}