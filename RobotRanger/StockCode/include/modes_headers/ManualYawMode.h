#pragma once
#include "Mode.h"

/*
************************************************************************************
* The following is the header file for the manual yaw mode,
* which is responsible for manually controlling the YAW of the robot with the potentiometer.
************************************************************************************
*/

class ManualYawMode : public Mode
{
public:
    void enter() override;
    void exit() override;
    void runStateMachine() override;
    const char *name() override;

private:
    void setColor(CRGB color) override;

    int16_t initialPotValue = 0;               // Store initial potentiometer value
    int16_t lastYawAngle = visualYawCenterPWM; // Store the last yaw angle

    void manualYaw();
};