#pragma once
#include "Mode.h"

/*
************************************************************************************
* The following is the header file for the manual pitch mode,
* which is responsible for manually controlling the PITCH of the robot with the potentiometer.
************************************************************************************
*/

#define JSswitch A0 
#define JSpinY A1
#define JSpinX A2

class JoystickMode : public Mode
{
public:
    void enter() override;
    void exit() override;
    void runStateMachine() override;
    const char *name() override;

private:
    void setColor(CRGB color) override;
    void moveServos(); 
};