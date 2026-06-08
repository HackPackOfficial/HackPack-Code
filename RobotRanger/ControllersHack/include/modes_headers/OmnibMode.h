#pragma once
#include "Mode.h"

/*
************************************************************************************
* The following is the header file for the manual pitch mode,
* which is responsible for manually controlling the PITCH of the robot with the potentiometer.
************************************************************************************
*/

// bunched group of 6 wires, with S3_G being the yellow wire, plug in sequentially from left to right
#define S1_V 4     
#define S1_G 6     
#define S2_V 7
#define S2_G 8
#define S3_V 10
#define S3_G 12

#define S4_V A1
#define S4_G A2

class OmnibMode : public Mode
{
public:
    void enter() override;
    void exit() override;
    void runStateMachine() override;
    const char *name() override;

private:
    void setColor(CRGB color) override;

    void setupPins(); 
    int interpretRF(); 
    void relevantAction(int);
    void moveServo(bool, Servo*, Servo* = nullptr, bool = false);
    
    const int16_t START_INCREMENT = 10;
    int32_t currentIncrement = START_INCREMENT;
};