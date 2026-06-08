#pragma once
#include "Mode.h"

/*
************************************************************************************
* The following is the header file for the manual pitch mode,
* which is responsible for manually controlling the PITCH of the robot with the potentiometer.
************************************************************************************
*/

// defines the specific command code for each button on the remote
#define left 0x8
#define right 0x5A
#define up 0x18
#define down 0x52
#define ok 0x1C
#define cmd1 0x45
#define cmd2 0x46
#define cmd3 0x47
#define cmd4 0x44
#define cmd5 0x40
#define cmd6 0x43
#define cmd7 0x7
#define cmd8 0x15
#define cmd9 0x9
#define cmd0 0x19
#define star 0x16
#define hashtag 0xD

#define DECODE_NEC

#define IR_PIN 4

class IRMode : public Mode
{
public:
    void enter() override;
    void exit() override;
    void runStateMachine() override;
    const char *name() override;

private:
    void setColor(CRGB color) override;
    void interpretIR();
    void moveRelevantServo(bool, Servo*);

    const uint16_t START_INCREMENT = 15;
    const uint16_t RAMP_INCREMENT = 10;
    uint16_t currentIncrement = START_INCREMENT;
    uint16_t currCommand = 0;
    uint16_t lastCommand = 0;
};