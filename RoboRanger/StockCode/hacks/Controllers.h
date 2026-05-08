#pragma once
#include "Mode.h"


//IR Commands
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

//Pins for various controllers
#define SELECT_IR A0
#define SELECT_JS A2
#define SELECT_RF A1

#define JS_X A6
#define JS_Y A7
#define JS_BUTTON 4

const uint8_t pitchIncrement = 10;
const uint8_t yawIncrement = 10;


class ControllersModeHack : public Mode
{
public:
    void enter() override;
    void exit() override;
    void runStateMachine() override;
    const char *name() override;

private:
    void setColor(CRGB color) override;

    enum ControllersType_t
    {
        CONTROLLER_IR, 
        CONTROLLER_JS,
        CONTROLLER_RF, //controller from OMNIBOT
        CONTROLLER_OTHER //options to include others
    };

    ControllersType_t controlScheme;

    void configureHack(); 
    void handleIR();
    void handleJS();
    void handleRF(); 
    void handleOther();
};