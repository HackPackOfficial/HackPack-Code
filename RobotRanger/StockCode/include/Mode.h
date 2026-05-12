#pragma once
#include "Shared.h"

// Abstract class for all modes, in seperate header to avoid circular dependencies
// Every mode will inherit from this class, and will implement the five virtual functions listed + deconstructor
class Mode
{
public:
    virtual void enter() = 0;
    virtual void exit() = 0;
    virtual void runStateMachine() = 0;
    virtual const char *name() = 0;
    virtual ~Mode() {}

protected:
    virtual void setColor(CRGB color) = 0;
};