#pragma once
#include "Shared.h"

/*
************************************************************************************
* The following is the abstract class for all modes, in a seperate header to avoid circular dependencies.
* Every mode will inherit from this class, and will implement the virtual functions listed + constructor/deconstructor.
* Each mode has its own unique LED color and state machine.

* See FOOTNOTES for more information about the mode class and virtual functions.
************************************************************************************
*/

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