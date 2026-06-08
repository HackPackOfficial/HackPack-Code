#pragma once
#include "Mode.h"

/*
************************************************************************************
* The following is the class for the modes manager, which is responsible for moving between modes.
* It is adaptable to any number of modes, and is used to run the state machine of the current mode.
* Essentially, it is a wrapper for an array of the modes, making it easy to call moving between them. 

* See FOOTNOTES for more information about the modes manager class and pointers. 
************************************************************************************
*/

class ModesManager
{
public:
    ModesManager(Mode **modeList, int modeCount)
        : modes(modeList), count(modeCount), index(0) {}

    void runStateMachine()
    {
        if (currentMode() != nullptr)
        {
            currentMode()->runStateMachine(); //Points to the current mode's state machine.
        }
    }

    void changeMode(int direction) //correlated to button presses on the resistor ladder
    { 
        if (direction == 2)
        {
            currentMode()->exit();
            index = (index - 1 + count) % count;
            currentMode()->enter();
        }
        else if (direction == 3)
        {
            currentMode()->exit();
            index = (index + 1) % count;
            currentMode()->enter();
        }
    }

    void setMode(int modeNumber)
    {
        currentMode()->exit();
        index = modeNumber;
        currentMode()->enter();
    }

    void setMode(Mode* mode)
    {
        for (int i = 0; i < count; i++)
        {
            if (modes[i] == mode)
            {
                currentMode()->exit();
                index = i;
                currentMode()->enter();
                return;
            }
        }
    }

    int getCurrentModeIndex()
    {
        return index;
    }

    Mode *currentMode()
    {
        return modes[index];
    }

private:
    Mode **modes;
    int count;
    int index;
};
