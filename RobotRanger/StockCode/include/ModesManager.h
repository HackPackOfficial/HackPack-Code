#pragma once
#include "Mode.h"
// Implements moving between modes, and is adaptable to any number of modes

class ModesManager
{
public:
    ModesManager(Mode **modeList, int modeCount)
        : modes(modeList), count(modeCount), index(0) {}

    void runStateMachine()
    {
        if (currentMode() != nullptr)
        {
            currentMode()->runStateMachine();
        }
    }

    void changeMode(int direction)
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
