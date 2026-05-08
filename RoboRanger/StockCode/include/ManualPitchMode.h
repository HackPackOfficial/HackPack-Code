#pragma once
#include "Mode.h"

class ManualPitchMode : public Mode {
    public:
        void enter() override;
        void exit() override;
        void runStateMachine() override;
        const char* name() override;
       
    private:
        void setColor(CRGB color) override;
        
        int initialPotValue = 0;  // Store initial potentiometer value
        int lastPitchAngle = startPWM;  // Store the last pitch angle

        void manualPitch();
        
}; 