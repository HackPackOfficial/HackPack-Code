#pragma once
#include "Mode.h"

class TestMode : public Mode {
    public:
       void enter() override;
       void exit() override;
       void runStateMachine() override;
       const char* name() override;
       
    private:
        void setColor(CRGB color) override;

        enum StatesTest_t {
            STATE_TEST_STAGE_1,
            STATE_COUNT //ALWAYS keep last, this is used to set the number of states
        };

        StatesTest_t testState = STATE_TEST_STAGE_1;

        void testRoutine();
        void testRoutine2();
        
};