#pragma once
#include "Mode.h"

class AutoMode : public Mode {
    public:
       void enter() override;
       void exit() override;
       void runStateMachine() override;
       const char* name() override;

        void processingDistances();
        void processingSmoothedDistances(); 
        void processingHits(int); 
       
    private:
        enum StatesAuto_t {
            STATE_IDLE,
            STATE_SWEEP,
            STATE_FIND,
            STATE_LOCK,
            STATE_SHOOT_AUTO,
            STATE_DONE,
        };

        StatesAuto_t autoState = STATE_IDLE;
        uint32_t yawPosition = 0;
        uint32_t distance = 0;
        uint32_t lastMinima = -5;
        uint32_t currTime = 0;
        uint32_t hitsIndex = 0;
        uint32_t closeDistance = 200;


        #define LOWER_OFFSET_THRESHOLD 10
        #define UPPER_OFFSET_THRESHOLD 20

        // TUNING
        #define THRESHOLD 1           // difference for minima
        #define THRESHOLD_TUNING 0.01 // distance based change
        #define WINDOW 5              // size of rolling average window
       
        // ARRAYS
        #define POSITIONS 201 // sets number of angles to poll, correlated to PWM or steps
        #define SPACES  200 //always one less than positions
        #define OBJECTS 10    // max number of objects you expect

        static const uint8_t bufferSize = POSITIONS/8;

        int smoothedDistances[POSITIONS]; // moving average values 
        int smoothedDistancesBuffer[bufferSize]; // ring buffer
        int window[WINDOW];               // average window
        int hits[OBJECTS];                // targets to find
        

        float degreesPerStep = 180.0 / (POSITIONS - 1);

        void handleSweep();
        void handleFind();
        void handleLockServo();
        void updateWindow();
        void smoothingDistance();
        void detect();
        void filterHits();
        void reverseHits();
        int calcThreshold(int distance);
        int calculateSpacer(int distance);
        void resetMemory();
        void setColor(CRGB color) override;
};

