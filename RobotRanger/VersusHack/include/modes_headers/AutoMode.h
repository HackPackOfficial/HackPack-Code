#pragma once
#include "Mode.h"

/*
************************************************************************************
* The following is the header file for the auto mode, which is responsible for finding and locking on to targets.
* It is used to sweep the surrounding area, find targets (cups), lock on to them, calculate projectile angle, then launch metal balls into them.
************************************************************************************
*/

// ARRAYS
#define POSITIONS 201 // sets number of positions to poll, correlated to PWM or steps
#define SPACES 200    // always one less than positions
#define OBJECTS 8     // max number of objects you can reasonably expect, assuming some double counts
#define WINDOW 5      // size of rolling average window

class AutoMode : public Mode
{
public:
    void enter() override;
    void exit() override;
    void runStateMachine() override;
    const char *name() override;

    void processingDistances();
    void processingSmoothedDistances();
    void processingHits(int);

private:
    void setColor(CRGB color) override;

    enum StatesAuto_t
    {
        STATE_IDLE,
        STATE_SWEEP,
        STATE_FIND,
        STATE_LOCK,
        STATE_DONE,
    };

    StatesAuto_t autoState = STATE_IDLE;
    uint16_t yawPosition = 0;
    uint16_t distance = 0;
    uint16_t lastMinima = -5;
    uint16_t hitsIndex = 0;
    uint32_t currTime = 0;

    int16_t smoothedDistances[POSITIONS]; // moving average values
    int16_t window[WINDOW];               // average window
    int16_t hits[OBJECTS];                // targets to find

    float degreesPerStep = 180.0 / (POSITIONS - 1);

    void handleSweep();
    void handleFind();
    void handleLockServo();
    void updateWindow();
    void smoothingDistance();
    void detect();
    void filterHits();
    void reverseHits();
    void resetMemory();
    int16_t calculateSpacer(int16_t distance);
};
