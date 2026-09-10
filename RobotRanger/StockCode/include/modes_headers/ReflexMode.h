#pragma once
#include "Mode.h"

/*
************************************************************************************
* The following is the header file for the reflex mode, which is responsible for the robot's game mode.
* It generates a random sequence of behaviors, and launches metal balls, which you (the user) must catch!
************************************************************************************
*/

#define RANDOM_SIZE 5 ///{"range":[1,6]}
#define difficultyLevels 3
#define numParameters 7

class ReflexMode : public Mode
{
public:
    void enter() override;
    void exit() override;
    void runStateMachine() override;
    const char *name() override;

private:
    void setColor(CRGB color) override;

    enum StatesReflex_t
    {
        STATE_GENERATION,
        STATE_BEHAVIOR,
        STATE_SHOOTING,
    };

    StatesReflex_t reflexState = STATE_GENERATION;
    int8_t difficulty = 0;
    int16_t consecutiveFakeouts = 0; // Track consecutive fake-outs
    bool fakeoutUsed = false;        // Track if fake-out has been used in current sequence
    const int16_t collisionPrevention = 500;

    // Single 2D array for all difficulty values [difficulty][parameter]
    // Parameters: 0=movement_delay, 1=fakeout_delay, 2=fakeout_prob, 3=yaw_range, 4=pitch_range, 5=pattern_complexity, 6=delay speed
    // Generate your own values here for your own robot!
    const int16_t DIFFICULTY_VALUES[difficultyLevels][numParameters] = {
        // movement_delay, fakeout_delay, fakeout_prob, yaw_range, pitch_range, pattern_complexity, delay speed
        // Easy
        {
            50, ///{"min":0,"max":100}
            300, ///{"min":0,"max":500}
            30, ///{"min":0,"max":80}
            1600, ///{"min":1000,"max":2200}
            1600, ///{"min":1000,"max":2200}
            0,
            10 ///{"min":0,"max":20}
        },
        // Medium
        {
            25, ///{"min":0,"max":100}
            250, ///{"min":0,"max":500}
            35, ///{"min":0,"max":80}
            1800, ///{"min":1000,"max":2200}
            1800, ///{"min":1000,"max":2200}
            1,
            5 ///{"min":0,"max":20}
        },
        // Hard  
        {
            0, ///{"min":0,"max":100}
            200, ///{"min":0,"max":500}
            40, ///{"min":0,"max":80}
            2000, ///{"min":1000,"max":2200}
            2000, ///{"min":1000,"max":2200}
            2,
            0 ///{"min":0,"max":20}
        }
    };

    // Parameter constants for indexing
    static const int8_t MOVEMENT_DELAY = 0;
    static const int8_t FAKEOUT_DELAY = 1;
    const int8_t FAKEOUT_PROB = 2;
    const int8_t YAW_RANGE = 3;
    const int8_t PITCH_RANGE = 4;
    const int8_t PATTERN_COMPLEXITY = 5;
    const int8_t DELAY_SPEED = 6;

    // Function to get any difficulty value
    int16_t getDifficultyValue(int8_t parameter);

    int16_t yawValues[RANDOM_SIZE];
    int16_t pitchValues[RANDOM_SIZE];
    void generation(int16_t valueArr[], bool isYawArray);
    void fixValues(int16_t arr1[], int16_t arr2[]);
    bool fakeOut();
    void randomShoot();
    void startSequence();
};
