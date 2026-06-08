#pragma once
#include "Mode.h"

/*
************************************************************************************
* The following is the header file for the reflex mode, which is responsible for the robot's game mode.
* It generates a random sequence of behaviors, and launches metal balls, which you (the user) must catch!
************************************************************************************
*/

#define RANDOM_SIZE 5
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
        {50, 300, 30, 1600, 1600, 0, 10}, // Easy
        {25, 250, 35, 1800, 1800, 1, 5},  // Medium
        {0, 200, 40, 2000, 2000, 2, 0}    // Hard
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
