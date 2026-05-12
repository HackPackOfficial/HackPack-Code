#pragma once
#include "Mode.h"

class ReflexMode : public Mode {
    public:
       void enter() override;
       void exit() override;
       void runStateMachine() override;
       const char* name() override;
       
    private:
        enum StatesReflex_t {
            STATE_GENERATION,
            STATE_BEHAVIOR,
            STATE_SHOOTING,
        };

        StatesReflex_t reflexState = STATE_GENERATION;
        #define RANDOM_SIZE 5
        int difficulty = 0;
        int difficultyLevels = 3;
        int consecutiveFakeouts = 0; // Track consecutive fake-outs
        bool fakeoutUsed = false; // Track if fake-out has been used in current sequence
        const int collisionPrevention = 500;

        // Single 2D array for all difficulty values [difficulty][parameter]
        // Parameters: 0=movement_delay, 1=fakeout_delay, 2=fakeout_prob, 3=yaw_range, 4=pitch_range, 5=pattern_complexity
        static const int DIFFICULTY_VALUES[3][7];
        
        // Parameter constants for indexing
        static const int MOVEMENT_DELAY = 0;
        static const int FAKEOUT_DELAY = 1;
        const int FAKEOUT_PROB = 2;
        const int YAW_RANGE = 3;
        const int PITCH_RANGE = 4;
        const int PATTERN_COMPLEXITY = 5;
        const int SPEED = 6;
        
        // Function to get any difficulty value
        int getDifficultyValue(int parameter);

        int yawValues[RANDOM_SIZE];
        int pitchValues[RANDOM_SIZE];
        void generation(int valueArr[], bool isYawArray);
        void fixValues(int arr1[], int arr2[]);
        bool fakeOut();
        void randomShoot();
        void setColor(CRGB color) override;
        void startSequence();
};