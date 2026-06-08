#include "modes_headers/OmnibMode.h"

/*
************************************************************************************
* The following is the implementation file for the manual pitch mode. The header file contains
* the declarations for the class and the functions as well as mode description.
************************************************************************************
*/

void OmnibMode::enter()
{
    resetPositions();
    setupPins();
    SERIAL_PRINTLN("OmnibMode entered");
}

void OmnibMode::exit()
{
    SERIAL_PRINTLN("OmnibMode exited");
}

void OmnibMode::runStateMachine()
{
    relevantAction(interpretRF());
    delay(SMOOTHING / 2);
}

const char *OmnibMode::name()
{
    return "Omnib_Mode";
}

void OmnibMode::setColor(CRGB color)
{
    robot.leds[0] = color;
    FastLED.show();
}

/*
************************************************************************************
* The following function is used to manually control the pitch servo.
* It reads the potentiometer value and maps it to the pitch range.
* It then moves the servo to the new position.
************************************************************************************
*/
void OmnibMode::setupPins()
{
    pinMode(S1_V, INPUT_PULLUP);
    pinMode(S1_G, INPUT_PULLUP);
    pinMode(S2_V, INPUT_PULLUP);
    pinMode(S2_G, INPUT_PULLUP);
    pinMode(S3_V, INPUT_PULLUP);
    pinMode(S3_G, INPUT_PULLUP);
    pinMode(S4_V, INPUT_PULLUP);
    pinMode(S4_G, INPUT_PULLUP);
}

int OmnibMode::interpretRF()
{
    int command;

    // inputs stores meaured RF controller states, inputArr stores processed data
    int inputArr[8];
    int inputs[8] = {1, 1, 1, 1, 1, 1, 1, 1};

    // create time variable
    unsigned long t = millis();

    while (millis() < (t + SMOOTHING / 2))
    {
        if (digitalRead(S1_G) == 0)
        {
            inputs[0] = 0;
        }
        if (digitalRead(S1_V) == 0)
        {
            inputs[1] = 0;
        }
        if (digitalRead(S2_G) == 0)
        {
            inputs[2] = 0;
        }
        if (digitalRead(S2_V) == 0)
        {
            inputs[3] = 0;
        }
        if (digitalRead(S3_G) == 0)
        {
            inputs[4] = 0;
        }
        if (digitalRead(S3_V) == 0)
        {
            inputs[5] = 0;
        }
        if (digitalRead(S4_G) == 0)
        {
            inputs[6] = 0;
        }
        if (digitalRead(S4_V) == 0)
        {
            inputs[7] = 0;
        }
    }

    // Save result of 5ms sample
    inputArr[0] = inputs[0];
    inputArr[1] = inputs[1];
    inputArr[2] = inputs[2];
    inputArr[3] = inputs[3];
    inputArr[4] = inputs[4];
    inputArr[5] = inputs[5];
    inputArr[6] = inputs[6];
    inputArr[7] = inputs[7];

    // define raw commands as robot actions
    int F[8] = {1, 1, 0, 1, 1, 0, 1, 1};    // 1. FWD
    int B[8] = {1, 1, 1, 0, 0, 1, 1, 1};    // 2. REV
    int L[8] = {0, 1, 0, 1, 0, 1, 1, 1};    // 3. L
    int R[8] = {1, 0, 1, 0, 1, 0, 1, 1};    // 4. R
    int U[8] = {1, 1, 1, 1, 1, 1, 1, 0};    // 5. UP
    int D[8] = {1, 1, 1, 1, 1, 1, 0, 1};    // 6. DOWN
    int FL[8] = {0, 1, 0, 1, 1, 0, 1, 1};   // 7. FWD + LEFT
    int FR[8] = {1, 0, 0, 1, 1, 0, 1, 1};   // 8. FWD + RIGHT
    int BL[8] = {0, 1, 1, 0, 0, 1, 1, 1};   // 9. REV + LEFT
    int BR[8] = {1, 0, 1, 0, 0, 1, 1, 1};   // 10. REV + RIGHT
    int FU[8] = {1, 1, 0, 1, 1, 0, 1, 0};   // 11. FWD + UP
    int FD[8] = {1, 1, 0, 1, 1, 0, 0, 1};   // 12. FWD + DOWN
    int BU[8] = {1, 1, 1, 0, 0, 1, 1, 0};   // 13. REV + UP
    int BD[8] = {1, 1, 1, 0, 0, 1, 0, 1};   // 14. REV + DOWN
    int NONE[8] = {1, 1, 1, 1, 1, 1, 1, 1}; // 15. NONE, do nothing.

    // create array that enumerates every robot action
    int cmdArr[15] = {0};

    // Only one command can have a perfect match degree of 8.
    for (int i = 0; i < 8; i++)
    {
        if (inputArr[i] == F[i])
            cmdArr[0] += 1;
        if (inputArr[i] == B[i])
            cmdArr[1] += 1;
        if (inputArr[i] == L[i])
            cmdArr[2] += 1;
        if (inputArr[i] == R[i])
            cmdArr[3] += 1;
        if (inputArr[i] == U[i])
            cmdArr[4] += 1;
        if (inputArr[i] == D[i])
            cmdArr[5] += 1;
        if (inputArr[i] == FL[i])
            cmdArr[6] += 1;
        if (inputArr[i] == FR[i])
            cmdArr[7] += 1;
        if (inputArr[i] == BL[i])
            cmdArr[8] += 1;
        if (inputArr[i] == BR[i])
            cmdArr[9] += 1;
        if (inputArr[i] == FU[i])
            cmdArr[10] += 1;
        if (inputArr[i] == FD[i])
            cmdArr[11] += 1;
        if (inputArr[i] == BU[i])
            cmdArr[12] += 1;
        if (inputArr[i] == BD[i])
            cmdArr[13] += 1;
        if (inputArr[i] == NONE[i])
            cmdArr[14] += 1;
    }

    // loop thru match degree list to find the selected robot action
    for (int i = 0; i < 15; i++)
    {
        if (cmdArr[i] == 8)
        {
            command = i;
            break;
        }
    }
    return command; // return the command index
}

void OmnibMode::relevantAction(int command)
{
    currentIncrement = START_INCREMENT;
    // Assign speed vector vlaues based on chosen command
    switch (command)
    {
    case 0: // Forward
        currentIncrement *= 2;
        moveServo(true, &robot.pitchServo);
        break;
    case 1: // Back
        currentIncrement *= 2;
        moveServo(false, &robot.pitchServo);
        break;
    case 2: // Rotate Left
        moveServo(true, &robot.yawServo);
        break;
    case 3: // Rotate Right
        moveServo(false, &robot.yawServo);
        break;
    case 4: // Launch (UP)
        launchRoutine(robot.waitTime);
        break;
    case 5: // (DOWN)
        launchRoutine(robot.waitTime);
        break;
    case 6: // Forward + Left
        currentIncrement *= 1.5;
        moveServo(true, &robot.pitchServo, &robot.yawServo, true);
        break;
    case 7: // Forward + Right
        currentIncrement *= 1.5;
        moveServo(true, &robot.pitchServo, &robot.yawServo, false);
        break;
    case 8: // Back + Left
        currentIncrement *= 1.5;
        moveServo(false, &robot.pitchServo, &robot.yawServo, true);
        break;
    case 9: // Back + Right
        currentIncrement *= 1.5;
        moveServo(false, &robot.pitchServo, &robot.yawServo, false);
        break;
    case 10: // Forward + Fork Up
        currentIncrement *= 2;
        moveServo(true, &robot.pitchServo);
        break;
    case 11: // Forward + Fork Down
        currentIncrement *= 2;
        moveServo(true, &robot.pitchServo);
        break;
    case 12: // Back + Fork Up
        currentIncrement *= 2;
        moveServo(false, &robot.pitchServo);
        break;
    case 13: // Back + Fork Down
        currentIncrement *= 2;
        moveServo(false, &robot.pitchServo);
        break;
    case 14: // None
        // filler
        break;
    default:
        break;
    }
}

void OmnibMode::moveServo(bool dir, Servo *servo, Servo *servo2, bool dir2)
{
    bool direction = dir;

    // Accounts for double servo movement
    const int numServos = (servo2 != nullptr) ? 2 : 1;

    // Creates an array for servos, will always have two elements, but second element is only used when numServos is 2
    Servo *servos[] = {servo, servo2 != nullptr ? servo2 : servo};

    // Loops through array and applies increment logic to each servo
    for (int i = 0; i < numServos; i++)
    {
        int16_t currPosition = servos[i]->readMicroseconds();
        if (i == 1) // decides increment direction for second servo
            direction = dir2;
        int32_t increment = (direction) ? currentIncrement : -currentIncrement;
        int32_t nextPosition = currPosition + increment;

        if (nextPosition <= startPWM)
        {
            nextPosition = startPWM;
        }
        else if (nextPosition >= endPWM)
        {
            nextPosition = endPWM;
        }
        SERIAL_PRINTLN(nextPosition);
        servos[i]->writeMicroseconds(nextPosition);
    }
}