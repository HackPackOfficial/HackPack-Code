#include "modes_headers/ReflexMode.h"

/*
************************************************************************************
* The following is the implementation file for the reflex mode. The header file contains
* the declarations for the class and the functions as well as mode description.
************************************************************************************
*/

#define magazineCollisionAngle 2250 // any steeper than this and loading magazine potentially jams when rapid shooting (reflex mode only)
#define minReflexAngle 750          // any shallower than this and the magazine may shoot too low (reflex mode only)

void ReflexMode::enter()
{
  setColor(CRGB(
    0,
    0,
    0
  )); // White
  SERIAL_PRINTLN("ReflexMode entered");
  resetPositions();
}

void ReflexMode::exit()
{
  resetPositions();
  robot.waitTime = 500;
  reflexState = STATE_GENERATION;
}

void ReflexMode::runStateMachine()
{
  switch (reflexState)
  {
  case STATE_GENERATION:
    setColor(CRGB(
      0,
      0,
      0
    )); // White
    if (robot.currButton == 1)
    {
      startSequence();
      randomSeed(millis());
      generation(yawValues, true);    // Generate yaw values
      generation(pitchValues, false); // Generate pitch values
      fixValues(yawValues, pitchValues);
      consecutiveFakeouts = 0; // Reset fake-out counter for new sequence
      reflexState = STATE_BEHAVIOR;
    }
    break;
  case STATE_BEHAVIOR:
    reflexState = STATE_SHOOTING;
    break;
  case STATE_SHOOTING:
    randomShoot();
    resetPositions();
    reflexState = STATE_GENERATION;
    break;
  }
}

const char *ReflexMode::name()
{
  return "Reflex_Mode";
}

void ReflexMode::setColor(CRGB color)
{ // does not use color since pulls from potentiometer
  difficulty = round(map(analogRead(POT_PIN), 0, 1000, 0, difficultyLevels - 1));
  switch (difficulty)
  {
  case 0:
    robot.waitTime = 300;
    break;
  case 1:
    robot.waitTime = 150;
    break;
  case 2:
    robot.waitTime = 0;
    break;
  }
  uint8_t hue = map(difficulty, 0, difficultyLevels - 1, 60, 0);
  robot.leds[0] = CHSV(hue, 255, 255);
  FastLED.show();
}

// Single function to get any difficulty value
int16_t ReflexMode::getDifficultyValue(int8_t parameter)
{
  return DIFFICULTY_VALUES[difficulty][parameter];
}

/*
************************************************************************************
* The following function is used to generate the yaw or pitch values for the random launch sequence.
* It generates a random value within the range of the yaw or pitch range, using the difficulty value
* to determine certain weighting factors. These add complexity to the sequence.
************************************************************************************
*/

void ReflexMode::generation(int16_t valueArr[], bool isYaw)
{
  // Determine range and center based on whether this is yaw or pitch
  int16_t range, center;
  if (isYaw)
  {
    range = getDifficultyValue(YAW_RANGE);
    center = visualYawCenterPWM;
  }
  else
  {
    range = getDifficultyValue(PITCH_RANGE);
    center = startPWM + endPWM / 2;
  }

  int16_t patternComplexity = getDifficultyValue(PATTERN_COMPLEXITY);

  // Generate first value
  int16_t previousValue = random(center - range / 2, center + range / 2);
  if (!isYaw)
    previousValue = max(startPWM, previousValue - collisionPrevention);
  valueArr[0] = previousValue;

  for (int16_t i = 1; i <= RANDOM_SIZE; i++)
  {
    bool complete = false;
    while (!complete) // Prevent infinite loops
    {
      int16_t candidate = random(center - range / 2, center + range / 2);
      int16_t distance = abs(candidate - previousValue); // Calculate the distance

      // Calculate weighting with pattern complexity
      float weight = float(distance) / float(range);
      int16_t weightFactor = difficultyLevels - difficulty; // Higher difficulty = lower weightFactor (more selective)

      // Higher complexity = Larger movements
      int16_t complexityBonus = (difficultyLevels - patternComplexity) * 10;
      weightFactor += complexityBonus / 100;

      if (random(100) < weight * 100 * weightFactor)
      { // Accept based on weighting
        if (!isYaw)
          candidate = max(startPWM, candidate - collisionPrevention);
        valueArr[i] = candidate;   // Store the accepted value
        previousValue = candidate; // Update the previous value
        complete = true;
      }
    }
  }
}

/*
************************************************************************************
* The following function is used to fix the yaw or pitch values for the random launch sequence.
* It ensures that the values are within the acceptable range (deemed via empirical testing).
************************************************************************************
*/

void ReflexMode::fixValues(int16_t arr1[], int16_t arr2[])
{
  for (int16_t i = 0; i < RANDOM_SIZE; i++)
  {
    if (arr1[i] < minReflexAngle)
    {
      arr1[i] += startPWM;
    }
    else if (arr1[i] > magazineCollisionAngle - 150)
    {
      arr1[i] -= startPWM;
    }

    if (arr2[i] <= minReflexAngle)
      arr2[i] += collisionPrevention / 2;
  }
}

/*
************************************************************************************
* The following function is used to shoot the random launch sequence.
* Depending on the difficulty, the sequence may include fakeout shots.
************************************************************************************
*/

void ReflexMode::randomShoot()
{
  int16_t ammoShot = 0;
  int16_t movementDelay = getDifficultyValue(MOVEMENT_DELAY);
  int16_t fakeOutDelay = getDifficultyValue(FAKEOUT_DELAY);
  int16_t speed = getDifficultyValue(DELAY_SPEED);

  while (ammoShot < RANDOM_SIZE)
  {
    if (fakeOut())
    {
      servoEase(random(startPWM, endPWM), true, speed);
      delay(fakeOutDelay);
      servoEase(random(startPWM, magazineCollisionAngle - collisionPrevention / 2), false, speed);
      delay(fakeOutDelay);
    }
    else
    {
      servoEase(yawValues[ammoShot], true, speed);
      delay(movementDelay);
      servoEase(pitchValues[ammoShot], false, speed);
      delay(movementDelay);
      launchRoutine(0);
      ammoShot++;
    }
  }
}

/*
************************************************************************************
* The following function is used to determine if a fakeout shot should be taken.
* It uses the difficulty value to determine the probability of a fakeout.
* It also reduces the probability based on consecutive fake-outs.
************************************************************************************
*/

bool ReflexMode::fakeOut()
{

  float baseProbability = getDifficultyValue(FAKEOUT_PROB) / 100.0; // Convert percentage to decimal

  // Reduce probability based on consecutive fake-outs (smaller penalty)
  float penalty = consecutiveFakeouts * 0.25;                      // 25% reduction per consecutive fake-out
  float adjustedProbability = max(0.1, baseProbability - penalty); // Minimum 10% chance

  int32_t randomValue = random(0, 100);
  bool isFakeout = randomValue < (adjustedProbability * 100);

  // Update consecutive fake-out counter
  if (isFakeout)
  {
    consecutiveFakeouts++;
  }
  else
  {
    consecutiveFakeouts = 0; // Reset counter when real shot occurs
  }
  return isFakeout;
}

/*
************************************************************************************
* The following function is used to start signal the start of the sequence. This allows
* the use to visually confirm that the sequence is starting, preparing themselves.
************************************************************************************
*/

void ReflexMode::startSequence()
{
  for (int16_t i = 0; i < 3; i++)
  {
    robot.pitchServo.writeMicroseconds(startPWM + 300);
    delay(200);
    robot.pitchServo.writeMicroseconds(startPWM);
    delay(200);
  }
}