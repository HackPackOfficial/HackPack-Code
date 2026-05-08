#include "ReflexMode.h"

// Define the static const array
const int ReflexMode::DIFFICULTY_VALUES[3][7] = {
    // movement_delay, fakeout_delay, fakeout_prob, yaw_range, pitch_range, pattern_complexity, delay speed
    {50, 300, 30, 1600, 1600, 0, 10}, // Easy
    {25, 250, 40, 1800, 1800, 1, 5}, // Medium
    {0, 200, 50, 2000, 2000, 2, 0}  // Hard
};

void ReflexMode::enter()
{
  setColor(CRGB(0, 0, 0)); // White       
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
    setColor(CRGB(0, 0, 0)); // White
    if (robot.currButton == 1)
    {
      startSequence();
      randomSeed(millis());
      consecutiveFakeouts = 0; // Reset fake-out counter for new sequence
      generation(yawValues, true);    // Generate yaw values
      generation(pitchValues, false); // Generate pitch values
      fixValues(yawValues, pitchValues);
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
  difficulty = round(map(analogRead(POT_PIN), 0, 1000, 0, 2));
  switch (difficulty) {
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
  uint8_t hue = map(difficulty, 0, 2, 60, 0);
  robot.leds[0] = CHSV(hue, 255, 255);
  FastLED.show();
}

// Single function to get any difficulty value
int ReflexMode::getDifficultyValue(int parameter)
{
  return DIFFICULTY_VALUES[difficulty][parameter];
}

void ReflexMode::generation(int valueArr[], bool isYaw)
{
  // Determine range and center based on whether this is yaw or pitch
  int range, center;
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

  int patternComplexity = getDifficultyValue(PATTERN_COMPLEXITY);
 
  // Generate first value
  int previousValue = random(center - range / 2, center + range / 2);
  if (!isYaw) previousValue = max(startPWM, previousValue - collisionPrevention);  
  valueArr[0] = previousValue;

  for (int i = 1; i <= RANDOM_SIZE; i++)
  {
    int complete = false;
    while (!complete) // Prevent infinite loops
    {
      int candidate = random(center - range / 2, center + range / 2);
      int distance = abs(candidate - previousValue); // Calculate the distance

      // Calculate weighting with pattern complexity
      float weight = float(distance) / float(range);
      int weightFactor = difficultyLevels - difficulty; // Higher difficulty = lower weightFactor (more selective)
      
      // Higher complexity = Larger movements
      int complexityBonus = (difficultyLevels - patternComplexity) * 10; 
      weightFactor += complexityBonus / 100;

      if (random(100) < weight * 100 * weightFactor)
      {                            // Accept based on weighting
        if (!isYaw) candidate = max(startPWM, candidate - collisionPrevention); 
        valueArr[i] = candidate;   // Store the accepted value
        previousValue = candidate; // Update the previous value
        complete = true;
      }
    }
  }
}

void ReflexMode::fixValues(int arr1[], int arr2[])
{
  for (int i = 0; i < RANDOM_SIZE; i++)
  {
    if (arr1[i] < 750)
    {
      arr1[i] += startPWM;
    }
    else if (arr1[i] > 2250)
    {
      arr1[i] -= startPWM;
    }

    if (arr2[i] <= 750) arr2[i] += collisionPrevention/2;
  }
}

void ReflexMode::randomShoot()
{
  int ammoShot = 0;
  int movementDelay = getDifficultyValue(MOVEMENT_DELAY);
  int fakeOutDelay = getDifficultyValue(FAKEOUT_DELAY);
  int speed = getDifficultyValue(SPEED);

  while (ammoShot < RANDOM_SIZE)
  {
    if (fakeOut())
    {
      servoEase(random(startPWM, endPWM), true, speed);
      delay(fakeOutDelay);
      servoEase(random(startPWM, magazineCollisionAngle - collisionPrevention/2), false, speed);
      delay(fakeOutDelay);
    }
    else
    {
      servoEase(yawValues[ammoShot], true, speed);
      delay(movementDelay);
      servoEase(pitchValues[ammoShot], false, speed);
      delay(movementDelay);
      launchRoutine();
      //delay(1000);
      ammoShot++;
    }
  }
}

bool ReflexMode::fakeOut()
{

  
  float baseProbability = getDifficultyValue(FAKEOUT_PROB) / 100.0; // Convert percentage to decimal
  
  // Reduce probability based on consecutive fake-outs (smaller penalty)
  float penalty = consecutiveFakeouts * 0.25; // 25% reduction per consecutive fake-out
  float adjustedProbability = max(0.1, baseProbability - penalty); // Minimum 10% chance
  
  int randomValue = random(0, 100);
  bool isFakeout = randomValue < (adjustedProbability * 100);
  
  // Update consecutive fake-out counter
  if (isFakeout) {
    consecutiveFakeouts++;
  } else {
    consecutiveFakeouts = 0; // Reset counter when real shot occurs
  }
  
  return isFakeout;
}

void ReflexMode::startSequence() {
  for(int i = 0; i < 3; i++) {
  robot.pitchServo.writeMicroseconds(startPWM + 300);
  delay(200);
  robot.pitchServo.writeMicroseconds(startPWM);
  delay(200);
  }
}